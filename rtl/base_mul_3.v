`timescale 1ns/1ps
`include "common.vh"

/*
 * base_mul_3: Flat 192×192 GF(2) Karatsuba multiplier (SIZE=3).
 *
 * Performs A[2:0] * B[2:0] over GF(2), each word = 64-bit.
 * Interface: BRAM-style read ports for A/B (2-cycle latency), write port for C.
 *
 * Karatsuba math (SIZE=3, split L=2, H=1):
 *   A = [a2, a1, a0],  B = [b2, b1, b0]
 *
 *   K1 = [a1,a0] * [b1,b0]              → 4 words (SIZE=2 Karatsuba: 3 muls)
 *     p0=a0*b0, p1=a1*b1, p2=(a0^a1)*(b0^b1)
 *   K2 = a2*b2                           → 2 words (1 mul): q0=a2*b2
 *   K3 = [a0^a2, a1] * [b0^b2, b1]     → 4 words (SIZE=2 Karatsuba: 3 muls)
 *     r1=(a0^a2)*(b0^b2), p1=a1*b1 (shared w/ K1!), r2=(a0^a1^a2)*(b0^b1^b2)
 *
 *   Total: 6 clmul64 calls (p0,p1,p2,q0,r1,r2; p1 shared K1↔K3)
 *
 *   mid = K1 ^ K2pad ^ K3,  K2pad = {0,0,K2[1],K2[0]} (256-bit zero-extended)
 *   c[0] = K1[0]
 *   c[1] = K1[1]
 *   c[2] = K1[2] ^ mid[0]
 *   c[3] = K1[3] ^ mid[1]
 *   c[4] = K2[0] ^ mid[2]
 *   c[5] = K2[1] ^ mid[3]
 *
 * Timing: 3 reads × 3 cycles + 1 compute + 6 writes = 17 cycles
 *   (REQ0→WAITs→CAP0→WAITs→CAP1→WAITs→CAP2→COMP_REG→WRITE×5→IDLE)
 */
module base_mul_3 #(
    parameter SIZE = 3
) (
    input  wire                            CLK,
    input  wire                            RST,
    input  wire                            start_i,

    output wire                            a_rd_req_o,
    output wire [`SAFE_CLOG2(SIZE)-1:0]    a_rd_addr_o,
    input  wire [64-1:0]                   a_rd_data_i,

    output wire                            b_rd_req_o,
    output wire [`SAFE_CLOG2(SIZE)-1:0]    b_rd_addr_o,
    input  wire [64-1:0]                   b_rd_data_i,

    output wire                            c_wr_en_o,
    output wire [`SAFE_CLOG2(2*SIZE)-1:0]  c_wr_addr_o,
    output wire [64-1:0]                   c_wr_data_o,

    output wire                            done_o,
    output wire [3:0]                      debug_state_o
);

    localparam DW = 64;

    // Carry-less 64×64 multiply over GF(2).
    function [2*DW-1:0] clmul64;
        input [DW-1:0] a;
        input [DW-1:0] b;
        integer i;
        reg [2*DW-1:0] acc;
        begin
            acc = {(2*DW){1'b0}};
            for (i = 0; i < DW; i = i + 1) begin
                if (b[i])
                    acc = acc ^ ({{DW{1'b0}}, a} << i);
            end
            clmul64 = acc;
        end
    endfunction

    // FSM states
    localparam B3_IDLE     = 4'd0;
    localparam B3_REQ0     = 4'd1;
    localparam B3_WAIT0A   = 4'd2;
    localparam B3_WAIT0B   = 4'd3;
    localparam B3_CAP0     = 4'd4;   // capture word 0; req word 1
    localparam B3_WAIT1A   = 4'd5;
    localparam B3_WAIT1B   = 4'd6;
    localparam B3_CAP1     = 4'd7;   // capture word 1; req word 2
    localparam B3_WAIT2A   = 4'd8;
    localparam B3_WAIT2B   = 4'd9;
    localparam B3_CAP2     = 4'd10;  // capture word 2; compute all products
    localparam B3_COMP_REG = 4'd11;  // products stable; begin write c[0]
    localparam B3_WRITE    = 4'd12;  // write c[0..5] (all 6 words)

    reg [3:0]      b3_state_r;
    reg            b3_busy_r;

    // Captured input operands
    reg [DW-1:0]   a0_r, a1_r;   // a2 captured directly from a_rd_data_i in CAP2
    reg [DW-1:0]   b0_r, b1_r;

    // 6 clmul64 products (registered in COMP_REG state):
    //   p0 = a0*b0            (K1 lo*lo)
    //   p1 = a1*b1            (K1 hi*hi AND K3 hi*hi — shared!)
    //   p2 = (a0^a1)*(b0^b1) (K1 cross-term)
    //   q0 = a2*b2            (K2)
    //   r1 = (a0^a2)*(b0^b2) (K3 lo*lo)
    //   r2 = (a0^a1^a2)*(b0^b1^b2)  (K3 cross-term)
    reg [2*DW-1:0] p0_r, p1_r, p2_r, q0_r, r1_r, r2_r;

    // Fully assembled K1, K2, K3 (registered)
    reg [4*DW-1:0] K1_r, K3_r;
    reg [2*DW-1:0] K2_r;

    reg [2:0]      wr_idx_r;   // 0..5
    reg            b3_done_r;

    // Combinational Karatsuba middle terms used in COMP_REG state
    wire [2*DW-1:0] b3_mid_K1_w = p0_r ^ p1_r ^ p2_r;
    wire [2*DW-1:0] b3_mid_K3_w = r1_r ^ p1_r ^ r2_r;

    always @(posedge CLK or negedge RST) begin
        if (!RST) begin
            b3_state_r <= B3_IDLE;
            b3_busy_r  <= 1'b0;
            a0_r <= {DW{1'b0}}; a1_r <= {DW{1'b0}};
            b0_r <= {DW{1'b0}}; b1_r <= {DW{1'b0}};
            p0_r <= {(2*DW){1'b0}}; p1_r <= {(2*DW){1'b0}};
            p2_r <= {(2*DW){1'b0}}; q0_r <= {(2*DW){1'b0}};
            r1_r <= {(2*DW){1'b0}}; r2_r <= {(2*DW){1'b0}};
            K1_r <= {(4*DW){1'b0}};
            K2_r <= {(2*DW){1'b0}};
            K3_r <= {(4*DW){1'b0}};
            wr_idx_r  <= 3'd0;
            b3_done_r <= 1'b0;
        end else begin
            b3_done_r <= 1'b0;
            case (b3_state_r)
                B3_IDLE: if (start_i && !b3_busy_r) begin
                    b3_busy_r  <= 1'b1;
                    wr_idx_r   <= 3'd0;
                    b3_state_r <= B3_REQ0;
                end

                B3_REQ0:   b3_state_r <= B3_WAIT0A;
                B3_WAIT0A: b3_state_r <= B3_WAIT0B;
                B3_WAIT0B: b3_state_r <= B3_CAP0;

                B3_CAP0: begin
                    a0_r <= a_rd_data_i;
                    b0_r <= b_rd_data_i;
                    b3_state_r <= B3_WAIT1A; // req addr=1 asserted combinationally
                end

                B3_WAIT1A: b3_state_r <= B3_WAIT1B;
                B3_WAIT1B: b3_state_r <= B3_CAP1;

                B3_CAP1: begin
                    a1_r <= a_rd_data_i;
                    b1_r <= b_rd_data_i;
                    b3_state_r <= B3_WAIT2A; // req addr=2 asserted combinationally
                end

                B3_WAIT2A: b3_state_r <= B3_WAIT2B;
                B3_WAIT2B: b3_state_r <= B3_CAP2;

                B3_CAP2: begin
                    // a_rd_data_i = a2, b_rd_data_i = b2
                    // Compute all 6 products combinationally, register them.
                    p0_r <= clmul64(a0_r, b0_r);
                    p1_r <= clmul64(a1_r, b1_r);
                    p2_r <= clmul64(a0_r ^ a1_r, b0_r ^ b1_r);
                    q0_r <= clmul64(a_rd_data_i, b_rd_data_i);
                    r1_r <= clmul64(a0_r ^ a_rd_data_i, b0_r ^ b_rd_data_i);
                    r2_r <= clmul64(a0_r ^ a1_r ^ a_rd_data_i,
                                    b0_r ^ b1_r ^ b_rd_data_i);
                    b3_state_r <= B3_COMP_REG;
                end

                B3_COMP_REG: begin
                    // Assemble K1 using b3_mid_K1_w (p0^p1^p2)
                    K1_r[1*DW-1:0*DW] <= p0_r[DW-1:0];
                    K1_r[2*DW-1:1*DW] <= p0_r[2*DW-1:DW] ^ b3_mid_K1_w[DW-1:0];
                    K1_r[3*DW-1:2*DW] <= p1_r[DW-1:0]   ^ b3_mid_K1_w[2*DW-1:DW];
                    K1_r[4*DW-1:3*DW] <= p1_r[2*DW-1:DW];
                    // K2 = a2*b2 = q0
                    K2_r <= q0_r;
                    // Assemble K3 using b3_mid_K3_w (r1^p1^r2): lo=r1, hi=p1
                    K3_r[1*DW-1:0*DW] <= r1_r[DW-1:0];
                    K3_r[2*DW-1:1*DW] <= r1_r[2*DW-1:DW] ^ b3_mid_K3_w[DW-1:0];
                    K3_r[3*DW-1:2*DW] <= p1_r[DW-1:0]   ^ b3_mid_K3_w[2*DW-1:DW];
                    K3_r[4*DW-1:3*DW] <= p1_r[2*DW-1:DW];
                    wr_idx_r   <= 3'd0;
                    b3_state_r <= B3_WRITE;
                end

                B3_WRITE: begin
                    if (wr_idx_r == 3'd5) begin
                        b3_done_r  <= 1'b1;
                        b3_busy_r  <= 1'b0;
                        b3_state_r <= B3_IDLE;
                    end else begin
                        wr_idx_r <= wr_idx_r + 1'b1;
                    end
                end

                default: b3_state_r <= B3_IDLE;
            endcase
        end
    end

    // Read address sequencer
    // addr=0 in B3_REQ0
    // addr=1 in B3_CAP0 (captures word 0, issues req for word 1)
    // addr=2 in B3_CAP1 (captures word 1, issues req for word 2)
    wire [1:0] b3_rd_addr_w =
        (b3_state_r == B3_CAP0) ? 2'd1 :
        (b3_state_r == B3_CAP1) ? 2'd2 : 2'd0;
    assign a_rd_req_o  = (b3_state_r == B3_REQ0) ||
                          (b3_state_r == B3_CAP0) ||
                          (b3_state_r == B3_CAP1);
    assign a_rd_addr_o = b3_rd_addr_w;   // SIZE=3: a_rd_addr is 2-bit, b3_rd_addr_w is 2-bit
    assign b_rd_req_o  = a_rd_req_o;
    assign b_rd_addr_o = a_rd_addr_o;

    // mid = K1 ^ K2pad ^ K3,  K2pad = {0,0,K2[1],K2[0]} (256-bit)
    wire [4*DW-1:0] b3_K2pad_w = {{(2*DW){1'b0}}, K2_r};
    wire [4*DW-1:0] b3_mid_w   = K1_r ^ b3_K2pad_w ^ K3_r;

    // Write mux (wr_idx_r: 0..5, shift L=2)
    wire b3_wr_en_w = (b3_state_r == B3_WRITE);
    wire [DW-1:0] b3_wr_data_w =
        (wr_idx_r == 3'd0) ? K1_r[1*DW-1:0*DW]                                :
        (wr_idx_r == 3'd1) ? K1_r[2*DW-1:1*DW]                                :
        (wr_idx_r == 3'd2) ? K1_r[3*DW-1:2*DW] ^ b3_mid_w[1*DW-1:0*DW]      :
        (wr_idx_r == 3'd3) ? K1_r[4*DW-1:3*DW] ^ b3_mid_w[2*DW-1:1*DW]      :
        (wr_idx_r == 3'd4) ? K2_r[DW-1:0]      ^ b3_mid_w[3*DW-1:2*DW]      :
                             K2_r[2*DW-1:DW]   ^ b3_mid_w[4*DW-1:3*DW];  // idx=5

    assign c_wr_en_o   = b3_wr_en_w;
    assign c_wr_addr_o = wr_idx_r;          // SIZE=3: c_wr_addr is 3-bit, wr_idx_r is 3-bit
    assign c_wr_data_o = b3_wr_data_w;
    assign done_o      = b3_done_r;
    assign debug_state_o = b3_state_r;

endmodule
