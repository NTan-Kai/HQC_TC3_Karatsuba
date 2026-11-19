`timescale 1ns/1ps
`include "common.vh"

/*
 * base_mul_4: Flat 256×256 GF(2) Karatsuba multiplier (SIZE=4).
 *
 * Performs A[3:0] * B[3:0] over GF(2), each word = 64-bit.
 * Interface: BRAM-style read ports for A/B (2-cycle latency), write port for C.
 *
 * Karatsuba math (SIZE=4, split L=H=2):
 *   A = [a3,a2,a1,a0],  B = [b3,b2,b1,b0]
 *   A_lo=[a1,a0], A_hi=[a3,a2];  B_lo=[b1,b0], B_hi=[b3,b2]
 *
 *   K1 = A_lo * B_lo  → 4 words (SIZE=2 Karatsuba)
 *     p0=a0*b0, p1=a1*b1, p2=(a0^a1)*(b0^b1)
 *   K2 = A_hi * B_hi  → 4 words (SIZE=2 Karatsuba)
 *     p3=a2*b2, p4=a3*b3, p5=(a2^a3)*(b2^b3)
 *   K3 = (A_lo^A_hi) * (B_lo^B_hi)  → 4 words (SIZE=2 Karatsuba)
 *     A3_lo=a0^a2, A3_hi=a1^a3; B3_lo=b0^b2, B3_hi=b1^b3
 *     p6=A3_lo*B3_lo, p7=A3_hi*B3_hi, p8=(A3_lo^A3_hi)*(B3_lo^B3_hi)
 *
 *   Total: 9 clmul64 calls (p0..p8), all computed in parallel in one cycle.
 *
 *   mid = K1 ^ K2 ^ K3
 *   c[0] = K1[0]
 *   c[1] = K1[1]
 *   c[2] = K1[2] ^ mid[0]
 *   c[3] = K1[3] ^ mid[1]
 *   c[4] = K2[0] ^ mid[2]
 *   c[5] = K2[1] ^ mid[3]
 *   c[6] = K2[2]
 *   c[7] = K2[3]
 *
 * Timing: 4 reads × 3 cycles + 1 compute + 8 writes = 22 cycles
 *   (REQ0→WAITs→CAP0→WAITs→CAP1→WAITs→CAP2→WAITs→CAP3→COMP_REG→WRITE×7→IDLE)
 */
module base_mul_4 #(
    parameter SIZE = 4
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
    localparam B4_IDLE     = 4'd0;
    localparam B4_REQ0     = 4'd1;
    localparam B4_WAIT0A   = 4'd2;
    localparam B4_WAIT0B   = 4'd3;
    localparam B4_CAP0     = 4'd4;   // capture word 0; req word 1
    localparam B4_WAIT1A   = 4'd5;
    localparam B4_WAIT1B   = 4'd6;
    localparam B4_CAP1     = 4'd7;   // capture word 1; req word 2
    localparam B4_WAIT2A   = 4'd8;
    localparam B4_WAIT2B   = 4'd9;
    localparam B4_CAP2     = 4'd10;  // capture word 2; req word 3
    localparam B4_WAIT3A   = 4'd11;
    localparam B4_WAIT3B   = 4'd12;
    localparam B4_CAP3     = 4'd13;  // capture word 3; compute all 9 products
    localparam B4_COMP_REG = 4'd14;  // products stable; begin write c[0]
    localparam B4_WRITE    = 4'd15;  // write c[1..7]

    reg [3:0]      b4_state_r;
    reg            b4_busy_r;

    reg [DW-1:0]   a0_r, a1_r, a2_r;  // a3 captured directly from port in CAP3
    reg [DW-1:0]   b0_r, b1_r, b2_r;

    // 9 clmul64 products
    reg [2*DW-1:0] p0_r, p1_r, p2_r;  // K1: a0*b0, a1*b1, (a0^a1)*(b0^b1)
    reg [2*DW-1:0] p3_r, p4_r, p5_r;  // K2: a2*b2, a3*b3, (a2^a3)*(b2^b3)
    reg [2*DW-1:0] p6_r, p7_r, p8_r;  // K3: A3lo*B3lo, A3hi*B3hi, (A3lo^A3hi)*(B3lo^B3hi)

    // Assembled K1, K2, K3 (4 words each)
    reg [4*DW-1:0] K1_r, K2_r, K3_r;

    reg [2:0]      wr_idx_r;   // 0..7
    reg            b4_done_r;

    // Combinational Karatsuba middle terms used in B4_COMP_REG state
    wire [2*DW-1:0] b4_mid_K1_w = p0_r ^ p1_r ^ p2_r;
    wire [2*DW-1:0] b4_mid_K2_w = p3_r ^ p4_r ^ p5_r;
    wire [2*DW-1:0] b4_mid_K3_w = p6_r ^ p7_r ^ p8_r;

    always @(posedge CLK or negedge RST) begin
        if (!RST) begin
            b4_state_r <= B4_IDLE;
            b4_busy_r  <= 1'b0;
            a0_r <= {DW{1'b0}}; a1_r <= {DW{1'b0}}; a2_r <= {DW{1'b0}};
            b0_r <= {DW{1'b0}}; b1_r <= {DW{1'b0}}; b2_r <= {DW{1'b0}};
            p0_r <= {(2*DW){1'b0}}; p1_r <= {(2*DW){1'b0}}; p2_r <= {(2*DW){1'b0}};
            p3_r <= {(2*DW){1'b0}}; p4_r <= {(2*DW){1'b0}}; p5_r <= {(2*DW){1'b0}};
            p6_r <= {(2*DW){1'b0}}; p7_r <= {(2*DW){1'b0}}; p8_r <= {(2*DW){1'b0}};
            K1_r <= {(4*DW){1'b0}};
            K2_r <= {(4*DW){1'b0}};
            K3_r <= {(4*DW){1'b0}};
            wr_idx_r  <= 3'd0;
            b4_done_r <= 1'b0;
        end else begin
            b4_done_r <= 1'b0;
            case (b4_state_r)
                B4_IDLE: if (start_i && !b4_busy_r) begin
                    b4_busy_r  <= 1'b1;
                    wr_idx_r   <= 3'd0;
                    b4_state_r <= B4_REQ0;
                end

                B4_REQ0:   b4_state_r <= B4_WAIT0A;
                B4_WAIT0A: b4_state_r <= B4_WAIT0B;
                B4_WAIT0B: b4_state_r <= B4_CAP0;

                B4_CAP0: begin
                    a0_r <= a_rd_data_i;
                    b0_r <= b_rd_data_i;
                    b4_state_r <= B4_WAIT1A; // req addr=1 asserted combinationally
                end

                B4_WAIT1A: b4_state_r <= B4_WAIT1B;
                B4_WAIT1B: b4_state_r <= B4_CAP1;

                B4_CAP1: begin
                    a1_r <= a_rd_data_i;
                    b1_r <= b_rd_data_i;
                    b4_state_r <= B4_WAIT2A; // req addr=2
                end

                B4_WAIT2A: b4_state_r <= B4_WAIT2B;
                B4_WAIT2B: b4_state_r <= B4_CAP2;

                B4_CAP2: begin
                    a2_r <= a_rd_data_i;
                    b2_r <= b_rd_data_i;
                    b4_state_r <= B4_WAIT3A; // req addr=3
                end

                B4_WAIT3A: b4_state_r <= B4_WAIT3B;
                B4_WAIT3B: b4_state_r <= B4_CAP3;

                B4_CAP3: begin
                    // a_rd_data_i=a3, b_rd_data_i=b3
                    // Compute all 9 products in parallel
                    // K1 products (lo half)
                    p0_r <= clmul64(a0_r, b0_r);
                    p1_r <= clmul64(a1_r, b1_r);
                    p2_r <= clmul64(a0_r ^ a1_r, b0_r ^ b1_r);
                    // K2 products (hi half)
                    p3_r <= clmul64(a2_r, b2_r);
                    p4_r <= clmul64(a_rd_data_i, b_rd_data_i);
                    p5_r <= clmul64(a2_r ^ a_rd_data_i, b2_r ^ b_rd_data_i);
                    // K3 products (cross term)
                    p6_r <= clmul64(a0_r ^ a2_r, b0_r ^ b2_r);
                    p7_r <= clmul64(a1_r ^ a_rd_data_i, b1_r ^ b_rd_data_i);
                    p8_r <= clmul64((a0_r ^ a2_r) ^ (a1_r ^ a_rd_data_i),
                                    (b0_r ^ b2_r) ^ (b1_r ^ b_rd_data_i));
                    b4_state_r <= B4_COMP_REG;
                end

                B4_COMP_REG: begin
                    // Assemble K1, K2, K3 using combinational mid wires
                    K1_r[1*DW-1:0*DW] <= p0_r[DW-1:0];
                    K1_r[2*DW-1:1*DW] <= p0_r[2*DW-1:DW] ^ b4_mid_K1_w[DW-1:0];
                    K1_r[3*DW-1:2*DW] <= p1_r[DW-1:0]   ^ b4_mid_K1_w[2*DW-1:DW];
                    K1_r[4*DW-1:3*DW] <= p1_r[2*DW-1:DW];

                    K2_r[1*DW-1:0*DW] <= p3_r[DW-1:0];
                    K2_r[2*DW-1:1*DW] <= p3_r[2*DW-1:DW] ^ b4_mid_K2_w[DW-1:0];
                    K2_r[3*DW-1:2*DW] <= p4_r[DW-1:0]   ^ b4_mid_K2_w[2*DW-1:DW];
                    K2_r[4*DW-1:3*DW] <= p4_r[2*DW-1:DW];

                    K3_r[1*DW-1:0*DW] <= p6_r[DW-1:0];
                    K3_r[2*DW-1:1*DW] <= p6_r[2*DW-1:DW] ^ b4_mid_K3_w[DW-1:0];
                    K3_r[3*DW-1:2*DW] <= p7_r[DW-1:0]   ^ b4_mid_K3_w[2*DW-1:DW];
                    K3_r[4*DW-1:3*DW] <= p7_r[2*DW-1:DW];
                    wr_idx_r   <= 3'd0;
                    b4_state_r <= B4_WRITE;
                end

                B4_WRITE: begin
                    if (wr_idx_r == 3'd7) begin
                        b4_done_r  <= 1'b1;
                        b4_busy_r  <= 1'b0;
                        b4_state_r <= B4_IDLE;
                    end else begin
                        wr_idx_r <= wr_idx_r + 1'b1;
                    end
                end

                default: b4_state_r <= B4_IDLE;
            endcase
        end
    end

    // Read address sequencer
    wire [1:0] b4_rd_addr_w =
        (b4_state_r == B4_CAP0) ? 2'd1 :
        (b4_state_r == B4_CAP1) ? 2'd2 :
        (b4_state_r == B4_CAP2) ? 2'd3 : 2'd0;
    assign a_rd_req_o  = (b4_state_r == B4_REQ0) ||
                          (b4_state_r == B4_CAP0) ||
                          (b4_state_r == B4_CAP1) ||
                          (b4_state_r == B4_CAP2);
    assign a_rd_addr_o = b4_rd_addr_w;   // SIZE=4: a_rd_addr is 2-bit, b4_rd_addr_w is 2-bit
    assign b_rd_req_o  = a_rd_req_o;
    assign b_rd_addr_o = a_rd_addr_o;

    // mid = K1 ^ K2 ^ K3
    wire [4*DW-1:0] b4_mid_w = K1_r ^ K2_r ^ K3_r;

    // Write mux (wr_idx_r: 0..7)
    wire b4_wr_en_w = (b4_state_r == B4_WRITE);
    wire [DW-1:0] b4_wr_data_w =
        (wr_idx_r == 3'd0) ? K1_r[1*DW-1:0*DW]                          :
        (wr_idx_r == 3'd1) ? K1_r[2*DW-1:1*DW]                          :
        (wr_idx_r == 3'd2) ? K1_r[3*DW-1:2*DW] ^ b4_mid_w[1*DW-1:0*DW] :
        (wr_idx_r == 3'd3) ? K1_r[4*DW-1:3*DW] ^ b4_mid_w[2*DW-1:1*DW] :
        (wr_idx_r == 3'd4) ? K2_r[1*DW-1:0*DW] ^ b4_mid_w[3*DW-1:2*DW] :
        (wr_idx_r == 3'd5) ? K2_r[2*DW-1:1*DW] ^ b4_mid_w[4*DW-1:3*DW] :
        (wr_idx_r == 3'd6) ? K2_r[3*DW-1:2*DW]                          :
                             K2_r[4*DW-1:3*DW];

    assign c_wr_en_o   = b4_wr_en_w;
    assign c_wr_addr_o = wr_idx_r;          // SIZE=4: c_wr_addr is 3-bit, wr_idx_r is 3-bit
    assign c_wr_data_o = b4_wr_data_w;
    assign done_o      = b4_done_r;
    assign debug_state_o = b4_state_r;

endmodule
