`timescale 1ns/1ps
`include "common.vh"

/*
 * base_mul_2: Flat 128×128 GF(2) Karatsuba multiplier (SIZE=2).
 *
 * Performs A[1:0] * B[1:0] over GF(2), each word = 64-bit.
 * Interface: BRAM-style read ports for A/B (2-cycle latency), write port for C.
 *
 * Karatsuba math (SIZE=2, split L=H=1):
 *   K1 = a0*b0              → 2 words
 *   K2 = a1*b1              → 2 words
 *   K3 = (a0^a1)*(b0^b1)   → 2 words (cross-term)
 *   mid = K1 ^ K2 ^ K3
 *   c[0] = K1[0]
 *   c[1] = K1[1] ^ mid[0]
 *   c[2] = K2[0] ^ mid[1]
 *   c[3] = K2[1]
 *
 * Timing: 2 reads × 3 cycles + 1 compute + 4 writes = 11 cycles
 *   (REQ0→WAIT0A→WAIT0B→CAP0→WAIT1A→WAIT1B→CAP1→COMP_REG→WRITE×3→IDLE)
 */
module base_mul_2 #(
    parameter SIZE = 2
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
    localparam B2_IDLE     = 4'd0;
    localparam B2_REQ0     = 4'd1;   // assert req addr=0
    localparam B2_WAIT0A   = 4'd2;
    localparam B2_WAIT0B   = 4'd3;
    localparam B2_CAP0     = 4'd4;   // capture A[0],B[0]; assert req addr=1
    localparam B2_WAIT1A   = 4'd5;
    localparam B2_WAIT1B   = 4'd6;
    localparam B2_CAP1     = 4'd7;   // capture A[1],B[1]; compute k1,k2,k3
    localparam B2_COMP_REG = 4'd8;   // products registered; begin write c[0]
    localparam B2_WRITE    = 4'd9;   // write c[1..3]

    reg [3:0]      b2_state_r;
    reg            b2_busy_r;
    reg [DW-1:0]   a0_r, b0_r;
    reg [2*DW-1:0] k1_r, k2_r, k3_r;
    reg [1:0]      wr_idx_r;
    reg            b2_done_r;

    always @(posedge CLK or negedge RST) begin
        if (!RST) begin
            b2_state_r <= B2_IDLE;
            b2_busy_r  <= 1'b0;
            a0_r <= {DW{1'b0}}; b0_r <= {DW{1'b0}};
            k1_r <= {(2*DW){1'b0}};
            k2_r <= {(2*DW){1'b0}};
            k3_r <= {(2*DW){1'b0}};
            wr_idx_r  <= 2'd0;
            b2_done_r <= 1'b0;
        end else begin
            b2_done_r <= 1'b0;
            case (b2_state_r)
                B2_IDLE: if (start_i && !b2_busy_r) begin
                    b2_busy_r  <= 1'b1;
                    wr_idx_r   <= 2'd0;
                    b2_state_r <= B2_REQ0;
                end
                B2_REQ0:   b2_state_r <= B2_WAIT0A;
                B2_WAIT0A: b2_state_r <= B2_WAIT0B;
                B2_WAIT0B: b2_state_r <= B2_CAP0;
                B2_CAP0: begin
                    a0_r <= a_rd_data_i;
                    b0_r <= b_rd_data_i;
                    b2_state_r <= B2_WAIT1A;
                end
                B2_WAIT1A: b2_state_r <= B2_WAIT1B;
                B2_WAIT1B: b2_state_r <= B2_CAP1;
                B2_CAP1: begin
                    // All 3 products computed combinationally and registered here
                    k1_r <= clmul64(a0_r, b0_r);
                    k2_r <= clmul64(a_rd_data_i, b_rd_data_i);
                    k3_r <= clmul64(a0_r ^ a_rd_data_i, b0_r ^ b_rd_data_i);
                    b2_state_r <= B2_COMP_REG;
                end
                B2_COMP_REG: begin
                    wr_idx_r   <= 2'd1;
                    b2_state_r <= B2_WRITE;
                end
                B2_WRITE: begin
                    if (wr_idx_r == 2'd3) begin
                        b2_done_r  <= 1'b1;
                        b2_busy_r  <= 1'b0;
                        b2_state_r <= B2_IDLE;
                    end else begin
                        wr_idx_r <= wr_idx_r + 1'b1;
                    end
                end
                default: b2_state_r <= B2_IDLE;
            endcase
        end
    end

    assign a_rd_req_o  = (b2_state_r == B2_REQ0) || (b2_state_r == B2_CAP0);
    assign a_rd_addr_o = (b2_state_r == B2_CAP0) ? 1'b1 : 1'b0;
    assign b_rd_req_o  = a_rd_req_o;
    assign b_rd_addr_o = a_rd_addr_o;

    wire b2_wr_en_w = (b2_state_r == B2_COMP_REG) || (b2_state_r == B2_WRITE);
    // mid = K1 ^ K2 ^ K3  (Karatsuba middle term)
    wire [2*DW-1:0] b2_mid_w = k1_r ^ k2_r ^ k3_r;
    wire [DW-1:0] b2_wr_data_w =
        (wr_idx_r == 2'd0) ? k1_r[DW-1:0]                         :
        (wr_idx_r == 2'd1) ? k1_r[2*DW-1:DW] ^ b2_mid_w[DW-1:0]  :
        (wr_idx_r == 2'd2) ? k2_r[DW-1:0] ^ b2_mid_w[2*DW-1:DW]  :
                             k2_r[2*DW-1:DW];

    assign c_wr_en_o   = b2_wr_en_w;
    assign c_wr_addr_o = wr_idx_r;          // SIZE=2: c_wr_addr is 2-bit, wr_idx_r is 2-bit
    assign c_wr_data_o = b2_wr_data_w;
    assign done_o      = b2_done_r;
    assign debug_state_o = b2_state_r;

endmodule
