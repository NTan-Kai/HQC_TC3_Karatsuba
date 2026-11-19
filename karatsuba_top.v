`timescale 1ns/1ps
`include "common.vh"

/*
 * 4Flat Karatsuba wrapper with optional K1/K2/K3 parallelism.
 *
 * PAR_LEVELS = 0 : original 4Flat recursive core (sequential K1 -> K2 -> K3)
 * PAR_LEVELS = 1 : parallelize K1/K2/K3 at the current/top recursive level only
 * PAR_LEVELS = 2 : parallelize two recursive levels (resource cost grows quickly)
 *
 * Base cases SIZE=1..4 are identical to the original 4Flat implementation.
 */
module karatsuba_top #(
    parameter SIZE = `VEC_N_SIZE_64,
    parameter integer PAR_LEVELS = 1
) (
    input  wire                            CLK,
    input  wire                            RST,
    input  wire                            start_i,

    output wire                            a_rd_req_o,
    output wire [`SAFE_CLOG2(SIZE)-1:0]    a_rd_addr_o,
    input  wire [63:0]                     a_rd_data_i,

    output wire                            b_rd_req_o,
    output wire [`SAFE_CLOG2(SIZE)-1:0]    b_rd_addr_o,
    input  wire [63:0]                     b_rd_data_i,

    output wire                            c_wr_en_o,
    output wire [`SAFE_CLOG2(2*SIZE)-1:0]  c_wr_addr_o,
    output wire [63:0]                     c_wr_data_o,

    output wire                            done_o,
    output wire [3:0]                      debug_state_o
);

    generate
        if ((SIZE <= 4) || (PAR_LEVELS == 0)) begin : GEN_SEQ_OR_BASE
            // Reuse the original 4Flat implementation unchanged below this point.
            karatsuba_top_seq #(
                .SIZE(SIZE)
            ) u_seq (
                .CLK(CLK),
                .RST(RST),
                .start_i(start_i),
                .a_rd_req_o(a_rd_req_o),
                .a_rd_addr_o(a_rd_addr_o),
                .a_rd_data_i(a_rd_data_i),
                .b_rd_req_o(b_rd_req_o),
                .b_rd_addr_o(b_rd_addr_o),
                .b_rd_data_i(b_rd_data_i),
                .c_wr_en_o(c_wr_en_o),
                .c_wr_addr_o(c_wr_addr_o),
                .c_wr_data_o(c_wr_data_o),
                .done_o(done_o),
                .debug_state_o(debug_state_o)
            );
        end else begin : GEN_PARALLEL
            karatsuba_parallel_node #(
                .SIZE(SIZE),
                .PAR_LEVELS(PAR_LEVELS)
            ) u_parallel (
                .CLK(CLK),
                .RST(RST),
                .start_i(start_i),
                .a_rd_req_o(a_rd_req_o),
                .a_rd_addr_o(a_rd_addr_o),
                .a_rd_data_i(a_rd_data_i),
                .b_rd_req_o(b_rd_req_o),
                .b_rd_addr_o(b_rd_addr_o),
                .b_rd_data_i(b_rd_data_i),
                .c_wr_en_o(c_wr_en_o),
                .c_wr_addr_o(c_wr_addr_o),
                .c_wr_data_o(c_wr_data_o),
                .done_o(done_o),
                .debug_state_o(debug_state_o)
            );
        end
    endgenerate
endmodule
