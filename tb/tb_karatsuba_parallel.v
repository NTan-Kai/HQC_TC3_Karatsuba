`timescale 1ns/1ps
`include "common.vh"

// Small smoke test for the parallel node.
// SIZE=5 => top node is parallel when PAR_LEVELS=1, children are SIZE_L=3 base_mul_3.
module tb_karatsuba_parallel;
    localparam integer SIZE = 5;
    localparam integer AW_IN  = `SAFE_CLOG2(SIZE);
    localparam integer AW_OUT = `SAFE_CLOG2(2*SIZE);

    reg CLK = 1'b0;
    reg RST = 1'b0;
    reg start_i = 1'b0;
    always #5 CLK = ~CLK;

    wire a_rd_req_o;
    wire [AW_IN-1:0] a_rd_addr_o;
    reg  [63:0] a_rd_data_i;
    wire b_rd_req_o;
    wire [AW_IN-1:0] b_rd_addr_o;
    reg  [63:0] b_rd_data_i;
    wire c_wr_en_o;
    wire [AW_OUT-1:0] c_wr_addr_o;
    wire [63:0] c_wr_data_o;
    wire done_o;
    wire [3:0] debug_state_o;

    reg [63:0] A [0:SIZE-1];
    reg [63:0] B [0:SIZE-1];
    reg [63:0] C [0:2*SIZE-1];
    reg [63:0] EXP [0:2*SIZE-1];

    integer i, j, k;
    integer cycles;
    reg [127:0] pp;

    function [127:0] clmul64_tb;
        input [63:0] a;
        input [63:0] b;
        integer n;
        reg [127:0] r;
        begin
            r = 128'd0;
            for (n = 0; n < 64; n = n + 1)
                if (b[n]) r = r ^ ({64'd0, a} << n);
            clmul64_tb = r;
        end
    endfunction

    karatsuba_top #(
        .SIZE(SIZE),
        .PAR_LEVELS(1)
    ) dut (
        .CLK(CLK), .RST(RST), .start_i(start_i),
        .a_rd_req_o(a_rd_req_o), .a_rd_addr_o(a_rd_addr_o), .a_rd_data_i(a_rd_data_i),
        .b_rd_req_o(b_rd_req_o), .b_rd_addr_o(b_rd_addr_o), .b_rd_data_i(b_rd_data_i),
        .c_wr_en_o(c_wr_en_o), .c_wr_addr_o(c_wr_addr_o), .c_wr_data_o(c_wr_data_o),
        .done_o(done_o), .debug_state_o(debug_state_o)
    );

    // One-cycle external memory model.
    always @(posedge CLK) begin
        if (a_rd_req_o) a_rd_data_i <= A[a_rd_addr_o];
        if (b_rd_req_o) b_rd_data_i <= B[b_rd_addr_o];
        if (c_wr_en_o)  C[c_wr_addr_o] <= c_wr_data_o;
    end

    initial begin
        a_rd_data_i = 64'd0;
        b_rd_data_i = 64'd0;
        cycles = 0;
        for (i = 0; i < 2*SIZE; i = i + 1) begin
            C[i] = 64'd0;
            EXP[i] = 64'd0;
        end

        // Deterministic non-trivial operands.
        A[0] = 64'h0123_4567_89ab_cdef;
        A[1] = 64'h1111_2222_3333_4444;
        A[2] = 64'hdead_beef_0123_4567;
        A[3] = 64'h0000_0000_0000_0001;
        A[4] = 64'h8000_0000_0000_0001;

        B[0] = 64'hfedc_ba98_7654_3210;
        B[1] = 64'h5555_aaaa_5555_aaaa;
        B[2] = 64'h1357_9bdf_2468_ace0;
        B[3] = 64'h0000_0000_0000_0003;
        B[4] = 64'h4000_0000_0000_0001;

        // Reference word-level carry-less polynomial multiplication.
        for (i = 0; i < SIZE; i = i + 1) begin
            for (j = 0; j < SIZE; j = j + 1) begin
                pp = clmul64_tb(A[i], B[j]);
                EXP[i+j]   = EXP[i+j]   ^ pp[63:0];
                EXP[i+j+1] = EXP[i+j+1] ^ pp[127:64];
            end
        end

        repeat (5) @(posedge CLK);
        RST <= 1'b1;
        repeat (3) @(posedge CLK);
        start_i <= 1'b1;
        @(posedge CLK);
        start_i <= 1'b0;

        while (!done_o && cycles < 200000) begin
            @(posedge CLK);
            cycles = cycles + 1;
        end

        if (!done_o) begin
            $display("FAIL: timeout, debug_state=%0d", debug_state_o);
            $finish;
        end

        // Allow final nonblocking RAM write to settle.
        @(posedge CLK);
        k = 0;
        for (i = 0; i < 2*SIZE; i = i + 1) begin
            if (C[i] !== EXP[i]) begin
                $display("MISMATCH word %0d: got=%016h exp=%016h", i, C[i], EXP[i]);
                k = k + 1;
            end
        end
        if (k == 0)
            $display("PASS: 4Flat Parallel SIZE=%0d, cycles=%0d", SIZE, cycles);
        else
            $display("FAIL: %0d mismatched output words", k);
        $finish;
    end
endmodule
