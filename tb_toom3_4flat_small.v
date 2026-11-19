`timescale 1ns/1ps
`include "common.vh"

// Small direct smoke test for toom3_4flat_top.
// Uses SIZE=9 so TC3 path is exercised while simulation remains manageable.
module tb_toom3_4flat_small;
    localparam integer SIZE = 9;
    localparam integer AW_IN = `SAFE_CLOG2(SIZE);
    localparam integer AW_OUT = `SAFE_CLOG2(2*SIZE);

    reg CLK, RST, start_i;
    wire a_req, b_req;
    wire [AW_IN-1:0] a_addr, b_addr;
    reg [63:0] a_data, b_data;
    wire c_we;
    wire [AW_OUT-1:0] c_addr;
    wire [63:0] c_data;
    wire done;

    reg [63:0] A [0:SIZE-1];
    reg [63:0] B [0:SIZE-1];
    reg [63:0] C [0:2*SIZE-1];
    reg [63:0] EXP [0:2*SIZE-1];
    reg [AW_IN-1:0] a_addr_q, b_addr_q;
    integer i, j, k, errors, watchdog;
    reg [127:0] pp;

    function [127:0] clmul64;
        input [63:0] x;
        input [63:0] y;
        integer bitn;
        reg [127:0] t;
        begin
            t = 128'd0;
            for (bitn=0; bitn<64; bitn=bitn+1)
                if (y[bitn]) t = t ^ ({64'd0,x} << bitn);
            clmul64 = t;
        end
    endfunction

    toom3_4flat_top #(.SIZE(SIZE), .CHILD_PAR_LEVELS(0)) dut (
        .CLK(CLK), .RST(RST), .start_i(start_i),
        .a_rd_req_o(a_req), .a_rd_addr_o(a_addr), .a_rd_data_i(a_data),
        .b_rd_req_o(b_req), .b_rd_addr_o(b_addr), .b_rd_data_i(b_data),
        .c_wr_en_o(c_we), .c_wr_addr_o(c_addr), .c_wr_data_o(c_data),
        .done_o(done), .debug_state_o()
    );

    always #5 CLK = ~CLK;

    // One-cycle synchronous source RAM model; data then remains stable until next request.
    always @(posedge CLK) begin
        if (a_req) begin a_addr_q <= a_addr; a_data <= A[a_addr]; end
        if (b_req) begin b_addr_q <= b_addr; b_data <= B[b_addr]; end
        if (c_we) C[c_addr] <= c_data;
    end

    initial begin
        CLK=0; RST=0; start_i=0; a_data=0; b_data=0; errors=0;
        a_addr_q=0; b_addr_q=0;
        for (i=0;i<SIZE;i=i+1) begin
            A[i] = 64'h1020_3040_5060_7080 ^ i;
            B[i] = 64'h8877_6655_4433_2211 ^ (i*3);
        end
        for (i=0;i<2*SIZE;i=i+1) begin C[i]=0; EXP[i]=0; end

        // Reference word convolution using 64x64 carry-less products.
        for (i=0;i<SIZE;i=i+1) begin
            for (j=0;j<SIZE;j=j+1) begin
                pp = clmul64(A[i], B[j]);
                EXP[i+j]   = EXP[i+j]   ^ pp[63:0];
                EXP[i+j+1] = EXP[i+j+1] ^ pp[127:64];
            end
        end

        repeat(6) @(posedge CLK); RST=1;
        repeat(3) @(posedge CLK);
        @(negedge CLK); start_i=1;
        @(negedge CLK); start_i=0;

        watchdog=0;
        while (!done && watchdog < 2000000) begin
            @(posedge CLK); watchdog=watchdog+1;
        end
        if (watchdog >= 2000000) begin
            $display("FAIL: timeout");
            $finish;
        end
        @(posedge CLK);

        for (k=0;k<2*SIZE;k=k+1) begin
            if (C[k] !== EXP[k]) begin
                $display("MISMATCH[%0d] got=%016h exp=%016h", k, C[k], EXP[k]);
                errors=errors+1;
            end
        end
        if (errors==0) $display("PASS: Hybrid TC3+4Flat SIZE=%0d", SIZE);
        else $display("FAIL: %0d mismatches", errors);
        $finish;
    end
endmodule
