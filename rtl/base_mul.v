`timescale 1ns/1ps
`include "common.vh"

// Low-latency carry-less 64x64 multiply over GF(2).
// Interface is kept identical to the previous base_mul so upper levels do not change.
// Behavior:
// - start_i pulse captures a_i/b_i and computes product.
// - done_o pulses high for one cycle on the next clock.
module base_mul(
    input  wire                      CLK,
    input  wire                      RST,
    input  wire                      start_i,
    input  wire [`DATA_WIDTH-1:0]    a_i,
    input  wire [`DATA_WIDTH-1:0]    b_i,
    output wire                      done_o,
    output wire [2*`DATA_WIDTH-1:0]  c_o
);

    localparam integer DW = `DATA_WIDTH;

    reg [2*DW-1:0] c_r;
    reg            done_r;

    // Schoolbook carry-less multiplication:
    // c = XOR over i where b[i]=1 of (a << i)
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

    always @(posedge CLK or negedge RST) begin
        if (!RST) begin
            c_r <= {(2*DW){1'b0}};
            done_r <= 1'b0;
        end else begin
            done_r <= 1'b0;
            if (start_i) begin
                c_r <= clmul64(a_i, b_i);
                done_r <= 1'b1;
            end
        end
    end

    assign c_o = c_r;
    assign done_o = done_r;

endmodule
