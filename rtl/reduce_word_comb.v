`ifndef REDUCE_WORD_COMB_RUNTIME_V
`define REDUCE_WORD_COMB_RUNTIME_V

`include "common.vh"

// Runtime-configurable one-word reduction step:
//   raw = low ^ high (aligned) or low ^ (high_prev >> rem) ^ (high << (64-rem))
//   last-word masking also follows rem.
module reduce_word_comb (
    input  wire [`DATA_WIDTH-1:0] low_i,
    input  wire [`DATA_WIDTH-1:0] high_prev_i,
    input  wire [`DATA_WIDTH-1:0] high_i,
    input  wire [5:0]             rem_bits_i,
    input  wire                   is_last_i,
    output reg  [`DATA_WIDTH-1:0] word_o
);

    localparam DW = `DATA_WIDTH;

    reg [DW-1:0] raw_r;
    reg [DW-1:0] mask_r;

    always @(*) begin
        if (rem_bits_i == 6'd0)
            raw_r = low_i ^ high_i;
        else
            raw_r = low_i ^ (high_prev_i >> rem_bits_i) ^ (high_i << (DW - rem_bits_i));

        if (rem_bits_i == 6'd0)
            mask_r = {DW{1'b1}};
        else
            mask_r = ({DW{1'b1}} >> (DW - rem_bits_i));

        if (is_last_i)
            word_o = raw_r & mask_r;
        else
            word_o = raw_r;
    end

endmodule

`endif
