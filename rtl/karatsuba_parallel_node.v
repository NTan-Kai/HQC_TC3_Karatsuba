`timescale 1ns/1ps
`include "common.vh"

/*
 * One recursive Karatsuba node with three independent children.
 *
 * Mathematical split:
 *   K1 = AL * BL
 *   K2 = AH * BH
 *   K3 = (AL ^ AH) * (BL ^ BH)
 *
 * Unlike the original 4Flat node, K1/K2/K3 are launched together.
 * Each child has private operand RAM and private result RAM.
 * The children recursively receive PAR_LEVELS-1.
 *
 * This module is only instantiated for SIZE >= 5 and PAR_LEVELS > 0.
 */
module karatsuba_parallel_node #(
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
    localparam integer DW      = 64;
    localparam integer SIZE_L  = (SIZE + 1) / 2;
    localparam integer SIZE_H  = SIZE / 2;
    localparam integer AW_IN   = `SAFE_CLOG2(SIZE);
    localparam integer AW_OUT  = `SAFE_CLOG2(2*SIZE);
    localparam integer AW_L    = `SAFE_CLOG2(SIZE_L);
    localparam integer AW_CL   = `SAFE_CLOG2(2*SIZE_L);

    localparam [3:0] P_IDLE       = 4'd0;
    localparam [3:0] P_LOAD       = 4'd1;
    localparam [3:0] P_PREP_K1    = 4'd2;
    localparam [3:0] P_PREP_K2    = 4'd3;
    localparam [3:0] P_PREP_K3    = 4'd4;
    localparam [3:0] P_RUN        = 4'd5;
    localparam [3:0] P_MID_REQ    = 4'd6;
    localparam [3:0] P_MID_WAIT   = 4'd7;
    localparam [3:0] P_MID_WRITE  = 4'd8;
    localparam [3:0] P_OUT_REQ    = 4'd9;
    localparam [3:0] P_OUT_WAIT   = 4'd10;
    localparam [3:0] P_OUT_WRITE  = 4'd11;
    localparam [3:0] P_DONE       = 4'd12;

    reg [3:0] state_r, state_prev_r;

    // ---------------------------------------------------------------------
    // Input A/B local storage
    // ---------------------------------------------------------------------
    reg a_we_r, b_we_r;
    reg [AW_IN-1:0] ab_wr_addr_r;
    reg [DW-1:0] a_wr_data_r, b_wr_data_r;
    reg ab_rd_en_r;
    reg [AW_IN-1:0] ab_rd_addr_r;
    wire [DW-1:0] a_local_dout_w, b_local_dout_w;

    dual_port_ram #(.AWIDTH(AW_IN), .DWIDTH(DW)) u_a_local (
        .clka(CLK), .ena(1'b1), .wea(a_we_r), .addra(ab_wr_addr_r), .dina(a_wr_data_r), .douta(),
        .clkb(CLK), .enb(ab_rd_en_r), .web(1'b0), .addrb(ab_rd_addr_r), .dinb({DW{1'b0}}), .doutb(a_local_dout_w)
    );
    dual_port_ram #(.AWIDTH(AW_IN), .DWIDTH(DW)) u_b_local (
        .clka(CLK), .ena(1'b1), .wea(b_we_r), .addra(ab_wr_addr_r), .dina(b_wr_data_r), .douta(),
        .clkb(CLK), .enb(ab_rd_en_r), .web(1'b0), .addrb(ab_rd_addr_r), .dinb({DW{1'b0}}), .doutb(b_local_dout_w)
    );

    // ---------------------------------------------------------------------
    // Private operand RAMs for K1, K2 and K3
    // ---------------------------------------------------------------------
    reg k1_a_we_r, k1_b_we_r, k2_a_we_r, k2_b_we_r, k3_a_we_r, k3_b_we_r;
    reg [AW_L-1:0] op_wr_addr_r;
    reg [DW-1:0] k1_a_wr_data_r, k1_b_wr_data_r;
    reg [DW-1:0] k2_a_wr_data_r, k2_b_wr_data_r;
    reg [DW-1:0] k3_a_wr_data_r, k3_b_wr_data_r;

    wire k1_a_req_w, k1_b_req_w, k2_a_req_w, k2_b_req_w, k3_a_req_w, k3_b_req_w;
    wire [AW_L-1:0] k1_a_addr_w, k1_b_addr_w, k2_a_addr_w, k2_b_addr_w, k3_a_addr_w, k3_b_addr_w;
    wire [DW-1:0] k1_a_dout_w, k1_b_dout_w, k2_a_dout_w, k2_b_dout_w, k3_a_dout_w, k3_b_dout_w;

    dual_port_ram #(.AWIDTH(AW_L), .DWIDTH(DW)) u_k1_a (
        .clka(CLK), .ena(1'b1), .wea(k1_a_we_r), .addra(op_wr_addr_r), .dina(k1_a_wr_data_r), .douta(),
        .clkb(CLK), .enb(k1_a_req_w), .web(1'b0), .addrb(k1_a_addr_w), .dinb({DW{1'b0}}), .doutb(k1_a_dout_w)
    );
    dual_port_ram #(.AWIDTH(AW_L), .DWIDTH(DW)) u_k1_b (
        .clka(CLK), .ena(1'b1), .wea(k1_b_we_r), .addra(op_wr_addr_r), .dina(k1_b_wr_data_r), .douta(),
        .clkb(CLK), .enb(k1_b_req_w), .web(1'b0), .addrb(k1_b_addr_w), .dinb({DW{1'b0}}), .doutb(k1_b_dout_w)
    );
    dual_port_ram #(.AWIDTH(AW_L), .DWIDTH(DW)) u_k2_a (
        .clka(CLK), .ena(1'b1), .wea(k2_a_we_r), .addra(op_wr_addr_r), .dina(k2_a_wr_data_r), .douta(),
        .clkb(CLK), .enb(k2_a_req_w), .web(1'b0), .addrb(k2_a_addr_w), .dinb({DW{1'b0}}), .doutb(k2_a_dout_w)
    );
    dual_port_ram #(.AWIDTH(AW_L), .DWIDTH(DW)) u_k2_b (
        .clka(CLK), .ena(1'b1), .wea(k2_b_we_r), .addra(op_wr_addr_r), .dina(k2_b_wr_data_r), .douta(),
        .clkb(CLK), .enb(k2_b_req_w), .web(1'b0), .addrb(k2_b_addr_w), .dinb({DW{1'b0}}), .doutb(k2_b_dout_w)
    );
    dual_port_ram #(.AWIDTH(AW_L), .DWIDTH(DW)) u_k3_a (
        .clka(CLK), .ena(1'b1), .wea(k3_a_we_r), .addra(op_wr_addr_r), .dina(k3_a_wr_data_r), .douta(),
        .clkb(CLK), .enb(k3_a_req_w), .web(1'b0), .addrb(k3_a_addr_w), .dinb({DW{1'b0}}), .doutb(k3_a_dout_w)
    );
    dual_port_ram #(.AWIDTH(AW_L), .DWIDTH(DW)) u_k3_b (
        .clka(CLK), .ena(1'b1), .wea(k3_b_we_r), .addra(op_wr_addr_r), .dina(k3_b_wr_data_r), .douta(),
        .clkb(CLK), .enb(k3_b_req_w), .web(1'b0), .addrb(k3_b_addr_w), .dinb({DW{1'b0}}), .doutb(k3_b_dout_w)
    );

    // ---------------------------------------------------------------------
    // Child result RAMs K1/K2/K3
    // ---------------------------------------------------------------------
    wire k1_c_we_w, k2_c_we_w, k3_c_we_w;
    wire [AW_CL-1:0] k1_c_addr_w, k2_c_addr_w, k3_c_addr_w;
    wire [DW-1:0] k1_c_data_w, k2_c_data_w, k3_c_data_w;
    wire k1_done_w, k2_done_w, k3_done_w;

    reg k1_res_rd_en_r, k2_res_rd_en_r, k3_res_rd_en_r;
    reg [AW_CL-1:0] k1_res_rd_addr_r, k2_res_rd_addr_r, k3_res_rd_addr_r;
    wire [DW-1:0] k1_res_dout_w, k2_res_dout_w, k3_res_dout_w;

    dual_port_ram #(.AWIDTH(AW_CL), .DWIDTH(DW)) u_k1_res (
        .clka(CLK), .ena(k1_c_we_w), .wea(k1_c_we_w), .addra(k1_c_addr_w), .dina(k1_c_data_w), .douta(),
        .clkb(CLK), .enb(k1_res_rd_en_r), .web(1'b0), .addrb(k1_res_rd_addr_r), .dinb({DW{1'b0}}), .doutb(k1_res_dout_w)
    );
    dual_port_ram #(.AWIDTH(AW_CL), .DWIDTH(DW)) u_k2_res (
        .clka(CLK), .ena(k2_c_we_w), .wea(k2_c_we_w), .addra(k2_c_addr_w), .dina(k2_c_data_w), .douta(),
        .clkb(CLK), .enb(k2_res_rd_en_r), .web(1'b0), .addrb(k2_res_rd_addr_r), .dinb({DW{1'b0}}), .doutb(k2_res_dout_w)
    );
    dual_port_ram #(.AWIDTH(AW_CL), .DWIDTH(DW)) u_k3_res (
        .clka(CLK), .ena(k3_c_we_w), .wea(k3_c_we_w), .addra(k3_c_addr_w), .dina(k3_c_data_w), .douta(),
        .clkb(CLK), .enb(k3_res_rd_en_r), .web(1'b0), .addrb(k3_res_rd_addr_r), .dinb({DW{1'b0}}), .doutb(k3_res_dout_w)
    );

    // ---------------------------------------------------------------------
    // MID = K1 ^ K2 ^ K3 RAM. K2 is masked above its real 2*SIZE_H range.
    // ---------------------------------------------------------------------
    reg mid_we_r, mid_rd_en_r;
    reg [AW_CL-1:0] mid_wr_addr_r, mid_rd_addr_r;
    reg [DW-1:0] mid_wr_data_r;
    wire [DW-1:0] mid_dout_w;
    dual_port_ram #(.AWIDTH(AW_CL), .DWIDTH(DW)) u_mid (
        .clka(CLK), .ena(1'b1), .wea(mid_we_r), .addra(mid_wr_addr_r), .dina(mid_wr_data_r), .douta(),
        .clkb(CLK), .enb(mid_rd_en_r), .web(1'b0), .addrb(mid_rd_addr_r), .dinb({DW{1'b0}}), .doutb(mid_dout_w)
    );

    // ---------------------------------------------------------------------
    // Three independent recursive children
    // ---------------------------------------------------------------------
    wire child_start_w = (state_r == P_RUN) && (state_prev_r != P_RUN);

    karatsuba_top #(.SIZE(SIZE_L), .PAR_LEVELS(PAR_LEVELS-1)) child_k1 (
        .CLK(CLK), .RST(RST), .start_i(child_start_w),
        .a_rd_req_o(k1_a_req_w), .a_rd_addr_o(k1_a_addr_w), .a_rd_data_i(k1_a_dout_w),
        .b_rd_req_o(k1_b_req_w), .b_rd_addr_o(k1_b_addr_w), .b_rd_data_i(k1_b_dout_w),
        .c_wr_en_o(k1_c_we_w), .c_wr_addr_o(k1_c_addr_w), .c_wr_data_o(k1_c_data_w),
        .done_o(k1_done_w), .debug_state_o()
    );
    karatsuba_top #(.SIZE(SIZE_L), .PAR_LEVELS(PAR_LEVELS-1)) child_k2 (
        .CLK(CLK), .RST(RST), .start_i(child_start_w),
        .a_rd_req_o(k2_a_req_w), .a_rd_addr_o(k2_a_addr_w), .a_rd_data_i(k2_a_dout_w),
        .b_rd_req_o(k2_b_req_w), .b_rd_addr_o(k2_b_addr_w), .b_rd_data_i(k2_b_dout_w),
        .c_wr_en_o(k2_c_we_w), .c_wr_addr_o(k2_c_addr_w), .c_wr_data_o(k2_c_data_w),
        .done_o(k2_done_w), .debug_state_o()
    );
    karatsuba_top #(.SIZE(SIZE_L), .PAR_LEVELS(PAR_LEVELS-1)) child_k3 (
        .CLK(CLK), .RST(RST), .start_i(child_start_w),
        .a_rd_req_o(k3_a_req_w), .a_rd_addr_o(k3_a_addr_w), .a_rd_data_i(k3_a_dout_w),
        .b_rd_req_o(k3_b_req_w), .b_rd_addr_o(k3_b_addr_w), .b_rd_data_i(k3_b_dout_w),
        .c_wr_en_o(k3_c_we_w), .c_wr_addr_o(k3_c_addr_w), .c_wr_data_o(k3_c_data_w),
        .done_o(k3_done_w), .debug_state_o()
    );

    // ---------------------------------------------------------------------
    // External interface and state-dependent output
    // ---------------------------------------------------------------------
    reg [AW_IN-1:0] load_idx_r;
    reg [1:0] load_step_r;
    reg [AW_L:0] prep_idx_r;
    reg [2:0] prep_step_r;
    reg [DW-1:0] k3_low_a_r, k3_low_b_r;
    reg k1_done_seen_r, k2_done_seen_r, k3_done_seen_r;
    reg [AW_CL:0] mid_idx_r;
    reg [AW_OUT:0] out_idx_r;

    reg c_wr_en_r;
    reg [AW_OUT-1:0] c_wr_addr_r;
    reg [DW-1:0] c_wr_data_r;
    reg done_r;

    assign a_rd_req_o  = (state_r == P_LOAD) && (load_step_r == 2'd0);
    assign b_rd_req_o  = (state_r == P_LOAD) && (load_step_r == 2'd0);
    assign a_rd_addr_o = load_idx_r;
    assign b_rd_addr_o = load_idx_r;
    assign c_wr_en_o   = c_wr_en_r;
    assign c_wr_addr_o = c_wr_addr_r;
    assign c_wr_data_o = c_wr_data_r;
    assign done_o      = done_r;
    assign debug_state_o = state_r;

    // ---------------------------------------------------------------------
    // Main FSM
    // ---------------------------------------------------------------------
    always @(posedge CLK or negedge RST) begin
        if (!RST) begin
            state_r <= P_IDLE;
            state_prev_r <= P_IDLE;
            load_idx_r <= {AW_IN{1'b0}};
            load_step_r <= 2'd0;
            prep_idx_r <= {(AW_L+1){1'b0}};
            prep_step_r <= 3'd0;
            k3_low_a_r <= {DW{1'b0}};
            k3_low_b_r <= {DW{1'b0}};
            k1_done_seen_r <= 1'b0;
            k2_done_seen_r <= 1'b0;
            k3_done_seen_r <= 1'b0;
            mid_idx_r <= {(AW_CL+1){1'b0}};
            out_idx_r <= {(AW_OUT+1){1'b0}};

            a_we_r <= 1'b0; b_we_r <= 1'b0;
            ab_wr_addr_r <= {AW_IN{1'b0}};
            a_wr_data_r <= {DW{1'b0}}; b_wr_data_r <= {DW{1'b0}};
            ab_rd_en_r <= 1'b0; ab_rd_addr_r <= {AW_IN{1'b0}};

            k1_a_we_r <= 1'b0; k1_b_we_r <= 1'b0;
            k2_a_we_r <= 1'b0; k2_b_we_r <= 1'b0;
            k3_a_we_r <= 1'b0; k3_b_we_r <= 1'b0;
            op_wr_addr_r <= {AW_L{1'b0}};
            k1_a_wr_data_r <= {DW{1'b0}}; k1_b_wr_data_r <= {DW{1'b0}};
            k2_a_wr_data_r <= {DW{1'b0}}; k2_b_wr_data_r <= {DW{1'b0}};
            k3_a_wr_data_r <= {DW{1'b0}}; k3_b_wr_data_r <= {DW{1'b0}};

            k1_res_rd_en_r <= 1'b0; k2_res_rd_en_r <= 1'b0; k3_res_rd_en_r <= 1'b0;
            k1_res_rd_addr_r <= {AW_CL{1'b0}};
            k2_res_rd_addr_r <= {AW_CL{1'b0}};
            k3_res_rd_addr_r <= {AW_CL{1'b0}};
            mid_we_r <= 1'b0; mid_rd_en_r <= 1'b0;
            mid_wr_addr_r <= {AW_CL{1'b0}}; mid_rd_addr_r <= {AW_CL{1'b0}};
            mid_wr_data_r <= {DW{1'b0}};

            c_wr_en_r <= 1'b0;
            c_wr_addr_r <= {AW_OUT{1'b0}};
            c_wr_data_r <= {DW{1'b0}};
            done_r <= 1'b0;
        end else begin
            state_prev_r <= state_r;

            // One-cycle strobes default low.
            a_we_r <= 1'b0; b_we_r <= 1'b0; ab_rd_en_r <= 1'b0;
            k1_a_we_r <= 1'b0; k1_b_we_r <= 1'b0;
            k2_a_we_r <= 1'b0; k2_b_we_r <= 1'b0;
            k3_a_we_r <= 1'b0; k3_b_we_r <= 1'b0;
            k1_res_rd_en_r <= 1'b0; k2_res_rd_en_r <= 1'b0; k3_res_rd_en_r <= 1'b0;
            mid_we_r <= 1'b0; mid_rd_en_r <= 1'b0;
            c_wr_en_r <= 1'b0;
            done_r <= 1'b0;

            case (state_r)
                P_IDLE: begin
                    load_idx_r <= {AW_IN{1'b0}};
                    load_step_r <= 2'd0;
                    prep_idx_r <= {(AW_L+1){1'b0}};
                    prep_step_r <= 3'd0;
                    k1_done_seen_r <= 1'b0;
                    k2_done_seen_r <= 1'b0;
                    k3_done_seen_r <= 1'b0;
                    if (start_i)
                        state_r <= P_LOAD;
                end

                // External A/B request -> wait -> capture, one word at a time.
                P_LOAD: begin
                    if (load_step_r == 2'd0) begin
                        load_step_r <= 2'd1;
                    end else if (load_step_r == 2'd1) begin
                        load_step_r <= 2'd2;
                    end else begin
                        ab_wr_addr_r <= load_idx_r;
                        a_wr_data_r <= a_rd_data_i;
                        b_wr_data_r <= b_rd_data_i;
                        a_we_r <= 1'b1;
                        b_we_r <= 1'b1;
                        load_step_r <= 2'd0;
                        if (load_idx_r == SIZE-1) begin
                            load_idx_r <= {AW_IN{1'b0}};
                            prep_idx_r <= 0;
                            prep_step_r <= 0;
                            state_r <= P_PREP_K1;
                        end else begin
                            load_idx_r <= load_idx_r + 1'b1;
                        end
                    end
                end

                // K1 operand = low halves AL, BL.
                P_PREP_K1: begin
                    if (prep_step_r == 3'd0) begin
                        ab_rd_en_r <= 1'b1;
                        ab_rd_addr_r <= prep_idx_r[AW_IN-1:0];
                        prep_step_r <= 3'd1;
                    end else if (prep_step_r == 3'd1) begin
                        prep_step_r <= 3'd2;
                    end else begin
                        op_wr_addr_r <= prep_idx_r[AW_L-1:0];
                        k1_a_wr_data_r <= a_local_dout_w;
                        k1_b_wr_data_r <= b_local_dout_w;
                        k1_a_we_r <= 1'b1; k1_b_we_r <= 1'b1;
                        prep_step_r <= 3'd0;
                        if (prep_idx_r == SIZE_L-1) begin
                            prep_idx_r <= 0;
                            state_r <= P_PREP_K2;
                        end else begin
                            prep_idx_r <= prep_idx_r + 1'b1;
                        end
                    end
                end

                // K2 operand = high halves AH, BH padded to SIZE_L when SIZE is odd.
                P_PREP_K2: begin
                    if (prep_idx_r >= SIZE_H) begin
                        op_wr_addr_r <= prep_idx_r[AW_L-1:0];
                        k2_a_wr_data_r <= {DW{1'b0}};
                        k2_b_wr_data_r <= {DW{1'b0}};
                        k2_a_we_r <= 1'b1; k2_b_we_r <= 1'b1;
                        prep_step_r <= 3'd0;
                        if (prep_idx_r == SIZE_L-1) begin
                            prep_idx_r <= 0;
                            state_r <= P_PREP_K3;
                        end else begin
                            prep_idx_r <= prep_idx_r + 1'b1;
                        end
                    end else if (prep_step_r == 3'd0) begin
                        ab_rd_en_r <= 1'b1;
                        ab_rd_addr_r <= (SIZE_L + prep_idx_r);
                        prep_step_r <= 3'd1;
                    end else if (prep_step_r == 3'd1) begin
                        prep_step_r <= 3'd2;
                    end else begin
                        op_wr_addr_r <= prep_idx_r[AW_L-1:0];
                        k2_a_wr_data_r <= a_local_dout_w;
                        k2_b_wr_data_r <= b_local_dout_w;
                        k2_a_we_r <= 1'b1; k2_b_we_r <= 1'b1;
                        prep_step_r <= 3'd0;
                        if (prep_idx_r == SIZE_L-1) begin
                            prep_idx_r <= 0;
                            state_r <= P_PREP_K3;
                        end else begin
                            prep_idx_r <= prep_idx_r + 1'b1;
                        end
                    end
                end

                // K3 operand = AL^AH, BL^BH. For odd SIZE the last AH/BH word is zero.
                P_PREP_K3: begin
                    case (prep_step_r)
                        3'd0: begin
                            ab_rd_en_r <= 1'b1;
                            ab_rd_addr_r <= prep_idx_r[AW_IN-1:0];
                            prep_step_r <= 3'd1;
                        end
                        3'd1: prep_step_r <= 3'd2;
                        3'd2: begin
                            k3_low_a_r <= a_local_dout_w;
                            k3_low_b_r <= b_local_dout_w;
                            if (prep_idx_r < SIZE_H) begin
                                ab_rd_en_r <= 1'b1;
                                ab_rd_addr_r <= (SIZE_L + prep_idx_r);
                                prep_step_r <= 3'd3;
                            end else begin
                                op_wr_addr_r <= prep_idx_r[AW_L-1:0];
                                k3_a_wr_data_r <= a_local_dout_w;
                                k3_b_wr_data_r <= b_local_dout_w;
                                k3_a_we_r <= 1'b1; k3_b_we_r <= 1'b1;
                                prep_step_r <= 3'd0;
                                if (prep_idx_r == SIZE_L-1) begin
                                    prep_idx_r <= 0;
                                    state_r <= P_RUN;
                                end else begin
                                    prep_idx_r <= prep_idx_r + 1'b1;
                                end
                            end
                        end
                        3'd3: prep_step_r <= 3'd4;
                        default: begin
                            op_wr_addr_r <= prep_idx_r[AW_L-1:0];
                            k3_a_wr_data_r <= k3_low_a_r ^ a_local_dout_w;
                            k3_b_wr_data_r <= k3_low_b_r ^ b_local_dout_w;
                            k3_a_we_r <= 1'b1; k3_b_we_r <= 1'b1;
                            prep_step_r <= 3'd0;
                            if (prep_idx_r == SIZE_L-1) begin
                                prep_idx_r <= 0;
                                state_r <= P_RUN;
                            end else begin
                                prep_idx_r <= prep_idx_r + 1'b1;
                            end
                        end
                    endcase
                end

                // Three children execute concurrently. Sticky flags tolerate different done cycles.
                P_RUN: begin
                    if (k1_done_w) k1_done_seen_r <= 1'b1;
                    if (k2_done_w) k2_done_seen_r <= 1'b1;
                    if (k3_done_w) k3_done_seen_r <= 1'b1;
                    if ((k1_done_seen_r || k1_done_w) &&
                        (k2_done_seen_r || k2_done_w) &&
                        (k3_done_seen_r || k3_done_w)) begin
                        mid_idx_r <= 0;
                        state_r <= P_MID_REQ;
                    end
                end

                // Build MID[t] = K1[t] ^ K2[t] ^ K3[t].
                P_MID_REQ: begin
                    k1_res_rd_en_r <= 1'b1;
                    k3_res_rd_en_r <= 1'b1;
                    k1_res_rd_addr_r <= mid_idx_r[AW_CL-1:0];
                    k3_res_rd_addr_r <= mid_idx_r[AW_CL-1:0];
                    if (mid_idx_r < 2*SIZE_H) begin
                        k2_res_rd_en_r <= 1'b1;
                        k2_res_rd_addr_r <= mid_idx_r[AW_CL-1:0];
                    end
                    state_r <= P_MID_WAIT;
                end
                P_MID_WAIT: begin
                    state_r <= P_MID_WRITE;
                end
                P_MID_WRITE: begin
                    mid_we_r <= 1'b1;
                    mid_wr_addr_r <= mid_idx_r[AW_CL-1:0];
                    mid_wr_data_r <= k1_res_dout_w ^ k3_res_dout_w ^
                                     ((mid_idx_r < 2*SIZE_H) ? k2_res_dout_w : {DW{1'b0}});
                    if (mid_idx_r == 2*SIZE_L-1) begin
                        out_idx_r <= 0;
                        state_r <= P_OUT_REQ;
                    end else begin
                        mid_idx_r <= mid_idx_r + 1'b1;
                        state_r <= P_MID_REQ;
                    end
                end

                // C = K1 + (MID << SIZE_L words) + (K2 << 2*SIZE_L words).
                P_OUT_REQ: begin
                    if (out_idx_r < 2*SIZE_L) begin
                        k1_res_rd_en_r <= 1'b1;
                        k1_res_rd_addr_r <= out_idx_r[AW_CL-1:0];
                    end
                    if ((out_idx_r >= SIZE_L) && (out_idx_r < 3*SIZE_L)) begin
                        mid_rd_en_r <= 1'b1;
                        mid_rd_addr_r <= (out_idx_r - SIZE_L);
                    end
                    if ((out_idx_r >= 2*SIZE_L) && (out_idx_r < (2*SIZE_L + 2*SIZE_H))) begin
                        k2_res_rd_en_r <= 1'b1;
                        k2_res_rd_addr_r <= (out_idx_r - 2*SIZE_L);
                    end
                    state_r <= P_OUT_WAIT;
                end
                P_OUT_WAIT: begin
                    state_r <= P_OUT_WRITE;
                end
                P_OUT_WRITE: begin
                    c_wr_en_r <= 1'b1;
                    c_wr_addr_r <= out_idx_r[AW_OUT-1:0];
                    c_wr_data_r <=
                        ((out_idx_r < 2*SIZE_L) ? k1_res_dout_w : {DW{1'b0}}) ^
                        (((out_idx_r >= SIZE_L) && (out_idx_r < 3*SIZE_L)) ? mid_dout_w : {DW{1'b0}}) ^
                        (((out_idx_r >= 2*SIZE_L) && (out_idx_r < (2*SIZE_L + 2*SIZE_H))) ? k2_res_dout_w : {DW{1'b0}});
                    if (out_idx_r == 2*SIZE-1) begin
                        state_r <= P_DONE;
                    end else begin
                        out_idx_r <= out_idx_r + 1'b1;
                        state_r <= P_OUT_REQ;
                    end
                end

                P_DONE: begin
                    done_r <= 1'b1;
                    if (!start_i)
                        state_r <= P_IDLE;
                end

                default: state_r <= P_IDLE;
            endcase
        end
    end
endmodule
