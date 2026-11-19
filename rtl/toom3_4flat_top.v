`timescale 1ns/1ps
`include "common.vh"

/*
 * Hybrid Toom-Cook-3 + 4Flat Karatsuba multiplier.
 *
 * Intended use: top-level HQC polynomial multiplication (e.g. SIZE=277 for HQC-128).
 * The top level is split into 3 blocks of M=ceil(SIZE/3) 64-bit words:
 *
 *   A(X) = a0 + a1*X + a2*X^2
 *   B(X) = b0 + b1*X + b2*X^2
 *   X    = x^(64*M)
 *
 * Toom-3 uses five point multiplications at 0, infinity, 1, W and W+1,
 * where W=x^64 is a one-word shift.  All five point multiplications reuse ONE
 * child multiplier of fixed size TC_SIZE=M+2, so the child can be the existing
 * 4Flat/Karatsuba core without replicating five arithmetic subtrees.
 *
 * Point products:
 *   P0   = a0*b0
 *   P4   = a2*b2
 *   P1   = (a0+a1+a2)*(b0+b1+b2)
 *   PW   = (a0 + W*a1 + W^2*a2) * (b0 + W*b1 + W^2*b2)
 *   PW1  = (a0+a1+a2 + W*a1 + W^2*a2)
 *          *(b0+b1+b2 + W*b1 + W^2*b2)
 *
 * Word-wise interpolation follows Algorithm 2 from the supplied HQC Cortex-M4
 * paper. Division by (1+W) is implemented as a prefix XOR recurrence; division
 * by W*(1+W) is a one-word look-ahead followed by the same recurrence.
 *
 * CHILD_PAR_LEVELS controls only the reused 4Flat child:
 *   0 = sequential 4Flat child (recommended first area-efficient benchmark)
 *   1 = parallelize one level inside the child (faster, higher area)
 *
 * Interface is intentionally identical to karatsuba_top so vect_mul_top can
 * switch between the old Karatsuba core and this hybrid core with a generate.
 */
module toom3_4flat_top #(
    parameter integer SIZE = `VEC_N_SIZE_64,
    parameter integer CHILD_PAR_LEVELS = 0
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
    localparam integer DW       = 64;
    localparam integer M        = (SIZE + 2) / 3;   // ceil(SIZE/3)
    localparam integer TC_SIZE  = M + 2;            // W/W^2 evaluation growth
    localparam integer PWORDS   = 2 * TC_SIZE;

    localparam integer AW_IN    = `SAFE_CLOG2(SIZE);
    localparam integer AW_OUT   = `SAFE_CLOG2(2*SIZE);
    localparam integer AW_M     = `SAFE_CLOG2(M);
    localparam integer AW_TC    = `SAFE_CLOG2(TC_SIZE);
    localparam integer AW_P     = `SAFE_CLOG2(PWORDS);

    // Five Toom evaluation points.
    localparam [2:0] PT_0   = 3'd0;
    localparam [2:0] PT_INF = 3'd1;
    localparam [2:0] PT_1   = 3'd2;
    localparam [2:0] PT_W   = 3'd3;
    localparam [2:0] PT_W1  = 3'd4;

    // Compact 16-state controller.
    localparam [3:0] T_IDLE       = 4'd0;
    localparam [3:0] T_LOAD_REQ   = 4'd1;
    localparam [3:0] T_LOAD_W1    = 4'd2;
    localparam [3:0] T_LOAD_W2    = 4'd3;
    localparam [3:0] T_LOAD_CAP   = 4'd4;
    localparam [3:0] T_PREP_REQ   = 4'd5;
    localparam [3:0] T_PREP_WAIT  = 4'd6;
    localparam [3:0] T_PREP_WRITE = 4'd7;
    localparam [3:0] T_RUN        = 4'd8;
    localparam [3:0] T_INT_REQ    = 4'd9;
    localparam [3:0] T_INT_WAIT   = 4'd10;
    localparam [3:0] T_INT_WRITE  = 4'd11;
    localparam [3:0] T_OUT_REQ    = 4'd12;
    localparam [3:0] T_OUT_WAIT   = 4'd13;
    localparam [3:0] T_OUT_WRITE  = 4'd14;
    localparam [3:0] T_DONE       = 4'd15;

    reg [3:0] state_r;
    reg [AW_IN-1:0] load_idx_r;
    reg [AW_TC-1:0] prep_idx_r;
    reg [2:0] point_r;
    reg [AW_P-1:0] interp_idx_r;
    reg [AW_OUT-1:0] out_idx_r;
    reg child_start_r;

    // Interpolation recurrence state.
    reg [DW-1:0] c2_prev_r, c3_prev_r;
    reg [DW-1:0] inf_d1_r, inf_d2_r, inf_d3_r;

    // ------------------------------------------------------------------
    // Split memories a0/a1/a2 and b0/b1/b2.
    // Port A: load write OR current-index read during point preparation.
    // Port B: shifted read (a1[j-1], a2[j-2], etc.) during preparation.
    // ------------------------------------------------------------------
    wire load_cap_w = (state_r == T_LOAD_CAP);
    wire prep_req_w = (state_r == T_PREP_REQ);

    wire load_to_0_w = (load_idx_r < M);
    wire load_to_1_w = (load_idx_r >= M) && (load_idx_r < (2*M));
    wire load_to_2_w = (load_idx_r >= (2*M));

    wire [AW_M-1:0] load_addr0_w = load_idx_r[AW_M-1:0];
    wire [AW_M-1:0] load_addr1_w = (load_idx_r - M);
    wire [AW_M-1:0] load_addr2_w = (load_idx_r - 2*M);

    wire prep_cur_in_m_w = (prep_idx_r < M);
    wire prep_sh1_in_m_w = (prep_idx_r >= 1) && ((prep_idx_r - 1'b1) < M);
    wire prep_sh2_in_m_w = (prep_idx_r >= 2) && ((prep_idx_r - 2'd2) < M);

    wire a0_cur_valid_w = prep_cur_in_m_w && (prep_idx_r < SIZE);
    wire a1_cur_valid_w = prep_cur_in_m_w && ((M + prep_idx_r) < SIZE);
    wire a2_cur_valid_w = prep_cur_in_m_w && ((2*M + prep_idx_r) < SIZE);
    wire a1_sh1_valid_w = prep_sh1_in_m_w && ((M + prep_idx_r - 1) < SIZE);
    wire a2_sh2_valid_w = prep_sh2_in_m_w && ((2*M + prep_idx_r - 2) < SIZE);

    wire [AW_M-1:0] prep_cur_addr_w = prep_cur_in_m_w ? prep_idx_r[AW_M-1:0] : {AW_M{1'b0}};
    wire [AW_M-1:0] prep_sh1_addr_w = prep_sh1_in_m_w ? (prep_idx_r - 1'b1) : {AW_M{1'b0}};
    wire [AW_M-1:0] prep_sh2_addr_w = prep_sh2_in_m_w ? (prep_idx_r - 2'd2) : {AW_M{1'b0}};

    wire [DW-1:0] a0_cur_mem_w, a1_cur_mem_w, a2_cur_mem_w;
    wire [DW-1:0] a1_sh1_mem_w, a2_sh2_mem_w;
    wire [DW-1:0] b0_cur_mem_w, b1_cur_mem_w, b2_cur_mem_w;
    wire [DW-1:0] b1_sh1_mem_w, b2_sh2_mem_w;

    // A0
    dual_port_ram #(.AWIDTH(AW_M), .DWIDTH(DW)) tc_a0_ram (
        .clka(CLK),
        .ena((load_cap_w && load_to_0_w) || prep_req_w),
        .wea(load_cap_w && load_to_0_w),
        .addra(load_cap_w ? load_addr0_w : prep_cur_addr_w),
        .dina(a_rd_data_i), .douta(a0_cur_mem_w),
        .clkb(CLK), .enb(1'b0), .web(1'b0), .addrb({AW_M{1'b0}}),
        .dinb({DW{1'b0}}), .doutb()
    );

    // A1: current on A, j-1 on B.
    dual_port_ram #(.AWIDTH(AW_M), .DWIDTH(DW)) tc_a1_ram (
        .clka(CLK),
        .ena((load_cap_w && load_to_1_w) || prep_req_w),
        .wea(load_cap_w && load_to_1_w),
        .addra(load_cap_w ? load_addr1_w : prep_cur_addr_w),
        .dina(a_rd_data_i), .douta(a1_cur_mem_w),
        .clkb(CLK), .enb(prep_req_w && prep_sh1_in_m_w), .web(1'b0),
        .addrb(prep_sh1_addr_w), .dinb({DW{1'b0}}), .doutb(a1_sh1_mem_w)
    );

    // A2: current on A, j-2 on B.
    dual_port_ram #(.AWIDTH(AW_M), .DWIDTH(DW)) tc_a2_ram (
        .clka(CLK),
        .ena((load_cap_w && load_to_2_w) || prep_req_w),
        .wea(load_cap_w && load_to_2_w),
        .addra(load_cap_w ? load_addr2_w : prep_cur_addr_w),
        .dina(a_rd_data_i), .douta(a2_cur_mem_w),
        .clkb(CLK), .enb(prep_req_w && prep_sh2_in_m_w), .web(1'b0),
        .addrb(prep_sh2_addr_w), .dinb({DW{1'b0}}), .doutb(a2_sh2_mem_w)
    );

    // B0
    dual_port_ram #(.AWIDTH(AW_M), .DWIDTH(DW)) tc_b0_ram (
        .clka(CLK),
        .ena((load_cap_w && load_to_0_w) || prep_req_w),
        .wea(load_cap_w && load_to_0_w),
        .addra(load_cap_w ? load_addr0_w : prep_cur_addr_w),
        .dina(b_rd_data_i), .douta(b0_cur_mem_w),
        .clkb(CLK), .enb(1'b0), .web(1'b0), .addrb({AW_M{1'b0}}),
        .dinb({DW{1'b0}}), .doutb()
    );

    // B1
    dual_port_ram #(.AWIDTH(AW_M), .DWIDTH(DW)) tc_b1_ram (
        .clka(CLK),
        .ena((load_cap_w && load_to_1_w) || prep_req_w),
        .wea(load_cap_w && load_to_1_w),
        .addra(load_cap_w ? load_addr1_w : prep_cur_addr_w),
        .dina(b_rd_data_i), .douta(b1_cur_mem_w),
        .clkb(CLK), .enb(prep_req_w && prep_sh1_in_m_w), .web(1'b0),
        .addrb(prep_sh1_addr_w), .dinb({DW{1'b0}}), .doutb(b1_sh1_mem_w)
    );

    // B2
    dual_port_ram #(.AWIDTH(AW_M), .DWIDTH(DW)) tc_b2_ram (
        .clka(CLK),
        .ena((load_cap_w && load_to_2_w) || prep_req_w),
        .wea(load_cap_w && load_to_2_w),
        .addra(load_cap_w ? load_addr2_w : prep_cur_addr_w),
        .dina(b_rd_data_i), .douta(b2_cur_mem_w),
        .clkb(CLK), .enb(prep_req_w && prep_sh2_in_m_w), .web(1'b0),
        .addrb(prep_sh2_addr_w), .dinb({DW{1'b0}}), .doutb(b2_sh2_mem_w)
    );

    // Gate padded/invalid portions of the three blocks.
    wire [DW-1:0] a0_cur_w = a0_cur_valid_w ? a0_cur_mem_w : {DW{1'b0}};
    wire [DW-1:0] a1_cur_w = a1_cur_valid_w ? a1_cur_mem_w : {DW{1'b0}};
    wire [DW-1:0] a2_cur_w = a2_cur_valid_w ? a2_cur_mem_w : {DW{1'b0}};
    wire [DW-1:0] a1_sh1_w = a1_sh1_valid_w ? a1_sh1_mem_w : {DW{1'b0}};
    wire [DW-1:0] a2_sh2_w = a2_sh2_valid_w ? a2_sh2_mem_w : {DW{1'b0}};

    wire [DW-1:0] b0_cur_w = a0_cur_valid_w ? b0_cur_mem_w : {DW{1'b0}};
    wire [DW-1:0] b1_cur_w = a1_cur_valid_w ? b1_cur_mem_w : {DW{1'b0}};
    wire [DW-1:0] b2_cur_w = a2_cur_valid_w ? b2_cur_mem_w : {DW{1'b0}};
    wire [DW-1:0] b1_sh1_w = a1_sh1_valid_w ? b1_sh1_mem_w : {DW{1'b0}};
    wire [DW-1:0] b2_sh2_w = a2_sh2_valid_w ? b2_sh2_mem_w : {DW{1'b0}};

    reg [DW-1:0] eval_a_w, eval_b_w;
    always @* begin
        case (point_r)
            PT_0: begin
                eval_a_w = a0_cur_w;
                eval_b_w = b0_cur_w;
            end
            PT_INF: begin
                eval_a_w = a2_cur_w;
                eval_b_w = b2_cur_w;
            end
            PT_1: begin
                eval_a_w = a0_cur_w ^ a1_cur_w ^ a2_cur_w;
                eval_b_w = b0_cur_w ^ b1_cur_w ^ b2_cur_w;
            end
            PT_W: begin
                eval_a_w = a0_cur_w ^ a1_sh1_w ^ a2_sh2_w;
                eval_b_w = b0_cur_w ^ b1_sh1_w ^ b2_sh2_w;
            end
            default: begin // PT_W1
                eval_a_w = a0_cur_w ^ a1_cur_w ^ a2_cur_w ^ a1_sh1_w ^ a2_sh2_w;
                eval_b_w = b0_cur_w ^ b1_cur_w ^ b2_cur_w ^ b1_sh1_w ^ b2_sh2_w;
            end
        endcase
    end

    // ------------------------------------------------------------------
    // Reused 4Flat child and its operand RAM.
    // ------------------------------------------------------------------
    wire child_a_req_w, child_b_req_w;
    wire [AW_TC-1:0] child_a_addr_w, child_b_addr_w;
    wire [DW-1:0] child_a_data_w, child_b_data_w;
    wire child_c_we_w;
    wire [AW_P-1:0] child_c_addr_w;
    wire [DW-1:0] child_c_data_w;
    wire child_done_w;

    dual_port_ram #(.AWIDTH(AW_TC), .DWIDTH(DW)) tc_child_a_ram (
        .clka(CLK), .ena(state_r == T_PREP_WRITE), .wea(state_r == T_PREP_WRITE),
        .addra(prep_idx_r), .dina(eval_a_w), .douta(),
        .clkb(CLK), .enb(child_a_req_w), .web(1'b0), .addrb(child_a_addr_w),
        .dinb({DW{1'b0}}), .doutb(child_a_data_w)
    );

    dual_port_ram #(.AWIDTH(AW_TC), .DWIDTH(DW)) tc_child_b_ram (
        .clka(CLK), .ena(state_r == T_PREP_WRITE), .wea(state_r == T_PREP_WRITE),
        .addra(prep_idx_r), .dina(eval_b_w), .douta(),
        .clkb(CLK), .enb(child_b_req_w), .web(1'b0), .addrb(child_b_addr_w),
        .dinb({DW{1'b0}}), .doutb(child_b_data_w)
    );

    karatsuba_top #(
        .SIZE(TC_SIZE),
        .PAR_LEVELS(CHILD_PAR_LEVELS)
    ) tc_child (
        .CLK(CLK), .RST(RST), .start_i(child_start_r),
        .a_rd_req_o(child_a_req_w), .a_rd_addr_o(child_a_addr_w), .a_rd_data_i(child_a_data_w),
        .b_rd_req_o(child_b_req_w), .b_rd_addr_o(child_b_addr_w), .b_rd_data_i(child_b_data_w),
        .c_wr_en_o(child_c_we_w), .c_wr_addr_o(child_c_addr_w), .c_wr_data_o(child_c_data_w),
        .done_o(child_done_w), .debug_state_o()
    );

    // ------------------------------------------------------------------
    // Point-product RAMs. During RUN port A receives child output.
    // During interpolation: A=current index, B=next index.
    // During final output: P0/PINF port A are reused for coefficient reads.
    // ------------------------------------------------------------------
    wire int_req_w = (state_r == T_INT_REQ);
    wire out_req_w = (state_r == T_OUT_REQ);
    wire interp_next_valid_w = (interp_idx_r < (PWORDS-1));
    wire [AW_P-1:0] interp_next_addr_w = interp_idx_r + 1'b1;

    wire child_wr_p0_w   = (state_r == T_RUN) && child_c_we_w && (point_r == PT_0);
    wire child_wr_pinf_w = (state_r == T_RUN) && child_c_we_w && (point_r == PT_INF);
    wire child_wr_p1_w   = (state_r == T_RUN) && child_c_we_w && (point_r == PT_1);
    wire child_wr_pw_w   = (state_r == T_RUN) && child_c_we_w && (point_r == PT_W);
    wire child_wr_pw1_w  = (state_r == T_RUN) && child_c_we_w && (point_r == PT_W1);

    // Final-output read address/validity for c0=P0 and c4=PINF.
    wire out_p0_valid_w = (out_idx_r < PWORDS);
    wire out_p4_valid_w = (out_idx_r >= (4*M)) && ((out_idx_r - 4*M) < PWORDS);
    wire [AW_P-1:0] out_p0_addr_w = out_idx_r[AW_P-1:0];
    wire [AW_P-1:0] out_p4_addr_w = out_idx_r - 4*M;

    wire [DW-1:0] p0_cur_mem_w, p0_next_mem_w;
    wire [DW-1:0] pinf_cur_mem_w, pinf_next_mem_w;
    wire [DW-1:0] p1_cur_mem_w, p1_next_mem_w;
    wire [DW-1:0] pw_cur_mem_w, pw_next_mem_w;
    wire [DW-1:0] pw1_cur_mem_w, pw1_next_mem_w;

    dual_port_ram #(.AWIDTH(AW_P), .DWIDTH(DW)) tc_p0_ram (
        .clka(CLK),
        .ena(child_wr_p0_w || int_req_w || (out_req_w && out_p0_valid_w)),
        .wea(child_wr_p0_w),
        .addra(child_wr_p0_w ? child_c_addr_w : (int_req_w ? interp_idx_r : out_p0_addr_w)),
        .dina(child_c_data_w), .douta(p0_cur_mem_w),
        .clkb(CLK), .enb(int_req_w && interp_next_valid_w), .web(1'b0),
        .addrb(interp_next_addr_w), .dinb({DW{1'b0}}), .doutb(p0_next_mem_w)
    );

    dual_port_ram #(.AWIDTH(AW_P), .DWIDTH(DW)) tc_pinf_ram (
        .clka(CLK),
        .ena(child_wr_pinf_w || int_req_w || (out_req_w && out_p4_valid_w)),
        .wea(child_wr_pinf_w),
        .addra(child_wr_pinf_w ? child_c_addr_w : (int_req_w ? interp_idx_r : out_p4_addr_w)),
        .dina(child_c_data_w), .douta(pinf_cur_mem_w),
        .clkb(CLK), .enb(int_req_w && interp_next_valid_w), .web(1'b0),
        .addrb(interp_next_addr_w), .dinb({DW{1'b0}}), .doutb(pinf_next_mem_w)
    );

    dual_port_ram #(.AWIDTH(AW_P), .DWIDTH(DW)) tc_p1_ram (
        .clka(CLK), .ena(child_wr_p1_w || int_req_w), .wea(child_wr_p1_w),
        .addra(child_wr_p1_w ? child_c_addr_w : interp_idx_r),
        .dina(child_c_data_w), .douta(p1_cur_mem_w),
        .clkb(CLK), .enb(int_req_w && interp_next_valid_w), .web(1'b0),
        .addrb(interp_next_addr_w), .dinb({DW{1'b0}}), .doutb(p1_next_mem_w)
    );

    dual_port_ram #(.AWIDTH(AW_P), .DWIDTH(DW)) tc_pw_ram (
        .clka(CLK), .ena(child_wr_pw_w || int_req_w), .wea(child_wr_pw_w),
        .addra(child_wr_pw_w ? child_c_addr_w : interp_idx_r),
        .dina(child_c_data_w), .douta(pw_cur_mem_w),
        .clkb(CLK), .enb(int_req_w && interp_next_valid_w), .web(1'b0),
        .addrb(interp_next_addr_w), .dinb({DW{1'b0}}), .doutb(pw_next_mem_w)
    );

    dual_port_ram #(.AWIDTH(AW_P), .DWIDTH(DW)) tc_pw1_ram (
        .clka(CLK), .ena(child_wr_pw1_w || int_req_w), .wea(child_wr_pw1_w),
        .addra(child_wr_pw1_w ? child_c_addr_w : interp_idx_r),
        .dina(child_c_data_w), .douta(pw1_cur_mem_w),
        .clkb(CLK), .enb(int_req_w && interp_next_valid_w), .web(1'b0),
        .addrb(interp_next_addr_w), .dinb({DW{1'b0}}), .doutb(pw1_next_mem_w)
    );

    // Gate the look-ahead word at the final interpolation index.
    wire [DW-1:0] p0_next_w  = interp_next_valid_w ? p0_next_mem_w  : {DW{1'b0}};
    wire [DW-1:0] p1_next_w  = interp_next_valid_w ? p1_next_mem_w  : {DW{1'b0}};
    wire [DW-1:0] pw_next_w  = interp_next_valid_w ? pw_next_mem_w  : {DW{1'b0}};
    wire [DW-1:0] pw1_next_w = interp_next_valid_w ? pw1_next_mem_w : {DW{1'b0}};

    // ------------------------------------------------------------------
    // Streaming interpolation.
    //
    // t3  = PW1 + PW
    // u   = (PW + P0)/W + t3
    // C2d = [u + (1+W^3)P4] / (1+W)
    // C3  = [t3 + (P1+P0)] / [W(1+W)]
    // C1  = (P1+P0) + C2d + P4
    // C2  = C2d + C3
    // ------------------------------------------------------------------
    wire [DW-1:0] t3_cur_w = pw1_cur_mem_w ^ pw_cur_mem_w;
    wire [DW-1:0] v_cur_w  = p1_cur_mem_w ^ p0_cur_mem_w;
    wire [DW-1:0] u_cur_w  = (pw_next_w ^ p0_next_w) ^ t3_cur_w;
    wire [DW-1:0] c2_pre_cur_w = u_cur_w ^ pinf_cur_mem_w ^ inf_d3_r;
    wire [DW-1:0] c2_div_cur_w = c2_pre_cur_w ^ c2_prev_r;

    wire [DW-1:0] t3b_next_w = (pw1_next_w ^ pw_next_w) ^ (p1_next_w ^ p0_next_w);
    wire [DW-1:0] c3_cur_w = t3b_next_w ^ c3_prev_r;
    wire [DW-1:0] c1_cur_w = v_cur_w ^ c2_div_cur_w ^ pinf_cur_mem_w;
    wire [DW-1:0] c2_cur_w = c2_div_cur_w ^ c3_cur_w;

    // Interpolated coefficient memories C1/C2/C3.
    wire out_c1_valid_w = (out_idx_r >= M) && ((out_idx_r - M) < PWORDS);
    wire out_c2_valid_w = (out_idx_r >= (2*M)) && ((out_idx_r - 2*M) < PWORDS);
    wire out_c3_valid_w = (out_idx_r >= (3*M)) && ((out_idx_r - 3*M) < PWORDS);
    wire [AW_P-1:0] out_c1_addr_w = out_idx_r - M;
    wire [AW_P-1:0] out_c2_addr_w = out_idx_r - 2*M;
    wire [AW_P-1:0] out_c3_addr_w = out_idx_r - 3*M;

    wire [DW-1:0] c1_out_mem_w, c2_out_mem_w, c3_out_mem_w;

    dual_port_ram #(.AWIDTH(AW_P), .DWIDTH(DW)) tc_c1_ram (
        .clka(CLK), .ena(state_r == T_INT_WRITE), .wea(state_r == T_INT_WRITE),
        .addra(interp_idx_r), .dina(c1_cur_w), .douta(),
        .clkb(CLK), .enb(out_req_w && out_c1_valid_w), .web(1'b0),
        .addrb(out_c1_addr_w), .dinb({DW{1'b0}}), .doutb(c1_out_mem_w)
    );

    dual_port_ram #(.AWIDTH(AW_P), .DWIDTH(DW)) tc_c2_ram (
        .clka(CLK), .ena(state_r == T_INT_WRITE), .wea(state_r == T_INT_WRITE),
        .addra(interp_idx_r), .dina(c2_cur_w), .douta(),
        .clkb(CLK), .enb(out_req_w && out_c2_valid_w), .web(1'b0),
        .addrb(out_c2_addr_w), .dinb({DW{1'b0}}), .doutb(c2_out_mem_w)
    );

    dual_port_ram #(.AWIDTH(AW_P), .DWIDTH(DW)) tc_c3_ram (
        .clka(CLK), .ena(state_r == T_INT_WRITE), .wea(state_r == T_INT_WRITE),
        .addra(interp_idx_r), .dina(c3_cur_w), .douta(),
        .clkb(CLK), .enb(out_req_w && out_c3_valid_w), .web(1'b0),
        .addrb(out_c3_addr_w), .dinb({DW{1'b0}}), .doutb(c3_out_mem_w)
    );

    // Final recomposition: C = C0 + C1*X + C2*X^2 + C3*X^3 + C4*X^4,
    // with X = x^(64*M), i.e. M-word shifts.
    wire [DW-1:0] out_word_w =
        (out_p0_valid_w ? p0_cur_mem_w : {DW{1'b0}}) ^
        (out_c1_valid_w ? c1_out_mem_w : {DW{1'b0}}) ^
        (out_c2_valid_w ? c2_out_mem_w : {DW{1'b0}}) ^
        (out_c3_valid_w ? c3_out_mem_w : {DW{1'b0}}) ^
        (out_p4_valid_w ? pinf_cur_mem_w : {DW{1'b0}});

    // ------------------------------------------------------------------
    // Controller.
    // ------------------------------------------------------------------
    always @(posedge CLK or negedge RST) begin
        if (!RST) begin
            state_r       <= T_IDLE;
            load_idx_r    <= {AW_IN{1'b0}};
            prep_idx_r    <= {AW_TC{1'b0}};
            point_r       <= PT_0;
            interp_idx_r  <= {AW_P{1'b0}};
            out_idx_r     <= {AW_OUT{1'b0}};
            child_start_r <= 1'b0;
            c2_prev_r     <= {DW{1'b0}};
            c3_prev_r     <= {DW{1'b0}};
            inf_d1_r      <= {DW{1'b0}};
            inf_d2_r      <= {DW{1'b0}};
            inf_d3_r      <= {DW{1'b0}};
        end else begin
            child_start_r <= 1'b0;

            case (state_r)
                T_IDLE: begin
                    if (start_i) begin
                        load_idx_r <= {AW_IN{1'b0}};
                        point_r    <= PT_0;
                        state_r    <= T_LOAD_REQ;
                    end
                end

                T_LOAD_REQ: state_r <= T_LOAD_W1;
                T_LOAD_W1:  state_r <= T_LOAD_W2;
                T_LOAD_W2:  state_r <= T_LOAD_CAP;

                T_LOAD_CAP: begin
                    if (load_idx_r == (SIZE-1)) begin
                        prep_idx_r <= {AW_TC{1'b0}};
                        point_r    <= PT_0;
                        state_r    <= T_PREP_REQ;
                    end else begin
                        load_idx_r <= load_idx_r + 1'b1;
                        state_r    <= T_LOAD_REQ;
                    end
                end

                T_PREP_REQ:  state_r <= T_PREP_WAIT;
                T_PREP_WAIT: state_r <= T_PREP_WRITE;

                T_PREP_WRITE: begin
                    if (prep_idx_r == (TC_SIZE-1)) begin
                        child_start_r <= 1'b1;
                        state_r       <= T_RUN;
                    end else begin
                        prep_idx_r <= prep_idx_r + 1'b1;
                        state_r    <= T_PREP_REQ;
                    end
                end

                T_RUN: begin
                    if (child_done_w) begin
                        if (point_r == PT_W1) begin
                            interp_idx_r <= {AW_P{1'b0}};
                            c2_prev_r    <= {DW{1'b0}};
                            c3_prev_r    <= {DW{1'b0}};
                            inf_d1_r     <= {DW{1'b0}};
                            inf_d2_r     <= {DW{1'b0}};
                            inf_d3_r     <= {DW{1'b0}};
                            state_r      <= T_INT_REQ;
                        end else begin
                            point_r    <= point_r + 1'b1;
                            prep_idx_r <= {AW_TC{1'b0}};
                            state_r    <= T_PREP_REQ;
                        end
                    end
                end

                T_INT_REQ:  state_r <= T_INT_WAIT;
                T_INT_WAIT: state_r <= T_INT_WRITE;

                T_INT_WRITE: begin
                    c2_prev_r <= c2_div_cur_w;
                    c3_prev_r <= c3_cur_w;
                    inf_d3_r  <= inf_d2_r;
                    inf_d2_r  <= inf_d1_r;
                    inf_d1_r  <= pinf_cur_mem_w;

                    if (interp_idx_r == (PWORDS-1)) begin
                        out_idx_r <= {AW_OUT{1'b0}};
                        state_r   <= T_OUT_REQ;
                    end else begin
                        interp_idx_r <= interp_idx_r + 1'b1;
                        state_r      <= T_INT_REQ;
                    end
                end

                T_OUT_REQ:  state_r <= T_OUT_WAIT;
                T_OUT_WAIT: state_r <= T_OUT_WRITE;

                T_OUT_WRITE: begin
                    if (out_idx_r == (2*SIZE-1)) begin
                        state_r <= T_DONE;
                    end else begin
                        out_idx_r <= out_idx_r + 1'b1;
                        state_r   <= T_OUT_REQ;
                    end
                end

                T_DONE: state_r <= T_IDLE;
                default: state_r <= T_IDLE;
            endcase
        end
    end

    // Parent input request. A and B are fetched together, matching the optimized
    // 4Flat load convention.
    assign a_rd_req_o  = (state_r == T_LOAD_REQ);
    assign a_rd_addr_o = load_idx_r;
    assign b_rd_req_o  = (state_r == T_LOAD_REQ);
    assign b_rd_addr_o = load_idx_r;

    // Direct streaming write during final recomposition.
    assign c_wr_en_o   = (state_r == T_OUT_WRITE);
    assign c_wr_addr_o = out_idx_r;
    assign c_wr_data_o = out_word_w;

    assign done_o = (state_r == T_DONE);
    assign debug_state_o = state_r;

endmodule
