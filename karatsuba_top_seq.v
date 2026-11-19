`timescale 1ns/1ps
`include "common.vh"

/*
 * Streaming-recursive Karatsuba with flat base cases for SIZE=1,2,3,4.
 *
 * Base case coverage by HQC profile:
 *   HQC-128 (SIZE=277): tree path 277→139→70→35→18→9→5→3→2  → needs SIZE=3
 *   HQC-192 (SIZE=561): tree path 561→281→141→71→36→18→9→5→3→2 → needs SIZE=3
 *   HQC-256 (SIZE=901): tree path 901→451→226→113→57→29→15→8→4→2 → needs SIZE=4
 *
 * Flat base case cycle counts (all use clmul64 function, no BRAM intermediates):
 *   SIZE=1: 11 cycles  (1 clmul, sequential read + compute + write)
 *   SIZE=2: 11 cycles  (3 clmul parallel, 2 reads × 3 cyc + 1 compute + 4 writes)
 *   SIZE=3: 17 cycles  (6 clmul parallel, 1+3+3+3 read cyc + 1 comp_reg + 6 writes)
 *   SIZE=4: 22 cycles  (9 clmul parallel, 1+3+3+3+3 read cyc + 1 comp_reg + 8 writes)
 *
 * GEN_RECURSIVE optimizations applied:
 *   OPT-A: S_LOAD_A drives both a_rd and b_rd ports simultaneously; S_LOAD_B eliminated.
 *          Saves SIZE cycles per recursive invocation (all HQC profiles benefit).
 *   OPT-B: MERGE_WRITE outputs directly to parent c_wr port via combinational mux.
 *          Eliminates u_c BRAM read-back and STREAM_REQ/SEND/FLUSH states.
 *
 * Karatsuba math summary:
 *
 * SIZE=3 (split L=2, H=1):   A=[a2,a1,a0], B=[b2,b1,b0]
 *   K1 = [a1,a0]*[b1,b0]              (4 words, SIZE=2 Karatsuba: 3 muls)
 *   K2 = a2*b2                        (2 words, 1 mul)
 *   K3 = [a1^a2, a0]*[b1^b2, b0]     (4 words, SIZE=2 Karatsuba: 3 muls; p1=a1*b1 shared w/ K1)
 *   Total: 6 clmul64 (p0,p1,p2,q0,r1,r2; p1 shared → saves 1 vs naive 7), output 6 words
 *   C = K1 + x^64*mid + x^128*K2,  mid = K1 ^ {0,0,K2[1],K2[0]} ^ K3
 *   c[0] = K1[0]
 *   c[1] = K1[1] ^ mid[0]
 *   c[2] = K1[2] ^ mid[1] ^ K2[0]
 *   c[3] = K1[3] ^ mid[2] ^ K2[1]
 *   c[4] = mid[3]
 *   c[5] = 0
 *
 * SIZE=4 (split L=H=2):   A=[a3,a2,a1,a0], B=[b3,b2,b1,b0]
 *   K1 = [a1,a0]*[b1,b0]              (4 words, 3 muls)
 *   K2 = [a3,a2]*[b3,b2]             (4 words, 3 muls)
 *   K3 = [a3^a1,a2^a0]*[b3^b1,b2^b0] (4 words, 3 muls)
 *   Total: 9 clmul64, output 8 words
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
 * Read latency note (applies to all flat base cases):
 *   Parent BRAM has 2-cycle read latency: req asserted → data valid 2 cycles later.
 *   (Matches GEN_BASE comment: "In recursive mode, extra cycle due to registered read-enable.")
 *   FSM read pattern per word: REQ (assert req) → WAIT_A → WAIT_B → CAP (data valid).
 *   Words are read sequentially; compute happens after last word is captured.
 */
module karatsuba_top_seq #(
    parameter SIZE = `VEC_N_SIZE_64
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

    localparam DW      = 64;
    localparam AW_IN   = `SAFE_CLOG2(SIZE);
    localparam AW_OUT  = `SAFE_CLOG2(2*SIZE);

    localparam SIZE_L  = (SIZE + 1) / 2;
    localparam SIZE_H  = SIZE / 2;
    localparam AW_L    = `SAFE_CLOG2(SIZE_L);
    localparam AW_CL   = `SAFE_CLOG2(2*SIZE_L);

    localparam S_IDLE          = 4'd0;
    localparam S_LOAD_A        = 4'd1;
    localparam S_LOAD_B        = 4'd2;
    localparam S_PREP_K1       = 4'd3;
    localparam S_RUN_K1        = 4'd4;
    localparam S_PREP_K2       = 4'd5;
    localparam S_RUN_K2        = 4'd6;
    localparam S_PREP_K3       = 4'd7;
    localparam S_RUN_K3        = 4'd8;
    localparam S_MERGE_WRITE   = 4'd9;
    localparam S_STREAM_REQ    = 4'd10;
    localparam S_STREAM_SEND   = 4'd11;
    localparam S_DONE          = 4'd12;
    localparam S_STREAM_FLUSH  = 4'd13;


    generate
        // =====================================================================
        // GEN_BASE: SIZE==1  (1 × 64-bit multiply, uses base_mul module)
        // =====================================================================
        if (SIZE == 1) begin : GEN_BASE
            reg pending_start_r;
            reg [1:0] rd_wait_r;
            reg busy_r;
            reg result_ready_r;
            reg base_start_r;
            reg done_pulse_r;
            reg [DW-1:0] a_val_r, b_val_r;
            reg [2*DW-1:0] c_result_r;
            reg [1:0] wr_idx_r;

            wire base_done_w;
            wire [2*DW-1:0] base_c_w;

            base_mul u_base (
                .CLK(CLK), .RST(RST), .start_i(base_start_r),
                .a_i(a_val_r), .b_i(b_val_r),
                .done_o(base_done_w), .c_o(base_c_w)
            );

            always @(posedge CLK or negedge RST) begin
                if (!RST) begin
                    pending_start_r <= 1'b0;
                    rd_wait_r <= 2'd0;
                    busy_r <= 1'b0;
                    result_ready_r <= 1'b0;
                    base_start_r <= 1'b0;
                    done_pulse_r <= 1'b0;
                    a_val_r <= {DW{1'b0}};
                    b_val_r <= {DW{1'b0}};
                    c_result_r <= {2*DW{1'b0}};
                    wr_idx_r <= 2'd0;
                end else begin
                    base_start_r <= 1'b0;
                    done_pulse_r <= 1'b0;

                    if (start_i && !busy_r) begin
                        pending_start_r <= 1'b1;
                        rd_wait_r <= 2'd0;
                        busy_r <= 1'b1;
                        result_ready_r <= 1'b0;
                    end

                    if (pending_start_r && (rd_wait_r == 2'd0)) begin
                        rd_wait_r <= 2'd1;
                    end else if (pending_start_r && (rd_wait_r == 2'd1)) begin
                        rd_wait_r <= 2'd2;
                    end else if (pending_start_r && (rd_wait_r == 2'd2)) begin
                        a_val_r <= a_rd_data_i;
                        b_val_r <= b_rd_data_i;
                        pending_start_r <= 1'b0;
                        rd_wait_r <= 2'd0;
                        base_start_r <= 1'b1;
                    end

                    if (base_done_w) begin
                        c_result_r <= base_c_w;
                        wr_idx_r <= 2'd0;
                        result_ready_r <= 1'b1;
                    end

                    if (c_wr_en_o)
                        wr_idx_r <= wr_idx_r + 1'b1;

                    if (wr_idx_r == 2'd1 && c_wr_en_o)
                        done_pulse_r <= 1'b1;
                    else if (done_pulse_r) begin
                        result_ready_r <= 1'b0;
                        busy_r <= 1'b0;
                    end
                end
            end

            assign a_rd_req_o  = pending_start_r && (rd_wait_r == 2'd0);
            assign a_rd_addr_o = 1'b0;
            assign b_rd_req_o  = pending_start_r && (rd_wait_r == 2'd0);
            assign b_rd_addr_o = 1'b0;
            assign c_wr_en_o   = (busy_r && result_ready_r && (wr_idx_r < 2'd2));
            assign c_wr_addr_o = wr_idx_r;
            assign c_wr_data_o = (wr_idx_r == 2'd0) ? c_result_r[DW-1:0] : c_result_r[2*DW-1:DW];
            assign done_o      = done_pulse_r;
            assign debug_state_o = {2'b00, wr_idx_r};

        // =====================================================================
        // GEN_BASE_2: SIZE==2  (3 clmul64, 11 cycles)  → see base_mul_2.v
        // =====================================================================
        end else if (SIZE == 2) begin : GEN_BASE_2
            base_mul_2 #(.SIZE(SIZE)) u_base (
                .CLK(CLK), .RST(RST), .start_i(start_i),
                .a_rd_req_o(a_rd_req_o), .a_rd_addr_o(a_rd_addr_o), .a_rd_data_i(a_rd_data_i),
                .b_rd_req_o(b_rd_req_o), .b_rd_addr_o(b_rd_addr_o), .b_rd_data_i(b_rd_data_i),
                .c_wr_en_o(c_wr_en_o), .c_wr_addr_o(c_wr_addr_o), .c_wr_data_o(c_wr_data_o),
                .done_o(done_o), .debug_state_o(debug_state_o)
            );

        // =====================================================================
        // GEN_BASE_3: SIZE==3  (6 clmul64, 17 cycles)  → see base_mul_3.v
        // =====================================================================
        end else if (SIZE == 3) begin : GEN_BASE_3
            base_mul_3 #(.SIZE(SIZE)) u_base (
                .CLK(CLK), .RST(RST), .start_i(start_i),
                .a_rd_req_o(a_rd_req_o), .a_rd_addr_o(a_rd_addr_o), .a_rd_data_i(a_rd_data_i),
                .b_rd_req_o(b_rd_req_o), .b_rd_addr_o(b_rd_addr_o), .b_rd_data_i(b_rd_data_i),
                .c_wr_en_o(c_wr_en_o), .c_wr_addr_o(c_wr_addr_o), .c_wr_data_o(c_wr_data_o),
                .done_o(done_o), .debug_state_o(debug_state_o)
            );

        // =====================================================================
        // GEN_BASE_4: SIZE==4  (9 clmul64, 22 cycles)  → see base_mul_4.v
        // =====================================================================
        end else if (SIZE == 4) begin : GEN_BASE_4
            base_mul_4 #(.SIZE(SIZE)) u_base (
                .CLK(CLK), .RST(RST), .start_i(start_i),
                .a_rd_req_o(a_rd_req_o), .a_rd_addr_o(a_rd_addr_o), .a_rd_data_i(a_rd_data_i),
                .b_rd_req_o(b_rd_req_o), .b_rd_addr_o(b_rd_addr_o), .b_rd_data_i(b_rd_data_i),
                .c_wr_en_o(c_wr_en_o), .c_wr_addr_o(c_wr_addr_o), .c_wr_data_o(c_wr_data_o),
                .done_o(done_o), .debug_state_o(debug_state_o)
            );

        // =====================================================================
        // GEN_RECURSIVE: SIZE>=5  (standard recursive Karatsuba with BRAM intermediates)
        // =====================================================================
        end else begin : GEN_RECURSIVE
            reg [3:0] state_r, state_prev_r;
            reg [3:0] next_state_r;
            reg [AW_IN-1:0] load_idx_r;
            reg [1:0] load_pending_r;
            reg [AW_OUT-1:0] stream_idx_r;
            reg stream_valid_r;
            reg done_r;

            reg ctrl_done_w;
            reg ctrl_a_we_w;
            reg ctrl_b_we_w;
            reg ctrl_prep_rd_en_w;
            reg ctrl_c_we_a_w;
            reg ctrl_c_en_b_w;
            reg ctrl_child_a_we_w;
            reg ctrl_child_b_we_w;
            reg ctrl_child_a_rd_en_w;
            reg ctrl_child_b_rd_en_w;
            reg ctrl_o_we_w;
            reg ctrl_o_rd_en_w;
            reg ctrl_tmp1_we_w;
            reg ctrl_tmp1_rd_en_w;
            reg ctrl_tmp2_we_w;
            reg ctrl_tmp2_rd_en_w;

            reg fsm_a_rd_req_r;
            reg [AW_IN-1:0] fsm_a_rd_addr_r;
            reg fsm_b_rd_req_r;
            reg [AW_IN-1:0] fsm_b_rd_addr_r;
            reg fsm_c_wr_en_r;
            reg [AW_OUT-1:0] fsm_c_wr_addr_r;
            reg [DW-1:0] fsm_c_wr_data_r;
            reg fsm_done_r;

            reg a_we_r, b_we_r;
            reg [AW_IN-1:0] a_wr_addr_r, b_wr_addr_r;
            reg [DW-1:0] a_wr_data_r, b_wr_data_r;

            reg prep_rd_en_r;
            reg [AW_IN-1:0] prep_rd_addr_r;
            wire [DW-1:0] a_ram_dout_b_w;
            wire [DW-1:0] b_ram_dout_b_w;

            dual_port_ram #(.AWIDTH(AW_IN), .DWIDTH(DW)) u_a (
                .clka(CLK), .ena(1'b1), .wea(a_we_r), .addra(a_wr_addr_r), .dina(a_wr_data_r), .douta(),
                .clkb(CLK), .enb(prep_rd_en_r), .web(1'b0), .addrb(prep_rd_addr_r), .dinb({DW{1'b0}}), .doutb(a_ram_dout_b_w)
            );

            dual_port_ram #(.AWIDTH(AW_IN), .DWIDTH(DW)) u_b (
                .clka(CLK), .ena(1'b1), .wea(b_we_r), .addra(b_wr_addr_r), .dina(b_wr_data_r), .douta(),
                .clkb(CLK), .enb(prep_rd_en_r), .web(1'b0), .addrb(prep_rd_addr_r), .dinb({DW{1'b0}}), .doutb(b_ram_dout_b_w)
            );

            reg c_we_a_r;
            reg [AW_OUT-1:0] c_addr_a_r;
            reg [DW-1:0] c_din_a_r;
            reg c_en_b_r;
            reg [AW_OUT-1:0] c_addr_b_r;
            wire [DW-1:0] c_dout_b_w;

            dual_port_ram #(.AWIDTH(AW_OUT), .DWIDTH(DW)) u_c (
                .clka(CLK), .ena(1'b1), .wea(c_we_a_r), .addra(c_addr_a_r), .dina(c_din_a_r), .douta(),
                .clkb(CLK), .enb(c_en_b_r), .web(1'b0), .addrb(c_addr_b_r), .dinb({DW{1'b0}}), .doutb(c_dout_b_w)
            );

            reg child_a_we_r, child_b_we_r;
            reg [AW_L-1:0] child_a_wr_addr_r, child_b_wr_addr_r;
            reg [DW-1:0] child_a_wr_data_r, child_b_wr_data_r;
            reg child_a_rd_en_r, child_b_rd_en_r;
            reg [AW_L-1:0] child_a_rd_addr_r, child_b_rd_addr_r;
            wire [DW-1:0] child_a_dout_w, child_b_dout_w;

            dual_port_ram #(.AWIDTH(AW_L), .DWIDTH(DW)) u_child_a (
                .clka(CLK), .ena(1'b1), .wea(child_a_we_r), .addra(child_a_wr_addr_r), .dina(child_a_wr_data_r), .douta(),
                .clkb(CLK), .enb(child_a_rd_en_r), .web(1'b0), .addrb(child_a_rd_addr_r), .dinb({DW{1'b0}}), .doutb(child_a_dout_w)
            );

            dual_port_ram #(.AWIDTH(AW_L), .DWIDTH(DW)) u_child_b (
                .clka(CLK), .ena(1'b1), .wea(child_b_we_r), .addra(child_b_wr_addr_r), .dina(child_b_wr_data_r), .douta(),
                .clkb(CLK), .enb(child_b_rd_en_r), .web(1'b0), .addrb(child_b_rd_addr_r), .dinb({DW{1'b0}}), .doutb(child_b_dout_w)
            );

            reg o_we_r;
            reg [AW_CL-1:0] o_wr_addr_r;
            reg [DW-1:0] o_wr_data_r;
            reg o_rd_en_r;
            reg [AW_CL-1:0] o_rd_addr_r;
            wire [DW-1:0] o_dout_w;

            dual_port_ram #(.AWIDTH(AW_CL), .DWIDTH(DW)) u_o (
                .clka(CLK), .ena(1'b1), .wea(o_we_r), .addra(o_wr_addr_r), .dina(o_wr_data_r), .douta(),
                .clkb(CLK), .enb(o_rd_en_r), .web(1'b0), .addrb(o_rd_addr_r), .dinb({DW{1'b0}}), .doutb(o_dout_w)
            );

            reg tmp1_we_r;
            reg [AW_CL-1:0] tmp1_wr_addr_r;
            reg [DW-1:0] tmp1_wr_data_r;
            reg tmp1_rd_en_r;
            reg [AW_CL-1:0] tmp1_rd_addr_r;
            wire [DW-1:0] tmp1_dout_w;

            dual_port_ram #(.AWIDTH(AW_CL), .DWIDTH(DW)) u_tmp1 (
                .clka(CLK), .ena(1'b1), .wea(tmp1_we_r), .addra(tmp1_wr_addr_r), .dina(tmp1_wr_data_r), .douta(),
                .clkb(CLK), .enb(tmp1_rd_en_r), .web(1'b0), .addrb(tmp1_rd_addr_r), .dinb({DW{1'b0}}), .doutb(tmp1_dout_w)
            );

            localparam AW_CL_H = `SAFE_CLOG2(2*SIZE_H);
            reg tmp2_we_r;
            reg [AW_CL_H-1:0] tmp2_wr_addr_r;
            reg [DW-1:0] tmp2_wr_data_r;
            reg tmp2_rd_en_r;
            reg [AW_CL_H-1:0] tmp2_rd_addr_r;
            wire [DW-1:0] tmp2_dout_w;

            dual_port_ram #(.AWIDTH(AW_CL_H), .DWIDTH(DW)) u_tmp2 (
                .clka(CLK), .ena(1'b1), .wea(tmp2_we_r), .addra(tmp2_wr_addr_r), .dina(tmp2_wr_data_r), .douta(),
                .clkb(CLK), .enb(tmp2_rd_en_r), .web(1'b0), .addrb(tmp2_rd_addr_r), .dinb({DW{1'b0}}), .doutb(tmp2_dout_w)
            );

            reg mid_we_r;
            reg [AW_CL-1:0] mid_wr_addr_r;
            reg [DW-1:0] mid_wr_data_r;
            reg mid_rd_en_r;
            reg [AW_CL-1:0] mid_rd_addr_r;
            wire [DW-1:0] mid_dout_w;

            dual_port_ram #(.AWIDTH(AW_CL), .DWIDTH(DW)) u_mid (
                .clka(CLK), .ena(1'b1), .wea(mid_we_r), .addra(mid_wr_addr_r), .dina(mid_wr_data_r), .douta(),
                .clkb(CLK), .enb(mid_rd_en_r), .web(1'b0), .addrb(mid_rd_addr_r), .dinb({DW{1'b0}}), .doutb(mid_dout_w)
            );

            reg [DW-1:0] o_rd_data_r, tmp1_rd_data_r, tmp2_rd_data_r;

            wire child_a_req_w;
            wire [AW_L-1:0] child_a_addr_w;
            wire child_b_req_w;
            wire [AW_L-1:0] child_b_addr_w;
            wire [DW-1:0] child_a_data_w;
            wire [DW-1:0] child_b_data_w;
            wire child_c_wr_en_w;
            wire [AW_CL-1:0] child_c_wr_addr_w;
            wire [DW-1:0] child_c_wr_data_w;
            wire child_done_w;

            wire run_k1_enter_w, run_k2_enter_w, run_k3_enter_w;
            wire load_a_enter_w, load_b_enter_w;
            wire prep_k1_enter_w, prep_k2_enter_w, prep_k3_enter_w;
            wire merge_enter_w, child_start_w;

            assign load_a_enter_w  = (state_r == S_LOAD_A)    && (state_prev_r != S_LOAD_A);
            assign load_b_enter_w  = (state_r == S_LOAD_B)    && (state_prev_r != S_LOAD_B);
            assign prep_k1_enter_w = (state_r == S_PREP_K1)   && (state_prev_r != S_PREP_K1);
            assign prep_k2_enter_w = (state_r == S_PREP_K2)   && (state_prev_r != S_PREP_K2);
            assign prep_k3_enter_w = (state_r == S_PREP_K3)   && (state_prev_r != S_PREP_K3);
            assign merge_enter_w   = (state_r == S_MERGE_WRITE) && (state_prev_r != S_MERGE_WRITE);
            assign run_k1_enter_w  = (state_r == S_RUN_K1)    && (state_prev_r != S_RUN_K1);
            assign run_k2_enter_w  = (state_r == S_RUN_K2)    && (state_prev_r != S_RUN_K2);
            assign run_k3_enter_w  = (state_r == S_RUN_K3)    && (state_prev_r != S_RUN_K3);
            assign child_start_w   = run_k1_enter_w || run_k2_enter_w || run_k3_enter_w;

            assign child_a_data_w = child_a_dout_w;
            assign child_b_data_w = child_b_dout_w;

            karatsuba_top_seq #(.SIZE(SIZE_L)) child_inst (
                .CLK(CLK), .RST(RST), .start_i(child_start_w),
                .a_rd_req_o(child_a_req_w), .a_rd_addr_o(child_a_addr_w), .a_rd_data_i(child_a_data_w),
                .b_rd_req_o(child_b_req_w), .b_rd_addr_o(child_b_addr_w), .b_rd_data_i(child_b_data_w),
                .c_wr_en_o(child_c_wr_en_w), .c_wr_addr_o(child_c_wr_addr_w), .c_wr_data_o(child_c_wr_data_w),
                .done_o(child_done_w)
            );

            reg [AW_IN:0] prep_idx_r;
            reg [AW_IN:0] prep_cap_idx_r;
            reg [1:0] prep_pending_r;
            reg [2:0] k3_step_r;
            reg [DW-1:0] k3_low_a_r, k3_low_b_r;

            reg [AW_OUT-1:0] merge_rd_idx_r;
            reg [1:0] merge_rd_pend_r;
            reg [DW-1:0] o_rd_latch_r, tmp1_rd_latch_r, tmp2_rd_latch_r;

            wire [AW_IN:0] k2_addr_base_w = SIZE_L + prep_idx_r;
            wire [AW_IN:0] k3_addr_base_w = SIZE_L + prep_cap_idx_r;
            wire [AW_OUT:0] merge_idx_minus_l_w    = merge_rd_idx_r - SIZE_L;
            wire [AW_OUT:0] merge_idx_minus_2l_w   = merge_rd_idx_r - 2*SIZE_L;
            wire [AW_OUT:0] merge_mid_hi_idx_w     = SIZE_L + merge_idx_minus_2l_w;
            wire [AW_OUT-1:0] merge_idx_next_w     = merge_rd_idx_r + {{(AW_OUT-1){1'b0}}, 1'b1};
            wire [AW_OUT:0] merge_idx_next_minus_2l_w = merge_idx_next_w - 2*SIZE_L;

            wire load_a_done_w, load_b_done_w;
            wire prep_k1_done_w, prep_k2_done_w, prep_k3_done_w;
            wire run_k1_done_w, run_k2_done_w, run_k3_done_w;
            wire merge_done_w, stream_last_w;

            assign load_a_done_w  = !load_a_enter_w  && (load_pending_r == 2'd2) && (load_idx_r == SIZE-1);
            assign load_b_done_w  = !load_b_enter_w  && (load_pending_r == 2'd2) && (load_idx_r == SIZE-1);
            assign prep_k1_done_w = !prep_k1_enter_w && !(prep_cap_idx_r < SIZE_L);
            assign prep_k2_done_w = !prep_k2_enter_w && !(prep_cap_idx_r < SIZE_L);
            assign prep_k3_done_w = !prep_k3_enter_w && !(prep_cap_idx_r < SIZE_L);
            assign run_k1_done_w  = (state_r == S_RUN_K1) && child_done_w;
            assign run_k2_done_w  = (state_r == S_RUN_K2) && child_done_w;
            assign run_k3_done_w  = (state_r == S_RUN_K3) && child_done_w;
            assign merge_done_w   = (merge_rd_pend_r == 2'd3) && (merge_rd_idx_r == 2*SIZE - 1);
            assign stream_last_w  = (stream_idx_r == 2*SIZE-1);

            // OPT-B: Combinational output data for direct parent write (bypasses u_c BRAM)
            reg [DW-1:0] merge_out_data_w;
            always @(*) begin
                if (merge_rd_idx_r < 2*SIZE_L) begin
                    if (merge_rd_idx_r < SIZE_L)
                        merge_out_data_w = o_rd_latch_r;
                    else
                        merge_out_data_w = o_rd_latch_r ^ mid_dout_w;
                end else begin
                    if (merge_idx_minus_2l_w < SIZE_L)
                        merge_out_data_w = tmp2_rd_latch_r ^ mid_dout_w;
                    else
                        merge_out_data_w = tmp2_rd_latch_r;
                end
            end

            always @(state_r or start_i or load_a_done_w or load_b_done_w or
                     prep_k1_done_w or prep_k2_done_w or prep_k3_done_w or
                     run_k1_done_w or run_k2_done_w or run_k3_done_w or
                     merge_done_w or stream_last_w) begin
                next_state_r = state_r;
                case (state_r)
                    S_IDLE:        if (start_i)         next_state_r = S_LOAD_A;
                    S_LOAD_A:      if (load_a_done_w)   next_state_r = S_PREP_K1;  // OPT-A: LOAD_B merged into LOAD_A
                    S_PREP_K1:     if (prep_k1_done_w)  next_state_r = S_RUN_K1;
                    S_RUN_K1:      if (run_k1_done_w)   next_state_r = S_PREP_K2;
                    S_PREP_K2:     if (prep_k2_done_w)  next_state_r = S_RUN_K2;
                    S_RUN_K2:      if (run_k2_done_w)   next_state_r = S_PREP_K3;
                    S_PREP_K3:     if (prep_k3_done_w)  next_state_r = S_RUN_K3;
                    S_RUN_K3:      if (run_k3_done_w)   next_state_r = S_MERGE_WRITE;
                    S_MERGE_WRITE: if (merge_done_w)    next_state_r = S_DONE;  // OPT-B: skip STREAM
                    S_STREAM_REQ:                       next_state_r = S_STREAM_SEND;
                    S_STREAM_SEND: next_state_r = stream_last_w ? S_STREAM_FLUSH : S_STREAM_REQ;
                    S_STREAM_FLUSH:                     next_state_r = S_DONE;
                    S_DONE:        if (!start_i)        next_state_r = S_IDLE;
                    default:                            next_state_r = S_IDLE;
                endcase
            end

            always @(state_r or load_idx_r or stream_valid_r or stream_idx_r or c_dout_b_w or done_r or
                     merge_rd_pend_r or merge_rd_idx_r or merge_out_data_w) begin
                case (state_r)
                    S_LOAD_A: begin  // OPT-A: drive both A and B ports simultaneously
                        fsm_a_rd_req_r = 1'b1; fsm_a_rd_addr_r = load_idx_r;
                        fsm_b_rd_req_r = 1'b1; fsm_b_rd_addr_r = load_idx_r;
                        fsm_c_wr_en_r = 1'b0; fsm_c_wr_addr_r = {AW_OUT{1'b0}};
                        fsm_c_wr_data_r = {DW{1'b0}}; fsm_done_r = 1'b0;
                    end
                    S_MERGE_WRITE: begin  // OPT-B: output directly to parent, no u_c BRAM
                        fsm_a_rd_req_r = 1'b0; fsm_a_rd_addr_r = {AW_IN{1'b0}};
                        fsm_b_rd_req_r = 1'b0; fsm_b_rd_addr_r = {AW_IN{1'b0}};
                        fsm_c_wr_en_r   = (merge_rd_pend_r == 2'd3);
                        fsm_c_wr_addr_r = merge_rd_idx_r[AW_OUT-1:0];
                        fsm_c_wr_data_r = merge_out_data_w;
                        fsm_done_r = 1'b0;
                    end
                    S_STREAM_SEND: begin
                        fsm_a_rd_req_r = 1'b0; fsm_a_rd_addr_r = {AW_IN{1'b0}};
                        fsm_b_rd_req_r = 1'b0; fsm_b_rd_addr_r = {AW_IN{1'b0}};
                        fsm_c_wr_en_r = stream_valid_r; fsm_c_wr_addr_r = stream_idx_r - 1'b1;
                        fsm_c_wr_data_r = c_dout_b_w; fsm_done_r = 1'b0;
                    end
                    S_STREAM_FLUSH: begin
                        fsm_a_rd_req_r = 1'b0; fsm_a_rd_addr_r = {AW_IN{1'b0}};
                        fsm_b_rd_req_r = 1'b0; fsm_b_rd_addr_r = {AW_IN{1'b0}};
                        fsm_c_wr_en_r = 1'b1; fsm_c_wr_addr_r = stream_idx_r;
                        fsm_c_wr_data_r = c_dout_b_w; fsm_done_r = 1'b0;
                    end
                    S_DONE: begin
                        fsm_a_rd_req_r = 1'b0; fsm_a_rd_addr_r = {AW_IN{1'b0}};
                        fsm_b_rd_req_r = 1'b0; fsm_b_rd_addr_r = {AW_IN{1'b0}};
                        fsm_c_wr_en_r = 1'b0; fsm_c_wr_addr_r = {AW_OUT{1'b0}};
                        fsm_c_wr_data_r = {DW{1'b0}}; fsm_done_r = 1'b1;
                    end
                    default: begin
                        fsm_a_rd_req_r = 1'b0; fsm_a_rd_addr_r = {AW_IN{1'b0}};
                        fsm_b_rd_req_r = 1'b0; fsm_b_rd_addr_r = {AW_IN{1'b0}};
                        fsm_c_wr_en_r = 1'b0; fsm_c_wr_addr_r = {AW_OUT{1'b0}};
                        fsm_c_wr_data_r = {DW{1'b0}}; fsm_done_r = 1'b0;
                    end
                endcase
            end

            always @(state_r or child_a_req_w or child_b_req_w or child_c_wr_en_w or
                     child_c_wr_addr_w or merge_rd_pend_r or merge_rd_idx_r) begin
                ctrl_done_w = 1'b0; ctrl_a_we_w = 1'b0; ctrl_b_we_w = 1'b0;
                ctrl_prep_rd_en_w = 1'b0; ctrl_c_we_a_w = 1'b0; ctrl_c_en_b_w = 1'b0;
                ctrl_child_a_we_w = 1'b0; ctrl_child_b_we_w = 1'b0;
                ctrl_child_a_rd_en_w = 1'b0; ctrl_child_b_rd_en_w = 1'b0;
                ctrl_o_we_w = 1'b0; ctrl_o_rd_en_w = 1'b0;
                ctrl_tmp1_we_w = 1'b0; ctrl_tmp1_rd_en_w = 1'b0;
                ctrl_tmp2_we_w = 1'b0; ctrl_tmp2_rd_en_w = 1'b0;
                case (state_r)
                    S_RUN_K1: begin
                        ctrl_child_a_rd_en_w = child_a_req_w;
                        ctrl_child_b_rd_en_w = child_b_req_w;
                        ctrl_o_we_w = child_c_wr_en_w;
                    end
                    S_RUN_K2: begin
                        ctrl_child_a_rd_en_w = child_a_req_w;
                        ctrl_child_b_rd_en_w = child_b_req_w;
                        ctrl_tmp2_we_w = child_c_wr_en_w && (child_c_wr_addr_w < 2*SIZE_H);
                    end
                    S_RUN_K3: begin
                        ctrl_child_a_rd_en_w = child_a_req_w;
                        ctrl_child_b_rd_en_w = child_b_req_w;
                        ctrl_tmp1_we_w = child_c_wr_en_w;
                    end
                    S_MERGE_WRITE: begin
                        ctrl_o_rd_en_w    = ((merge_rd_pend_r == 2'd0) && (merge_rd_idx_r < 2*SIZE) && (merge_rd_idx_r < 2*SIZE_L));
                        ctrl_tmp1_rd_en_w = ((merge_rd_pend_r == 2'd0) && (merge_rd_idx_r < 2*SIZE) && (merge_rd_idx_r < 2*SIZE_L));
                        ctrl_tmp2_rd_en_w = ((merge_rd_pend_r == 2'd0) && (merge_rd_idx_r < 2*SIZE) &&
                                            ((merge_rd_idx_r < 2*SIZE_H) || (merge_rd_idx_r >= 2*SIZE_L)));
                    end
                    S_STREAM_REQ: ctrl_c_en_b_w = 1'b1;
                    S_DONE: ctrl_done_w = 1'b1;
                    default: begin end
                endcase
            end

            always @(posedge CLK or negedge RST) begin
                if (!RST) begin
                    state_r <= S_IDLE; state_prev_r <= S_IDLE;
                    load_idx_r <= {AW_IN{1'b0}}; load_pending_r <= 2'd0;
                    stream_idx_r <= {AW_OUT{1'b0}}; stream_valid_r <= 1'b0;
                    done_r <= 1'b0;
                    a_we_r <= 1'b0; b_we_r <= 1'b0;
                    a_wr_addr_r <= {AW_IN{1'b0}}; b_wr_addr_r <= {AW_IN{1'b0}};
                    a_wr_data_r <= {DW{1'b0}}; b_wr_data_r <= {DW{1'b0}};
                    prep_rd_en_r <= 1'b0; prep_rd_addr_r <= {AW_IN{1'b0}};
                    c_we_a_r <= 1'b0; c_addr_a_r <= {AW_OUT{1'b0}}; c_din_a_r <= {DW{1'b0}};
                    c_en_b_r <= 1'b0; c_addr_b_r <= {AW_OUT{1'b0}};
                    child_a_we_r <= 1'b0; child_b_we_r <= 1'b0;
                    child_a_wr_addr_r <= {AW_L{1'b0}}; child_b_wr_addr_r <= {AW_L{1'b0}};
                    child_a_wr_data_r <= {DW{1'b0}}; child_b_wr_data_r <= {DW{1'b0}};
                    child_a_rd_en_r <= 1'b0; child_b_rd_en_r <= 1'b0;
                    child_a_rd_addr_r <= {AW_L{1'b0}}; child_b_rd_addr_r <= {AW_L{1'b0}};
                    o_we_r <= 1'b0; o_wr_addr_r <= {AW_CL{1'b0}}; o_wr_data_r <= {DW{1'b0}};
                    o_rd_en_r <= 1'b0; o_rd_addr_r <= {AW_CL{1'b0}};
                    tmp1_we_r <= 1'b0; tmp1_wr_addr_r <= {AW_CL{1'b0}}; tmp1_wr_data_r <= {DW{1'b0}};
                    tmp1_rd_en_r <= 1'b0; tmp1_rd_addr_r <= {AW_CL{1'b0}};
                    tmp2_we_r <= 1'b0; tmp2_wr_addr_r <= {AW_CL_H{1'b0}}; tmp2_wr_data_r <= {DW{1'b0}};
                    tmp2_rd_en_r <= 1'b0; tmp2_rd_addr_r <= {AW_CL_H{1'b0}};
                    mid_we_r <= 1'b0; mid_wr_addr_r <= {AW_CL{1'b0}}; mid_wr_data_r <= {DW{1'b0}};
                    mid_rd_en_r <= 1'b0; mid_rd_addr_r <= {AW_CL{1'b0}};
                    prep_idx_r <= {(AW_IN+1){1'b0}}; prep_cap_idx_r <= {(AW_IN+1){1'b0}};
                    prep_pending_r <= 2'd0; k3_step_r <= 3'd0;
                    k3_low_a_r <= {DW{1'b0}}; k3_low_b_r <= {DW{1'b0}};
                    merge_rd_idx_r <= {AW_OUT{1'b0}}; merge_rd_pend_r <= 2'd0;
                    o_rd_latch_r <= {DW{1'b0}}; tmp1_rd_latch_r <= {DW{1'b0}}; tmp2_rd_latch_r <= {DW{1'b0}};
                end else begin
                    state_r <= next_state_r; state_prev_r <= state_r;
                    done_r <= ctrl_done_w;
                    a_we_r <= ctrl_a_we_w; b_we_r <= ctrl_b_we_w;
                    prep_rd_en_r <= ctrl_prep_rd_en_w;
                    c_we_a_r <= ctrl_c_we_a_w; c_en_b_r <= ctrl_c_en_b_w;
                    child_a_we_r <= ctrl_child_a_we_w; child_b_we_r <= ctrl_child_b_we_w;
                    child_a_rd_en_r <= ctrl_child_a_rd_en_w; child_b_rd_en_r <= ctrl_child_b_rd_en_w;
                    o_we_r <= ctrl_o_we_w; o_rd_en_r <= ctrl_o_rd_en_w;
                    tmp1_we_r <= ctrl_tmp1_we_w; tmp1_rd_en_r <= ctrl_tmp1_rd_en_w;
                    tmp2_we_r <= ctrl_tmp2_we_w; tmp2_rd_en_r <= ctrl_tmp2_rd_en_w;
                    mid_we_r <= 1'b0; mid_rd_en_r <= 1'b0;

                    case (state_r)
                        S_IDLE: begin
                            load_idx_r <= {AW_IN{1'b0}}; load_pending_r <= 2'd0;
                            stream_idx_r <= {AW_OUT{1'b0}}; stream_valid_r <= 1'b0;
                        end
                        S_LOAD_A: begin  // OPT-A: write u_a and u_b in the same cycle
                            if (load_a_enter_w) begin
                                load_pending_r <= 2'd0;
                            end else begin
                                if (load_pending_r == 2'd0) load_pending_r <= 2'd1;
                                else if (load_pending_r == 2'd1) load_pending_r <= 2'd2;
                                else begin
                                    a_wr_addr_r <= load_idx_r; a_wr_data_r <= a_rd_data_i; a_we_r <= 1'b1;
                                    b_wr_addr_r <= load_idx_r; b_wr_data_r <= b_rd_data_i; b_we_r <= 1'b1;
                                    load_pending_r <= 2'd0;
                                    if (load_idx_r == SIZE-1) load_idx_r <= {AW_IN{1'b0}};
                                    else load_idx_r <= load_idx_r + 1'b1;
                                end
                            end
                        end
                        S_PREP_K1: begin
                            if (prep_k1_enter_w) begin
                                prep_idx_r <= 0; prep_cap_idx_r <= 0; prep_pending_r <= 2'd0;
                            end else if (prep_cap_idx_r < SIZE_L) begin
                                case (prep_pending_r)
                                    2'd0: begin
                                        prep_rd_en_r <= 1'b1;
                                        prep_rd_addr_r <= prep_idx_r[AW_IN-1:0];
                                        prep_idx_r <= prep_idx_r + 1'b1;
                                        prep_pending_r <= 2'd1;
                                    end
                                    2'd1: prep_pending_r <= 2'd2;
                                    default: begin
                                        child_a_we_r <= 1'b1; child_a_wr_addr_r <= prep_cap_idx_r[AW_L-1:0];
                                        child_a_wr_data_r <= a_ram_dout_b_w;
                                        child_b_we_r <= 1'b1; child_b_wr_addr_r <= prep_cap_idx_r[AW_L-1:0];
                                        child_b_wr_data_r <= b_ram_dout_b_w;
                                        prep_cap_idx_r <= prep_cap_idx_r + 1'b1;
                                        if ((prep_cap_idx_r + 1'b1) < SIZE_L && (prep_idx_r < SIZE_L)) begin
                                            prep_rd_en_r <= 1'b1;
                                            prep_rd_addr_r <= prep_idx_r[AW_IN-1:0];
                                            prep_idx_r <= prep_idx_r + 1'b1;
                                            prep_pending_r <= 2'd1;
                                        end else prep_pending_r <= 2'd0;
                                    end
                                endcase
                            end
                        end
                        S_RUN_K1: begin
                            if (child_a_req_w) child_a_rd_addr_r <= child_a_addr_w;
                            if (child_b_req_w) child_b_rd_addr_r <= child_b_addr_w;
                            if (child_c_wr_en_w) begin
                                o_wr_addr_r <= child_c_wr_addr_w; o_wr_data_r <= child_c_wr_data_w;
                            end
                        end
                        S_PREP_K2: begin
                            if (prep_k2_enter_w) begin
                                prep_idx_r <= 0; prep_cap_idx_r <= 0; prep_pending_r <= 2'd0;
                            end else if (prep_cap_idx_r < SIZE_L) begin
                                if (prep_cap_idx_r >= SIZE_H) begin
                                    child_a_we_r <= 1'b1; child_a_wr_addr_r <= prep_cap_idx_r[AW_L-1:0];
                                    child_a_wr_data_r <= {DW{1'b0}};
                                    child_b_we_r <= 1'b1; child_b_wr_addr_r <= prep_cap_idx_r[AW_L-1:0];
                                    child_b_wr_data_r <= {DW{1'b0}};
                                    prep_cap_idx_r <= prep_cap_idx_r + 1'b1;
                                end else begin
                                    case (prep_pending_r)
                                        2'd0: begin
                                            prep_rd_en_r <= 1'b1;
                                            prep_rd_addr_r <= k2_addr_base_w[AW_IN-1:0];
                                            prep_idx_r <= prep_idx_r + 1'b1; prep_pending_r <= 2'd1;
                                        end
                                        2'd1: prep_pending_r <= 2'd2;
                                        default: begin
                                            child_a_we_r <= 1'b1; child_a_wr_addr_r <= prep_cap_idx_r[AW_L-1:0];
                                            child_a_wr_data_r <= a_ram_dout_b_w;
                                            child_b_we_r <= 1'b1; child_b_wr_addr_r <= prep_cap_idx_r[AW_L-1:0];
                                            child_b_wr_data_r <= b_ram_dout_b_w;
                                            prep_cap_idx_r <= prep_cap_idx_r + 1'b1;
                                            if ((prep_cap_idx_r + 1'b1) < SIZE_H && (prep_idx_r < SIZE_H)) begin
                                                prep_rd_en_r <= 1'b1;
                                                prep_rd_addr_r <= k2_addr_base_w[AW_IN-1:0];
                                                prep_idx_r <= prep_idx_r + 1'b1; prep_pending_r <= 2'd1;
                                            end else prep_pending_r <= 2'd0;
                                        end
                                    endcase
                                end
                            end
                        end
                        S_RUN_K2: begin
                            if (child_a_req_w) child_a_rd_addr_r <= child_a_addr_w;
                            if (child_b_req_w) child_b_rd_addr_r <= child_b_addr_w;
                            if (child_c_wr_en_w) begin
                                if (child_c_wr_addr_w < 2*SIZE_H) begin
                                    tmp2_wr_addr_r <= child_c_wr_addr_w[AW_CL_H-1:0];
                                    tmp2_wr_data_r <= child_c_wr_data_w;
                                end
                            end
                            if (child_done_w) prep_cap_idx_r <= 0;
                        end
                        S_PREP_K3: begin
                            if (prep_k3_enter_w) begin
                                prep_cap_idx_r <= 0; k3_step_r <= 3'd0;
                            end else if (prep_cap_idx_r < SIZE_L) begin
                                if (prep_cap_idx_r >= SIZE_H) begin
                                    case (k3_step_r)
                                        3'd0: begin
                                            prep_rd_en_r <= 1'b1;
                                            prep_rd_addr_r <= prep_cap_idx_r[AW_IN-1:0];
                                            k3_step_r <= 3'd1;
                                        end
                                        3'd1: k3_step_r <= 3'd2;
                                        default: begin
                                            child_a_we_r <= 1'b1; child_a_wr_addr_r <= prep_cap_idx_r[AW_L-1:0];
                                            child_a_wr_data_r <= a_ram_dout_b_w;
                                            child_b_we_r <= 1'b1; child_b_wr_addr_r <= prep_cap_idx_r[AW_L-1:0];
                                            child_b_wr_data_r <= b_ram_dout_b_w;
                                            prep_cap_idx_r <= prep_cap_idx_r + 1'b1; k3_step_r <= 3'd0;
                                        end
                                    endcase
                                end else begin
                                    case (k3_step_r)
                                        3'd0: begin
                                            prep_rd_en_r <= 1'b1;
                                            prep_rd_addr_r <= prep_cap_idx_r[AW_IN-1:0];
                                            k3_step_r <= 3'd1;
                                        end
                                        3'd1: k3_step_r <= 3'd2;
                                        3'd2: begin
                                            k3_low_a_r <= a_ram_dout_b_w; k3_low_b_r <= b_ram_dout_b_w;
                                            prep_rd_en_r <= 1'b1;
                                            prep_rd_addr_r <= k3_addr_base_w[AW_IN-1:0];
                                            k3_step_r <= 3'd3;
                                        end
                                        3'd3: k3_step_r <= 3'd4;
                                        default: begin
                                            child_a_we_r <= 1'b1; child_a_wr_addr_r <= prep_cap_idx_r[AW_L-1:0];
                                            child_a_wr_data_r <= k3_low_a_r ^ a_ram_dout_b_w;
                                            child_b_we_r <= 1'b1; child_b_wr_addr_r <= prep_cap_idx_r[AW_L-1:0];
                                            child_b_wr_data_r <= k3_low_b_r ^ b_ram_dout_b_w;
                                            prep_cap_idx_r <= prep_cap_idx_r + 1'b1; k3_step_r <= 3'd0;
                                        end
                                    endcase
                                end
                            end
                        end
                        S_RUN_K3: begin
                            if (child_a_req_w) child_a_rd_addr_r <= child_a_addr_w;
                            if (child_b_req_w) child_b_rd_addr_r <= child_b_addr_w;
                            if (child_c_wr_en_w) begin
                                tmp1_wr_addr_r <= child_c_wr_addr_w; tmp1_wr_data_r <= child_c_wr_data_w;
                            end
                            if (child_done_w) begin
                                merge_rd_idx_r <= {AW_OUT{1'b0}}; merge_rd_pend_r <= 2'd0;
                            end
                        end
                        S_MERGE_WRITE: begin
                            if (merge_enter_w) begin
                                merge_rd_idx_r <= 0; merge_rd_pend_r <= 2'd0;
                            end
                            if ((merge_rd_pend_r == 2'd0) && (merge_rd_idx_r < 2*SIZE)) begin
                                if (merge_rd_idx_r < 2*SIZE_L) begin
                                    o_rd_addr_r <= merge_rd_idx_r[AW_CL-1:0];
                                    tmp1_rd_addr_r <= merge_rd_idx_r[AW_CL-1:0];
                                end
                                if (merge_rd_idx_r < 2*SIZE_H)
                                    tmp2_rd_addr_r <= merge_rd_idx_r[AW_CL_H-1:0];
                                else if (merge_rd_idx_r >= 2*SIZE_L)
                                    tmp2_rd_addr_r <= merge_idx_minus_2l_w[AW_CL_H-1:0];
                                merge_rd_pend_r <= 2'd1;
                            end else if (merge_rd_pend_r == 2'd1) begin
                                if ((merge_rd_idx_r >= SIZE_L) && (merge_rd_idx_r < 2*SIZE_L)) begin
                                    mid_rd_en_r <= 1'b1;
                                    mid_rd_addr_r <= merge_idx_minus_l_w[AW_CL-1:0];
                                end else if ((merge_rd_idx_r >= 2*SIZE_L) && (merge_idx_minus_2l_w < SIZE_L)) begin
                                    mid_rd_en_r <= 1'b1;
                                    mid_rd_addr_r <= merge_mid_hi_idx_w[AW_CL-1:0];
                                end
                                merge_rd_pend_r <= 2'd2;
                            end else if (merge_rd_pend_r == 2'd2) begin
                                o_rd_latch_r <= o_dout_w; tmp1_rd_latch_r <= tmp1_dout_w;
                                tmp2_rd_latch_r <= tmp2_dout_w; merge_rd_pend_r <= 2'd3;
                            end else if (merge_rd_pend_r == 2'd3) begin
                                if (merge_rd_idx_r < 2*SIZE_L) begin
                                    if (merge_rd_idx_r < 2*SIZE_H)
                                        mid_wr_data_r <= o_rd_latch_r ^ tmp1_rd_latch_r ^ tmp2_rd_latch_r;
                                    else
                                        mid_wr_data_r <= o_rd_latch_r ^ tmp1_rd_latch_r;
                                    mid_we_r <= 1'b1; mid_wr_addr_r <= merge_rd_idx_r[AW_CL-1:0];
                                end
                                // OPT-B: no longer write to u_c; FSM goes MERGE→DONE on last index
                                if (merge_rd_idx_r != 2*SIZE - 1) begin
                                    merge_rd_idx_r <= merge_idx_next_w;
                                    if (merge_idx_next_w < 2*SIZE_L) begin
                                        o_rd_en_r <= 1'b1; o_rd_addr_r <= merge_idx_next_w[AW_CL-1:0];
                                        tmp1_rd_en_r <= 1'b1; tmp1_rd_addr_r <= merge_idx_next_w[AW_CL-1:0];
                                    end
                                    if (merge_idx_next_w < 2*SIZE_H) begin
                                        tmp2_rd_en_r <= 1'b1; tmp2_rd_addr_r <= merge_idx_next_w[AW_CL_H-1:0];
                                    end else if (merge_idx_next_w >= 2*SIZE_L) begin
                                        tmp2_rd_en_r <= 1'b1;
                                        tmp2_rd_addr_r <= merge_idx_next_minus_2l_w[AW_CL_H-1:0];
                                    end
                                    merge_rd_pend_r <= 2'd1;
                                end
                            end
                        end
                        S_STREAM_REQ: c_addr_b_r <= stream_idx_r;
                        S_STREAM_SEND: begin
                            stream_valid_r <= 1'b1;
                            if (stream_idx_r != 2*SIZE-1)
                                stream_idx_r <= stream_idx_r + 1'b1;
                        end
                        default: begin end
                    endcase
                end
            end

            assign a_rd_req_o  = fsm_a_rd_req_r;
            assign a_rd_addr_o = fsm_a_rd_addr_r;
            assign b_rd_req_o  = fsm_b_rd_req_r;
            assign b_rd_addr_o = fsm_b_rd_addr_r;
            assign c_wr_en_o   = fsm_c_wr_en_r;
            assign c_wr_addr_o = fsm_c_wr_addr_r;
            assign c_wr_data_o = fsm_c_wr_data_r;
            assign done_o      = fsm_done_r;
            assign debug_state_o = state_r;
        end
    endgenerate

endmodule
