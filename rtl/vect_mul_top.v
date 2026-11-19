`timescale 1ns/1ps
`include "common.vh"

// Single-profile Karatsuba multiplier (SIZE and PARAM_N fixed at synthesis time)
// Adapted from KV260 working version with multi-profile removed
module vect_mul_top #(
    parameter integer AXI_DW = `AXI_DATA_WIDTH,
    parameter integer ADDR_W = `ADDR_WIDTH,
    parameter integer MODE_ADDR_W = `MODE_ADDR_WIDTH,
    parameter integer SIZE = `VEC_N_SIZE_64,
    parameter integer PARAM_N_BITS = `PARAM_N,
    parameter integer PAR_LEVELS = 0,
    parameter integer USE_TOOM3 = 1,
    parameter integer TC_CHILD_PAR_LEVELS = 1
    
) (
    input  wire                             CLK,
    input  wire                             RST,
    input  wire [AXI_DW-1:0]                dina_i,
    input  wire [(AXI_DW/8)-1:0]            wstrb_i,
    input  wire [MODE_ADDR_W+ADDR_W-1:0]    addra_i,
    input  wire                             wea_i,
    input  wire                             ena_i,
    output wire [AXI_DW-1:0]                douta_o
);

    localparam integer DW = `DATA_WIDTH;
    localparam integer IN_AW = (SIZE <= 1) ? 1 : $clog2(SIZE);
    localparam integer PROD_AW = (2*SIZE <= 1) ? 1 : $clog2(2*SIZE);
    localparam integer PACK_DEPTH = (SIZE + 1) / 2;
    localparam integer PACK_AW = (PACK_DEPTH <= 1) ? 1 : $clog2(PACK_DEPTH);

    localparam [1:0] BANK_A = 2'b00;
    localparam [1:0] BANK_B = 2'b01;
    localparam [1:0] BANK_O = 2'b10;

    // Single-profile elaboration-time constants
    localparam integer REM_BITS_W = PARAM_N_BITS % 64;
    localparam integer ACTIVE_WORDS_W = SIZE;

    // FSM states (matched to working KV260 architecture)
    localparam [2:0] ST_IDLE      = 3'd0;
    localparam [2:0] ST_RUN_CORE  = 3'd1;
    localparam [2:0] ST_RED_RD    = 3'd2;  // Read: No control signals, just read from product RAM
    localparam [2:0] ST_RED_WR    = 3'd3;  // Write: Store reduced data to output bank
    localparam [2:0] ST_DONE      = 3'd4;

    // Upper bit of address selects mode (data vs control)
    wire [MODE_ADDR_W-1:0] mode_w = addra_i[MODE_ADDR_W+ADDR_W-1:ADDR_W];
    wire [ADDR_W-1:0] addr_w = addra_i[ADDR_W-1:0];
    wire [1:0] bank_w = addr_w[ADDR_W-1:ADDR_W-2];
    wire [PACK_AW-1:0] idx_pack_w = addr_w[PACK_AW-1:0];

    wire host_data_mode_w = (mode_w == {{(MODE_ADDR_W-1){1'b0}}, 1'b0});
    wire host_ctrl_mode_w = (mode_w == {{(MODE_ADDR_W-1){1'b0}}, 1'b1});

    // Write enable decoding
    wire host_a_wr_en_w = ena_i && wea_i && host_data_mode_w && (bank_w == BANK_A);
    wire host_b_wr_en_w = ena_i && wea_i && host_data_mode_w && (bank_w == BANK_B);

    wire host_wr_lo_lane_w = |wstrb_i[(DW/8)-1:0];
    wire host_wr_hi_lane_w = |wstrb_i[(AXI_DW/8)-1:(DW/8)];

    wire host_a_wr_lo_en_w = host_a_wr_en_w && host_wr_lo_lane_w;
    wire host_a_wr_hi_en_w = host_a_wr_en_w && host_wr_hi_lane_w;
    wire host_b_wr_lo_en_w = host_b_wr_en_w && host_wr_lo_lane_w;
    wire host_b_wr_hi_en_w = host_b_wr_en_w && host_wr_hi_lane_w;

    // Read enable decoding
    wire host_a_rd_en_w = ena_i && (!wea_i) && host_data_mode_w && (bank_w == BANK_A);
    wire host_b_rd_en_w = ena_i && (!wea_i) && host_data_mode_w && (bank_w == BANK_B);
    wire host_o_rd_en_w = ena_i && (!wea_i) && host_data_mode_w && (bank_w == BANK_O);

    // Output multiplexer (staged for BRAM latency)
    reg [AXI_DW-1:0] douta_r;
    assign douta_o = douta_r;

    // State and control registers
    reg start_cmd_r;
    reg done_r;
    reg [2:0] state_r, next_state_r;
    reg [IN_AW-1:0] red_idx_r;
    reg [PROD_AW:0] prod_recv_count_r;
    reg core_start_r;

    // Per-run performance counters
    reg [63:0] cyc_total_r;
    reg [63:0] cyc_run_core_r;
    reg [63:0] cyc_reduce_r;

    // FSM control decode signals
    reg ctrl_store_red_word_w;
    reg ctrl_red_idx_clr_w;
    reg ctrl_red_idx_inc_w;
    reg ctrl_prod_cnt_clr_w;
    reg ctrl_prod_cnt_inc_w;
    reg ctrl_done_set_w;

    // Core input port A
    wire core_a_rd_req_w;
    wire [IN_AW-1:0] core_a_rd_addr_w;
    reg  [DW-1:0] core_a_rd_data_r;
    reg  core_a_hi_sel_r;
    reg  core_a_in_range_r;

    // Core input port B
    wire core_b_rd_req_w;
    wire [IN_AW-1:0] core_b_rd_addr_w;
    reg  [DW-1:0] core_b_rd_data_r;
    reg  core_b_hi_sel_r;
    reg  core_b_in_range_r;

    // Core output port
    wire core_c_wr_en_w;
    wire [PROD_AW-1:0] core_c_wr_addr_w;
    wire [DW-1:0] core_c_wr_data_w;
    wire core_done_w;

    // Product RAM outputs
    wire [DW-1:0] prod0_douta_w;
    wire [DW-1:0] prod0_doutb_w;
    wire [DW-1:0] prod1_douta_w;

    // Reduction module outputs
    wire [DW-1:0] reduce_word_w;
    wire red_last_w;
    wire prod_wr_en_w;

    // Address computation for reduction
    wire [PROD_AW-1:0] active_words_ext_w = {{(PROD_AW-IN_AW){1'b0}}, ACTIVE_WORDS_W};
    wire [PROD_AW-1:0] active_words_m1_ext_w = active_words_ext_w - {{(PROD_AW-1){1'b0}}, 1'b1};
    wire [PROD_AW-1:0] red_idx_ext_w = {{(PROD_AW-IN_AW){1'b0}}, red_idx_r};
    wire [PROD_AW-1:0] red_addr_hprev_w = red_idx_ext_w + active_words_m1_ext_w;
    wire [PROD_AW-1:0] red_addr_h_w = red_idx_ext_w + active_words_ext_w;

    // Packed address computation
    wire [PACK_AW-1:0] core_a_pack_idx_w = core_a_rd_addr_w[IN_AW-1:1];
    wire [PACK_AW-1:0] core_b_pack_idx_w = core_b_rd_addr_w[IN_AW-1:1];
    wire [PACK_AW-1:0] red_pack_idx_w = red_idx_r[IN_AW-1:1];

    // Host read pipeline (for BRAM latency compensation)
    reg host_rd_pending_r;
    reg [1:0] host_rd_bank_r;

    // BRAM port outputs
    wire [DW-1:0] a_lo_host_dout_w;
    wire [DW-1:0] a_hi_host_dout_w;
    wire [DW-1:0] a_lo_core_dout_w;
    wire [DW-1:0] a_hi_core_dout_w;

    wire [DW-1:0] b_lo_host_dout_w;
    wire [DW-1:0] b_hi_host_dout_w;
    wire [DW-1:0] b_lo_core_dout_w;
    wire [DW-1:0] b_hi_core_dout_w;

    wire [DW-1:0] o_lo_host_dout_w;
    wire [DW-1:0] o_hi_host_dout_w;

    // Output bank write enables
    wire o_lo_wr_en_w = ctrl_store_red_word_w && (red_idx_r[0] == 1'b0);
    wire o_hi_wr_en_w = ctrl_store_red_word_w && (red_idx_r[0] == 1'b1);

    // Reduction completion check
    assign red_last_w = (red_idx_r == (ACTIVE_WORDS_W - {{(IN_AW-1){1'b0}}, 1'b1}));
    assign prod_wr_en_w = core_c_wr_en_w && (state_r == ST_RUN_CORE);

    // ===== BRAM BANK A (inputs A) =====
    dual_port_ram #(.AWIDTH(PACK_AW), .DWIDTH(DW)) a_lo_ram (
        .clka (CLK),
        .ena  (host_a_wr_lo_en_w || host_a_rd_en_w),
        .wea  (host_a_wr_lo_en_w),
        .addra(idx_pack_w),
        .dina (dina_i[DW-1:0]),
        .douta(a_lo_host_dout_w),
        .clkb (CLK),
        .enb  (core_a_rd_req_w),
        .web  (1'b0),
        .addrb(core_a_pack_idx_w),
        .dinb ({DW{1'b0}}),
        .doutb(a_lo_core_dout_w)
    );

    dual_port_ram #(.AWIDTH(PACK_AW), .DWIDTH(DW)) a_hi_ram (
        .clka (CLK),
        .ena  (host_a_wr_hi_en_w || host_a_rd_en_w),
        .wea  (host_a_wr_hi_en_w),
        .addra(idx_pack_w),
        .dina (dina_i[AXI_DW-1:DW]),
        .douta(a_hi_host_dout_w),
        .clkb (CLK),
        .enb  (core_a_rd_req_w),
        .web  (1'b0),
        .addrb(core_a_pack_idx_w),
        .dinb ({DW{1'b0}}),
        .doutb(a_hi_core_dout_w)
    );

    // ===== BRAM BANK B (inputs B) =====
    dual_port_ram #(.AWIDTH(PACK_AW), .DWIDTH(DW)) b_lo_ram (
        .clka (CLK),
        .ena  (host_b_wr_lo_en_w || host_b_rd_en_w),
        .wea  (host_b_wr_lo_en_w),
        .addra(idx_pack_w),
        .dina (dina_i[DW-1:0]),
        .douta(b_lo_host_dout_w),
        .clkb (CLK),
        .enb  (core_b_rd_req_w),
        .web  (1'b0),
        .addrb(core_b_pack_idx_w),
        .dinb ({DW{1'b0}}),
        .doutb(b_lo_core_dout_w)
    );

    dual_port_ram #(.AWIDTH(PACK_AW), .DWIDTH(DW)) b_hi_ram (
        .clka (CLK),
        .ena  (host_b_wr_hi_en_w || host_b_rd_en_w),
        .wea  (host_b_wr_hi_en_w),
        .addra(idx_pack_w),
        .dina (dina_i[AXI_DW-1:DW]),
        .douta(b_hi_host_dout_w),
        .clkb (CLK),
        .enb  (core_b_rd_req_w),
        .web  (1'b0),
        .addrb(core_b_pack_idx_w),
        .dinb ({DW{1'b0}}),
        .doutb(b_hi_core_dout_w)
    );

    // ===== BRAM BANK O (output) =====
    dual_port_ram #(.AWIDTH(PACK_AW), .DWIDTH(DW)) o_lo_ram (
        .clka (CLK),
        .ena  (o_lo_wr_en_w),
        .wea  (o_lo_wr_en_w),
        .addra(red_pack_idx_w),
        .dina (reduce_word_w),
        .douta(),
        .clkb (CLK),
        .enb  (host_o_rd_en_w),
        .web  (1'b0),
        .addrb(idx_pack_w),
        .dinb ({DW{1'b0}}),
        .doutb(o_lo_host_dout_w)
    );

    dual_port_ram #(.AWIDTH(PACK_AW), .DWIDTH(DW)) o_hi_ram (
        .clka (CLK),
        .ena  (o_hi_wr_en_w),
        .wea  (o_hi_wr_en_w),
        .addra(red_pack_idx_w),
        .dina (reduce_word_w),
        .douta(),
        .clkb (CLK),
        .enb  (host_o_rd_en_w),
        .web  (1'b0),
        .addrb(idx_pack_w),
        .dinb ({DW{1'b0}}),
        .doutb(o_hi_host_dout_w)
    );

    // 128->64 bridge for core input A (select lo/hi based on address LSB)
    always @(core_a_hi_sel_r or a_lo_core_dout_w or a_hi_core_dout_w) begin
        if (!core_a_in_range_r)
            core_a_rd_data_r = {DW{1'b0}};
        else if (core_a_hi_sel_r)
            core_a_rd_data_r = a_hi_core_dout_w;
        else
            core_a_rd_data_r = a_lo_core_dout_w;
    end

    // 128->64 bridge for core input B
    always @(core_b_hi_sel_r or b_lo_core_dout_w or b_hi_core_dout_w) begin
        if (!core_b_in_range_r)
            core_b_rd_data_r = {DW{1'b0}};
        else if (core_b_hi_sel_r)
            core_b_rd_data_r = b_hi_core_dout_w;
        else
            core_b_rd_data_r = b_lo_core_dout_w;
    end

    // Product RAMs for reduction pipeline
    dual_port_ram #(
        .AWIDTH(PROD_AW),
        .DWIDTH(DW)
    ) prod_ram0 (
        .clka (CLK),
        .ena  (prod_wr_en_w || (state_r == ST_RED_RD)),
        .wea  (prod_wr_en_w),
        .addra(prod_wr_en_w ? core_c_wr_addr_w : red_idx_ext_w),
        .dina (core_c_wr_data_w),
        .douta(prod0_douta_w),
        .clkb (CLK),
        .enb  (state_r == ST_RED_RD),
        .web  (1'b0),
        .addrb(red_addr_hprev_w),
        .dinb ({DW{1'b0}}),
        .doutb(prod0_doutb_w)
    );

    dual_port_ram #(
        .AWIDTH(PROD_AW),
        .DWIDTH(DW)
    ) prod_ram1 (
        .clka (CLK),
        .ena  (prod_wr_en_w || (state_r == ST_RED_RD)),
        .wea  (prod_wr_en_w),
        .addra(prod_wr_en_w ? core_c_wr_addr_w : red_addr_h_w),
        .dina (core_c_wr_data_w),
        .douta(prod1_douta_w),
        .clkb (CLK),
        .enb  (1'b0),
        .web  (1'b0),
        .addrb({PROD_AW{1'b0}}),
        .dinb ({DW{1'b0}}),
        .doutb()
    );

    // Reduction word computation
    reduce_word_comb reduce_step_inst (
        .low_i      (prod0_douta_w),
        .high_prev_i(prod0_doutb_w),
        .high_i     (prod1_douta_w),
        .rem_bits_i (REM_BITS_W),
        .is_last_i  (red_last_w),
        .word_o     (reduce_word_w)
    );

    // Core selection. USE_TOOM3=1 selects top-level TC3 with one reused 4Flat child.
    // USE_TOOM3=0 restores the previous Karatsuba-only core for A/B comparison.
    generate
        if (USE_TOOM3 != 0) begin : GEN_HYBRID_TC3
            toom3_4flat_top #(
                .SIZE(SIZE),
                .CHILD_PAR_LEVELS(TC_CHILD_PAR_LEVELS)
            ) core_inst (
                .CLK        (CLK),
                .RST        (RST),
                .start_i    (core_start_r),
                .a_rd_req_o (core_a_rd_req_w),
                .a_rd_addr_o(core_a_rd_addr_w),
                .a_rd_data_i(core_a_rd_data_r),
                .b_rd_req_o (core_b_rd_req_w),
                .b_rd_addr_o(core_b_rd_addr_w),
                .b_rd_data_i(core_b_rd_data_r),
                .c_wr_en_o  (core_c_wr_en_w),
                .c_wr_addr_o(core_c_wr_addr_w),
                .c_wr_data_o(core_c_wr_data_w),
                .done_o     (core_done_w),
                .debug_state_o()
            );
        end else begin : GEN_KARATSUBA_ONLY
            karatsuba_top #(
                .SIZE(SIZE),
                .PAR_LEVELS(PAR_LEVELS)
            ) core_inst (
                .CLK        (CLK),
                .RST        (RST),
                .start_i    (core_start_r),
                .a_rd_req_o (core_a_rd_req_w),
                .a_rd_addr_o(core_a_rd_addr_w),
                .a_rd_data_i(core_a_rd_data_r),
                .b_rd_req_o (core_b_rd_req_w),
                .b_rd_addr_o(core_b_rd_addr_w),
                .b_rd_data_i(core_b_rd_data_r),
                .c_wr_en_o  (core_c_wr_en_w),
                .c_wr_addr_o(core_c_wr_addr_w),
                .c_wr_data_o(core_c_wr_data_w),
                .done_o     (core_done_w),
                .debug_state_o()
            );
        end
    endgenerate

    // ===== FSM NEXT STATE LOGIC =====
    always @(state_r or start_cmd_r or core_done_w or red_last_w) begin
        case (state_r)
            ST_IDLE:     next_state_r = start_cmd_r ? ST_RUN_CORE : ST_IDLE;
            ST_RUN_CORE: next_state_r = core_done_w ? ST_RED_RD : ST_RUN_CORE;
            ST_RED_RD:   next_state_r = ST_RED_WR;  // Always proceed to write
            ST_RED_WR:   next_state_r = red_last_w ? ST_DONE : ST_RED_RD;  // Loop back or done
            ST_DONE:     next_state_r = start_cmd_r ? ST_RUN_CORE : ST_DONE;
            default:     next_state_r = ST_IDLE;
        endcase
    end

    // ===== FSM CONTROL DECODE (Full assignment per state) =====
    always @(state_r or core_c_wr_en_w or prod_recv_count_r or red_last_w) begin
        case (state_r)
            ST_IDLE: begin
                ctrl_store_red_word_w = 1'b0;
                ctrl_red_idx_clr_w    = 1'b0;
                ctrl_red_idx_inc_w    = 1'b0;
                ctrl_prod_cnt_clr_w   = 1'b0;
                ctrl_prod_cnt_inc_w   = 1'b0;
                ctrl_done_set_w       = 1'b0;
            end

            ST_RUN_CORE: begin
                ctrl_store_red_word_w = 1'b0;
                ctrl_red_idx_clr_w    = 1'b0;
                ctrl_red_idx_inc_w    = 1'b0;
                ctrl_prod_cnt_clr_w   = 1'b0;
                ctrl_prod_cnt_inc_w   = core_c_wr_en_w && (prod_recv_count_r < (2*SIZE));
                ctrl_done_set_w       = 1'b0;
            end

            ST_RED_RD: begin
                // Read phase: No control signals, just wait for BRAM latency
                ctrl_store_red_word_w = 1'b0;
                ctrl_red_idx_clr_w    = 1'b0;
                ctrl_red_idx_inc_w    = 1'b0;
                ctrl_prod_cnt_clr_w   = 1'b0;
                ctrl_prod_cnt_inc_w   = 1'b0;
                ctrl_done_set_w       = 1'b0;
            end

            ST_RED_WR: begin
                // Write phase: Store reduced data and increment index
                ctrl_store_red_word_w = 1'b1;
                ctrl_red_idx_clr_w    = red_last_w;
                ctrl_red_idx_inc_w    = !red_last_w;
                ctrl_prod_cnt_clr_w   = 1'b0;
                ctrl_prod_cnt_inc_w   = 1'b0;
                ctrl_done_set_w       = red_last_w;
            end

            ST_DONE: begin
                ctrl_store_red_word_w = 1'b0;
                ctrl_red_idx_clr_w    = 1'b0;
                ctrl_red_idx_inc_w    = 1'b0;
                ctrl_prod_cnt_clr_w   = 1'b0;
                ctrl_prod_cnt_inc_w   = 1'b0;
                ctrl_done_set_w       = 1'b0;
            end

            default: begin
                ctrl_store_red_word_w = 1'b0;
                ctrl_red_idx_clr_w    = 1'b0;
                ctrl_red_idx_inc_w    = 1'b0;
                ctrl_prod_cnt_clr_w   = 1'b0;
                ctrl_prod_cnt_inc_w   = 1'b0;
                ctrl_done_set_w       = 1'b0;
            end
        endcase
    end

    // ===== SEQUENTIAL LOGIC =====
    always @(posedge CLK or negedge RST) begin
        if (!RST) begin
            state_r <= ST_IDLE;
            red_idx_r <= {IN_AW{1'b0}};
            prod_recv_count_r <= {(PROD_AW+1){1'b0}};
            core_start_r <= 1'b0;
            cyc_total_r <= 64'd0;
            cyc_run_core_r <= 64'd0;
            cyc_reduce_r <= 64'd0;
            core_a_hi_sel_r <= 1'b0;
            core_a_in_range_r <= 1'b0;
            core_b_hi_sel_r <= 1'b0;
            core_b_in_range_r <= 1'b0;
            start_cmd_r <= 1'b0;
            done_r <= 1'b0;
            host_rd_pending_r <= 1'b0;
            host_rd_bank_r <= 2'b00;
            douta_r <= {AXI_DW{1'b0}};
        end else begin
            state_r <= next_state_r;
            core_start_r <= 1'b0;

            // Track core address MSB for lo/hi lane selection
            if (core_a_rd_req_w)
                core_a_hi_sel_r <= core_a_rd_addr_w[0];
            if (core_a_rd_req_w)
                core_a_in_range_r <= (core_a_rd_addr_w < ACTIVE_WORDS_W);

            if (core_b_rd_req_w)
                core_b_hi_sel_r <= core_b_rd_addr_w[0];
            if (core_b_rd_req_w)
                core_b_in_range_r <= (core_b_rd_addr_w < ACTIVE_WORDS_W);

            // Host read pipeline (1-cycle delay for BRAM latency)
            if (host_rd_pending_r) begin
                if (host_rd_bank_r == BANK_A)
                    douta_r <= {a_hi_host_dout_w, a_lo_host_dout_w};
                else if (host_rd_bank_r == BANK_B)
                    douta_r <= {b_hi_host_dout_w, b_lo_host_dout_w};
                else if (host_rd_bank_r == BANK_O)
                    douta_r <= {o_hi_host_dout_w, o_lo_host_dout_w};
                else
                    douta_r <= {AXI_DW{1'b0}};

                host_rd_pending_r <= 1'b0;
            end

            // ===== Host bus handling =====
            if (ena_i) begin
                if (wea_i) begin
                    if (host_ctrl_mode_w) begin
                        // Control mode: CTRL[0] = start command pulse
                        if ((addr_w[3:0] == 4'h0) && dina_i[0]) begin
                            start_cmd_r <= 1'b1;
                        end
                    end
                end else begin
                    // Read path
                    if (host_data_mode_w) begin
                        // Data read: Set pending flag for 1-cycle BRAM latency
                        host_rd_pending_r <= 1'b1;
                        host_rd_bank_r <= bank_w;
                    end else if (host_ctrl_mode_w) begin
                        // Control mode reads: Return status/profile info directly to douta_r
                        // CTRL[0]: Status register
                        // CTRL[1]: Profile info (SIZE, REM_BITS)
                        // CTRL[2-7]: Performance counters
                        if (addr_w[3:0] == 4'h0) begin
                            douta_r <= { {(AXI_DW-5){1'b0}}, state_r, done_r,
                                         ((state_r != ST_IDLE) && (state_r != ST_DONE)) };
                        end else if (addr_w[3:0] == 4'h1) begin
                            // Return profile parameters: SIZE and REM_BITS
                            douta_r <= {{(AXI_DW-IN_AW-6){1'b0}}, REM_BITS_W, ACTIVE_WORDS_W};
                        end else if (addr_w[3:0] == 4'h2) begin
                            douta_r <= {{(AXI_DW-32){1'b0}}, cyc_total_r[31:0]};
                        end else if (addr_w[3:0] == 4'h3) begin
                            douta_r <= {{(AXI_DW-32){1'b0}}, cyc_total_r[63:32]};
                        end else if (addr_w[3:0] == 4'h4) begin
                            douta_r <= {{(AXI_DW-32){1'b0}}, cyc_run_core_r[31:0]};
                        end else if (addr_w[3:0] == 4'h5) begin
                            douta_r <= {{(AXI_DW-32){1'b0}}, cyc_run_core_r[63:32]};
                        end else if (addr_w[3:0] == 4'h6) begin
                            douta_r <= {{(AXI_DW-32){1'b0}}, cyc_reduce_r[31:0]};
                        end else if (addr_w[3:0] == 4'h7) begin
                            douta_r <= {{(AXI_DW-32){1'b0}}, cyc_reduce_r[63:32]};
                        end else if (addr_w[3:0] == 4'h8) begin
                            douta_r <= {{(AXI_DW-32){1'b0}}, 32'd0};
                        end else if (addr_w[3:0] == 4'h9) begin
                            douta_r <= {{(AXI_DW-32){1'b0}}, 32'd0};
                        end
                    end
                end
            end

            // ===== Core start pulse on transitions =====
            if ((state_r == ST_IDLE || state_r == ST_DONE) && start_cmd_r) begin
                core_start_r <= 1'b1;
                start_cmd_r <= 1'b0;
                done_r <= 1'b0;  // Clear done when accepting start command
                red_idx_r <= {IN_AW{1'b0}};
                prod_recv_count_r <= {(PROD_AW+1){1'b0}};
                cyc_total_r <= 64'd0;
                cyc_run_core_r <= 64'd0;
                cyc_reduce_r <= 64'd0;
            end

            // Performance counter updates
            if (state_r != ST_IDLE && state_r != ST_DONE)
                cyc_total_r <= cyc_total_r + 64'd1;
            if (state_r == ST_RUN_CORE)
                cyc_run_core_r <= cyc_run_core_r + 64'd1;
            if (state_r == ST_RED_RD || state_r == ST_RED_WR)
                cyc_reduce_r <= cyc_reduce_r + 64'd1;

            // Product count increment
            if (ctrl_prod_cnt_inc_w)
                prod_recv_count_r <= prod_recv_count_r + 1'b1;

            // Reduction index management
            if (ctrl_red_idx_clr_w)
                red_idx_r <= {IN_AW{1'b0}};
            else if (ctrl_red_idx_inc_w)
                red_idx_r <= red_idx_r + 1'b1;

            // Done flag update
            if (ctrl_done_set_w)
                done_r <= 1'b1;
        end
    end

endmodule
