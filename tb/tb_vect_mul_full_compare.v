`timescale 1ns/1ps
`include "common.vh"

// Common full-VectMul testbench for BOTH designs:
//   1) original RTL_BASE_CASE_4_FLAT/vect_mul_top.v
//   2) 4Flat Parallel vect_mul_top.v (default PAR_LEVELS=1)
//
// Test vector: A(x)=1, B(x)=deterministic pattern.
// For multiplication in GF(2)[x]/(x^n-1), expected output is C(x)=B(x).
// The testbench loads A/B through the public host interface, starts via CTRL[0],
// waits for DUT done, prints built-in performance counters, and reads output back.

module tb_vect_mul_full_compare;

    localparam integer AXI_DW       = `AXI_DATA_WIDTH;
    localparam integer ADDR_W       = `ADDR_WIDTH;
    localparam integer MODE_ADDR_W  = `MODE_ADDR_WIDTH;
    localparam integer BUS_AW       = ADDR_W + MODE_ADDR_W;
    localparam integer SIZE         = `VEC_N_SIZE_64;
    localparam integer PARAM_N_BITS = `PARAM_N;
    localparam integer PACK_DEPTH   = (SIZE + 1) / 2;
    localparam integer REM_BITS     = PARAM_N_BITS % 64;

    localparam [1:0] BANK_A = 2'b00;
    localparam [1:0] BANK_B = 2'b01;
    localparam [1:0] BANK_O = 2'b10;

    reg                         CLK;
    reg                         RST;
    reg  [AXI_DW-1:0]           dina_i;
    reg  [(AXI_DW/8)-1:0]       wstrb_i;
    reg  [BUS_AW-1:0]           addra_i;
    reg                         wea_i;
    reg                         ena_i;
    wire [AXI_DW-1:0]           douta_o;

    reg [63:0] b_word [0:SIZE-1];
    reg [127:0] rd_data;
    reg [63:0] exp_lo, exp_hi;
    integer i;
    integer errors;
    integer watchdog;
    reg [63:0] elapsed_cycles;
    reg [63:0] cyc_total_snap;
    reg [63:0] cyc_core_snap;
    reg [63:0] cyc_reduce_snap;

    // IMPORTANT: No parameter override here.
    // Old 4Flat has no PAR_LEVELS parameter.
    // Parallel 4Flat defaults to PAR_LEVELS=1.
    vect_mul_top dut (
        .CLK     (CLK),
        .RST     (RST),
        .dina_i  (dina_i),
        .wstrb_i (wstrb_i),
        .addra_i (addra_i),
        .wea_i   (wea_i),
        .ena_i   (ena_i),
        .douta_o (douta_o)
    );

    // 100 MHz simulation clock. Cycle count is independent of this frequency.
    always #5 CLK = ~CLK;

    function [BUS_AW-1:0] data_addr;
        input [1:0] bank;
        input integer idx;
        reg [BUS_AW-1:0] tmp;
        begin
            tmp = {BUS_AW{1'b0}};
            tmp[ADDR_W-1:ADDR_W-2] = bank;
            tmp[ADDR_W-3:0] = idx;
            data_addr = tmp;
        end
    endfunction

    function [BUS_AW-1:0] ctrl_addr;
        input [3:0] offset;
        reg [BUS_AW-1:0] tmp;
        begin
            tmp = {BUS_AW{1'b0}};
            // mode = 1 => control mode
            tmp[ADDR_W+MODE_ADDR_W-1:ADDR_W] = {{(MODE_ADDR_W-1){1'b0}}, 1'b1};
            tmp[3:0] = offset;
            ctrl_addr = tmp;
        end
    endfunction

    task host_write128;
        input [BUS_AW-1:0] addr;
        input [127:0] data;
        begin
            @(negedge CLK);
            addra_i = addr;
            dina_i  = data;
            wstrb_i = {(AXI_DW/8){1'b1}};
            ena_i   = 1'b1;
            wea_i   = 1'b1;
            @(posedge CLK);
            #1;
            @(negedge CLK);
            ena_i   = 1'b0;
            wea_i   = 1'b0;
            addra_i = {BUS_AW{1'b0}};
            dina_i  = {AXI_DW{1'b0}};
        end
    endtask

    task host_read128;
        input [BUS_AW-1:0] addr;
        output [127:0] data;
        begin
            // Issue synchronous BRAM read request.
            @(negedge CLK);
            addra_i = addr;
            ena_i   = 1'b1;
            wea_i   = 1'b0;
            @(posedge CLK);
            #1;
            @(negedge CLK);
            ena_i   = 1'b0;
            addra_i = {BUS_AW{1'b0}};

            // vect_mul_top has one extra pending stage after BRAM read latency.
            @(posedge CLK);
            #1;
            data = douta_o;
        end
    endtask

    initial begin
        CLK = 1'b0;
        RST = 1'b0;          // Active-low reset in vect_mul_top
        dina_i = {AXI_DW{1'b0}};
        wstrb_i = {(AXI_DW/8){1'b0}};
        addra_i = {BUS_AW{1'b0}};
        wea_i = 1'b0;
        ena_i = 1'b0;
        errors = 0;
        elapsed_cycles = 0;

        // Deterministic B. Data-independent FSM means this does not bias cycle count.
        for (i = 0; i < SIZE; i = i + 1)
            b_word[i] = 64'h0123_4567_89AB_CDEF ^ i;

        // Only PARAM_N_BITS bits are valid in the final 64-bit word.
        if (REM_BITS != 0)
            b_word[SIZE-1] = b_word[SIZE-1] & ((64'h1 << REM_BITS) - 1'b1);

        repeat (8) @(posedge CLK);
        RST = 1'b1;
        repeat (4) @(posedge CLK);

        $display("============================================================");
        $display("Full VectMul benchmark: PARAM_N=%0d, SIZE=%0d words", PARAM_N_BITS, SIZE);
        $display("Test: A(x)=1, therefore expected C(x)=B(x)");
        $display("============================================================");

        // ------------------------------------------------------------
        // Load bank A and bank B through the normal 128-bit host port.
        // ------------------------------------------------------------
        for (i = 0; i < PACK_DEPTH; i = i + 1) begin
            // A(x) = 1
            if (i == 0)
                host_write128(data_addr(BANK_A, i), {64'd0, 64'd1});
            else
                host_write128(data_addr(BANK_A, i), 128'd0);

            exp_lo = b_word[2*i];
            if ((2*i + 1) < SIZE)
                exp_hi = b_word[2*i + 1];
            else
                exp_hi = 64'd0;
            host_write128(data_addr(BANK_B, i), {exp_hi, exp_lo});
        end

        // Start through CTRL[0]. bit[0] = 1 is the start command.
        host_write128(ctrl_addr(4'h0), {{(AXI_DW-1){1'b0}}, 1'b1});

        // Wait for the internal done flag while counting external elapsed cycles.
        // The RTL also contains exact per-run counters; both are printed below.
        watchdog = 0;
        elapsed_cycles = 0;
        while ((dut.done_r !== 1'b1) && (watchdog < 5000000)) begin
            @(posedge CLK);
            watchdog = watchdog + 1;
            elapsed_cycles = elapsed_cycles + 1'b1;
        end

        if (watchdog >= 5000000) begin
            $display("FAIL: timeout waiting for vect_mul_top done");
            $finish;
        end

        #1;
        cyc_total_snap  = dut.cyc_total_r;
        cyc_core_snap   = dut.cyc_run_core_r;
        cyc_reduce_snap = dut.cyc_reduce_r;

        $display("RESULT: elapsed_cycles_from_host_start = %0d", elapsed_cycles);
        $display("RESULT: cyc_total_r                  = %0d", cyc_total_snap);
        $display("RESULT: cyc_run_core_r               = %0d", cyc_core_snap);
        $display("RESULT: cyc_reduce_r                 = %0d", cyc_reduce_snap);

        // ------------------------------------------------------------
        // Read result bank and verify C == B.
        // Ignore the unused upper 64-bit lane in the last packed word if SIZE is odd.
        // ------------------------------------------------------------
        for (i = 0; i < PACK_DEPTH; i = i + 1) begin
            host_read128(data_addr(BANK_O, i), rd_data);

            exp_lo = b_word[2*i];
            if (rd_data[63:0] !== exp_lo) begin
                $display("MISMATCH word %0d: got=%016h exp=%016h", 2*i, rd_data[63:0], exp_lo);
                errors = errors + 1;
            end

            if ((2*i + 1) < SIZE) begin
                exp_hi = b_word[2*i + 1];
                if (rd_data[127:64] !== exp_hi) begin
                    $display("MISMATCH word %0d: got=%016h exp=%016h", 2*i+1, rd_data[127:64], exp_hi);
                    errors = errors + 1;
                end
            end
        end

        if (errors == 0)
            $display("PASS: full VectMul output correct; cyc_total=%0d, core=%0d, reduce=%0d",
                     cyc_total_snap, cyc_core_snap, cyc_reduce_snap);
        else
            $display("FAIL: %0d output mismatches", errors);

        $finish;
    end

endmodule
