//============================================================================
// File        : dm_cache_tb.v
// Description : Self-checking testbench for dm_cache
//
//   - Behavioral main memory with configurable latency (MEM_LAT)
//   - Golden reference memory  -> checks every read returns correct data
//   - Reference tag model      -> predicts HIT/MISS for every access
//   - Directed tests  : cold misses, spatial hits, write hit, write miss
//                       (allocate), dirty eviction + write-back, clean
//                       eviction, full-cache fill/re-read, hit latency
//   - Random test     : mixed reads/writes with locality and conflicts
//   - Prints PASS/FAIL summary; dumps dm_cache_tb.vcd for waveforms
//============================================================================
`timescale 1ns/1ps

module dm_cache_tb;

    //------------------------------------------------------------------
    // Parameters (must match DUT)
    //------------------------------------------------------------------
    localparam ADDR_W         = 32;
    localparam DATA_W         = 32;
    localparam NUM_LINES      = 16;
    localparam WORDS_PER_LINE = 4;
    localparam LINE_W         = DATA_W * WORDS_PER_LINE;

    localparam OFFSET_W = 4;                       // 2 byte + 2 word
    localparam INDEX_W  = 4;                       // 16 lines
    localparam TAG_W    = ADDR_W - INDEX_W - OFFSET_W;

    localparam MEM_WORDS   = 4096;                 // 16 KB backing store
    localparam MEM_LAT     = 4;                    // memory latency (cycles)
    localparam NUM_RANDOM  = 3000;

    //------------------------------------------------------------------
    // Signals
    //------------------------------------------------------------------
    reg                 clk = 1'b0;
    reg                 rst_n;

    reg                 cpu_req;
    reg                 cpu_we;
    reg  [ADDR_W-1:0]   cpu_addr;
    reg  [DATA_W-1:0]   cpu_wdata;
    wire [DATA_W-1:0]   cpu_rdata;
    wire                cpu_ready;
    wire                cpu_busy;
    wire                hit;
    wire                miss;

    wire                mem_req;
    wire                mem_we;
    wire [ADDR_W-1:0]   mem_addr;
    wire [LINE_W-1:0]   mem_wdata;
    reg  [LINE_W-1:0]   mem_rdata;
    reg                 mem_ready;

    always #5 clk = ~clk;                          // 100 MHz

    //------------------------------------------------------------------
    // DUT
    //------------------------------------------------------------------
    dm_cache #(
        .ADDR_W         (ADDR_W),
        .DATA_W         (DATA_W),
        .NUM_LINES      (NUM_LINES),
        .WORDS_PER_LINE (WORDS_PER_LINE)
    ) dut (
        .clk       (clk),
        .rst_n     (rst_n),
        .cpu_req   (cpu_req),
        .cpu_we    (cpu_we),
        .cpu_addr  (cpu_addr),
        .cpu_wdata (cpu_wdata),
        .cpu_rdata (cpu_rdata),
        .cpu_ready (cpu_ready),
        .cpu_busy  (cpu_busy),
        .hit       (hit),
        .miss      (miss),
        .mem_req   (mem_req),
        .mem_we    (mem_we),
        .mem_addr  (mem_addr),
        .mem_wdata (mem_wdata),
        .mem_rdata (mem_rdata),
        .mem_ready (mem_ready)
    );

    //------------------------------------------------------------------
    // Main memory model (block transfers, fixed latency)
    //------------------------------------------------------------------
    reg  [DATA_W-1:0] main_mem [0:MEM_WORDS-1];
    wire [11:0]       mem_base = mem_addr[13:2];   // word index of block

    integer lat_cnt;
    integer k;
    integer mem_reads;
    integer mem_writes;

    always @(posedge clk) begin
        mem_ready <= 1'b0;
        if (mem_req && !mem_ready) begin
            if (lat_cnt == MEM_LAT-1) begin
                lat_cnt   <= 0;
                mem_ready <= 1'b1;
                if (mem_we) begin
                    for (k = 0; k < WORDS_PER_LINE; k = k + 1)
                        main_mem[mem_base + k] = mem_wdata[k*DATA_W +: DATA_W];
                    mem_writes = mem_writes + 1;
                end else begin
                    for (k = 0; k < WORDS_PER_LINE; k = k + 1)
                        mem_rdata[k*DATA_W +: DATA_W] <= main_mem[mem_base + k];
                    mem_reads = mem_reads + 1;
                end
            end else begin
                lat_cnt <= lat_cnt + 1;
            end
        end
    end

    //------------------------------------------------------------------
    // Reference models
    //------------------------------------------------------------------
    reg [DATA_W-1:0] ref_mem   [0:MEM_WORDS-1];    // golden data
    reg              ref_valid [0:NUM_LINES-1];    // golden tag store
    reg [TAG_W-1:0]  ref_tag   [0:NUM_LINES-1];

    //------------------------------------------------------------------
    // Scoreboard
    //------------------------------------------------------------------
    integer errors;
    integer checks;
    integer n_hits;
    integer n_misses;
    reg     verbose;

    //------------------------------------------------------------------
    // CPU access task: drives one request and checks the result
    //------------------------------------------------------------------
    task cpu_access;
        input              we;
        input [ADDR_W-1:0] addr;
        input [DATA_W-1:0] wdata;

        reg [INDEX_W-1:0] idx;
        reg [TAG_W-1:0]   tag;
        reg [11:0]        widx;
        reg               exp_hit;
        reg               saw_hit;
        reg               saw_miss;
        integer           cycles;
        begin
            idx     = addr[OFFSET_W +: INDEX_W];
            tag     = addr[ADDR_W-1 -: TAG_W];
            widx    = addr[13:2];
            exp_hit = ref_valid[idx] && (ref_tag[idx] == tag);

            // Issue request
            @(negedge clk);
            if (cpu_busy) begin
                $display("[%0t] ERROR: cache busy when request issued", $time);
                errors = errors + 1;
            end
            cpu_req   = 1'b1;
            cpu_we    = we;
            cpu_addr  = addr;
            cpu_wdata = wdata;
            @(negedge clk);
            cpu_req   = 1'b0;
            cpu_we    = 1'b0;

            // Wait for completion
            saw_hit  = 1'b0;
            saw_miss = 1'b0;
            cycles   = 0;
            while (!cpu_ready && cycles < 200) begin
                @(negedge clk);
                if (hit)  saw_hit  = 1'b1;
                if (miss) saw_miss = 1'b1;
                if (hit && miss) begin
                    $display("[%0t] ERROR: hit and miss asserted together", $time);
                    errors = errors + 1;
                end
                cycles = cycles + 1;
            end

            checks = checks + 1;

            if (!cpu_ready) begin
                $display("[%0t] ERROR: timeout, addr=0x%08h", $time, addr);
                errors = errors + 1;
            end else begin
                // Hit / miss check
                if (saw_hit !== exp_hit || saw_miss !== !exp_hit) begin
                    $display("[%0t] ERROR: addr=0x%08h expected %s, got hit=%b miss=%b",
                             $time, addr, exp_hit ? "HIT" : "MISS", saw_hit, saw_miss);
                    errors = errors + 1;
                end

                // Hit latency check (1 cycle after request accepted)
                if (exp_hit && cycles != 1) begin
                    $display("[%0t] ERROR: hit latency %0d cycles, expected 1", $time, cycles);
                    errors = errors + 1;
                end

                // Read data check
                if (!we && cpu_rdata !== ref_mem[widx]) begin
                    $display("[%0t] ERROR: READ addr=0x%08h got 0x%08h expected 0x%08h",
                             $time, addr, cpu_rdata, ref_mem[widx]);
                    errors = errors + 1;
                end

                if (verbose)
                    $display("[%0t] %s addr=0x%08h data=0x%08h  %s  (%0d cycles)",
                             $time, we ? "WRITE" : "READ ", addr,
                             we ? wdata : cpu_rdata,
                             saw_hit ? "HIT " : "MISS", cycles + 1);
            end

            // Update reference models
            if (we) ref_mem[widx] = wdata;
            ref_valid[idx] = 1'b1;
            ref_tag[idx]   = tag;
            if (exp_hit) n_hits   = n_hits + 1;
            else         n_misses = n_misses + 1;
        end
    endtask

    // Expect a given number of write-backs since a snapshot
    task check_writebacks;
        input integer before;
        input integer expected;
        begin
            checks = checks + 1;
            if (mem_writes - before !== expected) begin
                $display("[%0t] ERROR: expected %0d write-back(s), saw %0d",
                         $time, expected, mem_writes - before);
                errors = errors + 1;
            end else if (verbose) begin
                $display("[%0t] OK: %0d write-back(s) observed", $time, expected);
            end
        end
    endtask

    //------------------------------------------------------------------
    // Stimulus
    //------------------------------------------------------------------
    integer i;
    integer wb_snap;
    integer seed;
    reg [ADDR_W-1:0] r_addr;
    reg [DATA_W-1:0] r_data;
    reg              r_we;

    initial begin
        $dumpfile("dm_cache_tb.vcd");
        $dumpvars(0, dm_cache_tb);

        // Init
        errors     = 0;
        checks     = 0;
        n_hits     = 0;
        n_misses   = 0;
        mem_reads  = 0;
        mem_writes = 0;
        lat_cnt    = 0;
        verbose    = 1'b1;
        seed       = 32'h1234_5678;

        cpu_req   = 1'b0;
        cpu_we    = 1'b0;
        cpu_addr  = 0;
        cpu_wdata = 0;
        mem_ready = 1'b0;
        mem_rdata = 0;

        for (i = 0; i < MEM_WORDS; i = i + 1) begin
            main_mem[i] = 32'hC0DE_0000 ^ (i * 32'h0001_0101);
            ref_mem[i]  = 32'hC0DE_0000 ^ (i * 32'h0001_0101);
        end
        for (i = 0; i < NUM_LINES; i = i + 1) begin
            ref_valid[i] = 1'b0;
            ref_tag[i]   = 0;
        end

        // Reset
        rst_n = 1'b0;
        repeat (3) @(posedge clk);
        rst_n = 1'b1;

        //--------------------------------------------------------------
        $display("\n--- Test 1: cold miss + spatial locality hits ---");
        cpu_access(0, 32'h0000_0000, 0);   // miss (cold)
        cpu_access(0, 32'h0000_0004, 0);   // hit  (same block)
        cpu_access(0, 32'h0000_0008, 0);   // hit
        cpu_access(0, 32'h0000_000C, 0);   // hit
        cpu_access(0, 32'h0000_0010, 0);   // miss (index 1)

        //--------------------------------------------------------------
        $display("\n--- Test 2: write hit, read back ---");
        cpu_access(1, 32'h0000_0004, 32'h1111_1111);  // write hit -> dirty
        cpu_access(0, 32'h0000_0004, 0);              // read hit

        //--------------------------------------------------------------
        $display("\n--- Test 3: write miss (write-allocate) ---");
        cpu_access(1, 32'h0000_0028, 32'h2222_2222);  // miss, allocate, write
        cpu_access(0, 32'h0000_0028, 0);              // hit
        cpu_access(0, 32'h0000_0020, 0);              // hit, rest of block intact

        //--------------------------------------------------------------
        $display("\n--- Test 4: conflict miss with dirty victim (write-back) ---");
        wb_snap = mem_writes;
        cpu_access(0, 32'h0000_0104, 0);   // index 0, new tag -> evict dirty line
        check_writebacks(wb_snap, 1);
        cpu_access(0, 32'h0000_0004, 0);   // miss, must return 0x11111111 from memory
        check_writebacks(wb_snap, 1);      // victim (tag 1) was clean

        //--------------------------------------------------------------
        $display("\n--- Test 5: conflict miss with clean victim (no write-back) ---");
        wb_snap = mem_writes;
        cpu_access(0, 32'h0000_0210, 0);   // index 1, evict clean line
        cpu_access(0, 32'h0000_0010, 0);   // miss again
        check_writebacks(wb_snap, 0);

        //--------------------------------------------------------------
        $display("\n--- Test 6: fill whole cache, then re-read (all hits) ---");
        verbose = 1'b0;
        for (i = 0; i < NUM_LINES; i = i + 1)
            cpu_access(0, 32'h0000_0400 + i*16, 0);
        for (i = 0; i < NUM_LINES; i = i + 1)
            cpu_access(0, 32'h0000_0400 + i*16 + 12, 0);
        $display("  fill/re-read done: hits so far = %0d, misses so far = %0d",
                 n_hits, n_misses);

        //--------------------------------------------------------------
        $display("\n--- Test 7: thrash two blocks on the same index ---");
        verbose = 1'b1;
        wb_snap = mem_writes;
        cpu_access(1, 32'h0000_0A30, 32'hAAAA_0001);  // miss
        cpu_access(1, 32'h0000_0B30, 32'hBBBB_0001);  // miss, write back A
        cpu_access(0, 32'h0000_0A30, 0);              // miss, write back B
        cpu_access(0, 32'h0000_0B30, 0);              // miss, A clean
        check_writebacks(wb_snap, 2);

        //--------------------------------------------------------------
        $display("\n--- Test 8: %0d random reads/writes ---", NUM_RANDOM);
        verbose = 1'b0;
        for (i = 0; i < NUM_RANDOM; i = i + 1) begin
            r_we   = $random(seed);
            r_data = $random(seed);
            if (($random(seed) & 3) != 0)
                r_addr = $random(seed) & 32'h0000_03FC;   // 1 KB hot region
            else
                r_addr = $random(seed) & 32'h0000_3FFC;   // full 16 KB
            cpu_access(r_we, r_addr, r_data);
        end

        //--------------------------------------------------------------
        repeat (5) @(posedge clk);
        $display("\n==================================================");
        $display("  Checks       : %0d", checks);
        $display("  Hits         : %0d", n_hits);
        $display("  Misses       : %0d", n_misses);
        $display("  Hit rate     : %0d%%", (n_hits * 100) / (n_hits + n_misses));
        $display("  Mem reads    : %0d", mem_reads);
        $display("  Mem writes   : %0d (write-backs)", mem_writes);
        $display("  Errors       : %0d", errors);
        if (errors == 0)
            $display("  *** TEST PASSED ***");
        else
            $display("  *** TEST FAILED ***");
        $display("==================================================\n");
        $finish;
    end

    // Watchdog
    initial begin
        #5_000_000;
        $display("ERROR: global timeout");
        $finish;
    end

endmodule
