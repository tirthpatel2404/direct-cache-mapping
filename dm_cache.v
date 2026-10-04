//============================================================================
// File        : dm_cache.v
// Description : Direct-mapped cache controller (write-back, write-allocate)
//
//   Address breakdown (defaults: 32-bit addr, 16 lines, 4 words/line):
//
//     31                    8 7      4 3       2 1     0
//    +-----------------------+--------+---------+-------+
//    |          TAG          | INDEX  | WORD OFF| BYTE  |
//    +-----------------------+--------+---------+-------+
//          24 bits             4 bits   2 bits   2 bits
//
//   Per line storage : valid bit, dirty bit, tag, data block
//
//   FSM:
//     IDLE      -> wait for cpu_req, latch the request
//     COMPARE   -> tag compare; hit = valid && (tag match)
//                  hit  : serve read / update word on write, back to IDLE
//                  miss : dirty victim -> WRITEBACK, else -> ALLOCATE
//     WRITEBACK -> write dirty victim block to memory
//     ALLOCATE  -> fetch new block from memory, then re-run COMPARE
//
//   CPU protocol : pulse cpu_req for one cycle while cpu_busy == 0,
//                  wait for the one-cycle cpu_ready pulse.
//   Mem protocol : hold mem_req until a one-cycle mem_ready pulse.
//
//   Notes:
//     - Word-aligned accesses only (byte offset bits are ignored).
//     - WORDS_PER_LINE must be >= 2 and NUM_LINES must be a power of 2.
//     - hit / miss are one-cycle pulses, reported once per CPU request
//       (the re-compare after a refill does not raise hit).
//============================================================================
`timescale 1ns/1ps

module dm_cache #(
    parameter ADDR_W         = 32,
    parameter DATA_W         = 32,
    parameter NUM_LINES      = 16,
    parameter WORDS_PER_LINE = 4
)(
    input  wire                             clk,
    input  wire                             rst_n,

    // ---------------- CPU side ----------------
    input  wire                             cpu_req,
    input  wire                             cpu_we,
    input  wire [ADDR_W-1:0]                cpu_addr,
    input  wire [DATA_W-1:0]                cpu_wdata,
    output reg  [DATA_W-1:0]                cpu_rdata,
    output reg                              cpu_ready,
    output wire                             cpu_busy,
    output reg                              hit,
    output reg                              miss,

    // ---------------- Memory side -------------
    output reg                              mem_req,
    output reg                              mem_we,
    output reg  [ADDR_W-1:0]                mem_addr,
    output reg  [DATA_W*WORDS_PER_LINE-1:0] mem_wdata,
    input  wire [DATA_W*WORDS_PER_LINE-1:0] mem_rdata,
    input  wire                             mem_ready
);

    //------------------------------------------------------------------
    // Derived parameters
    //------------------------------------------------------------------
    localparam LINE_W     = DATA_W * WORDS_PER_LINE;
    localparam BYTE_OFF_W = $clog2(DATA_W/8);
    localparam WORD_OFF_W = $clog2(WORDS_PER_LINE);
    localparam OFFSET_W   = BYTE_OFF_W + WORD_OFF_W;
    localparam INDEX_W    = $clog2(NUM_LINES);
    localparam TAG_W      = ADDR_W - INDEX_W - OFFSET_W;

    //------------------------------------------------------------------
    // FSM encoding
    //------------------------------------------------------------------
    localparam [1:0] S_IDLE      = 2'd0,
                     S_COMPARE   = 2'd1,
                     S_WRITEBACK = 2'd2,
                     S_ALLOCATE  = 2'd3;

    reg [1:0] state, next_state;

    //------------------------------------------------------------------
    // Cache storage
    //------------------------------------------------------------------
    reg [TAG_W-1:0]   tag_arr  [0:NUM_LINES-1];
    reg [LINE_W-1:0]  data_arr [0:NUM_LINES-1];
    reg [NUM_LINES-1:0] valid;
    reg [NUM_LINES-1:0] dirty;

    //------------------------------------------------------------------
    // Latched CPU request
    //------------------------------------------------------------------
    reg              req_we;
    reg [ADDR_W-1:0] req_addr;
    reg [DATA_W-1:0] req_wdata;
    reg              missed;     // this request already reported a miss

    //------------------------------------------------------------------
    // Address decode: tag / index / offset
    //------------------------------------------------------------------
    wire [TAG_W-1:0]      req_tag  = req_addr[ADDR_W-1 -: TAG_W];
    wire [INDEX_W-1:0]    req_idx  = req_addr[OFFSET_W +: INDEX_W];
    wire [WORD_OFF_W-1:0] req_word = req_addr[BYTE_OFF_W +: WORD_OFF_W];

    //------------------------------------------------------------------
    // Lookup and hit/miss logic
    //------------------------------------------------------------------
    wire [TAG_W-1:0]  line_tag   = tag_arr[req_idx];
    wire [LINE_W-1:0] line_data  = data_arr[req_idx];
    wire              line_valid = valid[req_idx];
    wire              line_dirty = dirty[req_idx];

    wire              tag_match  = line_valid && (line_tag == req_tag);
    wire [DATA_W-1:0] rd_word    = line_data[req_word*DATA_W +: DATA_W];

    // Block with the CPU write data merged into the addressed word
    reg [LINE_W-1:0] merged_line;
    always @(*) begin
        merged_line = line_data;
        merged_line[req_word*DATA_W +: DATA_W] = req_wdata;
    end

    assign cpu_busy = (state != S_IDLE);

    //------------------------------------------------------------------
    // Next-state and memory-interface logic (combinational)
    //------------------------------------------------------------------
    always @(*) begin
        next_state = state;
        mem_req    = 1'b0;
        mem_we     = 1'b0;
        mem_addr   = {ADDR_W{1'b0}};
        mem_wdata  = line_data;

        case (state)
            S_IDLE: begin
                if (cpu_req)
                    next_state = S_COMPARE;
            end

            S_COMPARE: begin
                if (tag_match)
                    next_state = S_IDLE;
                else if (line_valid && line_dirty)
                    next_state = S_WRITEBACK;
                else
                    next_state = S_ALLOCATE;
            end

            S_WRITEBACK: begin
                mem_req  = 1'b1;
                mem_we   = 1'b1;
                mem_addr = {line_tag, req_idx, {OFFSET_W{1'b0}}};  // victim address
                if (mem_ready)
                    next_state = S_ALLOCATE;
            end

            S_ALLOCATE: begin
                mem_req  = 1'b1;
                mem_we   = 1'b0;
                mem_addr = {req_tag, req_idx, {OFFSET_W{1'b0}}};   // new block address
                if (mem_ready)
                    next_state = S_COMPARE;
            end

            default: next_state = S_IDLE;
        endcase
    end

    //------------------------------------------------------------------
    // State, control bits and CPU outputs (reset)
    //------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= S_IDLE;
            valid     <= {NUM_LINES{1'b0}};
            dirty     <= {NUM_LINES{1'b0}};
            req_we    <= 1'b0;
            req_addr  <= {ADDR_W{1'b0}};
            req_wdata <= {DATA_W{1'b0}};
            missed    <= 1'b0;
            cpu_rdata <= {DATA_W{1'b0}};
            cpu_ready <= 1'b0;
            hit       <= 1'b0;
            miss      <= 1'b0;
        end else begin
            state     <= next_state;
            cpu_ready <= 1'b0;
            hit       <= 1'b0;
            miss      <= 1'b0;

            case (state)
                S_IDLE: begin
                    if (cpu_req) begin
                        req_we    <= cpu_we;
                        req_addr  <= cpu_addr;
                        req_wdata <= cpu_wdata;
                        missed    <= 1'b0;
                    end
                end

                S_COMPARE: begin
                    if (tag_match) begin
                        cpu_ready <= 1'b1;
                        hit       <= ~missed;
                        if (req_we)
                            dirty[req_idx] <= 1'b1;
                        else
                            cpu_rdata <= rd_word;
                    end else begin
                        miss   <= ~missed;
                        missed <= 1'b1;
                    end
                end

                S_ALLOCATE: begin
                    if (mem_ready) begin
                        valid[req_idx] <= 1'b1;
                        dirty[req_idx] <= 1'b0;
                    end
                end

                default: ;
            endcase
        end
    end

    //------------------------------------------------------------------
    // Tag / data arrays (no reset -> maps cleanly to RAM)
    //------------------------------------------------------------------
    always @(posedge clk) begin
        if (state == S_COMPARE && tag_match && req_we) begin
            data_arr[req_idx] <= merged_line;               // write hit
        end else if (state == S_ALLOCATE && mem_ready) begin
            data_arr[req_idx] <= mem_rdata;                 // refill
            tag_arr[req_idx]  <= req_tag;
        end
    end

endmodule
