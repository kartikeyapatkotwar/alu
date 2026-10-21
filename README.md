// 4-stage pipelined processor
// Stages : IF  (Instruction Fetch)
//          ID  (Instruction Decode / Register Read)
//          EX  (Execute - ALU / address calc)
//          MW  (Memory access + Write Back, combined into one stage)
//
// Supported instructions : ADD, SUB, AND, LOAD   (no branches -> no control
// hazards; only data hazards, handled by EX-stage forwarding from the MW
// stage plus the regfile's natural "write-before-read" behaviour for
// 2-instruction-back hazards)
//
// Instruction format (16 bits)
//   R-type (ADD/SUB/AND):  [15:13]op [12:10]rd [9:7]rs1 [6:4]rs2 [3:0]unused
//   I-type (LOAD)       :  [15:13]op [12:10]rd [9:7]rs1 [6:0]imm7
//
// Opcodes
//   000 ADD   rd = rs1 + rs2
//   001 SUB   rd = rs1 - rs2
//   010 AND   rd = rs1 & rs2
//   011 LOAD  rd = DMEM[rs1 + imm7]
//   111 NOP   no operation
//==============================================================================
`timescale 1ns / 1ps
 
module cpu_pipeline (
    input wire clk,
    input wire rst_n
);
 
    localparam [2:0] OP_ADD  = 3'b000,
                      OP_SUB  = 3'b001,
                      OP_AND  = 3'b010,
                      OP_LOAD = 3'b011,
                      OP_NOP  = 3'b111;
 
    //--------------------------------------------------------------------
    // Program counter + Instruction memory (16 x 16-bit demo ROM)
    //--------------------------------------------------------------------
    reg [3:0]  pc;
    reg [15:0] imem [0:15];
 
    function [15:0] rtype_i;
        input [2:0] op; input [2:0] rd; input [2:0] rs1; input [2:0] rs2;
        begin rtype_i = {op, rd, rs1, rs2, 4'b0000}; end
    endfunction
    function [15:0] itype_i;
        input [2:0] op; input [2:0] rd; input [2:0] rs1; input [6:0] imm;
        begin itype_i = {op, rd, rs1, imm}; end
    endfunction
 
    integer p;
    initial begin
        // Demo program (see report for the annotated listing / expected values)
        imem[0]  = itype_i(OP_LOAD, 3'd1, 3'd0, 7'd0);   // LOAD R1, 0(R0)
        imem[1]  = itype_i(OP_LOAD, 3'd2, 3'd0, 7'd1);   // LOAD R2, 1(R0)
        imem[2]  = rtype_i(OP_ADD,  3'd3, 3'd1, 3'd2);   // ADD  R3, R1, R2   (uses R2 from previous instr -> forward)
        imem[3]  = rtype_i(OP_SUB,  3'd4, 3'd3, 3'd1);   // SUB  R4, R3, R1   (uses R3 from previous instr -> forward)
        imem[4]  = rtype_i(OP_AND,  3'd5, 3'd3, 3'd2);   // AND  R5, R3, R2   (R3 is 2 instrs back -> regfile)
        imem[5]  = itype_i(OP_LOAD, 3'd6, 3'd0, 7'd2);   // LOAD R6, 2(R0)
        imem[6]  = rtype_i(OP_ADD,  3'd7, 3'd6, 3'd5);   // ADD  R7, R6, R5   (R6 is load-use, immediately before -> forward)
        for (p = 7; p < 16; p = p + 1)
            imem[p] = itype_i(OP_NOP, 3'd0, 3'd0, 7'd0); // NOP flush
    end
 
    //--------------------------------------------------------------------
    // Register file (8 x 8-bit). Synchronous write / combinational read,
    // with an explicit write-through bypass on the read ports: if the
    // instruction currently in MW is writing the same register an ID-stage
    // read wants this same cycle, forward MW's result straight to the read
    // rather than the (not-yet-updated) stored value. This is what resolves
    // the "2-instructions-back" RAW hazard (e.g. ADD R3,R1,R2 reading R1,
    // which was produced by the instruction 2 slots earlier and is in MW
    // the same cycle ADD is in ID).
    //--------------------------------------------------------------------
    reg [7:0] regs [0:7];
    integer r;
    initial for (r = 0; r < 8; r = r + 1) regs[r] = 8'd0;
 
    wire [2:0] mw_rd;
    wire       mw_regwrite;
    wire [7:0] mw_result;
 
    always @(posedge clk) begin
        if (mw_regwrite)
            regs[mw_rd] <= mw_result;
    end
 
    //--------------------------------------------------------------------
    // Data memory (demo preload: DMEM[0]=10, DMEM[1]=20, DMEM[2]=5)
    //--------------------------------------------------------------------
    reg [7:0] dmem [0:127];
    integer d;
    initial begin
        for (d = 0; d < 128; d = d + 1) dmem[d] = 8'd0;
        dmem[0] = 8'd10;
        dmem[1] = 8'd20;
        dmem[2] = 8'd5;
    end
 
    //======================================================================
    // STAGE 1 : IF
    //======================================================================
    reg [15:0] ifid_instr;
    reg [3:0]  ifid_pc;
 
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pc         <= 4'd0;
            ifid_instr <= itype_i(OP_NOP, 0, 0, 0);
            ifid_pc    <= 4'd0;
        end else begin
            ifid_instr <= imem[pc];
            ifid_pc    <= pc;
            pc         <= pc + 1'b1;
        end
    end
 
    //======================================================================
    // STAGE 2 : ID  (decode + register read)
    //======================================================================
    wire [2:0] id_op   = ifid_instr[15:13];
    wire [2:0] id_rd   = ifid_instr[12:10];
    wire [2:0] id_rs1  = ifid_instr[9:7];
    wire [2:0] id_rs2  = ifid_instr[6:4];
    wire [6:0] id_imm  = ifid_instr[6:0];
 
    wire id_bypass1 = mw_regwrite && (mw_rd == id_rs1) && (mw_rd != 0);
    wire id_bypass2 = mw_regwrite && (mw_rd == id_rs2) && (mw_rd != 0);
 
    wire [7:0] id_rs1data = id_bypass1 ? mw_result : regs[id_rs1];
    wire [7:0] id_rs2data = id_bypass2 ? mw_result : regs[id_rs2];
 
    wire id_regwrite = (id_op == OP_ADD) || (id_op == OP_SUB) ||
                        (id_op == OP_AND) || (id_op == OP_LOAD);
    wire id_memread  = (id_op == OP_LOAD);
 
    reg [3:0] idex_pc;
    reg [2:0] idex_op, idex_rd, idex_rs1, idex_rs2;
    reg [7:0] idex_rs1data, idex_rs2data;
    reg [6:0] idex_imm;
    reg       idex_regwrite, idex_memread;
 
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            idex_pc <= 0; idex_op <= OP_NOP; idex_rd <= 0; idex_rs1 <= 0; idex_rs2 <= 0;
            idex_rs1data <= 0; idex_rs2data <= 0; idex_imm <= 0;
            idex_regwrite <= 0; idex_memread <= 0;
        end else begin
            idex_pc       <= ifid_pc;
            idex_op       <= id_op;
            idex_rd       <= id_rd;
            idex_rs1      <= id_rs1;
            idex_rs2      <= id_rs2;
            idex_rs1data  <= id_rs1data;
            idex_rs2data  <= id_rs2data;
            idex_imm      <= id_imm;
            idex_regwrite <= id_regwrite;
            idex_memread  <= id_memread;
        end
    end
 
    //======================================================================
    // STAGE 3 : EX  (ALU / address calc, with forwarding from MW stage)
    //======================================================================
    // Forward rs1 / rs2 from the instruction currently in MW if it will
    // write the same register this cycle (classic single-hop EX<-MW forward;
    // the 2-instruction-back case is already handled by the regfile itself).
    wire fwd1 = mw_regwrite && (mw_rd == idex_rs1) && (mw_rd != 0);
    wire fwd2 = mw_regwrite && (mw_rd == idex_rs2) && (mw_rd != 0);
 
    wire [7:0] ex_in1 = fwd1 ? mw_result : idex_rs1data;
    wire [7:0] ex_in2 = fwd2 ? mw_result : idex_rs2data;
 
    reg [7:0] ex_alu_result;
    always @(*) begin
        case (idex_op)
            OP_ADD : ex_alu_result = ex_in1 + ex_in2;
            OP_SUB : ex_alu_result = ex_in1 - ex_in2;
            OP_AND : ex_alu_result = ex_in1 & ex_in2;
            OP_LOAD: ex_alu_result = ex_in1 + {1'b0, idex_imm}; // base + offset
            default: ex_alu_result = 8'd0;
        endcase
    end
 
    reg [3:0] exmw_pc;
    reg [2:0] exmw_op, exmw_rd;
    reg [7:0] exmw_alu_result;
    reg       exmw_regwrite, exmw_memread;
 
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            exmw_pc <= 0; exmw_op <= OP_NOP; exmw_rd <= 0;
            exmw_alu_result <= 0; exmw_regwrite <= 0; exmw_memread <= 0;
        end else begin
            exmw_pc         <= idex_pc;
            exmw_op         <= idex_op;
            exmw_rd         <= idex_rd;
            exmw_alu_result <= ex_alu_result;
            exmw_regwrite   <= idex_regwrite;
            exmw_memread    <= idex_memread;
        end
    end
 
    //======================================================================
    // STAGE 4 : MW  (Memory access + Write Back)
    //======================================================================
    wire [7:0] mw_memdata = dmem[exmw_alu_result];
 
    assign mw_rd       = exmw_rd;
    assign mw_regwrite = exmw_regwrite;
    assign mw_result   = exmw_memread ? mw_memdata : exmw_alu_result;
 
    //--------------------------------------------------------------------
    // Debug: human-readable mnemonic per stage (simulation only)
    //--------------------------------------------------------------------
    function [31:0] mnemonic;
        input [2:0] op;
        begin
            case (op)
                OP_ADD : mnemonic = "ADD ";
                OP_SUB : mnemonic = "SUB ";
                OP_AND : mnemonic = "AND ";
                OP_LOAD: mnemonic = "LOAD";
                default: mnemonic = "NOP ";
            endcase
        end
    endfunction
 
endmodule
