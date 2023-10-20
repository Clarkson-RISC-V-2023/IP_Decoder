`include "../params/riscv_instr.sv"
module tb_decoder #(
    // Parameters
    parameter ALUOP_WIDTH = 5,
    parameter TYPE_WIDTH = 3,
    parameter DTYPE_WIDTH = 3,
    parameter BRANCH_TYPE_WIDTH = 3,
    parameter ALUSELECT_WIDTH = 2,
    parameter DATA_WIDTH = 32,
    
    parameter OPCODE_WIDTH = 7,
    parameter FUNCT3_WIDTH = 3,
    parameter FUNCT7_WIDTH = 7,
    parameter FUNCT3_RIGHT = 12,
    parameter FUNCT3_LEFT = FUNCT3_WIDTH-1+FUNCT3_RIGHT
);


reg [DATA_WIDTH-1:0] instr;
wire [ALUOP_WIDTH-1:0] ALUOp;
wire [TYPE_WIDTH-1:0] rawType;
wire load;
wire [DTYPE_WIDTH-1:0] dType;
wire MWE;
wire RWE;
wire [BRANCH_TYPE_WIDTH-1:0] branchType;
wire jump;
wire [ALUSELECT_WIDTH-1:0] ALUSelect;
wire auipcBit;
wire f_rd;
wire f_d1;
wire f_d2;


decoder #(
    .ALUOP_WIDTH(ALUOP_WIDTH),
    .TYPE_WIDTH(TYPE_WIDTH),
    .DTYPE_WIDTH(DTYPE_WIDTH),
    .BRANCH_TYPE_WIDTH(BRANCH_TYPE_WIDTH),
    .ALUSELECT_WIDTH(ALUSELECT_WIDTH),
    .DATA_WIDTH(DATA_WIDTH),
    .OPCODE_WIDTH(OPCODE_WIDTH),
    .FUNCT3_WIDTH(FUNCT3_WIDTH),
    .FUNCT7_WIDTH(FUNCT7_WIDTH),
    .FUNCT3_RIGHT(FUNCT3_RIGHT),
    .FUNCT3_LEFT(FUNCT3_LEFT)
) dut_decoder (
    .instr(instr),
    .ALUOp(ALUOp),
    .rawType(rawType),
    .load(load),
    .dType(dType),
    .MWE(MWE),
    .RWE(RWE),
    .branchType(branchType),
    .jump(jump),
    .ALUSelect(ALUSelect),
    .auipcBit(auipcBit),
    .f_rd(f_rd),
    .f_d1(f_d1),
    .f_d2(f_d2)
);

    initial begin 
        // Logic definition
        $dumpfile("tb_decoder.vcd");
        $dumpvars(0, tb_decoder);

        instr <= '0; //nop

        //check " SLT[FUNCT3_LEFT:FUNCT3_RIGHT]:   ALUOp <= `ALU_LESS_THAN_SIGNED;   " 
        #100 instr <= SLT;

        //check FLT_S
        #100 instr <= FLT_S;

        //check FMV_W_X
        #100 instr <= FMV_W_X;
        
        #100 instr <= '0; //nop
        #100 $finish;
    end
endmodule