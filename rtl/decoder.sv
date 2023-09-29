module decoder #(
    // Parameters
    parameter ALUOP_WIDTH = 5,
    parameter TYPE_WIDTH = 3,
    parameter DTYPE_WIDTH = 2,
    parameter BRANCH_TYPE_WIDTH = 3,
    parameter ALUSELECT_WIDTH = 2,
    parameter DATA_WIDTH = 32
)(
    // Ports
    input wire [DATA_WIDTH-1:0] instr,      //Current Full Instruction from ROM

    output reg [ALUOP_WIDTH-1:0] ALUOp,             // ALU Operation
    output reg [TYPE_WIDTH-1:0] rawType,            // Instruction Type
    output reg load,                                // for loading data from LSU to register file or 
                                                    //just write output of ALU to register file (1/0 respectively)
    output reg [DTYPE_WIDTH-1:0] dType,             // Data type for LSU
    output reg MWE,                                 // WE for LSU
    output reg RWE,                                 // WE for register file
    output reg [BRANCH_TYPE_WIDTH-1:0] branchType,   // Branch unit
    output reg jump,                                // muxes for PC and register file data_in 
    output reg [ALUSELECT_WIDTH-1:0] ALUSelect      // mux the output of IALU, FALU, and CORDIC

        //note: ALUSelect[1] will be appended in front of the rs1, rs2, rd, and dataIn 
        //for the register file that has 32 registers for integer and 32 registers for float
        
        //  11  select CORDIC   (1 in front will mean use float registers)
        //  10  select FALU     (1 in front will mean use float registers)
        //  01  select IALU     
        //  00  (not used currently)

        
);
    // Instruction Types
    `define R_TYPE 3'b000
    `define I_TYPE 3'b001
    `define S_TYPE 3'b010
    `define B_TYPE 3'b011
    `define U_TYPE 3'b100
    `define J_TYPE 3'b101

    // Branch Types
    `define BEQ  3'd0
    `define BNE  3'd1
    `define BLT  3'd2
    `define BGE  3'd3
    `define BLTU 3'd4
    `define BGEU 3'd5

    //--------------------------------------------------------------------
    // ALU Operations
    //--------------------------------------------------------------------
    `define ALU_NONE                                5'd00
    `define ALU_SHIFTL                              5'd01
    `define ALU_SHIFTR                              5'd02
    `define ALU_SHIFTR_ARITH                        5'd03
    `define ALU_ADD                                 5'd04
    `define ALU_SUB                                 5'd05
    `define ALU_AND                                 5'd06
    `define ALU_OR                                  5'd07
    `define ALU_XOR                                 5'd08
    `define ALU_LESS_THAN                           5'd09
    `define ALU_LESS_THAN_SIGNED                    5'd10
    `define ALU_MUL                                 5'd11
    `define ALU_MULH                                5'd12
    `define ALU_MULHSU                              5'd13
    `define ALU_MULHU                               5'd14
    `define ALU_DIV                                 5'd15
    `define ALU_DIVU                                5'd16
    `define ALU_REM                                 5'd17
    `define ALU_REMU                                5'd18

    //opcodes and funct3 and funct7
    `define LUI                         7'b0110111
    `define AUIPC                       7'b0010111
    `define JAL                         7'b1101111
    `define JALR                        7'b1100111

    `define op_BranchGeneral            7'b1100011
    `define funct3_BEQ  3'b000
    `define funct3_BNE  3'b001
    `define funct3_BLT  3'b100
    `define funct3_BGE  3'b101
    `define funct3_BLTU 3'b110
    `define funct3_BGEU 3'b111

    `define op_load                     7'b0000011
    `define funct3_LB 3'b000
    `define funct3_LH 3'b001
    `define funct3_LW 3'b010
    `define funct3_LBU 3'b100
    `define funct3_LHU 3'b101
    

    `define op_store                    7'b0100011
    `define funct3_SB 3'b000
    `define funct3_SH 3'b001
    `define funct3_SW 3'b010
    

    `define op_basicImmediateOps        7'b0010011
    `define funct3_ADDI      3'b000
    `define funct3_SLTI      3'b010
    `define funct3_SLTIU     3'b011
    `define funct3_XORI      3'b100
    `define funct3_ORI       3'b110
    `define funct3_ANDI      3'b111

    `define funct3_SLLI      3'b001
    
    `define funct3_SRLI      3'b101
    `define funct7_SRLI      7'b0000000

    `define funct3_SRAI      3'b101
    `define funct7_SRAI      7'b0100000


    `define op_basicOps                 7'b0110011
    `define funct3_ADD       3'b000
    
    `define funct3_SUB       3'b000
    `define funct7_SUB       7'b0100000

    `define funct3_SLL       3'b001
    `define funct3_SLT       3'b010
    `define funct3_SLTU      3'b011
    `define funct3_XOR       3'b100
    `define funct3_SRL       3'b101
    
    `define funct3_SRA       3'b101
    `define funct7_SRA       7'b0100000

    `define funct3_OR        3'b110
    `define funct3_AND       3'b111

    wire [6:0] opcode;

    assign opcode = instr[6:0];
    

    //determine instruction type
    always @(instr)
    begin 
        case (opcode)
            

            default: 
        endcase
    end
endmodule;