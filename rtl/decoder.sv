`include "../params/riscv_instr.sv"
module decoder #(
    // Parameters
    parameter ALUOP_WIDTH = 5,
    parameter TYPE_WIDTH = 3,
    parameter DTYPE_WIDTH = 3,
    parameter BRANCH_TYPE_WIDTH = 3,
    parameter ALUSELECT_WIDTH = 2,
    parameter DATA_WIDTH = 32,
    
    parameter OPCODE_WIDTH = 7,
    parameter FUNCT3_WIDTH = 3,
    parameter FUNCT7_WIDTH = 7
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
    output reg [ALUSELECT_WIDTH-1:0] ALUSelect,      // mux the output of IALU, FALU, and CORDIC
    output reg auipcBit                              //just for auipc instruction

        //note: ALUSelect[1] will be appended in front of the rs1, rs2, rd, and dataIn 
        //for the register file that has 32 registers for integer and 32 registers for float
        
        //  11  select CORDIC   (1 in front will mean use float registers)
        //  10  select FALU     (1 in front will mean use float registers)
        //  01  select IALU     
        //  00  (not used currently)

        
);
    //import riscv_instr::*;
    //ALUSelect
    `define SEL_CORDIC  2'b11
    `define SEL_FALU    2'b10
    `define SEL_IALU    2'b01
    `define SEL_NONE    2'b00

    // Defining the different DTYPES
    `define BYTE               3'b000
    `define HALF_WORD          3'b001
    `define FULL_WORD          3'b010
    `define BYTE_UNSIGNED      3'b011
    `define HALF_WORD_UNSIGNED 3'b100

    // Instruction Types (rawType)
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
    
    `define FALU_ADD            5'd0
    `define FALU_SUB            5'd1
    `define FALU_MUL            5'd2
    `define FALU_DIV            5'd3
    `define FALU_SQRT           5'd4
    `define FALU_SGNJ           5'd5
    `define FALU_SGNJN          5'd6
    `define FALU_SGNJX          5'd7
    `define FALU_MIN            5'd8
    `define FALU_MAX            5'd9
    `define FALU_EQ             5'd10
    `define FALU_LT             5'd11
    `define FALU_LE             5'd12


    wire [OPCODE_WIDTH-1:0] opcode;
    wire [FUNCT3_WIDTH-1:0] funct3;
    wire [FUNCT7_WIDTH-1:0] funct7;
    assign opcode = instr[OPCODE_WIDTH-1:0];
    assign funct3 = instr[FUNCT3_WIDTH-1+12:12];
    assign funct7 = instr[DATA_WIDTH-1:DATA_WIDTH-FUNCT7_WIDTH];

    //determine instruction type
    always @(instr)
    begin 
        case (opcode)
            LUI[OPCODE_WIDTH-1:0]: begin     //load 
                ALUOp       <= `ALU_NONE;  
                ALUSelect   <= `SEL_IALU;              
                branchType  <= '0;          //branching?
                dType       <= '0;          //write to LSU? -> data type?               
                MWE         <= '0;          //write to LSU? 
                RWE         <= '1;          //write to reg file?               
                jump        <= '0;          //jumping?
                load        <= '0;          //take data from LSU?
                auipcBit    <= '0;          
                rawType     <= `U_TYPE;     
            end
            AUIPC[OPCODE_WIDTH-1:0]: begin
                ALUOp       <= `ALU_ADD;  
                ALUSelect   <= `SEL_IALU;                              
                dType       <= '0;  //write to LSU? -> data type?               
                MWE         <= '0;  //write to LSU? 
                RWE         <= '1;  //write to reg file?               
                jump        <= '0;  //jumping?
                branchType  <= '0;  //branching?
                load        <= '0;  //take data from LSU?
                auipcBit    <= '1;
                rawType     <= `U_TYPE;
            end
            ADDI[OPCODE_WIDTH-1:0]: begin  //generic
                 
                ALUSelect   <= `SEL_IALU;                              
                dType       <= '0;  //write to LSU? -> data type?               
                MWE         <= '0;  //write to LSU? 
                RWE         <= '1;  //write to reg file?               
                jump        <= '0;  //jumping?
                branchType  <= '0;  //branching?
                load        <= '0;  //take data from LSU?
                auipcBit    <= '0;
                rawType     <= `I_TYPE;

                case (funct3)
                ADDI[FUNCT3_WIDTH+11:12]:   ALUOp <= `ALU_ADD;                    
                SLTI[FUNCT3_WIDTH+11:12]:   ALUOp <= `ALU_LESS_THAN_SIGNED;                
                SLTIU[FUNCT3_WIDTH+11:12]:  ALUOp <= `ALU_LESS_THAN;
                XORI[FUNCT3_WIDTH+11:12]:   ALUOp <= `ALU_XOR;
                ORI[FUNCT3_WIDTH+11:12]:   ALUOp <= `ALU_OR;
                ANDI[FUNCT3_WIDTH+11:12]:   ALUOp <= `ALU_AND;
                SLLI[FUNCT3_WIDTH+11:12]:   ALUOp <= `ALU_SHIFTL;
                SRLI[FUNCT3_WIDTH+11:12]:     
                begin 
                    case(funct7)
                        SRLI[DATA_WIDTH-1:DATA_WIDTH-FUNCT7_WIDTH]: ALUOp <= `ALU_SHIFTR;
                        SRAI[DATA_WIDTH-1:DATA_WIDTH-FUNCT7_WIDTH]: ALUOp <= `ALU_SHIFTR_ARITH;
                        default: ALUOp <= `ALU_NONE;
                    endcase
                end
                default: ALUOp <= `ALU_NONE;
                endcase
            end
            ADD[OPCODE_WIDTH-1:0]: begin  //generic
                 
                ALUSelect   <= `SEL_IALU;                              
                dType       <= '0;  //write to LSU? -> data type?               
                MWE         <= '0;  //write to LSU? 
                RWE         <= '1;  //write to reg file?               
                jump        <= '0;  //jumping?
                branchType  <= '0;  //branching?
                load        <= '0;  //take data from LSU?
                auipcBit    <= '0;
                rawType     <= `R_TYPE;

                case (funct3)
                ADD[FUNCT3_WIDTH+11:12]:   
                begin 
                    case(funct7)
                        ADD[DATA_WIDTH-1:DATA_WIDTH-FUNCT7_WIDTH]: ALUOp <= `ALU_ADD;  
                        SUB[DATA_WIDTH-1:DATA_WIDTH-FUNCT7_WIDTH]: ALUOp <= `ALU_SUB;
                        default: ALUOp <= `ALU_NONE;
                    endcase
                end                
                SLT[FUNCT3_WIDTH+11:12]:   ALUOp <= `ALU_LESS_THAN_SIGNED;                
                SLTU[FUNCT3_WIDTH+11:12]:  ALUOp <= `ALU_LESS_THAN;
                XOR[FUNCT3_WIDTH+11:12]:   ALUOp <= `ALU_XOR;
                OR[FUNCT3_WIDTH+11:12]:   ALUOp <= `ALU_OR;
                AND[FUNCT3_WIDTH+11:12]:   ALUOp <= `ALU_AND;
                SLL[FUNCT3_WIDTH+11:12]:   ALUOp <= `ALU_SHIFTL;
                SRL[FUNCT3_WIDTH+11:12]:   //both instructions have the same funct3  
                begin
                    case(funct7)                         
                        SRL[DATA_WIDTH-1:DATA_WIDTH-FUNCT7_WIDTH]: ALUOp <= `ALU_SHIFTR;
                        SRA[DATA_WIDTH-1:DATA_WIDTH-FUNCT7_WIDTH]: ALUOp <= `ALU_SHIFTR_ARITH;
                        default: ALUOp <= `ALU_NONE;
                    endcase
                end
                default: ALUOp <= `ALU_NONE;
                endcase
            end
            LB[OPCODE_WIDTH-1:0]: begin  //generic
                ALUOp       <= `ALU_ADD;
                ALUSelect   <= `SEL_IALU;                              
                MWE         <= '0;  //write to LSU? 
                RWE         <= '1;  //write to reg file?               
                jump        <= '0;  //jumping?
                branchType  <= '0;  //branching?
                load        <= '1;  //take data from LSU?
                auipcBit    <= '0;
                rawType     <= `I_TYPE;

                case(funct3)
                LB[FUNCT3_WIDTH+11:12]: dType <= `BYTE;  
                LH[FUNCT3_WIDTH+11:12]: dType <= `HALF_WORD;
                LW[FUNCT3_WIDTH+11:12]: dType <= `FULL_WORD;
                LBU[FUNCT3_WIDTH+11:12]: dType <= `BYTE_UNSIGNED;
                LHU[FUNCT3_WIDTH+11:12]: dType <= `HALF_WORD_UNSIGNED;
                endcase
            end
            SB[OPCODE_WIDTH-1:0]: begin  //generic
                ALUOp       <= `ALU_ADD;
                ALUSelect   <= `SEL_IALU;                              
                MWE         <= '1;  //write to LSU? 
                RWE         <= '0;  //write to reg file?               
                jump        <= '0;  //jumping?
                branchType  <= '0;  //branching?
                load        <= '0;  //take data from LSU?
                auipcBit    <= '0;
                rawType     <= `S_TYPE;

                case(funct3)
                SB[FUNCT3_WIDTH+11:12]: dType <= `BYTE;  
                SH[FUNCT3_WIDTH+11:12]: dType <= `HALF_WORD;
                SW[FUNCT3_WIDTH+11:12]: dType <= `FULL_WORD;
                endcase
            end
            BEQ[OPCODE_WIDTH-1:0]: begin  //generic
                ALUOp       <= `ALU_NONE;
                ALUSelect   <= `SEL_NONE;    
                dType       <= '0;                          
                MWE         <= '0;  //write to LSU? 
                RWE         <= '0;  //write to reg file?               
                jump        <= '0;  //jumping?
                
                load        <= '0;  //take data from LSU?
                auipcBit    <= '0;
                rawType     <= `B_TYPE;

                case(funct3)
                BEQ[FUNCT3_WIDTH+11:12]: branchType  <= `BEQ;
                BNE[FUNCT3_WIDTH+11:12]: branchType  <= `BNE;
                BLT[FUNCT3_WIDTH+11:12]: branchType  <= `BLT;
                BGE[FUNCT3_WIDTH+11:12]: branchType  <= `BGE;
                BLTU[FUNCT3_WIDTH+11:12]: branchType  <= `BLTU;
                BGEU[FUNCT3_WIDTH+11:12]: branchType  <= `BGEU;
                endcase    
            end
            JAL[OPCODE_WIDTH-1:0]: begin
                ALUOp       <= `ALU_NONE;
                ALUSelect   <= `SEL_NONE;    
                dType       <= '0;                          
                MWE         <= '0;  //write to LSU? 
                RWE         <= '1;  //write to reg file?               
                jump        <= '1;  //jumping?
                branchType  <= '0;
                load        <= '0;  //take data from LSU?
                auipcBit    <= '0;
                rawType     <= `J_TYPE;
            end
            JALR[OPCODE_WIDTH-1:0]: begin
                ALUOp       <= `ALU_ADD;
                ALUSelect   <= `SEL_IALU;    
                dType       <= '0;                          
                MWE         <= '0;  //write to LSU? 
                RWE         <= '1;  //write to reg file?               
                jump        <= '1;  //jumping?
                branchType  <= '0;
                load        <= '0;  //take data from LSU?
                auipcBit    <= '0;
                rawType     <= `I_TYPE;
            end
            default: begin
                ALUOp <= `ALU_NONE;
                ALUSelect <= '0;
                rawType <= `R_TYPE;
                branchType <= BEQ;
                dType <= '0;                
                MWE <= '0;
                RWE <= '0;                
                jump <= '0;
                load <= '0;
                auipcBit    <= '0;
            end
        endcase
    end
endmodule