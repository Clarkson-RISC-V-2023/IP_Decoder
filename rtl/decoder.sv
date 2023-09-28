module decoder #(
    // Parameters
    parameter ALUOP_WIDTH = 5,
    parameter TYPE_WIDTH = 3,
    parameter DTYPE_WIDTH = 2,
    parameter BRANCHTYPE_WIDTH = 3,
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
    output reg [BRANCHTYPE_WIDTH-1:0] branchType,   // Branch unit
    output reg jump,                                // muxes for PC and register file data_in 
    output reg [ALUSELECT_WIDTH-1:0] ALUSelect      // mux the output of IALU, FALU, and CORDIC
);
    // Instruction Types
    `define R_TYPE 3'b000
    `define I_TYPE 3'b001
    `define S_TYPE 3'b010
    `define B_TYPE 3'b011
    `define U_TYPE 3'b100
    `define J_TYPE 3'b101

    wire [6:0] opcode;

    assign opcode = instr[6:0];
    

    //determine instruction type
    always @(instr)
    begin 
        

    end
endmodule;