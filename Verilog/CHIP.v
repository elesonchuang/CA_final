// Your code
module CHIP(clk,
            rst_n,
            // For mem_D
            mem_wen_D,
            mem_addr_D,
            mem_wdata_D,
            mem_rdata_D,
            // For mem_I
            mem_addr_I,
            mem_rdata_I);

    input         clk, rst_n ;
    // For mem_D
    output        mem_wen_D  ;
    output [31:0] mem_addr_D ;
    output [31:0] mem_wdata_D;
    input  [31:0] mem_rdata_D;
    // For mem_I
    output [31:0] mem_addr_I ;
    input  [31:0] mem_rdata_I;
    
    //---------------------------------------//
    // Do not modify this part!!!            //
    // Exception: You may change wire to reg //
    reg    [31:0] PC          ;              //
    reg   [31:0] PC_nxt      ;               //
    wire          regWrite    ;              //
    wire   [ 4:0] rs1, rs2, rd;              //
    wire   [31:0] rs1_data    ;              //
    wire   [31:0] rs2_data    ;              //
    wire   [31:0] rd_data     ;              //
    //---------------------------------------//

    // Todo: other wire/reg
    wire [6:0] opcode;

    //---------------------------------------//
    // Do not modify this part!!!            //
    reg_file reg0(                           //
        .clk(clk),                           //
        .rst_n(rst_n),                       //
        .wen(regWrite),                      //
        .a1(rs1),                            //
        .a2(rs2),                            //
        .aw(rd),                             //
        .d(rd_data),                         //
        .q1(rs1_data),                       //
        .q2(rs2_data));                      //
    //---------------------------------------//
    
    // Todo: any combinational/sequential circuit

        //register file input 
    always @(*) begin
        assign rs1 = mem_rdata_I[19:15];
        assign rs2 = mem_rdata_I[24:20];
        assign rd = mem_rdata_I[11:7];
    end

        // opcode signal 
    wire Branch, AluSrc, Jump;
    always @(*) begin
        assign opcode = mem_rdata_I[6:0];
        case (opcode)
            7'b0010111:begin//AUIPC
                assign Branch = 0;
                assign Jump = 0;
            end 
            7'b1101111:begin//JAL
                assign Branch = 0;
                assign Jump = 1;
            end
            7'b1100111:begin//JALR
                assign Branch = 0;
                assign Jump = 1; 
            end
            7'b1100011:begin//BEQ
                assign Branch = 1;
                assign Jump = 0; 
            end
            7'b0000011:begin//LW
                assign Branch = 0;
                assign Jump = 0; 
            end
            7'b0100011:begin://SW
                assign Branch = 0;
                assign Jump = 0; 
            end
            7'b0010011:begin//ADDI
                assign Branch = 0;
                assign Jump = 0; 
            end
            7'b0010011:begin//SLTI
                assign Branch = 0;
                assign Jump = 0; 
            end
            7'b0110011:begin//ADD,SUB
                assign Branch = 0;
                assign Jump = 0; 
            end
            default: begin
                assign Branch = 0;
                assign Jump = 0;
            end
        endcase
    end    
    
        //IF stage, PC = PC + 4 or result from the jal, beq immediate 
    always @(*) begin
        // PCSrc logic, 0:PC=PC+4 | 1:PC=Jump or branch result
        assign PCSrc = Jump | (Branch & Zero);  
        PC_nxt = (PCSrc) ? (PC + /*TODO: result from the Immediate Gen*/ 32'd0 :PC + 32'd4 );
    end



    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            PC <= 32'h00010000; // Do not modify this value!!!
        end
        else begin
            PC <= PC_nxt;
            assign mem_addr_I = PC;
        end
    end
endmodule

module reg_file(clk, rst_n, wen, a1, a2, aw, d, q1, q2);
   
    parameter BITS = 32;
    parameter word_depth = 32;
    parameter addr_width = 5; // 2^addr_width >= word_depth
    
    input clk, rst_n, wen; // wen: 0:read | 1:write
    input [BITS-1:0] d;
    input [addr_width-1:0] a1, a2, aw;

    output [BITS-1:0] q1, q2;

    reg [BITS-1:0] mem [0:word_depth-1];
    reg [BITS-1:0] mem_nxt [0:word_depth-1];

    integer i;

    assign q1 = mem[a1];
    assign q2 = mem[a2];

    always @(*) begin
        for (i=0; i<word_depth; i=i+1)
            mem_nxt[i] = (wen && (aw == i)) ? d : mem[i];
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mem[0] <= 0;
            for (i=1; i<word_depth; i=i+1) begin
                case(i)
                    32'd2: mem[i] <= 32'hbffffff0;
                    32'd3: mem[i] <= 32'h10008000;
                    default: mem[i] <= 32'h0;
                endcase
            end
        end
        else begin
            mem[0] <= 0;
            for (i=1; i<word_depth; i=i+1)
                mem[i] <= mem_nxt[i];
        end       
    end
endmodule

module mulDiv(clk, rst_n, valid, ready, mode, in_A, in_B, out);
    // Todo: your HW2
    input         clk, rst_n;
    input         valid;
    input  [1:0]  mode; // mode: 0: mulu, 1: divu, 2: and, 3: or
    output        ready;
    input  [31:0] in_A, in_B;
    output [63:0] out;

    // Definition of states
    localparam IDLE = 3'd0;
    localparam MUL  = 3'd1;
    localparam DIV  = 3'd2;
    localparam AND = 3'd3;
    localparam OR = 3'd4;
    localparam OUT  = 3'd5;

    // Todo: Wire and reg if needed
    reg  [ 2:0] state, state_nxt;
    reg  [ 4:0] counter, counter_nxt;
    reg  [63:0] shreg, shreg_nxt;
    reg  [31:0] alu_in, alu_in_nxt;
    reg  [32:0] alu_out;

    // Todo: Instatiate any primitives if needed

    // Todo 5: Wire assignments
    assign out = shreg;
    assign ready = (state == OUT);
    // Combinational always block
    // Todo 1: Next-state logic of state machine
    always @(*) begin
        case(state)
            IDLE: begin
                if (valid) begin
                    case (mode)
                        2'd0 : state_nxt = MUL;
                        2'd1 : state_nxt = DIV;
                        2'd2 : state_nxt = AND;
                        2'd3 : state_nxt = OR;
                    endcase
                end
                else state_nxt = IDLE;
            end
            MUL : state_nxt = (counter == 5'd31) ? OUT : MUL;
            DIV : state_nxt = (counter == 5'd31) ? OUT : DIV;
            AND : state_nxt = OUT;
            OR  : state_nxt = OUT;
            OUT : state_nxt = IDLE;
            default : state_nxt = IDLE;
        endcase
    end
    // Todo 2: Counter
    always @(posedge clk) begin
        case(state)
            MUL:  if (counter < 5'd31) counter_nxt = counter + 5'd1;
            DIV:  if (counter < 5'd31) counter_nxt = counter + 5'd1;
        default : counter_nxt = 5'd0;
        endcase
        counter = counter_nxt;
    end
    // ALU input
    always @(*) begin
        case(state)
            IDLE: begin
                if (valid) alu_in_nxt = in_B;
                else       alu_in_nxt = 0;
            end
            OUT : alu_in_nxt = 0;
            default: alu_in_nxt = alu_in;
        endcase
    end

    // Todo 3: ALU output
    always @(*) begin
        case (state)
            MUL: begin 
                if (shreg[0]) alu_out = shreg[63:32] + alu_in;
                else    alu_out = shreg[63:32];
            end
            DIV: alu_out = (shreg[63:32] > alu_in) ? (shreg[63:32] - alu_in):(shreg[63:32]);
            AND: alu_out = shreg[31:0] & alu_in;
            OR: alu_out = shreg[31:0] | alu_in;
            default: alu_out = shreg[63:32];
        endcase
    end
    // Todo 4: Shift register
    always @(*) begin
        case (state)
            MUL: begin
                shreg_nxt[63:31] = alu_out;
                shreg_nxt[30:0] = shreg[31:1];
            end
            AND:begin
                shreg_nxt[63:33] = 0;
                shreg_nxt[32:0] = alu_out;
            end
            OR:begin
                shreg_nxt[63:33] = 0;
                shreg_nxt[32:0] = alu_out;
            end
            DIV:begin
                if (counter ==  5'd31) begin
                    shreg_nxt[63] = 0;
                    shreg_nxt[62:32] = alu_out[30:0];
                    shreg_nxt[31:1] = shreg[30:0];
                    shreg_nxt[0] = (shreg[63:32] > alu_in);
                end
                else begin
                    shreg_nxt[63:33] = alu_out[30:0];
                    shreg_nxt[32:1] = shreg[31:0];
                    shreg_nxt[0] = (shreg[63:32] > alu_in);
                end
            end 
            IDLE: begin
                if (valid) begin
                    case (mode)
                        2'd0 :begin 
                            shreg_nxt[31:0] = in_A;//state_nxt = MUL
                            shreg_nxt[63:32] = 0;
                        end
                        2'd1 :begin 
                            shreg_nxt[0] = 0;
                            shreg_nxt[32:1] = in_A;//state_nxt = DIV
                            shreg_nxt[63:33] = 0;
                        end
                        2'd2 :begin 
                            shreg_nxt[31:0] = in_A;//state_nxt = AND
                            shreg_nxt[63:32] = 0;
                        end
                        2'd3 :begin 
                            shreg_nxt[31:0] = in_A;//state_nxt = OR
                            shreg_nxt[63:32] = 0;
                        end
                    endcase
                end
                else shreg_nxt[63:0] = 0;
            end
            default: shreg_nxt[63:0] = shreg[63:0];
        endcase
    end
    // Todo: Sequential always block
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
        end
        else begin
            alu_in <= alu_in_nxt;
            shreg[63:0] <= shreg_nxt[63:0];
            state <= state_nxt;

        end
    end
endmodule


// the ALU, use 64 bits input inA, inB, and 4 bits control signal(generate by the ALU_control).

module ALU (inA, inB, shift_amount, alu_out, zero, control); 
	input [63:0] inA, inB;
	output [63:0] alu_out;
	output zero;
	reg zero;
	reg [63:0] alu_out;
	input [3:0] control;
    input [4:0] shift_amount;
	always @ (*) begin
        zero = 1'b0;
        case (control) // instruction[30, 14-12]
            //add
            4'b0000:begin  
                alu_out <= inA + inB; 
            end
            // sub and beq
            4'b1000:begin 
                alu_out <= inA - inB;  // sub
                zero <= (inA == inB) ? 1: 0;  // check if inA and inB are equal
            end
            // slti
            4'b0010: begin
                alu_out <= ($signed(inA) < $signed(inB)) ? 64'b1: 64'b0;
                // zero <= 0;
            end
            // slli
            4'b0001: begin
                alu_out <= inA << shift_amount;
            end
            // srli
            4'b0101: begin
                alu_out <= inA >> shift_amount;
            end
            default:begin 
                alu_out <= 32'b0; 
            end
        endcase
	end
endmodule

