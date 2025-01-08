`timescale 1ns/1ns

module mux3_to_1 #(parameter num_bit)(input clk,input [num_bit-1:0]data1,data2,data3, input [1:0]sel,output [num_bit-1:0]out);
	
	assign out=~sel[1] ? (sel[0] ? data2 : data1 ) : data3;	
endmodule
module mux4_to_1 #(parameter num_bit)(input clk,input [num_bit-1:0]data0,data1,data2,data3, input [1:0]sel,output [num_bit-1:0]out);
	
	assign out=~sel[1] ? (sel[0] ? data1 : data0 ) : (~sel[0] ? data2 : data3 );	
endmodule

module mux2_to_1 #(parameter num_bit)(input clk,input [num_bit-1:0]data1,data2, input sel,output [num_bit-1:0]out);
	
	assign out=~sel?data1:data2;
endmodule

module sign_extension(input clk,input [15:0]primary, output [31:0] extended);

	assign extended=$signed(primary);
endmodule

module shl2 #(parameter num_bit)(input clk,input [num_bit-1:0]adr, output [num_bit-1:0]sh_adr);

	assign sh_adr=adr<<2;
endmodule

module alu(input clk,input [31:0]data1,data2, input [3:0]alu_op, output reg[31:0]alu_result, output zero_flag);
	reg [31:0] extended;
	always@(alu_op,data1,data2) begin
		alu_result=32'b0;
		case (alu_op)
			4'b0000:	alu_result=data1 + data2;
			4'b0001:	alu_result=data1 - data2;
			4'b0011: alu_result=data1 & data2;
			4'b0100:	alu_result=data1 | data2;
			//4'b0010:	alu_result=data1 + data2;
			//4'b0011:	alu_result=data1 - data2;
			4'b0101: alu_result=( $signed(data1) < $signed(data2)) ? 32'b1:32'b0;
			4'b0111: alu_result=data1 ^ data2; // Estesna****************
			4'b0110: alu_result=~(data1|data2); // Estesna**********************
			4'b1000:alu_result = data2 << 2;
			4'b1001: alu_result = data1 >> data2;
			4'b1010: alu_result = data1 >>> data2;
			//f
			4'b1100: alu_result = data1 << data2;
			4'b1101: alu_result = data1 >> data2;
			4'b1110: alu_result = data1 >>> data2;
			4'b1111: alu_result = data2;
		endcase
	end
	assign zero_flag=(alu_result==32'b0) ? 1'b1:1'b0;
endmodule

module adder(input clk,input [31:0] data1,data2, output [31:0]sum);
	
	wire co;
	assign {co,sum}=data1+data2;
endmodule


module reg_file(input clk,rst,RegWrite,input [4:0] read_reg1,read_reg2,write_reg,input [31:0]write_data,
		output [31:0]read_data1,read_data2);

	reg [31:0] register[0:31];
	integer i;
	always@(posedge clk,rst) begin
		if(rst) begin
			for(i=0;i<32;i=i+1) register[i]<=32'b0;
		end
		else begin
			if(RegWrite) register[write_reg]<=write_data;
		end
	end
	assign read_data1=register[read_reg1];
	assign read_data2=register[read_reg2];
endmodule

module inst_memory(input clk,rst,input [31:0]adr,output [31:0]instruction);

	reg [31:0]mem_inst[0:255];
	initial begin
		$readmemb("D:/Daneshgah/Master/Term 4031/Computer Arch/Tamrin3/Erae/instructionmemory.txt",mem_inst);
  	end
	assign instruction=mem_inst[adr>>2];
endmodule

module data_memory(input clk,rst,mem_read,mem_write,input [31:0]adr,write_data,output reg[31:0]read_data,
		   output [31:0] out1,out2);

	reg [31:0]mem_data[0:511];
	integer i,f;

	initial begin
		$readmemb("D:/Daneshgah/Master/Term 4031/Computer Arch/Tamrin3/Erae/datamemory.txt",mem_data);
  	end

	always@(posedge clk) begin
		if(mem_write) mem_data[adr>>2]<=write_data;
	end

	always@(mem_read,adr) begin
		if(mem_read) read_data<=mem_data[adr>>2];
		else read_data<=32'b0;	
	end
	
	initial begin
		$writememb("D:/Daneshgah/Master/Term 4031/Computer Arch/Tamrin1/BugDetect/PipeLine/datamemory.txt",mem_data); 
  	end

	initial begin
  		f = $fopen("D:/Daneshgah/Master/Term 4031/Computer Arch/Tamrin1/BugDetect/PipeLine/datamemory.txt","w");
		for(i=0;i<512;i=i+1) begin
		$fwrite(f,"%b\n",mem_data[i]);
		end
		$fclose(f);  
	end

	assign out1=mem_data[500];
	assign out2=mem_data[501];
	
endmodule

module pc(input clk,rst,input [31:0]in,output reg[31:0]out);

	always @(posedge clk,rst) begin
		if(rst) out<=32'b0;
		else out<=in;
	end
endmodule

module Fetch_latch(input [31:0] instruction_in,input clk ,rst, enable , input [31:0] in_pc , output reg [31:0] out_pc_IF,output reg [31:0] instruction_out);

	always@(posedge clk,rst)
	begin
		//if(enable)
		//begin
			if(rst) out_pc_IF <= 32'b0;
			else if(~enable) out_pc_IF <= in_pc;
		//end

	end
endmodule

module Fetch_Decode_latch(input [31:0] instruction_in,input clk ,rst,enable,input [31:0] instruction_IF , pc_adder_IF , output reg[31:0] instruction_ID,pc_adder_ID,output reg [31:0] instruction_out);


	always @(posedge clk,rst)
	begin
		//if (enable) begin
			if(rst) begin
				instruction_out = 32'b0;
				instruction_ID = 32'b0;
				pc_adder_ID = 32'b0;
			end
			else if(~enable)
			begin
				instruction_out		<= instruction_in;
				instruction_ID <= instruction_IF;
				pc_adder_ID <= pc_adder_IF;
			end
		//end
	end

endmodule

module Decode_Exe_latch
(
input [31:0] instruction_ID,instruction_in,
input clk ,rst,enable,
input NBranch_ID,Branch_ID,Regwrite_ID,DataC_ID,MemtoReg_ID,MemWrite_ID,MemRead_ID,AluSrc1_ID,AluSrc_ID,
input [31:0] read_data1_reg_ID,read_data2_reg_ID , inst_extended_ID , pc_adder_ID ,
input [4:0] write_reg_ID , Shamnt_ID , 
input [3:0] AluOperation_ID,
output reg NBranch_EXE,Branch_EXE,Regwrite_EXE,DataC_EXE , MemtoReg_EXE , MemWrite_EXE ,MemRead_EXE ,AluSrc1_EXE,AluSrc_EXE,
output reg [31:0] read_data1_reg_EXE,read_data2_reg_EXE, inst_extended_EXE , pc_adder_EXE ,
output reg [3:0] AluOperation_EXE,
output reg [4:0] write_reg_EXE , Shamnt_EXE,
output reg[31:0] instruction_EXE,instruction_out);


	always @(posedge clk,rst,enable)
	begin
			
		//if (enable) begin
			if(rst | (instruction_in == 32'b0)) begin
				instruction_out = 32'b0;
				instruction_EXE = 32'b0;
				NBranch_EXE = 1'b0;
				Branch_EXE = 1'b0;
				Regwrite_EXE = 1'b0;
				DataC_EXE = 1'b0;
				MemtoReg_EXE = 1'b0;
				MemWrite_EXE = 1'b0;
				MemRead_EXE = 1'b0;
				AluOperation_EXE = 1'b0;
				AluSrc1_EXE = 1'b0;
				AluSrc_EXE = 1'b0;
				read_data1_reg_EXE = 32'b0;
				read_data2_reg_EXE = 32'b0;
				inst_extended_EXE = 32'b0;
				pc_adder_EXE = 32'b0;
				write_reg_EXE = 5'b0; 
				Shamnt_EXE = 5'b0;
			end
			else begin
				instruction_out		<= instruction_in;
				instruction_EXE		<= instruction_ID;
				NBranch_EXE			<= NBranch_ID;
				Branch_EXE			<= Branch_ID;
				Regwrite_EXE 		<= Regwrite_ID;
				DataC_EXE 			<= DataC_ID;
				MemtoReg_EXE 		<= MemtoReg_ID;
				MemWrite_EXE 		<= MemWrite_ID;
				MemRead_EXE 		<= MemRead_ID;
				AluOperation_EXE 	<= AluOperation_ID;
				AluSrc1_EXE 		<= AluSrc1_ID;
				AluSrc_EXE 			<= AluSrc_ID;
				read_data1_reg_EXE 	<= read_data1_reg_ID;
				read_data2_reg_EXE 	<= read_data2_reg_ID;
				inst_extended_EXE 	<= inst_extended_ID;
				pc_adder_EXE 		<= pc_adder_ID;
				write_reg_EXE 		<= write_reg_ID; 
				Shamnt_EXE 			<= Shamnt_ID;
			end
		//end
	end

endmodule
module Exe_Mem_latch
(
input [31:0] instruction_EXE,instruction_in,
input clk ,rst,enable,
input Regwrite_EXE,DataC_EXE,MemtoReg_EXE,MemWrite_EXE,MemRead_EXE,
input [31:0] alu_result_EXE,read_data2_reg_EXE , pc_adder_EXE ,
input [4:0] write_reg_EXE, 
output reg Regwrite_MEM,DataC_MEM,MemtoReg_MEM,MemWrite_MEM,MemRead_MEM,
output reg [31:0] alu_result_MEM,read_data2_reg_MEM , pc_adder_MEM ,
output reg [4:0] write_reg_MEM,
output reg[31:0] instruction_MEM,instruction_out);

	always @(posedge clk,rst)
	begin
		//if (enable) begin
			if(rst) begin
				instruction_out = 32'b0;
				instruction_MEM = 32'b0;
				Regwrite_MEM = 1'b0;
				DataC_MEM = 1'b0;
				MemtoReg_MEM = 1'b0;
				MemWrite_MEM = 1'b0;
				MemRead_MEM = 1'b0;
				alu_result_MEM = 32'b0;
				read_data2_reg_MEM = 32'b0;
				pc_adder_MEM = 32'b0;
				write_reg_MEM = 5'b0; 
			end
			else begin
				instruction_out		<= instruction_in;
				instruction_MEM		<= instruction_EXE;
				Regwrite_MEM		<= 	Regwrite_EXE;
				DataC_MEM 			<= 	DataC_EXE;
				MemtoReg_MEM		<= 	MemtoReg_EXE;
				MemWrite_MEM 		<= 	MemWrite_EXE;
				MemRead_MEM 		<= 	MemRead_EXE;
				alu_result_MEM 		<= 	alu_result_EXE;
				read_data2_reg_MEM 	<=	read_data2_reg_EXE;
				pc_adder_MEM 		<= 	pc_adder_EXE;
				write_reg_MEM 		<= 	write_reg_EXE; 
			end
		//end
	end

endmodule


module Mem_WB_latch
(
input [31:0] instruction_MEM,instruction_in,
input clk ,rst,enable,
input Regwrite_MEM,DataC_MEM,MemtoReg_MEM,
input [31:0] alu_result_MEM,read_data_mem_MEM, pc_adder_MEM ,
input [4:0] write_reg_MEM, 
output reg Regwrite_WB,DataC_WB,MemtoReg_WB,
output reg [31:0] alu_result_WB,read_data_mem_WB ,pc_adder_WB ,
output reg [4:0] write_reg_WB,
output reg[31:0] instruction_WB,instruction_out);

	always @(posedge clk,rst)
	begin
		//if (enable) begin
			if(rst) begin

				instruction_out = 32'b0;
				instruction_WB = 32'b0;
				Regwrite_WB = 1'b0;
				DataC_WB = 1'b0;
				MemtoReg_WB = 1'b0;
				alu_result_WB = 32'b0;
				read_data_mem_WB = 32'b0;
				pc_adder_WB = 32'b0;
				write_reg_WB = 5'b0; 
			end
			else begin
				
				instruction_out		<= instruction_in;
				instruction_WB		<= instruction_MEM;
				Regwrite_WB			<= 	Regwrite_MEM;
				DataC_WB 			<= 	DataC_MEM;
				MemtoReg_WB		<= 	MemtoReg_MEM;
				alu_result_WB 		<= 	alu_result_MEM;
				read_data_mem_WB 		<= 	read_data_mem_MEM;
				pc_adder_WB 		<= 	pc_adder_MEM;
				write_reg_WB 	<=	write_reg_MEM;
			end
		//end
	end

endmodule
module C_re(
	input [5:0] instruction,
	output reg re1,re2
);
	always@(instruction) begin
		re1 = 1'b0;
		re2 = 1'b0;
		case (instruction)
			6'b000000:begin	re1 = 1'b1; re2=1'b1; end // ALU 
			6'b001000:begin	re1 = 1'b1; re2=1'b0; end //ALUI addi
			6'b001010:begin	re1 = 1'b1; re2=1'b0; end //ALUI slti
			6'b010111:begin	re1 = 1'b1; re2=1'b0; end //LW
			6'b101011:begin	re1 = 1'b1; re2=1'b1; end //SW
			6'b000100:begin	re1 = 1'b1; re2=1'b1; end //ALU beq
			6'b001000:begin	re1 = 1'b1; re2=1'b0; end //ALUI addi
			6'b001101:begin	re1 = 1'b1; re2=1'b0; end //ALUI ori
			6'b001111:begin	re1 = 1'b1; re2=1'b0; end //ALUI xori
			6'b000101:begin	re1 = 1'b1; re2=1'b1; end //ALU bneq
			6'b000001:begin	re1 = 1'b1; re2=1'b0; end //ALUI andi
			6'b000111:begin	re1 = 1'b1; re2=1'b0; end //ALUI lui
			6'b000010:begin	re1 = 1'b0; re2=1'b0; end //J
			6'b000011:begin	re1 = 1'b0; re2=1'b0; end //Jal
		endcase
	end


endmodule
module C_dest(
	input [31:0] instruction,
	output reg[4:0] ws,
	output reg we
);
	always@(instruction) begin
		ws = 5'b00000;
		we = 1'b0;
		case (instruction[31:26])
			6'b000000:begin if(instruction[5:0]==010001)begin ws = 5'b11111; we =1'b1; end else begin ws = instruction[15:11]; if(instruction[15:11]!=5'b00000) we=1'b1; end end // ALU 
			6'b001000:begin	ws = instruction[20:16]; if(ws!=5'b00000) we=1'b1; end //ALUI addi
			6'b001010:begin	ws = instruction[20:16]; if(ws!=5'b00000) we=1'b1; end //ALUI slti 
			6'b010111:begin	ws = instruction[20:16]; if(ws!=5'b00000) we=1'b1; end //LW
			//6'b101011:begin	ws = 1'b1; we=1'b0 end; //S;
			6'b000100:begin	ws = instruction[15:11]; if(ws!=5'b00000) we=1'b1; end //ALU beq
			6'b001000:begin	ws = instruction[20:16]; if(ws!=5'b00000) we=1'b1; end //ALUI addi
			6'b001101:begin	ws = instruction[20:16]; if(ws!=5'b00000) we=1'b1; end //ALUI ori
			6'b001111:begin	ws = instruction[20:16]; if(ws!=5'b00000) we=1'b1; end //ALUI xori
			6'b000101:begin	ws = instruction[15:11]; if(ws!=5'b00000) we=1'b1; end //ALU bneq
			6'b000001:begin	ws = instruction[20:16]; if(ws!=5'b00000) we=1'b1; end //ALUI andi
			6'b000111:begin	ws = instruction[20:16]; if(ws!=5'b00000) we=1'b1; end //ALUI lui
			//6'b000010:begin	ws = 1'b0; we=1'b0 end; //J
			6'b000011:begin	ws = 5'b11111; we=1'b1; end //Jal
		endcase
	end


endmodule


//bypass
module C_dest_E(
	input [31:0] instruction,
	output reg[4:0] ws,
	output reg we_Bypass,
	output reg we_stall,
	output reg we
);
	always@(instruction) begin
		ws = 5'b00000;
		we_Bypass = 1'b0;
		we_stall = 1'b0;
		we = 1'b0;
		case (instruction[31:26])
			6'b000000:begin if(instruction[5:0]==010001)begin ws = 5'b11111;  we =1'b1; we_Bypass =1'b1; end else begin ws = instruction[15:11]; if(instruction[15:11]!=5'b00000)begin we_Bypass=1'b1; we_stall = 1'b0; we=1'b1; end end end // ALU 
			6'b001000:begin	ws = instruction[20:16]; if(ws!=5'b00000)begin we_Bypass=1'b1 ; we_stall = 1'b0 ;  we=1'b1;end end //ALUI addi
			6'b001010:begin	ws = instruction[20:16]; if(ws!=5'b00000)begin we_Bypass=1'b1 ; we_stall = 1'b0 ;  we=1'b1;end end //ALUI slti 
			6'b010111:begin	ws = instruction[20:16]; if(ws!=5'b00000)begin we_Bypass=1'b0 ; we_stall = 1'b1 ;  we=1'b1;end end //LW
			//6'b101011:begin	ws = 1'b1; we=1'b0 end; //S;
			6'b000100:begin	ws = instruction[15:11]; if(ws!=5'b00000) begin we_Bypass=1'b1 ; we_stall = 1'b0 ; we=1'b1;end end //ALU beq
			6'b001000:begin	ws = instruction[20:16]; if(ws!=5'b00000) begin we_Bypass=1'b1 ; we_stall = 1'b0 ; we=1'b1;end end //ALUI addi
			6'b001101:begin	ws = instruction[20:16]; if(ws!=5'b00000) begin we_Bypass=1'b1 ; we_stall = 1'b0 ; we=1'b1;end end //ALUI ori
			6'b001111:begin	ws = instruction[20:16]; if(ws!=5'b00000) begin we_Bypass=1'b1 ; we_stall = 1'b0 ; we=1'b1;end end //ALUI xori
			6'b000101:begin	ws = instruction[15:11]; if(ws!=5'b00000) begin we_Bypass=1'b1 ; we_stall = 1'b0 ; we=1'b1;end end //ALU bneq
			6'b000001:begin	ws = instruction[20:16]; if(ws!=5'b00000) begin we_Bypass=1'b1 ; we_stall = 1'b0 ; we=1'b1;end end //ALUI andi
			6'b000111:begin	ws = instruction[20:16]; if(ws!=5'b00000) begin we_Bypass=1'b1 ; we_stall = 1'b0 ; we=1'b1;end end //ALUI lui
			//6'b000010:begin	ws = 1'b0; we=1'b0 end; //J
			6'b000011:begin	ws = 5'b11111; we_Bypass=1'b0 ; we_stall = 1'b1; we=1'b1; end //Jal
		endcase
	end


endmodule
//
module Stall_Controller(
	input [4:0] rs_ID,rt_ID,ws_EXE,ws_MEM,ws_WB,
	input we_EXE,we_Bypass,we_Stall,we_MEM,we_WB,re1_ID,re2_ID,
	input [5:0] opcode_EXE,
	input zero,
	output stall
);
	// assign stall = (((rs_ID == ws_EXE)&we_EXE|(rs_ID == ws_MEM)&we_MEM|(rs_ID==ws_WB)&we_WB)&re1_ID) | (((rt_ID==ws_EXE)&we_EXE|(rt_ID == ws_MEM)&we_MEM|(rt_ID==ws_WB)&we_WB)&re1_ID);
	wire condition1;  
    wire condition2;  
    wire condition3;  

    // محاسبه شرایط  
    assign condition1 = ((rs_ID == ws_EXE) & we_Stall) | ((rs_ID == ws_MEM) & we_MEM) | ((rs_ID == ws_WB) & we_WB);  
    assign condition2 = ((rt_ID == ws_EXE) & we_EXE) | ((rt_ID == ws_MEM) & we_MEM) | ((rt_ID == ws_WB) & we_WB);  
    assign condition3 = ((opcode_EXE == 000100) & zero) | ((opcode_EXE == 000100) & ~zero);
    // محاسبه نهایی stall  
    assign stall = (condition1 & re1_ID) | (condition2 & re2_ID) & ~condition3;  

endmodule

//Branches
module IR_Src_branches(
	input [31:0] instruction_EXE,
	input [31:0] instruction_ID,
	input zero,
	input stall,
	output reg IRSrc_E,
	output reg IRSrc_D
);
	always@(*) begin
		IRSrc_E = 1'b0;
		IRSrc_D = 1'b0;
		case (instruction_EXE[31:26])
			//6'b000000:begin if(instruction[5:0]==010000 | instruction[5:0]==010001)begin IRSrc_D = 1'b1 ; if(stall) IRSrc_E=1'b1; else  IRSrc_E=IRSrc_D;   end  end // ALU 
			//6'b001000:begin	ws = instruction[20:16]; if(ws!=5'b00000)begin we_Bypass=1'b1 ; we_stall = 1'b0 ;  we=1'b1;end end //ALUI addi
			//6'b001010:begin	ws = instruction[20:16]; if(ws!=5'b00000)begin we_Bypass=1'b1 ; we_stall = 1'b0 ;  we=1'b1;end end //ALUI slti 
			//6'b010111:begin	ws = instruction[20:16]; if(ws!=5'b00000)begin we_Bypass=1'b0 ; we_stall = 1'b1 ;  we=1'b1;end end //LW
			//6'b101011:begin	ws = 1'b1; we=1'b0 end; //SW;
			6'b000100:begin	if(zero)begin IRSrc_D = 1'b1; IRSrc_E = 1'b1; end end //ALU beq
			//6'b001000:begin	ws = instruction[20:16]; if(ws!=5'b00000) begin we_Bypass=1'b1 ; we_stall = 1'b0 ; we=1'b1;end end //ALUI addi
			//6'b001101:begin	ws = instruction[20:16]; if(ws!=5'b00000) begin we_Bypass=1'b1 ; we_stall = 1'b0 ; we=1'b1;end end //ALUI ori
			//6'b001111:begin	ws = instruction[20:16]; if(ws!=5'b00000) begin we_Bypass=1'b1 ; we_stall = 1'b0 ; we=1'b1;end end //ALUI xori
			6'b000101:begin if(~zero)begin IRSrc_D = 1'b1; IRSrc_E = 1'b1; end end //ALU bneq
			//6'b000001:begin	ws = instruction[20:16]; if(ws!=5'b00000) begin we_Bypass=1'b1 ; we_stall = 1'b0 ; we=1'b1;end end //ALUI andi
			//6'b000111:begin	ws = instruction[20:16]; if(ws!=5'b00000) begin we_Bypass=1'b1 ; we_stall = 1'b0 ; we=1'b1;end end //ALUI lui
			//6'b000010:begin	IRSrc_D = 1'b1;if(stall) IRSrc_E=1'b1; else  IRSrc_E=IRSrc_D;  end; //J
			//6'b000011:begin	IRSrc_D = 1'b1;if(stall) IRSrc_E=1'b1; else  IRSrc_E=IRSrc_D;  end //Jal
			default:begin 
				case(instruction_ID[31:26])
					6'b000000:begin if(instruction_ID[5:0]==010000 | instruction_ID[5:0]==010001)begin IRSrc_D = 1'b1 ; if(stall) IRSrc_E=1'b1; else  IRSrc_E=IRSrc_D;   end  end // ALU 
					6'b000010:begin	IRSrc_D = 1'b1;if(stall) IRSrc_E=1'b1; else  IRSrc_E=IRSrc_D;  end //J
					6'b000011:begin	IRSrc_D = 1'b1;if(stall) IRSrc_E=1'b1; else  IRSrc_E=IRSrc_D;  end //Jal
				endcase
			end
			
		endcase
	end


endmodule
//Branch