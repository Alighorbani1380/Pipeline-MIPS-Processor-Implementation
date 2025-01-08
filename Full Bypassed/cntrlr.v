`define RT 6'b000000
`define addi 6'b001000
`define slti 6'b001010
`define lw 6'b010111
`define sw 6'b101011
`define beq 6'b000100
`define j 6'b000010 //
`define jal 6'b000011 //
`define addi 6'b001000
`define ori 6'b001101
`define xori 6'b001111
`define bne 6'b000101
//...................
`define andi 6'b000001
`define lui 6'b000111




module controller(
	clk,
	rst,
	opcode,
	func,
	RegDst,
	Jmp,
	DataC,
	Regwrite,
	AluSrc,
	AluSrc1,
	Branch,
	MemRead,
	MemWrite,
	MemtoReg,
	AluOperation,
	NBranch

	);
	input 				clk,rst;
	input      [5:0] 	opcode,func;
	output reg [1:0]	RegDst,Jmp;
	output reg		    DataC,Regwrite,AluSrc,AluSrc1,Branch,NBranch,MemRead,MemWrite,MemtoReg;
	output reg [3:0]    AluOperation;

	always@(opcode,func) begin
		    {RegDst,Jmp,DataC,Regwrite,AluSrc,Branch,MemRead,MemWrite,MemtoReg,AluOperation,AluSrc1,NBranch}=0;
			case(opcode) 
				`RT: begin
					case(func)
						010000:begin //jr
							Jmp=2'b10;
						end
						010001:begin //jalr
							RegDst=2'b10;
							DataC=1;
							Regwrite=1;
							Jmp=2'b10;
						end
						001011:begin
							RegDst=2'b01;
							Regwrite=1;
							AluOperation=4'b1000;
						end
						001100:begin
							RegDst=2'b01;
							Regwrite=1;
							AluOperation=4'b1001;
						end
						001101:begin
							RegDst=2'b01;
							Regwrite=1;
							AluOperation=4'b1010;
						end
						default :begin
							RegDst=2'b01;
							Regwrite=1;
							AluOperation=func[3:0];
							if(func[3:2]== 2'b10) AluSrc1 = 1'b1;
						end
					endcase
				 end
				`addi: begin
					Regwrite=1;
					AluSrc=1;
					AluOperation=4'b0000;
				 end
				`slti: begin
					Regwrite=1;
					AluSrc=1;
					AluOperation=4'b0101; //less
				 end
				`lw: begin
					Regwrite=1;
					AluSrc=1;
					AluOperation=4'b0000;
					MemRead=1;
					MemtoReg=1;
				 end
				`sw: begin
					AluSrc=1;
					AluOperation=4'b0000;
					MemWrite=1;
				 end
				`beq: begin
					AluOperation=4'b0001;
					Branch=1;
				 end
				`j: begin
					Jmp=2'b01;

				 end
				`jal: begin
					RegDst=2'b10;
					DataC=1;
					Regwrite=1;
					Jmp=2'b01;
				 end
				 `ori: begin
					Regwrite=1;
					AluSrc=1;
					AluOperation=4'b0100;
				 end
				 `xori: begin
					Regwrite=1;
					AluSrc=1;
					AluOperation=4'b0111;
				 end
				  `bne: begin
					AluOperation=4'b0001;
					NBranch=1;
				 end
				  `andi: begin
					Regwrite=1;
					AluSrc=1;
					AluOperation=4'b0011;
				 end
				 `ori: begin
					Regwrite=1;
					AluSrc=1;
					AluOperation=4'b0100;
				 end
				`lui: begin
					Regwrite=1;
					AluSrc=1;
					AluOperation=4'b1111;
				 end


				 
			endcase
	end
endmodule
