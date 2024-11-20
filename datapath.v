`timescale 1ns/1ns

module data_path(
	clk,
	rst,
	RegDst,
	Jmp,
	DataC,
	RegWrite,
	AluSrc,
	Branch,
	MemRead,
	MemWrite,
	MemtoReg,
	AluOperation,
	func,
	opcode,
	out1,
	out2,
	AluSrc1,
	NBranch
	);

	input 					clk,rst;
	input       [1:0]		RegDst,Jmp;
	input 					DataC,RegWrite,AluSrc,AluSrc1,Branch,NBranch,MemRead,MemWrite,MemtoReg;
	input       [3:0]		AluOperation;
	output   	[5:0] 		func,opcode;
	output wire [31:0] 		out1,out2;

	wire        [31:0] 		instruction_stall_branched,instruction_IF_branched,read_data1_reg_bypassed_ID,instruction_stall,instruction_WB_S,instruction_MEM_S,instruction_EXE_S,in_pc,out_pc,instruction_IF,instruction_ID,instruction_EXE,instruction_MEM,instruction_WB,write_data_reg_WB,read_data1_reg_ID,read_data1_reg_EXE,read_data2_reg_ID,read_data2_reg_EXE,read_data2_reg_MEM,pc_adder_IF,pc_adder_ID,pc_adder_EXE,pc_adder_MEM,pc_adder_WB,mem_read_data,
							inst_extended_ID,inst_extended_EXE,alu_input2,alu_input1,alu_result_EXE,alu_result_MEM,alu_result_WB,read_data_mem_MEM,read_data_mem_WB,shifted_inst_extended,out_adder2,out_branch;
	wire 		[4:0] 		Shamnt_ID,Shamnt_EXE,write_reg_ID,write_reg_EXE,write_reg_MEM,write_reg_WB;
	wire 		[25:0] 		shl2_inst;
	wire 					and_z_b,zero;
	wire		[3:0]		AluOperation_ID,AluOperation_EXE;
	wire					IRSrc_E,IRSrc_D,ASrc_Bypassing,we_EXE,we_Bypass,we_Stall,we_WB,we_MEM,enable_stall,re1_ID,re2_ID,FL_Reset,FD_Reset,DE_Reset,EM_Reset,MW_Reset,FL_Enable,FD_Enable,DE_Enable,EM_Enable,MW_Enable,DataC_ID,DataC_EXE,DataC_MEM,DataC_WB,RegWrite_ID,RegWrite_EXE,RegWrite_MEM,
							RegWrite_WB,AluSrc_ID,AluSrc_EXE,Branch_ID,Branch_EXE,MemRead_ID,MemRead_EXE,MemRead_MEM,MemWrite_ID,MemWrite_EXE,
							MemWrite_MEM,MemtoReg_ID,MemtoReg_EXE,MemtoReg_MEM,MemtoReg_WB,AluSrc1_ID,AluSrc1_EXE,NBranch_ID,NBranch_EXE;
	wire		[4:0]		ws_WB,ws_MEM,ws_EXE;
	
	//branches
		mux2_to_1 #32 mux_for_branches(.clk(clk),.data1(instruction_IF),.data2(32'b0),.sel(IRSrc_D),.out(instruction_IF_branched));
		mux2_to_1 #32 Mux_branched(.clk(clk),.data1(instruction_stall),.data2(32'b0),.sel(IRSrc_E),.out(instruction_stall_branched));
		IR_Src_branches IR_Src(.instruction_EXE(instruction_EXE),.instruction_ID(instruction_ID),.zero(zero),.stall(enable_stall),.IRSrc_E(IRSrc_E),.IRSrc_D(IRSrc_D));
		Stall_Controller stallcontroller(.zero(zero),.opcode_EXE(instruction_EXE[31:26]),.rs_ID(instruction_ID[25:21]),.rt_ID(instruction_ID[20:16]),.ws_EXE(ws_EXE),.ws_MEM(ws_MEM),.ws_WB(ws_WB),.we_EXE(we_EXE),.we_Bypass(we_Bypass),.we_Stall(we_Stall),.we_MEM(we_MEM),.we_WB(we_WB),.re1_ID(re1_ID),.re2_ID(re2_ID),.stall(enable_stall));

	//

	// ّFor bypassing

		mux2_to_1 #32 mux_for_bypassing(.clk(clk),.data1(read_data1_reg_ID),.data2(alu_result_EXE),.sel(ASrc_Bypassing),.out(read_data1_reg_bypassed_ID));
		assign ASrc_Bypassing = (instruction_ID[25:21]/*rs_ID*/==ws_EXE) & we_Bypass & re1_ID;
	//

	mux2_to_1 #32 Mux_stall(.clk(clk),.data1(instruction_ID),.data2(32'b0),.sel(enable_stall),.out(instruction_stall));
	C_re C_re_ID(.instruction(instruction_ID[5:0]),.re1(re1_ID),.re2(re2_ID));
	C_dest_E C_dest_EXE(.instruction(instruction_EXE_S),.ws(ws_EXE),.we_Bypass(we_Bypass),.we_stall(we_Stall),.we(we_EXE));
	C_dest C_dest_MEM(.instruction(instruction_MEM_S),.ws(ws_MEM),.we(we_MEM));
	C_dest C_dest_WB(.instruction(instruction_WB_S),.ws(ws_WB),.we(we_WB));
	//Stall_Controller stallcontroller(.rs_ID(instruction_ID[25:21]),.rt_ID(instruction_ID[20:16]),.ws_EXE(ws_EXE),.ws_MEM(ws_MEM),.ws_WB(ws_WB),.we_EXE(we_EXE),.we_Bypass(we_Bypass),.we_Stall(we_Stall),.we_MEM(we_MEM),.we_WB(we_WB),.re1_ID(re1_ID),.re2_ID(re2_ID),.stall(enable_stall));


	assign FL_Enable = enable_stall;
	assign FD_Enable = enable_stall;
	// FL_Reset = rst;
	assign FL_Reset = rst;
	assign FD_Reset = rst;
	assign DE_Reset = rst;
	assign EM_Reset = rst;
	assign MW_Reset = rst;
	//assign rst = FD_Reset;
	//assign rst = DE_Reset;
	//assign rst = EM_Reset;
	//assign rst = MW_Reset;
	// assign DataC = DataC_ID;
	// assign RegWrite = RegWrite_ID;
	// assign AluSrc = AluSrc_ID;
	// assign Branch = Branch_ID;
	// assign MemRead = MemRead_ID;
	// assign MemWrite = MemWrite_ID;
	// assign MemtoReg = MemtoReg_ID;
	// assign AluOperation = AluOperation_ID;
	// assign AluSrc1 = AluSrc1_ID;
	// assign NBranch = NBranch_ID;

	assign DataC_ID = DataC;
	assign RegWrite_ID = RegWrite;
	assign AluSrc_ID = AluSrc;
	assign Branch_ID = Branch;
	assign MemRead_ID = MemRead;
	assign MemWrite_ID = MemWrite;
	assign MemtoReg_ID = MemtoReg;
	assign AluOperation_ID = AluOperation;
	assign AluSrc1_ID = AluSrc1;
	assign NBranch_ID = NBranch;






	

	//pc PC(.clk(clk),.rst(rst),.in(in_pc),.out(out_pc));

	adder adder_of_pc(.clk(clk),.data1(out_pc),.data2(32'd4),.sum(pc_adder_IF));

	adder adder2(.clk(clk),.data1(shifted_inst_extended),.data2(pc_adder_EXE),.sum(out_adder2));

	alu ALU(.clk(clk),.data1(alu_input1),.data2(alu_input2),.alu_op(AluOperation_EXE),.alu_result(alu_result_EXE),.zero_flag(zero));

	inst_memory InstMem(.clk(clk),.rst(rst),.adr(out_pc),.instruction(instruction_IF));

	reg_file RegFile(.clk(clk),.rst(rst),.RegWrite(RegWrite_WB),.read_reg1(instruction_ID[25:21]),.read_reg2(instruction_ID[20:16]),
					 .write_reg(write_reg_WB),.write_data(write_data_reg_WB),.read_data1(read_data1_reg_ID),.read_data2(read_data2_reg_ID));

	data_memory data_mem(.clk(clk),.rst(rst),.mem_read(MemRead_MEM),.mem_write(MemWrite_MEM),.adr(alu_result_MEM),
						 .write_data(read_data2_reg_MEM),.read_data(read_data_mem_MEM),.out1(out1),.out2(out2)); 

	mux3_to_1 #5 mux3_reg_file(.clk(clk),.data1(instruction_ID[20:16]),.data2(instruction_ID[15:11]),.data3(5'd31),.sel(RegDst),.out(write_reg_ID));

	mux3_to_1 #32 mux3_jmp(.clk(clk),.data1(out_branch),.data2({pc_adder_IF[31:26],shl2_inst}),.data3(read_data1_reg_ID),.sel(Jmp),.out(in_pc));
	//mux3_to_1 #32 mux3_jmp(.clk(clk),.data1(pc_adder),.data2({pc_adder[31:26],shl2_inst}),.data3(read_data1_reg),.sel(Jmp),.out(in_pc));
	assign and_z_b=(zero & Branch_EXE) | (~zero & NBranch_EXE); //Useless

	mux2_to_1 #32 mux2_reg_file(.clk(clk),.data1(mem_read_data),.data2(pc_adder_WB),.sel(DataC_WB),.out(write_data_reg_WB));

	mux2_to_1 #32 alu_mux(.clk(clk),.data1(read_data2_reg_EXE),.data2(inst_extended_EXE),.sel(AluSrc_EXE),.out(alu_input2));
	mux2_to_1 #32 alu_mux2(.clk(clk),.data1(read_data1_reg_EXE),.data2({27'b0,Shamnt_EXE}),.sel(AluSrc1_EXE),.out(alu_input1));

	assign Shamnt_ID = instruction_ID[10:6];

	mux2_to_1 #32 mux_of_mem(.clk(clk),.data1(alu_result_WB),.data2(read_data_mem_WB),.sel(MemtoReg_WB),.out(mem_read_data));

	mux2_to_1 #32 mux2_branch(.clk(clk),.data1(pc_adder_IF),.data2(out_adder2),.sel(and_z_b),.out(out_branch));
	
	sign_extension sign_ext(.clk(clk),.primary(instruction_ID[15:0]),.extended(inst_extended_ID));

	shl2 #26 shl2_1(.clk(clk),.adr(instruction_ID[25:0]),.sh_adr(shl2_inst));

	shl2 #32 shl2_of_adder2(.clk(clk),.adr(inst_extended_EXE),.sh_adr(shifted_inst_extended));

	assign func=instruction_ID[5:0];

	assign opcode=instruction_ID[31:26];

	Fetch_latch FL( .clk(clk) ,.rst(FL_Reset), .enable(FL_Enable) , .in_pc(in_pc) ,  .out_pc_IF(out_pc));
	Fetch_Decode_latch FD(.clk(clk) ,.rst(FD_Reset),.enable(FD_Enable),.instruction_IF(instruction_IF_branched) , .pc_adder_IF(pc_adder_IF) ,  .instruction_ID(instruction_ID),.pc_adder_ID(pc_adder_ID));
	Decode_Exe_latch DE
	(.instruction_in(instruction_stall_branched),
	.clk(clk) ,.rst(DE_Reset),.enable(DE_Enable),
	.NBranch_ID(NBranch_ID),.Branch_ID(Branch_ID),.Regwrite_ID(RegWrite_ID),.DataC_ID(DataC_ID),.MemtoReg_ID(MemtoReg_ID),.MemWrite_ID(MemWrite_ID),.MemRead_ID(MemRead_ID),.AluOperation_ID(AluOperation_ID),.AluSrc1_ID(AluSrc1_ID),.AluSrc_ID(AluSrc_ID),
	.read_data1_reg_ID(read_data1_reg_bypassed_ID)/*bypassing*/,.read_data2_reg_ID(read_data2_reg_ID), .inst_extended_ID(inst_extended_ID) , .pc_adder_ID(pc_adder_ID) ,
	.write_reg_ID(write_reg_ID) , .Shamnt_ID(Shamnt_ID) , 
	.NBranch_EXE(NBranch_EXE),.Branch_EXE(Branch_EXE),.Regwrite_EXE(RegWrite_EXE),.DataC_EXE(DataC_EXE) , .MemtoReg_EXE(MemtoReg_EXE) , .MemWrite_EXE(MemWrite_EXE) ,.MemRead_EXE(MemRead_EXE) ,.AluOperation_EXE(AluOperation_EXE),.AluSrc1_EXE(AluSrc1_EXE),.AluSrc_EXE(AluSrc_EXE),
	.read_data1_reg_EXE(read_data1_reg_EXE),.read_data2_reg_EXE(read_data2_reg_EXE), .inst_extended_EXE(inst_extended_EXE) , .pc_adder_EXE(pc_adder_EXE) ,
	.write_reg_EXE(write_reg_EXE) , .Shamnt_EXE(Shamnt_EXE),
	.instruction_out(instruction_EXE_S),
	.instruction_ID(instruction_ID),. instruction_EXE(instruction_EXE));
	Exe_Mem_latch EM
	(.instruction_in(instruction_EXE_S),
	.clk(clk) ,.rst(EM_Reset),.enable(EM_Enable),
	.Regwrite_EXE(RegWrite_EXE),.DataC_EXE(DataC_EXE),.MemtoReg_EXE(MemtoReg_EXE),.MemWrite_EXE(MemWrite_EXE),.MemRead_EXE(MemRead_EXE),
	.alu_result_EXE(alu_result_EXE),.read_data2_reg_EXE(read_data2_reg_EXE) , .pc_adder_EXE(pc_adder_EXE) ,
	.write_reg_EXE(write_reg_EXE), 
	.Regwrite_MEM(RegWrite_MEM),.DataC_MEM(DataC_MEM),.MemtoReg_MEM(MemtoReg_MEM),.MemWrite_MEM(MemWrite_MEM),.MemRead_MEM(MemRead_MEM),
	.alu_result_MEM(alu_result_MEM),.read_data2_reg_MEM(read_data2_reg_MEM) , .pc_adder_MEM(pc_adder_MEM) ,
	.write_reg_MEM(write_reg_MEM),
	.instruction_out(instruction_MEM_S),
	.instruction_MEM(instruction_MEM),.instruction_EXE(instruction_EXE));
	Mem_WB_latch MW
	(.instruction_in(instruction_MEM_S),
	.clk(clk) ,.rst(MW_Reset),.enable(MW_Enable),
	.Regwrite_MEM(RegWrite_MEM),.DataC_MEM(DataC_MEM),.MemtoReg_MEM(MemtoReg_MEM),
	.alu_result_MEM(alu_result_MEM),.read_data_mem_MEM(read_data_mem_MEM), .pc_adder_MEM(pc_adder_MEM) ,
	.write_reg_MEM(write_reg_MEM), 
	.Regwrite_WB(RegWrite_WB),.DataC_WB(DataC_WB),.MemtoReg_WB(MemtoReg_WB),
	.alu_result_WB(alu_result_WB),.read_data_mem_WB(read_data_mem_WB) ,.pc_adder_WB(pc_adder_WB) ,
	.write_reg_WB(write_reg_WB),
	.instruction_out(instruction_WB_S),
	.instruction_MEM(instruction_MEM),.instruction_WB(instruction_WB));

endmodule
