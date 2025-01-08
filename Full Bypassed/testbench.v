`timescale 1ps/1ps

module mips_tb;

    // Inputs
    reg clk;
    reg rst;

    // Outputs
    wire [31:0] out1;
    wire [31:0] out2;

    // Instantiate the Unit Under Test (UUT)
    mips uut (
        .clk(clk), 
        .rst(rst), 
        .out1(out1), 
        .out2(out2)
    );

    // Clock generation: 100ns period (50ns high, 50ns low)
    always #50 clk = ~clk;

    initial begin
        // Initialize Inputs
        clk = 0;
        rst = 1;

        // Reset high for the first 100ns
        #100;
        rst = 0;

        // Run simulation for some time

    end

endmodule