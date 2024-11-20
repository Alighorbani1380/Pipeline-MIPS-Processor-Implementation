# Advanced-Computer-Architecture---Pipeline-MIPS-Processor-Implementation
Contains the Verilog implementation of a pipelined MIPS processor designed as part of the **Advanced Computer Architecture** course at the University of Tehran. The project focuses on implementing a pipeline architecture, addressing data and control hazards, and testing the design with a program to find the largest element in an array.

---


## Project Overview

### Objective
The goal of this project was to:
1. Extend the single-cycle MIPS processor from Exercise 1 to a 5-stage pipelined processor.
2. Implement hardware-based detection and resolution of data and control hazards.
3. Test the design using a custom program written in assembly.

### Pipeline Stages
The processor consists of the following pipeline stages:
1. **Instruction Fetch (IF)**: Fetches the instruction from memory.
2. **Instruction Decode (ID)**: Decodes the instruction and reads operands from registers.
3. **Execution (EXE)**: Performs arithmetic/logic operations using the ALU.
4. **Memory Access (MEM)**: Accesses data memory for load/store instructions.
5. **Write Back (WB)**: Writes results back to the register file.

---

## Hazard Control Techniques

### 1. **Data Hazard Handling**
- **Data Forwarding (Bypassing)**:
  - Implemented to forward data directly from the EXE or MEM stages to dependent instructions.
  - Avoids unnecessary stalls when the required data is available earlier in the pipeline.
  ```verilog
  mux2_to_1 #32 mux_for_bypassing(
      .clk(clk),
      .data1(read_data1_reg_ID),
      .data2(alu_result_EXE),
      .sel(ASrc_Bypassing),
      .out(read_data1_reg_bypassed_ID)
  );

  ```

- **Stalling**:
  - For cases where forwarding isn't sufficient, stalls are introduced to delay the pipeline until dependencies are resolved.
    ```verilog
    module Stall_Controller(
    input [4:0] rs_ID, rt_ID, ws_EXE, ws_MEM, ws_WB,
    input we_EXE, we_MEM, we_WB, re1_ID, re2_ID,
    output stall
    );
    ```
### 2. **Control Hazard Handling**
- **Speculative Execution**:
  - For branch instructions, speculative execution is used to fetch and decode instructions along both possible paths.
  - If the branch is mispredicted, the pipeline flushes the incorrect instructions.
  ```verilog
    module IR_Src_branches(
    input [31:0] instruction_EXE, instruction_ID,
    input zero, stall,
    output reg IRSrc_E, IRSrc_D
  );
  ```
- **Pipeline Flushing**:
  - Instructions fetched speculatively are flushed if the branch decision invalidates them.
### 3. **NOP Injection**
  - NOP instructions are dynamically injected into the pipeline to resolve dependencies that cannot be handled by forwarding or stalling.
---
## Testing and Validation

### Test Program
A custom MIPS assembly program was written to find the largest element in an array of 10 signed 32-bit integers. Key features of the program include:
- **Initialization**: The array is stored in data memory, and registers are initialized for tracking the maximum value and the loop index.
- **Loop and Conditional Logic**: A loop iterates through the array, comparing each element to the current maximum value, and updates the maximum value if a larger element is found.
- **Output**: At the end of execution, the largest array element is stored in a specific register.

### Simulation Results
- **Correct Execution**: The simulation verified that the largest element was correctly identified and stored in the designated register.
- **Pipeline Behavior**: Data and control hazards were managed efficiently with the implemented techniques:
  - **Data Forwarding** minimized stalls.
  - **Speculative Execution** reduced branch penalties.
  - **Stalls and NOPs** ensured correctness where forwarding wasn’t applicable.

### Performance Insights
- **Pipeline Efficiency**: The combination of forwarding and speculative execution significantly improved performance compared to introducing stalls alone.
- **Hazard Resolution**: Effective resolution of both data and control hazards ensured minimal disruptions in pipeline flow.

---

## Project Files

- **Verilog Files**:
  - `pipeline_mips.v`: Main pipeline design and integration.
  - `alu.v`: ALU implementation for arithmetic and logic operations.
  - `stall_controller.v`: Module for detecting and managing stalls.
  - `bypass_mux.v`: Logic for data forwarding to resolve dependencies.
- **Test Files**:
  - `test_program.s`: MIPS assembly code for finding the largest element in an array.
  - `instructionmemory.txt`: Machine code of the test program for loading into instruction memory.
  - `datamemory.txt`: Initial memory state for simulating data access.
- **Reports**:
  - `CA2.pdf`: Problem statement and exercise requirements.
  - `Gozaresh.pdf`: Comprehensive report on design methodology, implementation details, and results.

---

## How to Run

### Prerequisites
- **Verilog Simulator**: Use a tool such as ModelSim or Quartus for simulation.
- **Required Files**: Ensure `instructionmemory.txt` and `datamemory.txt` are correctly loaded into their respective modules.

### Steps
1. Load all Verilog files into the simulator.
2. Compile the project.
3. Initialize memory modules using `instructionmemory.txt` and `datamemory.txt`.
4. Run the simulation.
5. Observe the register file and memory contents to verify the results.

---

## Challenges and Solutions

1. **Control Signal Complexity**:
   - Addressed through systematic debugging and modular design of the control unit.
2. **Data Hazard Resolution**:
   - Forwarding logic was iteratively tested and improved to handle all dependency scenarios effectively.
3. **Branch Hazard Mitigation**:
   - Combined speculative execution with pipeline flushing to achieve optimal performance without compromising correctness.
4. **Verification Issues**:
   - Simulated edge cases and adjusted control logic for instructions with unusual dependencies.

---

## Author

Ali Ghorbani - [GitHub Profile](https://github.com/Alighorbani1380)

