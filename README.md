[README.pdf](https://github.com/user-attachments/files/17907593/README.pdf)


This is the final project of NC State ECE 564 ASIC and FPGA Design.
This project focuses on implementing key components of the Transformer architecture in 
hardware, specifically the self-attention mechanism, which is a combination of multiple matrix multiplication.

In this project, I completed the pre-synthesis simulation and synthesize in Design Vision compiler.
To maximize the clock frequency, the DeisgnWare pipelined multiplication-accumulation is employed, which leads to 0.65 ns and 400 um2 improvement in my design.

The synthesis report is located at /synthesis/reports .
The synthesis script is at /synthesis .
