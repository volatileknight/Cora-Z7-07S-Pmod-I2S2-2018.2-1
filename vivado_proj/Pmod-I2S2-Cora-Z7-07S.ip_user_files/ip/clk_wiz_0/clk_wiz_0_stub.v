// Copyright 1986-2018 Xilinx, Inc. All Rights Reserved.
// --------------------------------------------------------------------------------
// Tool Version: Vivado v.2018.2 (win64) Build 2258646 Thu Jun 14 20:03:12 MDT 2018
// Date        : Thu Sep 19 12:45:55 2019
// Host        : WK142 running 64-bit major release  (build 9200)
// Command     : write_verilog -force -mode synth_stub
//               D:/Github/Pmod-I2S2/Pmod-I2S2/cora-z7-07s/proj/Pmod-I2S2-Cora-Z7-07S.srcs/sources_1/ip/clk_wiz_0/clk_wiz_0_stub.v
// Design      : clk_wiz_0
// Purpose     : Stub declaration of top-level module interface
// Device      : xc7z007sclg400-1
// --------------------------------------------------------------------------------

// This empty module with port declaration file causes synthesis tools to infer a black box for IP.
// The synthesis directives are for Synopsys Synplify support to prevent IO buffer insertion.
// Please paste the declaration into a Verilog source file or add the file as an additional source.
module clk_wiz_0(axis_clk, clk_in1)
/* synthesis syn_black_box black_box_pad_pin="axis_clk,clk_in1" */;
  output axis_clk;
  input clk_in1;
endmodule
