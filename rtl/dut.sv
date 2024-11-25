`include "../rtl/common/common.vh"
`include "/mnt/apps/public/COE/synopsys_apps/syn/T-2022.03-SP4/dw/sim_ver/DW_piped_mac.v"




module MyDesign(
//System signals
  input wire reset_n                      ,  
  input wire clk                          ,

//Control signals
  input wire dut_valid                    , 
  output wire dut_ready                   ,

//input SRAM interface
  output wire                           dut__tb__sram_input_write_enable  ,               // 0 for Read
  output wire [`SRAM_ADDR_RANGE     ]   dut__tb__sram_input_write_address ,               // x
  output wire [`SRAM_DATA_RANGE     ]   dut__tb__sram_input_write_data    ,               // x
  output wire [`SRAM_ADDR_RANGE     ]   dut__tb__sram_input_read_address  , 
  input  wire [`SRAM_DATA_RANGE     ]   tb__dut__sram_input_read_data     ,               // Delay for one cycle

//weight SRAM interface
  output wire                           dut__tb__sram_weight_write_enable  ,              // 0 for Read
  output wire [`SRAM_ADDR_RANGE     ]   dut__tb__sram_weight_write_address ,              // x
  output wire [`SRAM_DATA_RANGE     ]   dut__tb__sram_weight_write_data    ,              // x
  output wire [`SRAM_ADDR_RANGE     ]   dut__tb__sram_weight_read_address  ,              //
  input  wire [`SRAM_DATA_RANGE     ]   tb__dut__sram_weight_read_data     ,              // Delay for one cycle

//result SRAM interface
  output wire                           dut__tb__sram_result_write_enable  ,              // 1 for write
  output wire [`SRAM_ADDR_RANGE     ]   dut__tb__sram_result_write_address ,              
  output wire [`SRAM_DATA_RANGE     ]   dut__tb__sram_result_write_data    ,              // Same cycle for data and address
  output wire [`SRAM_ADDR_RANGE     ]   dut__tb__sram_result_read_address  ,              // x
  input  wire [`SRAM_DATA_RANGE     ]   tb__dut__sram_result_read_data     ,              // x
//result SRAM interface
  output wire                           dut__tb__sram_scratchpad_write_enable  ,
  output wire [`SRAM_ADDR_RANGE    ]    dut__tb__sram_scratchpad_write_address ,
  output wire [`SRAM_DATA_RANGE    ]    dut__tb__sram_scratchpad_write_data    ,
  output wire [`SRAM_ADDR_RANGE    ]    dut__tb__sram_scratchpad_read_address  , 
  input  wire [`SRAM_DATA_RANGE    ]    tb__dut__sram_scratchpad_read_data                    // x
);

parameter IDLE = 0;
parameter Get_Dat = 1;
parameter ACCUM = 2;
parameter WR = 3;

wire Row_done_en;
wire EndCalc;
wire [`SRAM_ADDR_RANGE] Size_W, Size_R, Size_B;
reg  [`SRAM_DATA_RANGE] A;
reg [`SRAM_DATA_RANGE] B;
reg [`SRAM_ADDR_RANGE] add_Base_W, add_Base_R;
reg [`SRAM_ADDR_RANGE] Count2_add;
wire [`SRAM_ADDR_RANGE] Count1_minus, Count2_minus;


reg [1:0] cs, ns;
reg special;
reg Row_done;
reg ACCUM_en;
reg [2:0] Q_K_V_S_Z_reg;
reg [15:0] Base_W;
reg [15:0] Base_R;
reg [15:0] Base_S;
reg [15:0] RowNum_reg;
reg [15:0] ColNum_reg;
reg [15:0] XNum_reg;


reg [15:0] Count1;
reg [15:0] Count2;
reg [15:0] Count3;
wire [`SRAM_DATA_RANGE] accum_result;

// Useless
wire [2:0] pipe_census_inst;

always_ff @(posedge clk or negedge reset_n) begin
  if(!reset_n)
    cs <= 0;

  else
    cs <= ns;
end

always_comb begin
  case(cs)
    IDLE: ns = (dut_valid) ? Get_Dat : IDLE;
    Get_Dat: ns = (Count1==1) ? ACCUM : Get_Dat;
    ACCUM: ns = (EndCalc) ? (Q_K_V_S_Z_reg==4) ? IDLE : Get_Dat :
                            (XNum_reg==0) ? WR : ACCUM; 
    WR: ns = ACCUM;
    default: ns = IDLE;
  endcase
end

//--------------- Output Decoder-----------------
// Calculation ends when A reachs (RowNum) and B reachs(XNum * ColNum)

// Count1 for matrix A
assign Count1_en = (cs==Get_Dat && !Count1[0]) || (cs==ACCUM && XNum_reg!=0);
assign Count1_stay = (cs==WR && (Row_done_en || Row_done && Q_K_V_S_Z_reg==4)) || (cs==ACCUM && XNum_reg==0) || (cs==Get_Dat && Count1[0]);
assign Count1_deduct = (cs==WR);

// Count2 for matrix B
assign Count2_en = (cs==Get_Dat && Count1[0]) || (cs==ACCUM && XNum_reg!=0);
assign Count2_stay = (cs==ACCUM && XNum_reg==0) || (cs==WR && Count2!=Size_W);
assign Count2_deduct = (cs==WR && (Row_done_en || Row_done && Q_K_V_S_Z_reg==4));
assign Row_done_en = special && Count2 >= ((Q_K_V_S_Z_reg==4) ? Size_R : Size_W);

// Count3 for matrix C
assign Count3_en = (cs==WR);
assign EndCalc = (Count3==Size_R) && (cs==ACCUM);

// Input control
assign RowNum_reg_en = (cs==Get_Dat && Count1[0]);
assign XNum_reg_en = (cs==Get_Dat && Count1[0]) || (cs==WR);
assign ColNum_reg_en = (cs==Get_Dat && Count1[0]);
assign accum_result_en = (cs==ACCUM);
assign CountDown = (cs==ACCUM && XNum_reg!=0);
assign base_R_en = (cs==Get_Dat && Count1[0] && Q_K_V_S_Z_reg!=0);
assign base_W_en = (cs==Get_Dat && Count1[0] && Q_K_V_S_Z_reg!=0);
assign base_W_force = (cs==Get_Dat && Count1[0] && (Q_K_V_S_Z_reg==3));
assign Base_S_en = (cs==Get_Dat && Count1[0] && Q_K_V_S_Z_reg==4);

// SRAM control
assign dut_ready = (cs==0);

assign dut__tb__sram_input_read_address = (XNum_reg==0 && cs==ACCUM) ? 0 : Count1;
assign dut__tb__sram_input_write_enable = 0;

assign dut__tb__sram_weight_write_enable = 0;
assign dut__tb__sram_weight_read_address = (cs==Get_Dat || XNum_reg==0 && cs==ACCUM) ? 0 : (Count2 + Base_W);

assign dut__tb__sram_result_write_enable = (cs==WR);
assign dut__tb__sram_result_write_address = Count3 + Base_R;
assign dut__tb__sram_result_write_data = accum_result;
assign dut__tb__sram_result_read_address = Count2 + Base_W;      
// Count2 + Base_W + ((QKVZR==4) ? ((Count2 - 1) * ColNum) : 0) = ((Q==4) ? (-ColNum + Count2<<1) : Count2) + Base_W

assign dut__tb__sram_scratchpad_write_address = Count3 + Base_R;
assign dut__tb__sram_scratchpad_write_enable = (cs==WR);
assign dut__tb__sram_scratchpad_write_data = accum_result;
assign dut__tb__sram_scratchpad_read_address = Count1 - 1 + Base_S;

always @(*) begin
  case(Q_K_V_S_Z_reg)
    3: A = tb__dut__sram_scratchpad_read_data;
    4: A = tb__dut__sram_scratchpad_read_data;
    default: A = tb__dut__sram_input_read_data;
  endcase
end

always @(*) begin
  case(Q_K_V_S_Z_reg)
    3: B = tb__dut__sram_result_read_data;
    4: B = tb__dut__sram_result_read_data;
    default: B = tb__dut__sram_weight_read_data;
  endcase
end

//assign A = (Q_K_V_S_Z_reg==3) ? tb__dut__sram_scratchpad_read_data : tb__dut__sram_input_read_data;
// assign B = (Q_K_V_S_Z_reg==3) ? tb__dut__sram_result_read_data : tb__dut__sram_weight_read_data;

// Combinational
assign Size_W = ((Q_K_V_S_Z_reg==3 || Q_K_V_S_Z_reg==4) ? tb__dut__sram_weight_read_data[15:0] : tb__dut__sram_input_read_data[15:0]) * ColNum_reg;
assign Size_R = RowNum_reg * ColNum_reg;
assign Count2_minus = (Q_K_V_S_Z_reg==4) ? ColNum_reg : Size_W;
assign Count1_minus = (Q_K_V_S_Z_reg==4) ? tb__dut__sram_input_read_data[31:16] : 
                      (Q_K_V_S_Z_reg==3) ? tb__dut__sram_weight_read_data[15:0] : tb__dut__sram_input_read_data[15:0];

always @(*)
    case(Q_K_V_S_Z_reg)
        1: add_Base_W = Size_W;
        2: add_Base_W = Size_W;
        4: add_Base_W = Size_W;
        default: add_Base_W = 'dx;
    endcase

always @(*)
    case(Q_K_V_S_Z_reg)
        1: add_Base_R = Size_R;
        2: add_Base_R = Size_R;
        3: add_Base_R = Size_R;
        4: add_Base_R = Size_R;
        default: add_Base_R = 'dx;
    endcase

always @(*) begin
  if(Q_K_V_S_Z_reg != 4)
    Count2_add = 1;
  else begin
    if(XNum_reg==1)
      Count2_add = -((RowNum_reg - 1) * ColNum_reg - 1);
    else
      Count2_add = (cs==Get_Dat) ? 1 : ColNum_reg;
  end
end

//Register
always @(posedge clk or negedge reset_n) begin
  if(!reset_n)
    special <= 0;
  else if(cs==IDLE)
    special <= 0;
  else if(cs==ACCUM && RowNum_reg!=1)
    special <= 1;
end

always @(posedge clk or negedge reset_n) begin
  if(!reset_n)
    Row_done <= 0;
  else if(cs==WR)
    Row_done <= 0;
  else if(Row_done_en)
    Row_done <= 1;
end

always @(posedge clk or negedge reset_n) begin
  if(!reset_n)
    Base_S <= 0;
  else if(cs==IDLE)
    Base_S <= 0;
  else if(Base_S_en)
    Base_S <= Base_R;
end

always @(posedge clk or negedge reset_n) begin
  if(!reset_n)
    Base_W <= 0;
  else if(cs==IDLE)
    Base_W <= 0;
  else if(base_W_force)
    Base_W <= Size_R - 1;
  else if(base_W_en)
    Base_W <= Base_W + add_Base_W;
end

always @(posedge clk or negedge reset_n) begin
  if(!reset_n)
    Base_R <= 0;
  else if(cs==IDLE)
    Base_R <= 0;
  else if(base_R_en)
    Base_R <= Base_R + add_Base_R;
end

always_ff @(posedge clk or negedge reset_n) begin
  if(!reset_n)
    Count1 <= 0;

  else if(EndCalc)
    Count1 <= 0;

  else if(Count1_stay)
    Count1 <= Count1;

  else if(Count1_en)
    Count1 <= Count1 + 1;

  else if(Count1_deduct)
    Count1 <= Count1 - Count1_minus;
end

always_ff @(posedge clk or negedge reset_n) begin
  if(!reset_n)
    Count2 <= 0;

  else if(EndCalc)
    Count2 <= 0;

  else if(Count2_deduct)
    Count2 <= Count2 - Count2_minus;

  else if(Count2_stay)
    Count2 <= Count2;

  else if(Count2_en)
    Count2 <= Count2 + Count2_add;
end

always_ff @(posedge clk or negedge reset_n) begin
  if(!reset_n)
    Count3 <= 0;

  else if(EndCalc)
    Count3 <= 0;

  else if(Count3_en)
    Count3 <= Count3 + 1;
end

always_ff @(posedge clk or negedge reset_n) begin
  if(!reset_n)
    ACCUM_en <= 0;

  else
    ACCUM_en <= accum_result_en;
end

always_ff @(posedge clk or negedge reset_n) begin
  if(!reset_n)
    RowNum_reg <= 0;

  else if(RowNum_reg_en)
    RowNum_reg <= tb__dut__sram_input_read_data[31:16];
end

always_ff @(posedge clk or negedge reset_n) begin
  if(!reset_n)
    XNum_reg <= 0;

  else if(EndCalc)
    XNum_reg <= 0;

  else if(XNum_reg_en)
    XNum_reg <= (Q_K_V_S_Z_reg==4) ? tb__dut__sram_input_read_data[31:16] : 
                (Q_K_V_S_Z_reg==3) ? tb__dut__sram_weight_read_data[15:0] : tb__dut__sram_input_read_data[15:0];

  else
    XNum_reg <= XNum_reg - 1;
end

always_ff @(posedge clk or negedge reset_n) begin
  if(!reset_n)
    ColNum_reg <= 0;

  else if(ColNum_reg_en)
    ColNum_reg <= (Q_K_V_S_Z_reg==3) ? tb__dut__sram_input_read_data[31:16] : tb__dut__sram_weight_read_data[15:0];
end

always @(posedge clk or negedge reset_n) begin
  if(!reset_n)
    Q_K_V_S_Z_reg <= 0;
  else if(cs==IDLE)
    Q_K_V_S_Z_reg <= 0;
  else if(EndCalc)
    Q_K_V_S_Z_reg <= Q_K_V_S_Z_reg + 1;
end

DW_piped_mac_inst 
  mac( 
    .clk(clk), 
    .inst_rst_n(reset_n), 
    .inst_init_n(ACCUM_en), 
    .inst_clr_acc_n(1'b1), 
    .inst_a(A), 
    .inst_b(B), 
    .acc_inst(accum_result), 
    .inst_launch(ACCUM_en), 
    .inst_launch_id(1'b0), 
    .pipe_full_inst(pipe_full_inst), 
    .pipe_ovf_inst(pipe_ovf_inst), 
    .inst_accept_n(1'b0), 
    .arrive_inst(arrive_inst), 
    .arrive_id_inst(arrive_id_inst), 
    .push_out_n_inst(push_out_n_inst), 
    .pipe_census_inst(pipe_census_inst)
);

// always @(posedge clk or negedge reset_n) begin
//   if(!reset_n)
//     accum_result <= 0;
//   else if(ACCUM_en)
//     accum_result <= accum_result + A * B;
//   else
//     accum_result <= 0;
// end

// always @(posedge clk or negedge reset_n) begin
//   if(!reset_n)
//     C_reg <= 0;
//   else
//     C_reg <= A*B;
// end
// assign accum_temp = accum_result + C_reg[31:0];
// always @(posedge clk or negedge reset_n) begin
//   if(!reset_n)
//     accum_result <= 0;
//   else if(ACCUM_en)
//     accum_result <= accum_temp;
//   else
//     accum_result <= 0;
// end


endmodule


module DW_piped_mac_inst( 
    clk, inst_rst_n, inst_init_n, 
    inst_clr_acc_n, inst_a, inst_b, acc_inst, inst_launch, inst_launch_id, 
    pipe_full_inst, pipe_ovf_inst, inst_accept_n, arrive_inst, 
    arrive_id_inst, push_out_n_inst, pipe_census_inst
);
    parameter inst_a_width = 32;
    parameter inst_b_width = 32;
    parameter inst_acc_width = 32;
    parameter inst_tc = 0;
    parameter inst_pipe_reg = 0;
    parameter inst_id_width = 1;
    parameter inst_no_pm = 0;
    parameter inst_op_iso_mode = 0;
    input clk;
    input inst_rst_n;
    input inst_init_n;
    input inst_clr_acc_n;
    input [inst_a_width-1 : 0] inst_a;
    input [inst_b_width-1 : 0] inst_b;
    output [inst_acc_width-1 : 0] acc_inst;
    input inst_launch;
    input [inst_id_width-1 : 0] inst_launch_id;
    output pipe_full_inst;
    output pipe_ovf_inst;
    input inst_accept_n;
    output arrive_inst;
    output [inst_id_width-1 : 0] arrive_id_inst;
    output push_out_n_inst;
    output [2 : 0] pipe_census_inst;

    // Instance of DW_piped_mac 
    DW_piped_mac #(
            inst_a_width, inst_b_width, inst_acc_width,
            inst_tc, inst_pipe_reg, inst_id_width, inst_no_pm
    )
    U1 ( 
        .clk(clk), 
        .rst_n(inst_rst_n), 
        .init_n(inst_init_n), 
        .clr_acc_n(inst_clr_acc_n), 
        .a(inst_a), 
        .b(inst_b), 
        .acc(acc_inst), 
        .launch(inst_launch), 
        .launch_id(inst_launch_id), 
        .pipe_full(pipe_full_inst), 
        .pipe_ovf(pipe_ovf_inst), 
        .accept_n(inst_accept_n), 
        .arrive(arrive_inst), 
        .arrive_id(arrive_id_inst), 
        .push_out_n(push_out_n_inst), 
        .pipe_census(pipe_census_inst)
    );
endmodule: DW_piped_mac_inst