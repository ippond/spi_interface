`timescale 1ns/100ps
module spi_interface
#(
parameter C_CLK_FREQ      = 50000000 ,
parameter C_SPI_FREQ      = 200000   ,
parameter C_DATA_WE_LEN   = 1        , //这两个参数是多个cs共享的，选择多个cs的最大值
parameter C_DATA_RD_LEN   = 1        ,
parameter C_CS_NUM        = 1        ,
parameter C_MOSI_DEFLEVEL = 0        ,
parameter C_SPI3          = 0
//parameter C_MSB           = 1
)
(
input                             I_clk       ,
input                             I_rst       ,
input      [C_DATA_WE_LEN*8-1:0]  I_wdata     ,
input                             I_spi_en    ,
input                             I_spi_wnr   ,
input                             I_keep_read ,
input                             I_cpol      ,
input                             I_cpha      ,
input                             I_spi_av    ,
input      [7:0]                  I_wdata_len , 
input      [7:0]                  I_rdata_len , //决定输出有效数据的位宽，若C_DATA_RD_LEN大于I_rdata_len，则输出I_rdata_len长度，其余为0
input      [C_CS_NUM-1:0]         I_cs_sel    ,
output reg [C_DATA_RD_LEN*8-1:0]  O_rdata     ,
output reg                        O_rdata_v   ,  //连续读每byte都输出，单次读读完输出
output reg                        O_spi_ready ,
input                             I_spi_miso  ,
output reg                        O_spi_sck   ,
output reg                        O_spi_mosi  ,
output reg [C_CS_NUM-1:0]         O_spi_cs    ,
inout                             IO_spi_data
);

localparam C_BIT_PERIOD = C_CLK_FREQ/C_SPI_FREQ-1;
localparam C_BIT_HALF_PERIOD = C_CLK_FREQ/C_SPI_FREQ/2-1;
localparam C_CLK_CWIDTH = F_width(C_BIT_HALF_PERIOD);
localparam C_PRE_CWIDTH = 4;
localparam C_PRE_LIMIT = 8;
localparam C_PRO_CWIDTH = 4;
localparam C_PRO_LIMIT = 8;
localparam C_IDLE = 0;
localparam C_PRE = 1;
localparam C_TRANS = 2;
localparam C_BREAK = 3;
localparam C_PRO = 4;

reg S_spi_clk = 0;
reg [C_CLK_CWIDTH-1:0] S_clk_cnt = 0;
reg S_clk_toggle = 0;
reg [C_DATA_WE_LEN*8-1:0] S_wdata = 0;
reg [7:0] S_shift_len = 0;
reg [7:0] S_shift_len2 = 0;
reg [C_DATA_WE_LEN*8-1:0] S_wdata_d = 0;
reg [C_DATA_WE_LEN*8-1:0] S_wdata2_d = 0;
reg [C_DATA_WE_LEN*8-1:0] S_wdata_slr = 0;
reg S_spi_en = 0;
reg S_spi_en_d = 0;
reg [10:0] S_bit_cnt = 0;
reg [7:0] S_addr_len = 0;
reg [7:0] S_rdata_len = 0;
reg [10:0] S_bit_cnt_limit = 0;
reg S_bit_limit_id = 0;
reg S_trans_over = 0;
reg S_break_id = 0;
reg S_keep_read = 0;
reg S_data_read = 0;
reg [2:0] S_state_cur = 0;
reg [2:0] S_state_next = 0;
reg [2:0] S_state_cur_d = 0;
reg [C_PRE_CWIDTH-1:0] S_pre_cnt = 0;
reg S_pre_over = 0;
reg [C_PRE_CWIDTH-1:0] S_pro_cnt = 0;
reg S_pro_over = 0;
reg [C_DATA_RD_LEN*8-1:0] S_rdata = 0;
reg S_rdata_v = 0;
reg [10:0] S_rcnt_limit = 0;
reg [10:0] S_rcnt = 0;
reg [C_CS_NUM-1:0] S_spi_cs = 0;
reg S_pre_cs = 0;
reg S_spi_wnr = 0;
wire S_spi_datai;

assign IO_spi_data = S_data_read ? 1'bz : O_spi_mosi;
assign S_spi_datai = C_SPI3 ? IO_spi_data : I_spi_miso;

always @(posedge I_clk)
begin
    if(I_spi_en)
        S_spi_cs <= I_cs_sel;
    if(S_state_cur == C_PRE && S_pre_cs)
        O_spi_cs <= ~S_spi_cs;
    else if(I_spi_en || S_state_cur == C_IDLE)
        O_spi_cs <= ~0;
    
    O_spi_sck <= S_spi_clk;
    if(S_state_cur == C_IDLE)
        O_spi_mosi <= C_MOSI_DEFLEVEL;
    else
        O_spi_mosi <= S_wdata_slr[C_DATA_WE_LEN*8-1];   
    
    //if(I_spi_en)
    //    S_rshift_len <= C_DATA_RD_LEN - I_rdata_len;
        
    if(S_state_cur == C_PRE || S_state_cur == C_BREAK)
        S_rdata <= 'd0;
    else if(S_data_read && S_clk_toggle && ((S_spi_clk == I_cpol && !I_cpha) || (S_spi_clk != I_cpol && I_cpha)))
        S_rdata <= {S_rdata[C_DATA_RD_LEN*8-2:0],S_spi_datai};
    S_rdata_v <= S_rcnt == S_rcnt_limit && S_data_read && S_clk_toggle && ((S_spi_clk == I_cpol && !I_cpha) || (S_spi_clk != I_cpol && I_cpha));
    if(S_rdata_v)
        O_rdata <= S_rdata;
    O_rdata_v <= S_rdata_v;     
    O_spi_ready <= S_state_cur == C_IDLE;
end

always @(posedge I_clk)
begin
    if(S_state_cur != C_TRANS)
        S_spi_clk <= I_cpol;
    else if((S_clk_toggle || (S_state_cur == C_TRANS && (S_state_cur_d == C_PRE || S_state_cur_d == C_BREAK) && I_cpha)) && !(S_bit_limit_id && I_cpha))
        S_spi_clk <= !S_spi_clk;
    
    if(S_state_cur == C_TRANS)
    begin
        if(S_clk_cnt == C_BIT_HALF_PERIOD)
            S_clk_cnt <= 'd0;
        else
            S_clk_cnt <= S_clk_cnt + 'd1;
    end
    else
        S_clk_cnt <= 'd0;
    S_clk_toggle <= S_clk_cnt == C_BIT_HALF_PERIOD;
end

always @(posedge I_clk)
begin
    if(I_spi_en)
    begin
        S_wdata <= I_wdata;
        S_shift_len <= C_DATA_WE_LEN - I_wdata_len;
        S_shift_len2 <= C_DATA_WE_LEN - I_rdata_len;
    end
    S_wdata_d <= S_wdata << (S_shift_len<<3);
    S_wdata2_d <= S_wdata << (S_shift_len2<<3);
    S_spi_en <= I_spi_en;
    S_spi_en_d <= S_spi_en;
    if(S_spi_en_d)
        S_wdata_slr <= S_wdata_d;
    else if(S_state_cur == C_BREAK)
        S_wdata_slr <= S_wdata2_d;
    else if(S_clk_toggle && ((S_spi_clk == !I_cpol && !I_cpha) || (S_spi_clk == I_cpol && I_cpha)))
        S_wdata_slr <= S_wdata_slr << 1;
    
    if(S_state_cur != C_TRANS)
        S_bit_cnt <= 'd0;
    else if(S_clk_toggle && ((S_spi_clk == I_cpol && !I_cpha) || (S_spi_clk != I_cpol && I_cpha)))
        S_bit_cnt <= S_bit_cnt + 'd1;
    
    if(I_spi_en)
    begin
        S_addr_len <= I_wdata_len - I_rdata_len;
        S_rdata_len <= I_rdata_len;
    end

    if(I_spi_en)
        S_bit_cnt_limit <= {I_wdata_len,3'd0};
    else if(S_state_cur == C_BREAK)
        S_bit_cnt_limit <= {S_rdata_len,3'd0};
    S_bit_limit_id <= S_bit_cnt_limit == S_bit_cnt && S_state_cur == C_TRANS;
    S_trans_over <= S_bit_limit_id && S_clk_toggle && !S_keep_read;
    S_break_id <= S_bit_limit_id && S_clk_toggle && S_keep_read;
    if(I_spi_en && I_spi_wnr && I_keep_read)
        S_keep_read <= 1'b1;
    if(I_spi_en)
        S_spi_wnr <= I_spi_wnr;
    
    if(S_state_cur == C_IDLE || S_state_cur == C_PRE)
        S_data_read <= 1'b0;
    else if(S_bit_cnt == {S_addr_len,3'd0})
        S_data_read <= S_spi_wnr;
        
    S_rcnt_limit <= {S_rdata_len,3'd0} - 'd1;   
    
    if(S_state_cur == C_PRE || S_state_cur == C_BREAK)
        S_rcnt <= 'd0;
    else if(S_data_read && S_clk_toggle && ((S_spi_clk == I_cpol && !I_cpha) || (S_spi_clk != I_cpol && I_cpha)))
    begin
        if(S_rcnt == S_rcnt_limit)
            S_rcnt <= 'd0;
        else
            S_rcnt <= S_rcnt + 'd1;
    end  
end

always @(posedge I_clk)
begin
    if(I_rst)
        S_state_cur <= C_IDLE;
    else
        S_state_cur <= S_state_next;
    S_state_cur_d <= S_state_cur;
end

always @(*)
begin
    case(S_state_cur)
    C_IDLE:
        if(I_spi_en)
            S_state_next = C_PRE;
        else
            S_state_next = C_IDLE;
    C_PRE:
        if(S_pre_over && I_spi_av)  //S_pre_over信号要考虑I_spi_av
            S_state_next = C_TRANS;
        else
            S_state_next = C_PRE;
    C_TRANS:
        if(I_spi_en)
            S_state_next = C_PRE;
        else if(S_break_id)
            S_state_next = C_BREAK;
        else if(S_trans_over)
            S_state_next = C_PRO;
        else
            S_state_next = C_TRANS;
    C_BREAK:
        if(I_spi_en)
            S_state_next = C_PRE;
        else if(I_spi_av)
            S_state_next = C_TRANS;
        else
            S_state_next = C_BREAK;
    C_PRO:
        if(I_spi_en)
            S_state_next = C_PRE;
        else if(S_pro_over)
            S_state_next = C_IDLE;
        else
            S_state_next = C_PRO;
    default:
        S_state_next = C_IDLE;
    endcase
end

always @(posedge I_clk)
begin
    if(S_state_cur == C_PRE)
    begin
        if(S_pre_cnt != C_PRE_LIMIT)
            S_pre_cnt <= S_pre_cnt + 'd1;
    end
    else
        S_pre_cnt <= 'd0;
    S_pre_over <= S_pre_cnt == C_PRE_LIMIT;
    S_pre_cs <= S_pre_cnt == C_PRE_LIMIT/2;
    
    if(S_state_cur == C_PRO)
        S_pro_cnt <= S_pro_cnt + 'd1;
    else
        S_pro_cnt <= 'd0;
    S_pro_over <= S_pro_cnt == C_PRO_LIMIT;
end

function integer F_width;
input integer I_num;
integer i;
begin
    for(i=0;2**i<=I_num;i=i+1)
    F_width = i;
    F_width = i;
end
endfunction

endmodule
