`timescale 1ns/1ps
//==============================================================================
// pdw_fifo.v   (rtl-channelizer 6단계)
// PDW FIFO (show-ahead). 시분할 검출기는 한 클록에 PDW를 최대 1개 내므로 중재기 없이 FIFO 하나면 된다.
// 프레임 인코더는 PDW 하나에 22클록 이상 걸리므로, 같은 시각에 끝난 동시 펄스들을 여기서 기다리게 한다.
//   가득 찼을 때 들어온 PDW는 버리고 ovf_cnt를 올린다(정상 시나리오에서 0).
//   max_level: 지금까지의 최대 점유 수 (깊이 선택 근거로 기록)
//==============================================================================

module pdw_fifo #(
    parameter W     = 100,
    parameter DEPTH = 32,
    parameter AW    = 5
) (
    input  wire          clk,
    input  wire          rst_n,
    input  wire          in_valid,
    input  wire [W-1:0]  in_data,
    output wire          out_valid,
    output wire [W-1:0]  out_data,
    input  wire          out_ready,
    output reg  [15:0]   ovf_cnt,
    output reg  [AW:0]   max_level
);
    reg [W-1:0]  mem [0:DEPTH-1];
    reg [AW-1:0] wp, rp;
    reg [AW:0]   level;

    wire full = (level == DEPTH);
    wire push = in_valid && !full;
    wire pop  = out_valid && out_ready;

    assign out_valid = (level != 0);
    assign out_data  = mem[rp];

    always @(posedge clk) begin
        if (push) mem[wp] <= in_data;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wp <= {AW{1'b0}}; rp <= {AW{1'b0}}; level <= {(AW+1){1'b0}};
            ovf_cnt <= 16'd0; max_level <= {(AW+1){1'b0}};
        end else begin
            if (push) wp <= wp + 1'b1;
            if (pop)  rp <= rp + 1'b1;
            case ({push, pop})
                2'b10:   level <= level + 1'b1;
                2'b01:   level <= level - 1'b1;
                default: level <= level;
            endcase
            if (in_valid && full) ovf_cnt <= ovf_cnt + 16'd1;
            if (level > max_level) max_level <= level;
        end
    end
endmodule
