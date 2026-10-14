`timescale 1ns/1ps
//==============================================================================
// cordic_pipe.v   (rtl-channelizer 5단계)
// 파이프라인 CORDIC (Vectoring): 클록당 1샘플, 지연 18클록.
// cordic_engine.v(반복형, 이전 프로젝트에서 검증)의 Vectoring 모드를 그대로 펼친 구조라
// 출력이 비트 단위로 같아야 한다 (tb_cordic_pipe가 두 모듈을 나란히 돌려 비교).
//
//   0단 : x_in < 0 이면 (x, y) -> (-x, -y), z = (y_in < 0) ? -pi : +pi  (pi = 32767, -pi = -32768)
//   1~16단 : 반복 i = 0..15, y < 0 이면 x -= y>>>i, y += x>>>i, z -= atan(2^-i)
//                           아니면     x += y>>>i, y -= x>>>i, z += atan(2^-i)   (x, y 18비트, z 16비트, 넘치면 감김)
//   17단 : 크기 = K*|s|를 16비트로 포화, 위상 = z (Q1.15, pi = 32768)
// 태그(채널 번호 등 TAG_W비트)를 데이터와 같은 지연으로 함께 보낸다.
//==============================================================================

module cordic_pipe #(
    parameter TAG_W = 8
) (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               in_valid,
    input  wire signed [15:0] in_x,          // I
    input  wire signed [15:0] in_y,          // Q
    input  wire [TAG_W-1:0]   in_tag,
    output reg                out_valid,
    output reg  signed [15:0] out_mag,       // K*|s| (포화)
    output reg  signed [15:0] out_phase,     // 위상
    output reg  [TAG_W-1:0]   out_tag
);

    reg signed [17:0] x [0:16];
    reg signed [17:0] y [0:16];
    reg signed [15:0] z [0:16];
    reg               v [0:16];
    reg [TAG_W-1:0]   tg [0:16];

    function [15:0] atan_lut;
        input [3:0] i;
        begin
            case (i)
                4'd0:  atan_lut = 16'h2000;  4'd1:  atan_lut = 16'h12E4;
                4'd2:  atan_lut = 16'h09FB;  4'd3:  atan_lut = 16'h0511;
                4'd4:  atan_lut = 16'h028B;  4'd5:  atan_lut = 16'h0146;
                4'd6:  atan_lut = 16'h00A3;  4'd7:  atan_lut = 16'h0051;
                4'd8:  atan_lut = 16'h0029;  4'd9:  atan_lut = 16'h0014;
                4'd10: atan_lut = 16'h000A;  4'd11: atan_lut = 16'h0005;
                4'd12: atan_lut = 16'h0003;  4'd13: atan_lut = 16'h0001;
                4'd14: atan_lut = 16'h0001;  default: atan_lut = 16'h0000;
            endcase
        end
    endfunction

    wire signed [17:0] x_ext = {{2{in_x[15]}}, in_x};
    wire signed [17:0] y_ext = {{2{in_y[15]}}, in_y};

    // 0단: 사전 회전
    always @(posedge clk) begin
        if (in_x < 0) begin
            x[0] <= -x_ext;
            y[0] <= -y_ext;
            z[0] <= (in_y < 0) ? 16'sh8000 : 16'sh7FFF;
        end else begin
            x[0] <= x_ext;
            y[0] <= y_ext;
            z[0] <= 16'sh0000;
        end
        tg[0] <= in_tag;
    end

    // 1~16단: 반복
    genvar i;
    generate
        for (i = 0; i < 16; i = i + 1) begin : g_it
            localparam [3:0] II = i;
            wire signed [17:0] xs = x[i] >>> i;
            wire signed [17:0] ys = y[i] >>> i;
            wire [15:0]        a  = atan_lut(II);
            always @(posedge clk) begin
                if (y[i] < 0) begin
                    x[i+1] <= x[i] - ys;
                    y[i+1] <= y[i] + xs;
                    z[i+1] <= z[i] - $signed(a);
                end else begin
                    x[i+1] <= x[i] + ys;
                    y[i+1] <= y[i] - xs;
                    z[i+1] <= z[i] + $signed(a);
                end
                tg[i+1] <= tg[i];
            end
        end
    endgenerate

    // 유효 신호 (리셋 있음)
    integer k;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (k = 0; k <= 16; k = k + 1) v[k] <= 1'b0;
            out_valid <= 1'b0;
        end else begin
            v[0] <= in_valid;
            for (k = 1; k <= 16; k = k + 1) v[k] <= v[k-1];
            out_valid <= v[16];
        end
    end

    // 17단: 포화
    always @(posedge clk) begin
        out_mag   <= (x[16] > 18'sh07FFF) ? 16'sh7FFF : ((x[16] < -18'sh08000) ? -16'sh8000 : x[16][15:0]);
        out_phase <= z[16];
        out_tag   <= tg[16];
    end

endmodule
