`timescale 1ns/1ps
//==============================================================================
// chz_detect.v   (rtl-channelizer 5단계)
// 채널 출력 스트림 -> 파이프라인 CORDIC -> 시분할 검출기(이웃 크기 버퍼, 소유 판정) -> PDW
// 입력은 channelizer의 ch_* 출력을 그대로 받는다 (홉마다 16채널, FFT 출력 순서).
//==============================================================================

module chz_detect #(
    parameter TH_HI      = 3000,
    parameter TH_LO      = 2000,
    parameter MIN_W      = 4,
    parameter END_LOW    = 2,
    parameter FREQ_SKIP  = 3,
    parameter FREQ_GUARD = 4
) (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               ch_valid,
    input  wire signed [15:0] ch_i,
    input  wire signed [15:0] ch_q,
    input  wire        [3:0]  ch_k,
    input  wire               ch_last,

    // CORDIC 출력 (테스트벤치 관측용)
    output wire               c_valid,
    output wire signed [15:0] c_mag,
    output wire signed [15:0] c_phase,
    output wire        [3:0]  c_k,
    output wire               c_last,

    output wire               pdw_valid,
    output wire        [3:0]  pdw_ch,
    output wire        [31:0] pdw_toa,
    output wire        [15:0] pdw_pw,
    output wire        [15:0] pdw_pa,
    output wire        [15:0] pdw_freq,
    output wire        [15:0] pdw_flags,

    output wire               raw_valid,
    output wire               raw_own,
    output wire        [3:0]  raw_ch,
    output wire        [31:0] raw_toa,
    output wire        [15:0] raw_pw,
    output wire        [15:0] raw_pa,
    output wire        [15:0] raw_freq,
    output wire        [15:0] raw_flags,

    output wire        [15:0] glitch_cnt,
    output wire        [15:0] not_owned_cnt,
    output wire        [31:0] hop_cnt,
    output wire               overrun
);

    wire [4:0] tag_out;

    cordic_pipe #(.TAG_W(5)) u_cordic (
        .clk(clk), .rst_n(rst_n),
        .in_valid(ch_valid), .in_x(ch_i), .in_y(ch_q), .in_tag({ch_last, ch_k}),
        .out_valid(c_valid), .out_mag(c_mag), .out_phase(c_phase), .out_tag(tag_out)
    );
    assign c_k    = tag_out[3:0];
    assign c_last = tag_out[4];

    pdw_detector_tdm #(
        .TH_HI(TH_HI), .TH_LO(TH_LO), .MIN_W(MIN_W), .END_LOW(END_LOW),
        .FREQ_SKIP(FREQ_SKIP), .FREQ_GUARD(FREQ_GUARD)
    ) u_det (
        .clk(clk), .rst_n(rst_n),
        .in_valid(c_valid), .in_mag(c_mag), .in_phase(c_phase), .in_k(c_k), .in_last(c_last),
        .pdw_valid(pdw_valid), .pdw_ch(pdw_ch), .pdw_toa(pdw_toa), .pdw_pw(pdw_pw),
        .pdw_pa(pdw_pa), .pdw_freq(pdw_freq), .pdw_flags(pdw_flags),
        .raw_valid(raw_valid), .raw_own(raw_own), .raw_ch(raw_ch), .raw_toa(raw_toa), .raw_pw(raw_pw),
        .raw_pa(raw_pa), .raw_freq(raw_freq), .raw_flags(raw_flags),
        .glitch_cnt(glitch_cnt), .not_owned_cnt(not_owned_cnt), .hop_cnt(hop_cnt), .overrun(overrun)
    );

endmodule
