`timescale 1ns/1ps
//==============================================================================
// chz_top.v   (rtl-channelizer 6단계 최상위)
// 입력 ROM -> 채널라이저(FIR + FFT) -> 파이프라인 CORDIC -> 시분할 검출·소유 판정 -> PDW FIFO
//          -> 22바이트 텔레메트리 프레임 바이트 스트림
// 상태 출력은 테스트벤치, 보드 데모(7단계), Signal Tap 관측에 쓴다.
//==============================================================================

module chz_top #(
    parameter ROM_FILE  = "sim_data/iq_rom_s1.hex",
    parameter COEF_FILE = "sim_data/coef_h.hex",
    parameter N_SAMPLES = 65536,
    parameter HOP_DIV   = 50,
    parameter [15:0] CRC_INIT = 16'hFFFF
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        run,
    input  wire        fast,           // 1이면 최대 처리량 간격(16클록/홉)

    output wire [7:0]  m_data,
    output wire        m_valid,
    input  wire        m_ready,
    output wire        m_last,

    output reg         all_done,       // 모든 홉 처리, FIFO 비었음, 프레임 송신 끝
    output wire        fir_overrun,
    output wire        fft_ovf,
    output wire        out_sat,
    output wire        det_overrun,
    output wire [15:0] glitch_cnt,
    output wire [15:0] not_owned_cnt,
    output wire [15:0] fifo_ovf,
    output wire [5:0]  fifo_max,
    output wire [15:0] frame_cnt,
    output wire [31:0] hop_cnt
);
    localparam N_HOPS = N_SAMPLES / 8;

    wire               ch_valid, ch_last, ch_done;
    wire signed [15:0] ch_i, ch_q;
    wire [3:0]         ch_k, ch_pos;
    wire [15:0]        ch_hop;

    channelizer #(.ROM_FILE(ROM_FILE), .COEF_FILE(COEF_FILE), .N_SAMPLES(N_SAMPLES), .HOP_DIV(HOP_DIV)) u_chz (
        .clk(clk), .rst_n(rst_n), .run(run), .fast(fast),
        .ch_valid(ch_valid), .ch_i(ch_i), .ch_q(ch_q), .ch_k(ch_k), .ch_pos(ch_pos), .ch_last(ch_last), .ch_hop(ch_hop),
        .done(ch_done), .fir_overrun(fir_overrun), .fft_ovf(fft_ovf), .out_sat(out_sat)
    );

    wire        pdw_valid;
    wire [3:0]  pdw_ch;
    wire [31:0] pdw_toa;
    wire [15:0] pdw_pw, pdw_pa, pdw_freq, pdw_flags;

    chz_detect u_det (
        .clk(clk), .rst_n(rst_n),
        .ch_valid(ch_valid), .ch_i(ch_i), .ch_q(ch_q), .ch_k(ch_k), .ch_last(ch_last),
        .c_valid(), .c_mag(), .c_phase(), .c_k(), .c_last(),
        .pdw_valid(pdw_valid), .pdw_ch(pdw_ch), .pdw_toa(pdw_toa), .pdw_pw(pdw_pw),
        .pdw_pa(pdw_pa), .pdw_freq(pdw_freq), .pdw_flags(pdw_flags),
        .raw_valid(), .raw_own(), .raw_ch(), .raw_toa(), .raw_pw(), .raw_pa(), .raw_freq(), .raw_flags(),
        .glitch_cnt(glitch_cnt), .not_owned_cnt(not_owned_cnt), .hop_cnt(hop_cnt), .overrun(det_overrun)
    );

    wire         f_valid, f_ready;
    wire [99:0]  f_data;

    pdw_fifo #(.W(100), .DEPTH(32), .AW(5)) u_fifo (
        .clk(clk), .rst_n(rst_n),
        .in_valid(pdw_valid), .in_data({pdw_ch, pdw_toa, pdw_pw, pdw_pa, pdw_freq, pdw_flags}),
        .out_valid(f_valid), .out_data(f_data), .out_ready(f_ready),
        .ovf_cnt(fifo_ovf), .max_level(fifo_max)
    );

    chz_framer #(.CRC_INIT(CRC_INIT)) u_frm (
        .clk(clk), .rst_n(rst_n),
        .s_valid(f_valid), .s_ready(f_ready),
        .s_ch(f_data[99:96]), .s_toa(f_data[95:64]), .s_pw(f_data[63:48]), .s_pa(f_data[47:32]),
        .s_freq(f_data[31:16]), .s_flags(f_data[15:0]),
        .m_data(m_data), .m_valid(m_valid), .m_ready(m_ready), .m_last(m_last),
        .frame_cnt(frame_cnt)
    );

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) all_done <= 1'b0;
        else        all_done <= ch_done && (hop_cnt == N_HOPS) && !f_valid && !m_valid;
    end

endmodule
