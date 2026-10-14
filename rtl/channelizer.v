`timescale 1ns/1ps
//==============================================================================
// channelizer.v   (rtl-channelizer 4단계)
// 입력 ROM 재생 -> 다상 FIR(pfb_fir) -> 16점 FFT(fft16_sdf) -> 16비트 채널 출력
//
//   ROM: N_SAMPLES x 32비트 {I, Q} Q1.15 (M10K 추론 템플릿: 동기 읽기, 리셋 없음)
//   페이싱: run이 1이면 HOP_DIV 클록마다 8샘플을 연속으로 FIR에 넣는다 (홉 1개)
//           fast가 1이면 HOP_DIV_FAST(기본 16 = 최대 처리량, 입력 25 MSPS 상당) 클록마다 (7단계 보드 데모용)
//           처리는 항목 단위라 간격이 바뀌어도 출력은 같다 (재생 중에 바꿔도 되지만, 바꾸는 순간 한 홉이 길어질 수 있음)
//   비우기: 마지막 홉 뒤에 0 홉 하나를 더 넣어 FFT 파이프라인을 비우고, 그 홉의 출력은 버린다
//   출력: 채널 샘플 하나씩, 홉마다 FFT 출력 위치 j = 0..15 순서(채널 k = bitrev(j))
//         ch_i/ch_q = sat16(FFT 출력 Q3.15)  -> Q1.15 (|X| >= 1일 때만 포화, out_sat에 기록)
//   chz_model.channelizer_bit()과 비트 단위로 같다 (기대값 ch_ref_*.hex).
//
// 상태: done (모든 홉 출력 끝), fir_overrun (HOP_DIV < 16), fft_ovf, out_sat (설계상 모두 0)
// Quartus는 $readmemh 경로를 프로젝트(.qpf) 폴더 기준으로 찾는다 -> 프로젝트를 저장소 루트에.
//==============================================================================

module channelizer #(
    parameter ROM_FILE  = "sim_data/iq_rom_s1.hex",
    parameter COEF_FILE = "sim_data/coef_h.hex",
    parameter N_SAMPLES = 65536,          // 8의 배수
    parameter HOP_DIV   = 50,             // 16 이상
    parameter HOP_DIV_FAST = 16           // 16 이상
) (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               run,
    input  wire               fast,       // 1이면 HOP_DIV_FAST 간격

    output reg                ch_valid,
    output reg  signed [15:0] ch_i,
    output reg  signed [15:0] ch_q,
    output reg         [3:0]  ch_k,       // 채널 번호 0..15 (8..15는 음의 주파수 k-16)
    output reg         [3:0]  ch_pos,     // 홉 안 출력 위치 j
    output reg                ch_last,    // j == 15
    output reg         [15:0] ch_hop,     // 홉 번호

    output reg                done,
    output wire               fir_overrun,
    output wire               fft_ovf,
    output reg                out_sat
);

    localparam N_HOPS = N_SAMPLES / 8;
    localparam AW     = $clog2(N_SAMPLES);

    //--------------------------------------------------------------------
    // 입력 ROM과 페이싱
    //--------------------------------------------------------------------
    (* ramstyle = "M10K" *) reg [31:0] rom [0:N_SAMPLES-1];
    initial $readmemh(ROM_FILE, rom);

    reg [31:0] rom_q;
    reg [16:0] rd_idx;                    // 다음에 읽을 샘플 번호 (비우기 홉까지 N_SAMPLES + 8)
    always @(posedge clk) begin
        rom_q <= rom[rd_idx[AW-1:0]];
    end

    reg  [7:0] div_cnt;
    reg  [3:0] burst;                     // 이번 홉에서 남은 샘플 수
    reg        s_valid;
    reg        s_zero;                    // 비우기 홉 (ROM 대신 0)
    wire       src_more = (rd_idx < N_SAMPLES + 8);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            div_cnt <= 8'd0; burst <= 4'd0; rd_idx <= 17'd0; s_valid <= 1'b0; s_zero <= 1'b0;
        end else begin
            if (run && src_more) begin
                div_cnt <= (div_cnt == (fast ? HOP_DIV_FAST - 1 : HOP_DIV - 1)) ? 8'd0 : div_cnt + 8'd1;
                if (div_cnt == 8'd0) burst <= 4'd8;
            end
            if (burst != 4'd0) begin
                burst   <= burst - 4'd1;
                rd_idx  <= rd_idx + 17'd1;
                s_valid <= 1'b1;
                s_zero  <= (rd_idx >= N_SAMPLES);
            end else begin
                s_valid <= 1'b0;
            end
        end
    end

    wire signed [15:0] s_i = s_zero ? 16'sd0 : rom_q[31:16];
    wire signed [15:0] s_q = s_zero ? 16'sd0 : rom_q[15:0];

    //--------------------------------------------------------------------
    // FIR -> FFT
    //--------------------------------------------------------------------
    wire               f_valid, f_odd, f_last;
    wire signed [17:0] f_i, f_q;
    wire [3:0]         f_idx;

    pfb_fir #(.COEF_FILE(COEF_FILE)) u_fir (
        .clk(clk), .rst_n(rst_n),
        .s_valid(s_valid), .s_i(s_i), .s_q(s_q),
        .m_valid(f_valid), .m_i(f_i), .m_q(f_q), .m_idx(f_idx),
        .m_odd(f_odd), .m_last(f_last), .overrun(fir_overrun)
    );

    wire               x_valid, x_last;
    wire signed [17:0] x_r, x_i;
    wire [3:0]         x_pos, x_bin;

    fft16_sdf u_fft (
        .clk(clk), .rst_n(rst_n),
        .s_valid(f_valid), .s_r(f_i), .s_i(f_q),
        .m_valid(x_valid), .m_r(x_r), .m_i(x_i),
        .m_pos(x_pos), .m_bin(x_bin), .m_last(x_last), .ovf(fft_ovf)
    );

    //--------------------------------------------------------------------
    // 출력: 16비트 포화, 비우기 홉 출력 버림
    //--------------------------------------------------------------------
    function signed [15:0] sat16;
        input signed [17:0] v;
        begin
            if (v > 18'sd32767)       sat16 = 16'sd32767;
            else if (v < -18'sd32768) sat16 = -16'sd32768;
            else                      sat16 = v[15:0];
        end
    endfunction

    reg [16:0] blk;                        // FFT 출력 블록 번호
    wire       keep = x_valid && (blk < N_HOPS);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            blk <= 17'd0; ch_valid <= 1'b0; ch_i <= 16'sd0; ch_q <= 16'sd0;
            ch_k <= 4'd0; ch_pos <= 4'd0; ch_last <= 1'b0; ch_hop <= 16'd0;
            done <= 1'b0; out_sat <= 1'b0;
        end else begin
            ch_valid <= keep;
            if (keep) begin
                ch_i    <= sat16(x_r);
                ch_q    <= sat16(x_i);
                ch_k    <= x_bin;
                ch_pos  <= x_pos;
                ch_last <= x_last;
                ch_hop  <= blk[15:0];
                if (x_r > 18'sd32767 || x_r < -18'sd32768 || x_i > 18'sd32767 || x_i < -18'sd32768)
                    out_sat <= 1'b1;
            end
            if (x_valid && x_last) begin
                blk <= blk + 17'd1;
                if (blk == N_HOPS - 1) done <= 1'b1;
            end
        end
    end

endmodule
