`timescale 1ns/1ps
//==============================================================================
// pdw_detector_tdm.v   (rtl-channelizer 5단계)
// 시분할 펄스 검출기: 검출기 하나가 채널 상태 16벌을 돌려 가며 16채널을 처리한다.
// 알고리즘은 pdw_detector.v(rtl-pdw)와 같고, 아래 세 가지가 더해졌다.
//
//   1) 이웃 크기 버퍼 (이중 버퍼): CORDIC 출력(홉마다 16채널, 비트 반전 순서)을 홉 단위로 모은 뒤
//      in_last(홉의 16번째)가 오면 그 홉을 k = 0..15 순서로 한 클록에 한 채널씩 처리한다.
//      처리 중에 다음 홉은 다른 버퍼에 쌓인다. 채널 k는 같은 홉의 mag[k-1], mag[k+1]을 함께 읽는다(16채널 순환).
//   2) 진폭 우세 합: 펄스 안 TH_LO 이상 샘플(상대 인덱스 0..DOM_MAX_R)마다
//      dl += mag[k] - mag[k-1],  dr += mag[k] - mag[k+1]
//   3) 소유 판정 (펄스가 끝날 때):
//      주파수 유효(PW >= FREQ_SKIP + FREQ_WIN + FREQ_GUARD): FREQ in [-OWN_LIM, OWN_LIM) 이면 소유
//      짧은 펄스: dl > 0 이고 dr >= 0 이면 소유
//      소유한 펄스만 pdw_valid로 내보내고, 아니면 not_owned_cnt를 올린다.
//
//   raw_* : 소유 판정 전 원시 PDW (MIN_W 이상인 모든 펄스). 테스트벤치가 원본 pdw_detector 16개와 비교한다.
//   PDW 순서: (펄스가 끝난 홉, 채널 번호) -> chz_model.detect_all()과 같다.
//   TOA = 홉 번호(모든 채널 공통 카운터, 32비트), 1홉 = 채널 샘플 1개 = 1 us
//
// 같은 채널은 홉마다 한 번만 처리되고 처리 간격이 16클록 이상이라, 상태 읽기-계산-쓰기를
// 한 클록에 끝내도 해저드가 없다. 처리 중에 다음 홉이 다 차면 overrun = 1 (설계 오류 표시).
//==============================================================================

module pdw_detector_tdm #(
    parameter TH_HI      = 3000,
    parameter TH_LO      = 2000,
    parameter MIN_W      = 4,
    parameter END_LOW    = 2,
    parameter FREQ_SKIP  = 3,
    parameter FREQ_WIN   = 8,          // 8 고정 (3비트 시프트 평균)
    parameter FREQ_GUARD = 4,
    parameter DOM_MAX_R  = 15,
    parameter OWN_LIM    = 16384       // 250 kHz
) (
    input  wire               clk,
    input  wire               rst_n,

    input  wire               in_valid,     // CORDIC 출력 1개
    input  wire signed [15:0] in_mag,
    input  wire signed [15:0] in_phase,
    input  wire        [3:0]  in_k,         // 채널 번호
    input  wire               in_last,      // 홉의 16번째 출력

    output reg                pdw_valid,    // 소유한 PDW (1클록)
    output reg         [3:0]  pdw_ch,
    output reg         [31:0] pdw_toa,
    output reg         [15:0] pdw_pw,
    output reg         [15:0] pdw_pa,
    output reg         [15:0] pdw_freq,
    output reg         [15:0] pdw_flags,

    output reg                raw_valid,    // 소유 판정 전 PDW (1클록, pdw_valid와 같은 클록)
    output reg                raw_own,
    output reg         [3:0]  raw_ch,
    output reg         [31:0] raw_toa,
    output reg         [15:0] raw_pw,
    output reg         [15:0] raw_pa,
    output reg         [15:0] raw_freq,
    output reg         [15:0] raw_flags,

    output reg         [15:0] glitch_cnt,
    output reg         [15:0] not_owned_cnt,
    output reg         [31:0] hop_cnt,      // 처리한 홉 수
    output reg                overrun
);

    localparam MIN_FREQ_W = FREQ_SKIP + FREQ_WIN + FREQ_GUARD;

    //--------------------------------------------------------------------
    // 이웃 크기 버퍼 (2 x 16)
    //--------------------------------------------------------------------
    reg signed [15:0] bmag [0:31];          // {버퍼, 채널}
    reg signed [15:0] bph  [0:31];
    reg               wsel;                 // 지금 쌓는 버퍼
    reg               busy;                 // 처리 중
    reg               pb;                   // 처리 중인 버퍼
    reg        [3:0]  pk;                   // 처리 중인 채널

    always @(posedge clk) begin
        if (in_valid) begin
            bmag[{wsel, in_k}] <= in_mag;
            bph [{wsel, in_k}] <= in_phase;
        end
    end

    //--------------------------------------------------------------------
    // 채널 상태 16벌
    //--------------------------------------------------------------------
    reg               s_in    [0:15];
    reg        [31:0] s_toa   [0:15];
    reg        [15:0] s_r     [0:15];
    reg        [15:0] s_lhr   [0:15];       // 마지막으로 TH_LO 이상이던 샘플의 상대 인덱스
    reg        [15:0] s_peak  [0:15];
    reg        [1:0]  s_low   [0:15];
    reg signed [18:0] s_fsum  [0:15];
    reg        [17:0] s_msum  [0:15];
    reg signed [15:0] s_pph   [0:15];       // 직전 위상
    reg signed [20:0] s_dl    [0:15];
    reg signed [20:0] s_dr    [0:15];

    // 처리 중인 채널의 입력 (같은 홉의 이웃 크기 포함)
    wire signed [15:0] m   = bmag[{pb, pk}];
    wire signed [15:0] ph  = bph [{pb, pk}];
    wire signed [15:0] ml  = bmag[{pb, pk - 4'd1}];
    wire signed [15:0] mr  = bmag[{pb, pk + 4'd1}];
    wire        [15:0] mu  = m[15:0];
    wire signed [16:0] d_l = {m[15], m} - {ml[15], ml};
    wire signed [16:0] d_r = {m[15], m} - {mr[15], mr};

    // 현재 상태
    wire               c_in   = s_in[pk];
    wire        [15:0] c_r    = s_r[pk];
    wire        [15:0] c_lhr  = s_lhr[pk];
    wire        [15:0] c_peak = s_peak[pk];
    wire        [1:0]  c_low  = s_low[pk];
    wire signed [18:0] c_fsum = s_fsum[pk];
    wire        [17:0] c_msum = s_msum[pk];
    wire signed [20:0] c_dl   = s_dl[pk];
    wire signed [20:0] c_dr   = s_dr[pk];

    wire signed [15:0] dphi   = ph - s_pph[pk];               // 16비트 모듈로 뺄셈
    wire        [15:0] cur_r  = c_r + 16'd1;
    wire               is_hi  = (m >= TH_LO);
    wire        [1:0]  low_n  = is_hi ? 2'd0 : (c_low + 2'd1);
    wire               ending = c_in && !is_hi && (low_n >= END_LOW);
    wire        [15:0] pw_c   = c_lhr + 16'd1;
    wire               fvalid = (pw_c >= MIN_FREQ_W);
    wire signed [18:0] favg   = c_fsum >>> 3;
    wire signed [15:0] freq_c = fvalid ? favg[15:0] : 16'sd0;
    wire               own    = fvalid ? ((freq_c >= -OWN_LIM) && (freq_c < OWN_LIM))
                                       : ((c_dl > 0) && (c_dr >= 0));
    wire        [15:0] pa_c   = fvalid ? {1'b0, c_msum[17:3]} : c_peak;
    wire        [15:0] flg_c  = {14'd0, (c_peak >= 16'd32767), fvalid};

    integer k;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wsel <= 1'b0; busy <= 1'b0; pb <= 1'b0; pk <= 4'd0;
            for (k = 0; k < 16; k = k + 1) begin
                s_in[k] <= 1'b0; s_toa[k] <= 32'd0; s_r[k] <= 16'd0; s_lhr[k] <= 16'd0;
                s_peak[k] <= 16'd0; s_low[k] <= 2'd0; s_fsum[k] <= 19'sd0; s_msum[k] <= 18'd0;
                s_pph[k] <= 16'sd0; s_dl[k] <= 21'sd0; s_dr[k] <= 21'sd0;
            end
            pdw_valid <= 1'b0; pdw_ch <= 4'd0; pdw_toa <= 32'd0; pdw_pw <= 16'd0;
            pdw_pa <= 16'd0; pdw_freq <= 16'd0; pdw_flags <= 16'd0;
            raw_valid <= 1'b0; raw_own <= 1'b0; raw_ch <= 4'd0; raw_toa <= 32'd0; raw_pw <= 16'd0;
            raw_pa <= 16'd0; raw_freq <= 16'd0; raw_flags <= 16'd0;
            glitch_cnt <= 16'd0; not_owned_cnt <= 16'd0; hop_cnt <= 32'd0; overrun <= 1'b0;
        end else begin
            pdw_valid <= 1'b0;
            raw_valid <= 1'b0;

            //--------------------------------------------------------------
            // 채널 하나 처리 (pdw_detector.v와 같은 순서)
            //--------------------------------------------------------------
            if (busy) begin
                s_pph[pk] <= ph;
                if (!c_in) begin
                    if (m >= TH_HI) begin                     // 펄스 시작
                        s_in[pk]   <= 1'b1;
                        s_toa[pk]  <= hop_cnt;
                        s_r[pk]    <= 16'd0;
                        s_lhr[pk]  <= 16'd0;
                        s_peak[pk] <= mu;
                        s_low[pk]  <= 2'd0;
                        s_fsum[pk] <= 19'sd0;
                        s_msum[pk] <= 18'd0;
                        s_dl[pk]   <= {{4{d_l[16]}}, d_l};
                        s_dr[pk]   <= {{4{d_r[16]}}, d_r};
                    end
                end else begin
                    s_r[pk] <= cur_r;
                    if (is_hi) begin
                        s_lhr[pk] <= cur_r;
                        s_low[pk] <= 2'd0;
                        if (mu > c_peak) s_peak[pk] <= mu;
                        if (cur_r <= DOM_MAX_R) begin
                            s_dl[pk] <= c_dl + {{4{d_l[16]}}, d_l};
                            s_dr[pk] <= c_dr + {{4{d_r[16]}}, d_r};
                        end
                    end else begin
                        s_low[pk] <= low_n;
                    end
                    if ((cur_r >= FREQ_SKIP) && (cur_r < FREQ_SKIP + FREQ_WIN)) begin
                        s_fsum[pk] <= c_fsum + {{3{dphi[15]}}, dphi};
                        s_msum[pk] <= c_msum + {2'b00, mu};
                    end
                    if (ending) begin                         // 펄스 끝
                        s_in[pk] <= 1'b0;
                        if (pw_c >= MIN_W) begin
                            raw_valid <= 1'b1;
                            raw_own   <= own;
                            raw_ch    <= pk;
                            raw_toa   <= s_toa[pk];
                            raw_pw    <= pw_c;
                            raw_pa    <= pa_c;
                            raw_freq  <= freq_c;
                            raw_flags <= flg_c;
                            if (own) begin
                                pdw_valid <= 1'b1;
                                pdw_ch    <= pk;
                                pdw_toa   <= s_toa[pk];
                                pdw_pw    <= pw_c;
                                pdw_pa    <= pa_c;
                                pdw_freq  <= freq_c;
                                pdw_flags <= flg_c;
                            end else begin
                                not_owned_cnt <= not_owned_cnt + 16'd1;
                            end
                        end else begin
                            glitch_cnt <= glitch_cnt + 16'd1;
                        end
                    end
                end

                pk <= pk + 4'd1;
                if (pk == 4'd15) begin
                    busy    <= 1'b0;
                    hop_cnt <= hop_cnt + 32'd1;
                end
            end

            //--------------------------------------------------------------
            // 홉 하나가 다 모이면 처리 시작, 다음 홉은 다른 버퍼에
            //--------------------------------------------------------------
            if (in_valid && in_last) begin
                if (busy && pk != 4'd15) overrun <= 1'b1;
                busy <= 1'b1;
                pk   <= 4'd0;
                pb   <= wsel;
                wsel <= ~wsel;
            end
        end
    end

endmodule
