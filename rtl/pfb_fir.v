`timescale 1ns/1ps
//==============================================================================
// pfb_fir.v   (rtl-channelizer 2단계)
// 16갈래 x 8탭 다상 FIR (2배 오버샘플링 PFB의 앞단). 입력 8샘플(홉 1개)마다
// 갈래 출력 16개를 한 클록에 하나씩 FFT 입력 순서로 내보낸다.
//
//   v_p[m] = sum_{t=0}^{7} h[p+16t] * x[8m+7-p-16t]          (I, Q 따로)
//   출력 순서 q = 0..15 에서 갈래 p = (8*(m&1) - q) mod 16
//     짝수 홉: v0, v15, v14, ..., v1   -> 순방향 FFT의 빈 k = 채널 k
//     홀수 홉: 8칸 회전                -> (-1)^(km) 부호 보정을 대신함
//   반올림: v = sat18((acc + 2^15) >>> 16)   (계수 Q2.16 x 입력 Q1.15 -> 출력 Q3.15)
//   chz_model.fir_bit()과 비트 단위로 같아야 한다.
//
// 샘플 버퍼: 순환 256샘플(창 128의 2배라 다음 홉 쓰기가 현재 홉 읽기를 덮지 않음)
//   주소 a = n mod 256, 뱅크 = a[6:4], 뱅크 안 주소 = {a[7], a[3:0]} (뱅크당 32깊이)
//   탭 t의 주소 a0 - 16t 는 a[6:4]만 t만큼 줄어서 탭 8개가 서로 다른 뱅크에 있다.
//   읽기는 동기식(리셋 없음, RAM 추론용).
// 시작 직후: 아직 들어오지 않은 샘플(n < 0)은 버퍼를 지우는 대신 탭을 0으로 가린다.
//   탭 (p, t)의 샘플 번호 = n_last - (p + 16t) 이므로 {t, p} <= n_last 이면 유효.
//
// 타이밍: 8번째 샘플이 들어온 다음 클록부터 16클록 동안 갈래를 하나씩 발행,
//   발행 -> (뱅크 읽기) -> (곱셈 16개) -> (덧셈·반올림·포화) -> 출력, 지연 3클록.
//   홉 간격이 16클록보다 짧아 작업이 겹치면 overrun이 1이 된다(설계 오류 표시).
//==============================================================================

module pfb_fir #(
    parameter COEF_FILE = "sim_data/coef_h.hex"
) (
    input  wire               clk,
    input  wire               rst_n,

    input  wire               s_valid,       // 입력 샘플 1개
    input  wire signed [15:0] s_i,
    input  wire signed [15:0] s_q,

    output reg                m_valid,       // 갈래 출력 1개
    output reg  signed [17:0] m_i,
    output reg  signed [17:0] m_q,
    output reg         [3:0]  m_idx,         // FFT 입력 위치 q
    output reg                m_odd,         // 홉 번호 홀짝
    output reg                m_last,        // q == 15
    output reg                overrun
);

    //--------------------------------------------------------------------
    // 계수 ROM (128 x 18비트, 인덱스 = {t, p})
    //--------------------------------------------------------------------
    reg [17:0] coef_mem [0:127];
    initial $readmemh(COEF_FILE, coef_mem);

    //--------------------------------------------------------------------
    // 쓰기 쪽: 샘플 번호, 홉 안 위치
    //--------------------------------------------------------------------
    reg  [7:0] wp;           // 다음에 쓸 주소 = 지금까지 받은 샘플 수 mod 256
    reg  [7:0] cnt_sat;      // 지금까지 받은 샘플 수 (255에서 멈춤)
    reg  [2:0] hop_pos;      // 홉 안 샘플 위치 0..7
    reg        hop_odd;      // 다음에 끝날 홉의 홀짝

    wire [2:0] w_bank = wp[6:4];
    wire [4:0] w_addr = {wp[7], wp[3:0]};

    //--------------------------------------------------------------------
    // 작업(홉 1개 = 갈래 16개 발행)
    //--------------------------------------------------------------------
    reg        busy;
    reg  [3:0] q_cnt;
    reg  [7:0] job_base;     // 홉의 마지막 샘플 주소 (8m+7 mod 256)
    reg        job_odd;
    reg  [7:0] job_nsat;     // 홉의 마지막 샘플 번호 (255에서 멈춤)

    wire [3:0] p_c  = {job_odd, 3'b000} - q_cnt;          // (8*odd - q) mod 16
    wire [7:0] a0_c = job_base - {4'd0, p_c};             // 탭 0의 주소

    //--------------------------------------------------------------------
    // 샘플 버퍼 8뱅크
    //--------------------------------------------------------------------
    reg [31:0] rd [0:7];     // 뱅크 읽기 결과 (발행 다음 클록)

    genvar b;
    generate
        for (b = 0; b < 8; b = b + 1) begin : g_bank
            localparam [2:0] BI = b;                      // 뱅크 번호
            (* ramstyle = "MLAB, no_rw_check" *) reg [31:0] mem [0:31];
            wire [2:0] t_b   = a0_c[6:4] - BI;            // 이 뱅크가 맡는 탭 번호
            wire [3:0] hi_b  = a0_c[7:4] - {1'b0, t_b};   // 그 탭 주소의 상위 4비트
            wire [4:0] r_adr = {hi_b[3], a0_c[3:0]};
            always @(posedge clk) begin
                if (s_valid && (w_bank == BI)) mem[w_addr] <= {s_i, s_q};
                rd[b] <= mem[r_adr];
            end
        end
    endgenerate

    //--------------------------------------------------------------------
    // 파이프라인 1단: 발행 정보 보관 (뱅크 읽기와 같은 클록)
    //--------------------------------------------------------------------
    reg        v1, odd1, last1;
    reg  [3:0] p1, q1;
    reg  [2:0] ahi1;          // a0[6:4]
    reg  [7:0] nsat1;

    // 파이프라인 2단: 곱
    reg               v2, odd2, last2;
    reg  [3:0]        q2;
    reg signed [33:0] pi2 [0:7];
    reg signed [33:0] pq2 [0:7];

    integer t;
    reg   [2:0]        bsel;
    reg   [31:0]       xd;
    reg   [7:0]        tp;
    reg signed [15:0]  xi_t, xq_t;
    reg signed [17:0]  c_t;

    function signed [17:0] sat18;
        input signed [20:0] v;
        begin
            if (v > 21'sd131071)       sat18 = 18'sd131071;
            else if (v < -21'sd131072) sat18 = -18'sd131072;
            else                       sat18 = v[17:0];
        end
    endfunction

    //--------------------------------------------------------------------
    // 데이터 경로 (리셋 없음): 2단 곱셈, 3단 덧셈·반올림·포화
    //--------------------------------------------------------------------
    reg signed [36:0] si, sq, ri, rq;

    always @(posedge clk) begin
        for (t = 0; t < 8; t = t + 1) begin
            bsel = ahi1 - t[2:0];                 // 탭 t가 있는 뱅크
            xd   = rd[bsel];
            tp   = {t[2:0], p1};                  // 계수 인덱스 = p + 16t = 샘플 거리
            if (tp <= nsat1) begin
                xi_t = xd[31:16];
                xq_t = xd[15:0];
            end else begin
                xi_t = 16'sd0;                    // 아직 없는 샘플(n < 0)
                xq_t = 16'sd0;
            end
            c_t    = coef_mem[tp];
            pi2[t] <= xi_t * c_t;
            pq2[t] <= xq_t * c_t;
        end

        if (v2) begin
            si = 37'sd0;
            sq = 37'sd0;
            for (t = 0; t < 8; t = t + 1) begin
                si = si + {{3{pi2[t][33]}}, pi2[t]};
                sq = sq + {{3{pq2[t][33]}}, pq2[t]};
            end
            ri = si + 37'sd32768;                 // + 2^15 뒤 >>> 16
            rq = sq + 37'sd32768;
            m_i <= sat18(ri[36:16]);
            m_q <= sat18(rq[36:16]);
        end
    end

    //--------------------------------------------------------------------
    // 제어
    //--------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wp <= 8'd0; cnt_sat <= 8'd0; hop_pos <= 3'd0; hop_odd <= 1'b0;
            busy <= 1'b0; q_cnt <= 4'd0; job_base <= 8'd0; job_odd <= 1'b0; job_nsat <= 8'd0;
            v1 <= 1'b0; odd1 <= 1'b0; last1 <= 1'b0; p1 <= 4'd0; q1 <= 4'd0; ahi1 <= 3'd0; nsat1 <= 8'd0;
            v2 <= 1'b0; odd2 <= 1'b0; last2 <= 1'b0; q2 <= 4'd0;
            m_valid <= 1'b0; m_idx <= 4'd0; m_odd <= 1'b0; m_last <= 1'b0;
            overrun <= 1'b0;
        end else begin
            // 발행
            v1 <= busy;
            if (busy) begin
                p1    <= p_c;
                q1    <= q_cnt;
                ahi1  <= a0_c[6:4];
                odd1  <= job_odd;
                last1 <= (q_cnt == 4'd15);
                nsat1 <= job_nsat;
                q_cnt <= q_cnt + 4'd1;
                if (q_cnt == 4'd15) busy <= 1'b0;
            end

            // 샘플 쓰기와 홉 완료
            if (s_valid) begin
                wp      <= wp + 8'd1;
                if (cnt_sat != 8'd255) cnt_sat <= cnt_sat + 8'd1;
                hop_pos <= hop_pos + 3'd1;
                if (hop_pos == 3'd7) begin
                    if (busy && !(q_cnt == 4'd15)) overrun <= 1'b1;
                    busy     <= 1'b1;
                    q_cnt    <= 4'd0;
                    job_base <= wp;
                    job_odd  <= hop_odd;
                    job_nsat <= cnt_sat;          // 이 샘플의 번호 (255에서 멈춤)
                    hop_odd  <= ~hop_odd;
                end
            end

            // 2단 제어
            v2    <= v1;
            q2    <= q1;
            odd2  <= odd1;
            last2 <= last1;

            // 출력
            m_valid <= v2;
            if (v2) begin
                m_idx  <= q2;
                m_odd  <= odd2;
                m_last <= last2;
            end
        end
    end

endmodule
