`timescale 1ns/1ps
//==============================================================================
// fft16_sdf.v   (rtl-channelizer 3단계)
// 16점 radix-2 SDF(단일 경로 지연 궤환) DIF FFT. 단마다 1/2 스케일, 18비트 고정소수점.
// 입력이 유효할 때만 한 칸씩 진행(유효 신호 게이팅)하므로 데이터가 띄엄띄엄 와도 된다.
//
//   단 s (s = 0..3), 지연 D = 8 >> s, 구간 = 2D 항목:
//     앞쪽 절반: 입력 -> FIFO,  FIFO에서 나온 이전 구간의 차 d를 트위들과 곱해 출력
//     뒤쪽 절반: a = FIFO 출력, b = 입력,  출력 sat18((a+b+1)>>>1),  FIFO <- sat18((a-b+1)>>>1)
//       (a-b+1은 20비트로 계산: a = 131071, b = -131072일 때 19비트를 넘음)
//   트위들: 앞쪽 절반 위치 i의 차에 W16^(i << s)
//     s = 0, 1: 복소 곱  re = sat18((dr*wr - di*wi + 2^15) >>> 16), im = sat18((dr*wi + di*wr + 2^15) >>> 16)
//     s = 2   : i = 1 이면 -j  (dr, di) -> (di, sat18(-dr))
//     s = 3   : 없음
//   chz_model.fft16_bit()과 비트 단위로 같다. 출력 순서 = 위치 j (빈 bitrev(j)).
//
// 지연: 항목 기준 15 (8+4+2+1). 블록 m의 출력 j는 입력 항목 16m+j+15가 들어올 때 나온다.
//   그래서 블록 하나의 출력 대부분은 다음 블록 입력이 들어오는 동안 나오며,
//   마지막 블록을 다 꺼내려면 0 블록 하나를 더 넣어야 한다(비우기).
// 리셋 직후 첫 구간 앞쪽 절반의 출력은 FIFO가 비어 있어 버린다(단마다 primed 플래그).
// 포화가 한 번이라도 일어나면 ovf가 1로 남는다(정상 입력에서는 일어나지 않음).
//==============================================================================

module fft16_sdf_stage #(
    parameter D = 8,          // 지연 (8, 4, 2, 1)
    parameter S = 0,          // 단 번호 (트위들 지수 = i << S)
    parameter MODE = 0        // 0 = 일반 복소 곱, 1 = i==1일 때 -j, 2 = 트위들 없음
) (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               in_v,
    input  wire signed [17:0] in_r,
    input  wire signed [17:0] in_i,
    output reg                out_v,
    output reg  signed [17:0] out_r,
    output reg  signed [17:0] out_i,
    output reg                ovf
);
    localparam CW = (D == 1) ? 1 : $clog2(2 * D);   // 구간 카운터 폭

    reg [CW-1:0] cnt;
    reg          primed;
    wire         first = (D == 1) ? (cnt == 1'b0) : (cnt < D);
    wire [3:0]   i_pos = (D == 1) ? 4'd0 : cnt[CW-1:0] & (D - 1);   // 앞쪽 절반 위치 i

    // 지연 FIFO (시프트 레지스터, 유효 입력마다 한 칸)
    reg signed [17:0] fr [0:D-1];
    reg signed [17:0] fi [0:D-1];
    wire signed [17:0] a_r = fr[D-1];
    wire signed [17:0] a_i = fi[D-1];

    // 버터플라이 (뒤쪽 절반): 20비트로 계산해 (x + 1) >>> 1 뒤 18비트 포화
    //   a - b + 1 은 최대 262,144(a = 131071, b = -131072)라 19비트를 넘을 수 있다(그때만 포화).
    wire signed [19:0] sum_r = a_r + in_r + 20'sd1;
    wire signed [19:0] sum_i = a_i + in_i + 20'sd1;
    wire signed [19:0] dif_r = a_r - in_r + 20'sd1;
    wire signed [19:0] dif_i = a_i - in_i + 20'sd1;
    wire signed [20:0] sh_sr = {{2{sum_r[19]}}, sum_r[19:1]};
    wire signed [20:0] sh_si = {{2{sum_i[19]}}, sum_i[19:1]};
    wire signed [20:0] sh_dr = {{2{dif_r[19]}}, dif_r[19:1]};
    wire signed [20:0] sh_di = {{2{dif_i[19]}}, dif_i[19:1]};

    // 트위들 W16^e, Q2.16
    wire [3:0] e = (i_pos << S) & 4'hF;
    reg signed [17:0] wr, wi;
    always @(*) begin
        case (e[2:0])
            3'd0: begin wr =  18'sd65536; wi =  18'sd0;      end
            3'd1: begin wr =  18'sd60547; wi = -18'sd25080;  end
            3'd2: begin wr =  18'sd46341; wi = -18'sd46341;  end
            3'd3: begin wr =  18'sd25080; wi = -18'sd60547;  end
            3'd4: begin wr =  18'sd0;     wi = -18'sd65536;  end
            3'd5: begin wr = -18'sd25080; wi = -18'sd60547;  end
            3'd6: begin wr = -18'sd46341; wi = -18'sd46341;  end
            default: begin wr = -18'sd60547; wi = -18'sd25080; end
        endcase
    end

    // 앞쪽 절반 출력 = FIFO의 차 x 트위들
    wire signed [36:0] pr = a_r * wr - a_i * wi + 37'sd32768;
    wire signed [36:0] pi = a_r * wi + a_i * wr + 37'sd32768;
    wire signed [20:0] pr_h = pr[36:16];
    wire signed [20:0] pi_h = pi[36:16];

    function signed [17:0] sat18;
        input signed [20:0] v;
        begin
            if (v > 21'sd131071)       sat18 = 18'sd131071;
            else if (v < -21'sd131072) sat18 = -18'sd131072;
            else                       sat18 = v[17:0];
        end
    endfunction
    function is_sat;
        input signed [20:0] v;
        begin
            is_sat = (v > 21'sd131071) || (v < -21'sd131072);
        end
    endfunction

    wire signed [17:0] s_r = sat18(sh_sr), s_i = sat18(sh_si);
    wire signed [17:0] d_r = sat18(sh_dr), d_i = sat18(sh_di);
    wire               bf_ovf = is_sat(sh_sr) | is_sat(sh_si) | is_sat(sh_dr) | is_sat(sh_di);

    reg signed [17:0] tw_r, tw_i;
    reg               tw_ovf;
    always @(*) begin
        tw_ovf = 1'b0;
        if (MODE == 0) begin
            tw_r = sat18(pr_h);  tw_i = sat18(pi_h);
            tw_ovf = is_sat(pr_h) | is_sat(pi_h);
        end else if (MODE == 1 && i_pos == 4'd1) begin
            tw_r = a_i;
            tw_i = (a_r == -18'sd131072) ? 18'sd131071 : -a_r;      // -j, 부정은 포화
            tw_ovf = (a_r == -18'sd131072);
        end else begin
            tw_r = a_r;  tw_i = a_i;
        end
    end

    integer k;
    always @(posedge clk) begin
        if (in_v) begin
            for (k = D - 1; k > 0; k = k - 1) begin
                fr[k] <= fr[k-1];
                fi[k] <= fi[k-1];
            end
            if (first) begin
                fr[0] <= in_r;  fi[0] <= in_i;
            end else begin
                fr[0] <= d_r;   fi[0] <= d_i;
            end
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cnt <= {CW{1'b0}};  primed <= 1'b0;
            out_v <= 1'b0;  out_r <= 18'sd0;  out_i <= 18'sd0;  ovf <= 1'b0;
        end else begin
            out_v <= in_v && (primed || !first);
            if (in_v) begin
                cnt <= cnt + 1'b1;
                if (!first) primed <= 1'b1;
                if (first) begin
                    out_r <= tw_r;  out_i <= tw_i;
                    if (primed && tw_ovf) ovf <= 1'b1;
                end else begin
                    out_r <= s_r;   out_i <= s_i;
                    if (bf_ovf) ovf <= 1'b1;
                end
            end
        end
    end
endmodule


module fft16_sdf (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               s_valid,
    input  wire signed [17:0] s_r,
    input  wire signed [17:0] s_i,
    output wire               m_valid,
    output wire signed [17:0] m_r,
    output wire signed [17:0] m_i,
    output reg         [3:0]  m_pos,        // 출력 위치 j (m_valid일 때 유효)
    output wire        [3:0]  m_bin,        // 빈 번호 = bitrev(j) = 채널 k
    output wire               m_last,       // j == 15
    output wire               ovf           // 포화가 한 번이라도 있었음
);
    wire v1, v2, v3;
    wire signed [17:0] r1, i1, r2, i2, r3, i3;
    wire o0, o1, o2, o3;

    fft16_sdf_stage #(.D(8), .S(0), .MODE(0)) u_s0 (.clk(clk), .rst_n(rst_n), .in_v(s_valid), .in_r(s_r), .in_i(s_i),
                                                  .out_v(v1), .out_r(r1), .out_i(i1), .ovf(o0));
    fft16_sdf_stage #(.D(4), .S(1), .MODE(0)) u_s1 (.clk(clk), .rst_n(rst_n), .in_v(v1), .in_r(r1), .in_i(i1),
                                                  .out_v(v2), .out_r(r2), .out_i(i2), .ovf(o1));
    fft16_sdf_stage #(.D(2), .S(2), .MODE(1)) u_s2 (.clk(clk), .rst_n(rst_n), .in_v(v2), .in_r(r2), .in_i(i2),
                                                  .out_v(v3), .out_r(r3), .out_i(i3), .ovf(o2));
    fft16_sdf_stage #(.D(1), .S(3), .MODE(2)) u_s3 (.clk(clk), .rst_n(rst_n), .in_v(v3), .in_r(r3), .in_i(i3),
                                                  .out_v(m_valid), .out_r(m_r), .out_i(m_i), .ovf(o3));

    assign ovf    = o0 | o1 | o2 | o3;
    assign m_bin  = {m_pos[0], m_pos[1], m_pos[2], m_pos[3]};
    assign m_last = (m_pos == 4'd15);

    // 출력 위치 카운터: m_valid가 지나간 뒤 다음 위치로
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)       m_pos <= 4'd0;
        else if (m_valid) m_pos <= m_pos + 4'd1;
    end
endmodule
