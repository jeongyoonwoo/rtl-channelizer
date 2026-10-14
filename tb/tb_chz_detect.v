`timescale 1ns/1ps
//==============================================================================
// tb_chz_detect.v   (rtl-channelizer 5단계)
// channelizer -> chz_detect(파이프라인 CORDIC + 시분할 검출기 + 소유 판정)를 검사한다.
//
//   (a) 소유한 PDW: 비트 단위 모델(model_pdw_*.hex)과 순서까지 같은지 (CH, TOA, PW, PA, FREQ, FLAGS)
//   (b) 소유 판정 전 원시 PDW: 원본 pdw_detector(rtl-pdw, FREQ_GUARD = 4) 16개를 채널마다 하나씩 두고
//       같은 CORDIC 출력을 넣어, 채널별 PDW 순서열이 같은지 (Python과 독립된 RTL 기준)
//   (c) 글리치·비소유 개수가 모델과 같은지, overrun = 0
//   TEST 0~3 = S1~S4, HOP_DIV 50(기본) 또는 16
//
// ModelSim: vsim -onfinish stop -gTEST=0 work.tb_chz_detect
// $display 문자열은 영어
//==============================================================================

module tb_chz_detect;

    parameter TEST    = 0;
    parameter HOP_DIV = 50;

    localparam N_SAMP  = 65536;
    localparam N_HOPS  = N_SAMP / 8;
    localparam EXP_PDW = (TEST == 1) ? 42  : (TEST == 2) ? 9  : (TEST == 3) ? 31 : 204;
    localparam EXP_NO  = (TEST == 1) ? 39  : (TEST == 2) ? 9  : (TEST == 3) ? 11 : 206;
    localparam EXP_GL  = (TEST == 1) ? 148 : (TEST == 2) ? 34 : (TEST == 3) ? 0  : 210;

    reg clk = 1'b0;
    always #10 clk = ~clk;
    reg rst_n = 1'b0;
    reg run   = 1'b0;

    //--------------------------------------------------------------------
    // 채널라이저 (시험마다 ROM 파일이 달라 generate)
    //--------------------------------------------------------------------
    wire               ch_valid, ch_last, ch_done, fir_ovr, fft_ovf, out_sat;
    wire signed [15:0] ch_i, ch_q;
    wire [3:0]         ch_k, ch_pos;
    wire [15:0]        ch_hop;

    generate
        if (TEST == 1) begin : g_chz
            channelizer #(.ROM_FILE("sim_data/iq_rom_s2.hex"), .HOP_DIV(HOP_DIV)) u (
                .clk(clk), .rst_n(rst_n), .run(run), .fast(1'b0), .ch_valid(ch_valid), .ch_i(ch_i), .ch_q(ch_q), .ch_k(ch_k),
                .ch_pos(ch_pos), .ch_last(ch_last), .ch_hop(ch_hop), .done(ch_done),
                .fir_overrun(fir_ovr), .fft_ovf(fft_ovf), .out_sat(out_sat));
        end else if (TEST == 2) begin : g_chz
            channelizer #(.ROM_FILE("sim_data/iq_rom_s3.hex"), .HOP_DIV(HOP_DIV)) u (
                .clk(clk), .rst_n(rst_n), .run(run), .fast(1'b0), .ch_valid(ch_valid), .ch_i(ch_i), .ch_q(ch_q), .ch_k(ch_k),
                .ch_pos(ch_pos), .ch_last(ch_last), .ch_hop(ch_hop), .done(ch_done),
                .fir_overrun(fir_ovr), .fft_ovf(fft_ovf), .out_sat(out_sat));
        end else if (TEST == 3) begin : g_chz
            channelizer #(.ROM_FILE("sim_data/iq_rom_s4.hex"), .HOP_DIV(HOP_DIV)) u (
                .clk(clk), .rst_n(rst_n), .run(run), .fast(1'b0), .ch_valid(ch_valid), .ch_i(ch_i), .ch_q(ch_q), .ch_k(ch_k),
                .ch_pos(ch_pos), .ch_last(ch_last), .ch_hop(ch_hop), .done(ch_done),
                .fir_overrun(fir_ovr), .fft_ovf(fft_ovf), .out_sat(out_sat));
        end else begin : g_chz
            channelizer #(.ROM_FILE("sim_data/iq_rom_s1.hex"), .HOP_DIV(HOP_DIV)) u (
                .clk(clk), .rst_n(rst_n), .run(run), .fast(1'b0), .ch_valid(ch_valid), .ch_i(ch_i), .ch_q(ch_q), .ch_k(ch_k),
                .ch_pos(ch_pos), .ch_last(ch_last), .ch_hop(ch_hop), .done(ch_done),
                .fir_overrun(fir_ovr), .fft_ovf(fft_ovf), .out_sat(out_sat));
        end
    endgenerate

    //--------------------------------------------------------------------
    // 검사 대상
    //--------------------------------------------------------------------
    wire               c_valid, c_last;
    wire signed [15:0] c_mag, c_phase;
    wire [3:0]         c_k;
    wire               pdw_valid, raw_valid, raw_own, det_ovr;
    wire [3:0]         pdw_ch, raw_ch;
    wire [31:0]        pdw_toa, raw_toa, hop_cnt;
    wire [15:0]        pdw_pw, pdw_pa, pdw_freq, pdw_flags, raw_pw, raw_pa, raw_freq, raw_flags;
    wire [15:0]        glitch_cnt, not_owned_cnt;

    chz_detect dut (
        .clk(clk), .rst_n(rst_n),
        .ch_valid(ch_valid), .ch_i(ch_i), .ch_q(ch_q), .ch_k(ch_k), .ch_last(ch_last),
        .c_valid(c_valid), .c_mag(c_mag), .c_phase(c_phase), .c_k(c_k), .c_last(c_last),
        .pdw_valid(pdw_valid), .pdw_ch(pdw_ch), .pdw_toa(pdw_toa), .pdw_pw(pdw_pw),
        .pdw_pa(pdw_pa), .pdw_freq(pdw_freq), .pdw_flags(pdw_flags),
        .raw_valid(raw_valid), .raw_own(raw_own), .raw_ch(raw_ch), .raw_toa(raw_toa), .raw_pw(raw_pw),
        .raw_pa(raw_pa), .raw_freq(raw_freq), .raw_flags(raw_flags),
        .glitch_cnt(glitch_cnt), .not_owned_cnt(not_owned_cnt), .hop_cnt(hop_cnt), .overrun(det_ovr)
    );

    //--------------------------------------------------------------------
    // 독립 기준: 원본 pdw_detector 16개 (채널마다 하나)
    //--------------------------------------------------------------------
    wire [15:0]  r_valid;
    wire [31:0]  r_toa   [0:15];
    wire [15:0]  r_pw    [0:15];
    wire [15:0]  r_pa    [0:15];
    wire [15:0]  r_freq  [0:15];
    wire [15:0]  r_flags [0:15];
    wire [15:0]  r_gl    [0:15];

    genvar g;
    generate
        for (g = 0; g < 16; g = g + 1) begin : g_ref
            pdw_detector #(.FREQ_GUARD(4)) u_ref (
                .clk(clk), .rst_n(rst_n),
                .meas_valid(c_valid && (c_k == g)), .meas_mag(c_mag), .meas_phase(c_phase),
                .pdw_valid(r_valid[g]), .pdw_toa(r_toa[g]), .pdw_pw(r_pw[g]), .pdw_pa(r_pa[g]),
                .pdw_freq(r_freq[g]), .pdw_flags(r_flags[g]), .glitch_cnt(r_gl[g])
            );
        end
    endgenerate

    // 채널별 대기열: 원본이 먼저 내고(샘플이 오자마자), 시분할 검출기는 홉이 다 모인 뒤 낸다
    localparam QD = 256;
    reg [95:0] q [0:16*QD-1];                 // {toa, pw, pa, freq, flags}
    integer    q_wr [0:15];
    integer    q_rd [0:15];
    integer    kk, n_ref_total = 0;
    initial for (kk = 0; kk < 16; kk = kk + 1) begin q_wr[kk] = 0; q_rd[kk] = 0; end

    always @(posedge clk) begin
        for (kk = 0; kk < 16; kk = kk + 1) begin
            if (r_valid[kk]) begin
                q[kk * QD + (q_wr[kk] % QD)] = {r_toa[kk], r_pw[kk], r_pa[kk], r_freq[kk], r_flags[kk]};
                q_wr[kk] = q_wr[kk] + 1;
                n_ref_total = n_ref_total + 1;
            end
        end
    end

    integer n_raw = 0, n_raw_err = 0, n_raw_own = 0;
    reg [95:0] rexp;
    always @(posedge clk) begin
        if (raw_valid) begin
            n_raw = n_raw + 1;
            if (raw_own) n_raw_own = n_raw_own + 1;
            if (q_rd[raw_ch] >= q_wr[raw_ch]) begin
                n_raw_err = n_raw_err + 1;
                if (n_raw_err <= 5) $display("  RAW ch %0d toa %0d: no matching reference PDW", raw_ch, raw_toa);
            end else begin
                rexp = q[raw_ch * QD + (q_rd[raw_ch] % QD)];
                q_rd[raw_ch] = q_rd[raw_ch] + 1;
                if ({raw_toa, raw_pw, raw_pa, raw_freq, raw_flags} !== rexp) begin
                    n_raw_err = n_raw_err + 1;
                    if (n_raw_err <= 5)
                        $display("  RAW MISMATCH ch %0d: tdm toa %0d pw %0d pa %0d freq %0d flags %0h | ref toa %0d pw %0d pa %0d freq %0d flags %0h",
                                 raw_ch, raw_toa, raw_pw, raw_pa, $signed(raw_freq), raw_flags,
                                 rexp[95:64], rexp[63:48], rexp[47:32], $signed(rexp[31:16]), rexp[15:0]);
                end
            end
        end
    end

    //--------------------------------------------------------------------
    // 모델 PDW와 비교: CH(부호 있는 바이트) TOA PW PA FREQ FLAGS = 104비트
    //--------------------------------------------------------------------
    reg [103:0] mpdw [0:511];
    initial begin
        for (kk = 0; kk < 512; kk = kk + 1) mpdw[kk] = {104{1'b1}};
        case (TEST)
            1: $readmemh("sim_data/model_pdw_s2.hex", mpdw, 0, EXP_PDW - 1);
            2: $readmemh("sim_data/model_pdw_s3.hex", mpdw, 0, EXP_PDW - 1);
            3: $readmemh("sim_data/model_pdw_s4.hex", mpdw, 0, EXP_PDW - 1);
            default: $readmemh("sim_data/model_pdw_s1.hex", mpdw, 0, EXP_PDW - 1);
        endcase
    end

    integer n_pdw = 0, n_pdw_err = 0;
    reg [103:0] got;
    always @(posedge clk) begin
        if (pdw_valid) begin
            got = {{4{pdw_ch[3]}}, pdw_ch, pdw_toa, pdw_pw, pdw_pa, pdw_freq, pdw_flags};
            if (n_pdw >= EXP_PDW || got !== mpdw[n_pdw]) begin
                n_pdw_err = n_pdw_err + 1;
                if (n_pdw_err <= 5)
                    $display("  PDW MISMATCH #%0d: got %h  exp %h", n_pdw, got, mpdw[n_pdw]);
            end
            n_pdw = n_pdw + 1;
        end
    end

    //--------------------------------------------------------------------
    // 진행과 판정
    //--------------------------------------------------------------------
    initial begin
        repeat (5) @(posedge clk);
        rst_n <= 1'b1;
        repeat (3) @(posedge clk);
        run <= 1'b1;
    end

    initial begin
        repeat ((N_HOPS + 20) * HOP_DIV + 5000) @(posedge clk);
        $display("FAIL: timeout");
        $finish;
    end

    integer n_left, n_ref_gl;
    initial begin
        wait (ch_done);
        repeat (100) @(posedge clk);
        n_left = 0; n_ref_gl = 0;
        for (kk = 0; kk < 16; kk = kk + 1) begin
            n_left   = n_left + (q_wr[kk] - q_rd[kk]);
            n_ref_gl = n_ref_gl + r_gl[kk];
        end
        $display("------------------------------------------------------------");
        case (TEST)
            1: $display("tb_chz_detect TEST 1 (scenario S2): HOP_DIV %0d, hops %0d", HOP_DIV, hop_cnt);
            2: $display("tb_chz_detect TEST 2 (scenario S3): HOP_DIV %0d, hops %0d", HOP_DIV, hop_cnt);
            3: $display("tb_chz_detect TEST 3 (scenario S4): HOP_DIV %0d, hops %0d", HOP_DIV, hop_cnt);
            default: $display("tb_chz_detect TEST 0 (scenario S1): HOP_DIV %0d, hops %0d", HOP_DIV, hop_cnt);
        endcase
        $display("  (a) owned PDWs %0d / model %0d, mismatches %0d", n_pdw, EXP_PDW, n_pdw_err);
        $display("  (b) raw PDWs %0d vs 16 x pdw_detector %0d, mismatches %0d, unmatched reference %0d",
                 n_raw, n_ref_total, n_raw_err, n_left);
        $display("  (c) glitch %0d (model %0d, reference %0d), not owned %0d (model %0d), overrun %0d",
                 glitch_cnt, EXP_GL, n_ref_gl, not_owned_cnt, EXP_NO, det_ovr);
        if (n_pdw == EXP_PDW && n_pdw_err == 0 && n_raw_err == 0 && n_left == 0 && n_raw == n_ref_total &&
            glitch_cnt == EXP_GL && n_ref_gl == EXP_GL && not_owned_cnt == EXP_NO && n_raw_own == EXP_PDW &&
            !det_ovr && hop_cnt == N_HOPS)
            $display("PASS: PDWs match the bit-accurate model and 16 x pdw_detector (diff 0)");
        else
            $display("FAIL");
        $display("------------------------------------------------------------");
        $finish;
    end

endmodule
