`timescale 1ns/1ps
//==============================================================================
// tb_chz_top.v   (rtl-channelizer 6단계 통합)
// chz_top의 출력 바이트 스트림만 보고 판정한다.
//   1) 프레임 해독: ASM, FRAME_CNT 순번, N_PDW, m_last 위치, CRC-16 (idx 4~19)
//   2) 모델 프레임(model_frame_*.hex)과 바이트 단위로 같은지, 스트림 서명(전체 바이트 CRC-16)
//   3) 역압: m_valid가 선 뒤 m_ready가 올 때까지 m_data, m_last 유지, 프레임 중간에 m_valid가 내려가지 않음
//   4) 정답표(expected_pdw_*.hex) 대비 정확도: 검출 누락·여분(오검출/중복)·필드 오차 (계획서 5장 기준)
//   5) 상태: FIFO 넘침, 각 단 overrun/ovf, 출력 포화가 모두 0
//   TEST 0~4 = S1~S5 (S5: 8채널 동시 종료, FIFO 점유 시험), HOP_DIV 50(데모) / 16(최대 처리량),
//   READY 0 = 항상 1, 1 = 약 50%, 2 = 약 1/8
//
// ModelSim: vsim -onfinish stop -gTEST=0 work.tb_chz_top
// $display 문자열은 영어
//==============================================================================

module tb_chz_top;

    parameter TEST    = 0;
    parameter HOP_DIV = 50;
    parameter READY   = 0;

    localparam N_HOPS = 8192;
    localparam N_EXP  = (TEST == 1) ? 42 : (TEST == 2) ? 9 : (TEST == 3) ? 31 : (TEST == 4) ? 624 : 204;   // 정답 펄스 수 = 모델 프레임 수
    // 허용치 (S4는 잡음이 커서 따로)
    localparam integer TOA_LO = -1;
    localparam integer TOA_HI = (TEST == 3) ? 3 : 2;
    localparam integer PW_LO  = (TEST == 3) ? -3 : -2;
    localparam integer PW_HI  = 3;
    localparam real    FR_TOL = (TEST == 3) ? 6000.0 : 3000.0;   // Hz
    localparam real    PAV_A  = (TEST == 3) ? 400.0 : 230.0;

    reg clk = 1'b0;
    always #10 clk = ~clk;
    reg rst_n = 1'b0;
    reg run   = 1'b0;
    reg m_ready = 1'b0;

    wire [7:0]  m_data;
    wire        m_valid, m_last, all_done, fir_ovr, fft_ovf, out_sat, det_ovr;
    wire [15:0] glitch_cnt, not_owned_cnt, fifo_ovf, frame_cnt;
    wire [5:0]  fifo_max;
    wire [31:0] hop_cnt;

    generate
        if (TEST == 1) begin : g_top
            chz_top #(.ROM_FILE("sim_data/iq_rom_s2.hex"), .HOP_DIV(HOP_DIV)) dut (
                .clk(clk), .rst_n(rst_n), .run(run), .fast(1'b0), .m_data(m_data), .m_valid(m_valid), .m_ready(m_ready), .m_last(m_last),
                .all_done(all_done), .fir_overrun(fir_ovr), .fft_ovf(fft_ovf), .out_sat(out_sat), .det_overrun(det_ovr),
                .glitch_cnt(glitch_cnt), .not_owned_cnt(not_owned_cnt), .fifo_ovf(fifo_ovf), .fifo_max(fifo_max),
                .frame_cnt(frame_cnt), .hop_cnt(hop_cnt));
        end else if (TEST == 2) begin : g_top
            chz_top #(.ROM_FILE("sim_data/iq_rom_s3.hex"), .HOP_DIV(HOP_DIV)) dut (
                .clk(clk), .rst_n(rst_n), .run(run), .fast(1'b0), .m_data(m_data), .m_valid(m_valid), .m_ready(m_ready), .m_last(m_last),
                .all_done(all_done), .fir_overrun(fir_ovr), .fft_ovf(fft_ovf), .out_sat(out_sat), .det_overrun(det_ovr),
                .glitch_cnt(glitch_cnt), .not_owned_cnt(not_owned_cnt), .fifo_ovf(fifo_ovf), .fifo_max(fifo_max),
                .frame_cnt(frame_cnt), .hop_cnt(hop_cnt));
        end else if (TEST == 3) begin : g_top
            chz_top #(.ROM_FILE("sim_data/iq_rom_s4.hex"), .HOP_DIV(HOP_DIV)) dut (
                .clk(clk), .rst_n(rst_n), .run(run), .fast(1'b0), .m_data(m_data), .m_valid(m_valid), .m_ready(m_ready), .m_last(m_last),
                .all_done(all_done), .fir_overrun(fir_ovr), .fft_ovf(fft_ovf), .out_sat(out_sat), .det_overrun(det_ovr),
                .glitch_cnt(glitch_cnt), .not_owned_cnt(not_owned_cnt), .fifo_ovf(fifo_ovf), .fifo_max(fifo_max),
                .frame_cnt(frame_cnt), .hop_cnt(hop_cnt));
        end else if (TEST == 4) begin : g_top
            chz_top #(.ROM_FILE("sim_data/iq_rom_s5.hex"), .HOP_DIV(HOP_DIV)) dut (
                .clk(clk), .rst_n(rst_n), .run(run), .fast(1'b0), .m_data(m_data), .m_valid(m_valid), .m_ready(m_ready), .m_last(m_last),
                .all_done(all_done), .fir_overrun(fir_ovr), .fft_ovf(fft_ovf), .out_sat(out_sat), .det_overrun(det_ovr),
                .glitch_cnt(glitch_cnt), .not_owned_cnt(not_owned_cnt), .fifo_ovf(fifo_ovf), .fifo_max(fifo_max),
                .frame_cnt(frame_cnt), .hop_cnt(hop_cnt));
        end else begin : g_top
            chz_top #(.ROM_FILE("sim_data/iq_rom_s1.hex"), .HOP_DIV(HOP_DIV)) dut (
                .clk(clk), .rst_n(rst_n), .run(run), .fast(1'b0), .m_data(m_data), .m_valid(m_valid), .m_ready(m_ready), .m_last(m_last),
                .all_done(all_done), .fir_overrun(fir_ovr), .fft_ovf(fft_ovf), .out_sat(out_sat), .det_overrun(det_ovr),
                .glitch_cnt(glitch_cnt), .not_owned_cnt(not_owned_cnt), .fifo_ovf(fifo_ovf), .fifo_max(fifo_max),
                .frame_cnt(frame_cnt), .hop_cnt(hop_cnt));
        end
    endgenerate

    reg [175:0] mfr  [0:N_EXP-1];        // 모델 프레임 22바이트
    reg [103:0] tru  [0:N_EXP-1];        // 정답표 {CH8, TOA32, PW16, PA16, FREQ16, FLAGS16}
    initial begin
        case (TEST)
            1: begin $readmemh("sim_data/model_frame_s2.hex", mfr); $readmemh("sim_data/expected_pdw_s2.hex", tru); end
            2: begin $readmemh("sim_data/model_frame_s3.hex", mfr); $readmemh("sim_data/expected_pdw_s3.hex", tru); end
            3: begin $readmemh("sim_data/model_frame_s4.hex", mfr); $readmemh("sim_data/expected_pdw_s4.hex", tru); end
            4: begin $readmemh("sim_data/model_frame_s5.hex", mfr); $readmemh("sim_data/expected_pdw_s5.hex", tru); end
            default: begin $readmemh("sim_data/model_frame_s1.hex", mfr); $readmemh("sim_data/expected_pdw_s1.hex", tru); end
        endcase
    end

    //--------------------------------------------------------------------
    // 진행, 역압 패턴
    //--------------------------------------------------------------------
    integer seed = 2026;
    initial begin
        repeat (5) @(posedge clk);
        rst_n <= 1'b1;
        repeat (3) @(posedge clk);
        run <= 1'b1;
    end
    always @(posedge clk) begin
        if (READY == 1)      m_ready <= ($random(seed) & 1);
        else if (READY == 2) m_ready <= (($random(seed) & 7) == 0);
        else                 m_ready <= 1'b1;
    end

    //--------------------------------------------------------------------
    // CRC
    //--------------------------------------------------------------------
    function [15:0] crc16_byte;
        input [15:0] c_in;
        input [7:0]  d;
        integer      k;
        reg   [15:0] c;
        begin
            c = c_in ^ {d, 8'h00};
            for (k = 0; k < 8; k = k + 1) c = c[15] ? ((c << 1) ^ 16'h1021) : (c << 1);
            crc16_byte = c;
        end
    endfunction

    //--------------------------------------------------------------------
    // 바이트 스트림 해독
    //--------------------------------------------------------------------
    reg  [7:0]   fb [0:21];
    integer      bidx = 0, n_frames = 0, n_bytes = 0;
    integer      n_crc_err = 0, n_asm_err = 0, n_cnt_err = 0, n_len_err = 0, n_model_err = 0, n_hold_err = 0;
    reg  [15:0]  sig = 16'hFFFF, fcrc;
    reg  [175:0] fr_bits;
    integer      q;
    // 해독한 PDW
    integer      d_ch    [0:1023];
    integer      d_toa   [0:1023];
    integer      d_pw    [0:1023];
    integer      d_pa    [0:1023];
    integer      d_freq  [0:1023];
    integer      d_flags [0:1023];
    reg          d_used  [0:1023];

    reg          pv; reg [7:0] pd; reg pl;
    always @(posedge clk) begin
        // 역압 규칙: 지난 클록에 valid였고 ready가 아니었으면 이번 클록 값이 같아야 함
        if (pv && m_valid && (m_data !== pd || m_last !== pl)) n_hold_err = n_hold_err + 1;
        if (pv && !m_valid) n_hold_err = n_hold_err + 1;
        // 프레임 중간(바이트 1~21)에는 ready와 관계없이 valid가 계속 1이어야 한다 (valid가 ready에 의존하면 안 됨)
        if (bidx != 0 && !m_valid) n_hold_err = n_hold_err + 1;
        pv = m_valid && !m_ready; pd = m_data; pl = m_last;

        if (m_valid && m_ready) begin
            fb[bidx] = m_data;
            sig = crc16_byte(sig, m_data);
            n_bytes = n_bytes + 1;
            if (m_last !== (bidx == 21)) n_len_err = n_len_err + 1;
            if (bidx == 21 || m_last) begin
                // 프레임 하나 완성
                if ({fb[0], fb[1], fb[2], fb[3]} !== 32'h1ACFFC1D) n_asm_err = n_asm_err + 1;
                if ({fb[4], fb[5]} !== n_frames[15:0] || fb[6] !== 8'd1) n_cnt_err = n_cnt_err + 1;
                fcrc = 16'hFFFF;
                for (q = 4; q <= 19; q = q + 1) fcrc = crc16_byte(fcrc, fb[q]);
                if ({fb[20], fb[21]} !== fcrc) n_crc_err = n_crc_err + 1;
                for (q = 0; q < 22; q = q + 1) fr_bits[175 - 8*q -: 8] = fb[q];
                if (n_frames >= N_EXP || fr_bits !== mfr[n_frames]) begin
                    n_model_err = n_model_err + 1;
                    if (n_model_err <= 3) $display("  FRAME %0d differs from model: got %h", n_frames, fr_bits);
                end
                if (n_frames < 1024) begin
                    d_ch[n_frames]    = $signed(fb[7]);
                    d_toa[n_frames]   = {fb[8], fb[9], fb[10], fb[11]};
                    d_pw[n_frames]    = {fb[12], fb[13]};
                    d_pa[n_frames]    = {fb[14], fb[15]};
                    d_freq[n_frames]  = $signed({fb[16], fb[17]});
                    d_flags[n_frames] = {fb[18], fb[19]};
                    d_used[n_frames]  = 1'b0;
                end
                n_frames = n_frames + 1;
                bidx = 0;
            end else begin
                bidx = bidx + 1;
            end
        end
    end

    //--------------------------------------------------------------------
    // 정답표 대비 판정
    //--------------------------------------------------------------------
    integer e, i, hit, n_match, n_miss, n_bad, n_extra;
    integer e_ch, e_toa, e_pw, e_pa, e_freq, e_flags, dt, dw, dp, nbad_this;
    integer toa_min, toa_max, pw_min, pw_max, pa_vmax, pa_pmax, n_pav, n_pap, n_fr;
    real    toa_sum, pw_sum, fr_err, fr_max, fr_sum, tol;
    reg     fv_m, fv_e, fv_dc, sat_m, sat_e, sat_dc;

    initial begin
        repeat ((N_HOPS + 40) * HOP_DIV + N_EXP * 22 * 10 + 10000) @(posedge clk);
        $display("FAIL: timeout (frames %0d / %0d)", n_frames, N_EXP);
        $finish;
    end

    initial begin
        wait (all_done);
        repeat (20) @(posedge clk);
        n_match = 0; n_miss = 0; n_bad = 0; n_extra = 0;
        toa_min = 99; toa_max = -99; pw_min = 99; pw_max = -99; toa_sum = 0.0; pw_sum = 0.0;
        pa_vmax = 0; pa_pmax = 0; n_pav = 0; n_pap = 0; fr_max = 0.0; fr_sum = 0.0; n_fr = 0;
        for (e = 0; e < N_EXP; e = e + 1) begin
            e_ch = $signed(tru[e][103:96]); e_toa = tru[e][95:64]; e_pw = tru[e][63:48];
            e_pa = tru[e][47:32]; e_freq = $signed(tru[e][31:16]); e_flags = tru[e][15:0];
            hit = -1;
            for (i = 0; i < n_frames && i < 1024; i = i + 1)
                if (hit < 0 && !d_used[i] && d_ch[i] == e_ch && d_toa[i] - e_toa >= -2 && d_toa[i] - e_toa <= 4) hit = i;
            if (hit < 0) begin
                n_miss = n_miss + 1;
                $display("  MISS truth %0d: ch %0d toa %0d pw %0d", e, e_ch, e_toa, e_pw);
            end else begin
                d_used[hit] = 1'b1; n_match = n_match + 1; nbad_this = 0;
                dt = d_toa[hit] - e_toa; dw = d_pw[hit] - e_pw;
                if (dt < toa_min) toa_min = dt;  if (dt > toa_max) toa_max = dt;  toa_sum = toa_sum + dt;
                if (dw < pw_min) pw_min = dw;    if (dw > pw_max) pw_max = dw;    pw_sum = pw_sum + dw;
                if (dt < TOA_LO || dt > TOA_HI) nbad_this = nbad_this + 1;
                if (dw < PW_LO || dw > PW_HI) nbad_this = nbad_this + 1;
                fv_m = d_flags[hit] & 1; fv_e = e_flags & 1; fv_dc = (e_flags >> 15) & 1;
                sat_m = (d_flags[hit] >> 1) & 1; sat_e = (e_flags >> 1) & 1; sat_dc = (e_flags >> 14) & 1;
                if (!fv_dc && fv_m != fv_e) nbad_this = nbad_this + 1;
                if (!sat_dc && sat_m != sat_e) nbad_this = nbad_this + 1;
                if (fv_m) begin
                    fr_err = (d_freq[hit] - e_freq) * 500000.0 / 32768.0;
                    if (fr_err < 0) fr_err = -fr_err;
                    if (fr_err > fr_max) fr_max = fr_err;  fr_sum = fr_sum + fr_err;  n_fr = n_fr + 1;
                    if (fr_err > FR_TOL) nbad_this = nbad_this + 1;
                end
                if (!sat_m && !sat_e) begin
                    dp = d_pa[hit] - e_pa; if (dp < 0) dp = -dp;
                    if (fv_m) begin tol = 0.03 * e_pa + PAV_A; if (dp > pa_vmax) pa_vmax = dp; n_pav = n_pav + 1; end
                    else      begin tol = 0.08 * e_pa + 650.0; if (dp > pa_pmax) pa_pmax = dp; n_pap = n_pap + 1; end
                    if (dp > tol) nbad_this = nbad_this + 1;
                end
                if (nbad_this != 0) begin
                    n_bad = n_bad + 1;
                    $display("  FIELD truth %0d (ch %0d toa %0d): got toa %0d pw %0d pa %0d freq %0d flags %0h, exp pw %0d pa %0d freq %0d flags %0h",
                             e, e_ch, e_toa, d_toa[hit], d_pw[hit], d_pa[hit], d_freq[hit], d_flags[hit], e_pw, e_pa, e_freq, e_flags);
                end
            end
        end
        for (i = 0; i < n_frames && i < 1024; i = i + 1)
            if (!d_used[i]) begin
                n_extra = n_extra + 1;
                $display("  EXTRA frame %0d: ch %0d toa %0d pw %0d", i, d_ch[i], d_toa[i], d_pw[i]);
            end

        $display("------------------------------------------------------------");
        case (TEST)
            1: $display("tb_chz_top TEST 1 (scenario S2): HOP_DIV %0d, READY %0d", HOP_DIV, READY);
            2: $display("tb_chz_top TEST 2 (scenario S3): HOP_DIV %0d, READY %0d", HOP_DIV, READY);
            3: $display("tb_chz_top TEST 3 (scenario S4): HOP_DIV %0d, READY %0d", HOP_DIV, READY);
            4: $display("tb_chz_top TEST 4 (scenario S5, simultaneous ends): HOP_DIV %0d, READY %0d", HOP_DIV, READY);
            default: $display("tb_chz_top TEST 0 (scenario S1): HOP_DIV %0d, READY %0d", HOP_DIV, READY);
        endcase
        $display("  frames %0d (model %0d), bytes %0d, stream signature 0x%04h", n_frames, N_EXP, n_bytes, sig);
        $display("  CRC fail %0d, ASM err %0d, count err %0d, length err %0d, differs from model %0d, hold violations %0d",
                 n_crc_err, n_asm_err, n_cnt_err, n_len_err, n_model_err, n_hold_err);
        $display("  accuracy vs truth: matched %0d / %0d, miss %0d, extra (false/dup) %0d, field failures %0d",
                 n_match, N_EXP, n_miss, n_extra, n_bad);
        $display("    TOA err %0d..%0d mean %0.2f (limit %0d..%0d) | PW err %0d..%0d mean %0.2f (limit %0d..%0d)",
                 toa_min, toa_max, toa_sum / (n_match > 0 ? n_match : 1), TOA_LO, TOA_HI,
                 pw_min, pw_max, pw_sum / (n_match > 0 ? n_match : 1), PW_LO, PW_HI);
        $display("    FREQ |err| max %0.3f mean %0.3f kHz (n=%0d, limit %0.0f kHz) | PA |err| max window %0d (n=%0d), peak %0d (n=%0d)",
                 fr_max / 1000.0, fr_sum / (n_fr > 0 ? n_fr : 1) / 1000.0, n_fr, FR_TOL / 1000.0, pa_vmax, n_pav, pa_pmax, n_pap);
        $display("  status: fifo overflow %0d (max level %0d / 32), fir_overrun %0d, fft_ovf %0d, out_sat %0d, det_overrun %0d, glitch %0d, not owned %0d",
                 fifo_ovf, fifo_max, fir_ovr, fft_ovf, out_sat, det_ovr, glitch_cnt, not_owned_cnt);
        if (n_frames == N_EXP && n_crc_err == 0 && n_asm_err == 0 && n_cnt_err == 0 && n_len_err == 0 &&
            n_model_err == 0 && n_hold_err == 0 && n_miss == 0 && n_extra == 0 && n_bad == 0 &&
            fifo_ovf == 0 && !fir_ovr && !fft_ovf && !out_sat && !det_ovr)
            $display("PASS: frames match the model, all truth pulses detected within limits, no false or duplicate PDWs");
        else
            $display("FAIL");
        $display("------------------------------------------------------------");
        $finish;
    end

endmodule
