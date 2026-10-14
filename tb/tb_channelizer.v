`timescale 1ns/1ps
//==============================================================================
// tb_channelizer.v   (rtl-channelizer 4단계)
// channelizer (ROM 재생 -> FIR -> FFT -> 16비트)의 채널 출력을 검사한다.
//
//   TEST 0~3 = 시나리오 S1~S4 (8,192홉): 채널 출력이 비트 단위 모델(ch_ref_*.hex)과 같은지
//   TEST 4   = T1 톤 스윕 (3,072홉): 모델 비교 + 채널 응답을 테스트벤치가 직접 측정해 판정
//              통과대역(|오프셋| <= 0.5채널) 리플 <= 0.2 dB, 1채널 떨어진 톤 <= -40 dB, 1.5채널 이상 <= -80 dB
//   TEST 5   = N1 잡음 (4,096홉): 모델 비교 + 잡음 감소량(입력 전력 / 채널 출력 전력) 10.7 +- 0.5 dB
//   TEST 6   = 출력 포화 (1,024홉): 진폭 0.97 펄스의 가장자리 오버슈트로 16비트 포화 -> out_sat = 1 기대, 모델과 같아야 함
//   모든 시험: 채널 번호 = bitrev(위치), 홉 번호, done, fir_overrun = fft_ovf = 0, out_sat = (TEST == 6)
//
// ModelSim: vsim -onfinish stop -gTEST=4 work.tb_channelizer   (HOP_DIV 기본 50, -gHOP_DIV=16 가능)
// $display 문자열은 영어 (ModelSim 콘솔 한글 깨짐 방지)
//==============================================================================

module tb_channelizer;

    parameter TEST    = 0;
    parameter HOP_DIV = 50;

    localparam N_SAMP = (TEST == 4) ? 24576 : (TEST == 5) ? 32768 : (TEST == 6) ? 8192 : 65536;
    localparam N_HOPS = N_SAMP / 8;
    localparam N_OUT  = N_HOPS * 16;

    reg clk = 1'b0;
    always #10 clk = ~clk;
    reg rst_n = 1'b0;
    reg run   = 1'b0;

    wire               ch_valid, ch_last, done, fir_overrun, fft_ovf, out_sat;
    wire signed [15:0] ch_i, ch_q;
    wire [3:0]         ch_k, ch_pos;
    wire [15:0]        ch_hop;

    // ROM 파일은 문자열 파라미터라 시험마다 따로 인스턴스 (generate)
    generate
        if (TEST == 1) begin : g_dut
            channelizer #(.ROM_FILE("sim_data/iq_rom_s2.hex"), .N_SAMPLES(65536), .HOP_DIV(HOP_DIV)) dut (
                .clk(clk), .rst_n(rst_n), .run(run), .fast(1'b0), .ch_valid(ch_valid), .ch_i(ch_i), .ch_q(ch_q), .ch_k(ch_k),
                .ch_pos(ch_pos), .ch_last(ch_last), .ch_hop(ch_hop), .done(done),
                .fir_overrun(fir_overrun), .fft_ovf(fft_ovf), .out_sat(out_sat));
        end else if (TEST == 2) begin : g_dut
            channelizer #(.ROM_FILE("sim_data/iq_rom_s3.hex"), .N_SAMPLES(65536), .HOP_DIV(HOP_DIV)) dut (
                .clk(clk), .rst_n(rst_n), .run(run), .fast(1'b0), .ch_valid(ch_valid), .ch_i(ch_i), .ch_q(ch_q), .ch_k(ch_k),
                .ch_pos(ch_pos), .ch_last(ch_last), .ch_hop(ch_hop), .done(done),
                .fir_overrun(fir_overrun), .fft_ovf(fft_ovf), .out_sat(out_sat));
        end else if (TEST == 3) begin : g_dut
            channelizer #(.ROM_FILE("sim_data/iq_rom_s4.hex"), .N_SAMPLES(65536), .HOP_DIV(HOP_DIV)) dut (
                .clk(clk), .rst_n(rst_n), .run(run), .fast(1'b0), .ch_valid(ch_valid), .ch_i(ch_i), .ch_q(ch_q), .ch_k(ch_k),
                .ch_pos(ch_pos), .ch_last(ch_last), .ch_hop(ch_hop), .done(done),
                .fir_overrun(fir_overrun), .fft_ovf(fft_ovf), .out_sat(out_sat));
        end else if (TEST == 4) begin : g_dut
            channelizer #(.ROM_FILE("sim_data/iq_rom_t1.hex"), .N_SAMPLES(24576), .HOP_DIV(HOP_DIV)) dut (
                .clk(clk), .rst_n(rst_n), .run(run), .fast(1'b0), .ch_valid(ch_valid), .ch_i(ch_i), .ch_q(ch_q), .ch_k(ch_k),
                .ch_pos(ch_pos), .ch_last(ch_last), .ch_hop(ch_hop), .done(done),
                .fir_overrun(fir_overrun), .fft_ovf(fft_ovf), .out_sat(out_sat));
        end else if (TEST == 5) begin : g_dut
            channelizer #(.ROM_FILE("sim_data/iq_rom_n1.hex"), .N_SAMPLES(32768), .HOP_DIV(HOP_DIV)) dut (
                .clk(clk), .rst_n(rst_n), .run(run), .fast(1'b0), .ch_valid(ch_valid), .ch_i(ch_i), .ch_q(ch_q), .ch_k(ch_k),
                .ch_pos(ch_pos), .ch_last(ch_last), .ch_hop(ch_hop), .done(done),
                .fir_overrun(fir_overrun), .fft_ovf(fft_ovf), .out_sat(out_sat));
        end else if (TEST == 6) begin : g_dut
            channelizer #(.ROM_FILE("sim_data/iq_rom_sat.hex"), .N_SAMPLES(8192), .HOP_DIV(HOP_DIV)) dut (
                .clk(clk), .rst_n(rst_n), .run(run), .fast(1'b0), .ch_valid(ch_valid), .ch_i(ch_i), .ch_q(ch_q), .ch_k(ch_k),
                .ch_pos(ch_pos), .ch_last(ch_last), .ch_hop(ch_hop), .done(done),
                .fir_overrun(fir_overrun), .fft_ovf(fft_ovf), .out_sat(out_sat));
        end else begin : g_dut
            channelizer #(.ROM_FILE("sim_data/iq_rom_s1.hex"), .N_SAMPLES(65536), .HOP_DIV(HOP_DIV)) dut (
                .clk(clk), .rst_n(rst_n), .run(run), .fast(1'b0), .ch_valid(ch_valid), .ch_i(ch_i), .ch_q(ch_q), .ch_k(ch_k),
                .ch_pos(ch_pos), .ch_last(ch_last), .ch_hop(ch_hop), .done(done),
                .fir_overrun(fir_overrun), .fft_ovf(fft_ovf), .out_sat(out_sat));
        end
    endgenerate

    // 기대값과 (잡음 시험용) 입력
    reg [31:0] refm [0:N_OUT-1];
    reg [31:0] rom  [0:N_SAMP-1];
    initial begin
        case (TEST)
            1: $readmemh("sim_data/ch_ref_s2.hex", refm);
            2: $readmemh("sim_data/ch_ref_s3.hex", refm);
            3: $readmemh("sim_data/ch_ref_s4.hex", refm);
            4: $readmemh("sim_data/ch_ref_t1.hex", refm);
            5: begin $readmemh("sim_data/ch_ref_n1.hex", refm); $readmemh("sim_data/iq_rom_n1.hex", rom); end
            6: $readmemh("sim_data/ch_ref_sat.hex", refm);
            default: $readmemh("sim_data/ch_ref_s1.hex", refm);
        endcase
    end

    // 시간 초과 감시: done이 제때 서지 않으면 FAIL (예: 비우기 홉이 빠진 경우)
    initial begin
        repeat ((N_HOPS + 20) * HOP_DIV + 5000) @(posedge clk);
        $display("------------------------------------------------------------");
        $display("FAIL: timeout, done never asserted (channel outputs %0d / %0d)", n_out, N_OUT);
        $display("------------------------------------------------------------");
        $finish;
    end

    initial begin
        repeat (5) @(posedge clk);
        rst_n <= 1'b1;
        repeat (3) @(posedge clk);
        run <= 1'b1;
    end

    //--------------------------------------------------------------------
    // 비교와 측정
    //--------------------------------------------------------------------
    integer n_out = 0, n_err = 0, n_meta = 0;
    reg [31:0] exp_v;

    // T1: 톤 96개 x 채널 0..4, 톤당 32홉 중 18..31홉(정상 상태)의 평균 전력
    real    tpow [0:96*5-1];
    integer ti, step, inner;
    // N1: 홉 20부터 채널 출력 전력
    real    npow;
    integer ncnt;
    real    chpow [0:15];
    integer chcnt [0:15];
    initial begin
        for (ti = 0; ti < 96 * 5; ti = ti + 1) tpow[ti] = 0.0;
        for (ti = 0; ti < 16; ti = ti + 1) begin chpow[ti] = 0.0; chcnt[ti] = 0; end
        npow = 0.0; ncnt = 0;
    end

    always @(posedge clk) begin
        if (ch_valid) begin
            if (n_out < N_OUT) begin
                exp_v = refm[n_out];
                if ({ch_i, ch_q} !== exp_v) begin
                    n_err = n_err + 1;
                    if (n_err <= 5)
                        $display("  MISMATCH hop %0d pos %0d (ch %0d): got (%0d, %0d) exp (%0d, %0d)", n_out / 16, n_out % 16,
                                 ch_k, ch_i, ch_q, $signed(exp_v[31:16]), $signed(exp_v[15:0]));
                end
                if (ch_pos !== n_out % 16 || ch_k !== {ch_pos[0], ch_pos[1], ch_pos[2], ch_pos[3]} ||
                    ch_hop !== n_out / 16 || ch_last !== (n_out % 16 == 15)) begin
                    n_meta = n_meta + 1;
                    if (n_meta <= 5) $display("  META out %0d: hop %0d pos %0d ch %0d last %0d", n_out, ch_hop, ch_pos, ch_k, ch_last);
                end
                if (TEST == 4 && ch_k <= 4) begin
                    step  = ch_hop / 32;
                    inner = ch_hop % 32;
                    if (inner >= 18)
                        tpow[step * 5 + ch_k] = tpow[step * 5 + ch_k] + 1.0 * ch_i * ch_i + 1.0 * ch_q * ch_q;
                end
                if (TEST == 5 && ch_hop >= 20) begin
                    npow = npow + 1.0 * ch_i * ch_i + 1.0 * ch_q * ch_q;
                    ncnt = ncnt + 1;
                    chpow[ch_k] = chpow[ch_k] + 1.0 * ch_i * ch_i + 1.0 * ch_q * ch_q;
                    chcnt[ch_k] = chcnt[ch_k] + 1;
                end
            end
            n_out = n_out + 1;
        end
    end

    //--------------------------------------------------------------------
    // 판정
    //--------------------------------------------------------------------
    real    ref_p, db, off, pb_max, pb_min, adj_max, far_max, pin, red, red_min, red_max, rch;
    integer s, k, n_measure_fail;
    initial begin
        n_measure_fail = 0;
        wait (done);
        repeat (50) @(posedge clk);
        $display("------------------------------------------------------------");
        case (TEST)
            1: $display("tb_channelizer TEST 1 (scenario S2): HOP_DIV %0d, hops %0d", HOP_DIV, N_HOPS);
            2: $display("tb_channelizer TEST 2 (scenario S3): HOP_DIV %0d, hops %0d", HOP_DIV, N_HOPS);
            3: $display("tb_channelizer TEST 3 (scenario S4): HOP_DIV %0d, hops %0d", HOP_DIV, N_HOPS);
            4: $display("tb_channelizer TEST 4 (T1 tone sweep): HOP_DIV %0d, hops %0d", HOP_DIV, N_HOPS);
            5: $display("tb_channelizer TEST 5 (N1 noise only): HOP_DIV %0d, hops %0d", HOP_DIV, N_HOPS);
            6: $display("tb_channelizer TEST 6 (output saturation, out_sat = 1 expected): HOP_DIV %0d, hops %0d", HOP_DIV, N_HOPS);
            default: $display("tb_channelizer TEST 0 (scenario S1): HOP_DIV %0d, hops %0d", HOP_DIV, N_HOPS);
        endcase
        $display("  channel outputs %0d / %0d, data mismatches %0d, order errors %0d, fir_overrun %0d, fft_ovf %0d, out_sat %0d",
                 n_out, N_OUT, n_err, n_meta, fir_overrun, fft_ovf, out_sat);

        if (TEST == 4) begin
            // 톤 진폭 0.5 -> 기준 전력 = (0.5 * 32768)^2 x 14홉
            ref_p = (0.5 * 32768.0) * (0.5 * 32768.0) * 14.0;
            pb_max = -1000.0; pb_min = 1000.0; adj_max = -1000.0; far_max = -1000.0;
            for (s = 0; s < 96; s = s + 1) begin
                for (k = 0; k <= 4; k = k + 1) begin
                    off = 2.0 + (s - 48) / 32.0 - k;           // 톤 위치 - 채널 중심 (채널 간격 단위)
                    db  = 10.0 * $log10(tpow[s * 5 + k] / ref_p + 1.0e-30);
                    if (off <= 0.5 && off >= -0.5) begin
                        if (db > pb_max) pb_max = db;
                        if (db < pb_min) pb_min = db;
                    end
                    if (off == 1.0 || off == -1.0) begin
                        if (db > adj_max) adj_max = db;
                    end
                    if (off >= 1.5 || off <= -1.5) begin
                        if (db > far_max) far_max = db;
                    end
                end
            end
            $display("  response: passband (|offset| <= 0.5 ch) ripple %0.3f dB (%0.3f .. %0.3f)", pb_max - pb_min, pb_min, pb_max);
            $display("            1.0 ch away: max %0.1f dB (limit -40),  >= 1.5 ch away: max %0.1f dB (limit -80)", adj_max, far_max);
            $display("  channel 2 response by tone offset (0.25 ch steps):");
            for (s = 0; s < 96; s = s + 8) begin
                off = (s - 48) / 32.0;
                $display("    offset %6.2f ch (%6.0f kHz): %7.2f dB", off, off * 500.0,
                         10.0 * $log10(tpow[s * 5 + 2] / ref_p + 1.0e-30));
            end
            if (!(pb_max - pb_min <= 0.2 && adj_max <= -40.0 && far_max <= -80.0)) n_measure_fail = 1;
        end

        if (TEST == 5) begin
            pin = 0.0;
            for (s = 0; s < N_SAMP; s = s + 1)
                pin = pin + 1.0 * $signed(rom[s][31:16]) * $signed(rom[s][31:16]) + 1.0 * $signed(rom[s][15:0]) * $signed(rom[s][15:0]);
            pin = pin / N_SAMP;
            red = 10.0 * $log10(pin / (npow / ncnt));
            red_min = 1000.0; red_max = -1000.0;
            for (k = 0; k < 16; k = k + 1) begin
                rch = 10.0 * $log10(pin / (chpow[k] / chcnt[k]));
                if (rch < red_min) red_min = rch;
                if (rch > red_max) red_max = rch;
            end
            $display("  noise: input power %0.1f, channel power %0.1f -> reduction %0.2f dB (per channel %0.2f .. %0.2f, limit 10.7 +- 0.5)",
                     pin, npow / ncnt, red, red_min, red_max);
            if (!(red >= 10.2 && red <= 11.2)) n_measure_fail = 1;
        end

        if (n_out == N_OUT && n_err == 0 && n_meta == 0 && !fir_overrun && !fft_ovf && (out_sat == (TEST == 6)) && n_measure_fail == 0)
            if (TEST == 4 || TEST == 5) $display("PASS: channelizer matches bit-accurate model (diff 0) and measurement within limits");
            else                        $display("PASS: channelizer matches bit-accurate model (diff 0)");
        else
            $display("FAIL");
        $display("------------------------------------------------------------");
        $finish;
    end

endmodule
