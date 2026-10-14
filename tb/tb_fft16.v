`timescale 1ns/1ps
//==============================================================================
// tb_fft16.v   (rtl-channelizer 3단계)
// fft16_sdf 출력이 비트 단위 모델(fft_ref_*.hex)과 같은지 확인한다.
//   TEST 0 = S1의 FIR 출력 8,192블록, 1 = 18비트 최대 진폭 난수 256블록(포화 발생, ovf = 1 기대),
//   2 = 빈마다 단일 톤(최대 출력 빈 = 톤 빈인지 모델과 별개로 확인)
//   PACE 0 = HOP_DIV 클록마다 16개 연속, 1 = 매 클록 약 60% 확률로 띄엄띄엄(유효 게이팅 시험)
//   마지막에 0 블록 하나를 더 넣어 파이프라인을 비운다(출력은 검사하지 않음).
// $display 문자열은 영어 (ModelSim 콘솔 한글 깨짐 방지)
//==============================================================================

module tb_fft16;

    parameter TEST    = 0;
    parameter HOP_DIV = 50;          // PACE 0일 때 블록 간격(클록), 16 이상
    parameter PACE    = 0;
    parameter N_BLK_OVR = 0;         // 0이 아니면 앞에서 이 블록 수만 확인

    localparam N_BLK = (N_BLK_OVR != 0) ? N_BLK_OVR : ((TEST == 1 || TEST == 2) ? 256 : 8192);
    localparam N_IN  = N_BLK * 16;

    reg clk = 1'b0;
    always #10 clk = ~clk;
    reg rst_n = 1'b0;

    reg [35:0] din  [0:N_IN-1];
    reg [35:0] refm [0:N_IN-1];
    initial begin
        if (TEST == 1) begin
            $readmemh("sim_data/fft_in_frand.hex", din);
            $readmemh("sim_data/fft_ref_frand.hex", refm);
        end else if (TEST == 2) begin
            $readmemh("sim_data/fft_in_tone.hex", din);
            $readmemh("sim_data/fft_ref_tone.hex", refm);
        end else begin
            $readmemh("sim_data/fir_ref_s1.hex", din);
            $readmemh("sim_data/fft_ref_s1.hex", refm);
        end
    end

    reg               s_valid = 1'b0;
    reg signed [17:0] s_r = 18'sd0, s_i = 18'sd0;
    wire              m_valid, m_last, ovf;
    wire signed [17:0] m_r, m_i;
    wire [3:0]        m_pos, m_bin;

    fft16_sdf dut (
        .clk(clk), .rst_n(rst_n),
        .s_valid(s_valid), .s_r(s_r), .s_i(s_i),
        .m_valid(m_valid), .m_r(m_r), .m_i(m_i),
        .m_pos(m_pos), .m_bin(m_bin), .m_last(m_last), .ovf(ovf)
    );

    // 입력 (마지막에 0 블록 하나)
    integer blk, k, seed;
    reg     done_in = 1'b0;
    initial begin
        seed = 12345;
        repeat (5) @(posedge clk);
        rst_n <= 1'b1;
        repeat (3) @(posedge clk);
        for (blk = 0; blk <= N_BLK; blk = blk + 1) begin
            if (PACE == 0) begin
                for (k = 0; k < HOP_DIV; k = k + 1) begin
                    if (k < 16) begin
                        s_valid <= 1'b1;
                        s_r <= (blk < N_BLK) ? din[blk * 16 + k][35:18] : 18'sd0;
                        s_i <= (blk < N_BLK) ? din[blk * 16 + k][17:0]  : 18'sd0;
                    end else s_valid <= 1'b0;
                    @(posedge clk);
                end
            end else begin
                k = 0;
                while (k < 16) begin
                    if (($random(seed) & 255) < 154) begin
                        s_valid <= 1'b1;
                        s_r <= (blk < N_BLK) ? din[blk * 16 + k][35:18] : 18'sd0;
                        s_i <= (blk < N_BLK) ? din[blk * 16 + k][17:0]  : 18'sd0;
                        k = k + 1;
                    end else s_valid <= 1'b0;
                    @(posedge clk);
                end
            end
        end
        s_valid <= 1'b0;
        repeat (40) @(posedge clk);
        done_in = 1'b1;
    end

    // 출력 비교
    integer n_out = 0, n_err = 0, n_meta = 0, n_peak = 0;
    integer best_mag, cur_mag, best_bin, b;
    reg [35:0] exp_v;
    always @(posedge clk) begin
        if (m_valid && n_out < N_IN) begin
            exp_v = refm[n_out];
            if ({m_r, m_i} !== exp_v) begin
                n_err = n_err + 1;
                if (n_err <= 5)
                    $display("  MISMATCH blk %0d pos %0d: got (%0d, %0d) exp (%0d, %0d)", n_out / 16, n_out % 16,
                             m_r, m_i, $signed(exp_v[35:18]), $signed(exp_v[17:0]));
            end
            if (m_pos !== n_out % 16 || m_bin !== {m_pos[0], m_pos[1], m_pos[2], m_pos[3]} || m_last !== (n_out % 16 == 15)) begin
                n_meta = n_meta + 1;
                if (n_meta <= 5) $display("  META out %0d: pos %0d bin %0d last %0d", n_out, m_pos, m_bin, m_last);
            end
            // TEST 2: 블록 안에서 가장 큰 출력의 빈이 톤 빈(블록 번호 / 16)인지
            if (TEST == 2) begin
                cur_mag = (m_r / 16) * (m_r / 16) + (m_i / 16) * (m_i / 16);
                if (n_out % 16 == 0 || cur_mag > best_mag) begin
                    best_mag = cur_mag; best_bin = m_bin;
                end
                if (n_out % 16 == 15 && best_bin != (n_out / 16) / 16) begin
                    n_peak = n_peak + 1;
                    if (n_peak <= 5) $display("  PEAK blk %0d: peak bin %0d, tone bin %0d", n_out / 16, best_bin, (n_out / 16) / 16);
                end
            end
        end
        if (m_valid) n_out = n_out + 1;
    end

    initial begin
        wait (done_in);
        $display("------------------------------------------------------------");
        if (TEST == 1)      $display("tb_fft16 TEST 1 (full-scale random, saturation expected): PACE %0d HOP_DIV %0d, blocks %0d", PACE, HOP_DIV, N_BLK);
        else if (TEST == 2) $display("tb_fft16 TEST 2 (single tone per bin): PACE %0d HOP_DIV %0d, blocks %0d", PACE, HOP_DIV, N_BLK);
        else                $display("tb_fft16 TEST 0 (S1 FIR output): PACE %0d HOP_DIV %0d, blocks %0d", PACE, HOP_DIV, N_BLK);
        $display("  outputs %0d (checked %0d), data mismatches %0d, order/flag errors %0d, peak-bin errors %0d, ovf %0d (expected %0d)",
                 n_out, N_IN, n_err, n_meta, n_peak, ovf, (TEST == 1));
        if (n_out >= N_IN && n_err == 0 && n_meta == 0 && n_peak == 0 && ovf == (TEST == 1))
            $display("PASS: fft16_sdf matches bit-accurate model (diff 0)");
        else
            $display("FAIL");
        $display("------------------------------------------------------------");
        $finish;
    end

endmodule
