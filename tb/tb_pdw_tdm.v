`timescale 1ns/1ps
//==============================================================================
// tb_pdw_tdm.v   (rtl-channelizer 5단계 단위 시험)
// pdw_detector_tdm에 판정 여유가 작은 크기·위상 열(tdm_in.hex, gen_tdm_ref.py)을 직접 넣고
// 소유 판정 전 사건(own 포함)과 소유 PDW, 글리치·비소유 개수가 모델과 같은지 확인한다.
//   PACE 0 = 홉마다 16개 연속 후 HOP_DIV까지 쉼, PACE 1 = 빈틈없이 연속(홉당 16클록, 이중 버퍼가 꼭 필요한 조건)
// ModelSim: vsim -onfinish stop -gPACE=1 work.tb_pdw_tdm
// $display 문자열은 영어
//==============================================================================

module tb_pdw_tdm;

    parameter PACE    = 1;
    parameter HOP_DIV = 50;
    localparam H = 4096;

    reg clk = 1'b0;
    always #10 clk = ~clk;
    reg rst_n = 1'b0;

    reg [31:0]  din  [0:H*16-1];
    reg [103:0] rref [0:4095];
    reg [31:0]  cnt  [0:2];
    initial begin
        $readmemh("sim_data/tdm_in.hex", din);
        $readmemh("sim_data/tdm_cnt.hex", cnt);
        $readmemh("sim_data/tdm_raw_ref.hex", rref, 0, cnt[0] - 1);
    end

    reg               in_valid = 1'b0, in_last = 1'b0;
    reg signed [15:0] in_mag = 0, in_phase = 0;
    reg  [3:0]        in_k = 0;
    wire              pdw_valid, raw_valid, raw_own, overrun;
    wire [3:0]        pdw_ch, raw_ch;
    wire [31:0]       pdw_toa, raw_toa, hop_cnt;
    wire [15:0]       pdw_pw, pdw_pa, pdw_freq, pdw_flags, raw_pw, raw_pa, raw_freq, raw_flags;
    wire [15:0]       glitch_cnt, not_owned_cnt;

    pdw_detector_tdm dut (
        .clk(clk), .rst_n(rst_n),
        .in_valid(in_valid), .in_mag(in_mag), .in_phase(in_phase), .in_k(in_k), .in_last(in_last),
        .pdw_valid(pdw_valid), .pdw_ch(pdw_ch), .pdw_toa(pdw_toa), .pdw_pw(pdw_pw),
        .pdw_pa(pdw_pa), .pdw_freq(pdw_freq), .pdw_flags(pdw_flags),
        .raw_valid(raw_valid), .raw_own(raw_own), .raw_ch(raw_ch), .raw_toa(raw_toa), .raw_pw(raw_pw),
        .raw_pa(raw_pa), .raw_freq(raw_freq), .raw_flags(raw_flags),
        .glitch_cnt(glitch_cnt), .not_owned_cnt(not_owned_cnt), .hop_cnt(hop_cnt), .overrun(overrun)
    );

    integer h, j;
    reg done_in = 1'b0;
    initial begin
        repeat (5) @(posedge clk);
        rst_n <= 1'b1;
        repeat (3) @(posedge clk);
        for (h = 0; h < H; h = h + 1) begin
            for (j = 0; j < ((PACE == 1) ? 16 : HOP_DIV); j = j + 1) begin
                if (j < 16) begin
                    in_valid <= 1'b1;
                    in_mag   <= din[h * 16 + j][31:16];
                    in_phase <= din[h * 16 + j][15:0];
                    in_k     <= {j[0], j[1], j[2], j[3]};
                    in_last  <= (j == 15);
                end else begin
                    in_valid <= 1'b0; in_last <= 1'b0;
                end
                @(posedge clk);
            end
        end
        in_valid <= 1'b0; in_last <= 1'b0;
        repeat (40) @(posedge clk);
        done_in = 1'b1;
    end

    integer n_raw = 0, n_err = 0, n_pdw = 0, n_pdw_err = 0, n_own = 0;
    reg [103:0] got;
    always @(posedge clk) begin
        if (raw_valid) begin
            got = {3'b000, raw_own, raw_ch, raw_toa, raw_pw, raw_pa, raw_freq, raw_flags};
            if (n_raw >= cnt[0] || got !== rref[n_raw]) begin
                n_err = n_err + 1;
                if (n_err <= 5) $display("  RAW MISMATCH #%0d: got %h  exp %h", n_raw, got, rref[n_raw]);
            end
            if (raw_own) n_own = n_own + 1;
            n_raw = n_raw + 1;
        end
        // 소유 PDW는 원시 사건과 같은 클록, 같은 값이어야 한다
        if (pdw_valid !== (raw_valid && raw_own) ||
            (pdw_valid && {pdw_ch, pdw_toa, pdw_pw, pdw_pa, pdw_freq, pdw_flags} !==
                          {raw_ch, raw_toa, raw_pw, raw_pa, raw_freq, raw_flags})) begin
            n_pdw_err = n_pdw_err + 1;
        end
        if (pdw_valid) n_pdw = n_pdw + 1;
    end

    initial begin
        wait (done_in);
        $display("------------------------------------------------------------");
        if (PACE == 1) $display("tb_pdw_tdm: PACE 1 (back-to-back, 16 clocks per hop), hops %0d", hop_cnt);
        else           $display("tb_pdw_tdm: PACE 0 (paced, HOP_DIV %0d), hops %0d", HOP_DIV, hop_cnt);
        $display("  raw events %0d / %0d, mismatches %0d (own decisions included)", n_raw, cnt[0], n_err);
        $display("  owned PDWs %0d, pdw/raw consistency errors %0d", n_pdw, n_pdw_err);
        $display("  glitch %0d (model %0d), not owned %0d (model %0d), overrun %0d",
                 glitch_cnt, cnt[1], not_owned_cnt, cnt[2], overrun);
        if (n_raw == cnt[0] && n_err == 0 && n_pdw_err == 0 && glitch_cnt == cnt[1] &&
            not_owned_cnt == cnt[2] && n_pdw + not_owned_cnt == n_raw && !overrun && hop_cnt == H)
            $display("PASS: pdw_detector_tdm matches bit-accurate model (diff 0)");
        else
            $display("FAIL");
        $display("------------------------------------------------------------");
        $finish;
    end

endmodule
