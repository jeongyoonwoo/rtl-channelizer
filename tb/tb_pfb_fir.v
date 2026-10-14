`timescale 1ns/1ps
//==============================================================================
// tb_pfb_fir.v   (rtl-channelizer 2단계)
// pfb_fir 출력이 비트 단위 모델(fir_ref_*.hex)과 같은지 확인한다.
//   - HOP_DIV 클록마다 입력 8샘플을 연속으로 넣는다 (50 = 데모 속도, 16 = 최대 처리량)
//   - 출력 갈래를 차례로 받아 {v_I, v_Q}, 위치 q, 홀짝, last를 모두 비교
//   - overrun이 서면 FAIL
// $display 문자열은 영어 (ModelSim 콘솔 한글 깨짐 방지)
//==============================================================================

module tb_pfb_fir;

    // TEST 0 = S1 (8,192홉), 1 = 최대 진폭 난수 (256홉), 2 = 반올림 경계 임펄스 (512홉)
    // (ModelSim: vsim -gTEST=1 -gHOP_DIV=16 tb_pfb_fir  -- 정수만 넘기면 되도록 파일 이름은 안에서 고름)
    parameter TEST      = 0;
    parameter HOP_DIV   = 50;            // 홉 간격 (클록), 16 이상
    parameter N_HOPS_OVR = 0;            // 0이 아니면 앞에서 이 홉 수만 확인
    parameter COEF_FILE = "sim_data/coef_h.hex";

    localparam N_HOPS = (N_HOPS_OVR != 0) ? N_HOPS_OVR : ((TEST == 1) ? 256 : (TEST == 2) ? 512 : 8192);
    localparam N_SAMP = N_HOPS * 8;
    localparam N_OUT  = N_HOPS * 16;

    reg clk = 1'b0;
    always #10 clk = ~clk;               // 50 MHz
    reg rst_n = 1'b0;

    reg  [31:0] rom  [0:N_SAMP-1];
    reg  [35:0] refm [0:N_OUT-1];
    initial begin
        if (TEST == 1) begin
            $readmemh("sim_data/iq_rom_frand.hex", rom);
            $readmemh("sim_data/fir_ref_frand.hex", refm);
        end else if (TEST == 2) begin
            $readmemh("sim_data/iq_rom_fimp.hex", rom);
            $readmemh("sim_data/fir_ref_fimp.hex", refm);
        end else begin
            $readmemh("sim_data/iq_rom_s1.hex", rom);
            $readmemh("sim_data/fir_ref_s1.hex", refm);
        end
    end

    reg               s_valid = 1'b0;
    reg signed [15:0] s_i = 16'sd0, s_q = 16'sd0;
    wire              m_valid, m_odd, m_last, overrun;
    wire signed [17:0] m_i, m_q;
    wire [3:0]        m_idx;

    pfb_fir #(.COEF_FILE(COEF_FILE)) dut (
        .clk(clk), .rst_n(rst_n),
        .s_valid(s_valid), .s_i(s_i), .s_q(s_q),
        .m_valid(m_valid), .m_i(m_i), .m_q(m_q), .m_idx(m_idx),
        .m_odd(m_odd), .m_last(m_last), .overrun(overrun)
    );

    // 입력: HOP_DIV 클록마다 8샘플 연속
    integer hop, k;
    initial begin
        repeat (5) @(posedge clk);
        rst_n <= 1'b1;
        repeat (3) @(posedge clk);
        for (hop = 0; hop < N_HOPS; hop = hop + 1) begin
            for (k = 0; k < HOP_DIV; k = k + 1) begin
                if (k < 8) begin
                    s_valid <= 1'b1;
                    s_i     <= rom[hop * 8 + k][31:16];
                    s_q     <= rom[hop * 8 + k][15:0];
                end else begin
                    s_valid <= 1'b0;
                end
                @(posedge clk);
            end
        end
        s_valid <= 1'b0;
    end

    // 출력 비교
    integer n_out = 0, n_err = 0, n_meta = 0;
    reg [35:0] exp_v;
    always @(posedge clk) begin
        if (m_valid) begin
            if (n_out < N_OUT) begin
                exp_v = refm[n_out];
                if ({m_i, m_q} !== exp_v) begin
                    n_err = n_err + 1;
                    if (n_err <= 5)
                        $display("  MISMATCH out %0d (hop %0d q %0d): got I=%0d Q=%0d  exp I=%0d Q=%0d",
                                 n_out, n_out / 16, n_out % 16, m_i, m_q,
                                 $signed(exp_v[35:18]), $signed(exp_v[17:0]));
                end
                if (m_idx !== n_out % 16 || m_odd !== (n_out / 16) % 2 || m_last !== (n_out % 16 == 15)) begin
                    n_meta = n_meta + 1;
                    if (n_meta <= 5)
                        $display("  META out %0d: idx %0d odd %0d last %0d", n_out, m_idx, m_odd, m_last);
                end
            end
            n_out = n_out + 1;
        end
    end

    // 종료
    integer max_clk;
    initial begin
        max_clk = (N_HOPS + 4) * HOP_DIV + 200;
        repeat (max_clk) @(posedge clk);
        $display("------------------------------------------------------------");
        if (TEST == 1)      $display("tb_pfb_fir TEST 1 (full-scale random): HOP_DIV %0d, hops %0d", HOP_DIV, N_HOPS);
        else if (TEST == 2) $display("tb_pfb_fir TEST 2 (rounding-tie impulses): HOP_DIV %0d, hops %0d", HOP_DIV, N_HOPS);
        else                $display("tb_pfb_fir TEST 0 (scenario S1): HOP_DIV %0d, hops %0d", HOP_DIV, N_HOPS);
        $display("  outputs %0d / %0d, data mismatches %0d, order/flag errors %0d, overrun %0d",
                 n_out, N_OUT, n_err, n_meta, overrun);
        if (n_out == N_OUT && n_err == 0 && n_meta == 0 && overrun == 1'b0)
            $display("PASS: pfb_fir matches bit-accurate model (diff 0)");
        else
            $display("FAIL");
        $display("------------------------------------------------------------");
        $finish;
    end

endmodule
