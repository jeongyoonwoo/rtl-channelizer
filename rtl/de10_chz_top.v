`timescale 1ns/1ps
//==============================================================================
// de10_chz_top.v   (rtl-channelizer 7단계, DE10-Standard 보드 최상위)
// 포트 이름은 Terasic DE10-Standard 기본 최상위(golden top / System Builder)와 같다.
// 핀 배정은 그 qsf의 CLOCK_50, KEY, SW, LEDR, HEX0~HEX5 줄을 그대로 쓰면 된다 (rtl-pdw와 같음).
//
// 조작
//   KEY[0] : 리셋 (누르면 처음부터 다시)
//   SW[0]  : 1이면 재생 시작 (65,536샘플 = 8,192홉. 데모 속도 약 8.2 ms, 빠른 모드 약 2.6 ms)
//   SW[3]  : 0 = 데모 속도 (홉당 50클록, 입력 8 MSPS 상당), 1 = 최대 처리량 (홉당 16클록, 입력 25 MSPS 상당)
//            재생 전에 정한다. 두 속도에서 표시값이 같아야 한다 (처리량 시험)
//   SW[2:1]: 표시 선택
//     00 : HEX5~4 정상 프레임 수 하위 8비트, HEX3~0 스트림 서명
//     01 : HEX5~4 FIFO 넘침 수,           HEX3~0 오류 프레임 수        (모두 0이어야 함)
//     10 : HEX5~4 FIFO 최대 점유,         HEX3~0 소유하지 않아 버린 펄스 수 (이웃 채널 유령 억제)
//     11 : HEX5~4 00,                     HEX3~0 글리치 수 (MIN_W보다 짧아 버린 펄스)
// LED
//   LEDR[0] 끝 / [1] 처리 overrun·FFT 포화 / [2] FIFO 넘침 / [3] 오류 프레임 / [4] 합격
//   LEDR[5] 출력 16비트 포화 / [6] 빠른 모드 / [8] 동작 표시(약 1.5Hz) / [9] SW[0]
//   합격: 정상 프레임 수와 서명이 기대값과 같고, 오류·넘침·overrun·포화가 없음
//
// 시나리오는 합성할 때 SCEN 파라미터(1~5)로 고른다. ROM 파일은 프로젝트 폴더 기준 경로다.
// 출력 스트림의 ready는 1로 고정한다(받는 쪽이 항상 준비됨). 기대값 (SW[2:1] = 00 표시):
//   S1 CC 3356 / S2 2A FE8A / S3 09 41C7 / S4 1F 0ADC / S5 70 9491 (624 = 0x270, 하위 8비트)
//==============================================================================

module de10_chz_top #(
    parameter SCEN = 1
) (
    input  wire        CLOCK_50,
    input  wire [3:0]  KEY,
    input  wire [9:0]  SW,
    output wire [9:0]  LEDR,
    output wire [6:0]  HEX0,
    output wire [6:0]  HEX1,
    output wire [6:0]  HEX2,
    output wire [6:0]  HEX3,
    output wire [6:0]  HEX4,
    output wire [6:0]  HEX5
);

    // 파일 이름 길이가 모두 같아야 한다 (문자열 선택)
    localparam ROM_FILE = (SCEN == 2) ? "sim_data/iq_rom_s2.hex" :
                          (SCEN == 3) ? "sim_data/iq_rom_s3.hex" :
                          (SCEN == 4) ? "sim_data/iq_rom_s4.hex" :
                          (SCEN == 5) ? "sim_data/iq_rom_s5.hex" : "sim_data/iq_rom_s1.hex";
    localparam [15:0] EXP_SIG    = (SCEN == 2) ? 16'hFE8A : (SCEN == 3) ? 16'h41C7 :
                                   (SCEN == 4) ? 16'h0ADC : (SCEN == 5) ? 16'h9491 : 16'h3356;
    localparam [15:0] EXP_FRAMES = (SCEN == 2) ? 16'd42   : (SCEN == 3) ? 16'd9    :
                                   (SCEN == 4) ? 16'd31   : (SCEN == 5) ? 16'd624  : 16'd204;

    wire clk = CLOCK_50;

    //--------------------------------------------------------------------
    // 리셋: 누르면 바로, 뗄 때는 클록에 맞춰 풀림 (2단 동기화)
    //--------------------------------------------------------------------
    reg [1:0] rst_sync;
    always @(posedge clk or negedge KEY[0])
        if (!KEY[0]) rst_sync <= 2'b00;
        else         rst_sync <= {rst_sync[0], 1'b1};
    wire rst_n = rst_sync[1];

    // 스위치 동기화
    reg [1:0] run_s, fast_s, s1_s, s2_s;
    always @(posedge clk or negedge rst_n)
        if (!rst_n) begin run_s <= 2'b00; fast_s <= 2'b00; s1_s <= 2'b00; s2_s <= 2'b00; end
        else begin
            run_s  <= {run_s[0],  SW[0]};
            fast_s <= {fast_s[0], SW[3]};
            s1_s   <= {s1_s[0],   SW[1]};
            s2_s   <= {s2_s[0],   SW[2]};
        end
    wire       run  = run_s[1];
    wire       fast = fast_s[1];
    wire [1:0] sel  = {s2_s[1], s1_s[1]};

    //--------------------------------------------------------------------
    // 처리 경로와 수신 모니터
    //--------------------------------------------------------------------
    wire [7:0]  m_data;
    wire        m_valid, m_last;
    wire        m_ready = 1'b1;
    wire        all_done, fir_ovr, fft_ovf, out_sat, det_ovr;
    wire [15:0] glitch_cnt, not_owned_cnt, fifo_ovf, frame_cnt;
    wire [5:0]  fifo_max;
    wire [31:0] hop_cnt;
    wire [15:0] good_frames, bad_frames, signature;

    chz_top #(.ROM_FILE(ROM_FILE)) u_top (
        .clk(clk), .rst_n(rst_n), .run(run), .fast(fast),
        .m_data(m_data), .m_valid(m_valid), .m_ready(m_ready), .m_last(m_last),
        .all_done(all_done), .fir_overrun(fir_ovr), .fft_ovf(fft_ovf), .out_sat(out_sat), .det_overrun(det_ovr),
        .glitch_cnt(glitch_cnt), .not_owned_cnt(not_owned_cnt), .fifo_ovf(fifo_ovf), .fifo_max(fifo_max),
        .frame_cnt(frame_cnt), .hop_cnt(hop_cnt)
    );

    chz_stream_mon u_mon (
        .clk(clk), .rst_n(rst_n),
        .s_data(m_data), .s_valid(m_valid), .s_ready(m_ready), .s_last(m_last),
        .good_frames(good_frames), .bad_frames(bad_frames), .signature(signature)
    );

    //--------------------------------------------------------------------
    // 표시
    //--------------------------------------------------------------------
    reg [24:0] hb;
    always @(posedge clk or negedge rst_n)
        if (!rst_n) hb <= 25'd0;
        else        hb <= hb + 25'd1;

    wire any_ovr = fir_ovr | fft_ovf | det_ovr;
    wire pass = all_done && (good_frames == EXP_FRAMES) && (signature == EXP_SIG) && (bad_frames == 16'd0)
                && (fifo_ovf == 16'd0) && !any_ovr && !out_sat;

    assign LEDR[0] = all_done;
    assign LEDR[1] = any_ovr;
    assign LEDR[2] = (fifo_ovf != 16'd0);
    assign LEDR[3] = (bad_frames != 16'd0);
    assign LEDR[4] = pass;
    assign LEDR[5] = out_sat;
    assign LEDR[6] = fast;
    assign LEDR[7] = 1'b0;
    assign LEDR[8] = hb[24];
    assign LEDR[9] = run;

    reg [15:0] low16;
    reg [7:0]  high8;
    always @(*) begin
        case (sel)
            2'b01:   begin high8 = fifo_ovf[7:0];       low16 = bad_frames;    end
            2'b10:   begin high8 = {2'b00, fifo_max};   low16 = not_owned_cnt; end
            2'b11:   begin high8 = 8'h00;               low16 = glitch_cnt;    end
            default: begin high8 = good_frames[7:0];    low16 = signature;     end
        endcase
    end

    seg7_hex u_h0 (.v(low16[3:0]),   .seg(HEX0));
    seg7_hex u_h1 (.v(low16[7:4]),   .seg(HEX1));
    seg7_hex u_h2 (.v(low16[11:8]),  .seg(HEX2));
    seg7_hex u_h3 (.v(low16[15:12]), .seg(HEX3));
    seg7_hex u_h4 (.v(high8[3:0]),   .seg(HEX4));
    seg7_hex u_h5 (.v(high8[7:4]),   .seg(HEX5));

endmodule

//------------------------------------------------------------------------------
// 16진 한 자리 -> 7세그먼트 (DE10 보드는 0일 때 켜짐, 비트 순서 {g,f,e,d,c,b,a})
//------------------------------------------------------------------------------
module seg7_hex (
    input  wire [3:0] v,
    output reg  [6:0] seg
);
    always @(*) begin
        case (v)
            4'h0: seg = 7'b1000000;
            4'h1: seg = 7'b1111001;
            4'h2: seg = 7'b0100100;
            4'h3: seg = 7'b0110000;
            4'h4: seg = 7'b0011001;
            4'h5: seg = 7'b0010010;
            4'h6: seg = 7'b0000010;
            4'h7: seg = 7'b1111000;
            4'h8: seg = 7'b0000000;
            4'h9: seg = 7'b0010000;
            4'hA: seg = 7'b0001000;
            4'hB: seg = 7'b0000011;
            4'hC: seg = 7'b1000110;
            4'hD: seg = 7'b0100001;
            4'hE: seg = 7'b0000110;
            default: seg = 7'b0001110;   // F
        endcase
    end
endmodule
