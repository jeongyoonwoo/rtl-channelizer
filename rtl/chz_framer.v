`timescale 1ns/1ps
//==============================================================================
// chz_framer.v   (rtl-channelizer 6단계, rtl-pdw의 pdw_framer.v를 22바이트로 고침)
// PDW 한 건을 22바이트 텔레메트리 프레임 하나로 만들어 바이트 스트림으로 내보낸다.
//
// 프레임 (상위 바이트 먼저)
//   idx  0~3   ASM        0x1ACFFC1D
//   idx  4~5   FRAME_CNT  프레임 순번 (65,536에서 0으로)
//   idx  6     N_PDW      1
//   idx  7     CH         채널 번호, 부호 있는 바이트 (-8~7)
//   idx  8~11  TOA
//   idx 12~13  PW
//   idx 14~15  PA
//   idx 16~17  FREQ       채널 안 주파수 (Q1.15, 500 kHz = 32768)
//   idx 18~19  FLAGS
//   idx 20~21  CRC        CRC-16-CCITT (0x1021, 초기값 CRC_INIT), idx 4~19의 16바이트
//   절대 주파수 = CH x 500 kHz + FREQ x 500 kHz / 32768  (CH -8은 -4.25~-3.75 MHz로 표기)
//
// 입력: PDW FIFO에서 s_valid / s_ready로 꺼낸다 (rtl-pdw는 보관 레지스터 1개라 바쁠 때 버렸음)
//   s_ready = 비어 있거나 지금 막 마지막 바이트가 나가는 클록
// 출력: m_valid / m_ready / m_last. m_valid가 선 뒤 m_ready가 올 때까지 m_data, m_last는 바뀌지 않는다.
// ready가 항상 1이면 프레임 하나에 22클록.
//==============================================================================

module chz_framer #(
    parameter [15:0] CRC_INIT = 16'hFFFF
) (
    input  wire        clk,
    input  wire        rst_n,

    input  wire        s_valid,
    output wire        s_ready,
    input  wire [3:0]  s_ch,
    input  wire [31:0] s_toa,
    input  wire [15:0] s_pw,
    input  wire [15:0] s_pa,
    input  wire [15:0] s_freq,
    input  wire [15:0] s_flags,

    output wire [7:0]  m_data,
    output wire        m_valid,
    input  wire        m_ready,
    output wire        m_last,

    output reg  [15:0] frame_cnt       // 보낸 프레임 수
);

    //--------------------------------------------------------------------
    // CRC-16-CCITT 1바이트 갱신 (비트 직렬 8회를 조합 논리로 펼침)
    //--------------------------------------------------------------------
    function [15:0] crc16_byte;
        input [15:0] crc_in;
        input [7:0]  data;
        integer      k;
        reg   [15:0] c;
        begin
            c = crc_in ^ {data, 8'h00};
            for (k = 0; k < 8; k = k + 1)
                c = c[15] ? ((c << 1) ^ 16'h1021) : (c << 1);
            crc16_byte = c;
        end
    endfunction

    //--------------------------------------------------------------------
    // 상태
    //--------------------------------------------------------------------
    reg        busy;
    reg [4:0]  idx;          // 지금 내보내는 바이트 번호 0~21
    reg [7:0]  ch_h;
    reg [15:0] cnt_hold;     // 이 프레임의 FRAME_CNT
    reg [31:0] toa_h;
    reg [15:0] pw_h, pa_h, freq_h, flags_h;
    reg [15:0] crc;

    wire fire = busy && m_ready;            // 이번 클록에 바이트 하나가 나감
    wire done = fire && (idx == 5'd21);     // 이번 클록에 프레임 마지막 바이트가 나감
    assign s_ready = !busy || done;

    //--------------------------------------------------------------------
    // 바이트 선택 (모두 레지스터 값에서 고르므로 ready를 기다리는 동안 안정)
    //--------------------------------------------------------------------
    reg [7:0] byte_mux;
    always @(*) begin
        case (idx)
            5'd0:  byte_mux = 8'h1A;
            5'd1:  byte_mux = 8'hCF;
            5'd2:  byte_mux = 8'hFC;
            5'd3:  byte_mux = 8'h1D;
            5'd4:  byte_mux = cnt_hold[15:8];
            5'd5:  byte_mux = cnt_hold[7:0];
            5'd6:  byte_mux = 8'h01;
            5'd7:  byte_mux = ch_h;
            5'd8:  byte_mux = toa_h[31:24];
            5'd9:  byte_mux = toa_h[23:16];
            5'd10: byte_mux = toa_h[15:8];
            5'd11: byte_mux = toa_h[7:0];
            5'd12: byte_mux = pw_h[15:8];
            5'd13: byte_mux = pw_h[7:0];
            5'd14: byte_mux = pa_h[15:8];
            5'd15: byte_mux = pa_h[7:0];
            5'd16: byte_mux = freq_h[15:8];
            5'd17: byte_mux = freq_h[7:0];
            5'd18: byte_mux = flags_h[15:8];
            5'd19: byte_mux = flags_h[7:0];
            5'd20: byte_mux = crc[15:8];
            5'd21: byte_mux = crc[7:0];
            default: byte_mux = 8'h00;
        endcase
    end

    assign m_data  = byte_mux;
    assign m_valid = busy;
    assign m_last  = busy && (idx == 5'd21);

    //--------------------------------------------------------------------
    // 제어
    //--------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            busy      <= 1'b0;
            idx       <= 5'd0;
            ch_h      <= 8'd0;
            cnt_hold  <= 16'd0;
            toa_h     <= 32'd0;
            pw_h      <= 16'd0;
            pa_h      <= 16'd0;
            freq_h    <= 16'd0;
            flags_h   <= 16'd0;
            crc       <= CRC_INIT;
            frame_cnt <= 16'd0;
        end else begin
            // 바이트 전송
            if (fire) begin
                idx <= idx + 5'd1;
                if ((idx >= 5'd4) && (idx <= 5'd19))
                    crc <= crc16_byte(crc, byte_mux);
            end
            if (done) begin
                busy      <= 1'b0;
                frame_cnt <= frame_cnt + 16'd1;
            end

            // 새 PDW 받기: 비어 있거나 지금 막 프레임이 끝나는 클록이면 FIFO에서 꺼낸다
            if (s_valid && s_ready) begin
                busy     <= 1'b1;
                idx      <= 5'd0;
                crc      <= CRC_INIT;
                cnt_hold <= done ? (frame_cnt + 16'd1) : frame_cnt;
                ch_h     <= {{4{s_ch[3]}}, s_ch};
                toa_h    <= s_toa;
                pw_h     <= s_pw;
                pa_h     <= s_pa;
                freq_h   <= s_freq;
                flags_h  <= s_flags;
            end
        end
    end

endmodule
