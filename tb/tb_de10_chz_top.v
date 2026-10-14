`timescale 1ns/1ps
//==============================================================================
// tb_de10_chz_top.v   (rtl-channelizer 7단계 보드 최상위 테스트벤치)
// de10_chz_top을 보드처럼 조작(KEY[0], SW)하고 7세그먼트·LED에 보일 값을 그대로 읽어 출력한다.
// 보드에서 같은 값이 보여야 한다.
//   1회차: 데모 속도(SW[3] = 0)로 재생 -> SW[2:1] = 00, 01, 10, 11 표시값
//   2회차: 리셋 후 빠른 모드(SW[3] = 1)로 다시 재생 -> 표시값이 1회차와 같은지 (처리량 시험)
// ModelSim: vsim -onfinish stop work.tb_de10_chz_top   (-gSCEN=5 로 S5)
// $display 문자열은 영어
//==============================================================================

module tb_de10_chz_top;

    parameter SCEN = 1;

    reg        clk = 1'b0;
    reg  [3:0] key = 4'b1111;
    reg  [9:0] sw  = 10'd0;
    always #10 clk = ~clk;

    wire [9:0] led;
    wire [6:0] h0, h1, h2, h3, h4, h5;

    de10_chz_top #(.SCEN(SCEN)) dut (.CLOCK_50(clk), .KEY(key), .SW(sw), .LEDR(led),
        .HEX0(h0), .HEX1(h1), .HEX2(h2), .HEX3(h3), .HEX4(h4), .HEX5(h5));

    function [7:0] dig;
        input [6:0] s;
        begin
            case (s)
                7'b1000000: dig = "0"; 7'b1111001: dig = "1"; 7'b0100100: dig = "2";
                7'b0110000: dig = "3"; 7'b0011001: dig = "4"; 7'b0010010: dig = "5";
                7'b0000010: dig = "6"; 7'b1111000: dig = "7"; 7'b0000000: dig = "8";
                7'b0010000: dig = "9"; 7'b0001000: dig = "A"; 7'b0000011: dig = "b";
                7'b1000110: dig = "C"; 7'b0100001: dig = "d"; 7'b0000110: dig = "E";
                7'b0001110: dig = "F"; default:    dig = "?";
            endcase
        end
    endfunction

    reg [47:0] shown [0:3];               // 1회차 표시값 (6글자)
    reg [47:0] cur;

    task show;
        input [1:0] mode;
        begin
            sw[2:1] = mode;
            repeat (10) @(posedge clk);
            cur = {dig(h5), dig(h4), dig(h3), dig(h2), dig(h1), dig(h0)};
            $display("  SW[2:1]=%b  HEX5..0 = %s %s   LEDR[6:0] = %b (pass=%b done=%b ovr=%b fifo_ovf=%b bad=%b sat=%b fast=%b)",
                     mode, cur[47:32], cur[31:0], led[6:0], led[4], led[0], led[1], led[2], led[3], led[5], led[6]);
        end
    endtask

    integer t0, t_demo, t_fast, n_same;
    reg     pass1;
    initial begin
        // 전원 직후 리셋 한 번
        repeat (3) @(posedge clk);
        key[0] = 1'b0; repeat (5) @(posedge clk); key[0] = 1'b1;
        repeat (20) @(posedge clk);

        // 1회차: 데모 속도
        sw[3] = 1'b0;
        sw[0] = 1'b1;
        t0 = $time;
        wait (led[0]);
        t_demo = ($time - t0) / 1000;
        repeat (20) @(posedge clk);
        $display("==== run 1: demo speed (SW[3]=0), finished in %0d us ====", t_demo);
        show(2'b00); shown[0] = cur;
        show(2'b01); shown[1] = cur;
        show(2'b10); shown[2] = cur;
        show(2'b11); shown[3] = cur;
        pass1 = led[4];
        case (SCEN)
            2: $display("  expect SW[2:1]=00: 2A FE8A, 01: 00 0000, LEDR[4]=1");
            3: $display("  expect SW[2:1]=00: 09 41C7, 01: 00 0000, LEDR[4]=1");
            4: $display("  expect SW[2:1]=00: 1F 0ADC, 01: 00 0000, LEDR[4]=1");
            5: $display("  expect SW[2:1]=00: 70 9491, 01: 00 0000, LEDR[4]=1");
            default: $display("  expect SW[2:1]=00: CC 3356, 01: 00 0000, LEDR[4]=1");
        endcase

        // 2회차: 리셋 후 빠른 모드
        sw[0] = 1'b0;
        sw[2:1] = 2'b00;
        sw[3] = 1'b1;
        key[0] = 1'b0; repeat (5) @(posedge clk);
        if (led[4] || led[0]) $display("  reset check: FAIL (LEDs not cleared during reset)");
        else                  $display("  reset check: pass/done LEDs cleared");
        key[0] = 1'b1; repeat (5) @(posedge clk);
        sw[0] = 1'b1;
        t0 = $time;
        wait (led[0]);
        t_fast = ($time - t0) / 1000;
        repeat (20) @(posedge clk);
        $display("==== run 2: max throughput (SW[3]=1), finished in %0d us ====", t_fast);
        n_same = 0;
        show(2'b00); if (cur == shown[0]) n_same = n_same + 1;
        show(2'b01); if (cur == shown[1]) n_same = n_same + 1;
        show(2'b10); if (cur[31:0] == shown[2][31:0]) n_same = n_same + 1;   // FIFO 최대 점유(HEX5~4)는 속도에 따라 다를 수 있음
        show(2'b11); if (cur == shown[3]) n_same = n_same + 1;

        $display("------------------------------------------------------------");
        $display("tb_de10_chz_top SCEN %0d: run 1 %0d us (pass LED %0d), run 2 %0d us (pass LED %0d, %0.2fx faster), displays equal %0d / 4",
                 SCEN, t_demo, pass1, t_fast, led[4], 1.0 * t_demo / t_fast, n_same);
        if (pass1 && led[4] && n_same == 4) $display("PASS: board display matches expected values at both speeds");
        else                      $display("FAIL");
        $display("------------------------------------------------------------");
        $finish;
    end

    initial begin
        #40_000_000;
        $display("FAIL: timeout");
        $finish;
    end

endmodule
