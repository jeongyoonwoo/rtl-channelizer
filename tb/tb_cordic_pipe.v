`timescale 1ns/1ps
//==============================================================================
// tb_cordic_pipe.v   (rtl-channelizer 5단계)
// cordic_pipe(파이프라인)와 cordic_engine(반복형, rtl-pdw에서 검증)에 같은 입력을 넣고
// 크기·위상이 모두 같은지 비교한다.
//   입력 N_VEC개: 앞 64개는 극단값 조합(-32768, -1, 0, 1, 32767, ...), 나머지는 난수
//   파이프라인에는 매 클록 1개(태그 = 순번 하위 비트), 반복형에는 끝날 때마다 1개씩
// $display 문자열은 영어
//==============================================================================

module tb_cordic_pipe;

    parameter N_VEC = 4096;

    reg clk = 1'b0;
    always #10 clk = ~clk;
    reg rst_n = 1'b0;

    reg signed [15:0] vx [0:N_VEC-1];
    reg signed [15:0] vy [0:N_VEC-1];
    reg signed [15:0] pm [0:N_VEC-1];      // 파이프라인 결과
    reg signed [15:0] pp [0:N_VEC-1];
    reg signed [15:0] em [0:N_VEC-1];      // 반복형 결과
    reg signed [15:0] ep [0:N_VEC-1];

    integer i, j, seed;
    reg signed [15:0] ext [0:7];
    initial begin
        ext[0] = -16'sd32768; ext[1] = -16'sd32767; ext[2] = -16'sd1; ext[3] = 16'sd0;
        ext[4] = 16'sd1;      ext[5] = 16'sd32766;  ext[6] = 16'sd32767; ext[7] = 16'sd12345;
        for (i = 0; i < 8; i = i + 1)
            for (j = 0; j < 8; j = j + 1) begin
                vx[i * 8 + j] = ext[i];
                vy[i * 8 + j] = ext[j];
            end
        seed = 777;
        for (i = 64; i < N_VEC; i = i + 1) begin
            vx[i] = $random(seed);
            vy[i] = $random(seed);
        end
    end

    // 파이프라인
    reg               p_in_v = 1'b0;
    reg signed [15:0] p_in_x = 0, p_in_y = 0;
    reg  [12:0]       p_in_tag = 0;
    wire              p_out_v;
    wire signed [15:0] p_out_m, p_out_p;
    wire [12:0]       p_out_tag;

    cordic_pipe #(.TAG_W(13)) u_pipe (
        .clk(clk), .rst_n(rst_n), .in_valid(p_in_v), .in_x(p_in_x), .in_y(p_in_y), .in_tag(p_in_tag),
        .out_valid(p_out_v), .out_mag(p_out_m), .out_phase(p_out_p), .out_tag(p_out_tag)
    );

    // 반복형
    reg               e_start = 1'b0;
    reg signed [15:0] e_x = 0, e_y = 0;
    wire signed [15:0] e_xo, e_zo;
    wire              e_done, e_busy;

    cordic_engine u_eng (
        .clk(clk), .rst_n(rst_n), .start(e_start), .mode(1'b1),
        .x_in(e_x), .y_in(e_y), .z_in(16'sd0),
        .x_out(e_xo), .y_out(), .z_out(e_zo), .done(e_done), .busy(e_busy)
    );

    integer n_pipe = 0, n_tag_err = 0;
    always @(posedge clk) begin
        if (p_out_v) begin
            if (p_out_tag !== n_pipe[12:0]) n_tag_err = n_tag_err + 1;
            pm[n_pipe] = p_out_m;
            pp[n_pipe] = p_out_p;
            n_pipe = n_pipe + 1;
        end
    end

    reg pipe_done = 1'b0, eng_done = 1'b0;
    initial begin
        repeat (5) @(posedge clk);
        rst_n <= 1'b1;
        repeat (3) @(posedge clk);
        // 파이프라인: 매 클록 1개
        for (i = 0; i < N_VEC; i = i + 1) begin
            p_in_v <= 1'b1; p_in_x <= vx[i]; p_in_y <= vy[i]; p_in_tag <= i;
            @(posedge clk);
        end
        p_in_v <= 1'b0;
        repeat (30) @(posedge clk);
        pipe_done = 1'b1;
    end

    initial begin
        repeat (8) @(posedge clk);
        for (j = 0; j < N_VEC; j = j + 1) begin
            e_start <= 1'b1; e_x <= vx[j]; e_y <= vy[j];
            @(posedge clk);
            e_start <= 1'b0;
            @(posedge clk);
            while (!e_done) @(posedge clk);
            em[j] = e_xo;
            ep[j] = e_zo;
        end
        eng_done = 1'b1;
    end

    integer n_err = 0;
    initial begin
        wait (pipe_done && eng_done);
        for (i = 0; i < N_VEC; i = i + 1) begin
            if (pm[i] !== em[i] || pp[i] !== ep[i]) begin
                n_err = n_err + 1;
                if (n_err <= 5)
                    $display("  MISMATCH %0d: in (%0d, %0d)  pipe mag %0d ph %0d  engine mag %0d ph %0d",
                             i, vx[i], vy[i], pm[i], pp[i], em[i], ep[i]);
            end
        end
        $display("------------------------------------------------------------");
        $display("tb_cordic_pipe: vectors %0d (64 extreme combinations + random), pipeline outputs %0d",
                 N_VEC, n_pipe);
        $display("  mismatches vs cordic_engine %0d, tag errors %0d", n_err, n_tag_err);
        if (n_pipe == N_VEC && n_err == 0 && n_tag_err == 0)
            $display("PASS: cordic_pipe matches cordic_engine (diff 0)");
        else
            $display("FAIL");
        $display("------------------------------------------------------------");
        $finish;
    end

endmodule
