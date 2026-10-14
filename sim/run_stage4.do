# rtl-channelizer 4단계: 채널라이저 통합 테스트 (저장소 루트에서: do sim/run_stage4.do)
# TEST 0~3 = S1~S4 비트 일치, 4 = T1 톤 스윕 응답 측정, 5 = N1 잡음 감소 측정, 6 = 출력 포화(out_sat = 1 기대)
vlib work
vlog rtl/pfb_fir.v rtl/fft16_sdf.v rtl/channelizer.v tb/tb_channelizer.v
foreach {t d} {4 50  5 50  6 50  0 50  1 50  2 50  3 50  0 16} {
    vsim -onfinish stop -gTEST=$t -gHOP_DIV=$d work.tb_channelizer
    run -all
    quit -sim
}
