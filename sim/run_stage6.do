# rtl-channelizer 6단계: 최상위 통합 (바이트 스트림 해독, 모델 프레임 비교, 정답표 대비 정확도)
# 저장소 루트에서: do sim/run_stage6.do   (시험 하나에 수 분, 전부 돌리면 오래 걸림)
# TEST 0~4 = S1~S5, READY 0 = 항상 1, 1 = 약 50%, 2 = 약 1/8
vlib work
vlog rtl/pfb_fir.v rtl/fft16_sdf.v rtl/channelizer.v rtl/cordic_pipe.v rtl/pdw_detector_tdm.v rtl/chz_detect.v rtl/pdw_fifo.v rtl/chz_framer.v rtl/chz_top.v tb/tb_chz_top.v
foreach {t d r} {0 50 0  4 50 1  1 50 0  2 50 0  3 50 0  0 16 2} {
    vsim -onfinish stop -gTEST=$t -gHOP_DIV=$d -gREADY=$r work.tb_chz_top
    run -all
    quit -sim
}
