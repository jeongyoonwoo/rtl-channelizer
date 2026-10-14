# rtl-channelizer 7단계: 보드 최상위 테스트벤치 (저장소 루트에서: do sim/run_stage7.do)
# SCEN 1 = S1, 5 = S5. 데모 속도와 빠른 모드(SW[3])로 두 번 재생해 HEX·LED 표시값이 같은지 본다 (시험 하나에 수 분)
vlib work
vlog rtl/pfb_fir.v rtl/fft16_sdf.v rtl/channelizer.v rtl/cordic_pipe.v rtl/pdw_detector_tdm.v rtl/chz_detect.v rtl/pdw_fifo.v rtl/chz_framer.v rtl/chz_top.v rtl/chz_stream_mon.v rtl/de10_chz_top.v tb/tb_de10_chz_top.v
foreach s {1 5} {
    vsim -onfinish stop -gSCEN=$s work.tb_de10_chz_top
    run -all
    quit -sim
}
