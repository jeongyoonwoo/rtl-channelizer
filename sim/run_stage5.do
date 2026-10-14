# rtl-channelizer 5단계: 파이프라인 CORDIC, 시분할 검출기, 통합 (저장소 루트에서: do sim/run_stage5.do)
vlib work
vlog rtl/cordic_engine.v rtl/cordic_pipe.v rtl/pdw_detector.v rtl/pdw_detector_tdm.v rtl/chz_detect.v rtl/pfb_fir.v rtl/fft16_sdf.v rtl/channelizer.v tb/tb_cordic_pipe.v tb/tb_pdw_tdm.v tb/tb_chz_detect.v
vsim -onfinish stop work.tb_cordic_pipe
run -all
quit -sim
foreach p {1 0} {
    vsim -onfinish stop -gPACE=$p work.tb_pdw_tdm
    run -all
    quit -sim
}
foreach t {0 1 2 3} {
    vsim -onfinish stop -gTEST=$t work.tb_chz_detect
    run -all
    quit -sim
}
