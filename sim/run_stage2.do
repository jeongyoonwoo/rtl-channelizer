# rtl-channelizer 2단계: 다상 FIR 테스트 (저장소 루트에서 실행: vsim -c -do sim/run_stage2.do 또는 GUI 콘솔에서 do sim/run_stage2.do)
vlib work
vlog rtl/pfb_fir.v tb/tb_pfb_fir.v
foreach {t d} {0 50  0 16  1 50  1 16  2 50} {
    vsim -onfinish stop -gTEST=$t -gHOP_DIV=$d work.tb_pfb_fir
    run -all
    quit -sim
}
