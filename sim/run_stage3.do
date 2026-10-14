# rtl-channelizer 3단계: 16점 SDF FFT 테스트 (저장소 루트에서: do sim/run_stage3.do)
# TEST 0 = S1 FIR 출력, 1 = 최대 진폭 난수(포화, ovf = 1 기대), 2 = 빈마다 단일 톤
# PACE 0 = HOP_DIV 클록마다 16개 연속, 1 = 무작위로 끊기는 입력
vlib work
vlog rtl/fft16_sdf.v tb/tb_fft16.v
foreach {t d p} {0 50 0  0 16 0  0 50 1  1 50 0  1 50 1  2 50 0  2 50 1} {
    vsim -onfinish stop -gTEST=$t -gHOP_DIV=$d -gPACE=$p work.tb_fft16
    run -all
    quit -sim
}
