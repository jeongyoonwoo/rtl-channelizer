# rtl-channelizer 7단계: Quartus 프로젝트 만들기 (선택 사항, 새 프로젝트 마법사로 해도 됨)
# 저장소 루트에서 실행:  quartus_sh -t quartus/create_project.tcl
#   -> 저장소 루트에 de10_chz_top.qpf / .qsf가 생긴다 ($readmemh 경로가 프로젝트 폴더 기준이라 루트여야 함)
# 핀 배정은 넣지 않는다. rtl-pdw 프로젝트 .qsf의 set_location_assignment 줄과
# IO_STANDARD 줄(CLOCK_50, KEY, SW, LEDR, HEX0~5)을 새 .qsf 끝에 그대로 붙여 넣는다.
project_new de10_chz_top -overwrite
set_global_assignment -name FAMILY "Cyclone V"
set_global_assignment -name DEVICE 5CSXFC6D6F31C6
set_global_assignment -name TOP_LEVEL_ENTITY de10_chz_top
foreach f {pfb_fir.v fft16_sdf.v channelizer.v cordic_pipe.v pdw_detector_tdm.v chz_detect.v pdw_fifo.v chz_framer.v chz_top.v chz_stream_mon.v de10_chz_top.v} {
    set_global_assignment -name VERILOG_FILE rtl/$f
}
set_global_assignment -name SDC_FILE quartus/de10_chz_top.sdc
set_global_assignment -name PROJECT_OUTPUT_DIRECTORY output_files
project_close
