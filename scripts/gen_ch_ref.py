#!/usr/bin/env python3
"""
gen_ch_ref.py  --  4단계(채널라이저 통합) 추가 기대값

  ch_ref_s1~s4.hex, ch_ref_t1.hex 는 gen_channelizer.py가 이미 만든다. 여기서는 다시 계산해 같은지 확인하고,
  잡음 시험 입력을 만든다.

  iq_rom_n1.hex   잡음만 있는 입력 32,768샘플 (I, Q 각각 sigma 0.016, 시드 7)
  ch_ref_n1.hex   그 채널 출력 4,096홉 (FFT 출력 순서)
  -> 테스트벤치가 입력 전력과 채널 출력 전력의 비(잡음 감소량)를 직접 계산한다 (기준 10.7 +- 0.5 dB)
  iq_rom_sat.hex  출력 포화 시험 8,192샘플: 진폭 0.97 펄스 8개 (가장자리 오버슈트로 채널 출력이 1을 넘음)
  ch_ref_sat.hex  그 채널 출력 1,024홉 (포화 포함, out_sat = 1 기대)

사용법: python3 scripts/gen_ch_ref.py sim_data
"""
import os, sys
import numpy as np
import chz_model as C
from gen_fir_ref import read_rom, read_coef
from gen_channelizer import write_ch_ref, write_rom


def main():
    d = sys.argv[1] if len(sys.argv) > 1 else "sim_data"
    hq = read_coef(os.path.join(d, "coef_h.hex"))

    for tag in ("s1", "s2", "s3", "s4", "t1"):
        I, Q = read_rom(os.path.join(d, "iq_rom_%s.hex" % tag))
        ovf = C.OvfCounter()
        yi, yq = C.channelizer_bit(I, Q, hq, len(I) // 8, ovf)
        tmp = os.path.join(d, "_chk.hex")
        write_ch_ref(tmp, yi, yq)
        same = open(tmp).read() == open(os.path.join(d, "ch_ref_%s.hex" % tag)).read()
        os.remove(tmp)
        print("ch_ref_%s: hops %d, same as stage-1 file: %s, internal ovf %d, out sat %d"
              % (tag, len(I) // 8, same, ovf.n, ovf.out_sat.n))

    rng = np.random.default_rng(7)
    n, sigma = 32768, 0.016
    I = np.round(sigma * rng.standard_normal(n) * 32768).astype(np.int64)
    Q = np.round(sigma * rng.standard_normal(n) * 32768).astype(np.int64)
    write_rom(os.path.join(d, "iq_rom_n1.hex"), I, Q)
    yi, yq = C.channelizer_bit(I, Q, hq, n // 8)
    write_ch_ref(os.path.join(d, "ch_ref_n1.hex"), yi, yq)
    pin = np.mean(I.astype(float) ** 2 + Q.astype(float) ** 2)
    pout = np.mean(yi[20:].astype(float) ** 2 + yq[20:].astype(float) ** 2)
    print("n1: input power %.1f, channel power (hops >= 20) %.1f, reduction %.3f dB" % (pin, pout, 10 * np.log10(pin / pout)))

    import gen_channelizer as G
    rng = np.random.default_rng(97)
    ps = [G.Pulse(i, "S", 60 + 120 * i, 20, 0.97, (i - 4) * C.DF + 37e3 * (i % 3), rng.uniform(-3, 3)) for i in range(8)]
    I, Q = G.synthesize(ps, 0.004, rng, n_in=8192)
    write_rom(os.path.join(d, "iq_rom_sat.hex"), I, Q)
    ovf = C.OvfCounter()
    yi, yq = C.channelizer_bit(I, Q, hq, 1024, ovf)
    write_ch_ref(os.path.join(d, "ch_ref_sat.hex"), yi, yq)
    print("sat: hops 1024, output saturations %d (TB expects out_sat = 1), internal ovf %d" % (ovf.out_sat.n, ovf.n))


if __name__ == "__main__":
    main()
