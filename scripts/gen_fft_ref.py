#!/usr/bin/env python3
"""
gen_fft_ref.py  --  3단계(16점 SDF FFT) 테스트벤치 기대값

입력 파일은 FIR 출력과 같은 형식(한 줄 = {r[17:0], i[17:0]} 36비트, 블록마다 16줄 = FFT 입력 위치 q 순서).
기대값은 블록마다 출력 위치 j = 0..15 순서(빈 bitrev(j))로 16줄, 같은 36비트 형식.

  TEST 0  fft_in = fir_ref_s1.hex (S1의 FIR 출력 8,192블록)      -> fft_ref_s1.hex     포화 없음
  TEST 1  fft_in_frand.hex  18비트 최대 진폭 난수 256블록         -> fft_ref_frand.hex  포화 있음(ovf = 1 기대)
  TEST 2  fft_in_tone.hex   빈 k0마다 단일 톤 16블록 x 16빈       -> fft_ref_tone.hex   최대 출력이 빈 k0
          (w_q = A exp(+j 2 pi k0 q / 16), A = 100000 -> X[k0] = A, 나머지 빈 ~0)

사용법: python3 scripts/gen_fft_ref.py sim_data
"""
import os, sys
import numpy as np
import chz_model as C


def s18(v):
    return v - (1 << 18) if v >= (1 << 17) else v


def read36(path):
    r, i = [], []
    for line in open(path):
        v = int(line, 16)
        r.append(s18(v >> 18)); i.append(s18(v & 0x3FFFF))
    n = len(r) // 16
    return np.array(r, np.int64).reshape(n, 16), np.array(i, np.int64).reshape(n, 16)


def write36(path, r, i):
    with open(path, "w") as f:
        for a, b in zip(r.ravel(), i.ravel()):
            f.write("%09X\n" % (((int(a) & 0x3FFFF) << 18) | (int(b) & 0x3FFFF)))


def fft_out_order(wr, wi, ovf):
    Xr, Xi = C.fft16_bit(wr, wi, ovf)               # 자연 순서 빈
    j = np.arange(16)
    k = np.array([C.BITREV4[x] for x in j])
    return Xr[:, k], Xi[:, k]                        # 출력 위치 j 순서


def main():
    d = sys.argv[1] if len(sys.argv) > 1 else "sim_data"

    wr, wi = read36(os.path.join(d, "fir_ref_s1.hex"))
    ovf = C.OvfCounter(); r, i = fft_out_order(wr, wi, ovf)
    write36(os.path.join(d, "fft_ref_s1.hex"), r, i)
    print("fft_ref_s1: blocks %d, max |X| %d, saturations %d" % (r.shape[0], max(abs(r).max(), abs(i).max()), ovf.n))

    rng = np.random.default_rng(316)
    n = 256
    wr = rng.integers(-131072, 131072, (n, 16)); wi = rng.integers(-131072, 131072, (n, 16))
    ext = rng.random((n, 16)) < 0.3
    wr[ext] = rng.choice([-131072, 131071], ext.sum())
    ext = rng.random((n, 16)) < 0.3
    wi[ext] = rng.choice([-131072, 131071], ext.sum())
    write36(os.path.join(d, "fft_in_frand.hex"), wr, wi)
    ovf = C.OvfCounter(); r, i = fft_out_order(wr.astype(np.int64), wi.astype(np.int64), ovf)
    write36(os.path.join(d, "fft_ref_frand.hex"), r, i)
    print("fft_ref_frand: blocks %d, saturations %d (TB expects ovf = 1)" % (n, ovf.n))

    A = 100000; q = np.arange(16); rows_r, rows_i = [], []
    for k0 in range(16):
        for rep in range(16):
            ph = 2 * np.pi * (k0 * q / 16.0 + rep / 7.0)
            rows_r.append(np.round(A * np.cos(ph))); rows_i.append(np.round(A * np.sin(ph)))
    wr = np.array(rows_r, np.int64); wi = np.array(rows_i, np.int64)
    write36(os.path.join(d, "fft_in_tone.hex"), wr, wi)
    ovf = C.OvfCounter(); r, i = fft_out_order(wr, wi, ovf)
    write36(os.path.join(d, "fft_ref_tone.hex"), r, i)
    mag = r.astype(float) ** 2 + i.astype(float) ** 2
    peak_bin = [C.BITREV4[int(np.argmax(mag[b]))] for b in range(mag.shape[0])]
    ok = all(peak_bin[b] == b // 16 for b in range(len(peak_bin)))
    print("fft_ref_tone: blocks %d, peak bin == tone bin for all blocks: %s, saturations %d" % (mag.shape[0], ok, ovf.n))


if __name__ == "__main__":
    main()
