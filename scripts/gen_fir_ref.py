#!/usr/bin/env python3
"""
gen_fir_ref.py  --  2단계(다상 FIR) 테스트벤치 기대값

  fir_ref_s1.hex      S1 입력(iq_rom_s1.hex) 8,192홉의 갈래 출력. 홉마다 FFT 입력 순서 q = 0..15로 16줄
                      한 줄 = {v_I[17:0], v_Q[17:0]} 36비트 (9자리 hex)
  iq_rom_frand.hex    최대 진폭 난수 입력 2,048샘플 (30%는 -32768 또는 +32767)
  fir_ref_frand.hex   그 입력의 갈래 출력 256홉
  iq_rom_fimp.hex     임펄스 입력 4,096샘플: -32768 임펄스를 137샘플 간격으로 둠(위치가 mod 16으로 모두 돎)
                      -32768 x 홀수 계수는 하위 16비트가 정확히 0x8000이라 반올림 경계(동점)를 일부러 만든다
  fir_ref_fimp.hex    그 입력의 갈래 출력 512홉

ROM 파일을 다시 읽어 계산하므로 RTL이 읽는 입력과 기대값이 항상 같은 데이터에서 나온다.
사용법: python3 scripts/gen_fir_ref.py sim_data
"""
import os, sys
import numpy as np
import chz_model as C


def read_rom(path):
    I, Q = [], []
    for line in open(path):
        v = int(line, 16)
        i, q = v >> 16, v & 0xFFFF
        I.append(i - 65536 if i >= 32768 else i)
        Q.append(q - 65536 if q >= 32768 else q)
    return np.array(I, np.int64), np.array(Q, np.int64)


def read_coef(path):
    h = [int(l, 16) for l in open(path)]
    return np.array([v - (1 << 18) if v >= (1 << 17) else v for v in h], np.int64)


def write_ref(path, vi, vq):
    with open(path, "w") as f:
        for m in range(vi.shape[0]):
            for q in range(16):
                f.write("%09X\n" % (((int(vi[m, q]) & 0x3FFFF) << 18) | (int(vq[m, q]) & 0x3FFFF)))


def main():
    d = sys.argv[1] if len(sys.argv) > 1 else "sim_data"
    hq = read_coef(os.path.join(d, "coef_h.hex"))
    assert np.array_equal(hq, C.quantize_coef(C.design_prototype()))

    I, Q = read_rom(os.path.join(d, "iq_rom_s1.hex"))
    ovf = C.OvfCounter()
    vi, vq = C.fir_bit(I, Q, hq, len(I) // 8, ovf)
    write_ref(os.path.join(d, "fir_ref_s1.hex"), vi, vq)
    print("fir_ref_s1: hops %d, max |v| %d, sat %d" % (vi.shape[0], max(abs(vi).max(), abs(vq).max()), ovf.n))

    rng = np.random.default_rng(2026)
    n = 2048
    I = rng.integers(-32768, 32768, n); Q = rng.integers(-32768, 32768, n)
    ext = rng.random(n) < 0.3
    I[ext] = rng.choice([-32768, 32767], ext.sum()); Q[ext] = rng.choice([-32768, 32767], ext.sum())
    with open(os.path.join(d, "iq_rom_frand.hex"), "w") as f:
        for i, q in zip(I, Q):
            f.write("%04X%04X\n" % (int(i) & 0xFFFF, int(q) & 0xFFFF))
    ovf = C.OvfCounter()
    vi, vq = C.fir_bit(I.astype(np.int64), Q.astype(np.int64), hq, n // 8, ovf)
    write_ref(os.path.join(d, "fir_ref_frand.hex"), vi, vq)
    print("fir_ref_frand: hops %d, max |v| %d, sat %d" % (vi.shape[0], max(abs(vi).max(), abs(vq).max()), ovf.n))

    n = 4096
    I = np.zeros(n, np.int64); Q = np.zeros(n, np.int64)
    for k, pos in enumerate(range(5, n - 130, 137)):
        I[pos] = -32768
        Q[pos] = -32768 if k % 2 else 32767
    with open(os.path.join(d, "iq_rom_fimp.hex"), "w") as f:
        for i, q in zip(I, Q):
            f.write("%04X%04X\n" % (int(i) & 0xFFFF, int(q) & 0xFFFF))
    ovf = C.OvfCounter()
    vi, vq = C.fir_bit(I, Q, hq, n // 8, ovf)
    write_ref(os.path.join(d, "fir_ref_fimp.hex"), vi, vq)
    odd = int(np.count_nonzero(hq % 2))
    print("fir_ref_fimp: hops %d, impulses %d, odd coefficients %d/128" % (vi.shape[0], len(range(5, n - 130, 137)), odd))


if __name__ == "__main__":
    main()
