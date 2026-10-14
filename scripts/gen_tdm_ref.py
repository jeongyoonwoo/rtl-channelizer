#!/usr/bin/env python3
"""
gen_tdm_ref.py  --  5단계 시분할 검출기 단위 시험(판정 여유가 작은 입력)

시나리오 입력으로는 소유 판정이 큰 여유로 갈려서, 이웃 크기를 잘못 읽거나 진폭 우세 합을 잘못 더해도
결과가 같게 나올 수 있다(오류 주입 D1, D3, D6이 통과했음). 그래서 검출기에 직접 넣을 크기·위상 열을
만들어 판정이 아슬아슬하게 갈리도록 한다.

  short_close : 짧은 펄스, 양쪽 이웃 크기가 0.85~1.15배 + 샘플마다 3% 흔들림 -> dl, dr이 0 근처
  tie_right   : 오른쪽 이웃 크기가 정확히 같음 -> dr = 0 (소유), 이웃은 dl = 0 (비소유)
  tie_left    : 왼쪽 이웃 크기가 정확히 같음 -> dl = 0 (비소유)
  hop_vary    : 이웃 크기가 홉마다 0.5~1.5배로 크게 바뀜 -> 다른 홉의 이웃을 읽으면 판정이 달라짐
  long_edge   : 긴 펄스(PW 16~40), 주파수가 소유 경계(+-16384) 근처 -> FREQ 소유 판정 경계
  exact_edge  : 잡음 없는 긴 펄스, FREQ가 정확히 16383 / 16384 / -16384 / -16385 -> 경계의 < 와 <= 를 가름
  그 밖에 배경 잡음(TH_LO 아래), 포화(32767) 펄스, PW 3 글리치

출력
  tdm_in.hex       홉마다 16줄, FFT 출력 순서 j (채널 k = bitrev(j)), 한 줄 = {mag[15:0], phase[15:0]}
  tdm_raw_ref.hex  소유 판정 전 사건, (끝난 홉, 채널) 순서. 한 줄 = {own(4), ch(4), toa(32), pw, pa, freq, flags} 26자리
  tdm_cnt.hex      3줄: 사건 수, 글리치 수, 비소유 수

사용법: python3 scripts/gen_tdm_ref.py sim_data
"""
import os, sys
import numpy as np
import chz_model as C

H = 4096


def main():
    d = sys.argv[1] if len(sys.argv) > 1 else "sim_data"
    rng = np.random.default_rng(55)
    mag = np.abs(rng.normal(0, 400, (H, 16)))
    ph = rng.integers(-32768, 32768, (H, 16)).astype(float)
    kinds = ["short_close", "tie_right", "tie_left", "hop_vary", "long_edge", "exact_edge", "sat", "glitch"]
    count = {k: 0 for k in kinds}
    t = 40
    while t < H - 80:
        kind = kinds[rng.integers(0, len(kinds))]
        count[kind] += 1
        c = int(rng.integers(0, 16))
        A = float(rng.uniform(4000, 28000))
        if kind in ("long_edge", "exact_edge"):
            pw = int(rng.integers(16, 41))
        elif kind == "glitch":
            pw = 3
        else:
            pw = int(rng.integers(4, 14))
        f = float(rng.choice([1, -1]) * rng.uniform(16300, 16470)) if kind == "long_edge" else float(rng.uniform(-30000, 30000))
        if kind == "exact_edge":
            f = float([16383, 16384, -16384, -16385][count[kind] % 4])
        p0 = float(rng.uniform(-32768, 32768))
        rl, rr = float(rng.uniform(0.3, 0.7)), float(rng.uniform(0.3, 0.7))
        if kind == "short_close":
            rl, rr = float(rng.uniform(0.85, 1.15)), float(rng.uniform(0.85, 1.15))
        for n in range(pw):
            h = t + n
            a = 32767.0 if kind == "sat" else A * (1 + rng.uniform(-0.03, 0.03))
            mag[h, c] = a
            ph[h, c] = p0 + f * n + (0.0 if kind == "exact_edge" else rng.normal(0, 30))
            if kind == "tie_right":
                mag[h, (c + 1) % 16] = a
                mag[h, (c - 1) % 16] = a * rl
            elif kind == "tie_left":
                mag[h, (c - 1) % 16] = a
                mag[h, (c + 1) % 16] = a * rr
            elif kind == "hop_vary":
                mag[h, (c - 1) % 16] = a * rng.uniform(0.5, 1.5)
                mag[h, (c + 1) % 16] = a * rng.uniform(0.5, 1.5)
            elif kind in ("short_close",):
                mag[h, (c - 1) % 16] = a * rl * (1 + rng.uniform(-0.03, 0.03))
                mag[h, (c + 1) % 16] = a * rr * (1 + rng.uniform(-0.03, 0.03))
            elif kind not in ("long_edge", "exact_edge"):
                mag[h, (c - 1) % 16] = a * rl
                mag[h, (c + 1) % 16] = a * rr
        t += pw + int(rng.integers(4, 30))

    mag = np.clip(np.round(mag), 0, 32767).astype(np.int64)
    ph = ((np.round(ph).astype(np.int64) + 32768) % 65536) - 32768

    raw = []; nown = ngl = 0
    for k in range(16):
        _, a, b = C.detect_channel(mag[:, k], ph[:, k], mag[:, (k - 1) % 16], mag[:, (k + 1) % 16], k, raw=raw)
        nown += a; ngl += b
    raw.sort(key=lambda e: (e['end'], e['ch']))

    with open(os.path.join(d, "tdm_in.hex"), "w") as f:
        for h in range(H):
            for j in range(16):
                k = C.BITREV4[j]
                f.write("%04X%04X\n" % (int(mag[h, k]) & 0xFFFF, int(ph[h, k]) & 0xFFFF))
    with open(os.path.join(d, "tdm_raw_ref.hex"), "w") as f:
        for e in raw:
            f.write("%01X%01X%08X%04X%04X%04X%04X\n" % (e['own'], e['ch'], e['toa'], e['pw'], e['pa'],
                                                       e['freq'] & 0xFFFF, e['flags']))
    with open(os.path.join(d, "tdm_cnt.hex"), "w") as f:
        f.write("%08X\n%08X\n%08X\n" % (len(raw), ngl, nown))

    own = sum(e['own'] for e in raw)
    ties = sum(1 for e in raw if not (e['flags'] & 1) and (e['dl'] == 0 or e['dr'] == 0))
    close = sum(1 for e in raw if not (e['flags'] & 1) and min(abs(e['dl']), abs(e['dr'])) < 0.05 * 32767 * 4)
    edge = sum(1 for e in raw if (e['flags'] & 1) and abs(abs(C._w16(e['freq'])) - 16384) <= 64)
    exact = sorted(C._w16(e['freq']) for e in raw if (e['flags'] & 1) and C._w16(e['freq']) in (16383, 16384, -16384, -16385))
    print("pulses placed:", count)
    print("raw events %d (owned %d, not owned %d), glitches %d" % (len(raw), own, nown, ngl))
    print("short pulses with dl or dr exactly 0: %d, short with |dl| or |dr| small: %d, long with FREQ within 64 of boundary: %d"
          % (ties, close, edge))
    print("long pulses with FREQ exactly at the boundary values:", exact)


if __name__ == "__main__":
    main()
