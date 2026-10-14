#!/usr/bin/env python3
"""
analyze_channelizer.py  --  1단계 분석 (RTL과 무관한 모델 수준 확인)

  A1 다상 분해 검증       다상 FIR + 회전 + FFT (부동소수점) vs 정의식, 상대 오차 < 1e-9
  A2 고정소수점 오차       S1~S4 전 구간에서 비트 모델 vs 부동소수점 모델, RMS <= 채널 잡음 -20 dB
  A3 잡음 억압             잡음만 넣었을 때 채널 출력 전력 / 입력 전력, 10.7 +- 0.5 dB
  A4 임계 vs 2배           같은 톤을 임계(256탭, 1 MHz 간격) / 2배(128탭) 채널라이저에 넣어 이웃 채널 성분 비교
  A5 동시 신호 분리 간격   두 신호의 간격을 1.0~2.5채널로 바꾸며 유령·누락 수 (긴 펄스, 짧은 펄스)
  A6 S4 광대역 대조군      채널화 없이 8 MSPS 원신호에 같은 검출기를 돌린 오검출 수

사용법: python3 scripts/analyze_channelizer.py sim_data
"""
import math, os, sys
import numpy as np
import chz_model as C
import gen_channelizer as G


def rms(v):
    return float(np.sqrt(np.mean(np.abs(v) ** 2)))


def a1(L):
    rng = np.random.default_rng(1)
    h = C.design_prototype()
    x = (rng.standard_normal(8 * 600) + 1j * rng.standard_normal(8 * 600)) * 0.2
    d = C.pfb_direct(x, h, 600); p = C.pfb_polyphase_float(x, h, 600)
    e = np.abs(d - p).max() / np.abs(d).max()
    L.append("A1 polyphase+FFT vs definition: max rel err %.2e  %s" % (e, "PASS" if e < 1e-9 else "FAIL"))
    return e < 1e-9


def a2(L):
    rng = np.random.default_rng(G.SEED)
    h = C.design_prototype(); hq = C.quantize_coef(h)
    ok = True
    for tag in ("s1", "s2", "s3", "s4"):
        gen, sigma = G.SCEN[tag]
        pulses = gen(rng); I, Q = G.synthesize(pulses, sigma, rng)
        yi, yq = C.channelizer_bit(I, Q, hq, G.N_HOP)
        yf = C.pfb_direct((I + 1j * Q) / 32768.0, h, G.N_HOP) * 32768.0
        err = (yi + 1j * yq) - yf
        # 성분(I 또는 Q) 기준으로 정의: 채널 잡음 = sigma * 32768 * 10^(-10.7/20), 오차 RMS = sqrt(mean|err|^2 / 2)
        ch_noise = sigma * 32768 * 10 ** (-10.7 / 20)
        lim = ch_noise * 10 ** (-20 / 20)
        e = err[20:]
        r = math.sqrt(np.mean(np.abs(e) ** 2) / 2)
        emax = max(np.abs(e.real).max(), np.abs(e.imag).max())
        ok &= r <= lim
        L.append("A2 %s fixed vs float (per I/Q component): err RMS %.3f LSB, max %.2f LSB (limit %.2f = channel noise %.1f LSB -20 dB)"
                 % (tag.upper(), r, emax, lim, ch_noise))
    L[-1] += "  %s" % ("PASS" if ok else "FAIL")
    return ok


def a3(L):
    rng = np.random.default_rng(7)
    hq = C.quantize_coef(C.design_prototype())
    sigma = 0.016; n = 65536
    I = np.round(sigma * rng.standard_normal(n) * 32768).astype(np.int64)
    Q = np.round(sigma * rng.standard_normal(n) * 32768).astype(np.int64)
    yi, yq = C.channelizer_bit(I, Q, hq, n // 8)
    pin = np.mean(I.astype(float) ** 2 + Q.astype(float) ** 2)
    pout = np.mean(yi[20:].astype(float) ** 2 + yq[20:].astype(float) ** 2, axis=0)
    red = 10 * np.log10(pin / pout)
    ok = abs(red.mean() - 10.7) <= 0.5
    L.append("A3 noise reduction per channel: mean %.2f dB (min %.2f, max %.2f over 16 ch)  %s"
             % (red.mean(), red.min(), red.max(), "PASS" if ok else "FAIL"))
    return ok


def a4(L):
    hc = C.design_prototype(L_=256, beta=7.0, fc_ch=0.5) / 16.0      # 임계: 합 1
    ho = C.design_prototype() / 16.0
    L.append("A4 critical (16 MSPS, 256 taps, 1 MHz spacing) vs 2x oversampled (8 MSPS, 128 taps, 500 kHz spacing)")
    L.append("    tone offset from ch3 | critical ch3 / ch4                  | 2x ch3 / ch4")
    for off in (0.30, 0.45, 0.55):
        row = []
        for h, Mh, fs in ((hc, 16, 16e6), (ho, 8, 8e6)):
            sp = fs / 16; fout = fs / Mh
            n = np.arange(4000); f = (3 + off) * sp
            x = 0.5 * np.exp(2j * np.pi * f * n / fs)
            Lh = len(h)
            idx = Mh * np.arange(4000 // Mh - Lh // Mh)[:, None] + Lh - 1 - np.arange(Lh)[None, :]
            xw = x[idx]
            cells = []
            for k in (3, 4):
                y = (xw * h[None, :] * np.exp(-2j * np.pi * k * idx / 16)).sum(axis=1)
                a = np.abs(y[20:]).mean() / 0.5
                fm = np.angle(y[21:] * np.conj(y[20:-1])).mean() / (2 * np.pi) * fout
                ft = f - k * sp
                wrong = abs(fm - ft) > 5e3
                cells.append("%6.1f dB %+5.0fk%s" % (20 * np.log10(a + 1e-12), fm / 1e3, " WRONG" if wrong else "      "))
            row.append(" / ".join(cells))
        L.append("    %+.2f ch             | %s | %s" % (off, row[0], row[1]))
    return True


def run_pair(sep_ch, pw, rng, hq, amp=(0.35, 0.35)):
    """두 신호 동시 펄스: f1을 채널 3 안 여러 위치에 두고 f2 = f1 + sep. 반환 (기대 수, 맞춘 수, 여분 수)."""
    n_exp = n_hit = n_extra = 0
    for a in (-0.42, -0.28, -0.14, 0.02, 0.16, 0.30, 0.44):     # 경계(+-0.5)에 정확히 떨어지지 않게
        f1 = (3 + a) * C.DF; f2 = f1 + sep_ch * C.DF
        ps = [G.Pulse(0, "p", 100, pw, amp[0], f1, rng.uniform(-3, 3)),
              G.Pulse(1, "p", 100, pw, amp[1], f2, rng.uniform(-3, 3))]
        I, Q = G.synthesize(ps, G.SIGMA, rng, n_in=8 * 300)
        yi, yq = C.channelizer_bit(I, Q, hq, 300)
        pdws, _, _, _, _ = C.detect_all(yi, yq)
        exp = [G.truth(p) for p in ps]
        res = G.evaluate("default", exp, pdws)
        n_exp += 2; n_hit += res['match']; n_extra += len(res['extra'])
    return n_exp, n_hit, n_extra


def a5(L):
    rng = np.random.default_rng(11)
    hq = C.quantize_coef(C.design_prototype())
    L.append("A5 simultaneous two-signal separation (7 positions each, equal amp 0.35 / unequal 0.20+0.45)")
    L.append("    sep [ch] (kHz) | long PW 25: hit/exp extra | short PW 6: hit/exp extra | long unequal | short unequal")
    for sep in (1.0, 1.25, 1.5, 1.75, 2.0, 2.25, 2.5):
        cols = []
        for amp in ((0.35, 0.35), (0.20, 0.45)):
            for pw in (25, 6):
                e, h_, x = run_pair(sep, pw, rng, hq, amp)
                cols.append((e, h_, x))
        L.append("    %4.2f (%4.0f)    | %2d/%2d  %2d               | %2d/%2d  %2d                | %2d/%2d  %2d     | %2d/%2d  %2d"
                 % (sep, sep * 500, cols[0][1], cols[0][0], cols[0][2], cols[1][1], cols[1][0], cols[1][2],
                    cols[2][1], cols[2][0], cols[2][2], cols[3][1], cols[3][0], cols[3][2]))
    return True


def a6(L):
    rng = np.random.default_rng(G.SEED)
    for tag in ("s1", "s2", "s3"):          # 같은 난수 순서를 맞추려고 앞 시나리오를 먼저 소비
        gen, sigma = G.SCEN[tag]; G.synthesize(gen(rng), sigma, rng)
    gen, sigma = G.SCEN["s4"]
    pulses = gen(rng); I, Q = G.synthesize(pulses, sigma, rng)
    mag, ph = C.cordic_vec_arr(I, Q)
    zero = np.zeros_like(mag)
    pdws = C.detect_channel(mag, ph, zero, zero, 0, freq_guard=0, ownership=False)[0]
    # 원신호(8 MSPS)에서 검출: 정답 펄스 구간과 겹치지 않는 PDW = 오검출
    spans = [(8 * p.toa, 8 * (p.toa + p.pw)) for p in pulses]
    false = [d for d in pdws if not any(s - 16 <= d['toa'] <= e + 16 for s, e in spans)]
    hit = sum(1 for s, e in spans if any(s - 16 <= d['toa'] <= e + 16 for d in pdws))
    L.append("A6 S4 wideband reference (same detector on raw 8 MSPS |x|): PDWs %d, pulses hit %d/%d, false PDWs %d"
             % (len(pdws), hit, len(pulses), len(false)))
    L.append("    (channelized S4: 31/31 hit, 0 false - see report_stage1.txt)")
    return True


def main():
    outdir = sys.argv[1] if len(sys.argv) > 1 else "sim_data"
    L = ["rtl-channelizer stage 1 analysis", ""]
    ok = True
    for f in (a1, a2, a3, a4, a5, a6):
        ok &= f(L); L.append("")
    L.append("ANALYSIS: %s" % ("PASS" if ok else "FAIL"))
    print("\n".join(L))
    with open(os.path.join(outdir, "report_analysis.txt"), "w") as fh:
        fh.write("\n".join(L) + "\n")


if __name__ == "__main__":
    main()
