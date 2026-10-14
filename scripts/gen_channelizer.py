#!/usr/bin/env python3
"""
gen_channelizer.py  --  rtl-channelizer 합성 신호, 정답표, 비트 단위 모델 출력 생성

시나리오 (각 65,536 입력 샘플 = 8.192 ms, 채널당 8,192 샘플)
  S1 동시 방출원   방출원 4개(서로 1.6 MHz 이상), 모두 200 us에 시작해 같은 시각 펄스가 자주 겹침, 진폭 합 0.95
  S2 소유·경계     채널 내 오프셋 0~±240k / ±260k, ±4 MHz 채널, 짧은 펄스(진폭 우세 소유), 2채널 간격 동시 신호
  S3 경계 조건     rtl-pdw S3 계승: 최소 폭, 문턱 근처, 포화, 1 us 드롭, 같은 채널 연속 펄스
  S4 잡음 이득     입력 잡음 sigma 0.016(rtl-pdw의 4배), 진폭 0.08 펄스
  S5 동시 종료     8채널(2채널 간격)에 같은 시각·같은 PW 펄스를 100 us마다 -> 한 홉에 PDW 8개가 몰려 FIFO에 쌓임
                   (6단계에서 추가: S1은 PW가 달라 끝나는 홉이 달라서 FIFO 점유가 1을 넘지 않았음)
  T1 톤 스윕       채널 응답 측정용 (24,576 샘플, 톤 96개 x 256 샘플)

출력 (sim_data/)
  coef_h.hex            프로토타입 계수 128개, 18비트 Q2.16 (5자리 hex)
  iq_rom_<s>.hex        입력 ROM {I[15:0], Q[15:0]}
  ch_ref_<s>.hex        채널 출력, 홉마다 FFT 출력 순서(j = 0..15, 채널 bitrev(j))로 16줄 {I, Q}
  expected_pdw_<s>.hex  정답표  CH(2) TOA(8) PW(4) PA(4) FREQ(4) FLAGS(4)
                        CH는 부호 있는 바이트(-8~7, 프레임과 같음). 채널 -8은 +-4 MHz 경계 채널이며
                        절대 주파수 = CH*500k + FREQ*500k/32768 로 -4.25~-3.75 MHz에 표기(+3.9 MHz -> -4.1 MHz)
  model_pdw_<s>.hex     비트 단위 모델 PDW (RTL 출력 순서)
  model_frame_<s>.hex   기대 프레임 22바이트/줄
  report_stage1.txt     1단계 판정 결과

정답표 FLAGS: bit0 주파수 유효, bit1 포화, bit14 '포화 판정 생략', bit15 '주파수 유효 판정 생략'
  (필터가 PW를 -1~+2 바꾸므로 PW 12~14는 주파수 유효 여부가 진폭에 따라 갈림)
정답 TOA = 펄스 시작(us) + 7 (필터 군지연: 홉 m의 기준 시점 8m+7에서 (L-1)/2 = 63.5 입력 샘플 = 7.06 홉.
  문턱이 진폭의 절반보다 낮으면 그보다 조금 일찍 넘으므로 7)

사용법: python3 scripts/gen_channelizer.py sim_data
"""
import math, os, sys
from dataclasses import dataclass
from typing import Optional
import numpy as np
import chz_model as C

N_IN = 65536
N_HOP = N_IN // C.M            # 8192
SIGMA = 0.004
SEED = 20261013
TOA_DELAY = 7


@dataclass
class Pulse:
    pid: int; emitter: str; toa: int; pw: int; amp: float; f_hz: float; phase: float
    expect: str = "detect"; dip_at: Optional[int] = None; note: str = ""


# ---------------------------------------------------------------------------
# 시나리오
# ---------------------------------------------------------------------------
def scenario_s1(rng):
    # 진폭 합 0.95: 넷이 동시에 와도 입력이 +-1을 넘지 않음 (넘으면 포화 상호변조가 전 채널로 퍼짐)
    em = [("A", -2.94e6, 12, 0.30, 100), ("B", -1.12e6, 25, 0.20, 150),
          ("C", +0.68e6, 16, 0.20, 200), ("D", +2.30e6, 40, 0.25, 250)]
    pulses = []
    for name, f, pw, amp, pri in em:
        toa = 200
        while toa + pw < 8100:
            pulses.append(Pulse(0, name, toa, pw, amp, f, rng.uniform(-math.pi, math.pi)))
            toa += pri
    pulses.sort(key=lambda p: (p.toa, p.f_hz))
    for i, p in enumerate(pulses):
        p.pid = i
    return pulses


def scenario_s2(rng):
    slots = []
    c3 = 3 * C.DF
    for off in (0, 100e3, -100e3, 200e3, -200e3, 240e3, -240e3, 260e3, -260e3):
        slots.append([("off", c3 + off, 20, 0.40)])
    c5 = -5 * C.DF
    for off in (0, 240e3, -240e3, 260e3, -260e3):
        slots.append([("off-neg", c5 + off, 20, 0.40)])
    for f in (3.90e6, -3.95e6, 3.76e6, -3.76e6):
        slots.append([("nyq", f, 20, 0.40)])
    cm2 = -2 * C.DF
    for pw in (4, 6, 8, 10):
        for off in (0, 150e3, -230e3, 270e3):
            slots.append([("short", cm2 + off, pw, 0.40)])
    # 동시 신호 쌍: 2채널(1 MHz) 간격 = 분리 보장 간격. 짧은 펄스 쌍 포함
    pairs = [((-1.40e6, 20, 0.40), (-0.40e6, 30, 0.30)),
             ((0.775e6, 25, 0.35), (1.775e6, 20, 0.45)),
             ((2.725e6, 25, 0.35), (1.725e6, 20, 0.45)),
             ((-3.10e6, 6, 0.30), (-2.10e6, 8, 0.45))]
    for a, b in pairs:
        slots.append([("pair",) + a, ("pair",) + b])
    pulses, toa = [], 200
    for s in slots:
        for tag, f, pw, amp in s:
            pulses.append(Pulse(len(pulses), tag, toa, pw, amp, f, rng.uniform(-math.pi, math.pi)))
        toa += 120
    return pulses


def scenario_s3(rng):
    c2 = 2 * C.DF
    spec = [(4, 0.30, 150e3, "detect", None, "min width"),
            (2, 0.30, 150e3, "reject", None, "glitch (filtered PW 3)"),
            (20, 0.07, -120e3, "detect", None, "near threshold"),
            (15, 0.65, 50e3, "detect", None, "saturation"),
            (20, 0.35, 80e3, "detect", 14, "1 us drop"),
            (10, 0.35, -200e3, "detect", None, "PW 10"),
            (11, 0.35, 200e3, "detect", None, "PW 11"),
            (16, 0.35, 240e3, "detect", None, "near ownership edge")]
    pulses, toa = [], 300
    for pw, amp, off, ex, dip, note in spec:
        pulses.append(Pulse(len(pulses), "X", toa, pw, amp, c2 + off, rng.uniform(-math.pi, math.pi), ex, dip, note))
        toa += 400
    pulses.append(Pulse(len(pulses), "X", toa, 12, 0.40, c2 + 100e3, rng.uniform(-math.pi, math.pi), note="back-to-back 1"))
    pulses.append(Pulse(len(pulses), "X", toa + 12 + 5, 12, 0.40, c2 - 100e3, rng.uniform(-math.pi, math.pi), note="back-to-back 2"))
    return pulses


def scenario_s4(rng):
    chans = (-7, -4, -1, 2, 5)
    offs = (0.0, 120e3, -150e3, 60e3, -90e3, 150e3)
    pulses, toa, i = [], 300, 0
    while toa + 20 < 8000:
        f = chans[i % 5] * C.DF + offs[i % 6]
        pulses.append(Pulse(i, "W", toa, 20, 0.08, f, rng.uniform(-math.pi, math.pi)))
        toa += 250; i += 1
    return pulses


def scenario_s5(rng):
    # 채널 -8, -6, ..., +6, 진폭 합 0.88. 채널 안 오프셋은 버스트마다 바꾸되 한 버스트 안에서는 같게 둔다
    # -> 이웃 신호의 실제 주파수 간격이 정확히 2채널(1 MHz) = 분리 보장 간격.
    #    (처음에 펄스마다 오프셋을 따로 줬더니 간격이 1.36채널까지 좁아져 가운데 채널(ch 7)에 유령 PDW 9개가 생겼음)
    pulses, toa, i = [], 200, 0
    while toa + 20 < 8000:
        off = ((i * 7) % 9 - 4) * 40e3                       # -160k ~ +160k, 버스트 공통
        for c in range(-8, 8, 2):
            pulses.append(Pulse(len(pulses), "B%d" % c, toa, 20, 0.11, c * C.DF + off, rng.uniform(-math.pi, math.pi)))
        toa += 100; i += 1
    return pulses


SCEN = {"s1": (scenario_s1, SIGMA), "s2": (scenario_s2, SIGMA),
        "s3": (scenario_s3, SIGMA), "s4": (scenario_s4, 0.016), "s5": (scenario_s5, SIGMA)}


# ---------------------------------------------------------------------------
# 합성
# ---------------------------------------------------------------------------
def synthesize(pulses, sigma, rng, n_in=N_IN):
    s = np.zeros(n_in, dtype=np.complex128)
    for p in pulses:
        n = np.arange(C.M * p.toa, C.M * (p.toa + p.pw))
        a = np.full(len(n), p.amp)
        if p.dip_at is not None:
            k = (n - C.M * p.toa) // C.M
            a[k == p.dip_at] = 0.01
        s[n] += a * np.exp(1j * (2 * math.pi * p.f_hz * (n - C.M * p.toa) / C.FS + p.phase))
    s += sigma * (rng.standard_normal(n_in) + 1j * rng.standard_normal(n_in))
    I = np.clip(np.round(s.real * 32768.0), -32768, 32767).astype(np.int64)
    Q = np.clip(np.round(s.imag * 32768.0), -32768, 32767).astype(np.int64)
    return I, Q


def tone_sweep():
    """T1: 채널 2를 가운데 두고 -1.5 ~ +1.47 채널, 1/32 채널 간격, 톤당 256 샘플, 진폭 0.5, 잡음 없음."""
    steps, seg = 96, 256
    I = np.zeros(steps * seg, np.int64); Q = np.zeros_like(I); freqs = []
    for s in range(steps):
        f = (2 + (s - 48) / 32.0) * C.DF
        n = np.arange(seg)
        x = 0.5 * np.exp(2j * math.pi * f * n / C.FS)
        I[s * seg:(s + 1) * seg] = np.round(x.real * 32768)
        Q[s * seg:(s + 1) * seg] = np.round(x.imag * 32768)
        freqs.append(f)
    return I, Q, freqs, seg


# ---------------------------------------------------------------------------
# 정답표
# ---------------------------------------------------------------------------
def owner(f):
    kk = math.floor(f / C.DF + 0.5)
    off = f - kk * C.DF
    return kk % 16, off


def truth(p):
    ch, off = owner(p.f_hz)
    pa = min(int(round(p.amp * C.K * 32768.0)), 32767)
    g = p.amp * C.K
    sat = 1 if g >= 1.0 else 0
    sat_dc = 1 if 0.88 <= g < 1.0 else 0
    fr = int(round(off / C.DF * 32768.0))
    if p.pw >= 15:   fv, fv_dc = 1, 0     # 측정 PW >= 15 (FREQ_SKIP 3 + 창 8 + 보호 4)
    elif p.pw <= 11: fv, fv_dc = 0, 0
    else:            fv, fv_dc = 0, 1
    flags = fv | (sat << 1) | (sat_dc << 14) | (fv_dc << 15)
    return dict(ch=ch, toa=p.toa + TOA_DELAY, pw=p.pw, pa=pa, freq=fr if (fv or fv_dc) else 0,
                flags=flags, f_hz=p.f_hz, amp=p.amp, pid=p.pid)


def pdw_hex(d):
    return "%02X%08X%04X%04X%04X%04X" % (C.signed_ch(d['ch']) & 0xFF, d['toa'] & 0xFFFFFFFF, d['pw'] & 0xFFFF,
                                         d['pa'] & 0xFFFF, d['freq'] & 0xFFFF, d['flags'] & 0xFFFF)


def abs_freq(d):
    return C.signed_ch(d['ch']) * C.DF + C._w16(d['freq']) * C.DF / 32768.0


def wrap_hz(df):
    return (df + C.FS / 2) % C.FS - C.FS / 2


# ---------------------------------------------------------------------------
# 판정 (테스트벤치가 할 비교와 같은 규칙)
# ---------------------------------------------------------------------------
TOL = {"default": dict(toa=(-1, 2), pw=(-2, 3), fr=3e3, pa_v=(0.03, 230), pa_p=(0.08, 650)),
       "s4": dict(toa=(-1, 3), pw=(-3, 3), fr=6e3, pa_v=(0.03, 400), pa_p=(0.08, 650))}


def evaluate(tag, exp, pdws):
    tol = TOL.get(tag, TOL["default"])
    used = [False] * len(pdws)
    res = dict(n_exp=len(exp), match=0, miss=[], bad=[], toa=[], pw=[], fr=[], pa=[])
    for e in exp:
        hit = None
        for i, d in enumerate(pdws):
            if not used[i] and d['ch'] == e['ch'] and -2 <= d['toa'] - e['toa'] <= 4:
                hit = i; break
        if hit is None:
            res['miss'].append(e); continue
        used[hit] = True; d = pdws[hit]; res['match'] += 1
        errs = []
        dt = d['toa'] - e['toa']; dw = d['pw'] - e['pw']
        res['toa'].append(dt); res['pw'].append(dw)
        if not (tol['toa'][0] <= dt <= tol['toa'][1]): errs.append("TOA %+d" % dt)
        if not (tol['pw'][0] <= dw <= tol['pw'][1]): errs.append("PW %+d" % dw)
        fv_m = d['flags'] & 1; fv_e = e['flags'] & 1; fv_dc = (e['flags'] >> 15) & 1
        if not fv_dc and fv_m != fv_e: errs.append("fvalid %d/%d" % (fv_m, fv_e))
        sat_m = (d['flags'] >> 1) & 1; sat_e = (e['flags'] >> 1) & 1; sat_dc = (e['flags'] >> 14) & 1
        if not sat_dc and sat_m != sat_e: errs.append("sat %d/%d" % (sat_m, sat_e))
        if fv_m:
            dfz = abs(wrap_hz(abs_freq(d) - e['f_hz']))
            res['fr'].append(dfz)
            if dfz > tol['fr']: errs.append("FREQ %.2fkHz" % (dfz / 1e3))
        if not sat_m and not sat_e:
            dp = abs(d['pa'] - e['pa'])
            r, a = tol['pa_v'] if fv_m else tol['pa_p']
            res['pa'].append((dp, fv_m))
            if dp > r * e['pa'] + a: errs.append("PA %d" % dp)
        if errs: res['bad'].append((e, d, errs))
    res['extra'] = [pdws[i] for i in range(len(pdws)) if not used[i]]
    return res


def fmt_res(tag, res, pdws, nown, ngl, ovf):
    L = []
    ok = (not res['miss'] and not res['bad'] and not res['extra'] and ovf == 0)
    L.append("%s: %s  match %d/%d  miss %d  extra(false/dup) %d  field-fail %d  | not_owned %d glitch %d internal_ovf %d"
             % (tag.upper(), "PASS" if ok else "FAIL", res['match'], res['n_exp'], len(res['miss']),
                len(res['extra']), len(res['bad']), nown, ngl, ovf))
    if res['toa']:
        L.append("    TOA err %+d..%+d mean %+.2f | PW err %+d..%+d mean %+.2f"
                 % (min(res['toa']), max(res['toa']), np.mean(res['toa']),
                    min(res['pw']), max(res['pw']), np.mean(res['pw'])))
    if res['fr']:
        L.append("    FREQ |err| max %.3f mean %.3f kHz (n=%d)" % (max(res['fr']) / 1e3, np.mean(res['fr']) / 1e3, len(res['fr'])))
    pv = [a for a, v in res['pa'] if v]; pp = [a for a, v in res['pa'] if not v]
    if pv: L.append("    PA(window) |err| max %d mean %.1f LSB (n=%d)" % (max(pv), np.mean(pv), len(pv)))
    if pp: L.append("    PA(peak)   |err| max %d mean %.1f LSB (n=%d)" % (max(pp), np.mean(pp), len(pp)))
    for e in res['miss'][:10]:
        L.append("    MISS pid %d ch %d toa %d pw %d f %.0fk amp %.2f" % (e['pid'], C.signed_ch(e['ch']), e['toa'], e['pw'], e['f_hz'] / 1e3, e['amp']))
    for e, d, errs in res['bad'][:10]:
        L.append("    FIELD pid %d ch %d: %s" % (e['pid'], C.signed_ch(e['ch']), ", ".join(errs)))
    for d in res['extra'][:10]:
        L.append("    EXTRA ch %d toa %d pw %d freq %d flags %X (abs %.0fk)" % (C.signed_ch(d['ch']), d['toa'], d['pw'], C._w16(d['freq']), d['flags'], abs_freq(d) / 1e3))
    return ok, L


# ---------------------------------------------------------------------------
# 파일 쓰기
# ---------------------------------------------------------------------------
def write_rom(path, I, Q):
    with open(path, "w") as f:
        for i, q in zip(I, Q):
            f.write("%04X%04X\n" % (int(i) & 0xFFFF, int(q) & 0xFFFF))


def write_ch_ref(path, yi, yq):
    with open(path, "w") as f:
        for m in range(yi.shape[0]):
            for j in range(16):
                k = C.BITREV4[j]
                f.write("%04X%04X\n" % (int(yi[m, k]) & 0xFFFF, int(yq[m, k]) & 0xFFFF))


def main():
    outdir = sys.argv[1] if len(sys.argv) > 1 else "sim_data"
    os.makedirs(outdir, exist_ok=True)
    rng = np.random.default_rng(SEED)
    h = C.design_prototype(); hq = C.quantize_coef(h)
    with open(os.path.join(outdir, "coef_h.hex"), "w") as f:
        for v in hq:
            f.write("%05X\n" % (int(v) & 0x3FFFF))

    report = ["rtl-channelizer stage 1 report (bit-accurate model vs truth)", ""]
    all_ok = True
    for tag in ("s1", "s2", "s3", "s4", "s5"):
        gen, sigma = SCEN[tag]
        pulses = gen(rng)
        I, Q = synthesize(pulses, sigma, rng)
        ovf = C.OvfCounter()
        yi, yq = C.channelizer_bit(I, Q, hq, N_HOP, ovf)
        pdws, nown, ngl, _, _ = C.detect_all(yi, yq)
        exp = [truth(p) for p in pulses if p.expect == "detect"]
        exp.sort(key=lambda e: (e['toa'], e['ch']))
        res = evaluate(tag, exp, pdws)
        ok, lines = fmt_res(tag, res, pdws, nown, ngl, ovf.n)
        all_ok &= ok
        lines.insert(1, "    pulses %d (expected detect %d), PDWs %d, output saturations %d" % (len(pulses), len(exp), len(pdws), ovf.out_sat.n))
        report += lines
        write_rom(os.path.join(outdir, "iq_rom_%s.hex" % tag), I, Q)
        write_ch_ref(os.path.join(outdir, "ch_ref_%s.hex" % tag), yi, yq)
        with open(os.path.join(outdir, "expected_pdw_%s.hex" % tag), "w") as f:
            for e in exp: f.write(pdw_hex(e) + "\n")
        with open(os.path.join(outdir, "model_pdw_%s.hex" % tag), "w") as f:
            for d in pdws: f.write(pdw_hex(d) + "\n")
        frames = [C.make_frame(i, d) for i, d in enumerate(pdws)]
        with open(os.path.join(outdir, "model_frame_%s.hex" % tag), "w") as f:
            for fr in frames: f.write("".join("%02X" % b for b in fr) + "\n")
        report.append("    frames %d, stream signature 0x%04X" % (len(frames), C.stream_signature(frames)))
        print("\n".join(lines))

    # T1 톤 스윕
    I, Q, freqs, seg = tone_sweep()
    n_hop = len(I) // C.M
    ovf = C.OvfCounter()
    yi, yq = C.channelizer_bit(I, Q, hq, n_hop, ovf)
    write_rom(os.path.join(outdir, "iq_rom_t1.hex"), I, Q)
    write_ch_ref(os.path.join(outdir, "ch_ref_t1.hex"), yi, yq)
    hps = seg // C.M
    pts = []
    for s, f in enumerate(freqs):
        rows = slice(s * hps + 18, (s + 1) * hps)
        for k in range(5):
            p = np.mean(yi[rows, k].astype(float) ** 2 + yq[rows, k].astype(float) ** 2)
            pts.append(((f - k * C.DF) / C.DF, 10 * np.log10(p / (0.5 * 32768) ** 2 + 1e-30)))
    pts = np.array(pts)
    pb = pts[np.abs(pts[:, 0]) <= 0.5, 1]
    adj = pts[np.abs(np.abs(pts[:, 0]) - 1.0) < 1e-9, 1]
    far = pts[np.abs(pts[:, 0]) >= 1.5, 1]
    t1_ok = (pb.max() - pb.min() <= 0.2) and adj.max() <= -40 and far.max() <= -80 and ovf.n == 0
    all_ok &= t1_ok
    report += ["", "T1 tone sweep (bit model): %s" % ("PASS" if t1_ok else "FAIL"),
               "    passband (|off| <= 0.5 ch) ripple %.3f dB (%.3f .. %.3f)" % (pb.max() - pb.min(), pb.min(), pb.max()),
               "    1.0 ch away: max %.1f dB   >= 1.5 ch away: max %.1f dB   internal_ovf %d" % (adj.max(), far.max(), ovf.n)]
    np.savetxt(os.path.join(outdir, "t1_response.csv"), pts, delimiter=",", header="offset_ch,dB", fmt="%.5f")
    report += ["", "ALL: %s" % ("PASS" if all_ok else "FAIL")]
    with open(os.path.join(outdir, "report_stage1.txt"), "w") as f:
        f.write("\n".join(report) + "\n")
    print("\n".join(report[-6:]))


if __name__ == "__main__":
    main()
