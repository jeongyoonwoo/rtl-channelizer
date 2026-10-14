#!/usr/bin/env python3
"""
chz_model.py  --  rtl-channelizer 참조 모델 (부동소수점 + 비트 단위)

구성
  N = 16 채널, 홉 M = 8 (2배 오버샘플링), 프로토타입 L = 128 탭 (갈래당 8탭)
  입력 Fs = 8 MSPS (논리 단위), 채널 간격 500 kHz, 채널 출력 1 MSPS

정의식 (채널 k, 홉 m. k = 8..15는 음의 주파수 k-16)
  y_k[m] = (-1)^(k*m) * sum_{n=0}^{127} h[n] * x[8m+7-n] * exp(+j*2*pi*k*n/16)
  -> 채널 중심 톤은 진폭 그대로, 채널 안 오프셋 d Hz의 톤은 홉마다 2*pi*d/1MHz씩 위상이 돈다.

다상 분해 (RTL이 따르는 구조)
  v_p[m] = sum_{t=0}^{7} h[p+16t] * x[8m+7-p-16t]            (갈래 p = 0..15)
  FFT 입력 순서 q = 0..15 에서 갈래 p = (8*(m&1) - q) mod 16
     -> 짝수 홉: v0, v15, v14, ..., v1   (순방향 FFT의 빈 k = 채널 k)
     -> 홀수 홉: 8칸 회전              ((-1)^(km) 부호 보정을 대신함)
  y[m] = FFT16(w)   (순방향, 단마다 1/2이라 전체 1/16, 계수 합이 16이므로 이득 1)

고정소수점 (비트 단위 모델 = RTL이 똑같이 해야 하는 연산)
  x   : 16비트 Q1.15
  h   : 18비트 Q2.16, 합 = 16 (갈래 합 ~1)
  FIR : acc = sum h*x (I, Q 따로)  -> v = sat18((acc + 2^15) >> 16)        Q3.15
        (갈래별 sum|h| 최대 2.023 > 2 이라 Q2.16 대신 Q3.15: FIR·FFT 내부는 어떤 입력에도 넘치지 않음)
  FFT : radix-2 DIF 4단, 단마다 a+b, a-b를 sat18((s + 1) >> 1)
        1단 뒤 W16^i, 2단 뒤 W16^(2i) 복소 곱: sat18((dr*wr - di*wi + 2^15) >> 16) 등
        3단 뒤 W16^4 = -j : (dr, di) -> (di, -dr) (부정은 포화)
        출력 위치 j = 빈 bitrev(j)   (스트리밍 FFT가 내보내는 순서)
  출력: sat16(X)  Q3.15 -> Q1.15 (|X| >= 1이면 포화)  -> CORDIC 입력
  '>>'는 산술 시프트(Verilog >>>와 같음), sat는 2의 보수 범위로 자르기.
"""
import math
import numpy as np

# ---------------------------------------------------------------------------
# 상수
# ---------------------------------------------------------------------------
N = 16                 # 채널 수
M = 8                  # 홉 (입력 샘플)
T = 8                  # 갈래당 탭
L = N * T              # 128
FS = 8.0e6             # 입력 샘플률 (논리 단위)
DF = FS / N            # 채널 간격 500 kHz
FCH = FS / M           # 채널 출력 샘플률 1 MSPS
BETA = 8.0             # Kaiser 창
FC_CH = 0.75           # 차단(-6dB) 주파수, 채널 간격 단위
COEF_FRAC = 16         # 계수 Q2.16
FIR_SHIFT = 16         # FIR 출력 Q3.15
K = 1.646760258121066  # CORDIC 이득

# 검출기 (rtl-pdw와 같음)
TH_HI = 3000; TH_LO = 2000; MIN_W = 4; END_LOW = 2
FREQ_SKIP = 3; FREQ_WIN = 8; MIN_FREQ_W = FREQ_SKIP + FREQ_WIN
OWN_LIM = 16384        # 보조 소유 조건: FREQ in [-16384, 16383] = [-250k, +250k)

LUT = [0x2000, 0x12E4, 0x09FB, 0x0511, 0x028B, 0x0146, 0x00A3, 0x0051,
       0x0029, 0x0014, 0x000A, 0x0005, 0x0003, 0x0001, 0x0001, 0x0000]

BITREV4 = [int('{:04b}'.format(i)[::-1], 2) for i in range(16)]   # 0,8,4,12,2,...

# ---------------------------------------------------------------------------
# 정수 도우미
# ---------------------------------------------------------------------------
def sat(v, bits):
    """2의 보수 bits 범위로 포화 (numpy 배열/정수 모두)."""
    lo, hi = -(1 << (bits - 1)), (1 << (bits - 1)) - 1
    return np.clip(v, lo, hi)


class OvfCounter:
    """포화가 일어난 횟수 (RTL sticky 플래그의 근거)."""
    def __init__(self):
        self.n = 0

    def sat(self, v, bits):
        lo, hi = -(1 << (bits - 1)), (1 << (bits - 1)) - 1
        self.n += int(np.count_nonzero((v < lo) | (v > hi)))
        return np.clip(v, lo, hi)


# ---------------------------------------------------------------------------
# 프로토타입 필터
# ---------------------------------------------------------------------------
def design_prototype(L_=L, beta=BETA, fc_ch=FC_CH, n_ch=N, total=N):
    """Kaiser 창 sinc 저역통과. fc_ch(채널 간격 단위)에서 -6dB. 계수 합 = total."""
    n = np.arange(L_) - (L_ - 1) / 2.0
    fc = fc_ch / n_ch                       # 입력 Fs로 정규화한 차단 주파수 (cycles/sample)
    h = 2 * fc * np.sinc(2 * fc * n) * np.kaiser(L_, beta)
    return h / h.sum() * total


def quantize_coef(h):
    hq = np.round(h * (1 << COEF_FRAC)).astype(np.int64)
    assert hq.max() < (1 << 17) and hq.min() >= -(1 << 17), "계수가 18비트를 넘음"
    return hq


def twiddles():
    """W16^i = exp(-j 2 pi i/16), Q2.16 18비트 (1.0 = 65536)."""
    i = np.arange(16)
    wr = np.round(np.cos(2 * np.pi * i / 16) * 65536).astype(np.int64)
    wi = np.round(-np.sin(2 * np.pi * i / 16) * 65536).astype(np.int64)
    return wr, wi


def freq_response(h, f_ch):
    """정규화(DC=1) 진폭 응답 [dB], f_ch는 채널 간격 단위."""
    f = np.atleast_1d(np.asarray(f_ch, dtype=float))
    w = 2 * np.pi * f / N
    n = np.arange(len(h))
    H = np.exp(-1j * np.outer(w, n)) @ h
    return 20 * np.log10(np.abs(H) / abs(h.sum()) + 1e-300)


# ---------------------------------------------------------------------------
# 부동소수점 모델
# ---------------------------------------------------------------------------
def _window_idx(n_hops):
    """[n_hops, L] 입력 인덱스 8m+7-n."""
    m = np.arange(n_hops)[:, None]
    n = np.arange(L)[None, :]
    return M * m + (M - 1) - n


def _take(x, idx):
    """x[idx], 범위 밖은 0."""
    ok = (idx >= 0) & (idx < len(x))
    out = np.zeros(idx.shape, dtype=x.dtype)
    out[ok] = x[idx[ok]]
    return out


def pfb_direct(x, h, n_hops):
    """정의식 그대로 (기준). 반환 [n_hops, 16] complex, 열 = 채널 k."""
    xw = _take(x, _window_idx(n_hops))                    # [H, L]
    E = np.exp(2j * np.pi * np.outer(np.arange(L), np.arange(N)) / N)   # [L, N]
    Y = xw @ (h[:, None] * E)
    sign = np.where((np.arange(n_hops)[:, None] * np.arange(N)[None, :]) % 2 == 1, -1.0, 1.0)
    return Y * sign / N       # 계수 합이 16이므로 1/16 -> 채널 중심 톤 이득 1


def fft_input_order(m):
    """홉 m에서 FFT 입력 위치 q(0..15)에 들어가는 갈래 번호 p."""
    return [((8 * (m & 1)) - q) % 16 for q in range(16)]


def pfb_polyphase_float(x, h, n_hops):
    """다상 FIR + 입력 순서 회전 + 순방향 FFT/16 (부동소수점)."""
    xw = _take(x, _window_idx(n_hops))                    # x[8m+7-n], n = p + 16t
    v = (xw * h[None, :]).reshape(n_hops, T, N).sum(axis=1)   # [H, p]
    out = np.zeros((n_hops, N), dtype=complex)
    for par in (0, 1):
        rows = np.arange(par, n_hops, 2)
        order = fft_input_order(par)
        w = v[rows][:, order]
        out[rows] = np.fft.fft(w, axis=1)
    return out / N            # 계수 합 16, FFT 1/16 -> 채널 중심 톤 이득 1


# ---------------------------------------------------------------------------
# 비트 단위 모델
# ---------------------------------------------------------------------------
def fir_bit(xi, xq, hq, n_hops, ovf=None):
    """다상 FIR. 반환 (vi, vq) [n_hops, 16] int64, 열 = FFT 입력 위치 q (RTL 출력 순서)."""
    ovf = ovf or OvfCounter()
    m = np.arange(n_hops)
    q = np.arange(16)
    p = ((8 * (m[:, None] & 1)) - q[None, :]) % 16                 # [H, 16]
    t = np.arange(T)
    idx = (M * m[:, None, None] + (M - 1) - p[:, :, None] - 16 * t[None, None, :])  # [H,16,T]
    coef = hq[p[:, :, None] + 16 * t[None, None, :]]                 # [H,16,T]
    ai = (coef * _take(xi, idx)).sum(axis=2)
    aq = (coef * _take(xq, idx)).sum(axis=2)
    vi = ovf.sat((ai + (1 << 15)) >> 16, 18)
    vq = ovf.sat((aq + (1 << 15)) >> 16, 18)
    return vi, vq


def fft16_bit(wr_in, wi_in, ovf=None):
    """16점 radix-2 DIF, 단마다 1/2. 입력 [H,16] (위치 q), 반환 자연 순서 빈 [H,16]."""
    ovf = ovf or OvfCounter()
    TWR, TWI = twiddles()
    ar = wr_in.astype(np.int64).copy()
    ai = wi_in.astype(np.int64).copy()
    for s in range(4):
        span = 8 >> s
        nr = ar.copy(); ni = ai.copy()
        for g in range(0, 16, 2 * span):
            for i in range(span):
                a, b = g + i, g + i + span
                sr = ovf.sat((ar[:, a] + ar[:, b] + 1) >> 1, 18)
                si = ovf.sat((ai[:, a] + ai[:, b] + 1) >> 1, 18)
                dr = ovf.sat((ar[:, a] - ar[:, b] + 1) >> 1, 18)
                di = ovf.sat((ai[:, a] - ai[:, b] + 1) >> 1, 18)
                e = i << s                                           # W16 지수
                if s in (0, 1):
                    wr, wi = int(TWR[e]), int(TWI[e])
                    pr = ovf.sat((dr * wr - di * wi + (1 << 15)) >> 16, 18)
                    pi = ovf.sat((dr * wi + di * wr + (1 << 15)) >> 16, 18)
                    dr, di = pr, pi
                elif s == 2 and i == 1:                              # W16^4 = -j
                    dr, di = di, ovf.sat(-dr, 18)
                nr[:, a], ni[:, a] = sr, si
                nr[:, b], ni[:, b] = dr, di
        ar, ai = nr, ni
    # 위치 j에 빈 bitrev(j)가 있음 -> 자연 순서로
    outr = np.zeros_like(ar); outi = np.zeros_like(ai)
    for j in range(16):
        outr[:, BITREV4[j]] = ar[:, j]
        outi[:, BITREV4[j]] = ai[:, j]
    return outr, outi


def channelizer_bit(xi, xq, hq, n_hops, ovf=None):
    """FIR -> FFT -> 16비트 변환. 반환 (yi, yq) [n_hops, 16] 자연 순서 채널 k, Q1.15.
    ovf는 FIR·FFT 내부 포화, out_sat은 마지막 16비트 포화(입력이 거의 최대 진폭일 때)."""
    ovf = ovf or OvfCounter()
    vi, vq = fir_bit(xi, xq, hq, n_hops, ovf)
    Xr, Xi = fft16_bit(vi, vq, ovf)
    ovf_out = OvfCounter(); ovf.out_sat = ovf_out
    yi = ovf_out.sat(Xr, 16)      # 출력 포화는 FIR·FFT 오버플로와 따로 센다
    yq = ovf_out.sat(Xi, 16)
    return yi, yq


# ---------------------------------------------------------------------------
# CORDIC (cordic_engine Vectoring과 비트 단위로 같음, 배열 처리)
# ---------------------------------------------------------------------------
def _w16(v):
    return ((v + 32768) & 0xFFFF) - 32768


def _w18(v):
    return ((v + 131072) & 0x3FFFF) - 131072


def cordic_vec_arr(xi, yi):
    xi = np.asarray(xi, dtype=np.int64); yi = np.asarray(yi, dtype=np.int64)
    neg = xi < 0
    x = np.where(neg, -xi, xi)
    y = np.where(neg, -yi, yi)
    z = np.where(neg, np.where(yi < 0, -32768, 32767), 0).astype(np.int64)
    for i in range(16):
        xs, ys = x >> i, y >> i
        down = y < 0
        x, y, z = (np.where(down, _w18(x - ys), _w18(x + ys)),
                   np.where(down, _w18(y + xs), _w18(y - xs)),
                   np.where(down, _w16(z - LUT[i]), _w16(z + LUT[i])))
    xo = np.clip(x, -32768, 32767)
    return xo, z


# ---------------------------------------------------------------------------
# 검출기 + 소유 판정 (채널 하나, pdw_detector.v / rtl-pdw run_model과 같은 알고리즘에 두 가지 추가)
#   1) FREQ_GUARD: 주파수·PA 창이 뒤쪽 가장자리 링잉(약 4샘플)에 닿지 않도록
#      주파수 유효 조건을 PW >= FREQ_SKIP + FREQ_WIN + FREQ_GUARD로 올림
#   2) 소유 판정 (중복·유령 제거)
#      주파수 유효 펄스: FREQ in [-16384, 16383] = [-250k, +250k)이면 소유
#        (정상 구간에서 잰 주파수는 이웃 채널에서도 정확하므로, 채널 안에서 판정이 끝남)
#      짧은 펄스: 진폭 우세. 펄스 안(TH_LO 이상, 상대 인덱스 0..DOM_MAX_R) 샘플마다
#        dl += mag[k] - mag[k-1],  dr += mag[k] - mag[k+1]  (같은 홉의 이웃 채널, 16채널 순환)
#        dl > 0 이고 dr >= 0 이면 소유 (경계 신호가 한쪽만 소유하도록 > 와 >= 비대칭)
#        짧은 펄스는 가장자리 위상이 경계 쪽으로 끌려 주파수로는 판정할 수 없음
# ---------------------------------------------------------------------------
FREQ_GUARD = 4
DOM_MAX_R = 15         # 짧은 펄스(측정 PW < 15)만 쓰므로 16샘플이면 충분


def detect_channel(mag, phase, mag_l, mag_r, ch, freq_guard=FREQ_GUARD, ownership=True, raw=None):
    """mag, phase: 채널 k의 샘플열, mag_l/mag_r: 같은 홉의 채널 k-1, k+1 크기.
    반환 (보고할 PDW 목록, 비소유 수, 글리치 수). PDW dict: ch, toa, pw, pa, freq, flags, end.
    raw에 리스트를 주면 소유 판정 전 사건(MIN_W 이상 모든 펄스)을 own, dl, dr과 함께 덧붙인다."""
    min_freq_w = FREQ_SKIP + FREQ_WIN + freq_guard
    out = []; not_owned = 0; glitch = 0
    state = 0
    toa = last_hi = peak = low = fsum = msum = dl = dr = 0
    prev_phase = 0
    for n in range(len(mag)):
        m = int(mag[n]); ph = int(phase[n])
        dphi = _w16(ph - prev_phase)
        is_hi = m >= TH_LO
        if state == 0:
            if m >= TH_HI:
                state, toa, last_hi, peak, low, fsum, msum = 1, n, n, m, 0, 0, 0
                dl = m - int(mag_l[n]); dr = m - int(mag_r[n])
        else:
            r = n - toa
            if is_hi:
                last_hi, low = n, 0
                peak = max(peak, m)
                if r <= DOM_MAX_R:
                    dl += m - int(mag_l[n]); dr += m - int(mag_r[n])
            else:
                low += 1
            if FREQ_SKIP <= r < FREQ_SKIP + FREQ_WIN:
                fsum += dphi; msum += m
            if low >= END_LOW:
                pw = last_hi - toa + 1
                if pw >= MIN_W:
                    fvalid = 1 if pw >= min_freq_w else 0
                    freq = _w16(fsum >> 3) if fvalid else 0
                    if not ownership:
                        own = True
                    elif fvalid:
                        own = -OWN_LIM <= freq < OWN_LIM
                    else:
                        own = (dl > 0) and (dr >= 0)
                    flags = fvalid | ((1 if peak >= 32767 else 0) << 1)
                    ev = dict(ch=ch, toa=toa, pw=pw, pa=(msum >> 3) if fvalid else peak,
                              freq=freq, flags=flags, end=n)
                    if raw is not None:
                        raw.append(dict(ev, own=1 if own else 0, dl=dl, dr=dr))
                    if own:
                        out.append(ev)
                    else:
                        not_owned += 1
                else:
                    glitch += 1
                state = 0
        prev_phase = ph
    return out, not_owned, glitch


def detect_all(yi, yq, **kw):
    """채널 출력 [H,16] -> CORDIC -> 채널별 검출(이웃 크기 비교 포함).
    PDW는 RTL 출력 순서로 정렬: (펄스가 끝난 홉, 채널 번호 k 오름차순).
    RTL은 홉 하나의 16채널 크기를 모두 모은 뒤 k = 0..15 순서로 검출기를 돌린다."""
    mag, ph = cordic_vec_arr(yi, yq)
    pdws = []; nown = 0; ngl = 0
    for k in range(16):
        o, a, b = detect_channel(mag[:, k], ph[:, k], mag[:, (k - 1) % 16], mag[:, (k + 1) % 16], k, **kw)
        pdws += o; nown += a; ngl += b
    pdws.sort(key=lambda d: (d['end'], d['ch']))
    return pdws, nown, ngl, mag, ph


def signed_ch(k):
    return k if k < 8 else k - 16


# ---------------------------------------------------------------------------
# 프레임 (22바이트)
# ---------------------------------------------------------------------------
def crc16_ccitt(data, crc=0xFFFF):
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def make_frame(cnt, d):
    body = [(cnt >> 8) & 0xFF, cnt & 0xFF, 1, signed_ch(d['ch']) & 0xFF]
    body += list((d['toa'] & 0xFFFFFFFF).to_bytes(4, 'big'))
    for f in ('pw', 'pa', 'freq', 'flags'):
        body += list((d[f] & 0xFFFF).to_bytes(2, 'big'))
    c = crc16_ccitt(body)
    return [0x1A, 0xCF, 0xFC, 0x1D] + body + [c >> 8, c & 0xFF]


def stream_signature(frames):
    allb = [b for f in frames for b in f]
    return crc16_ccitt(allb)
