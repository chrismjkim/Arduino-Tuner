import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import rcParams
from csver import txt_to_csv

# ===== 공통 설정 =====
# 샘플 번호: 창함수/2차보간 기준 XX, OX, XO, OO

INPUT_PATH = "wave_sample_0.csv"      # txt, csv로 입력 모두 받을 수 있음
WINDOW_SEC = 0.1                                # 구간 길이(초)
APPLY_WINFUNC = True                            # 창함수 적용
APPLY_PARABOLIC_INTERPOLATION = True            # 2차보간 적용
FS_CAL = 1.0001                                 # 샘플링레이트 보정계수

# txt → csv 이름 자동 생성
base, _ = os.path.splitext(INPUT_PATH)
OUTPUT_PATH = base + ".csv"

rcParams['font.family'] = 'Malgun Gothic'       # 플롯될 글꼴
rcParams['axes.unicode_minus'] = False          # 마이너스 부호 깨짐 방지

def plot_soundwave():
    '''소리의 파형을 출력'''

def plot_dom_freq_over_time():
    '''각 시간대별 가장 우세한 주파수 그래프'''
    
    plt.figure(figsize=(12, 5))
    plt.plot(times_mid, peak_freqs, marker="o")
    plt.title(f"{WINDOW_SEC}초 구간별 지배 주파수 변화")
    plt.xlabel("Time [s]")
    plt.ylabel("Dominant Frequency [Hz]")
    plt.grid(True)
    annot_cnt = 0
    for x, y in zip(times_mid, peak_freqs):
        if (annot_cnt%4!=0):
            annot_cnt += 1
            continue
        plt.annotate(f"{y:.1f}", (x, y),
                    textcoords="offset points", xytext=(0, 6),
                    ha="center", va="bottom", fontsize=8)
        annot_cnt += 1
    plt.tight_layout()
    plt.show()

def plot_detected_freqs():
    ''' 탐지된 주파수별 빈도 출력'''
    plt.figure(figsize=(10, 5))
    plt.plot(freqs, magnitude)
    plt.xlabel("Frequency (Hz)")
    plt.ylabel("Magnitude")
    plt.title("FFT Spectrum")
    plt.grid(True)
    
    # 관심 대역만 보기:
    # plt.xlim(0, 5000)

    plt.show()

def parabolic_interpolation(mag: np.ndarray, k: int) -> float:
    """
    mag[k] 주변( k-1, k, k+1 )으로 포물선 보간해서 sub-bin peak 위치를 반환.
    반환값은 'bin index' 단위(예: k + delta). 경계면이면 그냥 k.
    """
    if k <= 0 or k >= len(mag) - 1:
        return float(k)
    a = mag[k - 1]
    b = mag[k]
    c = mag[k + 1]
    denom = (a - 2*b + c)
    if denom == 0:
        return float(k)
    delta = 0.5 * (a - c) / denom
    return float(k) + delta

if __name__=="__main__":
    # ===== txt 파일 csv로 변환 및 생성 =====
    if (not INPUT_PATH.endswith(".csv")):
        df = txt_to_csv(INPUT_PATH, OUTPUT_PATH)
    else:
        df = pd.read_csv(INPUT_PATH)
        
    # ===== DURATION_SEC(음성 입력의 총 시간) 계산 =====
    # time 형식: "HH:MM:SS.mmm"
    times = pd.to_datetime(df["time"], format="%H:%M:%S.%f", errors="coerce").dropna()
    DURATION_SEC = (times.max() - times.min()).total_seconds()
    print(f"[fft] 자동 계산된 DURATION_SEC = {DURATION_SEC:.3f} s")

    # DataFrame 전처리
    df["value"] = pd.to_numeric(df["value"], errors="coerce") # 숫자 열로 변환
    df = df.dropna(subset=["value"]).copy()

    samples = df["value"].to_numpy()
    FFT_N = len(samples)
    print(f"[fft] 유효 샘플 개수 N = {FFT_N}")

    # ===== fs(frequency of samping, 샘플링 주파수) 추정 =====
    FS = FFT_N / DURATION_SEC        # 샘플링 주파수 [Hz]
    FS_EFF = FS * FS_CAL             # 샘플링 주파수 보정 [Hz]
    dt = 1.0 / FS_EFF                # 샘플링 주기   [s]
    print(f"[fft] 추정 샘플링 주파수 fs ≈ {FS:.2f} Hz")
    print(f"[fft] 보정된 추정 샘플링 주파수 fs ≈ {FS_EFF:.2f} Hz")

    # ===== 전체 시간을 window 단위로 쪼개기 =====
    num_windows = int(DURATION_SEC / WINDOW_SEC)  # 예: 10초 / 0.1초

    times_mid = []     # 각 구간의 중앙 시각 (x축)
    peak_freqs = []    # 각 구간의 지배적 주파수 (y축)

    for k in range(num_windows):
    # 이 구간에 해당하는 샘플 인덱스 범위 계산 
        start = int(k * WINDOW_SEC * FS_EFF) 
        end = int((k + 1) * WINDOW_SEC * FS_EFF) 
        
        if start >= FFT_N: 
            break 
        if end > FFT_N: 
            end = FFT_N 
            
        segment = samples[start:end] 
        # DC 오프셋 제거 (raw 값 -> 편차 값) 
        x = segment - np.mean(segment)
        # 창함수(hanning 적용)
        if (APPLY_WINFUNC):
            window = np.hanning(len(x))
            xw = x * window
        else:
            xw = x
            
        # FFT
        X = np.fft.rfft(xw)
        freqs = np.fft.rfftfreq(len(xw), d=1.0/FS_EFF)
        magnitude = np.abs(X)

        if len(magnitude) > 0:
            magnitude[0] = 0
            
        # 피크 찾기
        peak_bin_idx = int(np.argmax(magnitude))

        # ===== parabolic interpolation 적용 =====
        if (APPLY_PARABOLIC_INTERPOLATION):
            peak_bin = parabolic_interpolation(magnitude, peak_bin_idx)   # k + delta (sub-bin)
        else:
            peak_bin = peak_bin_idx
        f_peak = peak_bin * FS_EFF / len(xw)                            # Hz로 변환
        center_time = (k + 0.5) * WINDOW_SEC
        times_mid.append(center_time)
        peak_freqs.append(f_peak)

    plot_dom_freq_over_time()