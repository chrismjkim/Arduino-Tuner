import time
import numpy as np
import serial
import csv

PORT = "COM5"      # 너 환경에 맞게
BAUD = 500_000
SECONDS = 15

OUTPUT_PATH = "wave_sample_7.csv"

ser = serial.Serial(PORT, BAUD, timeout=1)
time.sleep(3.0)           # UNO 리셋 + 부트로더 + setup 완료 대기(보통 1~2초)
ser.reset_input_buffer()  # 부팅 중 쌓인 쓰레기/중간값 제거
print("start timing")
time.sleep(0.32)
t0 = time.perf_counter()  # 여기서부터 타이밍 시작
buf = bytearray()

while True:
    if time.perf_counter() - t0 >= SECONDS:
        break
    buf += ser.read(4096)

t1 = time.perf_counter()
ser.close()

# 2바이트 단위 정렬 (홀수 바이트면 마지막 1바이트 버림)
buf = buf[:len(buf) - (len(buf) % 2)]

samples = np.frombuffer(buf, dtype="<u2")  # little-endian uint16

elapsed = t1 - t0
elapsed_ms = elapsed * 1000.0
n = len(samples)
fs = (n / elapsed) if elapsed > 0 else 0.0

print(f"samples: {n}, elapsed: {elapsed:.4f}s, fs ~= {fs:.1f} Hz")

def format_hh_mm_ss_xxxx(t_ms: float) -> str:
    # HH:MM:SS:XXX (hours:minutes:seconds:milliseconds)
    if t_ms < 0:
        t_ms = 0.0
    total_ms = int(round(t_ms))
    ms = total_ms % 1000
    total_s = total_ms // 1000
    s = total_s % 60
    total_min = total_s // 60
    m = total_min % 60
    h = total_min // 60
    return f"{h:02d}:{m:02d}:{s:02d}.{ms:04d}"


with open(OUTPUT_PATH, "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["time", "value"])
    # 요청대로: time = elapsed_ms * (index / total_sample_count)
    if n != 0:
        for i, v in enumerate(samples):
            t_ms = elapsed_ms * (i / n)
            w.writerow([format_hh_mm_ss_xxxx(t_ms), int(v)])
