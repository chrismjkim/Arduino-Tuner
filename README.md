# Arduino-Tuner
Arduino UNO와 GY-MAX4466 마이크 증폭 모듈, I2C 백팩 lcd 디스플레이로 만든 튜너기 프로젝트입니다.
마이크 증폭 모듈이 샘플링한 음압 값을 프레임 단위로 FFT를 적용하여 가장 우세한 주파수(Hz) 값을 도출합니다.

## serial_monitor.py
기존 Arduino의 serial monitor는 대량의 serial 출력을 저장 후 파일로 저장할 수가 없습니다. serial_monitor.py를 이용해 PC의 port에서 나오는 시리얼 출력을 인식해서, 데이터를 numpy의 NDArray 형태로 저장합니다.
이후 지정한 시간(SECONDS)만큼 분석이 끝나면, 각 시리얼 출력이 찍힌 시간(time)을 hh__mm__ss_xxxx 꼴로 변환하고, value 열에 출력값을 넣고 csv로 변환해 저장합니다.

## fft.py
serial_monitor를 통해 저장한 csv를 열어 time과 value 열 정보를 이용해 fft를 수행합니다. tuner_fft.ino와 똑같이 작동하도록 구현되었습니다. (단, 데이터셋의 품질 등으로 인해 결과가 상이할 수 있습니다)
fft를 통해 시간대별 가장 우세한 주파수 값을 기록하고, 이를 꺾은선그래프로 시각화하여 결과를 보여줍니다.

<img width="1200" height="500" alt="Image" src="https://github.com/user-attachments/assets/8fd5b83f-bc6d-44a8-aaf6-e951e67ad12a" />
실행 예시입니다. 가독성을 위해 4번째 frame마다 주파수 값을 출력합니다.

## wave_plots
wave_plots.zip 안에는 총 36개의 플롯된 그래프가 sample숫자1_숫자2.png로 저장되어 있습니다.

### 숫자1
음원 샘플의 종류입니다. 모두 단순 사인파를 재생한 음원입니다.
- 0: 배경소음(아무 음원 재생 없이 배경소음만 측정)
- 1: C4-B4 온음계 (도, 레, 미, 파, 솔, 라, 시)
- 2: C4-B4 12음계
- 3: 점점 증가하는 저주파(100Hz-162Hz)
- 4: 점점 증가하는 음역대(300Hz-600Hz)
- 5: 점점 증가하는 음역대(600Hz-1120Hz)
- 6: 12음계 왕복
- 7: 두 음 동시 재생(도, 솔)
- 8: 3옥타브 12음계 (C3~B5)

### 숫자2
1-4 사이의 숫자입니다.
- 1: 창함수, 보간 모두 적용 X
- 2: 창함수만 적용
- 3: 보간만 적용
- 4: 창함수+보간 적용
