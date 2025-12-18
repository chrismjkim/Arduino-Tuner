#include <Wire.h>               // I2C(TWI) 통신을 위한 라이브러리
#include <LiquidCrystal_I2C.h>  // I2C 모듈 탑재한 LCD 디스플레이 사용
#include <arduinoFFT.h>         // FFT 라이브러리
#include <math.h>               // 수학 툴

// ===== 디버깅 모드(Serial 출력) =====
static const bool debugMode = true;
static const int micPin = A0;

// ===== LCD =====
LiquidCrystal_I2C lcd(0x27, 16, 2); // 미작동 시 0x3F로 세팅하기

// ===== FFT/샘플링 설정 =====
static const uint16_t FFT_N = 128;      // UNO SRAM 고려(권장 128)
static const uint8_t  DECIM = 3;        // 1/DECIM로 다운샘플(프레임 길이↑, 안정성↑)

// ADC prescaler = 128 고정 (아래 setupADCFreeRunningA0에서 설정)
static const float FS_RAW = (float)F_CPU / (128.0f * 13.0f);      // ≈ 9615.38Hz
static const float FS     = FS_RAW / (float)DECIM;               // 유효 fs

// 오실레이터/환경 편차 보정용 (필요하면 0.995~1.010 같은 값으로 미세조정)
static const float FS_CAL = 0.9950f;

// ===== 피치 범위(기타+색소폰 목표) =====
static const float MIN_FREQ = 70.0f;         // 감지할 최저 주파수
static const float MAX_FREQ = 1200.0f;       // 감지할 최고 주파수
static const float A4_FREQ  = 442.0f;        // A4(라) 음 주파수

// 게이트/평활
static const float RMS_THRESH  = 16.0f;      // 입력 약하면 NO SIG
static const float CONF_THRESH = 6.0f;       // peak/avg magnitude 비율(경험값)
static const float EMA_ALPHA   = 0.95f;      // 새 입력값을 반영하는 정도 (0~1 사이 값)

// LCD 갱신 2Hz
static const uint32_t LCD_INTERVAL_MS = 500; // LCD 업데이트 주기(ms)

// ===== 버퍼/FFT 워킹 =====
volatile int16_t  adcBuf[FFT_N];      // 샘플을 저장할 버퍼
volatile uint16_t adcIdx = 0;         // 
volatile bool     frameReady = false; // 프레임이 준비되었는지
volatile uint8_t  decimCnt = 0;       // 

float vReal[FFT_N];    // 실수부 (샘플링 값 복사할 위치)
float vImag[FFT_N];    // 허수부 (FFT 연산에서만 사용)
ArduinoFFT<float> FFT(vReal, vImag, FFT_N, FS); // sampling freq는 아래에서 직접 사용

static float freqEma = 0.0f;   // 디스플레이에 업데이트되는 주파수 값을 주변 값으로 보정
static uint32_t lastLcdMs = 0; // LCD 업데이트 주기 돌리는 데 사용하는 변수

const char* NOTE_NAMES[12] = {"C","C#","D","D#","E","F","F#","G","G#","A","A#","B"};

// ===== ADC ISR (free-running) =====
// analogRead로 샘플링할 시 샘플링 간격이 들쭉날쭉해지므로 고정 주기로 샘플링
ISR(ADC_vect) {
  if (frameReady) return; // 프레임 소비 전엔 덮어쓰기 방지(샘플 드랍)

  // 디시메이션: DECIM번째 샘플만 저장(다운샘플링)
  decimCnt++;
  if (decimCnt < DECIM) return;
  decimCnt = 0;

  int16_t v = (int16_t)ADC - 512;  // DC offset 제거
  adcBuf[adcIdx++] = v;            // 버퍼에 샘플링 정수값(-512~511) 추가

  // FFT_N개의 샘플이 모였으면 frameReady를 true로 바꾼다
  if (adcIdx >= FFT_N) {
    adcIdx = 0;
    frameReady = true;
  }
}

// UNO 보드의 ADC를 자동으로 계속 샘플링하는 모드로 설정
void setupADCFreeRunningA0() {
  // AVcc reference, A0
  ADMUX  = (1 << REFS0) | 0; // MUX=0(A0)

  // ADC Enable, Auto Trigger, Interrupt Enable, Prescaler 128
  ADCSRA = (1 << ADEN)  |
           (1 << ADATE) |
           (1 << ADIE)  |
           (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

  ADCSRB = 0;                 // Free running
  ADCSRA |= (1 << ADSC);      // Start
}

// min과 max 사이의 값으로 x를 클램핑
static inline float clampf(float x, float min, float max) {
  if (x < min) return min;
  if (x > max) return max;
  return x;
}

// freqToNote 용 함수
static float hzToMidi(float f) {
  return 69.0f + 12.0f * (logf(f / A4_FREQ) / logf(2.0f));
}

// 추정된 주파수를 바탕으로 음과 센트수를 반환
static void freqToNote(float freq, int &midi, float &cents) {
  float n = hzToMidi(freq);
  midi = (int)lroundf(n);
  float target = A4_FREQ * powf(2.0f, ((float)midi - 69.0f) / 12.0f);
  cents = 1200.0f * (logf(freq / target) / logf(2.0f));
}

// LCD 업데이트(신호가 불량일 때) 
static void showNoSignal(float rms) {
  lcd.setCursor(0, 0);
  lcd.print(F("NO SIGNAL       "));
  lcd.setCursor(0, 1);
  lcd.print(F("rms "));
  int rr = (int)lroundf(rms);
  if (rr < 0) rr = 0;
  if (rr > 999) rr = 999;
  if (rr < 100) lcd.print(' ');
  if (rr < 10)  lcd.print(' ');
  lcd.print(rr);
  lcd.print(F("            "));
}

// LCD 업데이트(음이 추정되었을 때)
static void showPitch(float freq, float cents, int midi, float conf) {
  int noteIndex = ((midi % 12) + 12) % 12;
  int octave = (midi / 12) - 1;

  // 1행: NOTE OCT + cents
  lcd.setCursor(0, 0);
  lcd.print(NOTE_NAMES[noteIndex]);
  lcd.print(octave);
  lcd.print(F("  "));
  if (cents >= 0) lcd.print(F("+"));
  lcd.print(cents, 1);
  lcd.print(F("c     "));
  // 잔상 제거
  lcd.print(F("   "));

  // 2행: freq Hz + conf
  lcd.setCursor(0, 1);
  lcd.print(freq, 1);
  lcd.print(F("Hz "));
  lcd.print(F("q"));
  lcd.print(conf, 1);
  lcd.print(F("      "));
}

// Buffer에서 unsigned int로 저장된 값들을 꺼내와서 vReal에 복사한다.
static void copyFrameToFloatAndRms(float *outRms) {
  // adcBuf -> vReal (DC 제거 후) / vImag=0
  // RMS 계산
  // 버퍼에 있는 값들의 평균(mean)을 구한다.
  int32_t sum = 0;
  for (uint16_t i = 0; i < FFT_N; i++) sum += adcBuf[i];
  float mean = (float)sum / (float)FFT_N;

  float e = 0.0f; // e: 분산(편차 제곱의 합)
  for (uint16_t i = 0; i < FFT_N; i++) {
    float x = (float)adcBuf[i] - mean;
    vReal[i] = x; // 버퍼에 있는 값의 DC(평균)를 제거하고 편차를 집어넣는다
    vImag[i] = 0.0f;
    e += x * x;
  }

  // rms: 표준오차(standard error)
  float rms = sqrtf(e / (float)FFT_N);
  if (outRms) *outRms = rms;
}

// peak의 실제 위치를 2차보간을 1회 거쳐서 보정한다(peakBin(=k)은 정수값이기 때문)
static float quadraticInterpPeakHz(const float *mag, uint16_t k, float fsEff) {
  // k 주변 3점 파라볼릭 보간
  // k는 peakBin이므로 y2가 반드시 y1, y3보다 크다.
  float y1 = mag[k - 1];
  float y2 = mag[k];
  float y3 = mag[k + 1];
  float denom = (y1 - 2.0f * y2 + y3);
  float delta = 0.0f;
  if (fabsf(denom) > 1e-12f) {
    delta = 0.5f * (y1 - y3) / denom;
    delta = clampf(delta, -0.5f, 0.5f);
  }
  float kHat = (float)k + delta;        // 보정된 peak의 위치
  return (kHat * fsEff) / (float)FFT_N; // 보정된 peak의 frequency 리턴
}

static float magAtHzApprox(const float *mag, float hz, float fsEff) {
  // 주파수->bin 근처 magnitude(간단히 최근접 bin)
  float kf = hz * (float)FFT_N / fsEff;
  int k = (int)lroundf(kf);
  if (k < 1) k = 1;
  if (k > (int)(FFT_N / 2 - 1)) k = (int)(FFT_N / 2 - 1);
  return mag[k];
}

static bool estimatePitchFFT(float &outF0, float &outConf, float &outRms) {
  const float fsEff = FS * FS_CAL;

  // 1) 프레임 복사 + RMS
  copyFrameToFloatAndRms(&outRms);
  if (outRms < RMS_THRESH) {
    outConf = 0.0f;
    outF0 = 0.0f;
    return false;
  }

  // 2) FFT
  FFT.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD); // 피크 주변이 깨끗하므로 HAMMING을 창함수로 사용
  FFT.compute(FFT_FORWARD);
  FFT.complexToMagnitude();

  // 3) 피크 탐색(bin)
  uint16_t minBin = (uint16_t)ceilf(MIN_FREQ * (float)FFT_N / fsEff);
  uint16_t maxBin = (uint16_t)floorf(MAX_FREQ * (float)FFT_N / fsEff);
  if (minBin < 1) minBin = 1;
  if (maxBin > (FFT_N / 2 - 2)) maxBin = (FFT_N / 2 - 2); // 보간 위해 +1 필요
  if (minBin >= maxBin) {
    outConf = 0.0f;
    outF0 = 0.0f;
    return false;
  }

  float peakMag = 0.0f;         // 감지된 최대 magnitude 값
  uint16_t peakBin = minBin;    // peakMag가 나온 bin 인덱스

  float avg = 0.0f;
  uint16_t cnt = 0;
  // 각 bin들을 순회하면서 peakMag와 peakBin을 찾는다.
  for (uint16_t k = minBin; k <= maxBin; k++) {
    float m = vReal[k];
    avg += m;
    cnt++;
    // peakMag가 나온 bin을 갱신한다
    if (m > peakMag) { peakMag = m; peakBin = k; }
  }

  avg = (cnt > 0) ? (avg / (float)cnt) : 0.0f;

  // conf: 피크가 바닥(평균) 대비 얼마나 우세한지
  outConf = (avg > 1e-6f) ? (peakMag / avg) : 0.0f;
  if (outConf < CONF_THRESH) {
    outF0 = 0.0f;
    return false;
  }

  // 4) 보간으로 peak 주파수
  float fPeak = quadraticInterpPeakHz(vReal, peakBin, fsEff);

  // 5) Harmonic (옥타브) 보정
  // fundamental이 약해서 2배/3배가 더 커지는 케이스 완화
  float f0 = fPeak;
  float mPeak = peakMag;

  // f0의 1/2배, 1/3배, 1/4배 주파수인 음들을 f0의 후보로 검사한다.
  for (int div = 2; div <= 4; div++) {
    float cand_f = fPeak / (float)div;
    // 주파수가 너무 낮으면 검사를 건너뜀
    if (cand_f < MIN_FREQ) continue;
    float mc = magAtHzApprox(vReal, cand_f, fsEff);
    // cand가 충분히 존재하면 fundamental로 내림
    if (mc > 0.35f * mPeak) {
      f0 = cand_f;
    }
  }

  outF0 = f0;
  return true;
}

void setup() {
  Wire.begin();
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0,0);
  lcd.print(F("Tuner boot...   "));
  lcd.setCursor(0,1);
  lcd.print(F("FFT + ADC ISR   "));

  if (debugMode) {
    Serial.begin(500000);
    pinMode(micPin, INPUT);
    delay(600);
    lcd.clear();
    lcd.setCursor(3, 0);
    lcd.print("DEBUG MODE");
    lcd.setCursor(2, 1);
    lcd.print("CHECK SERIAL");
    frameReady = true;
  }  
  else {
    setupADCFreeRunningA0(); // 고정 주기로 자동 샘플링 활성화
    sei(); // Global Interrupt 활성화
    delay(600);
    lcd.clear();
  }
}

void loop() {
  if (!frameReady) return;

  if (debugMode) {
    // unsigned integer가 아닌 일반 integer로 값을 받을 시 자료형 변환 시간에 의한 sampling jitter 발생
    uint16_t v = analogRead(A0);   // 0~1023
    Serial.write((uint8_t*)&v, 2); // 2바이트 바이너리 송신 (리틀엔디안)
    return;
  }

  // 프레임을 빠르게 소비(복사)하고 ADC는 바로 재개
  noInterrupts();
  frameReady = false;
  interrupts();

  float f0 = 0.0f, conf = 0.0f, rms = 0.0f;
  bool ok = estimatePitchFFT(f0, conf, rms);

  // LCD 업데이트 쿨타임이 안 돌았으면 return한다
  uint32_t now = millis();
  if (now - lastLcdMs < LCD_INTERVAL_MS) return;
  lastLcdMs = now;

  // 신호 탐지가 안 되면 no signal을 LCD에 띄운다
  if (!ok || f0 < MIN_FREQ || f0 > MAX_FREQ) {
    showNoSignal(rms);
    freqEma = 0.0f;
    return;
  }

  // EMA 평활(표시 안정화)
  if (freqEma <= 0.0f) freqEma = f0;
  // 누적된 이전 값(freqEma)와 새 값(f0)의 내분점으로 freqEma를 업데이트
  else freqEma = (1.0f - EMA_ALPHA) * freqEma + EMA_ALPHA * f0;

  int midi;
  float cents;
  freqToNote(freqEma, midi, cents);

  showPitch(freqEma, cents, midi, conf);
}
