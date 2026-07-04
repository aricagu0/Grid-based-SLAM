# Grid-based-SLAM
# STM32F103RB 기반 격자 매핑 로봇 (Grid-based SLAM)

> STM32 Nucleo 보드와 초음파 센서, 자이로 센서를 이용해  
> 실내 환경을 자율 주행하며 2D 격자 지도를 작성하는 임베디드 프로젝트

[![Demo Video](https://img.shields.io/badge/시연영상-YouTube-red?logo=youtube)](https://youtu.be/YnWoP8pAKlE)
[![License](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
![Platform](https://img.shields.io/badge/Platform-STM32F103RB-green)
![Language](https://img.shields.io/badge/Language-C%20%7C%20Python-yellow)

---

## 목차

1. [프로젝트 개요](#프로젝트-개요)
2. [시스템 구성](#시스템-구성)
3. [주요 기술 도전 과제](#주요-기술-도전-과제)
4. [펌웨어 구조](#펌웨어-구조)
5. [통신 프로토콜](#통신-프로토콜)
6. [PC 시각화 소프트웨어](#pc-시각화-소프트웨어)
7. [실험 결과](#실험-결과)
8. [개발 환경](#개발-환경)
9. [빌드 방법](#빌드-방법)

---

## 프로젝트 개요

110×200cm 크기의 실내 공간을 10cm 단위로 이동하며 초음파 180° 스캔을 반복, 총 44스텝에 걸쳐 공간의 2D 점유 격자 지도를 생성합니다.

```
주행 경로: □─□─□─□─□
           │         │
           □─□─□─□─□
총 44스텝 × (10cm 전진 → 180° 스캔) → 전체 소요 시간 약 6분 35초
```

### 핵심 구현 포인트

| 항목 | 구현 방법 |
|------|-----------|
| 위치 추정 | 엔코더 펄스 카운팅 (Dead Reckoning) |
| 각도 제어 | MPU6050 자이로 Z축 적분 |
| 거리 측정 | HC-SR04 초음파 × 2 (180° 오프셋) |
| 스캔 구동 | SG90 서보 — GPIO 비트뱅잉 PWM |
| PC 전송 | HC-06 블루투스 → USART3 (115200 baud) |

---

## 시스템 구성

### Hardware

```
┌──────────────────────────────────────────────────┐
│                 STM32F103RB Nucleo                │
│                                                  │
│  I2C(GPIO) ──── MPU6050 (자이로/가속도)          │
│  SPI1      ──── ST7735 LCD (상태 표시)           │
│  USART3    ──── HC-06 블루투스 → PC             │
│  USART2    ──── 디버그 출력                      │
│  ADC1      ──── CDS 조도 센서                    │
│                                                  │
│  TIM1~4 PWM ── L298N × 2 ── DC 모터 4개         │
│  GPIO       ── HC-SR04 × 2 (TRIG/ECHO)          │
│  EXTI       ── KY-033 트래킹 모듈 × 2 (엔코더)  │
│  GPIO(SW)   ── SG90 서보모터                     │
│  EXTI4      ── IR 리모컨 수신부                  │
└──────────────────────────────────────────────────┘
```

### 핀 배치 요약

| 핀 | 기능 | 비고 |
|----|------|------|
| PA8 | TIM1_CH1 (서보 PWM) | SW PWM으로 최종 변경 |
| PB4/PB5 | TIM3_CH1/CH2 (RFF/RFB) | 우전방 모터 |
| PB12 | ECHO1_SLAM | EXTI15_10 공유 주의 |
| PB2 | ECHO2_SLAM | |
| PB13 | TRACK_L (엔코더) | EXTI15_10 공유 |
| PC3 | TRACK_R (엔코더) | EXTI3 |
| PB6/PB7 | SCL/SDA (Soft I2C) | MPU6050 |

---

## 주요 기술 도전 과제

### 1. 트래킹 모듈로 엔코더 구현

상용 엔코더 없이 KY-033 라인 트래킹 모듈을 바퀴에 부착해 엔코더 기능을 구현했습니다.

**시행착오 과정:**

```
시도 1: 검정 슬릿 감지 → 촘촘하여 미감지
시도 2: 슬릿 수 줄이기 → 펄스 생성되나 2회에 1회 건너뜀
시도 3: 흰색 슬릿 감지로 전환 → 안정적 펄스 생성 ✓
```

**노이즈 대응:**

```c
// 모터 PWM 전자기 간섭으로 인한 오신호 방지
#define DEBOUNCE_CYCLES (SystemCoreClock / 1000 * 3)  // 3ms

void EXTI15_10_IRQHandler(void) {
    static uint32_t last_tick = 0;
    uint32_t now = DWT->CYCCNT;
    if (now - last_tick > DEBOUNCE_CYCLES) {
        pulse_count_L++;
        last_tick = now;
    }
}
```

---

### 2. 10cm 정밀 주행 (Dead Reckoning)

**이론값 vs 실측값 보정:**

```
이론: 바퀴 둘레 18cm ÷ 슬릿 8개 = 2.25 cm/pulse
실측: 바퀴 미끄러짐 보정 → 1.0 cm/pulse
관성 코스팅: 모터 정지 후 5.3cm 추가 주행 → 선보정 적용
```

```c
#define CM_PER_PULSE      1.0f
#define STOP_OVERSHOOT_CM 5.3f
#define TARGET_DIST_CM    10.0f
#define TARGET_PULSE      ((uint32_t)((TARGET_DIST_CM - STOP_OVERSHOOT_CM) \
                          / CM_PER_PULSE + 0.5f))   // = 5 펄스

// 목표 펄스 도달 시 모터 정지
while (avg_pulse < TARGET_PULSE && !timeout) { /* 대기 */ }
motor_stop();
```

**EXTI 공유 문제 (ECHO1 ↔ TRACK_L):**

PB12(ECHO1)와 PB13(TRACK_L)이 동일한 EXTI15_10_IRQn을 공유합니다.  
인터럽트 방식 사용 시 핸들러 내 분기 처리가 복잡해지므로,  
초음파 에코는 **DWT 사이클카운터 기반 Busy-wait**으로 처리했습니다.

```c
// DWT 기반 마이크로초 타이머 (인터럽트 없이 정밀 측정)
static inline uint32_t delay_us_start(void) {
    return DWT->CYCCNT;
}
static inline uint32_t elapsed_us(uint32_t start) {
    return (DWT->CYCCNT - start) / (SystemCoreClock / 1000000);
}
```

---

### 3. 360° 초음파 스캔 및 노이즈 보정

서보 0~180° 스윕으로 두 센서가 0~360°를 커버합니다.

```
센서1 (0°  오프셋): 서보 0~180° → 실제 0~180°
센서2 (180° 오프셋): 서보 0~180° → 실제 180~360°
```

**2단계 중간값 필터:**

```c
// 1단계: 각 각도에서 3회 측정 → 중간값
float measure_echo1_median(void) {
    float v[3];
    for (int i = 0; i < 3; i++) {
        v[i] = measure_echo1();   // 타임아웃(-1) 제외
        HAL_Delay(4);
    }
    return median3(v);
}

// 2단계: N_SWEEP회 스윕 결과의 중간값 (현재 N_SWEEP=1)
float d = median_of_n(angle, N_SWEEP);
```

**서보 엔드포인트 캘리브레이션:**

```c
// 기계적 한계 보정: 500µs(0°) ~ 2500µs(180°)
// 실제 175°까지만 회전 → 캘리브레이션으로 보정
uint32_t pulse_us = 500 + (2500 - 500) * angle / 180;
```

**센서1 신뢰 불가 구간 처리:**

센서2 본체가 서보 90° 부근에서 센서1 시야를 가려  
40~140° 구간의 센서1 occupied 마킹을 억제합니다.

---

### 4. MPU6050 자이로 기반 90° 회전

```c
void test_rotate_90(void) {
    // 1. 영점 보정 (200회 평균)
    MPU6050_CalibrateGyroZ(200);
    
    float yaw = 0.0f;
    uint32_t prev = HAL_GetTick();
    
    // 2. 피벗 회전 시작 (좌측↑ 우측↓)
    motor_turn_left();
    
    // 3. 자이로 적분으로 누적 각도 계산
    while (yaw < 90.0f) {
        float gz = MPU6050_GetGyroZ();          // °/s
        float dt = (HAL_GetTick() - prev) / 1000.0f;
        yaw += gz * dt;
        prev = HAL_GetTick();
    }
    
    motor_stop();
    HAL_Delay(400);   // 관성 완전 정지 대기
}
```

**실측:** 반시계 방향 10° 오차 → 시계 방향 선택(오차 0°)

---

### 5. GPIO 비트뱅잉 서보 PWM

TIM 리소스를 모터 4개에 모두 사용하여 서보용 타이머 부족 → SW PWM으로 해결

```c
void servo_write_angle(float angle) {
    uint32_t pulse_us = 500 + (uint32_t)((2500 - 500) * angle / 180.0f);

    for (int i = 0; i < 2; i++) {          // 20ms 주기 × 2회
        HAL_GPIO_WritePin(SERVO_GPIO, SERVO_PIN, GPIO_PIN_SET);
        delay_us(pulse_us);                 // DWT 기반 정밀 딜레이
        HAL_GPIO_WritePin(SERVO_GPIO, SERVO_PIN, GPIO_PIN_RESET);
        delay_us(20000 - pulse_us);
    }
}
```

---

## 펌웨어 구조

```
firmware/
├── Core/
│   ├── Src/
│   │   ├── main.c          # 메인 루프, 모터 제어, 엔코더 ISR
│   │   ├── slam.c          # 서보 제어, 초음파 측정, 스캔 시퀀스
│   │   ├── mpu6050.c       # Soft I2C 드라이버, 자이로 캘리브레이션
│   │   └── soft_i2c.c      # GPIO 비트뱅잉 I2C
│   └── Inc/
│       ├── main.h
│       ├── mpu6050.h
│       └── soft_i2c.h
└── smart_car.ioc           # STM32CubeMX 핀 설정 파일
```

### 주요 상수 (main.c)

```c
#define CM_PER_PULSE      1.0f    // 실측 보정값 (이론: 2.25)
#define STOP_OVERSHOOT_CM 5.3f   // 모터 정지 후 관성 이동 거리
#define TARGET_DIST_CM    10.0f  // 1스텝 목표 거리
#define TEST_DUTY         32767  // TIM PWM 듀티 (50%, ARR=65535)
#define DEBOUNCE_CYCLES   (SystemCoreClock / 1000 * 3)  // 3ms
#define N_SWEEP           1      // 스캔 반복 횟수
```

---

## 통신 프로토콜

USART3 → HC-06 블루투스 → PC (ASCII, 115200 baud, 8N1)

| 패킷 타입 | 형식 | 크기 |
|-----------|------|------|
| 전진 Step | `STEP,<step>,<dist_cm>,<pL>,<pR>[,<ax>,<ay>,<az>,<gz>]\r\n` | ~22 byte |
| 스캔 포인트 | `S,<angle>,<dist1>,<dist2>\r\n` | ~18 byte |
| 회전 완료 | `ROT,<yaw>,<ax>,<ay>,<az>\r\n` | ~30 byte |

> **향후 개선:** ASCII → Binary 프로토콜 전환 시 스캔 전송 시간 174ms → 47ms 단축 가능

---

## PC 시각화 소프트웨어

`pc_software/slam_radar_view6.py`

- **블루투스 수신** → 실시간 극좌표 레이더 뷰 갱신
- **Dead Reckoning** 기반 로봇 위치 추적 및 경로 표시
- **yaw 90° 스냅 보정**: `round(yaw / 90) * 90`
- **센서1 신뢰 불가 구간** 자동 필터링 (40~140°)
- 44회 전체 스캔 결과 저장 및 CSV 로그 출력

```bash
pip install pyserial matplotlib numpy
python slam_radar_view6.py
```

---

## 실험 결과

| 항목 | 결과 |
|------|------|
| 10cm 주행 오차 | ±1.5cm |
| 90° 회전 정확도 | 시계방향 0° 오차 |
| 스캔 포인트 수 (1회) | 91개 (180°, 2°씩) |
| 총 매핑 소요 시간 | 약 6분 35초 |
| 소모 전력 (1회) | 약 34 mAh |
| 배터리 지속 | 약 76회 (2600mAh 기준) |

### 매핑 결과

| 환경 | Dead-Reckoning 궤적 | 스캔 포인트 클라우드 |
|------|---------------------|---------------------|
| 박스 없음 | 정사각형 경로 확인 | 외벽 형태 인식 |
| 박스 있음 | 정사각형 경로 확인 | 중앙 장애물 감지 |

---

## 개발 환경

| 구분 | 도구 |
|------|------|
| MCU | STM32F103RB (64MHz, Cortex-M3) |
| IDE | STM32CubeIDE 1.x |
| 펌웨어 라이브러리 | STM32 HAL |
| PC 소프트웨어 | Python 3.x, Anaconda |
| Python 라이브러리 | pyserial, matplotlib, numpy |
| 통신 | HC-06 블루투스 (115200 baud) |
| 개발 기간 | 2026.06.22 ~ 2026.07.02 (11일) |

---

## 빌드 방법

### 펌웨어

1. STM32CubeIDE에서 `firmware/` 폴더를 프로젝트로 Import
2. `smart_car.ioc`로 핀 설정 확인
3. Build → Flash (ST-Link)

### PC 소프트웨어

```bash
cd pc_software
pip install -r requirements.txt
python slam_radar_view6.py
# COM 포트는 코드 내 PORT 변수에서 수정
```

---

## 향후 개선 과제

- [ ] 초음파 → LiDAR 센서 대체 (정확도 향상)
- [ ] PC → Raspberry Pi 온보드화 (완전 자율 주행)
- [ ] ASCII → Binary 통신 프로토콜 전환 (전송 속도 3.7배↑)
- [ ] 루프 클로저 구현 (완전한 SLAM)
- [ ] 환경 데이터 (온도·습도·조도) 프로토콜 추가

---

## 팀 구성

| 이름 | 역할 |
|------|------|
| 구기범 | 펌웨어 개발 (모터 제어, 엔코더, 90° 회전) |
| 우애록 | 펌웨어 개발 (초음파 스캔, GUI, 통신) |

---

## 라이선스

MIT License — 자유롭게 참고·수정 가능합니다.
