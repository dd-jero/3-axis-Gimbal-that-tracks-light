# Embedded System Capstone Design Project


## ✔️프로젝트 명
Gimbal system that tracks light
## ✔️ 개발 기간
2022.05 ~ 2022.06
## ✔️ 팀원 
|윤대민|[김태완](https://github.com/Waan2y) |[이재영](https://github.com/dd-jero)|
| :---: | :---: | :---: |
|팀장, I2C 통신 및 데이터 처리|PWM 설정 및 서보 모터 제어|Tracking 로직 구현|

## ✔️ 개발 환경 
- Chipset: ATmega4809(MCU), MCP2221A(USB-to-UART/I2C Convertor)
- Sensor: MPU6050, GL5549
- Servo motor
- Programming: AVR registers manipulation with C/C++
- 3D printing: Autodesk 3ds Max, UltiMaker Cura, Ender-3

## ✔️ 개발 목표
- 타깃 조준을 유지하는 미사일 발사대 컨셉

## ✔️ 주요 기능

### I2C 통신
- 센서 데이터 수집: MPU6050 센서로부터 가속도 및 자이로 데이터 수집
- **Drift Calibration**
  1. 배열 초기화
  2. 자이로스코트 데이터 읽기
  3. 값 저장 및 차이 계산 
  4. 드리트프 오류 추정
  
### PWM 제어
- TCA 레지스터를 사용한 PWM 신호를 신호를 생성하고 서보 모터 제어

### ADC 데이터 처리 
- CDS 센서의 아날로그 입력을 읽고 평균값을 계산하여 센서 오프셋 보정
- **Offset 설정**
  1. 초기화 및 데이터 수집
  2. 오프셋 계산
- **Offset 보정**
  1. 오프셋 적용
  2. 평균 값 계산
  3. 이동 상태 결정

### Tracking 및 자세 제어
1. ADC 데이터 처리
    - 수직 및 수평 이동 상태 결정 위해 상하좌우 평균값을 비교
2. 움직임 상태 결정
    - 수직 및 수평 상태 플래그를 설정하여 움직임 방향 결정
    - 특정 임계값 기준 
3. 최종 위치 조정 
    - tracking 각도 기반의 수평 및 수직 상태에 따른 조정
    - 최종적으로 각도 변환 후 PWM 신호를 통해 서보 모터 제어 

## ✔️ 회로도
<p align="center"> <img alt="circuit" src="https://github.com/user-attachments/assets/d58a7ac7-3ecc-4c50-93e5-dc62fd996775"></p>

## ✔️ 시스템 구현 모델
<p align="center"> <img alt="model" src="https://user-images.githubusercontent.com/107921434/182002369-21683a1b-39f4-4c44-a1f4-7b13c62e6e0f.png"></p>