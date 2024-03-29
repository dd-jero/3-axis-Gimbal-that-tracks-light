## Embedded System Capstone Design Project : 3-axis Gimbal that tracks light
<p align="center"> <img width="210" alt="model" src="https://user-images.githubusercontent.com/107921434/182002369-21683a1b-39f4-4c44-a1f4-7b13c62e6e0f.png"> <center></p>

 <ul>
  <li>
   Using: ATmega4809, MCP2221A, MPU6050, GL5549
  </li>
  <li>
   구현 요소
   <ul>
    <li>
      MPU6050으로 자세 제어 데이터 수집 (I2C Communication)
    </li>
    <li>
     Offset 보정 활용한 Tracking Logic
    </li>
    <li>
     PWM을 이용한 Servo Motor 제어 <br>=> 자세 변화 각도(Drift Calibration) + tracking 위해 이동한 각도 반영 
    </li>
   </ul>
  </li>
 </ul>
