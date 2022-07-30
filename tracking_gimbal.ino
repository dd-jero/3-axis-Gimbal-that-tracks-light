/*
    2022-1 임베디드시스템설계 최종프로젝트

    Team. 윤대민, 김태완, 이재영
    ver.  final_1
    date. 2022-06-15 14:49

    == Note ==
 *  * 정상 동작 시험 완료 *
    1. arrsize 커지면, 계산하는데 오래걸려서 빛을 지나쳐서 계속 가다 멈춰버림 -> arrsize 이제 사용 안해서 기능 없앰 함수 형태만 약간 수정
    2. x 축 다리길이 부족해서 걸림
    3. 이외 기능 모두 정상 동작
*/


// __ UART __ 구현상에서 사용하지는 않는 부분임. Data 확인 용
#include <SoftwareSerial.h>
SoftwareSerial mySerial(9, 8); // RX, TX

// __ I2C __

#define TWI0_BAUD(F_SCL)      ((((float)F_CPU / (float)F_SCL)) - 10 )
#define I2C_SCL_FREQ                                    400000
#define I2C_DIRECTION_BIT_WRITE                         0
#define I2C_DIRECTION_BIT_READ                          1
#define RAD_TO_DEG  (180 / 3.14159)

int16_t raw_Acc_x, raw_Acc_y, raw_Acc_z;
int16_t raw_temp;
int16_t raw_Gyro_x, raw_Gyro_y, raw_Gyro_z;
float Acc_x, Acc_y, Acc_z;
float Gyro_x, Gyro_y, Gyro_z;
float angle_Acc_x, angle_Acc_y, angle_Acc_z;
float angle_Gyro_x, angle_Gyro_y, angle_Gyro_z;
float angle_x = 0, angle_y = 0, angle_z = 0;
float Gyro_x_offset, Gyro_y_offset, Gyro_z_offset;
float Gyro_x_avg, Gyro_y_avg, Gyro_z_avg;
float fin_x, fin_y, fin_z;
float Interval;
float preInterval;
float cali_gyro;
float AccCoef = 0.98f;
float GyroCoef = 0.02f;


void I2C_0_init() {
  TWI0_MBAUD = 30;
  TWI0_MCTRLA = B00000001;
  //TWI0_MSTATUS = B11000001;
  TWI0.MCTRLB |= TWI_FLUSH_bm;
  TWI0.MSTATUS |= (TWI_RIF_bm | TWI_WIF_bm);
  TWI0_MSTATUS |= TWI_BUSSTATE_IDLE_gc;
}

void I2C_setup() {
  I2C_0_init();

  I2C_0_start(0x68, I2C_DIRECTION_BIT_WRITE);
  I2C_0_writingPacket(0x6B);
  I2C_0_writingPacket(0);
  I2C_0_stop();

  I2C_0_start(0x68, I2C_DIRECTION_BIT_WRITE);
  I2C_0_writingPacket(0x1A);
  I2C_0_writingPacket(0x06);
  I2C_0_stop();

  I2C_0_start(0x68, I2C_DIRECTION_BIT_WRITE);
  I2C_0_writingPacket(0x1B);
  I2C_0_writingPacket(0x08);
  I2C_0_stop();


  I2C_0_start(0x68, I2C_DIRECTION_BIT_WRITE);
  I2C_0_writingPacket(0x1C);
  I2C_0_writingPacket(0x00);
  I2C_0_stop();
}

uint8_t I2C_0_start(uint8_t baseAddres, uint8_t directionBit) {
  TWI0_MADDR = (baseAddres << 1) + directionBit;
  while (!(TWI0_MSTATUS & (TWI_WIF_bm | TWI_RIF_bm)));    //wait for write or read interrupt flag
  if ((TWI0_MSTATUS & TWI_ARBLOST_bm)) return 0 ;         //return 0 if bus error or arbitration lost
  return !(TWI0_MSTATUS & TWI_RXACK_bm);                  //return 1 if slave gave an ack
}

uint8_t I2C_0_writingPacket(uint8_t data) {
  while (!(TWI0_MSTATUS & TWI_WIF_bm));               //wait for write interrupt flag
  TWI0_MDATA = data;
  TWI0_MCTRLB = TWI_MCMD_RECVTRANS_gc;
  return (!(TWI0_MSTATUS & TWI_RXACK_bm));        //returns 1 if slave gave an ack
}

uint8_t I2C_0_receivingPacket(uint8_t acknack) {
  while (!(TWI0_MSTATUS & TWI_RIF_bm));               //wait for read interrupt flag
  uint8_t data = TWI0_MDATA;
  if (acknack == 0) {
    TWI0_MCTRLB = (TWI_ACKACT_ACK_gc  | TWI_MCMD_RECVTRANS_gc);
  }
  else                {
    TWI0_MCTRLB = (TWI_ACKACT_NACK_gc | TWI_MCMD_STOP_gc);
  }
  return data;
}

void I2C_0_stop(void) {
  TWI0_MCTRLB = (TWI_MCMD_STOP_gc);

}

void calcoffset() {
  int x = 0, y = 0, z = 0;
  uint16_t rx, ry, rz;
  for (int i = 0; i < 100; i++) {
    I2C_0_start(0x68, I2C_DIRECTION_BIT_WRITE);
    I2C_0_writingPacket(0x43);
    I2C_0_start(0x68, I2C_DIRECTION_BIT_READ);
    rx = (I2C_0_receivingPacket(0) << 8) | I2C_0_receivingPacket(0);
    ry = (I2C_0_receivingPacket(0) << 8) | I2C_0_receivingPacket(0);
    rz = (I2C_0_receivingPacket(0) << 8) | I2C_0_receivingPacket(1);

    x += ((float)rx);
    y += ((float)ry);
    z += ((float)rz);
  }

  Gyro_x_offset = x / 100;
  Gyro_y_offset = y / 100;
  Gyro_z_offset = z / 100;
}
// __ PWM __ ( via TCA )

#define PORTD_DIR   (unsigned char *)(0x0460)
#define PORTD_OUT   (unsigned char *)(0x0464)

#define PORTMUX_TCA (unsigned char *)(0x05E4)

#define TCA_CTRLA   (unsigned char *)(0x0A00)
#define TCA_CTRLB   (unsigned char *)(0x0A01)
#define TCA_CTRLD   (unsigned char *)(0x0A03)
#define TCA_CTRLESET     (unsigned char *)(0x0A05)
#define TCA_INTCTRL   (unsigned char *)(0x0A0A)
#define TCA_LCNT     (unsigned char *)(0x0A20)
#define TCA_HCNT     (unsigned char *)(0x0A21)
#define TCA_HPER     (unsigned char *)(0x0A26)
#define TCA_LPER     (unsigned char *)(0x0A27)
#define TCA_LCMP0    (unsigned char *)(0x0A28)
#define TCA_HCMP0    (unsigned char *)(0x0A29)
#define TCA_LCMP1    (unsigned char *)(0x0A2A)
#define TCA_HCMP1    (unsigned char *)(0x0A2B)
#define TCA_LCMP2    (unsigned char *)(0x0A2C)
#define TCA_HCMP2    (unsigned char *)(0x0A2D)

void TCA_reset() {
  *TCA_CTRLA = 0;
  *TCA_CTRLESET = B00001100;
  *TCA_HCNT = 0;
  *TCA_LCNT = 0;
  *TCA_HCMP0 = 0;
  *TCA_LCMP0 = 0;
  *TCA_HCMP1 = 0;
  *TCA_LCMP1 = 0;
  *TCA_HCMP2 = 0;
  *TCA_LCMP2 = 0;
}

void TCA_init() {
  *PORTMUX_TCA = 0x3;
  *TCA_LPER = 1250;
  *TCA_HPER = 1250;
  *TCA_CTRLA = B00001101;   // DIV1024
  *TCA_CTRLB = B01110111;
  *TCA_CTRLD = B00000001;   //SPLITMODE EN
  // *TCA_INTCTRL = B01110000;
  // TCA0.SINGLE.EVCTRL &= ~(TCA_SINGLE_CNTEI_bm);

}

void PORT_init() {
  *PORTD_DIR = B00011111;
  *PORTD_OUT = B00011111;
}

int move_Servo(int angle) {
  if (angle < -90) angle = -90;
  else if (angle > 90) angle = 90;

  int mov = map(angle, -90, 90, 32, 157);
  return mov;
}


// __ ADC __

#define ADC_RESSEL_10BIT (0x00<<2)
#define ADC_RESSEL_8BIT (0x01<<2)
#define ADC_ENABLE (0x01)
#define ADC_STCONV (0x01)
#define ADC_FREERUN (0x02)
#define PIN0 (1<<0)


int16_t tmp_L_top_offset = 0;
int16_t tmp_R_top_offset = 0;
int16_t tmp_L_bottom_offset = 0;
int16_t tmp_R_bottom_offset = 0;

int16_t L_top_offset = 0;
int16_t R_top_offset = 0;
int16_t L_bottom_offset = 0;
int16_t R_bottom_offset = 0;

int16_t L_top = 0;     // Left top
int16_t L_bottom = 0;  // Left bottom
int16_t R_top = 0;     // Right top
int16_t R_bottom = 0;  // Right bottom

int16_t avg_L_top = 0;     // prev Left top
int16_t avg_L_bottom = 0;  // prev Left bottom
int16_t avg_R_top = 0;     // prev Right top
int16_t avg_R_bottom = 0;  // prev Right bottom

int16_t avg_top;     // average top
int16_t avg_bottom;  // average bottom
int16_t avg_left;    // average left
int16_t avg_right;   // average right

int16_t threshold = 28;

void ADC_init(void) {
  PORTF_PIN2CTRL = 0X04; // PF0 interrupt and digital input buffer disabled
  PORTF_PIN3CTRL = 0X04; // PF1 interrupt and digital input buffer disabled
  PORTF_PIN4CTRL = 0X04; // PF2 interrupt and digital input buffer disabled
  PORTF_PIN5CTRL = 0X04; // PF3 interrupt and digital input buffer disabled

  ADC0_CTRLA |=  ADC_RESSEL_8BIT | ADC_ENABLE | ADC_FREERUN; // 8bit resolution and ADC enabled, FREERUN
  ADC0_CTRLC |= ADC_PRESC_DIV16_gc;
  ADC0_COMMAND |= ADC_STCONV; // Start conversion
}


void ADC_Read_Value() {
  int addr = 0x0C;
  int16_t tmp[4];

  for (int i = 0; i < 4; i++) {
    ADC0_MUXPOS = addr + i;             // A12~ (left top)
    delayMicroseconds(400);
    while (!(ADC0_INTFLAGS & PIN0));
    delayMicroseconds(400);
    tmp[i] = ADC0_RESL;
  }
  delay(100);                     // 가능하면 줄이고 싶음 ( ADC delay 싹다 )


  L_top = tmp[0];
  R_top = tmp[1];
  L_bottom = tmp[2];
  R_bottom = tmp[3];
}

void ADC_calc_AVG(int16_t L_top, int16_t R_top, int16_t L_bottom, int16_t R_bottom) {
  avg_top = (L_top + R_top) / 2;
  avg_bottom = (L_bottom + R_bottom) / 2;
  avg_left = (L_top + L_bottom) / 2;
  avg_right = (R_top + R_bottom) / 2;
}

// __ main_ setup __
void setup() {

  I2C_0_init();
  I2C_setup();
  calcoffset();

  mySerial.begin(115200);

  TCA_reset();
  TCA_init();
  PORT_init();
  *TCA_LCMP0 = move_Servo(0);
  *TCA_LCMP1 = move_Servo(0);
  *TCA_LCMP2 = move_Servo(0);

  ADC_init();
}

// __ main_ loop __
int count = 0;
int mov_vert = 0;
int mov_horiz = 0;
bool mov_vert_flag = false;
bool mov_horiz_flag = false;
int offset_flag = 1;
bool offset_complete_flag = false;

#define  arrsize 10
int16_t tmp1, tmp2, tmp3, tmp4;
int16_t sum1, sum2, sum3, sum4;
int horiz_status = 0;
int vert_status = 0;
//////////////////////////////////////////////////////////////////for drift calibration///////////////////////////////////////////////
int driftnum = 0;
float values[50] = {0};
float diffs[49] = {0};
float drifterror = 0;
float nocalibz = 0;

void loop() {
  //  _____ Read GY-521(MPU6050) via I2C _____

  I2C_0_start(0x68, I2C_DIRECTION_BIT_WRITE);
  I2C_0_writingPacket(0x3B);
  I2C_0_start(0x68, I2C_DIRECTION_BIT_READ);
  raw_Acc_x = (I2C_0_receivingPacket(0) << 8) | I2C_0_receivingPacket(0);
  raw_Acc_y = (I2C_0_receivingPacket(0) << 8) | I2C_0_receivingPacket(0);
  raw_Acc_z = (I2C_0_receivingPacket(0) << 8) | I2C_0_receivingPacket(0);
  raw_temp = (I2C_0_receivingPacket(0) << 8) | I2C_0_receivingPacket(0);
  raw_Gyro_x = (I2C_0_receivingPacket(0) << 8) | I2C_0_receivingPacket(0);
  raw_Gyro_y = (I2C_0_receivingPacket(0) << 8) | I2C_0_receivingPacket(0);
  raw_Gyro_z = (I2C_0_receivingPacket(0) << 8) | I2C_0_receivingPacket(1);

  // raw -> data
  Acc_x = ((float)raw_Acc_x) / 16384.0;
  Acc_y = ((float)raw_Acc_y) / 16384.0;
  Acc_z = ((float)raw_Acc_z) / 16384.0;

  angle_Acc_x = atan2(Acc_y, sqrt(Acc_z * Acc_z + Acc_x * Acc_x)) * 360 / 2.0 / PI;
  angle_Acc_y = atan2(Acc_x, sqrt(Acc_z * Acc_z + Acc_y * Acc_y)) * 360 / -2.0 / PI;

  Gyro_x = ((float)raw_Gyro_x - (Gyro_x_offset)) / 65.5;
  Gyro_y = ((float)raw_Gyro_y - (Gyro_y_offset)) / 65.5;
  Gyro_z = ((float)raw_Gyro_z - (Gyro_z_offset)) / 65.5;

  Interval = (millis() - preInterval) * 0.001;

  angle_Gyro_x += Gyro_x * Interval;
  angle_Gyro_y += Gyro_y * Interval;
  angle_Gyro_z += Gyro_z * Interval;
  //cali_gyro += Gyro_z * Interval;

  angle_x = (GyroCoef * (angle_x + angle_Gyro_x)) + ((1 - GyroCoef) * angle_Acc_x);
  angle_y = (GyroCoef * (angle_y + angle_Gyro_y)) + ((1 - GyroCoef) * angle_Acc_y);
  angle_z = angle_Gyro_z;
  angle_Gyro_z -= drifterror;
  mySerial.print(angle_z);
  mySerial.print(" aftercali ");
  mySerial.println(cali_gyro);
  preInterval = millis();
////////////////////drift calibration///////////////////////////////
  if(driftnum == 0) {
      values[driftnum] = angle_Gyro_z;
      driftnum = 1;
    }
    else if(driftnum >= 1){
      values[driftnum] = angle_Gyro_z;
      diffs[driftnum-1] = values[driftnum] - values[driftnum-1];
      if(driftnum == 50) {
        float sum = 0;
        float avg = 0;
        float avgcal = 0;
        driftnum = 0;
        for(int k = 0; k<49; k++){
          sum = sum + diffs[k];
        }
        avg = sum/49;
        avgcal = (diffs[0]+diffs[49])/2;
        if(((avgcal - avg)*(avgcal - avg))<=0.01){
          mySerial.println("stop moving");
          drifterror = avg;
          mySerial.println(drifterror);
        }
      }
      else driftnum++;
      delay(50);
    }


  //  _____ Read ADC _____
  if (count < arrsize) {
    ADC_Read_Value();

    sum1 += L_top;
    sum2 += R_top;
    sum3 += L_bottom;
    sum4 += R_bottom;

    count++;
  }
  if (count == arrsize) {
    L_top_offset = sum1 / arrsize;
    R_top_offset = sum2 / arrsize;
    L_bottom_offset = sum3 / arrsize;
    R_bottom_offset = sum4 / arrsize;
    offset_complete_flag = 1;
  }
  ADC_Read_Value();


  /*mySerial.print(L_top - L_top_offset);
    mySerial.print(" , ");
    mySerial.println(R_top - R_top_offset);
    //mySerial.print(",");
    mySerial.print(L_bottom - L_bottom_offset);
    mySerial.print(" , ");
    mySerial.println(R_bottom - R_bottom_offset);*/

  /*
    mySerial.print(avg_L_top - L_top_offset);
    mySerial.print(",");
    mySerial.print(avg_R_top - R_top_offset);
    mySerial.print(",");
    mySerial.print(avg_L_bottom - L_bottom_offset);
    mySerial.print(",");
    mySerial.println(avg_R_bottom - R_bottom_offset);*/

  if (offset_complete_flag) {
    vert_status = 0;
    horiz_status = 0;
    int16_t top_avg = ((L_top - L_top_offset) + (R_top - R_top_offset)) / 2;
    int16_t bottom_avg = ((L_bottom - L_bottom_offset) + (R_bottom - R_bottom_offset)) / 2;
    int16_t left_avg = ((L_top - L_top_offset) + (L_bottom - L_bottom_offset)) / 2;
    int16_t right_avg = ((R_top - R_top_offset) + (R_bottom - R_bottom_offset)) / 2;

    bool mov_vert_flag = (abs(top_avg - bottom_avg) > threshold) ? true : false;
    bool mov_horiz_flag = (abs(left_avg - right_avg) > threshold) ? true : false;

    if (mov_vert_flag) {
      if (top_avg > bottom_avg) vert_status = 1;
      else vert_status = -1;
    }
    if (mov_horiz_flag) {
      if (left_avg > right_avg) horiz_status = 1;
      else horiz_status = -1;
    }

    //mySerial.print(vert_status);
    //mySerial.print(" // ");
    //mySerial.println(horiz_status);
  }
  if (horiz_status == 1) {
    fin_z = fin_z - 1;
  }
  else if (horiz_status == -1) {
    fin_z = fin_z + 1;
  }
  else fin_z = fin_z;
  if (vert_status == 1) {
    fin_x = fin_x - 1;
  }
  else if (vert_status == -1) {
    fin_x = fin_x + 1;
  }
  else fin_x = fin_x;

//  mySerial.print(angle_z - fin_z);
//  mySerial.print(",");
//  mySerial.print(angle_x - fin_x);
//  mySerial.print(",");
//  mySerial.println(-angle_y);






  *TCA_LCMP0 = move_Servo(-angle_z + fin_z);
  *TCA_LCMP1 = move_Servo(angle_x + fin_x);
  *TCA_LCMP2 = move_Servo(-angle_y);

}
