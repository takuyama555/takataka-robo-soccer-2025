#define normal_speed 50  //移動速度
#define gryo_p 1.1

#define wrap_forward 0  //5  //回り込み速度
#define wrap_side 0     //25
#define wrap_back 0     //30

#define line_p 55                                  //ラインの速さ
const int line_threshold[4] = { 40, 40, 40, 40 };  //ライン判定の基準
//const int line_threshold[4] = { 999, 999, 999,999 };

double print[30];
/*
0-17:line_val,flag,counter
18-
*/
///////////definiton_motordriver/////////////
const int dir[] = { 11, 6, 10, 7 };  //モーター0～3のpwmとdirの配列
const int pwm[] = { 9, 4, 8, 5 };
#define pi 3.1415  //円周率


const int line_zero[4][4] = {
  //ラインセンサーの値をデフォルトで０にする
  { 0, 28, 45, 18 },   //右
  { 29, 13, 28, 150 },  //前　外側から内側
  { 38, 9, 9, 23 },     //左
  { 16, 15, 20, 21 }   /*後*/
};
const int line[4][4] = {
  //ラインセンサーのぴん番号
  { 0, 1, 2, 3 },    //前　外側から内側
  { 4, 5, 6, 7 },    //左
  { 8, 9, 10, 11 },  //右
  { 12, 13, 14, 15 } /*後*/
};

int line_flag = 0;
double Line_cal() {
  double line_power[4];
  line_flag = 0;
  int line_counter = 0;

  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      double value = analogRead(line[i][j]) - line_zero[i][j];
      print[(i * 4) + j] = value;
      if (value > line_threshold[i]) {
        line_flag = 1;
        line_counter = i;
        break;
      }
    }
  }
  print[16] = line_flag;
  print[17] = line_counter;

  if (line_counter == 0) {  //左に避ける
    line_power[0] = 128 + line_p;
    line_power[1] = 128 + line_p;
    line_power[2] = 128 - line_p;
    line_power[3] = 128 - line_p;
  } else if (line_counter == 1) {  //下に避ける
    line_power[0] = 128 - line_p;
    line_power[1] = 128 + line_p;
    line_power[2] = 128 + line_p;
    line_power[3] = 128 - line_p;
  } else if (line_counter == 2) {  //右に避ける
    line_power[0] = 128 - line_p;
    line_power[1] = 128 - line_p;
    line_power[2] = 128 + line_p;
    line_power[3] = 128 + line_p;
  } else if (line_counter == 3) {  //上に避ける
    line_power[0] = 128 + line_p;
    line_power[1] = 128 - line_p;
    line_power[2] = 128 - line_p;
    line_power[3] = 128 + line_p;
  }
  Drive_Motor(line_power);
}

////////////////////////method_motor//////////////////////
void Drive_Motor(double power[]) {
  for (int i = 0; i < 4; i++) {
    digitalWrite(pwm[i], HIGH);
    if (i == 0) {
      analogWrite(dir[i], 256 - power[i]);  //反転修正
    } else {
      analogWrite(dir[i], power[i]);
    }
  }
}
//それぞれのモーターの出力を計算
double wrap[4];
double Cal_power(double degree, double speed, double gyro_value) {
  double power1[4]={0,0,0,0};
  double angle[4];
  double maxvalue = 0;  //power[]の一番大きい値
  double e=0;
  if (abs(degree) <= 365) {
    //進行角度（度数）,進行する速さ(0~128まで)
    if (degree >= 60 && degree <= 120) {
      wrap[0] = wrap_forward;
      wrap[1] = wrap_forward;
      wrap[2] = wrap_forward;
      wrap[3] = wrap_forward;

    } else if ((degree >= 30 && degree < 60) || (degree >= 240 && degree <= 270)) {
      wrap[0] = -wrap_side;
      wrap[1] = -wrap_side;
      wrap[2] = wrap_side;
      wrap[3] = wrap_side;

    } else if ((degree > 120 && degree <= 150) || (degree > 270 && degree <= 300)) {
      wrap[0] = wrap_side;
      wrap[1] = wrap_side;
      wrap[2] = -wrap_side;
      wrap[3] = -wrap_side;

    } else {  //-60～30,150-240
      wrap[0] = -wrap_back;
      wrap[1] = wrap_back;
      wrap[2] = wrap_back;
      wrap[3] = -wrap_back;
    }
    //double line_power[4];

    for (int i = 0; i < 4; i++) {
      angle[i] = (degree - (45 + 90 * i)) * pi / 180;
      power1[i] = sin(angle[i]);

      //Serial.print(i);
      //Serial.print(angle[i]*180/pi);
      //Serial.print(" ");
      //Serial.print(power1[i]);
      //Serial.print(" , ");

      if (abs(maxvalue) < abs(power1[i])) {
        maxvalue = power1[i];
      }
    }

    e = speed / abs(maxvalue);  //一番強いモーターの出力＝speedにする
  }
  for (int i = 0; i < 4; i++) {
    power1[i] = 128 + power1[i] * e + gyro_value + wrap[i];
    //Serial.print(i);
    print[20 + i] = power1[i];
  }
  Drive_Motor(power1);
  //if(speed==0){Stop();}
}

#include "MPU6050_6Axis_MotionApps20.h"
MPU6050 mpu;
// MPU control/status vars
uint8_t mpuIntStatus;    // holds actual interrupt status byte from MPU
uint8_t devStatus;       // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;     // expected DMP packet size (default is 42 bytes)
uint8_t fifoBuffer[64];  // FIFO storage buffer

// orientation/motion vars
Quaternion q;         // [w, x, y, z]         quaternion container
VectorFloat gravity;  // [x, y, z]            gravity vector
float ypr[3];         // [roll, pitch, yaw]   roll/pitch/yaw container and gravity vector

void setupMPU() {
  Wire.begin();
  Wire.setClock(400000);  // 400kHz I2C clock. Comment this line if having compilation difficulties
  mpu.initialize();
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXAccelOffset(-2759);
  mpu.setYAccelOffset(-1623);
  mpu.setZAccelOffset(897);
  mpu.setXGyroOffset(239);
  mpu.setYGyroOffset(7);
  mpu.setZGyroOffset(-2);

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.setDMPEnabled(true);
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    Serial.print("DMP Initialization failed.");
  }
}

int getYawPitchRoll() {
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {  // Get the Latest packet
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    //Serial.print(ypr[2] * 180 / M_PI);
    //Serial.print(",");
    //Serial.print(ypr[1] * 180 / M_PI);
    //Serial.print(",");
    double mpu_degree = ypr[0] * 180 / M_PI;
    print[18] = mpu_degree;
    return mpu_degree;
  }
}

// 定数の宣言
#define SERIAL_BAUD 115200
#define SERIAL1_BAUD 115200
//#define SERIAL_BAUD 115200
#define IR_BAUD 115200
// インスタンスの生成
//SerialPIO ir(2, 3, 32);
// 赤外線センサー制御系
double ir_angle = 0;  // 赤外線センサーの角度（-PI ~ PI）
double abs_ir_angle = 0;
uint8_t ir_dist = 0;      // 赤外線センサーの距離（0~254[cm]）
uint8_t ir_flag = 0;      // 0: normal, 1: stop (0~254)
void ir_uart_recv(void);  // 赤外線センサーの制御系の受信関数
double normalize_angle(double angle);

void ir_uart_recv(void) {
  Serial1.write(255);  // ヘッダー
  unsigned long long request_time = micros();
  while (!Serial1.available()) {
    if (micros() - request_time > 10000) {
      break;
    }
  }
  byte header = Serial1.read();
  if (header != 255)
    return;
  // 角度を-πからπまでに調整
  unsigned long long wait_time = micros();
  while (Serial1.available() < 4) {
    if (micros() - wait_time > 10000) {
      while (Serial1.available()) {
        Serial1.read();
        //Serial.println("a");
      }
      break;
    }
  }
  byte buf[4];
  Serial1.readBytes(buf, 4);
  if (buf[0] != 255 && buf[1] != 255 && buf[2] != 255 && buf[3] != 255) {
    ir_flag = buf[0];  // 0: normal, 1: stop (0~254)
    //ir_angle = (buf[1] + buf[2] * 128) / 100.0 - PI; // 0 ~ 200PI -> -PI ~ PI
    ir_angle = ((buf[1] + buf[2] * 128) / 100.0) * 180 / PI;  // 0 ~ 200PI -> -PI ~ PI
    //ir_angle = buf[1] + buf[2];
    ir_dist = buf[3];
    //Serial.println("b"); // 0~254[cm]
  }

  //修正
  //abs_ir_angle = ir_angle + gyro.angle;
  //abs_ir_angle = normalize_angle(abs_ir_angle);
}

double normalize_angle(double angle) {
  while (angle > PI) {
    angle -= TWO_PI;
  }
  while (angle <= -PI) {
    angle += TWO_PI;
  }
  return angle;
};

////////////////////////main_code///////////////////////////////////////
void setup() {
  Serial.begin(SERIAL_BAUD);
  //ir.begin(IR_BAUD);
  Serial1.begin(IR_BAUD);
  //pwmの周波数を最速に（基本Timer0は変えない）
  //TCCR0B = (TCCR0B & 0b11111000) | 0x01; //62.5 [kHz]  Timer0
  TCCR1B = (TCCR1B & 0b11111000) | 0x01;  //31.37255 [kHz]  Timer1 pin 11,12
  TCCR2B = (TCCR2B & 0b11111000) | 0x01;  //31.37255 [kHz]  Timer2 pin 9,10
  TCCR3B = (TCCR3B & 0b11111000) | 0x01;  //31.37255 [kHz]  Timer3 pin 2,3,5
  TCCR4B = (TCCR4B & 0b11111000) | 0x01;  //31.37255 [kHz]  Timer4 pin 6,7,8
  while (!Serial1);
  Serial.println("aaaa");

  setupMPU();  //mpuの初期化
}

void loop() {

  //Line_cal();
  if (line_flag == 0) {
    double gryo_val = getYawPitchRoll() * gryo_p;
    ir_uart_recv();
    ir_angle = ir_angle;
    print[19] = ir_angle;

    Cal_power(ir_angle, normal_speed, gryo_val);

    for (int i = 0; i < 24; i++) {
      switch (i) {
        case 0:
          Serial.print("L_val:");
          break;
        case 16:
          Serial.print("flag: ");
          break;
        case 17:
          Serial.print("L_num: ");
          break;
        case 18:
          Serial.print("gyro: ");
          break;
        case 19:
          Serial.print("Ir: ");
          break;
        case 20:
          Serial.print("power: ");
          break;
      }
      Serial.print(print[i]);
      Serial.print(" , ");
    }
  }
  Serial.println(" ");
  delay(50);
}