#define normal_speed 45  //移動速度 max speed
#define gryo_p 1.1
#define wrap_forward 0                             //5 //回り込み速度
#define wrap_side 0                                //25
#define wrap_back 0                                //30
#define line_p 55                                  //ラインの速さ
const int line_threshold[4] = { 50, 50, 50, 50 };  //ライン判定の基準
//const int line_threshold[4] = { 999, 999, 999,999 };
double print[30];
/*
0-17:line_val,flag,counter
18-
*/

///カメラ関連///
int color_angle = 0;
int color_previous = 0;
const int HISTORY_SIZE = 5; // 保存する履歴のサイズ
int color_history[HISTORY_SIZE] = {0}; // 過去のcolor_angleを保存する配列
int move_angle = 0; // 進行方向の角度
int history_index = 0; // 配列の挿入位置を管理


void camera() {
  uint8_t header;
  uint16_t left, center, right;
   Serial3.write(254); // ヘッダー送信
    delay(10); // 少し待ってからデータを受信（XIAOの処理時間確保）

  // シリアルバッファに7バイト以上あるか確認（ヘッダー + 3つの16ビットデータ）
  if (Serial3.available() >= 7) {
    header = Serial3.read();  // 1バイト受信
    if (header == 254) {                                // ヘッダーが正しいか確認
      left = Serial3.read() | (Serial3.read() << 8);    // 2バイト
      center = Serial3.read() | (Serial3.read() << 8);  // 2バイト
      right = Serial3.read() | (Serial3.read() << 8);   // 2バイト
      print[27] = left;
      print[28] = center;
      print[29] = right;

      // // 受信したデータをシリアルモニタに表示
      //  Serial.print("L: "); Serial.print(left);
      //  Serial.print(", C: "); Serial.print(center);
      //  Serial.print(", R: "); Serial.println(right);


      ////色からの方向決定///
      if (left > center && left > right) {  ///左が最大
        color_angle = 275;
      } else if (center > left && center > right) {  //中央が最大
        color_angle = 0;
      } else if (right > left && right > center) {  //右が最大
        color_angle = 45;
      } else {
        color_angle = 0;
      }
    }
  }else{
    color_angle = 0;
  }
  print[26] = color_angle;
}



// 回り込みのための計算式の係数
#define CIRC_BASE pow(0.6, 1.0 / 20.0)
#define CIRC_WEIGHT 1
int speed = 0;
int STRAIGHT_SPEED = 50;
int CIRC_SPEED = 50;
int ir_invert_angle = 0;
int move_invert_angle = 0;

///シリアルプリント関連

int game_mode = 0;  ///デバッグON---0 デバッグOFF---1

///////ボタン関連
const int buttonOn_Pin = 32;   // ラインセンサをONにするボタン
const int buttonOff_Pin = 33;  // ラインセンサをOFFにするボタン
bool game_flag = 0;            //ある角度以上になったらゲーム開始
int game_start = 0;            //ボタンが押されたら1に


///////////definiton_motordriver/////////////
const int dir[] = {10,6, 7, 11}; //モーター0～3のpwmとdirの配列
const int pwm[] = {8,4, 5, 9};
int move_agle = 0;


#define pi 3.1415  //円周率
const int line_zero[4][4] = {
  { 75, 53, 50, 24 },  // 前
  { 55, 65, 40, 15 },  // 右 外側から内側
  { 45, 60, 70, 40 },  // 後
  { 35, 60, 60, 55 }   // 左
};
const int line[4][4] = {
  { 0, 1, 2, 3 },     // 前 外側から内側
  { 4, 5, 6, 7 },     // 右
  { 8, 9, 10, 11 },   // 後ろ
  { 12, 13, 14, 15 }  // 左
};
int line_flag = 0;
int line_counter = 0;

void Line_Read() {
    int line_detected[4] = {0, 0, 0, 0}; // 各方向でラインがあるかどうか
    line_flag = 0;
    line_counter = 0;
    Serial2.write(253); // ヘッダー送信
    delay(10); // 少し待ってからデータを受信（XIAOの処理時間確保）
    // Serial2からデータを受信する
    if (Serial2.available() > 0) {
        byte header = Serial2.read();  // ヘッダー（253）を読み取る

        if (header == 253) {  // ヘッダーが253であれば、データが送信されている
            byte buf[5];
            Serial2.readBytes(buf, 5); // Serial2から5バイトのデータを読み取る

            line_detected[0] = buf[0]; // 前
            line_detected[1] = buf[1]; // 右
            line_detected[2] = buf[2]; // 後
            line_detected[3] = buf[3]; // 左
            line_flag = buf[4]; // 0: normal, 1: stop (0~254)
            if (line_detected[0] == 1){
            line_counter = 1;}
            if (line_detected[1] == 1){
            line_counter = 2;}
            if (line_detected[2] == 1){
            line_counter = 3;}
            if (line_detected[3] == 1){
            line_counter = 4;}
            print[16] = line_flag;
            print[17] = line_counter;

            // // デバッグ用に受信したデータを表示
            // Serial.print("Line detected: ");
            // if (line_detected[0]) Serial.print("前 ");
            // if (line_detected[1]) Serial.print("右 ");
            // if (line_detected[2]) Serial.print("後 ");
            // if (line_detected[3]) Serial.print("左 ");
            // Serial.print("Line flag: ");
            // Serial.println(line_flag);
        }
    }

}
////////////////////////method_motor//////////////////////
void Drive_Motor(double power[]) {
  for (int i = 0; i < 4; i++) {
    pinMode(dir[i], OUTPUT);
    pinMode(pwm[i], OUTPUT);
  }
  for (int i = 0; i < 4; i++) {
    digitalWrite(pwm[i], HIGH);
    if (i == 0) {
      analogWrite(dir[i], 256 - power[i]);  //反転修正
    } else if (i == 2) {
      analogWrite(dir[i], 256 - power[i]);  //反転修正
    } else {
      analogWrite(dir[i], power[i]);
    }
  }
}
//それぞれのモーターの出力を計算
double wrap[4];
double Cal_power(double degree, double speed, double gyro_value) {
  double power1[4] = { 0, 0, 0, 0 };
  double angle[4];
  double maxvalue = 0;  //power[]の一番大きい値
  double e = 0;
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
    print[22 + i] = power1[i];
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
Quaternion q;         // [w, x, y, z]    quaternion container
VectorFloat gravity;  // [x, y, z]      gravity vector
float ypr[3];         // [roll, pitch, yaw] roll/pitch/yaw container and gravity vector
void setupMPU() {
  Wire.begin();
  Wire.setClock(400000);  // 400kHz I2C clock. Comment this line if having compilation difficulties
  mpu.initialize();
  devStatus = mpu.dmpInitialize();
  // supply your own gyro offsets here, scaled for min sensitivity
  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXAccelOffset(489);
  mpu.setYAccelOffset(-284);
  mpu.setZAccelOffset(5168);
  mpu.setXGyroOffset(-3);
  mpu.setYGyroOffset(-22);
  mpu.setZGyroOffset(28);
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
  Serial2.begin(115200);
  Serial3.begin(115200);  // UnitVとの通信
  //pwmの周波数を最速に（基本Timer0は変えない）
  //TCCR0B = (TCCR0B & 0b11111000) | 0x01; //62.5 [kHz] Timer0
  TCCR1B = (TCCR1B & 0b11111000) | 0x01;  //31.37255 [kHz] Timer1 pin 11,12
  TCCR2B = (TCCR2B & 0b11111000) | 0x01;  //31.37255 [kHz] Timer2 pin 9,10
  TCCR3B = (TCCR3B & 0b11111000) | 0x01;  //31.37255 [kHz] Timer3 pin 2,3,5
  TCCR4B = (TCCR4B & 0b11111000) | 0x01;  //31.37255 [kHz] Timer4 pin 6,7,8
  while (!Serial1)
    ;
  Serial.println("aaaa");
  setupMPU();                            //mpuの初期化
                                         ///////ボタン関連
  pinMode(buttonOn_Pin, INPUT_PULLUP);   // ボタン1 (ON) をプルアップ設定
  pinMode(buttonOff_Pin, INPUT_PULLUP);  // ボタン2 (OFF) をプルアップ設定
}



//////////////////////////////////////////////////////////////////////////////////////メインループ////////////////////////////////////////////////////////////////////////////////////////

void loop() {
  double gryo_val = getYawPitchRoll() * gryo_p * -1;
  Serial.print(gryo_val);
  Serial.println(" ");
  while (game_flag == 1) {
    double gryo_val = getYawPitchRoll() * gryo_p  * -1 ;

    ir_uart_recv();  //IRセンサ読み取り関数
    ir_angle = ir_angle;
    print[19] = ir_angle;
    print[20] = ir_flag;
    print[21] = ir_dist;
    Line_Read();  ///ラインセンサ読み取り関数
    if (line_flag == 0) {

      if (ir_flag == 1) {
        int move_angle = 0;

        
        /////回り込み////
        if (ir_dist < 60) {
          if (ir_angle >= 0 && ir_angle <= 20 || ir_angle >= 340 && ir_angle <= 360) {
            camera();
             // color_history に最新の color_angle を追加（リングバッファ方式）
            color_history[history_index] = color_angle;
            history_index = (history_index + 1) % HISTORY_SIZE; // インデックスを循環させる

            // color_history 内で最も多く出現した color_angle を求める
            int maxCount = 0;
            int mostFrequentAngle = color_angle;

            // 各色の出現回数をカウント
            for (int i = 0; i < HISTORY_SIZE; i++) {
                int currentAngle = color_history[i];
                int count = 0;

                // 同じ値の出現回数をカウント
                for (int j = 0; j < HISTORY_SIZE; j++) {
                    if (color_history[j] == currentAngle) {
                        count++;
                    }
                }

                // 一番多く出現した値を記録
                if (count > maxCount) {
                    maxCount = count;
                    mostFrequentAngle = currentAngle;
                }
            }

            move_angle = mostFrequentAngle;
            speed = 50;


          } else {
            if (ir_angle >= 180 && ir_angle < 340) {
              float circ_exp = pow(CIRC_BASE, ir_dist);  // 回り込みのための計算式．これがどうして導かれたかは謎．誰ももう覚えてない．
              // 計算で得られた角度が不適切な場合があるので，移動する方向を制限する．
              ir_invert_angle = ir_angle - 360;
              move_invert_angle = (ir_invert_angle + constrain(ir_invert_angle * CIRC_WEIGHT, -90, 90));
              move_angle = 360 + move_invert_angle;
              speed = 70;
            } else if (ir_angle > 30 && ir_angle < 180) {
              float circ_exp = pow(CIRC_BASE, ir_dist);  // 回り込みのための計算式．これがどうして導かれたかは謎．誰ももう覚えてない．
              // 計算で得られた角度が不適切な場合があるので，移動する方向を制限する．
              move_angle = ir_angle + constrain(ir_angle * CIRC_WEIGHT, -90, 90);
              speed = 70;
            }
          }
          
        } else {
          move_angle = ir_angle;
        }
        Serial.print(move_angle);
        Serial.println(" ");

        Cal_power(move_angle, speed, gryo_val);
      } else {  ///IRフラグがゼロの時はジャイロだけ動作させる
        Cal_power(ir_angle, 0, gryo_val);
      }
    } else if (line_flag == 1) {
      if (line_counter == 1) {
        Cal_power(180, normal_speed , gryo_val);
        // delay(100);

      } else if (line_counter == 2) {
        Cal_power(270, normal_speed, gryo_val);
      } else if (line_counter == 3) {
        Cal_power(0, normal_speed, gryo_val);
      } else if (line_counter == 4) {
        Cal_power(90, normal_speed, gryo_val);
      }
    }



    if (game_mode == 0) {
      for (int i = 0; i < 30; i++) {
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
            Serial.print("Ir_flag: ");
            break;
          case 21:
            Serial.print("Ir_dist ");
            break;
          case 22:
            Serial.print("power ");
            break;
          case 26:
            Serial.print("color_angle ");
            break;
          case 27:
            Serial.print("camera_left  ");
            break;
          case 28:
            Serial.print("camera_center ");
            break;
          case 29:
            Serial.print("camera_right ");
            break;
        }
        Serial.print(print[i]);
        Serial.print(" , ");
      }
      Serial.println(" ");
      delay(5);
    }

    if (digitalRead(buttonOff_Pin) == LOW) {
      int cnt = 0;
      while (1) {
        if (digitalRead(buttonOff_Pin) == LOW) {
          cnt++;
          if (cnt == 80) {  // ボタンが十分な時間押されているかを確認（チャタリング対策）

            break;
          }
        } else {
          cnt = 0;
          break;
        }
        delay(2);
      }
      if (cnt == 80) {
        game_start = 0;
        game_flag = 0;
        Cal_power(0, 0, 0);
        Serial.println("Game Stop");
      }
    }


  }  ///




  if (digitalRead(buttonOn_Pin) == LOW) {
    int cnt = 0;

    while (1) {
      if (digitalRead(buttonOn_Pin) == LOW) {
        cnt++;
        if (cnt == 80) {  // ボタンが十分な時間押されているかを確認（チャタリング対策）
          break;
        }
      } else {
        cnt = 0;
        break;
      }
      delay(2);
    }
    if (cnt == 80) {

      game_start = 1;
      Serial.println("ON");
    }
  }
  if (game_start == 1) {

    if (90 <= gryo_val && gryo_val <= 200 || -90 <= gryo_val && gryo_val <= -200) {
      Cal_power(0, 0, -30);
    } else {
      game_start = 0;
      game_flag = 1;
      Serial.println("Game start!");
    }
  }
}