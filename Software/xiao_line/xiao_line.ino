const int SIG_PIN = A4;  // CD74HC4067のCOMピン
const int S0 = 0;  // PA02_A0_D0
const int S1 = 1;  // PA4_A1_D1
const int S2 = 2;  // PA10_A2_D2
const int S3 = 3;  // PA11_A3_D3
const int NUM_CHANNELS = 16;

int line_detected[4] = {0, 0, 0, 0}; // 各方向でラインがあるかどうか
int line_flag = 0;

void setup() {
    Serial.begin(115200);   // PC用のシリアル通信
    Serial1.begin(115200);   // Arduino用のシリアル通信
    pinMode(S0, OUTPUT);
    pinMode(S1, OUTPUT);
    pinMode(S2, OUTPUT);
    pinMode(S3, OUTPUT);
}

void selectChannel(int channel) {
    digitalWrite(S0, channel & 0x01);
    digitalWrite(S1, (channel >> 1) & 0x01);
    digitalWrite(S2, (channel >> 2) & 0x01);
    digitalWrite(S3, (channel >> 3) & 0x01);
}

const int line_zero[4][4] = {
  { 131,119,136,136 },   // 前
  { 129,114,107,74 },   // 右（外側から内側）
  { 109,96,111,115 },   // 後
  { 292,281,444,408 }   // 左
};

const int line_threshold[4] = { 50, 50, 50, 50 }; 

void loop() {
    line_flag = 0;  // フラグをリセット
    for (int i = 0; i < 4; i++) line_detected[i] = 0;

    int sensorValues[16];

    // 16個のセンサーをスキャン
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            int channel = i * 4 + j;
            selectChannel(channel);
            delayMicroseconds(500); // 安定化のための待機時間を短縮
            int sensorValue = analogRead(SIG_PIN) - line_zero[i][j]; 



            sensorValues[channel] = sensorValue;

            // デバッグ出力
            Serial.print("S");
            Serial.print(channel + 1);
            Serial.print(": ");
            Serial.print(sensorValue);
            Serial.print("\t");

            if (sensorValue > line_threshold[i]) {
                if (!line_detected[i]) { 
                    line_detected[i] = 1;
                    line_flag = 1;
                }
            }
        }
        Serial.println(); // 4つごとに改行
    }

    // ライン検出状況を表示
    Serial.print("Line detected: ");
    if (line_detected[0]) Serial.print("前 ");
    if (line_detected[1]) Serial.print("右 ");
    if (line_detected[2]) Serial.print("後 ");
    if (line_detected[3]) Serial.print("左 ");
    Serial.println();

    Serial.print("line flag: ");
    Serial.println(line_flag);
    Serial.println("---");
    // arduinoからデータを受信する
    if (Serial1.available() > 0) {
        byte header = Serial1.read();  // ヘッダー（253）を読み取る

        if (header == 253) {  // ヘッダーが253であれば、データが送信されている
           uart_send();
        }
    }
    // **ここにバッファクリアを追加**
    while (Serial1.available()) {
        Serial1.read();  // 余計なデータを削除
    }   

    delay(1); // 待機時間
}

void uart_send(void) {
    byte buf[5];

    buf[0] = line_detected[0]; // 前
    buf[1] = line_detected[1]; // 右
    buf[2] = line_detected[2]; // 後
    buf[3] = line_detected[3]; // 左
    buf[4] = line_flag; // flag

    Serial1.write(253);  // ヘッダー
    Serial1.write(buf, 5);  // 配列全体を送信
    Serial1.flush(); // 送信完了を待つ
}
