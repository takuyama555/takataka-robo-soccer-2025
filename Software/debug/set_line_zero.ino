const int SIG_PIN = A4;  // CD74HC4067のCOMピン
const int S0 = 0;  // PA02_A0_D0
const int S1 = 1;  // PA4_A1_D1
const int S2 = 2;  // PA10_A2_D2
const int S3 = 3;  // PA11_A3_D3
const int NUM_CHANNELS = 16;
const int NUM_SAMPLES = 50; // 平均化のためのサンプル数

void setup() {
    Serial.begin(115200);   // PC用のシリアル通信
    Serial1.begin(115200);   // Arduino用のシリアル通信
    pinMode(S0, OUTPUT);
    pinMode(S1, OUTPUT);
    pinMode(S2, OUTPUT);
    pinMode(S3, OUTPUT);
    Serial.println("計算中...");
}

void selectChannel(int channel) {
    digitalWrite(S0, channel & 0x01);
    digitalWrite(S1, (channel >> 1) & 0x01);
    digitalWrite(S2, (channel >> 2) & 0x01);
    digitalWrite(S3, (channel >> 3) & 0x01);
}

const int line_threshold[4] = { 50, 50, 50, 50 }; 

void loop() {
    static int sensorSum[4][4] = {0};
    static int loopCount = 0;
    static bool displayed = false;
    
    if (displayed) {
        delay(2000);  // 2秒待機
        Serial.println("計算中...");
        displayed = false; // 再計測開始
    }

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            int channel = i * 4 + j;
            selectChannel(channel);
            delay(5); // 安定化のための待機
            
            sensorSum[i][j] += analogRead(SIG_PIN);
        }
    }
    
    loopCount++;
    
    if (loopCount >= NUM_SAMPLES) {
        Serial.println("  { " + String(sensorSum[0][0] / NUM_SAMPLES) + "," + String(sensorSum[0][1] / NUM_SAMPLES) + "," + String(sensorSum[0][2] / NUM_SAMPLES) + "," + String(sensorSum[0][3] / NUM_SAMPLES) + " },   // 前");
        Serial.println("  { " + String(sensorSum[1][0] / NUM_SAMPLES) + "," + String(sensorSum[1][1] / NUM_SAMPLES) + "," + String(sensorSum[1][2] / NUM_SAMPLES) + "," + String(sensorSum[1][3] / NUM_SAMPLES) + " },   // 右（外側から内側）");
        Serial.println("  { " + String(sensorSum[2][0] / NUM_SAMPLES) + "," + String(sensorSum[2][1] / NUM_SAMPLES) + "," + String(sensorSum[2][2] / NUM_SAMPLES) + "," + String(sensorSum[2][3] / NUM_SAMPLES) + " },   // 後");
        Serial.println("  { " + String(sensorSum[3][0] / NUM_SAMPLES) + "," + String(sensorSum[3][1] / NUM_SAMPLES) + "," + String(sensorSum[3][2] / NUM_SAMPLES) + "," + String(sensorSum[3][3] / NUM_SAMPLES) + " }   // 左");
        Serial.println("___________________");
        
        // 値をリセット
        memset(sensorSum, 0, sizeof(sensorSum));
        loopCount = 0;
        displayed = true;
    }
    
    delay(10); // ループ間の待機
}
