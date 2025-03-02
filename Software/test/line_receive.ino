void setup() {
    Serial.begin(115200);   // PCとのシリアル通信
    Serial2.begin(115200);  // Serial2ポート（Arduino Mega）での受信設定
    Serial.println("Ready to receive data on Serial2...");
}

void loop() {
line_read();
delay(100);
}
void line_read(){
    int line_detected[4] = {0, 0, 0, 0}; // 各方向でラインがあるかどうか
    int line_flag = 0;
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

            // デバッグ用に受信したデータを表示
            Serial.print("Line detected: ");
            if (line_detected[0]) Serial.print("前 ");
            if (line_detected[1]) Serial.print("右 ");
            if (line_detected[2]) Serial.print("後 ");
            if (line_detected[3]) Serial.print("左 ");
            Serial.print("Line flag: ");
            Serial.println(line_flag);
        }
    }

}