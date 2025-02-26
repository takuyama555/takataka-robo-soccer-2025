#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// LCDのI2Cアドレス (通常は0x27または0x3F)
LiquidCrystal_I2C lcd(0x27, 16, 2);

void setup() {
    Serial.begin(115200);  // デバッグ用
    Serial3.begin(115200); // UnitVとの通信
    lcd.init();
    lcd.backlight();
    lcd.setCursor(0, 0);
    lcd.print("Waiting...");
}

void loop() {
    uint8_t header;
    uint16_t left, center, right;

    // シリアルバッファに7バイト以上あるか確認（ヘッダー + 3つの16ビットデータ）
    if (Serial3.available() >= 7) {  
        header = Serial3.read();  // 1バイト受信

        if (header == 254) {  // ヘッダーが正しいか確認
            left = Serial3.read() | (Serial3.read() << 8);   // 2バイト
            center = Serial3.read() | (Serial3.read() << 8); // 2バイト
            right = Serial3.read() | (Serial3.read() << 8);  // 2バイト

            // 受信したデータをシリアルモニタに表示
            Serial.print("L: "); Serial.print(left);
            Serial.print(", C: "); Serial.print(center);
            Serial.print(", R: "); Serial.println(right);

            // LCDに表示
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("L: "); lcd.print(left);
            lcd.setCursor(0, 1);
            lcd.print("C: "); lcd.print(center);
            lcd.print(" R: "); lcd.print(right); // 1行にまとめて表示
        }
    }
}
