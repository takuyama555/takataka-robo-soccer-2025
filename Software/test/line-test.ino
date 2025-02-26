const int line_zero[4][4] = {
  { 100, 100, 110, 80 },   // 右
  { 29, 13, 28, 150 }, // 前 外側から内側
  { 38, 9, 9, 23 },    // 左
  { 16, 15, 20, 21 }   // 後
};
const int line[4][4] = {
  { 0, 1, 2, 3 },      // 前 外側から内側
  { 4, 5, 6, 7 },      // 左
  { 8, 9, 10, 11 },    // 右
  { 12, 13, 14, 15 }   // 後
};

int line_flag = 0;
int print[4]; // 前だけのセンサー値を保存する配列

// 閾値 (line_threshold) を仮に設定
const int line_threshold = 50; // 前センサー用の閾値

double Line_cal_front() {
  line_flag = 0;

  // 前のセンサーの値を計算
  for (int j = 0; j < 4; j++) {
    double value = 1023 - analogRead(line[0][j]) - line_zero[0][j];
    print[j] = value; // 値を保存

    // 閾値を超えるかどうかを確認
    if (value > line_threshold) {
      line_flag = 1; // フラグを立てる
    }
  }

  // シリアルモニターに結果を表示
  Serial.println("Front Sensor Values:");
  for (int j = 0; j < 4; j++) {
    Serial.print("Sensor ");
    Serial.print(j);
    Serial.print(": ");
    Serial.println(print[j]);
  }
  Serial.print("Line Flag: ");
  Serial.println(line_flag);
  Serial.println("--------------------");

  return 0; // 必要なら他の値を返すように変更
}

void setup() {
  Serial.begin(9600); // シリアル通信を初期化
}

void loop() {
  Line_cal_front(); // 前センサー値を計算して表示
  delay(5);       // 0.5秒ごとに更新
}
