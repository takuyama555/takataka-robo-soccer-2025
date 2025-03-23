import sensor
import image
import time
import machine
import ustruct
from machine import UART
from fpioa_manager import fm

# UnitVのG34 (RX) と G35 (TX) に対応する UART1 を使用
fm.register(35, fm.fpioa.UART1_TX, force=True)
fm.register(34, fm.fpioa.UART1_RX, force=True)
uart = UART(UART.UART1, 115200, 8, 0, 0, timeout=1000, read_buf_len=4096)

# カメラの初期化
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)  # 320x240
sensor.set_auto_gain(1, 8)         # オートゲインをオフ＆設定
sensor.set_auto_whitebal(False, rgb_gain_db=(80, 60, 90))  # ゲインを設定
sensor.set_brightness(3)  # 明るさ調整
sensor.set_contrast(3)  # コントラスト強化
sensor.set_saturation(3)  # 彩度増加
sensor.set_hmirror(True)  # 水平反転
sensor.set_vflip(True)    # 垂直反転

sensor.skip_frames(time=2000)

# 青色と緑色のしきい値
blue_threshold = (24, 79, 81, -69, -115, -18)
green_threshold = (62, 84, -28, -69, 75, 27)

# セクションの幅
section_width_left = 140  # 左端の境界
section_width_right = 180  # 右端の境界

while True:
    img = sensor.snapshot()  # 画像取得
    img.rotation_corr(z_rotation=-90)

    # 緑色のブロブを取得
    green_blobs = img.find_blobs([green_threshold], pixels_threshold=50, area_threshold=50, merge=True)

    # 最も面積が大きい緑のブロブを取得
    largest_green_blob = None
    if green_blobs:
        largest_green_blob = max(green_blobs, key=lambda b: b.pixels())

    if largest_green_blob:
        # 緑のブロブの上端（y_min）を取得
        green_top = largest_green_blob.y()

        # 青色のブロブを取得
        blue_blobs = img.find_blobs([blue_threshold], pixels_threshold=50, area_threshold=50, merge=True)

        # 最も近い青色ブロブを探す
        closest_blue_blob = None
        closest_distance = float('inf')

        for b in blue_blobs:
            # 青色ブロブの中心 (cx, cy)
            blue_center = b.cx(), b.cy()

            # 緑色ブロブの上端と青色ブロブの中心の距離を計算
            distance = abs(green_top - blue_center[1])  # y方向の距離

            # 最も近い青色ブロブを探す
            if distance < closest_distance:
                closest_distance = distance
                closest_blue_blob = b

        if closest_blue_blob:
            # 最も近い青色ブロブの高さを求める
            goal_height = closest_blue_blob.h()

            # カウント用変数の初期化
            left_count, center_count, right_count = 0, 0, 0

            # 有効な青色ブロブの位置に基づき、左・中央・右をカウント
            for b in blue_blobs:
                cx = b.cx()  # ブロブの中心 X 座標
                pixels = b.pixels()  # そのブロブのピクセル数

                # X 座標で分類
                if cx < section_width_left:
                    left_count += pixels
                elif cx < section_width_right:
                    center_count += pixels
                else:
                    right_count += pixels

            # シリアル出力（デバッグ用）
            print("count_L:", left_count, "C:", center_count, "R:", right_count)
            print("Blue Goal Height:", goal_height)

            # UARTからデータを受信する
            if uart.any():
                header = uart.read(1)  # ヘッダー（1バイト）を受信
                if header and header[0] == 254:  # ヘッダーが254であれば送信開始
                    # 送信するデータをバイト配列にパック
                    buf = bytearray()
                    buf.extend(ustruct.pack('B', 254))  # ヘッダー
                    buf.extend(ustruct.pack('H', left_count))  # 左カウント
                    buf.extend(ustruct.pack('H', center_count))  # 中央カウント
                    buf.extend(ustruct.pack('H', right_count))  # 右カウント
                    buf.extend(ustruct.pack('H', int(goal_height)))  # 高さを送信

                    # UARTで送信
                    uart.write(buf)

    time.sleep_ms(10)  # 少し待機して負荷を下げる
