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
sensor.set_auto_whitebal(False, rgb_gain_db=(80, 60, 110))  # ゲインを設定
sensor.set_brightness(3)  # 明るさ調整
sensor.set_contrast(3)  # コントラスト強化
sensor.set_saturation(3)  # 彩度増加
sensor.set_hmirror(True)  # 水平反転
sensor.set_vflip(True)    # 垂直反転

sensor.skip_frames(time=2000)

# 黄色と緑色のしきい値
yellow_threshold = (35, 100, -32, 59, 101, 60)
green_threshold = (26, 73, -42, -10, 25, 55)

# セクションの幅
section_width_left = 140  # 左端の境界
section_width_right = 180  # 右端の境界

while True:
    img = sensor.snapshot()  # 画像取得
    img.rotation_corr(z_rotation=90)

    # 緑色のブロブを取得
    green_blobs = img.find_blobs([green_threshold], pixels_threshold=50, area_threshold=50, merge=True)

    # 最も大きい緑のブロブを取得
    largest_green_blob = max(green_blobs, key=lambda b: b.pixels()) if green_blobs else None

    # 初期化：常に送信するために goal_flag は初期値 0
    goal_flag = 0

    if largest_green_blob:
        # 緑色のブロブの上端（y座標）を取得
        green_top = largest_green_blob.y()

        # 黄色のブロブを取得
        yellow_blobs = img.find_blobs([yellow_threshold], pixels_threshold=50, area_threshold=50, merge=True)

        # 緑に近くて最も大きい黄色ブロブを探す
        closest_largest_yellow_blob = None
        closest_distance = float('inf')

        for b in yellow_blobs:
            distance = abs(green_top - b.cy())  # 緑の上端とのy方向の距離
            if distance < 50:  # 近いものに限定（閾値 50ピクセル）
                if closest_largest_yellow_blob is None or b.pixels() > closest_largest_yellow_blob.pixels():
                    closest_largest_yellow_blob = b
                    closest_distance = distance

        if closest_largest_yellow_blob:
            # 黄色ブロブが見つかった場合
            goal_flag = 1
            goal_height = closest_largest_yellow_blob.h()
            cx = closest_largest_yellow_blob.cx()

            # 各セクションのピクセル数をカウント
            left_count, center_count, right_count = 0, 0, 0
            for b in yellow_blobs:
                b_cx = b.cx()
                pixels = b.pixels()
                if b_cx < section_width_left:
                    left_count += pixels
                elif b_cx < section_width_right:
                    center_count += pixels
                else:
                    right_count += pixels

            # シリアル出力（デバッグ用）
            print("count_L:", left_count, "C:", center_count, "R:", right_count)
            print("yellow Goal Height:", goal_height)
            print("cx:", cx)

            # UARTからデータを受信し、ヘッダーが254なら送信
            if uart.any():
                header = uart.read(1)
                if header and header[0] == 254:
                    buf = bytearray()
                    buf.extend(ustruct.pack('B', 254))  # ヘッダー
                    buf.extend(ustruct.pack('H', left_count))  # 左カウント
                    buf.extend(ustruct.pack('H', center_count))  # 中央カウント
                    buf.extend(ustruct.pack('H', right_count))  # 右カウント
                    buf.extend(ustruct.pack('H', goal_height))  # 高さを送信
                    buf.extend(ustruct.pack('H', cx))  # ｃｘ座標を送信
                    buf.extend(ustruct.pack('B', goal_flag))  # goal_flagを送信
                    uart.write(buf)
    else:
        # 緑色のブロブが検出されなかった場合も goal_flag を送信
        buf = bytearray()
        buf.extend(ustruct.pack('B', 254))  # ヘッダー
        buf.extend(ustruct.pack('H', 0))  # 左カウント
        buf.extend(ustruct.pack('H', 0))  # 中央カウント
        buf.extend(ustruct.pack('H', 0))  # 右カウント
        buf.extend(ustruct.pack('H', 0))  # 高さを送信
        buf.extend(ustruct.pack('H', 0))  # ｃｘを送信
        buf.extend(ustruct.pack('B', goal_flag))  # goal_flagを送信
        uart.write(buf)

    print("flag:", goal_flag)
    time.sleep_ms(10)  # 少し待機して負荷を下げる
