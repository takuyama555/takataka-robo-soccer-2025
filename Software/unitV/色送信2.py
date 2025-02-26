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
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False, rgb_gain_db=(25, 20, 40))
sensor.set_brightness(0)
sensor.set_saturation(0)
sensor.set_contrast(2)
sensor.skip_frames(time=2000)

# しきい値
yellow_threshold = (69, 100, -33, 127, 42, 85) #黄色
blue_threshold = (3, 49, 12, 127, -24, -86)   #青

collow_threshold = (0, 0,0 ,0 , 0, 0) #最終


# 画像サイズと分割
section_width_left = 140  # 左端の境界
section_width_right = 180  # 右端の境界

while True:
    img = sensor.snapshot()  # 画像取得
    # 各色のピクセルカウント
    yellow_pixels,  blue_pixels = 0, 0
    # 画像全体のブロブを取得(黄色)
    blobs = img.find_blobs([yellow_threshold], pixels_threshold=10, area_threshold=10)
    for b in blobs:
        yellow_pixels = b.pixels()  # そのブロブのピクセル数
    # 画像全体のブロブを取得(黄色)
    blobs = img.find_blobs([blue_threshold], pixels_threshold=10, area_threshold=10)
    for b in blobs:
        blue_pixels = b.pixels()  # そのブロブのピクセル数
    if yellow_pixels > blue_pixels:
        [collow_threshold] = [yellow_threshold]
        #print("黄色")
    else:
        [collow_threshold] = [blue_threshold]
        #print("青")


    # 画像全体のブロブを取得(黄色)
    blobs = img.find_blobs([collow_threshold], pixels_threshold=10, area_threshold=10)
    # 各セクションのピクセルカウント
    left_count, center_count, right_count = 0, 0, 0
    # 直接 x 座標で分類
    for b in blobs:
        cx = b.cx()  # ブロブの中心 X 座標
        pixels = b.pixels()  # そのブロブのピクセル数

        if cx < section_width_left:
            left_count += pixels
        elif cx < section_width_right:
            center_count += pixels
        else:
            right_count += pixels
    time.sleep_ms(1)

    # シリアル出力（デバッグ用）
    print("count_L:", left_count, "C:", center_count, "R:", right_count)

    # 送信
    # 各データを16ビット（2バイト）としてパック
    uart.write(ustruct.pack('B',254))#ヘッダー
    uart.write(ustruct.pack('H',int(left_count)))
    uart.write(ustruct.pack('H',int(center_count)))
    uart.write(ustruct.pack('H',int(right_count)))

    time.sleep_ms(1)
