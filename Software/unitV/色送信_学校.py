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
sensor.set_auto_gain(1,8)         # オートゲインをオフ＆設定
sensor.set_auto_whitebal(False, rgb_gain_db = (80, 60, 90)) # ゲインを設定しておく
sensor.set_brightness(3)  # 明るさを調整
sensor.set_contrast(3)  # コントラストを強化
sensor.set_saturation(3)  # 彩度を増加

sensor.skip_frames(time=2000)

# しきい値
yellow_threshold = (91, 100, -62, 127, 96, 50) #黄色
blue_threshold = (0, 70, -49, -6, -20, 4)   #青

collow_threshold = (0, 0, 0, 0, 0, 0) #最終

# 画像サイズと分割
section_width_left = 140  # 左端の境界
section_width_right = 180  # 右端の境界

while True:
    img = sensor.snapshot()  # 画像取得
    img.rotation_corr(z_rotation=90)  # 画像を90度回転
    # 各色のピクセルカウント
    yellow_pixels, blue_pixels = 0, 0

    # 画像全体のブロブを取得(黄色)
    blobs = img.find_blobs([yellow_threshold], pixels_threshold=10, area_threshold=10)
    for b in blobs:
        yellow_pixels = b.pixels()  # そのブロブのピクセル数

    # 画像全体のブロブを取得(青)
    blobs = img.find_blobs([blue_threshold], pixels_threshold=10, area_threshold=10)
    for b in blobs:
        blue_pixels = b.pixels()  # そのブロブのピクセル数

    # 黄色か青かを判定
    if yellow_pixels > blue_pixels:
        collow_threshold = yellow_threshold
    else:
        collow_threshold = blue_threshold

    # カウント用の変数
    left_count, center_count, right_count = 0, 0, 0

    # 画像全体のブロブを取得(最終的なしきい値で)
    blobs = img.find_blobs([collow_threshold], pixels_threshold=10, area_threshold=10)

    # 各セクションのピクセルカウント
    for b in blobs:
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

    time.sleep_ms(1)

    # UARTからデータを受信する
    if uart.any():
        header = uart.read(1)  # ヘッダー（1バイト）を受信
        if header and header[0] == 254:  # ヘッダーが254であれば送信開始



            # 送信
            # 送信するデータをバイト配列にパック
            buf = bytearray()
            buf.extend(ustruct.pack('B', 254))  # ヘッダー
            buf.extend(ustruct.pack('H', int(left_count)))
            buf.extend(ustruct.pack('H', int(center_count)))
            buf.extend(ustruct.pack('H', int(right_count)))

            # UARTで送信
            uart.write(buf)

            time.sleep_ms(1)
