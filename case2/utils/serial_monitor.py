"""
serial_monitor.py — 实时监听串口上的舵机角度帧（带帧头同步解析）。

用法:
    python utils/serial_monitor.py
    python utils/serial_monitor.py --port COM3 --baud 115200

需要先装 pyserial:  pip install pyserial --user
按 Ctrl+C 退出。

帧协议（与 serial_controller.py 一致）:
    [0xAA] [0x55] [pan] [tilt] [checksum], checksum = (pan+tilt) & 0xFF
"""

import argparse
import serial
import time


FRAME_HEAD1 = 0xAA
FRAME_HEAD2 = 0x55


def parse_args():
    parser = argparse.ArgumentParser(description="Monitor servo angle frames on the serial line.")
    parser.add_argument("--port", default="/dev/ttyUSB0", help="Serial port (default: /dev/ttyUSB0).")
    parser.add_argument("--baud", type=int, default=115200, help="Baudrate (default: 115200).")
    return parser.parse_args()


def main():
    args = parse_args()
    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.1)
    except Exception as e:
        print(f"无法打开串口 {args.port}: {e}")
        return

    print(f"串口已连接: {args.port} @ {args.baud} baud")
    print("等待接收数据...")
    print("-" * 40)

    # 帧解析状态机：HEAD1 → HEAD2 → PAN → TILT → CHK
    state = "HEAD1"
    pan = tilt = 0
    last_print = 0.0
    frames = 0
    drops = 0

    try:
        while True:
            data = ser.read(1)
            if not data:
                print("\r等待数据...", end="")
                continue

            b = data[0]
            if state == "HEAD1":
                if b == FRAME_HEAD1:
                    state = "HEAD2"
            elif state == "HEAD2":
                if b == FRAME_HEAD2:
                    state = "PAN"
                elif b == FRAME_HEAD1:
                    state = "HEAD2"      # 连续 0xAA，继续等 0x55
                else:
                    state = "HEAD1"
            elif state == "PAN":
                pan = b
                state = "TILT"
            elif state == "TILT":
                tilt = b
                state = "CHK"
            elif state == "CHK":
                expected = (pan + tilt) & 0xFF
                if b == expected:
                    frames += 1
                    now = time.time()
                    if now - last_print >= 0.03:   # 限制刷新率 ~33Hz
                        bar_h = "█" * (pan // 5) + "░" * ((180 - pan) // 5)
                        bar_v = "█" * (tilt // 5) + "░" * ((180 - tilt) // 5)
                        print(f"\r水平: {pan:3d}° {bar_h}  |  俯仰: {tilt:3d}° {bar_v}"
                              f"  [帧:{frames} 丢:{drops}]", end="")
                        last_print = now
                else:
                    drops += 1
                state = "HEAD1"
    except KeyboardInterrupt:
        print(f"\n\n退出。共收到 {frames} 个有效帧，丢弃 {drops} 个坏帧。")
    finally:
        ser.close()


if __name__ == "__main__":
    main()
