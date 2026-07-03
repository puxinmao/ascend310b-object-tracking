"""
serial_monitor.py — 实时监听串口输出的水平/俯仰角度。

用法:
    python utils/serial_monitor.py

需要先装 pyserial:  pip install pyserial --user

按 Ctrl+C 退出。
"""

import serial
import time


def main():
    port = "/dev/ttyUSB0"
    baud = 115200

    try:
        ser = serial.Serial(port, baud, timeout=0.1)
    except Exception as e:
        print(f"无法打开串口 {port}: {e}")
        return

    print(f"串口已连接: {port} @ {baud} baud")
    print("等待接收数据...")
    print("-" * 40)

    buf = []  # 接收缓冲区
    last_print = 0

    try:
        while True:
            data = ser.read(1)
            if data:
                buf.append(data[0])

                # 每收到完整 2 字节就显示
                if len(buf) >= 2:
                    h_angle = buf[0]
                    v_angle = buf[1]
                    buf = []

                    now = time.time()
                    if now - last_print >= 0.03:  # 限制刷新率
                        bar_h = "█" * (h_angle // 5) + "░" * ((180 - h_angle) // 5)
                        bar_v = "█" * (v_angle // 5) + "░" * ((180 - v_angle) // 5)

                        print(f"\r水平: {h_angle:3d}° {bar_h}", end="")
                        print(f"  |  俯仰: {v_angle:3d}° {bar_v}", end="")
                        last_print = now

            else:
                # 超时无数据，显示等待
                print("\r等待数据...", end="")

    except KeyboardInterrupt:
        print("\n\n退出。")
    finally:
        ser.close()


if __name__ == "__main__":
    main()
