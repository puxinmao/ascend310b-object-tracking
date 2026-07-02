"""
串口控制器 — 通过 USB-TTL 将目标角度发送到 STM32。

用法:
    controller = SerialController()
    controller.send_angle(90)    # 发送 90°
    controller.close()

如果串口设备不存在或打开失败，会打印警告并进入无操作模式（不中断跟踪）。
"""

import serial


class SerialController:
    """通过串口发送舵机角度的控制器。"""

    def __init__(self, port: str = "/dev/ttyUSB0", baud: int = 115200, timeout: float = 0.01):
        """
        初始化串口控制器。

        Args:
            port: 串口设备路径，默认 /dev/ttyUSB0
            baud: 波特率，默认 115200
            timeout: 超时秒数
        """
        self.port = port
        self.baud = baud
        self.ser = None
        self._enabled = False

        try:
            self.ser = serial.Serial(port, baud, timeout=timeout)
            if self.ser.is_open:
                self._enabled = True
                print(f"[SerialController] 串口已打开: {port} @ {baud} baud")
        except Exception as exc:
            print(f"[SerialController] 警告: 无法打开串口 {port}: {exc}")
            print("[SerialController] 舵机控制已禁用，跟踪功能正常运行。")

    def send_angle(self, angle: int) -> None:
        """
        发送舵机角度 (0~180)。

        只发送单字节，丢帧不影响（下一帧会补发）。
        角度会自动钳位到 0~180 范围。

        Args:
            angle: 目标角度 0~180
        """
        if not self._enabled or self.ser is None:
            return

        angle = max(0, min(180, int(angle)))
        try:
            self.ser.write(bytes([angle]))
        except Exception as exc:
            print(f"[SerialController] 发送失败: {exc}")

    def close(self) -> None:
        """关闭串口。"""
        if self.ser is not None and self.ser.is_open:
            try:
                self.ser.close()
                print("[SerialController] 串口已关闭。")
            except Exception:
                pass
        self._enabled = False

    def __del__(self):
        self.close()
