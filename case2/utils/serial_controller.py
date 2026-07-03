"""
串口控制器 — 通过 USB-TTL 将目标角度发送到 STM32。

用法:
    controller = SerialController()
    controller.send_angles(90, 90)  # 发送水平90°, 俯仰90°
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

    def send_angles(self, h_angle: int, v_angle: int) -> None:
        """
        发送水平和俯仰角度 (各 0~180)。

        发送 2 字节：[水平, 俯仰]，丢帧不影响（下一帧会补发）。
        角度会自动钳位到 0~180 范围。

        Args:
            h_angle: 水平角度 0~180（左→右）
            v_angle: 俯仰角度 0~180（上→下）
        """
        if not self._enabled or self.ser is None:
            return

        h_angle = max(0, min(180, int(h_angle)))
        v_angle = max(0, min(180, int(v_angle)))
        try:
            self.ser.write(bytes([h_angle, v_angle]))
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
