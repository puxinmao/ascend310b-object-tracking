"""
串口控制器 — 通过 USB-TTL 将目标角度发送到 STM32。

帧协议（5 字节，带帧头 + 校验和，STM32 端可抵抗丢字节错位）:
    [0xAA] [0x55] [pan] [tilt] [checksum]
    checksum = (pan + tilt) & 0xFF
    pan/tilt 范围 0~180（uint8 可安全承载）。

用法:
    controller = SerialController()
    controller.send_angles(90, 90)  # 发送水平90°, 俯仰90°
    controller.close()

如果串口设备不存在或打开失败，会打印警告并进入无操作模式（不中断跟踪）。
"""

import serial


# ── 串口帧协议常量 ──────────────────────────────────────────────
FRAME_HEAD1 = 0xAA   # 帧头字节 1
FRAME_HEAD2 = 0x55   # 帧头字节 2


class SerialController:
    """通过串口发送舵机角度的控制器（带帧头+校验和的 5 字节协议）。"""

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

        发送 5 字节帧 [0xAA, 0x55, pan, tilt, checksum]：
        STM32 端用状态机按帧头同步，丢一字节也能在下一帧自动重新对齐
        （旧版裸 2 字节协议一旦丢字节就会永久错位）。
        角度会自动钳位到 0~180 范围。

        Args:
            h_angle: 水平角度 0~180（左→右）
            v_angle: 俯仰角度 0~180（上→下）
        """
        if not self._enabled or self.ser is None:
            return

        h = max(0, min(180, int(h_angle)))
        v = max(0, min(180, int(v_angle)))
        checksum = (h + v) & 0xFF
        frame = bytes([FRAME_HEAD1, FRAME_HEAD2, h, v, checksum])
        try:
            self.ser.write(frame)
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
