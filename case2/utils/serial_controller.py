"""
串口控制器 — 通过 USB-TTL 将目标角度发送到 STM32。

帧协议（6 字节，带帧头 + 校验和，STM32 端可抵抗丢字节错位）:
    [0xAA] [0x55] [pan] [tilt] [track_id] [checksum]
    checksum = (pan + tilt + track_id) & 0xFF
    pan/tilt 范围 0~180, track_id 0~255（uint8 可安全承载）。

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

    def send_angles(self, h_angle: int, v_angle: int, track_id: int = 0) -> None:
        """
        发送水平/俯仰角度 + 当前跟踪目标 ID。

        发送 6 字节帧 [0xAA, 0x55, pan, tilt, track_id, checksum]：
        checksum = (pan + tilt + track_id) & 0xFF
        STM32 端用状态机按帧头同步，丢一字节也能在下一帧自动重新对齐。
        track_id=0 表示无目标。

        Args:
            h_angle: 水平角度 0~180（左→右）
            v_angle: 俯仰角度 0~180（上→下）
            track_id: 当前跟踪目标 ID（0~255，0=无目标）
        """
        if not self._enabled or self.ser is None:
            return

        h = max(0, min(180, int(h_angle)))
        v = max(0, min(180, int(v_angle)))
        t = max(0, min(255, int(track_id)))
        checksum = (h + v + t) & 0xFF
        frame = bytes([FRAME_HEAD1, FRAME_HEAD2, h, v, t, checksum])
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
