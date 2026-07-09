"""
hand_gesture.py — 基于 MediaPipe Hands 的手势检测(握拳 / V字 / 张开)。

用法:
    detector = GestureDetector()
    hands = detector.detect_all(frame)
    # hands = [(gesture, (wx, wy), fingers_up), ...]
    #   gesture:    'fist'(握拳) / 'victory'(V字) / 'open'(张开) / 'unknown'
    #   (wx, wy):   手腕(landmark 0)在【原图】的像素坐标
    #   fingers_up: 中指是否朝上(中指尖 y < 手腕 y),用于判断手心朝向

性能优化（关键）:
    - model_complexity=0 (lite 模型,比 full 快约 1 倍)
    - 输入图缩到最长边 480 像素再喂给 MediaPipe（关键点是归一化坐标,
      缩放不影响映射回原图,但推理量大幅下降）
    - max_num_hands=4（覆盖多人场景,画面手少时不影响速度）

手势判断用"指尖到手腕距离 vs 指根到手腕距离",方向无关。
"""

from typing import List, Tuple

import mediapipe as mp
import cv2


class GestureDetector:
    """封装 MediaPipe Hands,提供握拳/V字/张开检测。"""

    def __init__(
        self,
        max_num_hands: int = 4,
        model_complexity: int = 0,
        min_detection_confidence: float = 0.6,
        min_tracking_confidence: float = 0.6,
        input_max_size: int = 480,
    ):
        self.hands = mp.solutions.hands.Hands(
            static_image_mode=False,
            max_num_hands=max_num_hands,
            model_complexity=model_complexity,
            min_detection_confidence=min_detection_confidence,
            min_tracking_confidence=min_tracking_confidence,
        )
        self._input_max_size = input_max_size

    def detect_all(self, frame) -> List[Tuple[str, Tuple[int, int], bool]]:
        """
        检测画面中所有手,返回 [(gesture, (wx, wy), fingers_up), ...]。
          gesture:    'fist' / 'victory' / 'open' / 'unknown'
          (wx, wy):   手腕在【原图】的像素坐标
          fingers_up: 中指是否朝上(用于判断手心朝向)
        """
        if frame is None:
            return []

        h, w = frame.shape[:2]
        # 缩小输入图加速(MediaPipe 关键点是归一化坐标,缩放后仍能映射回原图)
        scale = self._input_max_size / max(h, w)
        if scale < 1.0:
            small = cv2.resize(frame, (int(w * scale), int(h * scale)))
        else:
            small = frame

        rgb = cv2.cvtColor(small, cv2.COLOR_BGR2RGB)
        results = self.hands.process(rgb)
        if not results.multi_hand_landmarks:
            return []

        out: List[Tuple[str, Tuple[int, int], bool]] = []
        for hand_landmarks in results.multi_hand_landmarks:
            lm = hand_landmarks.landmark
            gesture = self._classify(lm)
            wx = int(lm[0].x * w)   # 归一化坐标 × 原图尺寸 → 原图像素
            wy = int(lm[0].y * h)
            fingers_up = lm[12].y < lm[0].y    # 中指尖(12) 在手腕(0) 上方 = 朝上
            out.append((gesture, (wx, wy), fingers_up))
        return out

    def detect_fists(self, frame) -> List[Tuple[int, int]]:
        """便利方法:只返回握拳手腕坐标列表。"""
        return [pos for (g, pos, _) in self.detect_all(frame) if g == 'fist']

    def detect_victory_hands(self, frame) -> List[Tuple[int, int]]:
        """便利方法:只返回 V 字手势(比耶)的手腕坐标列表。"""
        return [pos for (g, pos, _) in self.detect_all(frame) if g == 'victory']

    @staticmethod
    def _classify(lm) -> str:
        """
        判断手势:
          OK(拇指碰食指成圈 + 中/无名/小指伸直)     → 'ok'
          4 指全弯                                 → 'fist'(握拳)
          4 指伸 + 拇指伸                           → 'open'(五指张开)
          食+中伸、无名和小指都弯                     → 'victory'(V字)
          其他                                      → 'unknown'
        方向无关(手朝上/下/横着都行)。
        """
        wx, wy = lm[0].x, lm[0].y

        def dist_to_wrist(p) -> float:
            dx, dy = p.x - wx, p.y - wy
            return dx * dx + dy * dy

        def dist(a, b) -> float:
            dx, dy = a.x - b.x, a.y - b.y
            return dx * dx + dy * dy

        # 4 指是否伸直:True=伸直(指尖比指根离手腕更远)
        # 食指(8,5) 中指(12,9) 无名指(16,13) 小指(20,17)
        index_e  = dist_to_wrist(lm[8])  >= dist_to_wrist(lm[5])
        middle_e = dist_to_wrist(lm[12]) >= dist_to_wrist(lm[9])
        ring_e   = dist_to_wrist(lm[16]) >= dist_to_wrist(lm[13])
        pinky_e  = dist_to_wrist(lm[20]) >= dist_to_wrist(lm[17])
        thumb_e  = dist_to_wrist(lm[4])  >= dist_to_wrist(lm[2])

        # OK 手势:拇+食指圈起(dist 小), 中/无名/小指伸直
        hand_scale = max(dist_to_wrist(lm[9]) ** 0.5, 1e-6)   # 手腕到中指根距离做尺度
        tip_dist = dist(lm[4], lm[8]) ** 0.5                 # 拇指尖到食指尖距离
        if tip_dist < hand_scale * 0.25 and middle_e and ring_e and pinky_e:
            return 'ok'

        n_ext = (1 if index_e else 0) + (1 if middle_e else 0) + (1 if ring_e else 0) + (1 if pinky_e else 0)
        if n_ext == 0:
            return 'fist'                                     # 4 指全弯 = 握拳
        if n_ext >= 3:
            return 'open' if thumb_e else 'ok'                # 4指伸:拇指伸=open, 拇指收=ok(也开放,兼容)
        if index_e and middle_e and not ring_e and not pinky_e:
            return 'victory'                                  # 食中伸 + 无名和小指都弯 = V字
        return 'unknown'

    def close(self) -> None:
        if self.hands is not None:
            self.hands.close()
            self.hands = None

    def __del__(self):
        self.close()
