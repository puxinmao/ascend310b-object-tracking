"""
tracking_servo_app.py — SSD 跟踪 + 串口舵机控制 + 握拳手势锁定。

用法:
    python scripts/tracking_servo_app.py --device npu --source 0

功能:
    - SSD 检测 + DeepSORT 多目标跟踪 + 卡尔曼预测式舵机控制
    - 串口 5 字节帧协议发送角度到 STM32
    - 手势锁定(MediaPipe Hands):画面里谁握拳就跟踪谁
        · 握拳(单手)→ 锁定第一个握拳的人(锁定后别人握拳不抢)
        · 按键 1 → 切换手动俯仰模式(开/关)
        · 手动俯仰模式中 → 张开手掌上/下控制俯仰角度
        · V 字手势(单手,比耶,手动俯仰模式中)→ 退出手动俯仰,回到锁定状态
        · V 字手势(单手,比耶,普通锁定状态)→ 取消锁定,回到 AUTO
        · 锁定目标消失后回到"跟最近的人"自由模式
    - --no-gesture 关闭手势功能
"""

import argparse
import os
import sys
import time
from pathlib import Path
from typing import List, Optional
import queue
import threading
import select


ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT_DIR not in sys.path:
    sys.path.insert(0, ROOT_DIR)

from utils.opencv_runtime import (
    cv2,
    add_camera_arguments,
    compute_average_timings,
    compute_display_fps,
    create_backend,
    create_timing_totals,
    list_available_models,
    open_capture_context,
    print_capture_summary,
    print_runtime_banner,
    read_frame,
    resolve_writer_fps,
    update_timing_totals,
)
from utils.preprocessing import (
    MODEL_DIR,
    create_video_writer,
    load_labels,
    resolve_model_path,
)
from tracking.deepsort import DeepSORT
from utils.postprocessing import detections_to_tracker_inputs, draw_tracks
from utils.serial_controller import SerialController

# 手势模块可选(MediaPipe 未装时自动降级,--gesture 会报错退出)
try:
    from gesture.hand_gesture import GestureDetector
    _GESTURE_AVAILABLE = True
    _GESTURE_IMPORT_ERROR = ""
except ImportError as _exc:
    _GESTURE_AVAILABLE = False
    _GESTURE_IMPORT_ERROR = str(_exc)
    GestureDetector = None  # type: ignore

# ── 舵机追踪控制默认参数（可通过命令行覆盖） ─────────────────
_CAMERA_HFOV_DEG = 70.0          # 摄像头水平视场角（度）
_TRACKING_GAIN_PAN = 0.35        # 水平追踪增益（平衡响应与防抖）
_TRACKING_GAIN_TILT = 0.15       # 俯仰追踪增益（上下仍需克制）
_CENTER_DEAD_ZONE_RATIO = 0.15   # 中心死区比例（适度收窄提升响应）
_MAX_STEP_PAN = 4.0              # 水平每帧最大步长（度）
_MAX_STEP_TILT = 2.0             # 俯仰每帧最大步长（度）
_SMOOTH_ALPHA = 0.40             # 目标位置 EMA 平滑系数（加大以更快跟踪）
_PREDICT_LEAD_FRAMES = 3.0       # 卡尔曼速度前瞻帧数（补偿检测+舵机端到端延迟，0=关闭）
_PREDICT_SPEED_THRESHOLD = 8.0   # 前瞻速度门限（像素/帧）：速度低于此值时按比例衰减前瞻，避免静止目标晃动
_TRACK_ANCHOR_Y = 0.5            # 跟踪点垂直锚点（0=框顶/头部, 0.5=中心/腰部, 1=框底/脚）：调低可对准上身/头部
_LOCK_GRACE_FRAMES = 15          # 锁定目标漏检宽限帧数：期间用卡尔曼预测位置继续跟随，避免短暂漏检就切人
_GESTURE_INTERVAL = 3            # 手势检测间隔帧数（降频省 CPU，每 N 帧检测一次）
_FIST_CONFIRM_FRAMES = 3         # 握拳确认帧数（连续检测到才锁定，防误触）
_MANUAL_TILT_STEP = 3.0          # 手动俯仰每触发一次的 tilt 变化（度）
_MANUAL_TILT_THRESHOLD = 5       # 触发手动俯仰的最小手腕位移（像素，跨检测帧）


# ── 终端键盘监听（后台线程,Linux select 非阻塞读 stdin） ─────────────────
def _term_cmd_listener(cmd_queue, stop_event):
    """后台线程:非阻塞监听终端输入,每行作为一个命令序列。"""
    while not stop_event.is_set():
        try:
            r, _, _ = select.select([sys.stdin], [], [], 0.3)
            if r:
                line = sys.stdin.readline()
                if not line:
                    break
                for ch in line.strip():
                    cmd_queue.put(ch)
        except Exception:
            break


def parse_track_classes(track_classes_arg: str, labels: List[str]) -> Optional[List[int]]:
    if not track_classes_arg.strip():
        return None

    label_to_id = {label.strip().lower(): index for index, label in enumerate(labels)}
    class_ids = []
    for item in track_classes_arg.split(","):
        token = item.strip()
        if not token:
            continue
        if token.isdigit():
            class_id = int(token)
        else:
            class_id = label_to_id.get(token.lower())
            if class_id is None:
                available = ", ".join(labels[:20])
                raise ValueError(f"Unknown tracking class: {token}. Examples: {available}")
        if class_id <= 0 or class_id >= len(labels):
            raise ValueError(f"Tracking class id out of range: {class_id}")
        class_ids.append(class_id)

    if not class_ids:
        return None
    return sorted(set(class_ids))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Real-time SSD tracking with serial servo control.")
    parser.add_argument("--device", choices=["cpu", "npu"], default="npu", help="Inference backend.")
    parser.add_argument("--device-id", type=int, default=0, help="Ascend device id when --device=npu.")
    parser.add_argument("--backbone", default="mobilenetv3_large_100", help="Model backbone name used for auto-discovery.")
    parser.add_argument("--model", default="", help="Explicit model path. Overrides --backbone.")
    parser.add_argument("--model-dir", default=str(MODEL_DIR), help="Directory that stores SSD model files.")
    parser.add_argument("--source", default="0", help="Camera index or video path.")
    parser.add_argument("--score-threshold", type=float, default=0.35, help="Minimum confidence for tracking detections.")
    parser.add_argument("--nms-threshold", type=float, default=0.45, help="NMS IoU threshold for detection.")
    parser.add_argument("--max-detections", type=int, default=100, help="Maximum detections per frame.")
    add_camera_arguments(parser)
    parser.add_argument("--labels", default="", help="Optional label file path. Defaults to COCO labels.")
    parser.add_argument("--window-name", default="SSD Tracking", help="OpenCV display window name.")
    parser.add_argument("--save", default="", help="Optional output video path.")
    parser.add_argument("--no-display", action="store_true", help="Disable cv2.imshow for headless environments.")
    parser.add_argument("--list-models", action="store_true", help="List available models for the selected device and exit.")
    parser.add_argument("--track-max-age", type=int, default=90, help="Maximum frames to keep unmatched tracks alive.")
    parser.add_argument("--track-min-hits", type=int, default=1, help="Minimum matched detections before a track is displayed.")
    parser.add_argument("--track-iou-threshold", type=float, default=0.3, help="Minimum IoU required to associate detections to tracks.")
    parser.add_argument("--track-center-distance-threshold", type=float, default=1.8, help="Maximum normalized center distance used for fallback association.")
    parser.add_argument("--track-size-smoothing", type=float, default=0.8, help="Track box size smoothing factor in [0, 1). Higher means more stable but less responsive.")
    parser.add_argument("--track-score-smoothing", type=float, default=0.7, help="Track score smoothing factor in [0, 1). Higher means less score flicker.")
    parser.add_argument("--track-classes", default="person", help="Comma-separated class names or ids to track, for example 'person,bus' or '1,6'.")
    parser.add_argument("--serial-port", default="/dev/ttyUSB0", help="Serial port for servo control. Set to empty to disable.")
    parser.add_argument("--camera-fov", type=float, default=_CAMERA_HFOV_DEG,
                        help=f"Camera horizontal FOV in degrees (default: {_CAMERA_HFOV_DEG}).")
    parser.add_argument("--gain-pan", type=float, default=_TRACKING_GAIN_PAN,
                        help=f"Horizontal tracking gain (default: {_TRACKING_GAIN_PAN}, lower = less jitter).")
    parser.add_argument("--gain-tilt", type=float, default=_TRACKING_GAIN_TILT,
                        help=f"Vertical tracking gain (default: {_TRACKING_GAIN_TILT}, keep very low).")
    parser.add_argument("--dead-zone", type=float, default=_CENTER_DEAD_ZONE_RATIO,
                        help=f"Center dead-zone ratio of half-frame (default: {_CENTER_DEAD_ZONE_RATIO}).")
    parser.add_argument("--max-step-pan", type=float, default=_MAX_STEP_PAN,
                        help=f"Max horizontal servo step per frame (default: {_MAX_STEP_PAN}).")
    parser.add_argument("--max-step-tilt", type=float, default=_MAX_STEP_TILT,
                        help=f"Max vertical servo step per frame (default: {_MAX_STEP_TILT}).")
    parser.add_argument("--smooth-alpha", type=float, default=_SMOOTH_ALPHA,
                        help=f"Target position EMA smoothing (default: {_SMOOTH_ALPHA}, 0.1=very smooth, 0.5=responsive).")
    parser.add_argument("--predict-lead-frames", type=float, default=_PREDICT_LEAD_FRAMES,
                        help=f"Kalman velocity look-ahead in frames (default: {_PREDICT_LEAD_FRAMES}, compensates end-to-end latency; 0=off).")
    parser.add_argument("--predict-speed-threshold", type=float, default=_PREDICT_SPEED_THRESHOLD,
                        help=f"Speed gate for look-ahead in px/frame (default: {_PREDICT_SPEED_THRESHOLD}); below it look-ahead fades out to suppress static-target jitter.")
    parser.add_argument("--track-anchor-y", type=float, default=_TRACK_ANCHOR_Y,
                        help=f"Vertical anchor of tracking point in bbox (default: {_TRACK_ANCHOR_Y}; 0=top/head, 0.5=center, 1=bottom). Lower to follow upper body / head.")
    parser.add_argument("--lock-grace-frames", type=int, default=_LOCK_GRACE_FRAMES,
                        help=f"Locked-target grace frames (default: {_LOCK_GRACE_FRAMES}); brief detection gaps keep the lock via Kalman predict.")
    parser.add_argument("--gesture", dest="gesture", action="store_true", default=True,
                        help="Enable fist-gesture target locking (default: on).")
    parser.add_argument("--no-gesture", dest="gesture", action="store_false",
                        help="Disable fist-gesture locking.")
    parser.add_argument("--gesture-interval", type=int, default=_GESTURE_INTERVAL,
                        help=f"Run fist detection every N frames (default: {_GESTURE_INTERVAL}, lower = more responsive but more CPU).")
    parser.add_argument("--fist-confirm-frames", type=int, default=_FIST_CONFIRM_FRAMES,
                        help=f"Consecutive fist detections to confirm lock (default: {_FIST_CONFIRM_FRAMES}).")
    parser.add_argument("--manual-tilt-step", type=float, default=_MANUAL_TILT_STEP,
                        help=f"Tilt degrees per manual gesture trigger (default: {_MANUAL_TILT_STEP}).")
    return parser.parse_args()


def build_tracker(args: argparse.Namespace) -> DeepSORT:
    return DeepSORT(
        max_age=args.track_max_age,
        min_hits=args.track_min_hits,
        iou_threshold=args.track_iou_threshold,
        center_distance_threshold=args.track_center_distance_threshold,
        size_smoothing=args.track_size_smoothing,
        score_smoothing=args.track_score_smoothing,
    )


def prepare_tracking_runtime(args: argparse.Namespace, model_dir: Path):
    labels = load_labels(args.labels)
    allowed_track_class_ids = parse_track_classes(args.track_classes, labels)
    model_path = resolve_model_path(args.model, args.backbone, model_dir, args.device)
    backend = create_backend(args.device, model_path, args.device_id)
    capture_context = open_capture_context(args.source, args.camera_profile, args.camera_mjpeg)
    tracker = build_tracker(args)
    serial_ctrl = SerialController(args.serial_port) if args.serial_port else None
    gesture_detector = None
    if args.gesture:
        if not _GESTURE_AVAILABLE:
            raise ImportError(f"--gesture enabled but GestureDetector unavailable: {_GESTURE_IMPORT_ERROR}")
        gesture_detector = GestureDetector()
    return labels, allowed_track_class_ids, model_path, backend, capture_context, tracker, serial_ctrl, gesture_detector


def print_tracking_startup(
    args: argparse.Namespace,
    labels: List[str],
    allowed_track_class_ids: Optional[List[int]],
    model_path: Path,
    backend,
    capture_context,
    gesture_enabled: bool,
) -> None:
    print_runtime_banner(args.device, model_path)
    print_capture_summary(args.camera_profile, args.camera_mjpeg, capture_context)
    if allowed_track_class_ids is not None:
        selected_labels = ", ".join(labels[class_id] for class_id in allowed_track_class_ids)
        print(f"Tracking classes: {selected_labels}")
    backend.print_model_io()
    if gesture_enabled:
        print("Gesture locking: ENABLED — fist to lock, [1] toggle manual tilt, V to release/exit.")
    else:
        print("Gesture locking: disabled.")
    print("Press 'q' to quit.")


def _update_fist_lock(
    args: argparse.Namespace,
    hands: list,
    visible_tracks,
    all_tracks,
    fist_target: Optional[list],
    fist_candidate: Optional[list],
    release_candidate: Optional[list],
    manual_mode_active: Optional[list] = None,
) -> None:
    """
    手势锁定状态机（单手判定,均带防抖）：
      未锁定 → 某人【单手握拳】连续确认 → 锁定
      已锁定(手动模式中) → V 字 → 退出手动模式,回到锁定
      已锁定(普通模式) → V 字 → 取消锁定,回到 AUTO
      锁定人彻底消失(超 grace)→ 回自由
    hands: detect_all 的结果(可能为空),降频由调用方控制。
    """
    if fist_target is None or fist_candidate is None:
        return

    # ── 已锁定:检查 V 字取消 ──
    if fist_target[0] is not None:
        ft_id = fist_target[0]
        ft_alive = any(t.track_id == ft_id for t in visible_tracks) or any(
            t.track_id == ft_id and t.time_since_update <= args.lock_grace_frames for t in all_tracks
        )
        if not ft_alive:
            fist_target[0] = None
            fist_candidate[0] = None
            fist_candidate[1] = 0
            if release_candidate is not None:
                release_candidate[0] = None
                release_candidate[1] = 0
            if manual_mode_active is not None:
                manual_mode_active[0] = False
            return
        if not hands or release_candidate is None:
            return
        locked_bbox = None
        for t in visible_tracks:
            if t.track_id == ft_id:
                locked_bbox = t.bbox
                break
        if locked_bbox is None:
            return
        x1, y1, x2, y2 = locked_bbox
        v_in_lock = any(
            g == 'victory' and x1 <= wx <= x2 and y1 <= wy <= y2
            for (g, (wx, wy), _) in hands
        )
        if v_in_lock:
            if release_candidate[0] == ft_id:
                release_candidate[1] += 1
            else:
                release_candidate[0] = ft_id
                release_candidate[1] = 1
            if release_candidate[1] >= args.fist_confirm_frames:
                # 手动模式中? → 退出手动模式,回锁定; 普通模式? → 回 AUTO
                if manual_mode_active is not None and manual_mode_active[0]:
                    manual_mode_active[0] = False
                else:
                    fist_target[0] = None
                fist_candidate[0] = None
                fist_candidate[1] = 0
                release_candidate[0] = None
                release_candidate[1] = 0
        else:
            release_candidate[0] = None
            release_candidate[1] = 0
        return

    # ── 未锁定:检测握拳 ──
    if not hands:
        return
    fist_person_id = None
    for (g, (wx, wy), _) in hands:
        if g != 'fist':
            continue
        for t in visible_tracks:
            x1, y1, x2, y2 = t.bbox
            if x1 <= wx <= x2 and y1 <= wy <= y2:
                fist_person_id = t.track_id
                break
        if fist_person_id is not None:
            break

    if fist_person_id is not None:
        if fist_candidate[0] == fist_person_id:
            fist_candidate[1] += 1
        else:
            fist_candidate[0] = fist_person_id
            fist_candidate[1] = 1
        if fist_candidate[1] >= args.fist_confirm_frames:
            fist_target[0] = fist_person_id
            fist_candidate[0] = None
            fist_candidate[1] = 0
    else:
        fist_candidate[0] = None
        fist_candidate[1] = 0


def _check_manual_tilt(hands: list, manual_state: Optional[dict],
                       manual_mode_active: Optional[list] = None) -> Optional[str]:
    """
    手动俯仰模式中:张开手控制俯仰。
      仅当 manual_mode_active 为 True 时生效。
      中指朝上(手心朝上)→ 'up'
      中指朝下(手心朝下)→ 'down'
      无张开手 → None
    """
    if manual_mode_active is not None and not manual_mode_active[0]:
        return None
    for (g, (wx, wy), fingers_up) in hands:
        if g == 'open':
            return 'up' if fingers_up else 'down'
    return None


def _select_or_follow_target(visible_tracks, all_tracks, frame_center_x, frame_center_y,
                             locked_id, grace_frames, fist_target_id=None):
    """
    目标选择优先级:
      1. fist_target_id(握拳锁定的目标,最高优先级)
      2. locked_id(自由模式下锁定保持,带 grace 宽限)
      3. 离画面中心最近的可见目标
    返回 (target_track, new_locked_id)。
    """
    if not visible_tracks and not all_tracks:
        return None, None

    # 优先级1:握拳锁定的目标
    if fist_target_id is not None:
        for track in visible_tracks:
            if track.track_id == fist_target_id:
                return track, fist_target_id
        for track in all_tracks:
            if track.track_id == fist_target_id and track.time_since_update <= grace_frames:
                return track, fist_target_id

    # 优先级2:自由模式的锁定保持
    if locked_id is not None:
        for track in visible_tracks:
            if track.track_id == locked_id:
                return track, locked_id
        for track in all_tracks:
            if track.track_id == locked_id and track.time_since_update <= grace_frames:
                return track, locked_id

    # 优先级3:选离中心最近的可见目标
    if not visible_tracks:
        return None, None
    min_dist = float("inf")
    best_track = None
    for track in visible_tracks:
        cx = (track.bbox[0] + track.bbox[2]) * 0.5
        cy = (track.bbox[1] + track.bbox[3]) * 0.5
        dist = (cx - frame_center_x) ** 2 + (cy - frame_center_y) ** 2
        if dist < min_dist:
            min_dist = dist
            best_track = track
    if best_track is None:
        return None, None
    return best_track, best_track.track_id


def render_tracking_frame(
    args: argparse.Namespace,
    frame,
    read_ms: float,
    labels: List[str],
    model_path: Path,
    backend,
    tracker: DeepSORT,
    allowed_track_class_ids: Optional[List[int]],
    timing_totals: dict[str, float],
    frame_count: int,
    capture_context,
    serial_ctrl: Optional[SerialController] = None,
    target_lock: Optional[list] = None,
    servo_state: Optional[dict] = None,
    gesture_detector=None,
    fist_target: Optional[list] = None,
    fist_candidate: Optional[list] = None,
    release_candidate: Optional[list] = None,
    manual_state: Optional[dict] = None,
    manual_mode_active: Optional[list] = None,
):
    detections, profile_ms = backend.infer_with_profile(
        frame,
        args.score_threshold,
        args.nms_threshold,
        args.max_detections,
        allowed_class_ids=allowed_track_class_ids,
    )
    update_timing_totals(timing_totals, read_ms, profile_ms)
    draw_start = time.perf_counter()
    tracker_inputs = detections_to_tracker_inputs(detections)
    tracks = tracker.update(tracker_inputs)

    # ── 降频检测手(一次 process,锁定/手动俯仰复用)──
    hands = []
    if gesture_detector is not None and frame_count % max(args.gesture_interval, 1) == 0:
        hands = gesture_detector.detect_all(frame)

    # ── 握拳/V字 锁定状态机 ──
    _update_fist_lock(args, hands, tracks, tracker.tracks,
                      fist_target, fist_candidate, release_candidate,
                      manual_mode_active)

    # ── 手动俯仰方向(仅手动模式激活时生效,按键1切换)──
    manual_tilt_dir = _check_manual_tilt(hands, manual_state, manual_mode_active)

    avg_timings_ms = compute_average_timings(timing_totals, frame_count)
    fps = compute_display_fps(avg_timings_ms, capture_context.capture_fps)
    annotated = draw_tracks(frame, tracks, labels, fps, model_path.name, args.device, len(detections), avg_timings_ms)

    # ── 手势状态可视化(画面右上角)──
    if gesture_detector is not None and fist_target is not None and fist_candidate is not None:
        # 确定基础状态文字
        if manual_mode_active is not None and manual_mode_active[0]:
            status = f"FIST LOCK: ID {fist_target[0]} [MANUAL]"
            color = (255, 128, 0)        # 橙 = 锁定 + 手动俯仰模式
            if release_candidate is not None and release_candidate[0] is not None:
                status += f"  V? ({release_candidate[1]}/{args.fist_confirm_frames})"
        elif fist_target[0] is not None:
            status = f"FIST LOCK: ID {fist_target[0]}"
            color = (0, 0, 255)        # 红 = 已握拳锁定
            if release_candidate is not None and release_candidate[0] is not None:
                status += f"  V? ({release_candidate[1]}/{args.fist_confirm_frames})"
        elif fist_candidate[0] is not None:
            status = f"FIST? ID {fist_candidate[0]} ({fist_candidate[1]}/{args.fist_confirm_frames})"
            color = (0, 200, 200)                                                  # 黄 = 确认中
        else:
            status, color = "AUTO", (0, 200, 0)                                    # 绿 = 自由跟踪
        # 状态文字放右上角(右对齐)
        (tw, th), _ = cv2.getTextSize(status, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)
        cv2.putText(annotated, status, (annotated.shape[1] - tw - 10, th + 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

    draw_ms = (time.perf_counter() - draw_start) * 1000.0
    timing_totals["draw"] += draw_ms

    # ── 串口舵机控制:锁定跟随 → 速度前瞻 → EMA平滑 → 非线性增益追踪 → 发送角度 ──
    if serial_ctrl is not None and servo_state is not None:
        frame_h, frame_w = frame.shape[:2]
        locked_id = target_lock[0] if target_lock else None
        fist_id = fist_target[0] if fist_target else None
        target_track, new_locked_id = _select_or_follow_target(
            tracks, tracker.tracks, frame_w * 0.5, frame_h * 0.5,
            locked_id, args.lock_grace_frames, fist_id
        )
        if target_lock is not None:
            target_lock[0] = new_locked_id

        if target_track is not None and new_locked_id is not None:
            # ① 取目标跟踪点（默认框中心；--track-anchor-y<0.5 时上移到上身/头部）+ 卡尔曼速度前瞻
            target_cx = (target_track.bbox[0] + target_track.bbox[2]) * 0.5
            y1, y2 = target_track.bbox[1], target_track.bbox[3]
            target_cy = y1 + (y2 - y1) * args.track_anchor_y
            if args.predict_lead_frames > 0.0:
                kf_state = target_track.kalman_filter.x   # [x, y, vx, vy]
                vx = float(kf_state[2, 0])
                vy = float(kf_state[3, 0])
                # 速度门控：速度低时按比例衰减前瞻量，避免静止目标的卡尔曼噪声被放大成左右晃
                speed = (vx * vx + vy * vy) ** 0.5
                gate = min(1.0, speed / max(args.predict_speed_threshold, 1e-3))
                target_cx += vx * args.predict_lead_frames * gate
                target_cy += vy * args.predict_lead_frames * gate

            # ② EMA 平滑目标位置（消除检测框逐帧抖动）
            alpha = args.smooth_alpha
            if 'smooth_cx' not in servo_state:
                servo_state['smooth_cx'] = target_cx
                servo_state['smooth_cy'] = target_cy
            else:
                servo_state['smooth_cx'] = alpha * target_cx + (1.0 - alpha) * servo_state['smooth_cx']
                servo_state['smooth_cy'] = alpha * target_cy + (1.0 - alpha) * servo_state['smooth_cy']
            scx = servo_state['smooth_cx']
            scy = servo_state['smooth_cy']

            # ③ 计算平滑后目标偏离画面中心的角度误差（度）
            cam_vfov = args.camera_fov * frame_h / frame_w
            half_hfov = args.camera_fov * 0.5
            half_vfov = cam_vfov * 0.5
            err_x_deg = (scx - frame_w * 0.5) / frame_w * args.camera_fov
            err_y_deg = (scy - frame_h * 0.5) / frame_h * cam_vfov

            # ④ 死区：误差小于阈值 → 完全不动
            dead_h = args.camera_fov * args.dead_zone
            dead_v = cam_vfov * args.dead_zone
            abs_x = abs(err_x_deg)
            abs_y = abs(err_y_deg)

            # ⑤ 非线性增益：近中心轻柔防过冲，远边缘全速追
            def _soft_gain(abs_err, dead, half_fov, base_gain):
                """死区外：随误差增大，增益从 30% 平滑爬升到 100%"""
                if abs_err <= dead:
                    return 0.0
                # t: 0(刚出死区) → 1(边缘)
                t = min(1.0, (abs_err - dead) / max(half_fov - dead, 0.1))
                effective = base_gain * (0.30 + 0.70 * t)  # 30%~100% 爬升
                return effective

            gain_x = _soft_gain(abs_x, dead_h, half_hfov, args.gain_pan)
            gain_y = _soft_gain(abs_y, dead_v, half_vfov, args.gain_tilt)

            delta_pan = err_x_deg * gain_x
            delta_tilt = err_y_deg * gain_y

            # ⑥ 分别限制水平/俯仰每帧步长
            delta_pan = max(-args.max_step_pan, min(args.max_step_pan, delta_pan))
            delta_tilt = max(-args.max_step_tilt, min(args.max_step_tilt, delta_tilt))

            # ⑥' 手动俯仰(仅握拳锁定后生效,避免未锁定时破坏自动 tilt):覆盖 delta_tilt
            if fist_id is not None and manual_tilt_dir == 'up':
                delta_tilt = -args.manual_tilt_step
            elif fist_id is not None and manual_tilt_dir == 'down':
                delta_tilt = args.manual_tilt_step

            # ⑦ 累积舵机角度（跨帧持久化，实现大范围追踪）
            servo_state['pan'] += delta_pan
            servo_state['tilt'] += delta_tilt
            servo_state['pan'] = max(0.0, min(180.0, servo_state['pan']))
            servo_state['tilt'] = max(0.0, min(180.0, servo_state['tilt']))

            serial_ctrl.send_angles(int(servo_state['pan']), int(servo_state['tilt']),
                                    int(new_locked_id) if new_locked_id else 0)
        else:
            # 目标丢失：重置平滑位置，下次重新初始化；发一帧 id=0 让 STM32 知道无目标
            servo_state.pop('smooth_cx', None)
            servo_state.pop('smooth_cy', None)
            serial_ctrl.send_angles(int(servo_state['pan']), int(servo_state['tilt']), 0)

    return annotated, fps


def main() -> int:
    args = parse_args()
    model_dir = Path(args.model_dir).expanduser().resolve()
    backend = None
    capture_context = None
    gesture_detector = None

    if args.list_models:
        return list_available_models(model_dir, args.device)

    try:
        (labels, allowed_track_class_ids, model_path, backend, capture_context,
         tracker, serial_ctrl, gesture_detector) = prepare_tracking_runtime(args, model_dir)
    except Exception as exc:
        print(f"Failed to prepare tracking pipeline: {exc}")
        if backend is not None:
            backend.release()
        if gesture_detector is not None:
            gesture_detector.close()
        return 1

    writer = None
    frame_count = 0
    timing_totals = create_timing_totals()
    pending_frame = capture_context.first_frame
    pending_read_ms = capture_context.first_read_ms
    target_lock = [None]          # 跨帧锁定状态：当前跟随的目标 ID
    servo_state = {'pan': 90.0, 'tilt': 90.0}  # 舵机累积角度（跨帧持久化）
    fist_target = [None]          # 握拳锁定的目标 ID（None=自由模式）
    fist_candidate = [None, 0]    # [握拳候选 ID, 连续次数]（锁定防抖用）
    release_candidate = [None, 0] # [V字候选 ID, 连续次数]（取消锁定防抖用）
    manual_state = {'prev_y': None}  # 手动俯仰:上一帧张开手手腕 y（像素）
    manual_mode_active = [False]  # 手动俯仰模式是否激活（按键1切换, V 退出）

    # ── 终端命令监听 ──
    cmd_queue = queue.Queue()
    stop_event = threading.Event()
    reader_thread = threading.Thread(
        target=_term_cmd_listener, args=(cmd_queue, stop_event), daemon=True
    )
    reader_thread.start()

    try:
        print_tracking_startup(args, labels, allowed_track_class_ids, model_path,
                               backend, capture_context, args.gesture)
        print("\n  [终端控制] 输入 1+回车 = 切换手动俯仰模式  输入 q+回车 = 退出\n")

        while True:
            frame, read_ms, pending_frame, pending_read_ms = read_frame(capture_context, pending_frame, pending_read_ms)
            if frame is None:
                print("Video stream ended or camera frame read failed.")
                break

            frame_count += 1
            annotated, fps = render_tracking_frame(
                args,
                frame,
                read_ms,
                labels,
                model_path,
                backend,
                tracker,
                allowed_track_class_ids,
                timing_totals,
                frame_count,
                capture_context,
                serial_ctrl=serial_ctrl,
                target_lock=target_lock,
                servo_state=servo_state,
                gesture_detector=gesture_detector,
                fist_target=fist_target,
                fist_candidate=fist_candidate,
                release_candidate=release_candidate,
                manual_state=manual_state,
                manual_mode_active=manual_mode_active,
            )

            if args.save:
                if writer is None:
                    writer_fps = resolve_writer_fps(capture_context.capture_fps, fps)
                    writer = create_video_writer(args.save, writer_fps, annotated.shape)
                writer.write(annotated)

            if not args.no_display:
                cv2.imshow(args.window_name, annotated)
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    break
                if key == ord("1"):
                    manual_mode_active[0] = not manual_mode_active[0]
                    print(f"[Manual Tilt] {'ON' if manual_mode_active[0] else 'OFF'}")

            # ── 终端命令处理（非阻塞,支持输入 1+回车 / q+回车）──
            try:
                while not cmd_queue.empty():
                    cmd = cmd_queue.get_nowait()
                    if cmd == 'q':
                        print("终端输入 q, 退出程序")
                        stop_event.set()
                        return 0
                    if cmd == '1':
                        manual_mode_active[0] = not manual_mode_active[0]
                        print(f"[Manual Tilt] {'ON' if manual_mode_active[0] else 'OFF'}")
            except Exception:
                pass
    finally:
        capture_context.cap.release()
        if writer is not None:
            writer.release()
        if not args.no_display:
            cv2.destroyAllWindows()
        backend.release()
        if serial_ctrl is not None:
            serial_ctrl.close()
        if gesture_detector is not None:
            gesture_detector.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
