"""
tracking_servo_app.py — 在 tracking_app.py 基础上增加串口舵机控制。

用法:
    python scripts/tracking_servo_app.py --device npu --source 0

与原版的区别:
    - 新增 --serial-port 参数（默认 /dev/ttyUSB0）
    - 每帧选离画面中心最近的目标，算角度 0~180 通过串口发送
    - 串口不可用时自动降级，不影响跟踪功能
    - 利用卡尔曼速度估计做前瞻预测，补偿检测→舵机端到端延迟
"""

import argparse
import os
import sys
import time
from pathlib import Path
from typing import List, Optional


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
    return labels, allowed_track_class_ids, model_path, backend, capture_context, tracker, serial_ctrl


def print_tracking_startup(
    args: argparse.Namespace,
    labels: List[str],
    allowed_track_class_ids: Optional[List[int]],
    model_path: Path,
    backend,
    capture_context,
) -> None:
    print_runtime_banner(args.device, model_path)
    print_capture_summary(args.camera_profile, args.camera_mjpeg, capture_context)
    if allowed_track_class_ids is not None:
        selected_labels = ", ".join(labels[class_id] for class_id in allowed_track_class_ids)
        print(f"Tracking classes: {selected_labels}")
    backend.print_model_io()
    print("Press 'q' to quit.")


def _select_or_follow_target(visible_tracks, all_tracks, frame_center_x, frame_center_y, locked_id, grace_frames):
    """
    锁定跟随策略（带漏检宽限期）：
    1. locked_id 在可见 tracks → 继续跟
    2. locked_id 暂时漏检但仍在 all_tracks 且 time_since_update ≤ grace → 用其卡尔曼预测位置继续跟，不切人
    3. locked_id 彻底消失（超过 grace）→ 释放锁
    4. 无锁 → 选离画面中心最近的可见目标并锁定
    返回 (target_track, new_locked_id)。target_track 为 None 表示无目标。
    """
    if not visible_tracks and not all_tracks:
        return None, None

    # 有锁：先在可见 tracks 里找
    if locked_id is not None:
        for track in visible_tracks:
            if track.track_id == locked_id:
                return track, locked_id
        # 可见里没有：在全部 tracks 里找（宽限期内，用预测位置继续跟）
        for track in all_tracks:
            if track.track_id == locked_id and track.time_since_update <= grace_frames:
                return track, locked_id

    # 无锁 或 锁定目标彻底消失 → 选离中心最近的可见目标并锁定
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
    avg_timings_ms = compute_average_timings(timing_totals, frame_count)
    fps = compute_display_fps(avg_timings_ms, capture_context.capture_fps)
    annotated = draw_tracks(frame, tracks, labels, fps, model_path.name, args.device, len(detections), avg_timings_ms)
    draw_ms = (time.perf_counter() - draw_start) * 1000.0
    timing_totals["draw"] += draw_ms

    # ── 串口舵机控制：锁定跟随 → 速度前瞻 → EMA平滑 → 非线性增益追踪 → 发送角度 ──
    if serial_ctrl is not None and servo_state is not None:
        frame_h, frame_w = frame.shape[:2]
        locked_id = target_lock[0] if target_lock else None
        target_track, new_locked_id = _select_or_follow_target(
            tracks, tracker.tracks, frame_w * 0.5, frame_h * 0.5, locked_id, args.lock_grace_frames
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

            # ⑦ 累积舵机角度（跨帧持久化，实现大范围追踪）
            servo_state['pan'] += delta_pan
            servo_state['tilt'] += delta_tilt
            servo_state['pan'] = max(0.0, min(180.0, servo_state['pan']))
            servo_state['tilt'] = max(0.0, min(180.0, servo_state['tilt']))

            serial_ctrl.send_angles(int(servo_state['pan']), int(servo_state['tilt']))
        else:
            # 目标丢失：重置平滑位置，下次重新初始化
            servo_state.pop('smooth_cx', None)
            servo_state.pop('smooth_cy', None)

    return annotated, fps


def main() -> int:
    args = parse_args()
    model_dir = Path(args.model_dir).expanduser().resolve()
    backend = None
    capture_context = None

    if args.list_models:
        return list_available_models(model_dir, args.device)

    try:
        labels, allowed_track_class_ids, model_path, backend, capture_context, tracker, serial_ctrl = prepare_tracking_runtime(args, model_dir)
    except Exception as exc:
        print(f"Failed to prepare tracking pipeline: {exc}")
        if backend is not None:
            backend.release()
        return 1

    writer = None
    frame_count = 0
    timing_totals = create_timing_totals()
    pending_frame = capture_context.first_frame
    pending_read_ms = capture_context.first_read_ms
    target_lock = [None]          # 跨帧锁定状态：当前跟随的目标 ID
    servo_state = {'pan': 90.0, 'tilt': 90.0}  # 舵机累积角度（跨帧持久化）

    try:
        print_tracking_startup(args, labels, allowed_track_class_ids, model_path, backend, capture_context)

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
            )

            if args.save:
                if writer is None:
                    writer_fps = resolve_writer_fps(capture_context.capture_fps, fps)
                    writer = create_video_writer(args.save, writer_fps, annotated.shape)
                writer.write(annotated)

            if not args.no_display:
                cv2.imshow(args.window_name, annotated)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
    finally:
        capture_context.cap.release()
        if writer is not None:
            writer.release()
        if not args.no_display:
            cv2.destroyAllWindows()
        backend.release()
        if serial_ctrl is not None:
            serial_ctrl.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
