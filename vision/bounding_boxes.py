from .bootstrap import ensure_runtime_environment


ensure_runtime_environment()

import cv2

from .tracker import Detection


def get_class_name(class_names, class_id: int) -> str:
    if hasattr(class_names, "get"):
        return class_names.get(class_id, str(class_id))

    try:
        return class_names[class_id]
    except (IndexError, KeyError, TypeError):
        return str(class_id)


def draw_bbox_position_values(bbox):
    x1, y1, x2, y2 = bbox.astype(int)

    return x1, y1, x2, y2


def get_bbox_center(x1: int, y1: int, x2: int, y2: int):
    return (x1 + x2) // 2, (y1 + y2) // 2


def extract_detections(
    result,
    class_names,
    displayed_class_name="person",
):
    detections = []

    if result.boxes is None or len(result.boxes) == 0:
        return detections

    boxes = result.boxes.xyxy.cpu().numpy()
    confidences = result.boxes.conf.cpu().numpy()
    class_ids = result.boxes.cls.cpu().numpy().astype(int)

    for box, confidence, class_id in zip(
        boxes,
        confidences,
        class_ids,
    ):
        x1, y1, x2, y2 = draw_bbox_position_values(box)
        class_name = get_class_name(class_names, class_id)

        if class_name != displayed_class_name:
            continue

        center = get_bbox_center(x1, y1, x2, y2)
        detections.append(
            Detection(
                bbox=(x1, y1, x2, y2),
                confidence=float(confidence),
                class_id=int(class_id),
                class_name=class_name,
                center=center,
            )
        )

    return detections


def draw_label_block(frame, x: int, y: int, lines) -> None:
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.45
    thickness = 1
    padding = 4
    line_gap = 3

    sizes = [
        cv2.getTextSize(line, font, font_scale, thickness)
        for line in lines
    ]
    text_width = max(size[0][0] for size in sizes)
    text_height = sum(size[0][1] for size in sizes)
    baseline = max(size[1] for size in sizes)
    block_height = text_height + baseline + padding * 2 + line_gap * (len(lines) - 1)
    block_width = text_width + padding * 2

    frame_height, frame_width = frame.shape[:2]
    block_x = max(0, min(x, frame_width - block_width))
    block_bottom_y = max(block_height, y)
    block_top_y = block_bottom_y - block_height

    cv2.rectangle(
        frame,
        (block_x, block_top_y),
        (block_x + block_width, block_bottom_y),
        (0, 255, 0),
        -1,
    )

    text_y = block_top_y + padding
    for line, (text_size, line_baseline) in zip(lines, sizes):
        text_y += text_size[1]
        cv2.putText(
            frame,
            line,
            (block_x + padding, text_y),
            font,
            font_scale,
            (0, 0, 0),
            thickness,
            cv2.LINE_AA,
        )
        text_y += line_baseline + line_gap


def draw_bbox_overlay(frame, bbox, center, label_lines) -> None:
    x1, y1, x2, y2 = bbox
    center_x, center_y = center

    cv2.rectangle(
        frame,
        (x1, y1),
        (x2, y2),
        (0, 255, 0),
        2,
    )

    cv2.circle(
        frame,
        (center_x, center_y),
        4,
        (0, 0, 255),
        -1,
    )

    cv2.drawMarker(
        frame,
        (center_x, center_y),
        (0, 0, 255),
        cv2.MARKER_CROSS,
        14,
        1,
        cv2.LINE_AA,
    )

    draw_label_block(frame, x1, y1, label_lines)


def draw_track(frame, track) -> None:
    x1, y1, x2, y2 = track.bbox
    center_x, center_y = track.center
    label_lines = [
        f"ID {track.track_id} {track.class_name} {track.confidence:.2f}",
        f"bbox ({x1}, {y1})-({x2}, {y2})",
        f"centre ({center_x}, {center_y})",
    ]

    draw_bbox_overlay(frame, track.bbox, track.center, label_lines)


def draw_tracks(frame, tracks) -> None:
    for track in tracks:
        draw_track(frame, track)


def draw_detections(
    frame,
    result,
    class_names,
    displayed_class_name="person",
) -> None:
    """
    Draw boxes manually instead of calling result.plot().

    Manual drawing gives more control and avoids some additional plotting
    overhead.
    """

    detections = extract_detections(
        result,
        class_names,
        displayed_class_name,
    )

    for detection in detections:
        x1, y1, x2, y2 = detection.bbox
        center_x, center_y = detection.center
        label_lines = [
            f"{detection.class_name} {detection.confidence:.2f}",
            f"bbox ({x1}, {y1})-({x2}, {y2})",
            f"centre ({center_x}, {center_y})",
        ]
        draw_bbox_overlay(frame, detection.bbox, detection.center, label_lines)


def draw_status(
    frame,
    inference_ms: float,
    display_fps: float,
    detection_count: int,
) -> None:
    status = (
        f"Inference: {inference_ms:.1f} ms  "
        f"Display: {display_fps:.1f} FPS  "
        f"Objects: {detection_count}"
    )

    cv2.putText(
        frame,
        status,
        (10, 25),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (0, 255, 255),
        2,
        cv2.LINE_AA,
    )

def get_frame_centre_point(frame):
    height, width = frame.shape[:2]
    return (width // 2, height // 2)