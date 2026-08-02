from .bootstrap import ensure_runtime_environment


ensure_runtime_environment()

import cv2


def get_class_name(class_names, class_id: int) -> str:
    if hasattr(class_names, "get"):
        return class_names.get(class_id, str(class_id))

    try:
        return class_names[class_id]
    except (IndexError, KeyError, TypeError):
        return str(class_id)


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

    if result.boxes is None or len(result.boxes) == 0:
        return

    boxes = result.boxes.xyxy.cpu().numpy()
    confidences = result.boxes.conf.cpu().numpy()
    class_ids = result.boxes.cls.cpu().numpy().astype(int)

    for box, confidence, class_id in zip(
        boxes,
        confidences,
        class_ids,
    ):
        x1, y1, x2, y2 = box.astype(int)

        class_name = get_class_name(class_names, class_id)
        if class_name != displayed_class_name:
            continue

        label = f"{class_name} {confidence:.2f}"

        cv2.rectangle(
            frame,
            (x1, y1),
            (x2, y2),
            (0, 255, 0),
            2,
        )

        text_size, baseline = cv2.getTextSize(
            label,
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            1,
        )

        text_width, text_height = text_size
        text_y = max(y1, text_height + baseline + 2)

        cv2.rectangle(
            frame,
            (x1, text_y - text_height - baseline - 4),
            (x1 + text_width + 4, text_y),
            (0, 255, 0),
            -1,
        )

        cv2.putText(
            frame,
            label,
            (x1 + 2, text_y - baseline - 2),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 0, 0),
            1,
            cv2.LINE_AA,
        )


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
