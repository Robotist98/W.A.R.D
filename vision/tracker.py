import math
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Tuple


BBox = Tuple[int, int, int, int]
Point = Tuple[int, int]


@dataclass(frozen=True)
class Detection:
    bbox: BBox
    confidence: float
    class_id: int
    class_name: str
    center: Point


@dataclass
class Track:
    track_id: int
    bbox: BBox
    confidence: float
    class_id: int
    class_name: str
    center: Point
    age: int = 1
    missed_frames: int = 0

    def update(self, detection: Detection) -> None:
        self.bbox = detection.bbox
        self.confidence = detection.confidence
        self.class_id = detection.class_id
        self.class_name = detection.class_name
        self.center = detection.center
        self.age += 1
        self.missed_frames = 0


class CentroidTracker:
    """
    Keeps stable object IDs by matching bbox centres between frames.

    This is intentionally lightweight: YOLO handles detection, while this class
    only associates detections frame-to-frame.
    """

    def __init__(
        self,
        max_distance: float = 120.0,
        max_missed_frames: int = 10,
    ):
        self.max_distance = max_distance
        self.max_missed_frames = max_missed_frames
        self.tracks: Dict[int, Track] = {}
        self.next_track_id = 1

    def update(self, detections: Iterable[Detection]) -> List[Track]:
        detections = list(detections)

        if not detections:
            self._mark_all_missed()
            self._drop_lost_tracks()
            return []

        if not self.tracks:
            return [self._create_track(detection) for detection in detections]

        assignments = self._match_detections(detections)
        matched_track_ids = set()
        matched_detection_indexes = set()
        visible_tracks = []

        for track_id, detection_index in assignments:
            track = self.tracks[track_id]
            track.update(detections[detection_index])
            matched_track_ids.add(track_id)
            matched_detection_indexes.add(detection_index)
            visible_tracks.append(track)

        for track_id, track in self.tracks.items():
            if track_id not in matched_track_ids:
                track.missed_frames += 1

        for detection_index, detection in enumerate(detections):
            if detection_index not in matched_detection_indexes:
                visible_tracks.append(self._create_track(detection))

        self._drop_lost_tracks()
        visible_tracks.sort(key=lambda track: track.track_id)
        return visible_tracks

    def priority_target(
        self,
        tracks: Optional[Iterable[Track]] = None,
        class_name: Optional[str] = None,
    ) -> Optional[int]:
        if tracks is None:
            tracks = self.tracks.values()

        return priority_target(tracks, class_name=class_name)

    def _create_track(self, detection: Detection) -> Track:
        track = Track(
            track_id=self.next_track_id,
            bbox=detection.bbox,
            confidence=detection.confidence,
            class_id=detection.class_id,
            class_name=detection.class_name,
            center=detection.center,
        )
        self.tracks[track.track_id] = track
        self.next_track_id += 1
        return track

    def _match_detections(self, detections: List[Detection]):
        candidates = []

        for track_id, track in self.tracks.items():
            for detection_index, detection in enumerate(detections):
                distance = center_distance(track.center, detection.center)
                if distance <= self.max_distance:
                    candidates.append((distance, track_id, detection_index))

        candidates.sort(key=lambda candidate: candidate[0])

        used_track_ids = set()
        used_detection_indexes = set()
        assignments = []

        for _, track_id, detection_index in candidates:
            if track_id in used_track_ids:
                continue

            if detection_index in used_detection_indexes:
                continue

            assignments.append((track_id, detection_index))
            used_track_ids.add(track_id)
            used_detection_indexes.add(detection_index)

        return assignments

    def _mark_all_missed(self) -> None:
        for track in self.tracks.values():
            track.missed_frames += 1

    def _drop_lost_tracks(self) -> None:
        lost_track_ids = [
            track_id
            for track_id, track in self.tracks.items()
            if track.missed_frames > self.max_missed_frames
        ]

        for track_id in lost_track_ids:
            del self.tracks[track_id]


def center_distance(first: Point, second: Point) -> float:
    return math.hypot(first[0] - second[0], first[1] - second[1])


def bbox_area(bbox: BBox) -> int:
    x1, y1, x2, y2 = bbox
    return max(0, x2 - x1) * max(0, y2 - y1)


def priority_target(
    tracks: Iterable[Track],
    class_name: Optional[str] = None,
) -> Optional[int]:
    tracks = [
        track
        for track in tracks
        if class_name is None or track.class_name == class_name
    ]

    if not tracks:
        return None

    return max(tracks, key=lambda track: bbox_area(track.bbox)).track_id
