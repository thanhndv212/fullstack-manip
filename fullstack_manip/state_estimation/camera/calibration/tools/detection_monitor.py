#!/usr/bin/env python3
"""Visualize real-time ArUco marker detections from a webcam feed."""

from __future__ import annotations

import argparse
import time
from typing import Iterable, Optional, Set, Tuple

import cv2

DEFAULT_DICT_NAME = "DICT_4X4_50"


def get_detector(
    dictionary_name: str = DEFAULT_DICT_NAME,
) -> Tuple[object, object, Optional[cv2.aruco.ArucoDetector]]:
    """Return an ArUco dictionary, parameters, and optional detector."""

    dict_attr = getattr(cv2.aruco, dictionary_name)
    try:
        dictionary = cv2.aruco.getPredefinedDictionary(dict_attr)
        parameters = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(dictionary, parameters)
        print("✅ Using OpenCV 4.7+ ArUco API")
        return dictionary, parameters, detector
    except AttributeError:  # OpenCV < 4.7
        dictionary = cv2.aruco.Dictionary_get(dict_attr)
        parameters = cv2.aruco.DetectorParameters_create()
        print("✅ Using legacy ArUco API")
        return dictionary, parameters, None


def parse_args() -> argparse.Namespace:
    """Parse CLI arguments for the detection monitor."""

    parser = argparse.ArgumentParser(description="ArUco detection monitor")
    parser.add_argument(
        "--camera",
        type=int,
        default=0,
        help="Camera device ID",
    )
    parser.add_argument(
        "--dictionary",
        default=DEFAULT_DICT_NAME,
        help="OpenCV aruco dictionary name (e.g., DICT_4X4_50)",
    )
    parser.add_argument(
        "--highlight",
        type=int,
        nargs="*",
        default=(0,),
        help="Marker IDs to highlight when detected",
    )
    return parser.parse_args()


def format_ids(ids: Iterable[int]) -> str:
    """Return a compact string representation of detected IDs."""

    ids = list(ids)
    if not ids:
        return "[]"
    return "[" + ", ".join(str(marker_id) for marker_id in ids) + "]"


def run_detection_monitor(
    camera_id: int,
    dictionary_name: str,
    highlight_ids: Iterable[int],
) -> None:
    """Run the visualization loop until the user quits."""

    highlight_set: Set[int] = set(highlight_ids)
    dictionary, parameters, detector = get_detector(dictionary_name)

    cap = cv2.VideoCapture(camera_id)
    if not cap.isOpened():
        print(f"❌ Error: Could not open camera {camera_id}")
        return

    print("📹 Camera opened successfully")
    print("🎯 Looking for ArUco markers...")

    frame_count = 0
    last_detection_time: Optional[float] = None

    while True:
        ret, frame = cap.read()
        if not ret:
            print("❌ Error: Failed to read frame from camera")
            break

        frame_count += 1
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        if detector is not None:
            corners, ids, _ = detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(
                gray,
                dictionary,
                parameters=parameters,
            )

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            detected_ids = [marker_id[0] for marker_id in ids]
            last_detection_time = time.time()

            status_text = f"DETECTED: {len(detected_ids)} markers"
            cv2.putText(
                frame,
                status_text,
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 0),
                2,
            )

            ids_text = f"IDs: {format_ids(detected_ids)}"
            cv2.putText(
                frame,
                ids_text,
                (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2,
            )

            if highlight_set.intersection(detected_ids):
                cv2.putText(
                    frame,
                    "🎯 TARGET MARKER DETECTED",
                    (10, 90),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 255),
                    2,
                )
        else:
            cv2.putText(
                frame,
                "No markers detected",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 0, 255),
                2,
            )

        if last_detection_time is not None:
            time_since = time.time() - last_detection_time
            status_colour = (0, 255, 0) if time_since < 1.0 else (0, 165, 255)
            last_seen_text = f"Last seen: {time_since:.1f}s ago"
        else:
            status_colour = (0, 0, 255)
            last_seen_text = "Never detected"

        cv2.putText(
            frame,
            last_seen_text,
            (10, frame.shape[0] - 60),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            status_colour,
            2,
        )

        instructions = "Hold highlighted markers in front of the camera"
        cv2.putText(
            frame,
            instructions,
            (10, frame.shape[0] - 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 255),
            1,
        )

        cv2.putText(
            frame,
            f"Frame: {frame_count}",
            (frame.shape[1] - 160, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            1,
        )

        cv2.imshow("ArUco Detection Monitor", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()
    print("✨ Detection monitor closed")


def main() -> None:  # pragma: no cover - CLI utility
    args = parse_args()
    run_detection_monitor(args.camera, args.dictionary, args.highlight)


if __name__ == "__main__":  # pragma: no cover
    main()
