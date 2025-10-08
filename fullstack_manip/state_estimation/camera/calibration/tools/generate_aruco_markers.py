#!/usr/bin/env python3
"""Generate ArUco markers and pattern sheets for calibration workflows."""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Iterable, Tuple

import cv2
import matplotlib.pyplot as plt
import numpy as np

PACKAGE_DIR = Path(__file__).resolve().parents[1]
DEFAULT_PATTERN_DIR = PACKAGE_DIR / "patterns"
DEFAULT_PATTERN_DIR.mkdir(parents=True, exist_ok=True)

DEFAULT_DICT_NAME = "DICT_6X6_250"


def get_aruco_dictionary(
    dict_name: str = DEFAULT_DICT_NAME,
) -> Tuple[object, bool]:
    """Return the requested ArUco dictionary and API compatibility flag."""

    dict_attr = getattr(cv2.aruco, dict_name)
    try:
        dictionary = cv2.aruco.getPredefinedDictionary(dict_attr)
        return dictionary, True
    except AttributeError:  # OpenCV < 4.7
        dictionary = cv2.aruco.Dictionary_get(dict_attr)
        return dictionary, False


def _generate_single_marker(
    dictionary: object,
    marker_id: int,
    marker_size: int,
    modern_api: bool,
) -> np.ndarray:
    """Generate a marker image compatible with the installed OpenCV API."""

    if modern_api:
        return cv2.aruco.generateImageMarker(
            dictionary,
            marker_id,
            marker_size,
        )
    return cv2.aruco.drawMarker(dictionary, marker_id, marker_size)


def generate_aruco_marker(
    marker_id: int = 0,
    marker_size: int = 200,
    save_path: Path | None = None,
    display: bool = True,
    dict_name: str = DEFAULT_DICT_NAME,
) -> np.ndarray:
    """Generate a single ArUco marker, optionally saving and displaying it."""

    dictionary, modern_api = get_aruco_dictionary(dict_name)
    marker = _generate_single_marker(
        dictionary,
        marker_id,
        marker_size,
        modern_api,
    )

    if save_path is not None:
        save_path = Path(save_path)
        save_path.parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(save_path), marker)
        print(f"💾 Marker saved to: {save_path}")

    if display:
        plt.figure(figsize=(4, 4))
        plt.imshow(marker, cmap="gray")
        title = f"ArUco Marker ID: {marker_id}\nSize: {marker_size}px"
        plt.title(title)
        plt.axis("off")
        plt.figtext(
            0.5,
            0.02,
            "Print or display this marker for calibration (≈5cm square).",
            ha="center",
            fontsize=9,
            style="italic",
        )
        plt.tight_layout()
        plt.show()

    return marker


def generate_marker_sheet(
    marker_ids: Iterable[int],
    marker_size: int = 200,
    padding: int = 25,
    dict_name: str = DEFAULT_DICT_NAME,
    save_dir: Path | None = None,
) -> Path:
    """Generate a grid of markers and return the saved sheet path."""

    marker_ids = list(marker_ids)
    if not marker_ids:
        raise ValueError("marker_ids must contain at least one ID")

    save_dir = Path(save_dir) if save_dir else DEFAULT_PATTERN_DIR
    save_dir.mkdir(parents=True, exist_ok=True)

    dictionary, modern_api = get_aruco_dictionary(dict_name)
    grid_dim = int(np.ceil(np.sqrt(len(marker_ids))))
    cell_size = marker_size + (2 * padding)
    sheet_size = cell_size * grid_dim
    sheet = np.ones((sheet_size, sheet_size), dtype=np.uint8) * 255

    for idx, marker_id in enumerate(marker_ids):
        row = idx // grid_dim
        col = idx % grid_dim
        marker = _generate_single_marker(
            dictionary,
            marker_id,
            marker_size,
            modern_api,
        )
        y_start = row * cell_size + padding
        x_start = col * cell_size + padding
        y_end = y_start + marker_size
        x_end = x_start + marker_size
        sheet[y_start:y_end, x_start:x_end] = marker
        label_pos = (x_start, y_start - 5)
        cv2.putText(
            sheet,
            f"ID {marker_id}",
            label_pos,
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            0,
            2,
        )

    sheet_path = save_dir / "aruco_marker_sheet.png"
    cv2.imwrite(str(sheet_path), sheet)
    print(f"📄 Combined marker sheet saved to: {sheet_path}")

    return sheet_path


def generate_marker_set(
    num_markers: int = 4,
    marker_size: int = 200,
    dict_name: str = DEFAULT_DICT_NAME,
    save_dir: Path | None = None,
) -> Path:
    """Backward compatible wrapper using sequential marker IDs."""

    marker_ids = range(num_markers)
    return generate_marker_sheet(
        marker_ids=marker_ids,
        marker_size=marker_size,
        dict_name=dict_name,
        save_dir=save_dir,
    )


def parse_args() -> argparse.Namespace:
    """Parse CLI arguments for marker generation."""

    parser = argparse.ArgumentParser(
        description="ArUco marker generation tool",
    )
    parser.add_argument(
        "--marker-id",
        type=int,
        default=0,
        help="Marker ID to create",
    )
    parser.add_argument(
        "--size",
        type=int,
        default=400,
        help="Marker size in pixels",
    )
    parser.add_argument(
        "--dict",
        default=DEFAULT_DICT_NAME,
        help="OpenCV aruco dictionary name (e.g., DICT_6X6_250)",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="Optional explicit save path for the single marker",
    )
    parser.add_argument(
        "--sheet",
        type=int,
        default=0,
        help="Number of sequential markers for a sheet",
    )
    parser.add_argument(
        "--no-display",
        action="store_true",
        help="Skip rendering windows (useful on headless systems)",
    )
    return parser.parse_args()


def main() -> None:  # pragma: no cover - CLI utility
    args = parse_args()
    display = not args.no_display

    save_path = args.output
    if save_path is None:
        file_name = f"aruco_marker_{args.marker_id:02d}.png"
        save_path = DEFAULT_PATTERN_DIR / file_name

    print("🎯 Generating ArUco marker")
    generate_aruco_marker(
        marker_id=args.marker_id,
        marker_size=args.size,
        save_path=save_path,
        display=display,
        dict_name=args.dict,
    )

    if args.sheet > 0:
        marker_ids = range(args.marker_id, args.marker_id + args.sheet)
        print(f"📋 Building sheet for marker IDs {list(marker_ids)}")
        generate_marker_sheet(
            marker_ids=marker_ids,
            marker_size=args.size,
            dict_name=args.dict,
            save_dir=DEFAULT_PATTERN_DIR,
        )


if __name__ == "__main__":  # pragma: no cover
    main()
