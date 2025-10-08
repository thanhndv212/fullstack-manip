#!/usr/bin/env python3
"""
Camera Calibration Tool for FIGAROH System
Determines camera matrix and distortion coefficients using chessboard patterns
"""

import argparse
import json
import pickle
from datetime import datetime
from pathlib import Path
from typing import Tuple

import cv2
import numpy as np

# Default output directory inside the package
DEFAULT_OUTPUT_DIR = Path(__file__).resolve().parents[1] / "data"
DEFAULT_OUTPUT_DIR.mkdir(parents=True, exist_ok=True)


class CameraCalibrator:
    def __init__(self, chessboard_size: Tuple[int, int] = (10, 7),
                 square_size: float = 0.025):
        """
        Initialize camera calibrator

        Args:
            chessboard_size: (cols, rows) of inner corners in chessboard
            square_size: Size of each square in meters (default 2.5cm)
        """
        self.chessboard_size = chessboard_size
        self.square_size = square_size

        # Prepare object points (3D points in real world space)
        num_corners = chessboard_size[0] * chessboard_size[1]
        self.objp = np.zeros((num_corners, 3), np.float32)
        corners_grid = np.mgrid[0:chessboard_size[0],
                                0:chessboard_size[1]].T.reshape(-1, 2)
        self.objp[:, :2] = corners_grid
        self.objp *= square_size

        # Arrays to store object points and image points from all images
        self.objpoints = []  # 3D points in real world space
        self.imgpoints = []  # 2D points in image plane

        # Camera properties
        self.camera_matrix = None
        self.dist_coeffs = None
        self.image_size = None

        print("🎯 Initialized calibrator for "
              f"{chessboard_size[0]}x{chessboard_size[1]} chessboard")
        print(f"📏 Square size: {square_size * 1000:.1f}mm")

    def capture_calibration_images(self, num_images: int = 20,
                                   camera_id: int = 0) -> bool:
        """
        Capture calibration images from camera

        Args:
            num_images: Number of images to capture
            camera_id: Camera device ID

        Returns:
            bool: Success status
        """
        cap = cv2.VideoCapture(camera_id)
        if not cap.isOpened():
            print(f"❌ Error: Could not open camera {camera_id}")
            return False

        # Set camera resolution (adjust as needed for your camera)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

        captured_count = 0
        print("\n📸 Starting calibration image capture...")
        print(f"🎯 Target: {num_images} good chessboard detections")
        print("📋 Instructions:")
        print("   - Hold the chessboard pattern in view")
        print("   - Move it to different positions and angles")
        print("   - Press SPACE when corners are detected (green)")
        print("   - Press 'q' to quit early")
        print("   - Ensure good lighting and focus")

        while captured_count < num_images:
            ret, frame = cap.read()
            if not ret:
                print("❌ Error reading from camera")
                break

            # Convert to grayscale
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # Find chessboard corners
            flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_FAST_CHECK
            flags |= cv2.CALIB_CB_NORMALIZE_IMAGE
            ret_corners, corners = cv2.findChessboardCorners(
                gray, self.chessboard_size, flags)

            # Draw and display
            frame_display = frame.copy()
            if ret_corners:
                # Refine corner positions
                criteria_flags = (
                    cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER
                )
                criteria = (criteria_flags, 30, 0.001)
                corners_refined = cv2.cornerSubPix(
                    gray, corners, (11, 11), (-1, -1), criteria)

                # Draw corners
                cv2.drawChessboardCorners(
                    frame_display, self.chessboard_size,
                    corners_refined, ret_corners)

                # Add status text
                text = ("GOOD - Press SPACE to capture "
                        f"({captured_count}/{num_images})")
                cv2.putText(frame_display, text, (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            else:
                text = (f"No chessboard detected "
                        f"({captured_count}/{num_images})")
                cv2.putText(frame_display, text, (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            # Instructions
            instruction_y = frame_display.shape[0] - 60
            cv2.putText(frame_display,
                        "Move chessboard to different positions",
                        (10, instruction_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (255, 255, 255), 1)
            cv2.putText(frame_display, "SPACE: capture, Q: quit",
                        (10, frame_display.shape[0] - 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

            cv2.imshow('Camera Calibration', frame_display)

            key = cv2.waitKey(1) & 0xFF
            if key == ord(' ') and ret_corners:
                # Capture this image
                self.objpoints.append(self.objp)
                self.imgpoints.append(corners_refined)
                captured_count += 1
                self.image_size = gray.shape[::-1]  # (width, height)
                print(f"✅ Captured image {captured_count}/{num_images}")

                # Brief pause to prevent accidental multiple captures
                cv2.waitKey(500)

            elif key == ord('q'):
                print("🛑 Calibration stopped early with "
                      f"{captured_count} images")
                break

        cap.release()
        cv2.destroyAllWindows()

        if captured_count >= 9:  # Minimum for reasonable calibration
            print(f"✅ Captured {captured_count} calibration images")
            return True
        else:
            print(f"❌ Insufficient images captured ({captured_count}). "
                  "Need at least 10.")
            return False

    def calibrate_camera(self) -> Tuple[float, np.ndarray, np.ndarray]:
        """
        Perform camera calibration

        Returns:
            tuple: (reprojection_error, camera_matrix, dist_coeffs)
        """
        if len(self.objpoints) < 10:
            raise ValueError("Need at least 10 calibration images")

        print("\n🔧 Performing camera calibration with "
              f"{len(self.objpoints)} images...")

        # Perform calibration
        ret, self.camera_matrix, self.dist_coeffs, rvecs, tvecs = (
            cv2.calibrateCamera(
                self.objpoints, self.imgpoints, self.image_size, None, None
            )
        )

        print("✅ Calibration completed!")
        print(f"📊 Reprojection error: {ret:.4f} pixels")

        # Evaluate calibration quality
        if ret < 0.5:
            print("🌟 Excellent calibration quality!")
        elif ret < 1.0:
            print("✅ Good calibration quality")
        elif ret < 2.0:
            print("⚠️  Acceptable calibration quality")
        else:
            print("❌ Poor calibration quality - consider recalibrating")

        return ret, self.camera_matrix, self.dist_coeffs

    def save_calibration(self, filename: str = None) -> str:
        """
        Save calibration results to files

        Args:
            filename: Base filename (without extension)

        Returns:
            str: Path to saved file
        """
        if filename is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = DEFAULT_OUTPUT_DIR / f"camera_calibration_{timestamp}"
        else:
            filename = Path(filename)
            if not filename.is_absolute():
                filename = DEFAULT_OUTPUT_DIR / filename

        filename.parent.mkdir(parents=True, exist_ok=True)

        # Prepare calibration data
        reprojection_error = cv2.calibrateCamera(
            self.objpoints, self.imgpoints, self.image_size,
            self.camera_matrix, self.dist_coeffs,
            flags=cv2.CALIB_USE_INTRINSIC_GUESS
        )[0]

        calibration_data = {
            'camera_matrix': self.camera_matrix.tolist(),
            'dist_coeffs': self.dist_coeffs.tolist(),
            'image_size': self.image_size,
            'chessboard_size': self.chessboard_size,
            'square_size': self.square_size,
            'num_images': len(self.objpoints),
            'timestamp': datetime.now().isoformat(),
            'reprojection_error': float(reprojection_error)
        }

        # Save as JSON (human readable)
        json_file = f"{filename}.json"
        with open(json_file, 'w') as f:
            json.dump(calibration_data, f, indent=2)

        # Save as pickle (for easy loading in Python)
        pickle_file = f"{filename}.pkl"
        with open(pickle_file, 'wb') as f:
            pickle.dump(calibration_data, f)

        # Save camera matrix and distortion coefficients in FIGAROH format
        figaroh_file = f"{filename}_figaroh.py"
        with open(figaroh_file, 'w') as f:
            f.write("# Camera calibration parameters for FIGAROH\n")
            timestamp_str = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            f.write(f"# Generated on: {timestamp_str}\n\n")
            f.write("import numpy as np\n\n")
            f.write("# Camera intrinsic matrix\n")
            matrix_str = str(self.camera_matrix.tolist())
            f.write(f"camera_matrix = np.array({matrix_str}, "
                    "dtype=np.float32)\n\n")
            f.write("# Distortion coefficients\n")
            dist_str = str(self.dist_coeffs.flatten().tolist())
            f.write(f"dist_coeffs = np.array({dist_str}, "
                    "dtype=np.float32)\n\n")
            f.write("# Image size (width, height)\n")
            f.write(f"image_size = {self.image_size}\n\n")
            f.write("# Calibration info\n")
            error_val = calibration_data['reprojection_error']
            f.write(f"reprojection_error = {error_val:.4f}  # pixels\n")
            f.write(f"num_calibration_images = {len(self.objpoints)}\n")

        print("\n💾 Calibration saved:")
        print(f"   📄 JSON: {json_file}")
        print(f"   🐍 Pickle: {pickle_file}")
        print(f"   🤖 FIGAROH: {figaroh_file}")

        return json_file

    def load_calibration(self, filename: str) -> bool:
        """
        Load calibration from file

        Args:
            filename: Path to calibration file (JSON or pickle)

        Returns:
            bool: Success status
        """
        try:
            if filename.endswith('.json'):
                with open(filename, 'r') as f:
                    data = json.load(f)
            elif filename.endswith('.pkl'):
                with open(filename, 'rb') as f:
                    data = pickle.load(f)
            else:
                raise ValueError("File must be .json or .pkl")

            self.camera_matrix = np.array(data['camera_matrix'],
                                          dtype=np.float32)
            self.dist_coeffs = np.array(data['dist_coeffs'],
                                        dtype=np.float32)
            self.image_size = tuple(data['image_size'])

            print(f"✅ Loaded calibration from {filename}")
            return True
        except Exception as e:
            print(f"❌ Error loading calibration: {e}")
            return False

    def test_calibration(self, camera_id: int = 0):
        """
        Test calibration by showing undistorted camera feed

        Args:
            camera_id: Camera device ID
        """
        if self.camera_matrix is None:
            print("❌ No calibration data loaded")
            return

        cap = cv2.VideoCapture(camera_id)
        if not cap.isOpened():
            print(f"❌ Error: Could not open camera {camera_id}")
            return

        print("\n🧪 Testing calibration...")
        print("📋 Instructions:")
        print("   - Left side: Original image")
        print("   - Right side: Undistorted image")
        print("   - Press 'q' to quit")

        # Get optimal new camera matrix
        h, w = self.image_size[1], self.image_size[0]
        new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(
            self.camera_matrix, self.dist_coeffs, (w, h), 1, (w, h)
        )

        while True:
            ret, frame = cap.read()
            if not ret:
                break

            # Undistort image
            undistorted = cv2.undistort(frame, self.camera_matrix,
                                        self.dist_coeffs, None,
                                        new_camera_matrix)

            # Create side-by-side comparison
            h, w = frame.shape[:2]
            comparison = np.zeros((h, w * 2, 3), dtype=np.uint8)
            comparison[:, :w] = frame
            comparison[:, w:] = undistorted

            # Add labels
            cv2.putText(comparison, "Original", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            cv2.putText(comparison, "Undistorted", (w + 10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            cv2.imshow('Calibration Test', comparison)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cap.release()
        cv2.destroyAllWindows()

    def print_calibration_summary(self):
        """Print calibration parameters summary"""
        if self.camera_matrix is None:
            print("❌ No calibration data available")
            return

        print("\n" + "=" * 60)
        print("📷 CAMERA CALIBRATION SUMMARY")
        print("=" * 60)

        # Camera matrix
        print("\n🎯 Camera Matrix (K):")
        print(f"   fx = {self.camera_matrix[0, 0]:.2f} pixels")
        print(f"   fy = {self.camera_matrix[1, 1]:.2f} pixels")
        print(f"   cx = {self.camera_matrix[0, 2]:.2f} pixels")
        print(f"   cy = {self.camera_matrix[1, 2]:.2f} pixels")

        # Distortion coefficients
        print("\n🔧 Distortion Coefficients:")
        if len(self.dist_coeffs) >= 5:
            k1, k2, p1, p2, k3 = self.dist_coeffs.flatten()[:5]
            print(f"   k1 = {k1:.6f} (radial)")
            print(f"   k2 = {k2:.6f} (radial)")
            print(f"   p1 = {p1:.6f} (tangential)")
            print(f"   p2 = {p2:.6f} (tangential)")
            print(f"   k3 = {k3:.6f} (radial)")

        # Field of view estimation
        fx, fy = self.camera_matrix[0, 0], self.camera_matrix[1, 1]
        w, h = self.image_size
        fov_x = 2 * np.arctan(w / (2 * fx)) * 180 / np.pi
        fov_y = 2 * np.arctan(h / (2 * fy)) * 180 / np.pi
        print("\n📐 Estimated Field of View:")
        print(f"   Horizontal: {fov_x:.1f}°")
        print(f"   Vertical:   {fov_y:.1f}°")

        print(f"\n📊 Image Resolution: {w} x {h}")
        print("=" * 60)


def parse_size(value: str) -> Tuple[int, int]:
    """Parse pattern size arguments formatted as 'cols,rows'."""

    try:
        cols, rows = value.split(',')
        return int(cols), int(rows)
    except Exception as exc:  # pragma: no cover - user input helper
        raise argparse.ArgumentTypeError(
            "Pattern size must be formatted as 'cols,rows'"
        ) from exc


def main():  # pragma: no cover - CLI utility
    parser = argparse.ArgumentParser(description="Camera calibration tool")
    parser.add_argument('--camera', type=int, default=0,
                        help='Camera device ID (default: 0)')
    parser.add_argument(
        '--pattern-size',
        type=parse_size,
        default="10,7",
        help='Chessboard pattern size (cols,rows). Default: 10,7',
    )
    parser.add_argument('--square-size', type=float, default=0.025,
                        help='Square size in meters. Default: 0.025 (2.5cm)')
    parser.add_argument(
        '--images',
        type=int,
        default=20,
        help='Number of calibration images to capture (default: 20)',
    )
    parser.add_argument(
        '--output',
        type=str,
        help='Base filename for calibration output (no extension)',
    )
    parser.add_argument('--test-only', action='store_true',
                        help='Skip capture and just test existing calibration')
    parser.add_argument('--calibration-file', type=str,
                        help='Path to existing calibration file (JSON/PKL)')

    args = parser.parse_args()

    calibrator = CameraCalibrator(
        chessboard_size=args.pattern_size,
        square_size=args.square_size
    )

    if args.test_only:
        if not args.calibration_file:
            raise SystemExit("--calibration-file is required with --test-only")

        if calibrator.load_calibration(args.calibration_file):
            calibrator.print_calibration_summary()
            calibrator.test_calibration(args.camera)
        return

    success = calibrator.capture_calibration_images(
        num_images=args.images,
        camera_id=args.camera
    )

    if not success:
        raise SystemExit("Not enough calibration images collected")

    calibrator.calibrate_camera()
    calibrator.print_calibration_summary()
    saved_path = calibrator.save_calibration(args.output)
    print(f"\n✅ Calibration stored at: {saved_path}")


if __name__ == "__main__":  # pragma: no cover - CLI utility
    main()
