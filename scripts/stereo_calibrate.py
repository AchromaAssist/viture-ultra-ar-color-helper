#!/usr/bin/env python3
"""
Stereo Camera Calibration Script

Computes intrinsic and extrinsic parameters for stereo cameras
using checkerboard images captured with calib_capture tool.

Usage:
    python3 stereo_calibrate.py [options] /tmp/calib_left_*.pgm

Options:
    --cols N      Number of inner corners horizontally (default: 8)
    --rows N      Number of inner corners vertically (default: 6)
    --square N    Square size in mm (default: 25)

Examples:
    # 9x7 squares checkerboard (8x6 inner corners), 25mm squares:
    python3 stereo_calibrate.py --cols 8 --rows 6 --square 25 /tmp/calib_left_*.pgm

Outputs calibration parameters for use in stereo_depth.h
"""

import sys
import glob
import argparse
import numpy as np
import cv2

def find_checkerboard_corners(img_path, checkerboard_size, show_progress=True):
    """Find checkerboard corners in an image."""
    img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        print(f"  Failed to load: {img_path}")
        return None, None

    # Find corners
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
    ret, corners = cv2.findChessboardCorners(img, checkerboard_size, flags)

    if ret:
        # Refine corners
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        corners = cv2.cornerSubPix(img, corners, (11, 11), (-1, -1), criteria)
        return corners, img.shape[::-1]  # (width, height)

    return None, None

def main():
    parser = argparse.ArgumentParser(
        description='Stereo camera calibration using checkerboard images',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='''
Examples:
  # 9x7 squares (8x6 inner corners), 25mm squares:
  python3 stereo_calibrate.py --cols 8 --rows 6 --square 25 /tmp/calib_left_*.pgm

  # 10x7 squares (9x6 inner corners), 30mm squares:
  python3 stereo_calibrate.py --cols 9 --rows 6 --square 30 /tmp/calib_left_*.pgm
'''
    )
    parser.add_argument('--cols', type=int, default=8,
                        help='Inner corners horizontally (default: 8 for 9x? squares)')
    parser.add_argument('--rows', type=int, default=6,
                        help='Inner corners vertically (default: 6 for ?x7 squares)')
    parser.add_argument('--square', type=float, default=25.0,
                        help='Square size in mm (default: 25)')
    parser.add_argument('files', nargs='*',
                        help='Left image files or glob pattern (default: /tmp/calib_left_*.pgm)')

    args = parser.parse_args()

    CHECKERBOARD = (args.cols, args.rows)
    SQUARE_SIZE = args.square

    # Find all left images
    if args.files:
        # Files provided on command line (may already be expanded by shell)
        left_files = sorted(args.files)
    else:
        # Use default pattern
        left_files = sorted(glob.glob('/tmp/calib_left_*.pgm'))

    if not left_files:
        print(f"No files found matching: {args.pattern}")
        sys.exit(1)

    print(f"Found {len(left_files)} left images")
    print(f"Checkerboard: {CHECKERBOARD[0]}x{CHECKERBOARD[1]} inner corners ({CHECKERBOARD[0]+1}x{CHECKERBOARD[1]+1} squares)")
    print(f"Square size: {SQUARE_SIZE} mm\n")

    # Prepare object points (3D points in real world space)
    objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
    objp *= SQUARE_SIZE

    # Storage for calibration data
    obj_points = []  # 3D points
    img_points_left = []  # 2D points in left images
    img_points_right = []  # 2D points in right images
    img_size = None

    # Process each stereo pair
    for left_file in left_files:
        # Derive right filename
        right_file = left_file.replace('_left_', '_right_')

        print(f"Processing: {left_file}")

        # Find corners in left image
        corners_left, size_left = find_checkerboard_corners(left_file, CHECKERBOARD)
        if corners_left is None:
            print("  No checkerboard found in left image, skipping")
            continue

        # Find corners in right image
        corners_right, size_right = find_checkerboard_corners(right_file, CHECKERBOARD)
        if corners_right is None:
            print("  No checkerboard found in right image, skipping")
            continue

        if size_left != size_right:
            print("  Image sizes don't match, skipping")
            continue

        img_size = size_left
        obj_points.append(objp)
        img_points_left.append(corners_left)
        img_points_right.append(corners_right)
        print(f"  Found checkerboard in both images!")

    print(f"\n{len(obj_points)} valid stereo pairs found")

    if len(obj_points) < 5:
        print("Need at least 5 valid pairs for calibration!")
        sys.exit(1)

    print(f"Image size: {img_size}")
    print("\n" + "="*50)
    print("Running stereo calibration...")
    print("="*50 + "\n")

    # Calibrate left camera individually first
    print("Calibrating left camera...")
    ret_l, K_left, D_left, rvecs_l, tvecs_l = cv2.calibrateCamera(
        obj_points, img_points_left, img_size, None, None,
        flags=cv2.CALIB_FIX_K3
    )
    print(f"  RMS error: {ret_l:.4f}")

    # Calibrate right camera individually
    print("Calibrating right camera...")
    ret_r, K_right, D_right, rvecs_r, tvecs_r = cv2.calibrateCamera(
        obj_points, img_points_right, img_size, None, None,
        flags=cv2.CALIB_FIX_K3
    )
    print(f"  RMS error: {ret_r:.4f}")

    # Stereo calibration
    print("\nRunning stereo calibration...")
    flags = cv2.CALIB_FIX_INTRINSIC

    ret_stereo, K_left, D_left, K_right, D_right, R, T, E, F = cv2.stereoCalibrate(
        obj_points, img_points_left, img_points_right,
        K_left, D_left, K_right, D_right,
        img_size, flags=flags,
        criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-5)
    )

    print(f"  Stereo RMS error: {ret_stereo:.4f}")

    # Compute baseline
    baseline = np.linalg.norm(T)

    # Print results
    print("\n" + "="*50)
    print("CALIBRATION RESULTS")
    print("="*50)

    print("\n--- Left Camera Intrinsics ---")
    print(f"Focal length: fx={K_left[0,0]:.2f}, fy={K_left[1,1]:.2f}")
    print(f"Principal point: cx={K_left[0,2]:.2f}, cy={K_left[1,2]:.2f}")
    print(f"Distortion: k1={D_left[0,0]:.6f}, k2={D_left[0,1]:.6f}, p1={D_left[0,2]:.6f}, p2={D_left[0,3]:.6f}")

    print("\n--- Right Camera Intrinsics ---")
    print(f"Focal length: fx={K_right[0,0]:.2f}, fy={K_right[1,1]:.2f}")
    print(f"Principal point: cx={K_right[0,2]:.2f}, cy={K_right[1,2]:.2f}")
    print(f"Distortion: k1={D_right[0,0]:.6f}, k2={D_right[0,1]:.6f}, p1={D_right[0,2]:.6f}, p2={D_right[0,3]:.6f}")

    print("\n--- Stereo Extrinsics ---")
    print(f"Baseline: {baseline:.2f} mm")
    print(f"Translation: T = [{T[0,0]:.2f}, {T[1,0]:.2f}, {T[2,0]:.2f}]")

    # Compute rectification
    print("\nComputing rectification...")
    R1, R2, P1, P2, Q, roi1, roi2 = cv2.stereoRectify(
        K_left, D_left, K_right, D_right, img_size, R, T,
        flags=cv2.CALIB_ZERO_DISPARITY, alpha=0
    )

    # The focal length after rectification is in P1[0,0]
    focal_rectified = P1[0, 0]

    print("\n" + "="*50)
    print("VALUES FOR stereo_depth.h")
    print("="*50)
    print(f"""
// Calibrated stereo parameters for Viture Luma Ultra
#define STEREO_CALIB_DEFAULT {{ \\
    .baseline_mm = {baseline:.1f}f, \\
    .focal_px = {focal_rectified:.1f}f, \\
    .image_width = {img_size[0]}, \\
    .image_height = {img_size[1]} \\
}}

// Left camera intrinsics
// fx={K_left[0,0]:.2f}, fy={K_left[1,1]:.2f}
// cx={K_left[0,2]:.2f}, cy={K_left[1,2]:.2f}
// k1={D_left[0,0]:.6f}, k2={D_left[0,1]:.6f}

// Baseline: {baseline:.2f} mm
// Rectified focal length: {focal_rectified:.2f} px
""")

    # Save calibration data
    calib_file = "/tmp/stereo_calibration.npz"
    np.savez(calib_file,
             K_left=K_left, D_left=D_left,
             K_right=K_right, D_right=D_right,
             R=R, T=T, R1=R1, R2=R2, P1=P1, P2=P2, Q=Q,
             img_size=img_size)
    print(f"Full calibration saved to: {calib_file}")

    # Also save rectification maps for later use
    map1_left, map2_left = cv2.initUndistortRectifyMap(
        K_left, D_left, R1, P1, img_size, cv2.CV_32FC1)
    map1_right, map2_right = cv2.initUndistortRectifyMap(
        K_right, D_right, R2, P2, img_size, cv2.CV_32FC1)

    np.savez("/tmp/stereo_rectify_maps.npz",
             map1_left=map1_left, map2_left=map2_left,
             map1_right=map1_right, map2_right=map2_right)
    print("Rectification maps saved to: /tmp/stereo_rectify_maps.npz")

    print("\nDone!")

if __name__ == "__main__":
    main()
