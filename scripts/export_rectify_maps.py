#!/usr/bin/env python3
"""
Export rectification maps as binary files for use in C code.

Usage:
    python3 export_rectify_maps.py

Reads /tmp/stereo_calibration.npz (from stereo_calibrate.py)
Outputs /tmp/rectify_map_left.bin and /tmp/rectify_map_right.bin
"""

import numpy as np
import cv2
import struct

def export_maps():
    # Load calibration
    calib_file = "/tmp/stereo_calibration.npz"
    try:
        calib = np.load(calib_file)
    except FileNotFoundError:
        print(f"Calibration file not found: {calib_file}")
        print("Run stereo_calibrate.py first!")
        return False

    K_left = calib['K_left']
    D_left = calib['D_left']
    K_right = calib['K_right']
    D_right = calib['D_right']
    R1 = calib['R1']
    R2 = calib['R2']
    P1 = calib['P1']
    P2 = calib['P2']
    img_size = tuple(calib['img_size'])

    print(f"Image size: {img_size}")
    print(f"K_left:\n{K_left}")
    print(f"D_left: {D_left.flatten()}")

    # Compute rectification maps
    print("\nComputing rectification maps...")
    map1_left, map2_left = cv2.initUndistortRectifyMap(
        K_left, D_left, R1, P1, img_size, cv2.CV_32FC1)
    map1_right, map2_right = cv2.initUndistortRectifyMap(
        K_right, D_right, R2, P2, img_size, cv2.CV_32FC1)

    print(f"Map shape: {map1_left.shape}")

    # Export as binary files
    # Format: width (4 bytes), height (4 bytes), map1 (float32), map2 (float32)
    def save_maps(filename, map1, map2):
        height, width = map1.shape
        with open(filename, 'wb') as f:
            f.write(struct.pack('II', width, height))
            map1.astype(np.float32).tofile(f)
            map2.astype(np.float32).tofile(f)
            file_size = f.tell()
        print(f"Saved: {filename} ({width}x{height}, {file_size} bytes)")

    save_maps("/tmp/rectify_map_left.bin", map1_left, map2_left)
    save_maps("/tmp/rectify_map_right.bin", map1_right, map2_right)

    # Also create a header file with calibration constants
    header = f"""// Auto-generated stereo calibration parameters
// Generated from /tmp/stereo_calibration.npz

#ifndef STEREO_CALIB_DATA_H
#define STEREO_CALIB_DATA_H

// Image dimensions
#define STEREO_WIDTH {img_size[0]}
#define STEREO_HEIGHT {img_size[1]}

// Left camera intrinsics
#define CAM_LEFT_FX {K_left[0,0]:.4f}f
#define CAM_LEFT_FY {K_left[1,1]:.4f}f
#define CAM_LEFT_CX {K_left[0,2]:.4f}f
#define CAM_LEFT_CY {K_left[1,2]:.4f}f

// Left camera distortion (k1, k2, p1, p2)
#define CAM_LEFT_K1 {D_left[0,0]:.8f}f
#define CAM_LEFT_K2 {D_left[0,1]:.8f}f
#define CAM_LEFT_P1 {D_left[0,2]:.8f}f
#define CAM_LEFT_P2 {D_left[0,3]:.8f}f

// Rectified focal length and baseline
#define STEREO_FOCAL_RECTIFIED {P1[0,0]:.4f}f
#define STEREO_BASELINE_MM {abs(calib['T'][0,0]):.4f}f

#endif // STEREO_CALIB_DATA_H
"""

    header_file = "/tmp/stereo_calib_data.h"
    with open(header_file, 'w') as f:
        f.write(header)
    print(f"Saved: {header_file}")

    # Test: rectify one image pair if available
    import glob
    left_files = sorted(glob.glob("/tmp/calib_left_*.pgm"))
    if left_files:
        test_left = cv2.imread(left_files[0], cv2.IMREAD_GRAYSCALE)
        test_right = cv2.imread(left_files[0].replace('_left_', '_right_'), cv2.IMREAD_GRAYSCALE)

        if test_left is not None and test_right is not None:
            rect_left = cv2.remap(test_left, map1_left, map2_left, cv2.INTER_LINEAR)
            rect_right = cv2.remap(test_right, map1_right, map2_right, cv2.INTER_LINEAR)

            cv2.imwrite("/tmp/test_rectified_left.png", rect_left)
            cv2.imwrite("/tmp/test_rectified_right.png", rect_right)

            # Create side-by-side with horizontal lines to verify rectification
            combined = np.hstack([rect_left, rect_right])
            for y in range(0, combined.shape[0], 30):
                cv2.line(combined, (0, y), (combined.shape[1], y), 128, 1)
            cv2.imwrite("/tmp/test_rectified_pair.png", combined)
            print(f"\nTest rectification saved to /tmp/test_rectified_*.png")

    print("\nDone! Copy the .bin files to the target device:")
    print("  scp /tmp/rectify_map_*.bin mini:/tmp/")

    return True

if __name__ == "__main__":
    export_maps()
