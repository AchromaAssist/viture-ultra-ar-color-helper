/**
 * Stereo Depth Estimation
 *
 * Simple stereo matching using SAD (Sum of Absolute Differences)
 * block matching to compute disparity and depth.
 */

#ifndef STEREO_DEPTH_H
#define STEREO_DEPTH_H

#include <stdint.h>
#include <stdbool.h>

// Stereo camera calibration parameters
typedef struct {
    float baseline_mm;      // Distance between cameras in mm
    float focal_px;         // Focal length in pixels
    int image_width;
    int image_height;
} stereo_calib_t;

// Stereo matching parameters
typedef struct {
    int block_size;         // Block size for matching (odd number, e.g., 9, 15, 21)
    int min_disparity;      // Minimum disparity to search
    int max_disparity;      // Maximum disparity to search (larger = closer objects)
    int region_size;        // Size of center region to average depth over
} stereo_params_t;

// Calibrated stereo parameters for Viture Luma Ultra
// Calibration performed with 9x7 checkerboard (8x6 inner corners)
//
// Left camera intrinsics (raw fisheye):
//   fx=224.19, fy=240.79, cx=268.22, cy=201.71
//   k1=-0.071534, k2=0.019906
//
// After rectification:
//   Baseline: 131.74 mm
//   Rectified focal length: 421.44 px
//
// Note: The raw focal length (~224px) is much shorter than rectified (421px)
// because fisheye lenses compress the image. After rectification, the
// effective focal length increases significantly.
#define STEREO_CALIB_DEFAULT { \
    .baseline_mm = 131.7f, \
    .focal_px = 421.4f, \
    .image_width = 640, \
    .image_height = 480 \
}

// Default matching parameters
#define STEREO_PARAMS_DEFAULT { \
    .block_size = 15, \
    .min_disparity = 0, \
    .max_disparity = 128, \
    .region_size = 64 \
}

/**
 * Compute disparity map from stereo pair
 *
 * @param left Left image (grayscale, 8-bit)
 * @param right Right image (grayscale, 8-bit)
 * @param disparity Output disparity map (16-bit, scaled by 16 for sub-pixel)
 * @param width Image width
 * @param height Image height
 * @param params Stereo matching parameters
 * @return true on success
 */
bool stereo_compute_disparity(const uint8_t* left, const uint8_t* right,
                               uint16_t* disparity, int width, int height,
                               const stereo_params_t* params);

/**
 * Convert disparity to depth in mm
 *
 * @param disparity Disparity value (scaled by 16)
 * @param calib Calibration parameters
 * @return Depth in mm, or 0 if invalid
 */
float stereo_disparity_to_depth(uint16_t disparity, const stereo_calib_t* calib);

/**
 * Get average depth at image center
 *
 * @param left Left image (grayscale, 8-bit)
 * @param right Right image (grayscale, 8-bit)
 * @param width Image width
 * @param height Image height
 * @param calib Calibration parameters
 * @param params Stereo matching parameters
 * @return Depth in mm at center, or 0 if failed
 */
float stereo_get_center_depth(const uint8_t* left, const uint8_t* right,
                               int width, int height,
                               const stereo_calib_t* calib,
                               const stereo_params_t* params);

/**
 * Save disparity map as PGM image for visualization
 *
 * @param filename Output filename
 * @param disparity Disparity map (16-bit)
 * @param width Image width
 * @param height Image height
 * @param max_disparity Maximum disparity for normalization
 * @return true on success
 */
bool stereo_save_disparity_pgm(const char* filename, const uint16_t* disparity,
                                int width, int height, int max_disparity);

// Rectification map structure
typedef struct {
    int width;
    int height;
    float* map_x;   // X coordinate remap
    float* map_y;   // Y coordinate remap
} rectify_map_t;

/**
 * Load rectification map from binary file
 *
 * @param filename Path to .bin file (from export_rectify_maps.py)
 * @param map Output map structure (caller must free with stereo_free_rectify_map)
 * @return true on success
 */
bool stereo_load_rectify_map(const char* filename, rectify_map_t* map);

/**
 * Free rectification map
 */
void stereo_free_rectify_map(rectify_map_t* map);

/**
 * Apply rectification to an image using bilinear interpolation
 *
 * @param src Source image (grayscale)
 * @param dst Destination image (same size, caller allocated)
 * @param width Image width
 * @param height Image height
 * @param map Rectification map
 */
void stereo_rectify_image(const uint8_t* src, uint8_t* dst,
                          int width, int height, const rectify_map_t* map);

/**
 * Get depth at center with rectification
 *
 * @param left Left image (grayscale, 8-bit)
 * @param right Right image (grayscale, 8-bit)
 * @param width Image width
 * @param height Image height
 * @param map_left Left rectification map (or NULL to skip rectification)
 * @param map_right Right rectification map (or NULL to skip rectification)
 * @param calib Calibration parameters
 * @param params Stereo matching parameters
 * @return Depth in mm at center, or 0 if failed
 */
float stereo_get_center_depth_rectified(const uint8_t* left, const uint8_t* right,
                                         int width, int height,
                                         const rectify_map_t* map_left,
                                         const rectify_map_t* map_right,
                                         const stereo_calib_t* calib,
                                         const stereo_params_t* params);

#endif // STEREO_DEPTH_H
