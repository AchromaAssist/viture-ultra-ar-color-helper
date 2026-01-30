/**
 * Stereo Depth Estimation Implementation
 *
 * Uses SAD (Sum of Absolute Differences) block matching for disparity.
 */

#include "stereo_depth.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

// Compute SAD (Sum of Absolute Differences) for a block
static int compute_sad(const uint8_t* left, const uint8_t* right,
                       int width, int height,
                       int lx, int ly, int rx, int ry,
                       int block_size) {
    int half = block_size / 2;
    int sad = 0;

    for (int dy = -half; dy <= half; dy++) {
        int ly_pos = ly + dy;
        int ry_pos = ry + dy;

        if (ly_pos < 0 || ly_pos >= height || ry_pos < 0 || ry_pos >= height) {
            continue;
        }

        for (int dx = -half; dx <= half; dx++) {
            int lx_pos = lx + dx;
            int rx_pos = rx + dx;

            if (lx_pos < 0 || lx_pos >= width || rx_pos < 0 || rx_pos >= width) {
                continue;
            }

            int left_val = left[ly_pos * width + lx_pos];
            int right_val = right[ry_pos * width + rx_pos];
            sad += abs(left_val - right_val);
        }
    }

    return sad;
}

bool stereo_compute_disparity(const uint8_t* left, const uint8_t* right,
                               uint16_t* disparity, int width, int height,
                               const stereo_params_t* params) {
    if (!left || !right || !disparity || !params) {
        return false;
    }

    int block_size = params->block_size;
    int min_disp = params->min_disparity;
    int max_disp = params->max_disparity;
    int half = block_size / 2;

    // Initialize disparity to 0
    memset(disparity, 0, width * height * sizeof(uint16_t));

    // For each pixel in the left image
    for (int y = half; y < height - half; y++) {
        for (int x = max_disp + half; x < width - half; x++) {
            int best_disp = 0;
            int best_sad = INT32_MAX;

            // Search for best match in right image along epipolar line
            // (horizontal search since images are assumed rectified)
            for (int d = min_disp; d < max_disp; d++) {
                int rx = x - d;
                if (rx < half) break;

                int sad = compute_sad(left, right, width, height,
                                      x, y, rx, y, block_size);

                if (sad < best_sad) {
                    best_sad = sad;
                    best_disp = d;
                }
            }

            // Sub-pixel refinement using parabola fitting
            if (best_disp > min_disp && best_disp < max_disp - 1) {
                int sad_m1 = compute_sad(left, right, width, height,
                                         x, y, x - (best_disp - 1), y, block_size);
                int sad_p1 = compute_sad(left, right, width, height,
                                         x, y, x - (best_disp + 1), y, block_size);

                // Parabola fit: sub_disp = (sad_m1 - sad_p1) / (2 * (sad_m1 + sad_p1 - 2*best_sad))
                int denom = 2 * (sad_m1 + sad_p1 - 2 * best_sad);
                if (denom != 0) {
                    float sub_disp = (float)(sad_m1 - sad_p1) / denom;
                    // Clamp sub-pixel offset
                    if (sub_disp > 1.0f) sub_disp = 1.0f;
                    if (sub_disp < -1.0f) sub_disp = -1.0f;

                    // Store disparity scaled by 16 for sub-pixel precision
                    disparity[y * width + x] = (uint16_t)((best_disp + sub_disp) * 16);
                } else {
                    disparity[y * width + x] = (uint16_t)(best_disp * 16);
                }
            } else {
                disparity[y * width + x] = (uint16_t)(best_disp * 16);
            }
        }
    }

    return true;
}

float stereo_disparity_to_depth(uint16_t disparity, const stereo_calib_t* calib) {
    if (!calib || disparity == 0) {
        return 0.0f;
    }

    // disparity is scaled by 16, so divide to get actual disparity
    float disp = disparity / 16.0f;

    if (disp < 0.5f) {
        return 0.0f;  // Invalid disparity
    }

    // depth = baseline * focal_length / disparity
    float depth_mm = (calib->baseline_mm * calib->focal_px) / disp;

    return depth_mm;
}

float stereo_get_center_depth(const uint8_t* left, const uint8_t* right,
                               int width, int height,
                               const stereo_calib_t* calib,
                               const stereo_params_t* params) {
    if (!left || !right || !calib || !params) {
        return 0.0f;
    }

    // Only compute disparity for center region (faster)
    int region = params->region_size;
    int cx = width / 2;
    int cy = height / 2;
    int half_region = region / 2;

    // Extract center region
    int region_w = region;
    int region_h = region;

    uint8_t* left_region = malloc(region_w * region_h);
    uint8_t* right_region = malloc(region_w * region_h);
    uint16_t* disp_region = malloc(region_w * region_h * sizeof(uint16_t));

    if (!left_region || !right_region || !disp_region) {
        free(left_region);
        free(right_region);
        free(disp_region);
        return 0.0f;
    }

    // Need to copy a larger region from right image for disparity search
    // Actually, let's just compute on the full image center
    // This is simpler and works with the existing algorithm

    // Compute disparity for center region only by limiting the loop
    int block_size = params->block_size;
    int min_disp = params->min_disparity;
    int max_disp = params->max_disparity;
    int half = block_size / 2;

    float total_depth = 0.0f;
    int valid_count = 0;

    int y_start = cy - half_region;
    int y_end = cy + half_region;
    int x_start = cx - half_region;
    int x_end = cx + half_region;

    // Clamp to valid range
    if (y_start < half) y_start = half;
    if (y_end > height - half) y_end = height - half;
    if (x_start < max_disp + half) x_start = max_disp + half;
    if (x_end > width - half) x_end = width - half;

    for (int y = y_start; y < y_end; y += 4) {  // Sample every 4 pixels for speed
        for (int x = x_start; x < x_end; x += 4) {
            int best_disp = 0;
            int best_sad = INT32_MAX;

            for (int d = min_disp; d < max_disp; d++) {
                int rx = x - d;
                if (rx < half) break;

                int sad = compute_sad(left, right, width, height,
                                      x, y, rx, y, block_size);

                if (sad < best_sad) {
                    best_sad = sad;
                    best_disp = d;
                }
            }

            if (best_disp > 0) {
                uint16_t disp_scaled = (uint16_t)(best_disp * 16);
                float depth = stereo_disparity_to_depth(disp_scaled, calib);

                // Filter outliers (reasonable depth range: 100mm to 10000mm)
                if (depth > 100.0f && depth < 10000.0f) {
                    total_depth += depth;
                    valid_count++;
                }
            }
        }
    }

    free(left_region);
    free(right_region);
    free(disp_region);

    if (valid_count > 0) {
        return total_depth / valid_count;
    }

    return 0.0f;
}

bool stereo_save_disparity_pgm(const char* filename, const uint16_t* disparity,
                                int width, int height, int max_disparity) {
    if (!filename || !disparity) {
        return false;
    }

    FILE* f = fopen(filename, "wb");
    if (!f) {
        return false;
    }

    // PGM header
    fprintf(f, "P5\n%d %d\n255\n", width, height);

    // Normalize and write
    for (int i = 0; i < width * height; i++) {
        int disp = disparity[i] / 16;  // Remove sub-pixel scaling
        int val = (disp * 255) / max_disparity;
        if (val > 255) val = 255;
        uint8_t byte = (uint8_t)val;
        fwrite(&byte, 1, 1, f);
    }

    fclose(f);
    return true;
}

bool stereo_load_rectify_map(const char* filename, rectify_map_t* map) {
    if (!filename || !map) return false;

    FILE* f = fopen(filename, "rb");
    if (!f) {
        fprintf(stderr, "Cannot open rectification map: %s\n", filename);
        return false;
    }

    // Read header: width, height (2 x uint32)
    uint32_t dims[2];
    if (fread(dims, sizeof(uint32_t), 2, f) != 2) {
        fclose(f);
        return false;
    }

    map->width = dims[0];
    map->height = dims[1];
    size_t map_size = map->width * map->height;

    // Allocate maps
    map->map_x = malloc(map_size * sizeof(float));
    map->map_y = malloc(map_size * sizeof(float));

    if (!map->map_x || !map->map_y) {
        free(map->map_x);
        free(map->map_y);
        map->map_x = NULL;
        map->map_y = NULL;
        fclose(f);
        return false;
    }

    // Read map data
    size_t read_x = fread(map->map_x, sizeof(float), map_size, f);
    size_t read_y = fread(map->map_y, sizeof(float), map_size, f);

    fclose(f);

    if (read_x != map_size || read_y != map_size) {
        free(map->map_x);
        free(map->map_y);
        map->map_x = NULL;
        map->map_y = NULL;
        return false;
    }

    return true;
}

void stereo_free_rectify_map(rectify_map_t* map) {
    if (map) {
        free(map->map_x);
        free(map->map_y);
        map->map_x = NULL;
        map->map_y = NULL;
        map->width = 0;
        map->height = 0;
    }
}

void stereo_rectify_image(const uint8_t* src, uint8_t* dst,
                          int width, int height, const rectify_map_t* map) {
    if (!src || !dst || !map || !map->map_x || !map->map_y) return;
    if (map->width != width || map->height != height) return;

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            int idx = y * width + x;
            float src_x = map->map_x[idx];
            float src_y = map->map_y[idx];

            // Bilinear interpolation
            int x0 = (int)floorf(src_x);
            int y0 = (int)floorf(src_y);
            int x1 = x0 + 1;
            int y1 = y0 + 1;

            // Bounds check
            if (x0 < 0 || x1 >= width || y0 < 0 || y1 >= height) {
                dst[idx] = 0;
                continue;
            }

            float fx = src_x - x0;
            float fy = src_y - y0;

            // Get 4 neighbor pixels
            uint8_t p00 = src[y0 * width + x0];
            uint8_t p10 = src[y0 * width + x1];
            uint8_t p01 = src[y1 * width + x0];
            uint8_t p11 = src[y1 * width + x1];

            // Bilinear interpolation
            float val = (1 - fx) * (1 - fy) * p00 +
                        fx * (1 - fy) * p10 +
                        (1 - fx) * fy * p01 +
                        fx * fy * p11;

            dst[idx] = (uint8_t)(val + 0.5f);
        }
    }
}

float stereo_get_center_depth_rectified(const uint8_t* left, const uint8_t* right,
                                         int width, int height,
                                         const rectify_map_t* map_left,
                                         const rectify_map_t* map_right,
                                         const stereo_calib_t* calib,
                                         const stereo_params_t* params) {
    if (!left || !right || !calib || !params) return 0.0f;

    uint8_t* rect_left = NULL;
    uint8_t* rect_right = NULL;
    const uint8_t* proc_left = left;
    const uint8_t* proc_right = right;

    // Apply rectification if maps are provided
    if (map_left && map_right && map_left->map_x && map_right->map_x) {
        rect_left = malloc(width * height);
        rect_right = malloc(width * height);

        if (rect_left && rect_right) {
            stereo_rectify_image(left, rect_left, width, height, map_left);
            stereo_rectify_image(right, rect_right, width, height, map_right);
            proc_left = rect_left;
            proc_right = rect_right;
        }
    }

    // Compute depth using rectified images
    float depth = stereo_get_center_depth(proc_left, proc_right, width, height,
                                           calib, params);

    free(rect_left);
    free(rect_right);

    return depth;
}
