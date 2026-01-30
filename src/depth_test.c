/**
 * Depth Estimation Test
 *
 * Captures stereo frames and computes depth at image center.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <signal.h>
#include <unistd.h>
#include <pthread.h>
#include <libusb-1.0/libusb.h>

#include "sdks/viture_glasses_provider.h"
#include "sdks/viture_device.h"
#include "sdks/viture_device_carina.h"
#include "stereo_depth.h"

#define VITURE_VENDOR_ID 0x35ca

// Global state
static volatile bool g_running = true;
static XRDeviceProviderHandle g_provider = NULL;
static pthread_mutex_t g_frame_mutex = PTHREAD_MUTEX_INITIALIZER;

// Stereo calibration and parameters
static stereo_calib_t g_calib = STEREO_CALIB_DEFAULT;
static stereo_params_t g_params = STEREO_PARAMS_DEFAULT;

// Rectification maps
static rectify_map_t g_map_left = {0};
static rectify_map_t g_map_right = {0};
static bool g_rectify_enabled = false;

#define RECTIFY_MAP_LEFT "data/rectify_map_left.bin"
#define RECTIFY_MAP_RIGHT "data/rectify_map_right.bin"

// Frame buffers
static uint8_t* g_left = NULL;
static uint8_t* g_right = NULL;
static int g_width = 0;
static int g_height = 0;
static volatile bool g_new_frame = false;
static volatile int g_frame_count = 0;

// Signal handler
static void signal_handler(int sig) {
    (void)sig;
    printf("\nShutting down...\n");
    g_running = false;
}

// Camera callback
static void camera_callback(char* image_left0,
                            char* image_right0,
                            char* image_left1,
                            char* image_right1,
                            double timestamp,
                            int width,
                            int height) {
    (void)image_left1;
    (void)image_right1;
    (void)timestamp;

    pthread_mutex_lock(&g_frame_mutex);

    g_frame_count++;

    // Store frame if we don't have a pending one
    if (!g_new_frame && image_left0 && image_right0) {
        size_t frame_size = width * height;

        // Allocate buffers if needed
        if (g_width != width || g_height != height) {
            free(g_left);
            free(g_right);
            g_left = malloc(frame_size);
            g_right = malloc(frame_size);
            g_width = width;
            g_height = height;
        }

        if (g_left && g_right) {
            memcpy(g_left, image_left0, frame_size);
            memcpy(g_right, image_right0, frame_size);
            g_new_frame = true;
        }
    }

    pthread_mutex_unlock(&g_frame_mutex);
}

// IMU callback (required)
static void imu_callback(float* imu, double timestamp) {
    (void)imu;
    (void)timestamp;
}

// Find Viture device
static uint16_t find_viture_device(void) {
    libusb_context* ctx = NULL;
    libusb_device** dev_list = NULL;
    uint16_t found_product = 0;

    if (libusb_init(&ctx) != 0) return 0;

    ssize_t dev_count = libusb_get_device_list(ctx, &dev_list);
    for (ssize_t i = 0; i < dev_count && !found_product; i++) {
        struct libusb_device_descriptor desc;
        if (libusb_get_device_descriptor(dev_list[i], &desc) == 0) {
            if (desc.idVendor == VITURE_VENDOR_ID &&
                xr_device_provider_is_product_id_valid(desc.idProduct)) {
                found_product = desc.idProduct;
            }
        }
    }

    libusb_free_device_list(dev_list, 1);
    libusb_exit(ctx);
    return found_product;
}

int main(int argc, char* argv[]) {
    (void)argc;
    (void)argv;

    printf("===========================================\n");
    printf("  Stereo Depth Estimation Test\n");
    printf("===========================================\n\n");

    printf("Calibration parameters:\n");
    printf("  Baseline:     %.1f mm (%.2f inches)\n",
           g_calib.baseline_mm, g_calib.baseline_mm / 25.4f);
    printf("  Focal length: %.1f pixels\n", g_calib.focal_px);
    printf("  Block size:   %d\n", g_params.block_size);
    printf("  Max disparity: %d\n", g_params.max_disparity);
    printf("  Region size:  %d\n\n", g_params.region_size);

    // Try to load rectification maps
    printf("Loading rectification maps...\n");
    if (stereo_load_rectify_map(RECTIFY_MAP_LEFT, &g_map_left) &&
        stereo_load_rectify_map(RECTIFY_MAP_RIGHT, &g_map_right)) {
        printf("  Loaded: %s (%dx%d)\n", RECTIFY_MAP_LEFT, g_map_left.width, g_map_left.height);
        printf("  Loaded: %s (%dx%d)\n", RECTIFY_MAP_RIGHT, g_map_right.width, g_map_right.height);
        g_rectify_enabled = true;
        printf("  Rectification ENABLED\n\n");
    } else {
        printf("  Rectification maps not found, running WITHOUT rectification.\n");
        printf("  To enable: python3 scripts/export_rectify_maps.py\n");
        printf("             scp /tmp/rectify_map_*.bin mini:/tmp/\n\n");
        g_rectify_enabled = false;
    }

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    // Find device
    uint16_t product_id = find_viture_device();
    if (!product_id) {
        fprintf(stderr, "No Viture device found!\n");
        return 1;
    }
    printf("Found device: 0x%04x\n", product_id);

    // Create provider
    g_provider = xr_device_provider_create(product_id);
    if (!g_provider) {
        fprintf(stderr, "Failed to create provider\n");
        return 1;
    }

    // Check device type
    int dev_type = xr_device_provider_get_device_type(g_provider);
    if (dev_type != XR_DEVICE_TYPE_VITURE_CARINA) {
        fprintf(stderr, "Warning: Device is not Carina (Luma Ultra)\n");
    }

    // Register callbacks
    if (register_callbacks_carina(g_provider, NULL, NULL,
                                   imu_callback, camera_callback) != 0) {
        fprintf(stderr, "Failed to register callbacks\n");
        xr_device_provider_destroy(g_provider);
        return 1;
    }

    // Initialize and start
    if (xr_device_provider_initialize(g_provider, NULL) != 0) {
        fprintf(stderr, "Failed to initialize provider\n");
        xr_device_provider_destroy(g_provider);
        return 1;
    }

    sleep(1);
    if (xr_device_provider_start(g_provider) != 0) {
        fprintf(stderr, "Failed to start provider\n");
        xr_device_provider_shutdown(g_provider);
        xr_device_provider_destroy(g_provider);
        return 1;
    }

    printf("\nStreaming depth at image center (Ctrl+C to stop)...\n");
    printf("Point the glasses at objects at different distances.\n");
    printf("(Processing once per second to avoid lag)\n\n");

    int depth_count = 0;
    bool saved_disparity = false;

    // Local buffers for processing (avoid holding mutex during computation)
    uint8_t* local_left = NULL;
    uint8_t* local_right = NULL;
    int local_width = 0;
    int local_height = 0;

    // Wait for first frame
    printf("  Waiting for camera frames...\n");
    while (g_running && !g_new_frame) {
        usleep(100000);
    }
    printf("  Camera streaming!\n\n");

    while (g_running) {
        // Get the latest frame
        pthread_mutex_lock(&g_frame_mutex);

        if (g_new_frame && g_left && g_right && g_width > 0 && g_height > 0) {
            // Allocate local buffers if needed
            size_t frame_size = g_width * g_height;
            if (local_width != g_width || local_height != g_height) {
                free(local_left);
                free(local_right);
                local_left = malloc(frame_size);
                local_right = malloc(frame_size);
                local_width = g_width;
                local_height = g_height;
            }

            // Copy frame data
            if (local_left && local_right) {
                memcpy(local_left, g_left, frame_size);
                memcpy(local_right, g_right, frame_size);
            }
            g_new_frame = false;
        }

        pthread_mutex_unlock(&g_frame_mutex);

        // Process outside of mutex lock
        if (local_left && local_right && local_width > 0) {
            float depth_mm;
            if (g_rectify_enabled) {
                depth_mm = stereo_get_center_depth_rectified(
                    local_left, local_right, local_width, local_height,
                    &g_map_left, &g_map_right, &g_calib, &g_params);
            } else {
                depth_mm = stereo_get_center_depth(
                    local_left, local_right, local_width, local_height,
                    &g_calib, &g_params);
            }

            if (depth_mm > 0) {
                float depth_m = depth_mm / 1000.0f;
                float depth_ft = depth_mm / 304.8f;

                printf("  Depth: %6.0f mm  |  %5.2f m  |  %5.2f ft\n",
                       depth_mm, depth_m, depth_ft);
                depth_count++;

                // Save disparity map once for visualization
                if (!saved_disparity && depth_count == 3) {
                    printf("  Saving full disparity map...\n");

                    uint16_t* disparity = malloc(local_width * local_height * sizeof(uint16_t));
                    if (disparity) {
                        stereo_compute_disparity(local_left, local_right, disparity,
                                                  local_width, local_height, &g_params);
                        stereo_save_disparity_pgm("/tmp/disparity.pgm", disparity,
                                                   local_width, local_height,
                                                   g_params.max_disparity);
                        printf("  Saved /tmp/disparity.pgm\n\n");
                        free(disparity);
                    }
                    saved_disparity = true;
                }
            } else {
                printf("  Depth: ------ (no valid disparity)\n");
            }
        }

        // Wait 1 second before next measurement
        sleep(1);
    }

    free(local_left);
    free(local_right);

    printf("\n\nProcessed %d depth measurements\n", depth_count);

    // Cleanup
    printf("Cleaning up...\n");
    xr_device_provider_stop(g_provider);
    xr_device_provider_shutdown(g_provider);
    xr_device_provider_destroy(g_provider);

    free(g_left);
    free(g_right);

    // Free rectification maps
    stereo_free_rectify_map(&g_map_left);
    stereo_free_rectify_map(&g_map_right);

    printf("Done!\n");
    return 0;
}
