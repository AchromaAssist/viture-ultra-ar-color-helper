/**
 * Stereo Camera Test - Capture and save SLAM camera images
 *
 * This test captures stereo images from the Viture Luma Ultra's
 * internal SLAM cameras via the Carina SDK.
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

// Viture USB identifiers
#define VITURE_VENDOR_ID 0x35ca

// Global state
static volatile bool g_running = true;
static XRDeviceProviderHandle g_provider = NULL;
static volatile int g_frame_count = 0;
static pthread_mutex_t g_frame_mutex = PTHREAD_MUTEX_INITIALIZER;

// Frame storage
static char* g_left0 = NULL;
static char* g_right0 = NULL;
static char* g_left1 = NULL;
static char* g_right1 = NULL;
static int g_width = 0;
static int g_height = 0;
static volatile bool g_frame_ready = false;

// Signal handler
static void signal_handler(int sig) {
    (void)sig;
    printf("\nShutting down...\n");
    g_running = false;
}

// Camera callback - receives stereo frames
static void camera_callback(char* image_left0,
                            char* image_right0,
                            char* image_left1,
                            char* image_right1,
                            double timestamp,
                            int width,
                            int height) {
    pthread_mutex_lock(&g_frame_mutex);

    g_frame_count++;

    // Only save every 30th frame to avoid overwhelming
    if (g_frame_count % 30 == 1 && !g_frame_ready) {
        printf("[Camera] Frame %d: %.3fs, %dx%d\n", g_frame_count, timestamp, width, height);
        printf("  left0=%p, right0=%p, left1=%p, right1=%p\n",
               (void*)image_left0, (void*)image_right0,
               (void*)image_left1, (void*)image_right1);

        // Store frame dimensions
        g_width = width;
        g_height = height;

        // Allocate and copy frame data (assuming grayscale, 1 byte per pixel)
        size_t frame_size = width * height;

        if (image_left0) {
            if (!g_left0) g_left0 = malloc(frame_size);
            if (g_left0) memcpy(g_left0, image_left0, frame_size);
        }

        if (image_right0) {
            if (!g_right0) g_right0 = malloc(frame_size);
            if (g_right0) memcpy(g_right0, image_right0, frame_size);
        }

        if (image_left1) {
            if (!g_left1) g_left1 = malloc(frame_size);
            if (g_left1) memcpy(g_left1, image_left1, frame_size);
        }

        if (image_right1) {
            if (!g_right1) g_right1 = malloc(frame_size);
            if (g_right1) memcpy(g_right1, image_right1, frame_size);
        }

        g_frame_ready = true;
    }

    pthread_mutex_unlock(&g_frame_mutex);
}

// IMU callback (required but we don't use it here)
static void imu_callback(float* imu, double timestamp) {
    (void)imu;
    (void)timestamp;
}

// Save grayscale image as PGM (simple format, no external libs needed)
static bool save_pgm(const char* filename, const char* data, int width, int height) {
    if (!data || width <= 0 || height <= 0) {
        printf("  [Skip] %s - no data\n", filename);
        return false;
    }

    FILE* f = fopen(filename, "wb");
    if (!f) {
        printf("  [Error] Cannot open %s for writing\n", filename);
        return false;
    }

    // PGM header
    fprintf(f, "P5\n%d %d\n255\n", width, height);

    // Write pixel data
    fwrite(data, 1, width * height, f);

    fclose(f);
    printf("  [Saved] %s (%dx%d)\n", filename, width, height);
    return true;
}

// Scan for Viture devices
static uint16_t find_viture_device(void) {
    libusb_context* ctx = NULL;
    libusb_device** dev_list = NULL;
    uint16_t found_product = 0;

    if (libusb_init(&ctx) != 0) {
        return 0;
    }

    ssize_t dev_count = libusb_get_device_list(ctx, &dev_list);
    for (ssize_t i = 0; i < dev_count && !found_product; i++) {
        struct libusb_device_descriptor desc;
        if (libusb_get_device_descriptor(dev_list[i], &desc) == 0) {
            if (desc.idVendor == VITURE_VENDOR_ID) {
                if (xr_device_provider_is_product_id_valid(desc.idProduct)) {
                    found_product = desc.idProduct;
                    printf("Found Viture device: 0x%04x\n", found_product);
                }
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
    printf("  Viture Stereo Camera Test\n");
    printf("===========================================\n\n");

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    // Find device
    uint16_t product_id = find_viture_device();
    if (!product_id) {
        fprintf(stderr, "No Viture device found!\n");
        return 1;
    }

    // Create provider
    printf("Creating device provider...\n");
    g_provider = xr_device_provider_create(product_id);
    if (!g_provider) {
        fprintf(stderr, "Failed to create provider\n");
        return 1;
    }

    // Check device type
    int dev_type = xr_device_provider_get_device_type(g_provider);
    printf("Device type: %d\n", dev_type);

    if (dev_type != XR_DEVICE_TYPE_VITURE_CARINA) {
        fprintf(stderr, "Warning: Device is not Carina type (Luma Ultra). Stereo cameras may not be available.\n");
    }

    // Register callbacks - camera callback is key here
    printf("Registering Carina callbacks (with camera)...\n");
    int reg_result = register_callbacks_carina(g_provider,
                                                NULL,           // pose
                                                NULL,           // vsync
                                                imu_callback,   // imu (required)
                                                camera_callback); // camera - this is what we want!
    if (reg_result != 0) {
        fprintf(stderr, "Failed to register callbacks: %d\n", reg_result);
        xr_device_provider_destroy(g_provider);
        return 1;
    }

    // Initialize
    printf("Initializing provider...\n");
    if (xr_device_provider_initialize(g_provider, NULL) != 0) {
        fprintf(stderr, "Failed to initialize provider\n");
        xr_device_provider_destroy(g_provider);
        return 1;
    }

    // Start
    printf("Starting provider...\n");
    sleep(1);
    if (xr_device_provider_start(g_provider) != 0) {
        fprintf(stderr, "Failed to start provider\n");
        xr_device_provider_shutdown(g_provider);
        xr_device_provider_destroy(g_provider);
        return 1;
    }

    printf("\nWaiting for stereo camera frames...\n");
    printf("(Will save frames as PGM files, press Ctrl+C to stop)\n\n");

    int saved_count = 0;
    int timeout = 100; // 10 seconds max wait

    while (g_running && saved_count < 3 && timeout > 0) {
        if (g_frame_ready) {
            pthread_mutex_lock(&g_frame_mutex);

            char filename[64];
            printf("\nSaving frame set %d:\n", saved_count + 1);

            snprintf(filename, sizeof(filename), "/tmp/stereo_left0_%d.pgm", saved_count);
            save_pgm(filename, g_left0, g_width, g_height);

            snprintf(filename, sizeof(filename), "/tmp/stereo_right0_%d.pgm", saved_count);
            save_pgm(filename, g_right0, g_width, g_height);

            snprintf(filename, sizeof(filename), "/tmp/stereo_left1_%d.pgm", saved_count);
            save_pgm(filename, g_left1, g_width, g_height);

            snprintf(filename, sizeof(filename), "/tmp/stereo_right1_%d.pgm", saved_count);
            save_pgm(filename, g_right1, g_width, g_height);

            g_frame_ready = false;
            saved_count++;

            pthread_mutex_unlock(&g_frame_mutex);
        }

        usleep(100000); // 100ms
        timeout--;

        if (timeout % 10 == 0 && g_frame_count == 0) {
            printf("  Still waiting for frames... (frames received: %d)\n", g_frame_count);
        }
    }

    if (g_frame_count == 0) {
        printf("\nNo camera frames received!\n");
        printf("The stereo cameras may not be accessible via this API,\n");
        printf("or may require additional initialization.\n");
    } else {
        printf("\nReceived %d total frames, saved %d frame sets\n", g_frame_count, saved_count);
    }

    // Cleanup
    printf("\nCleaning up...\n");
    xr_device_provider_stop(g_provider);
    xr_device_provider_shutdown(g_provider);
    xr_device_provider_destroy(g_provider);

    free(g_left0);
    free(g_right0);
    free(g_left1);
    free(g_right1);

    printf("Done!\n");
    return 0;
}
