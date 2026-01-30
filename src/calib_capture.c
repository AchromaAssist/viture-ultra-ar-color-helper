/**
 * Stereo Calibration Capture Tool
 *
 * Captures stereo image pairs for checkerboard calibration.
 * Press SPACE to capture a pair, Q to quit.
 * Images are saved to /tmp/calib_left_N.pgm and /tmp/calib_right_N.pgm
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <signal.h>
#include <unistd.h>
#include <pthread.h>
#include <termios.h>
#include <fcntl.h>
#include <libusb-1.0/libusb.h>

#include "sdks/viture_glasses_provider.h"
#include "sdks/viture_device.h"
#include "sdks/viture_device_carina.h"

#define VITURE_VENDOR_ID 0x35ca
#define OUTPUT_DIR "/tmp"

// Global state
static volatile bool g_running = true;
static XRDeviceProviderHandle g_provider = NULL;
static pthread_mutex_t g_frame_mutex = PTHREAD_MUTEX_INITIALIZER;

// Frame storage
static uint8_t* g_left = NULL;
static uint8_t* g_right = NULL;
static int g_width = 0;
static int g_height = 0;
static volatile bool g_new_frame = false;
static int g_capture_count = 0;

// Terminal state
static struct termios g_orig_termios;
static bool g_termios_modified = false;

static void signal_handler(int sig) {
    (void)sig;
    g_running = false;
}

static void setup_terminal(void) {
    struct termios new_termios;
    if (tcgetattr(STDIN_FILENO, &g_orig_termios) < 0) return;
    new_termios = g_orig_termios;
    new_termios.c_lflag &= ~(ICANON | ECHO);
    new_termios.c_cc[VMIN] = 0;
    new_termios.c_cc[VTIME] = 0;
    if (tcsetattr(STDIN_FILENO, TCSANOW, &new_termios) < 0) return;
    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
    g_termios_modified = true;
}

static void restore_terminal(void) {
    if (g_termios_modified) {
        tcsetattr(STDIN_FILENO, TCSANOW, &g_orig_termios);
        g_termios_modified = false;
    }
}

static int check_keypress(void) {
    char c;
    if (read(STDIN_FILENO, &c, 1) == 1) return c;
    return -1;
}

// Camera callback
static void camera_callback(char* image_left0, char* image_right0,
                            char* image_left1, char* image_right1,
                            double timestamp, int width, int height) {
    (void)image_left1;
    (void)image_right1;
    (void)timestamp;

    if (!image_left0 || !image_right0) return;

    pthread_mutex_lock(&g_frame_mutex);

    size_t frame_size = width * height;
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

    pthread_mutex_unlock(&g_frame_mutex);
}

// IMU callback (required)
static void imu_callback(float* imu, double timestamp) {
    (void)imu;
    (void)timestamp;
}

// Save grayscale image as PGM
static bool save_pgm(const char* filename, const uint8_t* data, int width, int height) {
    FILE* f = fopen(filename, "wb");
    if (!f) return false;
    fprintf(f, "P5\n%d %d\n255\n", width, height);
    fwrite(data, 1, width * height, f);
    fclose(f);
    return true;
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
    printf("  Stereo Calibration Capture Tool\n");
    printf("===========================================\n\n");
    printf("Instructions:\n");
    printf("  1. Print a checkerboard pattern (e.g., 9x6 inner corners)\n");
    printf("  2. Hold the checkerboard at various angles and distances\n");
    printf("  3. Press SPACE to capture each stereo pair\n");
    printf("  4. Capture 15-20 pairs for good calibration\n");
    printf("  5. Press Q to quit when done\n\n");
    printf("Images will be saved to: %s/calib_left_N.pgm, calib_right_N.pgm\n\n", OUTPUT_DIR);

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

    // Register callbacks
    if (register_callbacks_carina(g_provider, NULL, NULL, imu_callback, camera_callback) != 0) {
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

    printf("Camera streaming. Press SPACE to capture, Q to quit.\n\n");

    setup_terminal();

    while (g_running) {
        int key = check_keypress();

        if (key == ' ') {
            pthread_mutex_lock(&g_frame_mutex);

            if (g_new_frame && g_left && g_right) {
                char left_file[256], right_file[256];
                snprintf(left_file, sizeof(left_file), "%s/calib_left_%02d.pgm", OUTPUT_DIR, g_capture_count);
                snprintf(right_file, sizeof(right_file), "%s/calib_right_%02d.pgm", OUTPUT_DIR, g_capture_count);

                bool ok_left = save_pgm(left_file, g_left, g_width, g_height);
                bool ok_right = save_pgm(right_file, g_right, g_width, g_height);

                if (ok_left && ok_right) {
                    printf("  [%02d] Captured: %s, %s\n", g_capture_count, left_file, right_file);
                    g_capture_count++;
                } else {
                    printf("  [!!] Failed to save images!\n");
                }
            } else {
                printf("  [!!] No frame available!\n");
            }

            pthread_mutex_unlock(&g_frame_mutex);
        } else if (key == 'q' || key == 'Q') {
            break;
        }

        usleep(50000);  // 50ms
    }

    restore_terminal();

    printf("\n\nCaptured %d stereo pairs.\n", g_capture_count);

    if (g_capture_count > 0) {
        printf("\nTo run calibration, use the Python script:\n");
        printf("  python3 scripts/stereo_calibrate.py %s/calib_left_*.pgm\n\n", OUTPUT_DIR);
    }

    // Cleanup
    xr_device_provider_stop(g_provider);
    xr_device_provider_shutdown(g_provider);
    xr_device_provider_destroy(g_provider);

    free(g_left);
    free(g_right);

    printf("Done!\n");
    return 0;
}
