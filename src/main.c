/**
 * Viture AR Glasses CLI Demo
 *
 * A simple CLI application demonstrating:
 * - Device detection and enumeration
 * - Connection status handling
 * - IMU data display (roll, pitch, yaw)
 * - DRM direct rendering to glasses display
 */

#define _USE_MATH_DEFINES
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <signal.h>
#include <unistd.h>
#include <pthread.h>
#include <time.h>
#include <termios.h>
#include <fcntl.h>
#include <libusb-1.0/libusb.h>
#include <jpeglib.h>

#include "sdks/viture_glasses_provider.h"
#include "sdks/viture_device.h"
#include "sdks/viture_device_carina.h"

#include "drm_render.h"
#include "v4l2_capture.h"
#include "color_detect.h"
#include "stereo_depth.h"
#include "mjpeg_stream.h"

// Camera settings
#define CAMERA_DEVICE "/dev/video0"
#define DEFAULT_CROP_SIZE 100   // Default crop size from camera center
#define DEFAULT_DISPLAY_W 100   // Default display width
#define DEFAULT_DISPLAY_H 100   // Default display height
#define DEFAULT_OFFSET_X 0      // Default X offset from center
#define DEFAULT_OFFSET_Y 0      // Default Y offset from center
#define DEFAULT_OPACITY 100     // Default opacity (100%)
#define DEFAULT_DOMINANCE 30    // Default color dominance threshold (30%)
#define DEFAULT_CAM_INTERVAL 10 // Update camera every N frames (default 10)
#define BORDER_THICKNESS 4      // Green border thickness
#define MAX_CROP_SIZE 640       // Maximum crop size
#define TEXT_SCALE 10           // Font scale for color label
#define TEXT_MAX_WIDTH (8 * TEXT_SCALE * 8)  // Max width for 8 chars

// SLAM correction constants
// Viture display: 1920x1200, ~43 deg diagonal FOV
// Approx 50 pixels per degree (adjustable)
#define PIXELS_PER_DEGREE_YAW   50.0f   // Horizontal correction
#define PIXELS_PER_DEGREE_PITCH 50.0f   // Vertical correction

// Viture USB identifiers
#define VITURE_VENDOR_ID 0x35ca

// Known Viture product IDs
static const uint16_t VITURE_PRODUCT_IDS[] = {
    0x1011, // One
    0x1013, // One
    0x1017, // One
    0x1015, // One Lite
    0x101b, // One Lite
    0x1019, // Pro
    0x101d, // Pro
    0x1131, // Luma
    0x1121, // Luma Pro
    0x1141, // Luma Pro
    0x1101, // Luma Ultra
    0x1104, // Luma Ultra
    0x1151, // Luma Cyber
    0x1201  // Viture Beast
};
#define VITURE_PRODUCT_COUNT (sizeof(VITURE_PRODUCT_IDS) / sizeof(VITURE_PRODUCT_IDS[0]))

static const char* VITURE_PRODUCT_NAMES[] = {
    "One", "One", "One",
    "One Lite", "One Lite",
    "Pro", "Pro",
    "Luma",
    "Luma Pro", "Luma Pro",
    "Luma Ultra", "Luma Ultra",
    "Luma Cyber",
    "Beast"
};

// IMU mode and frequency constants
#define IMU_MODE_RAW  0
#define IMU_MODE_POSE 1

#define IMU_FREQ_LOW         0  // 60Hz
#define IMU_FREQ_MEDIUM_LOW  1  // 90Hz
#define IMU_FREQ_MEDIUM      2  // 120Hz
#define IMU_FREQ_MEDIUM_HIGH 3  // 240Hz
#define IMU_FREQ_HIGH        4  // 500Hz

// Calibration parameters (set via CLI)
static int g_crop_size = DEFAULT_CROP_SIZE;    // Crop size from camera
static int g_display_w = DEFAULT_DISPLAY_W;    // Display width
static int g_display_h = DEFAULT_DISPLAY_H;    // Display height
static int g_offset_x = DEFAULT_OFFSET_X;      // X offset from center
static int g_offset_y = DEFAULT_OFFSET_Y;      // Y offset from center
static int g_opacity = DEFAULT_OPACITY;        // Opacity (0-100%)
static int g_dominance = DEFAULT_DOMINANCE;    // Color dominance threshold (0-100%)
static int g_cam_interval = DEFAULT_CAM_INTERVAL;  // Camera update interval (frames)
static int g_roi_offset_y = 0;                        // Y offset of crop ROI from center
static bool g_hide_image = false;                  // Hide camera image, show only label

// Dynamic image buffers
static uint8_t* g_crop_buffer = NULL;    // Cropped image from camera
static uint8_t* g_scaled_buffer = NULL;  // Scaled image for display
static uint8_t* g_rotated_buffer = NULL; // Rotated image for SLAM correction

// Scene buffers for full widget rotation (border + image + text)
static uint8_t* g_scene_buffer = NULL;   // Scene with border/image/text
static uint8_t* g_scene_rotated = NULL;  // Rotated scene
static int g_scene_w = 0;                // Scene buffer width
static int g_scene_h = 0;                // Scene buffer height

// MJPEG stream frame buffer (full camera resolution)
static uint8_t* g_stream_frame = NULL;
static int g_stream_frame_count = 0;  // For frame skipping

// Global state
static volatile bool g_running = true;
static XRDeviceProviderHandle g_provider = NULL;
static XRDeviceType g_device_type = XR_DEVICE_TYPE_VITURE_GEN1;
static pthread_mutex_t g_pose_mutex = PTHREAD_MUTEX_INITIALIZER;
static float g_roll = 0.0f;
static float g_pitch = 0.0f;
static float g_yaw = 0.0f;
static bool g_has_pose_data = false;
static uint64_t g_last_timestamp = 0;
static bool g_connected = false;
static uint16_t g_connected_product_id = 0;

// DRM display state
static drm_display_t g_display = {0};
static bool g_drm_initialized = false;

// Camera state
static camera_t g_camera = {0};
static bool g_camera_initialized = false;

// Last color detection results (for display)
static const char* g_last_color = "?";
static double g_last_process_ms = 0.0;

// SLAM correction state - using quaternions for orientation-independent tracking
static float g_ref_qw = 1.0f, g_ref_qx = 0.0f, g_ref_qy = 0.0f, g_ref_qz = 0.0f;  // Reference quaternion
static float g_cur_qw = 1.0f, g_cur_qx = 0.0f, g_cur_qy = 0.0f, g_cur_qz = 0.0f;  // Current quaternion
static pthread_mutex_t g_quat_mutex = PTHREAD_MUTEX_INITIALIZER;
static bool g_has_reference = false; // Whether we have a valid reference
// g_capture_requested removed - replaced by state machine
static volatile bool g_depth_debug_requested = false;  // D key pressed - debug depth

// Previous frame position (not used with shadow buffer, kept for reference)
static int g_prev_box_x __attribute__((unused)) = -1000;
static int g_prev_box_y __attribute__((unused)) = -1000;

// Stereo depth state
static pthread_mutex_t g_stereo_mutex = PTHREAD_MUTEX_INITIALIZER;
static uint8_t* g_stereo_left = NULL;
static uint8_t* g_stereo_right = NULL;
static int g_stereo_width = 0;
static int g_stereo_height = 0;
static volatile bool g_stereo_new_frame = false;
static float g_current_depth_mm = 0.0f;  // Latest depth measurement
static int g_current_disparity = 0;      // Latest stereo disparity (calculated on capture)
static int g_disparity_delta = 0;        // Manual disparity adjustment (+/- keys)
static stereo_calib_t g_stereo_calib = STEREO_CALIB_DEFAULT;
static stereo_params_t g_stereo_params = STEREO_PARAMS_DEFAULT;

// Rectification maps for stereo depth (stored locally, not in /tmp)
#define RECTIFY_MAP_LEFT "data/rectify_map_left.bin"
#define RECTIFY_MAP_RIGHT "data/rectify_map_right.bin"
static rectify_map_t g_rectify_map_left = {0};
static rectify_map_t g_rectify_map_right = {0};
static bool g_rectify_enabled = false;

// Stereo 3D display constants
// Display mode values from viture_protocol.h
#define DISPLAY_MODE_2D_1920x1200_60HZ  0x41
#define DISPLAY_MODE_3D_3840x1200_60HZ  0x42

// Stereo disparity calculation parameters
#define STEREO_IPD_MM           63.0f    // Interpupillary distance (adjustable)
#define STEREO_DISPLAY_DEPTH_MM 2000.0f  // Glasses focal plane at 2m
#define STEREO_FOCAL_PX         4800.0f  // Virtual focal length (calibrated empirically)
#define STEREO_MIN_DEPTH_MM     300.0f   // Minimum depth to consider (30cm)
#define STEREO_MAX_DISPARITY    300      // Maximum disparity in pixels (clamp)

static bool g_stereo_3d_enabled = false; // Whether 3D SBS mode is active
static volatile bool g_provider_started = false; // Whether provider is fully started

// State machine for head nod detection flow
typedef enum {
    STATE_INTRO,
    STATE_WAITING_NOD,
    STATE_COUNTDOWN,
    STATE_CAPTURE,
    STATE_DISPLAY,
    STATE_COOLDOWN
} app_state_t;

static app_state_t g_state = STATE_INTRO;
static struct timespec g_state_start;  // When current state began

// Nod detection
#define NOD_DOWN_THRESHOLD  5.0f   // degrees pitch down to trigger
#define NOD_RETURN_THRESHOLD 3.0f  // degrees return to confirm
#define NOD_TIMEOUT_MS 500         // max time for nod gesture
static float g_nod_baseline_pitch = 0.0f;
static bool g_nod_phase1 = false;  // Detected downward motion
static struct timespec g_nod_start;

// Terminal state for non-blocking input
static struct termios g_orig_termios;
static bool g_termios_modified = false;

// Forward declarations
static void cleanup(void);
static double elapsed_ms(const struct timespec* start);

// Calculate stereo disparity from depth
// Returns disparity in pixels (positive = closer than screen plane)
static int calculate_stereo_disparity(float depth_mm) {
    if (depth_mm < STEREO_MIN_DEPTH_MM) {
        depth_mm = STEREO_MIN_DEPTH_MM;  // Clamp minimum depth
    }

    // disparity = IPD * focal * (1/screen_depth - 1/object_depth)
    // Negative for close objects (need inward shift for convergence)
    float disparity = STEREO_IPD_MM * STEREO_FOCAL_PX *
                      (1.0f / STEREO_DISPLAY_DEPTH_MM - 1.0f / depth_mm);

    // Clamp to reasonable range
    if (disparity < -STEREO_MAX_DISPARITY) disparity = -STEREO_MAX_DISPARITY;
    if (disparity > STEREO_MAX_DISPARITY) disparity = STEREO_MAX_DISPARITY;

    return (int)disparity;
}

// Set up terminal for non-blocking input
static void setup_terminal(void) {
    struct termios new_termios;

    // Get current terminal settings
    if (tcgetattr(STDIN_FILENO, &g_orig_termios) < 0) {
        return;
    }

    new_termios = g_orig_termios;

    // Disable canonical mode and echo
    new_termios.c_lflag &= ~(ICANON | ECHO);
    new_termios.c_cc[VMIN] = 0;   // Non-blocking
    new_termios.c_cc[VTIME] = 0;  // No timeout

    if (tcsetattr(STDIN_FILENO, TCSANOW, &new_termios) < 0) {
        return;
    }

    // Set stdin to non-blocking
    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);

    g_termios_modified = true;
}

// Restore terminal settings
static void restore_terminal(void) {
    if (g_termios_modified) {
        tcsetattr(STDIN_FILENO, TCSANOW, &g_orig_termios);
        g_termios_modified = false;
    }
}

// Check for keypress (non-blocking), returns character or -1 if none
static int check_keypress(void) {
    char c;
    if (read(STDIN_FILENO, &c, 1) == 1) {
        return c;
    }
    return -1;
}

// Signal handler for graceful shutdown
static void signal_handler(int sig) {
    (void)sig;
    printf("\nShutting down...\n");
    g_running = false;
}

// Print usage
static void print_usage(const char* prog) {
    printf("Usage: %s [options]\n", prog);
    printf("\nCalibration options:\n");
    printf("  -c, --crop SIZE      Crop size from camera center (default: %d)\n", DEFAULT_CROP_SIZE);
    printf("  -w, --width WIDTH    Display width (default: %d)\n", DEFAULT_DISPLAY_W);
    printf("  -h, --height HEIGHT  Display height (default: %d)\n", DEFAULT_DISPLAY_H);
    printf("  -x, --offset-x X     X offset from display center (default: %d)\n", DEFAULT_OFFSET_X);
    printf("  -y, --offset-y Y     Y offset from display center (default: %d)\n", DEFAULT_OFFSET_Y);
    printf("  -o, --opacity PCT    Image opacity 0-100%% (default: %d)\n", DEFAULT_OPACITY);
    printf("  -d, --dominance PCT  Color dominance threshold 0-100%% (default: %d)\n", DEFAULT_DOMINANCE);
    printf("  -f, --frame-skip N   Update camera every N frames (default: %d)\n", DEFAULT_CAM_INTERVAL);
    printf("  -r, --roi-y Y        Y offset of crop ROI from center (default: 0)\n");
    printf("  -n, --no-image       Hide camera image, show only color label\n");
    printf("  --help               Show this help message\n");
    printf("\nExample:\n");
    printf("  %s -c 150 -w 200 -h 200 -x 50 -y -30\n", prog);
    printf("  Crops 150x150 from camera, scales to 200x200, offsets +50 right, -30 up\n");
}

// Parse command line arguments
static bool parse_args(int argc, char* argv[]) {
    for (int i = 1; i < argc; i++) {
        // Skip -- separator
        if (strcmp(argv[i], "--") == 0) {
            continue;
        }
        if (strcmp(argv[i], "--help") == 0) {
            print_usage(argv[0]);
            exit(0);
        } else if ((strcmp(argv[i], "-c") == 0 || strcmp(argv[i], "--crop") == 0) && i + 1 < argc) {
            g_crop_size = atoi(argv[++i]);
            if (g_crop_size < 10 || g_crop_size > MAX_CROP_SIZE) {
                fprintf(stderr, "Error: crop size must be between 10 and %d\n", MAX_CROP_SIZE);
                return false;
            }
        } else if ((strcmp(argv[i], "-w") == 0 || strcmp(argv[i], "--width") == 0) && i + 1 < argc) {
            g_display_w = atoi(argv[++i]);
            if (g_display_w < 10 || g_display_w > 1920) {
                fprintf(stderr, "Error: width must be between 10 and 1920\n");
                return false;
            }
        } else if ((strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--height") == 0) && i + 1 < argc) {
            g_display_h = atoi(argv[++i]);
            if (g_display_h < 10 || g_display_h > 1200) {
                fprintf(stderr, "Error: height must be between 10 and 1200\n");
                return false;
            }
        } else if ((strcmp(argv[i], "-x") == 0 || strcmp(argv[i], "--offset-x") == 0) && i + 1 < argc) {
            g_offset_x = atoi(argv[++i]);
        } else if ((strcmp(argv[i], "-y") == 0 || strcmp(argv[i], "--offset-y") == 0) && i + 1 < argc) {
            g_offset_y = atoi(argv[++i]);
        } else if ((strcmp(argv[i], "-o") == 0 || strcmp(argv[i], "--opacity") == 0) && i + 1 < argc) {
            g_opacity = atoi(argv[++i]);
            if (g_opacity < 0 || g_opacity > 100) {
                fprintf(stderr, "Error: opacity must be between 0 and 100\n");
                return false;
            }
        } else if ((strcmp(argv[i], "-d") == 0 || strcmp(argv[i], "--dominance") == 0) && i + 1 < argc) {
            g_dominance = atoi(argv[++i]);
            if (g_dominance < 0 || g_dominance > 100) {
                fprintf(stderr, "Error: dominance must be between 0 and 100\n");
                return false;
            }
        } else if ((strcmp(argv[i], "-f") == 0 || strcmp(argv[i], "--frame-skip") == 0) && i + 1 < argc) {
            g_cam_interval = atoi(argv[++i]);
            if (g_cam_interval < 1 || g_cam_interval > 100) {
                fprintf(stderr, "Error: frame-skip must be between 1 and 100\n");
                return false;
            }
        } else if ((strcmp(argv[i], "-r") == 0 || strcmp(argv[i], "--roi-y") == 0) && i + 1 < argc) {
            g_roi_offset_y = atoi(argv[++i]);
        } else if (strcmp(argv[i], "-n") == 0 || strcmp(argv[i], "--no-image") == 0) {
            g_hide_image = true;
        } else {
            fprintf(stderr, "Unknown option: %s\n", argv[i]);
            print_usage(argv[0]);
            return false;
        }
    }
    return true;
}

// Allocate image buffers based on calibration parameters
static bool allocate_buffers(void) {
    // Allocate crop buffer
    size_t crop_size = g_crop_size * g_crop_size * 3;
    g_crop_buffer = malloc(crop_size);
    if (!g_crop_buffer) {
        fprintf(stderr, "Failed to allocate crop buffer\n");
        return false;
    }

    // Allocate scaled buffer
    size_t scaled_size = g_display_w * g_display_h * 3;
    g_scaled_buffer = malloc(scaled_size);
    if (!g_scaled_buffer) {
        fprintf(stderr, "Failed to allocate scaled buffer\n");
        free(g_crop_buffer);
        g_crop_buffer = NULL;
        return false;
    }

    // Allocate rotated buffer (same size as scaled)
    g_rotated_buffer = malloc(scaled_size);
    if (!g_rotated_buffer) {
        fprintf(stderr, "Failed to allocate rotated buffer\n");
        free(g_scaled_buffer);
        free(g_crop_buffer);
        g_scaled_buffer = NULL;
        g_crop_buffer = NULL;
        return false;
    }

    // Allocate scene buffers for full widget rotation
    // Scene includes: border + image + text + margin for rotation
    int text_margin = 8 * TEXT_SCALE + 16;  // Space for text above
    int rotation_margin = 50;  // Extra margin for rotation
    g_scene_w = g_display_w + 2 * BORDER_THICKNESS + 2 * rotation_margin;
    // Ensure wide enough for intro text "NOD YOUR HEAD" at scale 4: 13*6*4 = 312px + margins
    int intro_text_w = 13 * 6 * TEXT_SCALE + 2 * rotation_margin;
    if (g_scene_w < intro_text_w) g_scene_w = intro_text_w;
    if (g_scene_w < 400) g_scene_w = 400;
    g_scene_h = g_display_h + 2 * BORDER_THICKNESS + text_margin + 2 * rotation_margin;
    size_t scene_size = g_scene_w * g_scene_h * 3;

    // Allocate MJPEG stream frame buffer (1920x1080 RGB)
    g_stream_frame = malloc(1920 * 1080 * 3);

    g_scene_buffer = malloc(scene_size);
    g_scene_rotated = malloc(scene_size);
    if (!g_scene_buffer || !g_scene_rotated || !g_stream_frame) {
        fprintf(stderr, "Failed to allocate scene buffers\n");
        free(g_rotated_buffer);
        free(g_scaled_buffer);
        free(g_crop_buffer);
        free(g_scene_buffer);
        free(g_scene_rotated);
        free(g_stream_frame);
        g_rotated_buffer = NULL;
        g_scaled_buffer = NULL;
        g_crop_buffer = NULL;
        g_scene_buffer = NULL;
        g_scene_rotated = NULL;
        g_stream_frame = NULL;
        return false;
    }

    return true;
}

// Free image buffers
static void free_buffers(void) {
    if (g_crop_buffer) {
        free(g_crop_buffer);
        g_crop_buffer = NULL;
    }
    if (g_scaled_buffer) {
        free(g_scaled_buffer);
        g_scaled_buffer = NULL;
    }
    if (g_rotated_buffer) {
        free(g_rotated_buffer);
        g_rotated_buffer = NULL;
    }
    if (g_scene_buffer) {
        free(g_scene_buffer);
        g_scene_buffer = NULL;
    }
    if (g_scene_rotated) {
        free(g_scene_rotated);
        g_scene_rotated = NULL;
    }
    if (g_stream_frame) {
        free(g_stream_frame);
        g_stream_frame = NULL;
    }
}

// Scene buffer drawing functions
static void scene_clear(void) {
    memset(g_scene_buffer, 0, g_scene_w * g_scene_h * 3);
}

static void scene_set_pixel(int x, int y, uint8_t r, uint8_t g, uint8_t b) {
    if (x < 0 || x >= g_scene_w || y < 0 || y >= g_scene_h) return;
    int offset = (y * g_scene_w + x) * 3;
    g_scene_buffer[offset + 0] = r;
    g_scene_buffer[offset + 1] = g;
    g_scene_buffer[offset + 2] = b;
}

static void scene_draw_rect(int x, int y, int w, int h, int thickness, uint8_t r, uint8_t g, uint8_t b) {
    // Top and bottom edges
    for (int i = 0; i < w; i++) {
        for (int t = 0; t < thickness; t++) {
            scene_set_pixel(x + i, y + t, r, g, b);
            scene_set_pixel(x + i, y + h - 1 - t, r, g, b);
        }
    }
    // Left and right edges
    for (int j = 0; j < h; j++) {
        for (int t = 0; t < thickness; t++) {
            scene_set_pixel(x + t, y + j, r, g, b);
            scene_set_pixel(x + w - 1 - t, y + j, r, g, b);
        }
    }
}

static void scene_draw_image(int x, int y, int w, int h, const uint8_t* rgb_data) {
    for (int j = 0; j < h; j++) {
        for (int i = 0; i < w; i++) {
            int src_offset = (j * w + i) * 3;
            scene_set_pixel(x + i, y + j, rgb_data[src_offset], rgb_data[src_offset + 1], rgb_data[src_offset + 2]);
        }
    }
}

// Simple 5x7 font for scene buffer (same as drm_render)
static const uint8_t scene_font[128][7] = {
    ['A'] = {0x0E, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11},
    ['B'] = {0x1E, 0x11, 0x11, 0x1E, 0x11, 0x11, 0x1E},
    ['C'] = {0x0E, 0x11, 0x10, 0x10, 0x10, 0x11, 0x0E},
    ['D'] = {0x1E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x1E},
    ['E'] = {0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x1F},
    ['G'] = {0x0E, 0x11, 0x10, 0x17, 0x11, 0x11, 0x0E},
    ['K'] = {0x11, 0x12, 0x14, 0x18, 0x14, 0x12, 0x11},
    ['L'] = {0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x1F},
    ['N'] = {0x11, 0x19, 0x15, 0x13, 0x11, 0x11, 0x11},
    ['O'] = {0x0E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E},
    ['R'] = {0x1E, 0x11, 0x11, 0x1E, 0x14, 0x12, 0x11},
    ['U'] = {0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E},
    ['W'] = {0x11, 0x11, 0x11, 0x15, 0x15, 0x1B, 0x11},
    ['Y'] = {0x11, 0x11, 0x0A, 0x04, 0x04, 0x04, 0x04},
    [' '] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    ['H'] = {0x11, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11},
    ['I'] = {0x0E, 0x04, 0x04, 0x04, 0x04, 0x04, 0x0E},
    ['T'] = {0x1F, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04},
    ['P'] = {0x1E, 0x11, 0x11, 0x1E, 0x10, 0x10, 0x10},
    ['F'] = {0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x10},
    ['S'] = {0x0E, 0x11, 0x10, 0x0E, 0x01, 0x11, 0x0E},
    ['M'] = {0x11, 0x1B, 0x15, 0x11, 0x11, 0x11, 0x11},
    ['0'] = {0x0E, 0x11, 0x13, 0x15, 0x19, 0x11, 0x0E},
    ['1'] = {0x04, 0x0C, 0x04, 0x04, 0x04, 0x04, 0x0E},
    ['2'] = {0x0E, 0x11, 0x01, 0x02, 0x04, 0x08, 0x1F},
    ['3'] = {0x0E, 0x11, 0x01, 0x06, 0x01, 0x11, 0x0E},
    [':'] = {0x00, 0x04, 0x04, 0x00, 0x04, 0x04, 0x00},
    ['.'] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x0C},
    ['4'] = {0x02, 0x06, 0x0A, 0x12, 0x1F, 0x02, 0x02},
    ['5'] = {0x1F, 0x10, 0x1E, 0x01, 0x01, 0x11, 0x0E},
    ['6'] = {0x0E, 0x11, 0x10, 0x1E, 0x11, 0x11, 0x0E},
    ['7'] = {0x1F, 0x01, 0x02, 0x04, 0x08, 0x08, 0x08},
    ['8'] = {0x0E, 0x11, 0x11, 0x0E, 0x11, 0x11, 0x0E},
    ['9'] = {0x0E, 0x11, 0x11, 0x0F, 0x01, 0x11, 0x0E},
};

static void scene_draw_char(int x, int y, char c, int scale, uint8_t r, uint8_t g, uint8_t b) {
    if (c < 0 || c > 127) return;
    const uint8_t* glyph = scene_font[(int)c];
    for (int row = 0; row < 7; row++) {
        for (int col = 0; col < 5; col++) {
            if (glyph[row] & (0x10 >> col)) {
                for (int sy = 0; sy < scale; sy++) {
                    for (int sx = 0; sx < scale; sx++) {
                        scene_set_pixel(x + col * scale + sx, y + row * scale + sy, r, g, b);
                    }
                }
            }
        }
    }
}

static void scene_draw_text(int x, int y, const char* text, int scale, uint8_t r, uint8_t g, uint8_t b) {
    int char_w = 6 * scale;
    for (int i = 0; text[i]; i++) {
        scene_draw_char(x + i * char_w, y, text[i], scale, r, g, b);
    }
}

static int scene_text_width(const char* text, int scale) {
    return strlen(text) * 6 * scale;
}

// Scale image using bilinear interpolation
static void scale_image(const uint8_t* src, int src_w, int src_h,
                        uint8_t* dst, int dst_w, int dst_h) {
    float x_ratio = (float)(src_w - 1) / (dst_w - 1);
    float y_ratio = (float)(src_h - 1) / (dst_h - 1);

    for (int y = 0; y < dst_h; y++) {
        float src_y = y * y_ratio;
        int y0 = (int)src_y;
        int y1 = y0 + 1;
        if (y1 >= src_h) y1 = src_h - 1;
        float y_frac = src_y - y0;

        for (int x = 0; x < dst_w; x++) {
            float src_x = x * x_ratio;
            int x0 = (int)src_x;
            int x1 = x0 + 1;
            if (x1 >= src_w) x1 = src_w - 1;
            float x_frac = src_x - x0;

            for (int c = 0; c < 3; c++) {
                // Get four corner pixels
                float p00 = src[(y0 * src_w + x0) * 3 + c];
                float p01 = src[(y0 * src_w + x1) * 3 + c];
                float p10 = src[(y1 * src_w + x0) * 3 + c];
                float p11 = src[(y1 * src_w + x1) * 3 + c];

                // Bilinear interpolation
                float p0 = p00 * (1 - x_frac) + p01 * x_frac;
                float p1 = p10 * (1 - x_frac) + p11 * x_frac;
                float p = p0 * (1 - y_frac) + p1 * y_frac;

                dst[(y * dst_w + x) * 3 + c] = (uint8_t)(p + 0.5f);
            }
        }
    }
}

// Apply opacity to image buffer (blends with black background)
static void apply_opacity(uint8_t* buffer, int width, int height, int opacity) {
    if (opacity >= 100) return;  // No change needed

    int size = width * height * 3;
    for (int i = 0; i < size; i++) {
        buffer[i] = (uint8_t)((buffer[i] * opacity) / 100);
    }
}

// Calculate edge fade opacity based on rectangle position
// Returns 0-100: 100 = fully visible (at center), 0 = fully transparent (at edge)
static int calculate_edge_opacity(int rect_x, int rect_y, int rect_w, int rect_h,
                                   int display_w, int display_h) {
    // Calculate center of rectangle
    float cx = rect_x + rect_w / 2.0f;
    float cy = rect_y + rect_h / 2.0f;

    // Display center
    float dcx = display_w / 2.0f;
    float dcy = display_h / 2.0f;

    // Calculate opacity for each edge direction
    // opacity = 100% at center, 0% at edge
    float opacity_left = (cx / dcx) * 100.0f;           // 0 at left edge, 100 at center
    float opacity_right = ((display_w - cx) / dcx) * 100.0f;  // 0 at right edge, 100 at center
    float opacity_top = (cy / dcy) * 100.0f;            // 0 at top edge, 100 at center
    float opacity_bottom = ((display_h - cy) / dcy) * 100.0f; // 0 at bottom edge, 100 at center

    // Take minimum (closest edge determines opacity)
    float min_opacity = opacity_left;
    if (opacity_right < min_opacity) min_opacity = opacity_right;
    if (opacity_top < min_opacity) min_opacity = opacity_top;
    if (opacity_bottom < min_opacity) min_opacity = opacity_bottom;

    // Clamp to 0-100
    if (min_opacity < 0) min_opacity = 0;
    if (min_opacity > 100) min_opacity = 100;

    return (int)min_opacity;
}

// Rotate image by angle (degrees) around center using bilinear interpolation
// Positive angle = counter-clockwise rotation
static void rotate_image(const uint8_t* src, uint8_t* dst, int width, int height, float angle_deg) {
    // Convert to radians
    float angle_rad = angle_deg * M_PI / 180.0f;
    float cos_a = cosf(angle_rad);
    float sin_a = sinf(angle_rad);

    // Center of image
    float cx = width / 2.0f;
    float cy = height / 2.0f;

    // Clear destination to black
    memset(dst, 0, width * height * 3);

    // For each destination pixel, find source pixel using inverse rotation
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            // Translate to center
            float dx = x - cx;
            float dy = y - cy;

            // Inverse rotation (rotate source coords)
            float src_x = dx * cos_a + dy * sin_a + cx;
            float src_y = -dx * sin_a + dy * cos_a + cy;

            // Check bounds
            if (src_x < 0 || src_x >= width - 1 || src_y < 0 || src_y >= height - 1) {
                continue;  // Outside source image, leave as black
            }

            // Bilinear interpolation
            int x0 = (int)src_x;
            int y0 = (int)src_y;
            int x1 = x0 + 1;
            int y1 = y0 + 1;
            float x_frac = src_x - x0;
            float y_frac = src_y - y0;

            for (int c = 0; c < 3; c++) {
                float p00 = src[(y0 * width + x0) * 3 + c];
                float p01 = src[(y0 * width + x1) * 3 + c];
                float p10 = src[(y1 * width + x0) * 3 + c];
                float p11 = src[(y1 * width + x1) * 3 + c];

                float p0 = p00 * (1 - x_frac) + p01 * x_frac;
                float p1 = p10 * (1 - x_frac) + p11 * x_frac;
                float p = p0 * (1 - y_frac) + p1 * y_frac;

                dst[(y * width + x) * 3 + c] = (uint8_t)(p + 0.5f);
            }
        }
    }
}

// Compute relative rotation quaternion: q_rel = q_ref^(-1) * q_cur
// This gives the rotation from reference frame to current frame in LOCAL coordinates
static void quat_relative(float ref_w, float ref_x, float ref_y, float ref_z,
                          float cur_w, float cur_x, float cur_y, float cur_z,
                          float* rel_w, float* rel_x, float* rel_y, float* rel_z) {
    // Conjugate of reference quaternion (inverse for unit quaternions)
    float inv_w = ref_w;
    float inv_x = -ref_x;
    float inv_y = -ref_y;
    float inv_z = -ref_z;

    // Quaternion multiplication: inv * cur
    *rel_w = inv_w * cur_w - inv_x * cur_x - inv_y * cur_y - inv_z * cur_z;
    *rel_x = inv_w * cur_x + inv_x * cur_w + inv_y * cur_z - inv_z * cur_y;
    *rel_y = inv_w * cur_y - inv_x * cur_z + inv_y * cur_w + inv_z * cur_x;
    *rel_z = inv_w * cur_z + inv_x * cur_y - inv_y * cur_x + inv_z * cur_w;
}

// Extract local Euler angles from quaternion (in degrees)
// Returns rotation in head-local coordinates: pitch (left/right), roll (up/down), yaw (tilt)
static void quat_to_local_euler(float qw, float qx, float qy, float qz,
                                 float* local_pitch, float* local_roll, float* local_yaw) {
    // Standard ZYX Euler extraction, but we map to our local head coords:
    // X-axis = forward (roll when tilting head side to side)
    // Y-axis = right (pitch when looking up/down)
    // Z-axis = up (yaw when turning left/right)

    // Roll (x-axis rotation) - head tilt left/right
    float sinr_cosp = 2.0f * (qw * qx + qy * qz);
    float cosr_cosp = 1.0f - 2.0f * (qx * qx + qy * qy);
    *local_yaw = atan2f(sinr_cosp, cosr_cosp) * 180.0f / M_PI;

    // Pitch (y-axis rotation) - head up/down
    float sinp = 2.0f * (qw * qy - qz * qx);
    if (fabsf(sinp) >= 1.0f) {
        *local_roll = copysignf(90.0f, sinp);
    } else {
        *local_roll = asinf(sinp) * 180.0f / M_PI;
    }

    // Yaw (z-axis rotation) - head left/right
    float siny_cosp = 2.0f * (qw * qz + qx * qy);
    float cosy_cosp = 1.0f - 2.0f * (qy * qy + qz * qz);
    *local_pitch = atan2f(siny_cosp, cosy_cosp) * 180.0f / M_PI;
}

// Get model name for a product ID
static const char* get_model_name(uint16_t product_id) {
    for (size_t i = 0; i < VITURE_PRODUCT_COUNT; i++) {
        if (VITURE_PRODUCT_IDS[i] == product_id) {
            return VITURE_PRODUCT_NAMES[i];
        }
    }
    return "Unknown";
}

// Get device type string
static const char* get_device_type_string(XRDeviceType type) {
    switch (type) {
        case XR_DEVICE_TYPE_VITURE_GEN1:   return "Gen1 (One/Pro)";
        case XR_DEVICE_TYPE_VITURE_GEN2:   return "Gen2 (Luma)";
        case XR_DEVICE_TYPE_VITURE_CARINA: return "Carina (Luma Ultra/Cyber)";
        default: return "Unknown";
    }
}

// Pose callback for Gen1/Gen2 devices
// Data format: [roll, pitch, yaw, quat_w, quat_x, quat_y, quat_z]
static void pose_callback(float* data, uint64_t timestamp) {
    if (data == NULL) return;

    pthread_mutex_lock(&g_pose_mutex);
    g_roll = data[0];
    g_pitch = data[1];
    g_yaw = data[2];
    g_last_timestamp = timestamp;
    g_has_pose_data = true;
    pthread_mutex_unlock(&g_pose_mutex);
}

// IMU callback for Carina devices (Luma Ultra)
static void carina_imu_callback(float* imu, double timestamp) {
    if (imu == NULL || g_provider == NULL || !g_provider_started) return;

    // For Carina, we need to get the pose from get_gl_pose_carina
    float pose[7] = {0};
    if (get_gl_pose_carina(g_provider, pose, 0.0) == 0) {
        // pose format: [pos_x, pos_y, pos_z, quat_w, quat_x, quat_y, quat_z]
        float qw = pose[3], qx = pose[4], qy = pose[5], qz = pose[6];

        // Store quaternion for SLAM (orientation-independent)
        pthread_mutex_lock(&g_quat_mutex);
        g_cur_qw = qw;
        g_cur_qx = qx;
        g_cur_qy = qy;
        g_cur_qz = qz;
        pthread_mutex_unlock(&g_quat_mutex);

        // Convert quaternion to euler angles for display
        // Roll (x-axis rotation)
        float sinr_cosp = 2.0f * (qw * qx + qy * qz);
        float cosr_cosp = 1.0f - 2.0f * (qx * qx + qy * qy);
        float roll = atan2f(sinr_cosp, cosr_cosp);

        // Pitch (y-axis rotation)
        float sinp = 2.0f * (qw * qy - qz * qx);
        float pitch;
        if (fabsf(sinp) >= 1.0f) {
            pitch = copysignf(M_PI / 2.0f, sinp);
        } else {
            pitch = asinf(sinp);
        }

        // Yaw (z-axis rotation)
        float siny_cosp = 2.0f * (qw * qz + qx * qy);
        float cosy_cosp = 1.0f - 2.0f * (qy * qy + qz * qz);
        float yaw = atan2f(siny_cosp, cosy_cosp);

        // Convert to degrees
        pthread_mutex_lock(&g_pose_mutex);
        g_roll = roll * 180.0f / M_PI;
        g_pitch = pitch * 180.0f / M_PI;
        g_yaw = yaw * 180.0f / M_PI;
        g_last_timestamp = (uint64_t)(timestamp * 1000000.0);
        g_has_pose_data = true;
        pthread_mutex_unlock(&g_pose_mutex);
    }
}

// Stereo camera callback for Carina devices (Luma Ultra)
static void carina_camera_callback(char* image_left0,
                                    char* image_right0,
                                    char* image_left1,
                                    char* image_right1,
                                    double timestamp,
                                    int width,
                                    int height) {
    (void)image_left1;
    (void)image_right1;
    (void)timestamp;

    if (!image_left0 || !image_right0 || !g_provider_started) return;

    pthread_mutex_lock(&g_stereo_mutex);

    // Allocate buffers if needed
    size_t frame_size = width * height;
    if (g_stereo_width != width || g_stereo_height != height) {
        free(g_stereo_left);
        free(g_stereo_right);
        g_stereo_left = malloc(frame_size);
        g_stereo_right = malloc(frame_size);
        g_stereo_width = width;
        g_stereo_height = height;
    }

    // Always copy latest frame data (overwrite old unconsumed frame)
    if (g_stereo_left && g_stereo_right) {
        memcpy(g_stereo_left, image_left0, frame_size);
        memcpy(g_stereo_right, image_right0, frame_size);
        g_stereo_new_frame = true;
    }

    pthread_mutex_unlock(&g_stereo_mutex);
}

// Draw a pixel on an RGB buffer (bounds-checked)
static void rgb_set_pixel(uint8_t* buf, int w, int h, int x, int y, uint8_t r, uint8_t g, uint8_t b) {
    if (x < 0 || x >= w || y < 0 || y >= h) return;
    int off = (y * w + x) * 3;
    buf[off] = r; buf[off+1] = g; buf[off+2] = b;
}

// Draw a rectangle outline on an RGB buffer
static void rgb_draw_rect(uint8_t* buf, int bw, int bh, int x, int y, int w, int h, int thick,
                           uint8_t r, uint8_t g, uint8_t b) {
    for (int i = 0; i < w; i++)
        for (int t = 0; t < thick; t++) {
            rgb_set_pixel(buf, bw, bh, x+i, y+t, r, g, b);
            rgb_set_pixel(buf, bw, bh, x+i, y+h-1-t, r, g, b);
        }
    for (int j = 0; j < h; j++)
        for (int t = 0; t < thick; t++) {
            rgb_set_pixel(buf, bw, bh, x+t, y+j, r, g, b);
            rgb_set_pixel(buf, bw, bh, x+w-1-t, y+j, r, g, b);
        }
}

// Draw a character on an RGB buffer using scene_font
static void rgb_draw_char(uint8_t* buf, int bw, int bh, int x, int y, char c, int scale,
                           uint8_t r, uint8_t g, uint8_t b) {
    if (c < 0 || c > 127) return;
    const uint8_t* glyph = scene_font[(int)c];
    for (int row = 0; row < 7; row++)
        for (int col = 0; col < 5; col++)
            if (glyph[row] & (0x10 >> col))
                for (int sy = 0; sy < scale; sy++)
                    for (int sx = 0; sx < scale; sx++)
                        rgb_set_pixel(buf, bw, bh, x + col*scale + sx, y + row*scale + sy, r, g, b);
}

// Draw text on an RGB buffer
static void rgb_draw_text(uint8_t* buf, int bw, int bh, int x, int y, const char* text, int scale,
                           uint8_t r, uint8_t g, uint8_t b) {
    int char_w = 6 * scale;
    for (int i = 0; text[i]; i++)
        rgb_draw_char(buf, bw, bh, x + i * char_w, y, text[i], scale, r, g, b);
}

// Save annotated capture: full camera frame with crop highlight and color/depth text
static void save_annotated_capture(const camera_t* cam, const char* color, float depth_mm) {
    if (!cam->rgb_frame || cam->width == 0 || cam->height == 0) return;

    int w = (int)cam->width;
    int h = (int)cam->height;
    size_t size = w * h * 3;

    uint8_t* img = malloc(size);
    if (!img) return;
    memcpy(img, cam->rgb_frame, size);

    // Draw crop rectangle highlight (green, 3px thick)
    int cx = w / 2, cy = h / 2 + g_roi_offset_y;
    int half = g_crop_size / 2;
    int rx = cx - half, ry = cy - half;
    rgb_draw_rect(img, w, h, rx, ry, g_crop_size, g_crop_size, 3, 0, 255, 0);

    // Build label text: "COLOR D:1.23M"
    char label[64];
    snprintf(label, sizeof(label), "%s D:%.2fM", color, depth_mm / 1000.0f);

    // Draw text inside crop rectangle at top, scale 3
    int tscale = 3;
    int text_w = (int)strlen(label) * 6 * tscale;
    int tx = rx + (g_crop_size - text_w) / 2;
    int ty = ry + 6;  // Just inside top of crop rect

    // Draw dark background strip for readability
    for (int j = ty - 2; j < ty + 7 * tscale + 2; j++)
        for (int i = tx - 2; i < tx + text_w + 2; i++)
            rgb_set_pixel(img, w, h, i, j, 0, 0, 0);

    rgb_draw_text(img, w, h, tx, ty, label, tscale, 255, 255, 255);

    // Save as JPEG
    char filename[128];
    struct timespec now;
    clock_gettime(CLOCK_REALTIME, &now);
    snprintf(filename, sizeof(filename), "/tmp/capture_%ld.jpg", (long)now.tv_sec);

    FILE* f = fopen(filename, "wb");
    if (f) {
        struct jpeg_compress_struct cinfo;
        struct jpeg_error_mgr jerr;
        cinfo.err = jpeg_std_error(&jerr);
        jpeg_create_compress(&cinfo);
        jpeg_stdio_dest(&cinfo, f);

        cinfo.image_width = w;
        cinfo.image_height = h;
        cinfo.input_components = 3;
        cinfo.in_color_space = JCS_RGB;
        jpeg_set_defaults(&cinfo);
        jpeg_set_quality(&cinfo, 90, TRUE);
        jpeg_start_compress(&cinfo, TRUE);

        while (cinfo.next_scanline < cinfo.image_height) {
            uint8_t* row = img + cinfo.next_scanline * w * 3;
            jpeg_write_scanlines(&cinfo, &row, 1);
        }

        jpeg_finish_compress(&cinfo);
        jpeg_destroy_compress(&cinfo);
        fclose(f);
        printf("[Save] %s\n", filename);
    }

    free(img);
}

// Compose annotated frame for MJPEG debug stream
// Shows the camera feed with the glasses scene overlay (same content as DRM output)
static void compose_stream_frame(void) {
    if (!g_camera_initialized || !g_stream_frame) return;

    // Grab a fresh frame from the camera
    uint8_t* frame = camera_capture_frame(&g_camera);
    if (!frame) return;

    int w = (int)g_camera.width;
    int h = (int)g_camera.height;
    if (w <= 0 || h <= 0) return;

    memcpy(g_stream_frame, frame, (size_t)w * h * 3);

    // Draw green crop rectangle (matches crop_center ROI offset)
    int cx = w / 2, cy = h / 2 + g_roi_offset_y;
    int half = g_crop_size / 2;
    int rx = cx - half, ry = cy - half;
    rgb_draw_rect(g_stream_frame, w, h, rx, ry, g_crop_size, g_crop_size, 3, 0, 255, 0);

    // Overlay the scene buffer (same content as glasses display) centered on camera frame
    // The scene buffer contains the text/widgets rendered by the state machine.
    // Non-black pixels are composited (additive blend) so they're visible on the camera.
    if (g_scene_buffer && g_scene_w > 0 && g_scene_h > 0) {
        int ox = (w - g_scene_w) / 2;
        int oy = (h - g_scene_h) / 2 + g_roi_offset_y;
        for (int j = 0; j < g_scene_h; j++) {
            int dy = oy + j;
            if (dy < 0 || dy >= h) continue;
            for (int i = 0; i < g_scene_w; i++) {
                int dx = ox + i;
                if (dx < 0 || dx >= w) continue;
                int si = (j * g_scene_w + i) * 3;
                uint8_t sr = g_scene_buffer[si];
                uint8_t sg = g_scene_buffer[si + 1];
                uint8_t sb = g_scene_buffer[si + 2];
                // Skip black (transparent) pixels
                if (sr == 0 && sg == 0 && sb == 0) continue;
                int di = (dy * w + dx) * 3;
                // Override pixels with overlay
                g_stream_frame[di]     = sr;
                g_stream_frame[di + 1] = sg;
                g_stream_frame[di + 2] = sb;
            }
        }
    }

    // Debug info: state name + elapsed time (top-left, yellow)
    static const char* state_names[] = {
        "INTRO", "WAITING", "COUNTDOWN", "CAPTURE", "DISPLAY", "COOLDOWN"
    };
    const char* sname = (g_state >= 0 && g_state <= STATE_COOLDOWN) ? state_names[g_state] : "?";
    char line[128];
    double ms = elapsed_ms(&g_state_start);
    snprintf(line, sizeof(line), "%s %.1fs", sname, ms / 1000.0);
    rgb_draw_text(g_stream_frame, w, h, 10, 10, line, 3, 255, 255, 0);

    // IMU data bottom-left (cyan)
    pthread_mutex_lock(&g_pose_mutex);
    snprintf(line, sizeof(line), "R:%+.1f P:%+.1f Y:%+.1f", g_roll, g_pitch, g_yaw);
    pthread_mutex_unlock(&g_pose_mutex);
    rgb_draw_text(g_stream_frame, w, h, 10, h - 30, line, 2, 0, 255, 255);

    // Color + depth (top-right area, when available)
    if (g_state == STATE_DISPLAY || g_state == STATE_COOLDOWN) {
        snprintf(line, sizeof(line), "%s D:%.2fM", g_last_color, g_current_depth_mm / 1000.0f);
        int tscale = 3;
        int text_w = (int)strlen(line) * 6 * tscale;
        rgb_draw_text(g_stream_frame, w, h, w - text_w - 10, 10, line, tscale, 255, 255, 255);
    }
}

// Save grayscale image as PGM file
static bool save_pgm(const char* filename, const uint8_t* data, int width, int height) {
    if (!data || width <= 0 || height <= 0) return false;

    FILE* f = fopen(filename, "wb");
    if (!f) return false;

    fprintf(f, "P5\n%d %d\n255\n", width, height);
    fwrite(data, 1, width * height, f);
    fclose(f);
    return true;
}

// Debug depth detection - save images and print detailed info
static void debug_depth_detection(void) {
    pthread_mutex_lock(&g_stereo_mutex);

    if (!g_stereo_left || !g_stereo_right || g_stereo_width <= 0 || g_stereo_height <= 0) {
        printf("\n[Depth Debug] No stereo frames available!\n");
        pthread_mutex_unlock(&g_stereo_mutex);
        return;
    }

    // Copy frames locally
    size_t frame_size = g_stereo_width * g_stereo_height;
    uint8_t* left = malloc(frame_size);
    uint8_t* right = malloc(frame_size);
    int width = g_stereo_width;
    int height = g_stereo_height;

    if (left && right) {
        memcpy(left, g_stereo_left, frame_size);
        memcpy(right, g_stereo_right, frame_size);
    }

    pthread_mutex_unlock(&g_stereo_mutex);

    if (!left || !right) {
        printf("\n[Depth Debug] Failed to allocate memory!\n");
        free(left);
        free(right);
        return;
    }

    printf("\n\n========== DEPTH DEBUG ==========\n");
    printf("Stereo frame size: %dx%d\n", width, height);

    // Save images
    const char* left_file = "/tmp/debug_left.pgm";
    const char* right_file = "/tmp/debug_right.pgm";
    const char* disp_file = "/tmp/debug_disparity.pgm";

    if (save_pgm(left_file, left, width, height)) {
        printf("Saved left image:  %s\n", left_file);
    } else {
        printf("Failed to save left image!\n");
    }

    if (save_pgm(right_file, right, width, height)) {
        printf("Saved right image: %s\n", right_file);
    } else {
        printf("Failed to save right image!\n");
    }

    // Compute full disparity map
    printf("\nComputing disparity map...\n");
    printf("  Block size: %d\n", g_stereo_params.block_size);
    printf("  Max disparity: %d\n", g_stereo_params.max_disparity);
    printf("  Region size: %d\n", g_stereo_params.region_size);

    uint16_t* disparity = malloc(width * height * sizeof(uint16_t));
    if (disparity) {
        struct timespec t_start, t_end;
        clock_gettime(CLOCK_MONOTONIC, &t_start);
        stereo_compute_disparity(left, right, disparity, width, height, &g_stereo_params);
        clock_gettime(CLOCK_MONOTONIC, &t_end);
        double disparity_ms = (t_end.tv_sec - t_start.tv_sec) * 1000.0 +
                              (t_end.tv_nsec - t_start.tv_nsec) / 1000000.0;
        printf("  Disparity computation: %.1f ms\n", disparity_ms);

        stereo_save_disparity_pgm(disp_file, disparity, width, height, g_stereo_params.max_disparity);
        printf("Saved disparity:   %s\n", disp_file);

        // Sample disparity values at different points
        printf("\nDisparity samples (raw values, /16 for pixels):\n");
        int cx = width / 2;
        int cy = height / 2;
        printf("  Center (%d,%d): %d (%.1f px)\n", cx, cy,
               disparity[cy * width + cx], disparity[cy * width + cx] / 16.0f);
        printf("  Top-center (%d,%d): %d (%.1f px)\n", cx, cy - 100,
               disparity[(cy-100) * width + cx], disparity[(cy-100) * width + cx] / 16.0f);
        printf("  Bottom-center (%d,%d): %d (%.1f px)\n", cx, cy + 100,
               disparity[(cy+100) * width + cx], disparity[(cy+100) * width + cx] / 16.0f);

        free(disparity);
    }

    // Compute depth at center
    printf("\nCalibration:\n");
    printf("  Baseline: %.1f mm\n", g_stereo_calib.baseline_mm);
    printf("  Focal length: %.1f px\n", g_stereo_calib.focal_px);
    printf("  Rectification: %s\n", g_rectify_enabled ? "ENABLED" : "disabled");

    struct timespec t_depth_start, t_depth_end;
    clock_gettime(CLOCK_MONOTONIC, &t_depth_start);

    float depth;
    if (g_rectify_enabled) {
        depth = stereo_get_center_depth_rectified(left, right, width, height,
                                                   &g_rectify_map_left, &g_rectify_map_right,
                                                   &g_stereo_calib, &g_stereo_params);
    } else {
        depth = stereo_get_center_depth(left, right, width, height,
                                         &g_stereo_calib, &g_stereo_params);
    }

    clock_gettime(CLOCK_MONOTONIC, &t_depth_end);
    double depth_ms = (t_depth_end.tv_sec - t_depth_start.tv_sec) * 1000.0 +
                      (t_depth_end.tv_nsec - t_depth_start.tv_nsec) / 1000000.0;

    printf("\nDepth at center: %.0f mm (%.2f m)  [computed in %.1f ms]\n",
           depth, depth / 1000.0f, depth_ms);
    printf("=================================\n\n");

    free(left);
    free(right);
}

// Structure to hold detected device info
typedef struct {
    uint16_t vendor_id;
    uint16_t product_id;
    uint8_t bus;
    uint8_t address;
    char serial[256];
    char manufacturer[256];
    char product[256];
} detected_device_t;

// Scan for connected Viture devices using libusb
static int scan_devices(detected_device_t* devices, int max_devices) {
    libusb_context* ctx = NULL;
    libusb_device** dev_list = NULL;
    int count = 0;

    if (libusb_init(&ctx) != 0) {
        fprintf(stderr, "Failed to initialize libusb\n");
        return 0;
    }

    ssize_t dev_count = libusb_get_device_list(ctx, &dev_list);
    if (dev_count < 0) {
        fprintf(stderr, "Failed to get device list\n");
        libusb_exit(ctx);
        return 0;
    }

    for (ssize_t i = 0; i < dev_count && count < max_devices; i++) {
        struct libusb_device_descriptor desc;
        if (libusb_get_device_descriptor(dev_list[i], &desc) != 0) {
            continue;
        }

        if (desc.idVendor == VITURE_VENDOR_ID) {
            // Check if this is a known product
            bool known = false;
            for (size_t j = 0; j < VITURE_PRODUCT_COUNT; j++) {
                if (desc.idProduct == VITURE_PRODUCT_IDS[j]) {
                    known = true;
                    break;
                }
            }

            if (known || xr_device_provider_is_product_id_valid(desc.idProduct)) {
                detected_device_t* dev = &devices[count];
                dev->vendor_id = desc.idVendor;
                dev->product_id = desc.idProduct;
                dev->bus = libusb_get_bus_number(dev_list[i]);
                dev->address = libusb_get_device_address(dev_list[i]);
                dev->serial[0] = '\0';
                dev->manufacturer[0] = '\0';
                dev->product[0] = '\0';

                // Try to get device strings
                libusb_device_handle* handle = NULL;
                if (libusb_open(dev_list[i], &handle) == 0) {
                    if (desc.iManufacturer) {
                        libusb_get_string_descriptor_ascii(handle, desc.iManufacturer,
                            (unsigned char*)dev->manufacturer, sizeof(dev->manufacturer));
                    }
                    if (desc.iProduct) {
                        libusb_get_string_descriptor_ascii(handle, desc.iProduct,
                            (unsigned char*)dev->product, sizeof(dev->product));
                    }
                    if (desc.iSerialNumber) {
                        libusb_get_string_descriptor_ascii(handle, desc.iSerialNumber,
                            (unsigned char*)dev->serial, sizeof(dev->serial));
                    }
                    libusb_close(handle);
                }

                count++;
            }
        }
    }

    libusb_free_device_list(dev_list, 1);
    libusb_exit(ctx);

    return count;
}

// Print device information
static void print_device_info(const detected_device_t* dev, int index) {
    const char* model = get_model_name(dev->product_id);

    printf("\n  Device #%d:\n", index + 1);
    printf("    Model:        VITURE %s\n", model);
    printf("    Vendor ID:    0x%04x\n", dev->vendor_id);
    printf("    Product ID:   0x%04x\n", dev->product_id);
    printf("    Bus:          %d\n", dev->bus);
    printf("    Address:      %d\n", dev->address);

    if (dev->manufacturer[0]) {
        printf("    Manufacturer: %s\n", dev->manufacturer);
    }
    if (dev->product[0]) {
        printf("    Product:      %s\n", dev->product);
    }
    if (dev->serial[0]) {
        printf("    Serial:       %s\n", dev->serial);
    }

    // Get market name from SDK if available
    char market_name[64] = {0};
    int name_len = sizeof(market_name);
    if (xr_device_provider_get_market_name(dev->product_id, market_name, &name_len) == 0) {
        printf("    SDK Name:     %s\n", market_name);
    }

    // Check if SDK supports this device
    bool sdk_valid = xr_device_provider_is_product_id_valid(dev->product_id);
    printf("    SDK Support:  %s\n", sdk_valid ? "Yes" : "No");
}

// Initialize camera
static bool init_camera(void) {
    printf("\n-------------------------------------------\n");
    printf("Initializing camera...\n");
    printf("-------------------------------------------\n");

    if (!camera_init(&g_camera, CAMERA_DEVICE, 1920, 1080)) {
        printf("[Camera] Failed to initialize camera\n");
        return false;
    }

    if (!camera_start(&g_camera)) {
        printf("[Camera] Failed to start camera\n");
        camera_cleanup(&g_camera);
        return false;
    }

    g_camera_initialized = true;
    printf("[Camera] Camera initialized: %ux%u\n", g_camera.width, g_camera.height);
    return true;
}

// Crop center of camera frame to buffer (uses g_crop_size)
static void crop_center(camera_t* camera, uint8_t* crop_buffer) {
    if (!camera->rgb_frame) return;

    int src_cx = camera->width / 2;
    int src_cy = camera->height / 2 + g_roi_offset_y;
    int half_crop = g_crop_size / 2;

    for (int y = 0; y < g_crop_size; y++) {
        int src_y = src_cy - half_crop + y;
        if (src_y < 0 || src_y >= (int)camera->height) continue;

        for (int x = 0; x < g_crop_size; x++) {
            int src_x = src_cx - half_crop + x;
            if (src_x < 0 || src_x >= (int)camera->width) continue;

            int src_idx = (src_y * camera->width + src_x) * 3;
            int dst_idx = (y * g_crop_size + x) * 3;

            crop_buffer[dst_idx + 0] = camera->rgb_frame[src_idx + 0];
            crop_buffer[dst_idx + 1] = camera->rgb_frame[src_idx + 1];
            crop_buffer[dst_idx + 2] = camera->rgb_frame[src_idx + 2];
        }
    }
}

// Initialize DRM display
static bool init_drm_display(void) {
    printf("\n-------------------------------------------\n");
    printf("Initializing DRM display...\n");
    printf("-------------------------------------------\n");

    if (!drm_init(&g_display)) {
        printf("[DRM] Failed to initialize DRM display\n");
        return false;
    }

    g_drm_initialized = true;

    printf("[DRM] Display initialized: %ux%u (%s)\n",
           g_display.width, g_display.height, g_display.mode_name);

    // Clear to black
    printf("[DRM] Clearing display to black...\n");
    drm_clear(&g_display, 0, 0, 0);

    // Draw initial green border at center with offset (placeholder until camera frame arrives)
    int img_x = (g_display.width - g_display_w) / 2 + g_offset_x;
    int img_y = (g_display.height - g_display_h) / 2 + g_offset_y;
    int box_x = img_x - BORDER_THICKNESS;
    int box_y = img_y - BORDER_THICKNESS;
    int box_w = g_display_w + BORDER_THICKNESS * 2;
    int box_h = g_display_h + BORDER_THICKNESS * 2;

    printf("[DRM] Drawing green border at (%d, %d) size %dx%d (offset: %+d, %+d)\n",
           box_x, box_y, box_w, box_h, g_offset_x, g_offset_y);

    drm_draw_border(&g_display, box_x, box_y, box_w, box_h, BORDER_THICKNESS, 0, 255, 0);

    drm_present(&g_display);

    printf("[DRM] Initial rendering complete!\n");

    return true;
}

// Cleanup camera
static void cleanup_camera(void) {
    if (g_camera_initialized) {
        printf("[Camera] Shutting down camera...\n");
        camera_cleanup(&g_camera);
        g_camera_initialized = false;
    }
}

// Cleanup DRM display
static void cleanup_drm_display(void) {
    if (g_drm_initialized) {
        printf("[DRM] Shutting down DRM display...\n");
        drm_cleanup(&g_display);
        g_drm_initialized = false;
    }
}

// Connect to a device
static bool connect_device(uint16_t product_id) {
    if (g_provider != NULL) {
        printf("Already connected to a device\n");
        return false;
    }

    printf("Connecting to device 0x%04x...\n", product_id);

    // Set log level to show errors
    xr_device_provider_set_log_level(1);

    // Create provider
    g_provider = xr_device_provider_create(product_id);
    if (g_provider == NULL) {
        fprintf(stderr, "Failed to create device provider\n");
        return false;
    }

    // Get device type
    int dev_type = xr_device_provider_get_device_type(g_provider);
    g_device_type = (dev_type >= 0) ? (XRDeviceType)dev_type : XR_DEVICE_TYPE_VITURE_GEN1;
    printf("Device type: %s\n", get_device_type_string(g_device_type));

    // Register callbacks based on device type
    int reg_result;
    if (g_device_type == XR_DEVICE_TYPE_VITURE_CARINA) {
        reg_result = register_callbacks_carina(g_provider, NULL, NULL,
                                                carina_imu_callback,
                                                carina_camera_callback);
    } else {
        reg_result = register_pose_callback(g_provider, pose_callback);
    }

    if (reg_result != 0) {
        fprintf(stderr, "Failed to register callbacks (error: %d)\n", reg_result);
        xr_device_provider_destroy(g_provider);
        g_provider = NULL;
        return false;
    }

    // Initialize provider
    if (xr_device_provider_initialize(g_provider, NULL) != 0) {
        fprintf(stderr, "Failed to initialize provider\n");
        xr_device_provider_destroy(g_provider);
        g_provider = NULL;
        return false;
    }

    // Start provider
    sleep(1); // Wait for USB to stabilize
    if (xr_device_provider_start(g_provider) != 0) {
        fprintf(stderr, "Failed to start provider\n");
        xr_device_provider_shutdown(g_provider);
        xr_device_provider_destroy(g_provider);
        g_provider = NULL;
        return false;
    }
    g_provider_started = true;

    // Open IMU stream for non-Carina devices
    if (g_device_type != XR_DEVICE_TYPE_VITURE_CARINA) {
        int imu_result = open_imu(g_provider, IMU_MODE_POSE, IMU_FREQ_MEDIUM);
        if (imu_result != 0) {
            fprintf(stderr, "Failed to open IMU (error: %d)\n", imu_result);
            // Continue anyway, some devices might not need this
        }
    }

    g_connected = true;
    g_connected_product_id = product_id;

    // Get and display firmware version
    char version[64] = {0};
    int ver_len = sizeof(version);
    if (xr_device_provider_get_glasses_version(g_provider, version, &ver_len) == 0) {
        printf("Firmware version: %s\n", version);
    }

    // Get and display brightness, set to max if not already
    int max_brightness = (g_device_type == XR_DEVICE_TYPE_VITURE_CARINA) ? 8 : 6;
    int brightness = xr_device_provider_get_brightness_level(g_provider);
    if (brightness >= 0) {
        printf("Brightness level: %d/%d\n", brightness, max_brightness);
        if (brightness < max_brightness) {
            printf("Setting brightness to maximum (%d)...\n", max_brightness);
            if (xr_device_provider_set_brightness_level(g_provider, max_brightness) == 0) {
                printf("Brightness set to %d\n", max_brightness);
            } else {
                printf("Warning: Failed to set brightness\n");
            }
        }
    }

    printf("Connected successfully!\n");

    // Switch to 3D SBS mode for stereo depth-based rendering
    printf("[3D] Checking current display mode...\n");
    int current_mode = xr_device_provider_get_display_mode(g_provider);
    printf("[3D] Current display mode: 0x%02x\n", current_mode);

    // Try to switch to 3D mode
    printf("[3D] Attempting to switch to 3D SBS mode (3840x1200)...\n");
    int result = xr_device_provider_switch_dimension(g_provider, true);
    if (result == 0) {
        printf("[3D] Successfully requested 3D mode switch\n");
        g_stereo_3d_enabled = true;
        // Wait for display mode to propagate
        printf("[3D] Waiting for mode change...\n");
        sleep(2);
        // Verify the mode changed
        int new_mode = xr_device_provider_get_display_mode(g_provider);
        printf("[3D] New display mode: 0x%02x\n", new_mode);
    } else {
        printf("[3D] Warning: Failed to switch to 3D mode (error: %d)\n", result);
        printf("[3D] Continuing in 2D mode\n");
        g_stereo_3d_enabled = false;
    }

    // Initialize DRM display after successful connection (and mode switch)
    if (!init_drm_display()) {
        printf("[DRM] Warning: DRM display initialization failed, continuing without rendering\n");
    }

    // Initialize camera
    if (!init_camera()) {
        printf("[Camera] Warning: Camera initialization failed, continuing without camera\n");
    }

    // Load rectification maps for stereo depth (Carina only)
    if (g_device_type == XR_DEVICE_TYPE_VITURE_CARINA) {
        printf("[Stereo] Loading rectification maps...\n");
        if (stereo_load_rectify_map(RECTIFY_MAP_LEFT, &g_rectify_map_left) &&
            stereo_load_rectify_map(RECTIFY_MAP_RIGHT, &g_rectify_map_right)) {
            printf("[Stereo] Rectification ENABLED (%dx%d)\n",
                   g_rectify_map_left.width, g_rectify_map_left.height);
            g_rectify_enabled = true;
        } else {
            printf("[Stereo] Rectification maps not found, depth may be inaccurate.\n");
            printf("[Stereo] To generate: python3 scripts/export_rectify_maps.py\n");
            printf("[Stereo]              scp /tmp/rectify_map_*.bin mini:/tmp/\n");
            g_rectify_enabled = false;
        }
    }

    return true;
}

// Disconnect from device
static void disconnect_device(void) {
    if (g_provider == NULL) {
        return;
    }

    printf("Disconnecting...\n");
    g_provider_started = false;  // Stop callbacks first

    // Switch back to 2D mode if we enabled 3D
    if (g_stereo_3d_enabled) {
        printf("[3D] Switching back to 2D mode...\n");
        xr_device_provider_switch_dimension(g_provider, false);
        g_stereo_3d_enabled = false;
    }

    // Cleanup camera and DRM first
    cleanup_camera();
    cleanup_drm_display();

    // Close IMU for non-Carina devices
    if (g_device_type != XR_DEVICE_TYPE_VITURE_CARINA) {
        close_imu(g_provider, IMU_MODE_POSE);
    }

    // Stop and shutdown provider
    xr_device_provider_stop(g_provider);
    xr_device_provider_shutdown(g_provider);
    xr_device_provider_destroy(g_provider);

    g_provider = NULL;
    g_connected = false;
    g_connected_product_id = 0;
    g_has_pose_data = false;

    printf("Disconnected\n");
}

// Cleanup on exit
static void cleanup(void) {
    mjpeg_stream_stop();
    restore_terminal();
    cleanup_camera();
    cleanup_drm_display();
    disconnect_device();
    free_buffers();

    // Cleanup stereo buffers
    free(g_stereo_left);
    free(g_stereo_right);
    g_stereo_left = NULL;
    g_stereo_right = NULL;

    // Free rectification maps
    stereo_free_rectify_map(&g_rectify_map_left);
    stereo_free_rectify_map(&g_rectify_map_right);
    g_rectify_enabled = false;
}

// Check if device is still connected
static bool check_device_present(uint16_t product_id) {
    detected_device_t devices[16];
    int count = scan_devices(devices, 16);

    for (int i = 0; i < count; i++) {
        if (devices[i].product_id == product_id) {
            return true;
        }
    }
    return false;
}

// Returns milliseconds elapsed since a timespec
static double elapsed_ms(const struct timespec* start) {
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return (now.tv_sec - start->tv_sec) * 1000.0 +
           (now.tv_nsec - start->tv_nsec) / 1000000.0;
}

// Check for head nod gesture using pitch data
// Returns true when a complete nod (down then back up) is detected
static bool check_nod(void) {
    float pitch;
    pthread_mutex_lock(&g_pose_mutex);
    pitch = g_roll;  // Viture IMU: roll axis corresponds to head nod
    pthread_mutex_unlock(&g_pose_mutex);

    if (!g_nod_phase1) {
        // Phase 1: waiting for downward pitch
        float delta = pitch - g_nod_baseline_pitch;
        if (delta < -NOD_DOWN_THRESHOLD) {
            g_nod_phase1 = true;
            clock_gettime(CLOCK_MONOTONIC, &g_nod_start);
        } else {
            // Update baseline with slow EMA when idle
            g_nod_baseline_pitch = g_nod_baseline_pitch * 0.95f + pitch * 0.05f;
        }
    } else {
        // Phase 2: waiting for return to baseline
        if (elapsed_ms(&g_nod_start) > NOD_TIMEOUT_MS) {
            // Timeout - reset and update baseline
            g_nod_phase1 = false;
            g_nod_baseline_pitch = pitch;
            return false;
        }
        float delta = fabsf(pitch - g_nod_baseline_pitch);
        if (delta < NOD_RETURN_THRESHOLD) {
            // Nod complete
            g_nod_phase1 = false;
            g_nod_baseline_pitch = pitch;
            return true;
        }
    }
    return false;
}

// Set state and record start time
static void set_state(app_state_t new_state) {
    g_state = new_state;
    clock_gettime(CLOCK_MONOTONIC, &g_state_start);
    g_nod_phase1 = false;  // Reset nod detection on state change
}

// Update depth from stereo cameras (lock, copy, compute)
static void update_depth_from_stereo(void) {
    pthread_mutex_lock(&g_stereo_mutex);
    if (g_stereo_new_frame && g_stereo_left && g_stereo_right) {
        size_t frame_size = g_stereo_width * g_stereo_height;
        uint8_t* local_left = malloc(frame_size);
        uint8_t* local_right = malloc(frame_size);
        int local_w = g_stereo_width;
        int local_h = g_stereo_height;
        if (local_left && local_right) {
            memcpy(local_left, g_stereo_left, frame_size);
            memcpy(local_right, g_stereo_right, frame_size);
        }
        g_stereo_new_frame = false;
        pthread_mutex_unlock(&g_stereo_mutex);

        if (local_left && local_right) {
            float depth;
            if (g_rectify_enabled) {
                depth = stereo_get_center_depth_rectified(
                    local_left, local_right, local_w, local_h,
                    &g_rectify_map_left, &g_rectify_map_right,
                    &g_stereo_calib, &g_stereo_params);
            } else {
                depth = stereo_get_center_depth(
                    local_left, local_right, local_w, local_h,
                    &g_stereo_calib, &g_stereo_params);
            }
            if (depth > 0) {
                g_current_depth_mm = depth;
                g_current_disparity = g_stereo_3d_enabled ?
                    calculate_stereo_disparity(g_current_depth_mm) : 0;
                printf("[Depth] %.2fm, disparity=%+dpx\n",
                       g_current_depth_mm / 1000.0f, g_current_disparity);
            }
        }
        free(local_left);
        free(local_right);
    } else {
        pthread_mutex_unlock(&g_stereo_mutex);
    }
}

// Update state machine - called each frame in main loop
static void update_state_machine(void) {
    double ms = elapsed_ms(&g_state_start);

    switch (g_state) {
    case STATE_INTRO: {
        // Show "NOD YOUR HEAD" centered, visible 3s then fade over 5s
        scene_clear();
        const char* intro_text = "NOD YOUR HEAD";
        int text_w = scene_text_width(intro_text, TEXT_SCALE);
        int tx = (g_scene_w - text_w) / 2;
        int ty = (g_scene_h - 7 * TEXT_SCALE) / 2;

        uint8_t brightness = 255;
        if (ms > 3000.0) {
            // Fade from 3s to 8s (5s fade)
            double fade = 1.0 - (ms - 3000.0) / 5000.0;
            if (fade < 0.0) fade = 0.0;
            brightness = (uint8_t)(255.0 * fade);
        }
        scene_draw_text(tx, ty, intro_text, TEXT_SCALE, 0, brightness, 0);

        // Begin nod detection after 3s
        if (ms > 3000.0 && check_nod()) {
            set_state(STATE_COUNTDOWN);
            return;
        }
        // After 8s transition to waiting
        if (ms > 8000.0) {
            set_state(STATE_WAITING_NOD);
            return;
        }

        // Render scene to display
        memcpy(g_scene_rotated, g_scene_buffer, g_scene_w * g_scene_h * 3);
        drm_clear(&g_display, 0, 0, 0);
        int cx = (drm_is_stereo_mode(&g_display) ? g_display.width / 2 : g_display.width);
        int bx = (cx - g_scene_w) / 2 + g_offset_x;
        int by = (g_display.height - g_scene_h) / 2 + g_offset_y;
        if (drm_is_stereo_mode(&g_display)) {
            drm_draw_image_stereo(&g_display, bx, by, g_scene_w, g_scene_h, g_scene_rotated, g_current_disparity + g_disparity_delta);
        } else {
            drm_draw_image(&g_display, bx, by, g_scene_w, g_scene_h, g_scene_rotated);
        }
        drm_present(&g_display);
        break;
    }

    case STATE_WAITING_NOD: {
        // Show faint prompt and wait for nod
        scene_clear();
        const char* prompt = "NOD YOUR HEAD";
        int text_w = scene_text_width(prompt, TEXT_SCALE);
        int tx = (g_scene_w - text_w) / 2;
        int ty = (g_scene_h - 7 * TEXT_SCALE) / 2;
        scene_draw_text(tx, ty, prompt, TEXT_SCALE, 0, 40, 0);

        if (check_nod()) {
            set_state(STATE_COUNTDOWN);
            return;
        }

        memcpy(g_scene_rotated, g_scene_buffer, g_scene_w * g_scene_h * 3);
        drm_clear(&g_display, 0, 0, 0);
        int cx = (drm_is_stereo_mode(&g_display) ? g_display.width / 2 : g_display.width);
        int bx = (cx - g_scene_w) / 2 + g_offset_x;
        int by = (g_display.height - g_scene_h) / 2 + g_offset_y;
        if (drm_is_stereo_mode(&g_display)) {
            drm_draw_image_stereo(&g_display, bx, by, g_scene_w, g_scene_h, g_scene_rotated, g_current_disparity + g_disparity_delta);
        } else {
            drm_draw_image(&g_display, bx, by, g_scene_w, g_scene_h, g_scene_rotated);
        }
        drm_present(&g_display);
        break;
    }

    case STATE_COUNTDOWN: {
        // Show 2, 1 countdown with green rectangle
        scene_clear();
        int count = 2 - (int)(ms / 1000.0);
        if (count < 1) {
            set_state(STATE_CAPTURE);
            return;
        }
        char digit[2] = { '0' + count, '\0' };
        int text_w = scene_text_width(digit, TEXT_SCALE);
        int tx = (g_scene_w - text_w) / 2;
        int ty = (g_scene_h - 7 * TEXT_SCALE) / 2;

        // Green rectangle background
        int rect_margin = 10;
        scene_draw_rect(tx - rect_margin, ty - rect_margin,
                        text_w + 2 * rect_margin, 7 * TEXT_SCALE + 2 * rect_margin,
                        3, 0, 255, 0);
        scene_draw_text(tx, ty, digit, TEXT_SCALE, 0, 255, 0);

        memcpy(g_scene_rotated, g_scene_buffer, g_scene_w * g_scene_h * 3);
        drm_clear(&g_display, 0, 0, 0);
        int cx = (drm_is_stereo_mode(&g_display) ? g_display.width / 2 : g_display.width);
        int bx = (cx - g_scene_w) / 2 + g_offset_x;
        int by = (g_display.height - g_scene_h) / 2 + g_offset_y;
        if (drm_is_stereo_mode(&g_display)) {
            drm_draw_image_stereo(&g_display, bx, by, g_scene_w, g_scene_h, g_scene_rotated, g_current_disparity + g_disparity_delta);
        } else {
            drm_draw_image(&g_display, bx, by, g_scene_w, g_scene_h, g_scene_rotated);
        }
        drm_present(&g_display);
        break;
    }

    case STATE_CAPTURE: {
        // Capture frame, detect color, compute depth, store SLAM ref
        if (g_camera_initialized) {
            uint8_t* frame = camera_capture_frame_fresh(&g_camera);
            if (frame) {
                struct timespec start_time, end_time;
                clock_gettime(CLOCK_MONOTONIC, &start_time);

                crop_center(&g_camera, g_crop_buffer);

                if (g_crop_size != g_display_w || g_crop_size != g_display_h) {
                    scale_image(g_crop_buffer, g_crop_size, g_crop_size,
                                g_scaled_buffer, g_display_w, g_display_h);
                } else {
                    memcpy(g_scaled_buffer, g_crop_buffer, g_display_w * g_display_h * 3);
                }

                int color_pct = 0;
                detected_color_t color = color_detect_dominant(g_scaled_buffer,
                                                                g_display_w, g_display_h,
                                                                g_dominance, &color_pct);
                g_last_color = color_get_name(color);

                // Log average RGB of the crop for debugging color detection
                {
                    long sum_r = 0, sum_g = 0, sum_b = 0;
                    int npx = g_display_w * g_display_h;
                    for (int pi = 0; pi < npx; pi++) {
                        sum_r += g_scaled_buffer[pi*3+0];
                        sum_g += g_scaled_buffer[pi*3+1];
                        sum_b += g_scaled_buffer[pi*3+2];
                    }
                    printf("[ColorDbg] Avg RGB: %ld,%ld,%ld  detected=%s (%d%%)\n",
                           sum_r/npx, sum_g/npx, sum_b/npx, g_last_color, color_pct);
                }

                // Stereo depth
                update_depth_from_stereo();

                clock_gettime(CLOCK_MONOTONIC, &end_time);
                g_last_process_ms = (end_time.tv_sec - start_time.tv_sec) * 1000.0 +
                                    (end_time.tv_nsec - start_time.tv_nsec) / 1000000.0;

                printf("\n[Capture] Color: %s, Depth: %.2fm, Disparity: %+dpx, Time: %.1fms\n",
                       g_last_color, g_current_depth_mm / 1000.0f, g_current_disparity,
                       g_last_process_ms);

                g_disparity_delta = 0;

                // Save annotated full-frame image
                save_annotated_capture(&g_camera, g_last_color, g_current_depth_mm);

                // Store SLAM reference
                pthread_mutex_lock(&g_quat_mutex);
                g_ref_qw = g_cur_qw;
                g_ref_qx = g_cur_qx;
                g_ref_qy = g_cur_qy;
                g_ref_qz = g_cur_qz;
                pthread_mutex_unlock(&g_quat_mutex);
                g_has_reference = true;
            }
        }
        set_state(STATE_DISPLAY);
        break;
    }

    case STATE_DISPLAY: {
        // Show result with SLAM correction for 2 seconds
        if (ms > 2000.0) {
            set_state(STATE_COOLDOWN);
            return;
        }

        if (g_has_reference) {
            float cur_qw, cur_qx, cur_qy, cur_qz;
            pthread_mutex_lock(&g_quat_mutex);
            cur_qw = g_cur_qw; cur_qx = g_cur_qx;
            cur_qy = g_cur_qy; cur_qz = g_cur_qz;
            pthread_mutex_unlock(&g_quat_mutex);

            float rel_qw, rel_qx, rel_qy, rel_qz;
            quat_relative(g_ref_qw, g_ref_qx, g_ref_qy, g_ref_qz,
                          cur_qw, cur_qx, cur_qy, cur_qz,
                          &rel_qw, &rel_qx, &rel_qy, &rel_qz);

            float local_pitch, local_roll, local_yaw;
            quat_to_local_euler(rel_qw, rel_qx, rel_qy, rel_qz,
                                &local_pitch, &local_roll, &local_yaw);

            int slam_offset_x = (int)(local_roll * PIXELS_PER_DEGREE_YAW);
            int slam_offset_y = (int)(local_yaw * PIXELS_PER_DEGREE_PITCH);
            slam_offset_x += (int)(local_pitch * PIXELS_PER_DEGREE_YAW * 0.5f);

            int stereo_disparity = g_current_disparity + g_disparity_delta;
            int logical_width = drm_is_stereo_mode(&g_display) ?
                                g_display.width / 2 : g_display.width;

            int base_x = (logical_width - g_scene_w) / 2 + g_offset_x + slam_offset_x;
            int base_y = (g_display.height - g_scene_h) / 2 + g_offset_y + slam_offset_y;

            // Just show color name centered, no border or image
            scene_clear();
            int scene_text_w_val = scene_text_width(g_last_color, TEXT_SCALE);
            int scene_text_x = (g_scene_w - scene_text_w_val) / 2;
            int scene_text_y = (g_scene_h - 7 * TEXT_SCALE) / 2;
            scene_draw_text(scene_text_x, scene_text_y, g_last_color, TEXT_SCALE,
                            0, 255, 0);

            if (fabsf(local_pitch) > 0.5f) {
                rotate_image(g_scene_buffer, g_scene_rotated, g_scene_w, g_scene_h, local_pitch);
            } else {
                memcpy(g_scene_rotated, g_scene_buffer, g_scene_w * g_scene_h * 3);
            }

            drm_clear(&g_display, 0, 0, 0);
            if (drm_is_stereo_mode(&g_display)) {
                drm_draw_image_stereo(&g_display, base_x, base_y, g_scene_w, g_scene_h,
                                      g_scene_rotated, stereo_disparity);
            } else {
                drm_draw_image(&g_display, base_x, base_y, g_scene_w, g_scene_h,
                               g_scene_rotated);
            }
            drm_present(&g_display);
        }
        break;
    }

    case STATE_COOLDOWN: {
        // 0.5s pause then back to waiting
        if (ms > 500.0) {
            set_state(STATE_WAITING_NOD);
            return;
        }
        // Clear display during cooldown
        drm_clear(&g_display, 0, 0, 0);
        drm_present(&g_display);
        break;
    }
    }
}

// Main function
int main(int argc, char* argv[]) {
    // Parse command line arguments
    if (!parse_args(argc, argv)) {
        return 1;
    }

    // Allocate image buffers
    if (!allocate_buffers()) {
        return 1;
    }

    // Set up signal handlers
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    // Register cleanup
    atexit(cleanup);

    printf("===========================================\n");
    printf("  Viture AR Glasses CLI Demo\n");
    printf("  (with Stereo 3D Depth-Based Rendering)\n");
    printf("===========================================\n");

    // Print calibration settings
    printf("\nCalibration settings:\n");
    printf("  Crop size:    %d x %d (from camera center)\n", g_crop_size, g_crop_size);
    printf("  Display size: %d x %d\n", g_display_w, g_display_h);
    printf("  Offset:       %+d, %+d (from display center)\n", g_offset_x, g_offset_y);
    printf("  Opacity:      %d%%\n", g_opacity);
    printf("  Dominance:    %d%% (color detection threshold)\n", g_dominance);
    printf("  ROI Y offset: %+d\n", g_roi_offset_y);
    printf("  Cam interval: %d frames\n", g_cam_interval);

    // Scan for devices
    printf("\nScanning for Viture devices...\n");

    detected_device_t devices[16];
    int device_count = scan_devices(devices, 16);

    if (device_count == 0) {
        printf("\nNo Viture devices found!\n");
        printf("Please connect your Viture glasses and try again.\n");
        return 1;
    }

    printf("\nFound %d Viture device(s):\n", device_count);
    for (int i = 0; i < device_count; i++) {
        print_device_info(&devices[i], i);
    }

    // Connect to the first device
    printf("\n-------------------------------------------\n");
    printf("Connecting to first available device...\n");

    if (!connect_device(devices[0].product_id)) {
        fprintf(stderr, "Failed to connect to device\n");
        return 1;
    }

    // Main loop - capture camera and display IMU data
    printf("\n-------------------------------------------\n");
    printf("Streaming camera + IMU data (Ctrl+C to exit):\n");
    if (g_drm_initialized) {
        printf("DRM: Displaying camera crop with green border\n");
    }
    if (g_camera_initialized) {
        printf("Camera: Capturing from %s (%ux%u)\n", CAMERA_DEVICE,
               g_camera.width, g_camera.height);
    }
    printf("Nod your head to capture. SPACE=force capture, +/-=disparity, D=debug, Q=quit\n");
    printf("-------------------------------------------\n\n");

    // Start MJPEG debug stream
    if (g_camera_initialized) {
        mjpeg_stream_start(8080);
    }

    // Setup terminal for non-blocking keyboard input
    setup_terminal();

    // Initialize state machine
    clock_gettime(CLOCK_MONOTONIC, &g_state_start);
    // Initialize nod baseline from current pitch
    pthread_mutex_lock(&g_pose_mutex);
    g_nod_baseline_pitch = g_pitch;
    pthread_mutex_unlock(&g_pose_mutex);

    int loop_count = 0;

    while (g_running) {
        // Check for keyboard input
        int key = check_keypress();
        if (key == ' ') {
            // Debug shortcut: force capture
            set_state(STATE_CAPTURE);
        } else if (key == 'd' || key == 'D') {
            g_depth_debug_requested = true;
        } else if (key == 'q' || key == 'Q') {
            g_running = false;
            break;
        } else if (key == '+' || key == '=') {
            g_disparity_delta += 10;
            printf("\n[Disparity] Delta: %+dpx (total: %+dpx)\n",
                   g_disparity_delta, g_current_disparity + g_disparity_delta);
        } else if (key == '-' || key == '_') {
            g_disparity_delta -= 10;
            printf("\n[Disparity] Delta: %+dpx (total: %+dpx)\n",
                   g_disparity_delta, g_current_disparity + g_disparity_delta);
        }

        // Handle depth debug request
        if (g_depth_debug_requested) {
            g_depth_debug_requested = false;
            debug_depth_detection();
        }

        // Check device connection periodically (every ~5 seconds at 30fps = 150 frames)
        if (++loop_count >= 150) {
            loop_count = 0;
            if (!check_device_present(g_connected_product_id)) {
                printf("\n[!] Device disconnected!\n");
                disconnect_device();

                // Wait for reconnection
                printf("Waiting for device to reconnect...\n");
                while (g_running) {
                    sleep(1);
                    int count = scan_devices(devices, 16);
                    if (count > 0) {
                        printf("\n[+] Device reconnected!\n");
                        if (connect_device(devices[0].product_id)) {
                            printf("Resuming stream...\n\n");
                            break;
                        }
                    }
                }
                continue;
            }
        }

        // Periodic depth update (~every 60 frames / ~1s at 60fps)
        {
            static int depth_check_count = 0;
            if (++depth_check_count >= 60) {
                depth_check_count = 0;
                update_depth_from_stereo();
            }
        }

        // Run state machine (handles capture, display, nod detection)
        if (g_drm_initialized) {
            update_state_machine();
        }

        // Push frame to MJPEG stream (~every 4th frame to save CPU)
        if (g_camera_initialized && (++g_stream_frame_count & 3) == 0) {
            compose_stream_frame();
            mjpeg_stream_push_frame(g_stream_frame, (int)g_camera.width, (int)g_camera.height);
        }

        // Display pose data with SLAM info and depth (every 30 frames ~ 0.5 second)
        if (loop_count % 30 == 0) {
            pthread_mutex_lock(&g_pose_mutex);
            if (g_has_pose_data && g_has_reference) {
                // Compute local delta using quaternions
                float cur_qw, cur_qx, cur_qy, cur_qz;
                pthread_mutex_lock(&g_quat_mutex);
                cur_qw = g_cur_qw;
                cur_qx = g_cur_qx;
                cur_qy = g_cur_qy;
                cur_qz = g_cur_qz;
                pthread_mutex_unlock(&g_quat_mutex);

                float rel_qw, rel_qx, rel_qy, rel_qz;
                quat_relative(g_ref_qw, g_ref_qx, g_ref_qy, g_ref_qz,
                              cur_qw, cur_qx, cur_qy, cur_qz,
                              &rel_qw, &rel_qx, &rel_qy, &rel_qz);

                float local_pitch, local_roll, local_yaw;
                quat_to_local_euler(rel_qw, rel_qx, rel_qy, rel_qz,
                                    &local_pitch, &local_roll, &local_yaw);

                // Show depth and disparity (use pre-calculated values)
                float depth_m = g_current_depth_mm / 1000.0f;
                printf("\r  R:%+5.1f P:%+5.1f Y:%+5.1f [%s] D:%.2fm disp:%+d lP:%+.1f lR:%+.1f lY:%+.1f   ",
                       g_roll, g_pitch, g_yaw, g_last_color, depth_m, g_current_disparity,
                       local_pitch, local_roll, local_yaw);
                fflush(stdout);
            } else if (g_has_pose_data) {
                float depth_m = g_current_depth_mm / 1000.0f;
                printf("\r  Roll: %+7.2f  Pitch: %+7.2f  Yaw: %+7.2f  [%s, D:%.2fm, disp:%+d]   ",
                       g_roll, g_pitch, g_yaw, g_last_color, depth_m, g_current_disparity);
                fflush(stdout);
            } else {
                printf("\r  Waiting for IMU data...                              ");
                fflush(stdout);
            }
            pthread_mutex_unlock(&g_pose_mutex);
        }

        // Small delay to target ~60fps for smooth SLAM updates
        usleep(16000);  // ~16ms
    }

    printf("\n\nExiting...\n");
    return 0;
}
