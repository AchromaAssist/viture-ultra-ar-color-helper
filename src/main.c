/**
 * Viture AR Glasses CLI Demo
 *
 * A simple CLI application demonstrating:
 * - Device detection and enumeration
 * - Connection status handling
 * - IMU data display (roll, pitch, yaw)
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

// Forward declarations
static void cleanup(void);

// Signal handler for graceful shutdown
static void signal_handler(int sig) {
    (void)sig;
    printf("\nShutting down...\n");
    g_running = false;
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
    if (imu == NULL || g_provider == NULL) return;

    // For Carina, we need to get the pose from get_gl_pose_carina
    float pose[7] = {0};
    if (get_gl_pose_carina(g_provider, pose, 0.0) == 0) {
        // pose format: [pos_x, pos_y, pos_z, quat_w, quat_x, quat_y, quat_z]
        // Convert quaternion to euler angles
        float qw = pose[3], qx = pose[4], qy = pose[5], qz = pose[6];

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
        reg_result = register_callbacks_carina(g_provider, NULL, NULL, carina_imu_callback, NULL);
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

    // Get and display brightness
    int brightness = xr_device_provider_get_brightness_level(g_provider);
    if (brightness >= 0) {
        printf("Brightness level: %d\n", brightness);
    }

    printf("Connected successfully!\n");
    return true;
}

// Disconnect from device
static void disconnect_device(void) {
    if (g_provider == NULL) {
        return;
    }

    printf("Disconnecting...\n");

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
    disconnect_device();
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

// Main function
int main(int argc, char* argv[]) {
    (void)argc;
    (void)argv;

    // Set up signal handlers
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    // Register cleanup
    atexit(cleanup);

    printf("===========================================\n");
    printf("  Viture AR Glasses CLI Demo\n");
    printf("===========================================\n");

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

    // Main loop - display IMU data every second
    printf("\n-------------------------------------------\n");
    printf("Streaming IMU data (Ctrl+C to exit):\n");
    printf("-------------------------------------------\n\n");

    int last_device_check = 0;

    while (g_running) {
        // Check device connection periodically (every 5 seconds)
        if (++last_device_check >= 5) {
            last_device_check = 0;
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
                            printf("Resuming IMU data stream...\n\n");
                            break;
                        }
                    }
                }
                continue;
            }
        }

        // Display pose data
        pthread_mutex_lock(&g_pose_mutex);
        if (g_has_pose_data) {
            printf("\r  Roll: %+7.2f   Pitch: %+7.2f   Yaw: %+7.2f   ",
                   g_roll, g_pitch, g_yaw);
            fflush(stdout);
        } else {
            printf("\r  Waiting for IMU data...                              ");
            fflush(stdout);
        }
        pthread_mutex_unlock(&g_pose_mutex);

        sleep(1);
    }

    printf("\n\nExiting...\n");
    return 0;
}
