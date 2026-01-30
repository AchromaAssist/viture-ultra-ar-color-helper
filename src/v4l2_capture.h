/**
 * V4L2 Camera Capture for Viture UVC Camera
 */

#ifndef V4L2_CAPTURE_H
#define V4L2_CAPTURE_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

// Camera buffer count for mmap streaming
#define CAMERA_BUFFER_COUNT 4

// Camera capture structure
typedef struct {
    int fd;                     // V4L2 file descriptor
    uint32_t width;             // Frame width
    uint32_t height;            // Frame height
    uint32_t format;            // V4L2 pixel format (e.g., V4L2_PIX_FMT_YUYV)
    uint32_t bytesperline;      // Bytes per line
    uint32_t sizeimage;         // Total image size in bytes

    // Memory-mapped buffers
    struct {
        void* start;
        size_t length;
    } buffers[CAMERA_BUFFER_COUNT];
    int buffer_count;

    // Current frame (converted to RGB)
    uint8_t* rgb_frame;         // RGB24 frame buffer
    uint32_t rgb_size;          // Size of RGB buffer

    bool streaming;             // True if streaming is active
} camera_t;

/**
 * Initialize camera capture
 * @param camera Output camera structure
 * @param device Device path (e.g., "/dev/video0")
 * @param width Requested width (0 for default)
 * @param height Requested height (0 for default)
 * @return true on success, false on failure
 */
bool camera_init(camera_t* camera, const char* device, uint32_t width, uint32_t height);

/**
 * Start camera streaming
 * @param camera Camera structure
 * @return true on success
 */
bool camera_start(camera_t* camera);

/**
 * Stop camera streaming
 * @param camera Camera structure
 */
void camera_stop(camera_t* camera);

/**
 * Capture a frame (blocks until frame is ready)
 * @param camera Camera structure
 * @return Pointer to RGB frame data, or NULL on failure
 */
uint8_t* camera_capture_frame(camera_t* camera);

/**
 * Capture the freshest frame (drains buffer queue and returns newest)
 * Use this when you need the most current frame, not the oldest queued one.
 * @param camera Camera structure
 * @return Pointer to RGB frame data, or NULL on failure
 */
uint8_t* camera_capture_frame_fresh(camera_t* camera);

/**
 * Clean up camera resources
 * @param camera Camera structure
 */
void camera_cleanup(camera_t* camera);

/**
 * Get pixel at (x, y) from RGB frame
 * @param camera Camera structure
 * @param x X coordinate
 * @param y Y coordinate
 * @param r Output red component
 * @param g Output green component
 * @param b Output blue component
 */
void camera_get_pixel(camera_t* camera, int x, int y, uint8_t* r, uint8_t* g, uint8_t* b);

#endif // V4L2_CAPTURE_H
