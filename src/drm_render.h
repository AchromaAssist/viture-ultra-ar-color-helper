/**
 * DRM Direct Rendering for Viture AR Glasses
 */

#ifndef DRM_RENDER_H
#define DRM_RENDER_H

#include <stdbool.h>
#include <stdint.h>

// Display info structure
typedef struct {
    int fd;                     // DRM file descriptor
    uint32_t connector_id;
    uint32_t crtc_id;
    uint32_t fb_id;
    uint32_t width;
    uint32_t height;
    uint32_t pitch;
    uint32_t size;
    uint32_t handle;
    uint8_t* framebuffer;       // Memory-mapped framebuffer (DRM)
    uint8_t* shadow_buffer;     // Shadow buffer for preparing frames
    char connector_name[32];
    char mode_name[32];
} drm_display_t;

/**
 * Initialize DRM and find Viture display
 * @param display Output display structure
 * @return true on success, false on failure
 */
bool drm_init(drm_display_t* display);

/**
 * Clean up DRM resources
 * @param display Display structure to clean up
 */
void drm_cleanup(drm_display_t* display);

/**
 * Clear the framebuffer to a solid color
 * @param display Display structure
 * @param r Red component (0-255)
 * @param g Green component (0-255)
 * @param b Blue component (0-255)
 */
void drm_clear(drm_display_t* display, uint8_t r, uint8_t g, uint8_t b);

/**
 * Copy front buffer to back buffer (for incremental updates)
 * @param display Display structure
 */
void drm_copy_front_to_back(drm_display_t* display);

/**
 * Draw a filled rectangle
 * @param display Display structure
 * @param x X position
 * @param y Y position
 * @param w Width
 * @param h Height
 * @param r Red component (0-255)
 * @param g Green component (0-255)
 * @param b Blue component (0-255)
 */
void drm_fill_rect(drm_display_t* display, int x, int y, int w, int h,
                   uint8_t r, uint8_t g, uint8_t b);

/**
 * Swap buffers / present the framebuffer
 * @param display Display structure
 * @return true on success
 */
bool drm_present(drm_display_t* display);

/**
 * Draw an RGB image at the specified position
 * @param display Display structure
 * @param x X position
 * @param y Y position
 * @param width Image width
 * @param height Image height
 * @param rgb_data RGB24 image data (3 bytes per pixel: R, G, B)
 */
void drm_draw_image(drm_display_t* display, int x, int y, int width, int height,
                    const uint8_t* rgb_data);

/**
 * Draw a rectangle border (outline only)
 * @param display Display structure
 * @param x X position
 * @param y Y position
 * @param w Width
 * @param h Height
 * @param thickness Border thickness
 * @param r Red component (0-255)
 * @param g Green component (0-255)
 * @param b Blue component (0-255)
 */
void drm_draw_border(drm_display_t* display, int x, int y, int w, int h,
                     int thickness, uint8_t r, uint8_t g, uint8_t b);

/**
 * Draw text at the specified position
 * @param display Display structure
 * @param x X position
 * @param y Y position
 * @param text Text string to draw
 * @param scale Font scale (1 = 8x8 pixels per char, 2 = 16x16, etc.)
 * @param r Red component (0-255)
 * @param g Green component (0-255)
 * @param b Blue component (0-255)
 */
void drm_draw_text(drm_display_t* display, int x, int y, const char* text,
                   int scale, uint8_t r, uint8_t g, uint8_t b);

/**
 * Get the width of text in pixels
 * @param text Text string
 * @param scale Font scale
 * @return Width in pixels
 */
int drm_text_width(const char* text, int scale);

/**
 * Check if display is in stereo (side-by-side) mode
 * @param display Display structure
 * @return true if width > 1920 (3840x1200 SBS mode)
 */
bool drm_is_stereo_mode(drm_display_t* display);

/**
 * Draw an RGB image with stereo disparity (for 3D SBS mode)
 * Renders to both left and right eye regions with horizontal offset
 * @param display Display structure
 * @param x Base X position (center of display)
 * @param y Y position
 * @param width Image width
 * @param height Image height
 * @param rgb_data RGB24 image data
 * @param disparity Stereo disparity in pixels (positive = closer than screen)
 */
void drm_draw_image_stereo(drm_display_t* display, int x, int y, int width, int height,
                           const uint8_t* rgb_data, int disparity);

/**
 * Draw a border with stereo disparity (for 3D SBS mode)
 * @param display Display structure
 * @param x Base X position
 * @param y Y position
 * @param w Width
 * @param h Height
 * @param thickness Border thickness
 * @param r Red component
 * @param g Green component
 * @param b Blue component
 * @param disparity Stereo disparity in pixels
 */
void drm_draw_border_stereo(drm_display_t* display, int x, int y, int w, int h,
                            int thickness, uint8_t r, uint8_t g, uint8_t b, int disparity);

/**
 * Draw text with stereo disparity (for 3D SBS mode)
 * @param display Display structure
 * @param x Base X position
 * @param y Y position
 * @param text Text string
 * @param scale Font scale
 * @param r Red component
 * @param g Green component
 * @param b Blue component
 * @param disparity Stereo disparity in pixels
 */
void drm_draw_text_stereo(drm_display_t* display, int x, int y, const char* text,
                          int scale, uint8_t r, uint8_t g, uint8_t b, int disparity);

#endif // DRM_RENDER_H
