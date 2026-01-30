/**
 * Color Detection Module
 * Identifies dominant colors in an image
 */

#ifndef COLOR_DETECT_H
#define COLOR_DETECT_H

#include <stdint.h>
#include <stdbool.h>

// Recognized colors
typedef enum {
    COLOR_UNKNOWN = 0,
    COLOR_WHITE,
    COLOR_BLACK,
    COLOR_RED,
    COLOR_GREEN,
    COLOR_BLUE,
    COLOR_YELLOW,
    COLOR_ORANGE,
    COLOR_PURPLE,
    COLOR_BROWN,
    COLOR_COUNT
} detected_color_t;

/**
 * Get the name of a detected color
 * @param color The detected color enum
 * @return Color name string (e.g., "RED", "BLUE", "?")
 */
const char* color_get_name(detected_color_t color);

/**
 * Detect the dominant color in an RGB image
 * @param rgb_data RGB24 image data (3 bytes per pixel)
 * @param width Image width
 * @param height Image height
 * @param dominance_threshold Minimum percentage (0-100) for a color to be dominant
 * @param out_percentage Output: percentage of the dominant color (can be NULL)
 * @return The detected dominant color, or COLOR_UNKNOWN if none is dominant
 */
detected_color_t color_detect_dominant(const uint8_t* rgb_data, int width, int height,
                                        int dominance_threshold, int* out_percentage);

#endif // COLOR_DETECT_H
