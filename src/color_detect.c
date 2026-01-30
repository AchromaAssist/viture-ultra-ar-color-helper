/**
 * Color Detection Implementation
 */

#include "color_detect.h"
#include <string.h>

// Color names
static const char* COLOR_NAMES[] = {
    "?",        // UNKNOWN
    "WHITE",
    "BLACK",
    "RED",
    "GREEN",
    "BLUE",
    "YELLOW",
    "ORANGE",
    "PURPLE",
    "BROWN"
};

const char* color_get_name(detected_color_t color) {
    if (color >= 0 && color < COLOR_COUNT) {
        return COLOR_NAMES[color];
    }
    return "?";
}

// Convert RGB to HSV
// H: 0-360, S: 0-100, V: 0-100
static void rgb_to_hsv(uint8_t r, uint8_t g, uint8_t b, int* h, int* s, int* v) {
    float rf = r / 255.0f;
    float gf = g / 255.0f;
    float bf = b / 255.0f;

    float max = rf > gf ? (rf > bf ? rf : bf) : (gf > bf ? gf : bf);
    float min = rf < gf ? (rf < bf ? rf : bf) : (gf < bf ? gf : bf);
    float delta = max - min;

    // Value
    *v = (int)(max * 100);

    // Saturation
    if (max == 0) {
        *s = 0;
    } else {
        *s = (int)((delta / max) * 100);
    }

    // Hue
    if (delta == 0) {
        *h = 0;
    } else if (max == rf) {
        *h = (int)(60 * (((gf - bf) / delta)));
        if (*h < 0) *h += 360;
    } else if (max == gf) {
        *h = (int)(60 * (((bf - rf) / delta) + 2));
    } else {
        *h = (int)(60 * (((rf - gf) / delta) + 4));
    }
}

// Classify a single pixel color based on HSV
static detected_color_t classify_pixel(uint8_t r, uint8_t g, uint8_t b) {
    int h, s, v;
    rgb_to_hsv(r, g, b, &h, &s, &v);

    // Black: very low value
    if (v < 15) {
        return COLOR_BLACK;
    }

    // White: low saturation, high value
    if (s < 15 && v > 70) {
        return COLOR_WHITE;
    }

    // Gray (treat as unknown for now, or could be white/black based on value)
    if (s < 15) {
        if (v > 50) return COLOR_WHITE;
        return COLOR_BLACK;
    }

    // Brown: low saturation, orange-ish hue, medium value
    if (h >= 10 && h <= 40 && s >= 20 && s <= 70 && v >= 15 && v <= 60) {
        return COLOR_BROWN;
    }

    // Chromatic colors based on hue
    // Red: 0-15 or 345-360
    if (h <= 15 || h >= 345) {
        return COLOR_RED;
    }
    // Orange: 15-40
    if (h > 15 && h <= 40) {
        return COLOR_ORANGE;
    }
    // Yellow: 40-70
    if (h > 40 && h <= 70) {
        return COLOR_YELLOW;
    }
    // Green: 70-165
    if (h > 70 && h <= 165) {
        return COLOR_GREEN;
    }
    // Blue: 165-260
    if (h > 165 && h <= 260) {
        return COLOR_BLUE;
    }
    // Purple: 260-345
    if (h > 260 && h < 345) {
        return COLOR_PURPLE;
    }

    return COLOR_UNKNOWN;
}

detected_color_t color_detect_dominant(const uint8_t* rgb_data, int width, int height,
                                        int dominance_threshold, int* out_percentage) {
    if (!rgb_data || width <= 0 || height <= 0) {
        if (out_percentage) *out_percentage = 0;
        return COLOR_UNKNOWN;
    }

    // Count pixels for each color
    int color_counts[COLOR_COUNT] = {0};
    int total_pixels = width * height;

    for (int i = 0; i < total_pixels; i++) {
        uint8_t r = rgb_data[i * 3 + 0];
        uint8_t g = rgb_data[i * 3 + 1];
        uint8_t b = rgb_data[i * 3 + 2];

        detected_color_t color = classify_pixel(r, g, b);
        color_counts[color]++;
    }

    // Find the most common color (excluding unknown)
    int max_count = 0;
    detected_color_t dominant = COLOR_UNKNOWN;

    for (int i = 1; i < COLOR_COUNT; i++) {  // Start from 1 to skip UNKNOWN
        if (color_counts[i] > max_count) {
            max_count = color_counts[i];
            dominant = (detected_color_t)i;
        }
    }

    // Calculate percentage
    int percentage = (max_count * 100) / total_pixels;

    if (out_percentage) {
        *out_percentage = percentage;
    }

    // Check if it meets the dominance threshold
    if (percentage >= dominance_threshold) {
        return dominant;
    }

    return COLOR_UNKNOWN;
}
