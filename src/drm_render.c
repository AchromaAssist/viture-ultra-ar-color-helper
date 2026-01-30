/**
 * DRM Direct Rendering Implementation for Viture AR Glasses
 */

#include "drm_render.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <sys/mman.h>
#include <sys/ioctl.h>

#include <xf86drm.h>
#include <xf86drmMode.h>
#include <drm_fourcc.h>

// Property name for non-desktop displays (VR headsets, AR glasses)
#define NON_DESKTOP_PROPERTY "non-desktop"

// Find property ID by name
static uint32_t find_property_id(int fd, uint32_t connector_id, const char* name) {
    drmModeObjectProperties* props = drmModeObjectGetProperties(fd, connector_id,
                                                                  DRM_MODE_OBJECT_CONNECTOR);
    if (!props) return 0;

    uint32_t prop_id = 0;
    for (uint32_t i = 0; i < props->count_props; i++) {
        drmModePropertyRes* prop = drmModeGetProperty(fd, props->props[i]);
        if (prop) {
            if (strcmp(prop->name, name) == 0) {
                prop_id = prop->prop_id;
            }
            drmModeFreeProperty(prop);
            if (prop_id) break;
        }
    }

    drmModeFreeObjectProperties(props);
    return prop_id;
}

// Get current property value
static bool get_property_value(int fd, uint32_t connector_id, const char* name, uint64_t* value) {
    drmModeObjectProperties* props = drmModeObjectGetProperties(fd, connector_id,
                                                                  DRM_MODE_OBJECT_CONNECTOR);
    if (!props) return false;

    bool found = false;
    for (uint32_t i = 0; i < props->count_props; i++) {
        drmModePropertyRes* prop = drmModeGetProperty(fd, props->props[i]);
        if (prop) {
            if (strcmp(prop->name, name) == 0) {
                *value = props->prop_values[i];
                found = true;
            }
            drmModeFreeProperty(prop);
            if (found) break;
        }
    }

    drmModeFreeObjectProperties(props);
    return found;
}

// Check if connector is marked as non-desktop
static bool is_non_desktop(int fd, uint32_t connector_id) {
    uint64_t value = 0;
    if (get_property_value(fd, connector_id, NON_DESKTOP_PROPERTY, &value)) {
        return value != 0;
    }
    return false;
}

// Set non-desktop property on connector
static bool set_non_desktop(int fd, uint32_t connector_id, bool non_desktop) {
    uint32_t prop_id = find_property_id(fd, connector_id, NON_DESKTOP_PROPERTY);
    if (!prop_id) {
        printf("[DRM] non-desktop property not found on connector %u\n", connector_id);
        return false;
    }

    int ret = drmModeConnectorSetProperty(fd, connector_id, prop_id, non_desktop ? 1 : 0);
    if (ret != 0) {
        printf("[DRM] Failed to set non-desktop=%d on connector %u: %s\n",
               non_desktop, connector_id, strerror(errno));
        return false;
    }

    printf("[DRM] Set non-desktop=%d on connector %u\n", non_desktop, connector_id);
    return true;
}

// Try to create a DRM lease for the given objects
static int try_drm_lease(int fd, uint32_t* objects, int count) {
    uint32_t lessee_id = 0;
    int lease_fd = drmModeCreateLease(fd, objects, count, 0, &lessee_id);
    if (lease_fd >= 0) {
        printf("[DRM] Created DRM lease (lessee_id=%u, fd=%d)\n", lessee_id, lease_fd);
    }
    return lease_fd;
}

// Check if connector has Viture-like resolution (1920x1200)
static bool has_viture_resolution(drmModeConnector* connector) {
    for (int i = 0; i < connector->count_modes; i++) {
        // Viture glasses typically have 1920x1200 resolution
        if (connector->modes[i].hdisplay == 1920 &&
            connector->modes[i].vdisplay == 1200) {
            return true;
        }
    }
    return false;
}

// Get connector type name
static const char* get_connector_type_name(uint32_t type) {
    switch (type) {
        case DRM_MODE_CONNECTOR_HDMIA:
        case DRM_MODE_CONNECTOR_HDMIB:
            return "HDMI";
        case DRM_MODE_CONNECTOR_DisplayPort:
            return "DP";
        case DRM_MODE_CONNECTOR_USB:
            return "USB";
        case DRM_MODE_CONNECTOR_VGA:
            return "VGA";
        case DRM_MODE_CONNECTOR_DVID:
        case DRM_MODE_CONNECTOR_DVII:
            return "DVI";
        case DRM_MODE_CONNECTOR_eDP:
            return "eDP";
        default:
            return "Unknown";
    }
}

// Check if a connector is likely a Viture display
static bool is_viture_display(drmModeConnector* connector, int fd) {
    if (connector->connection != DRM_MODE_CONNECTED) {
        return false;
    }

    printf("[DRM] Checking connector: %s-%u (type=%d)\n",
           get_connector_type_name(connector->connector_type),
           connector->connector_type_id, connector->connector_type);

    // Check if already marked as non-desktop (VR/AR device)
    if (is_non_desktop(fd, connector->connector_id)) {
        printf("[DRM]   -> Marked as non-desktop (VR/AR device)\n");
        return true;
    }

    // Check for Viture-specific resolution (1920x1200)
    if (has_viture_resolution(connector)) {
        printf("[DRM]   -> Has Viture resolution (1920x1200), likely Viture glasses\n");
        return true;
    }

    return false;
}

// Find a CRTC for the connector
static uint32_t find_crtc(int fd, drmModeRes* res, drmModeConnector* connector, uint32_t* taken_crtcs) {
    // First, try the encoder currently attached to the connector
    if (connector->encoder_id) {
        drmModeEncoder* encoder = drmModeGetEncoder(fd, connector->encoder_id);
        if (encoder) {
            uint32_t crtc_id = encoder->crtc_id;
            drmModeFreeEncoder(encoder);
            if (crtc_id && !(*taken_crtcs & (1 << crtc_id))) {
                *taken_crtcs |= (1 << crtc_id);
                return crtc_id;
            }
        }
    }

    // Find a CRTC that can work with this connector
    for (int i = 0; i < connector->count_encoders; i++) {
        drmModeEncoder* encoder = drmModeGetEncoder(fd, connector->encoders[i]);
        if (!encoder) continue;

        for (int j = 0; j < res->count_crtcs; j++) {
            if (!(encoder->possible_crtcs & (1 << j))) continue;

            uint32_t crtc_id = res->crtcs[j];
            if (!(*taken_crtcs & (1 << crtc_id))) {
                *taken_crtcs |= (1 << crtc_id);
                drmModeFreeEncoder(encoder);
                return crtc_id;
            }
        }
        drmModeFreeEncoder(encoder);
    }

    return 0;
}

// Create a dumb buffer with shadow buffer for flicker-free rendering
static bool create_dumb_buffer(int fd, drm_display_t* display) {
    struct drm_mode_create_dumb create = {0};
    struct drm_mode_map_dumb map = {0};

    // Create dumb buffer
    create.width = display->width;
    create.height = display->height;
    create.bpp = 32;  // XRGB8888

    if (ioctl(fd, DRM_IOCTL_MODE_CREATE_DUMB, &create) < 0) {
        printf("[DRM] Failed to create dumb buffer: %s\n", strerror(errno));
        return false;
    }

    display->handle = create.handle;
    display->pitch = create.pitch;
    display->size = create.size;

    printf("[DRM] Created dumb buffer: %ux%u, pitch=%u, size=%u\n",
           display->width, display->height, display->pitch, display->size);

    // Create framebuffer
    uint32_t handles[4] = {display->handle, 0, 0, 0};
    uint32_t pitches[4] = {display->pitch, 0, 0, 0};
    uint32_t offsets[4] = {0, 0, 0, 0};

    if (drmModeAddFB2(fd, display->width, display->height, DRM_FORMAT_XRGB8888,
                      handles, pitches, offsets, &display->fb_id, 0) != 0) {
        printf("[DRM] Failed to create framebuffer: %s\n", strerror(errno));
        goto err_destroy;
    }

    printf("[DRM] Created framebuffer: id=%u\n", display->fb_id);

    // Map DRM buffer for CPU access
    map.handle = display->handle;
    if (ioctl(fd, DRM_IOCTL_MODE_MAP_DUMB, &map) < 0) {
        printf("[DRM] Failed to map dumb buffer: %s\n", strerror(errno));
        goto err_fb;
    }

    uint8_t* drm_buffer = mmap(NULL, display->size, PROT_READ | PROT_WRITE,
                                MAP_SHARED, fd, map.offset);
    if (drm_buffer == MAP_FAILED) {
        printf("[DRM] Failed to mmap framebuffer: %s\n", strerror(errno));
        goto err_fb;
    }

    // Allocate shadow buffer in regular memory (faster for drawing)
    display->shadow_buffer = malloc(display->size);
    if (!display->shadow_buffer) {
        printf("[DRM] Failed to allocate shadow buffer\n");
        munmap(drm_buffer, display->size);
        goto err_fb;
    }

    // Store DRM buffer pointer temporarily, but use shadow for drawing
    display->framebuffer = display->shadow_buffer;

    // Store the DRM mapped buffer for present (we'll copy shadow -> drm)
    // We need to keep track of the drm buffer, let's reuse a field
    // Actually, let's just store it in framebuffer and use shadow for API
    // Swap them: framebuffer = shadow (for drawing API), we store drm separately

    // Clear both buffers to black
    memset(display->shadow_buffer, 0, display->size);
    memset(drm_buffer, 0, display->size);

    // Store DRM buffer address - we'll retrieve it in present
    // Use a simple approach: store in the memory right after shadow_buffer allocation
    // Actually simpler: just keep drm_buffer mapped and store its address
    // We need another field... let's repurpose: shadow_buffer for drawing, framebuffer for DRM
    display->framebuffer = drm_buffer;  // DRM buffer
    // shadow_buffer already set above

    printf("[DRM] Shadow buffer rendering enabled\n");

    return true;

err_fb:
    drmModeRmFB(fd, display->fb_id);
    display->fb_id = 0;
err_destroy:
    {
        struct drm_mode_destroy_dumb destroy = {.handle = display->handle};
        ioctl(fd, DRM_IOCTL_MODE_DESTROY_DUMB, &destroy);
        display->handle = 0;
    }
    return false;
}

// Destroy dumb buffer
static void destroy_dumb_buffer(int fd, drm_display_t* display) {
    if (display->shadow_buffer) {
        free(display->shadow_buffer);
        display->shadow_buffer = NULL;
    }

    if (display->framebuffer) {
        munmap(display->framebuffer, display->size);
        display->framebuffer = NULL;
    }

    if (display->fb_id) {
        drmModeRmFB(fd, display->fb_id);
        display->fb_id = 0;
    }

    if (display->handle) {
        struct drm_mode_destroy_dumb destroy = {.handle = display->handle};
        ioctl(fd, DRM_IOCTL_MODE_DESTROY_DUMB, &destroy);
        display->handle = 0;
    }
    display->fb_id = 0;
    display->handle = 0;
}

bool drm_init(drm_display_t* display) {
    memset(display, 0, sizeof(*display));
    display->fd = -1;

    printf("[DRM] Initializing DRM...\n");

    // Try to open DRM devices
    char drm_path[64];
    int fd = -1;

    for (int i = 0; i < 16; i++) {
        snprintf(drm_path, sizeof(drm_path), "/dev/dri/card%d", i);
        fd = open(drm_path, O_RDWR | O_CLOEXEC);
        if (fd >= 0) {
            // Check if this device has the required capabilities
            uint64_t has_dumb = 0;
            if (drmGetCap(fd, DRM_CAP_DUMB_BUFFER, &has_dumb) < 0 || !has_dumb) {
                printf("[DRM] %s: No dumb buffer support\n", drm_path);
                close(fd);
                fd = -1;
                continue;
            }

            printf("[DRM] Opened %s (fd=%d)\n", drm_path, fd);
            break;
        }
    }

    if (fd < 0) {
        printf("[DRM] Failed to open any DRM device\n");
        return false;
    }

    // Get DRM resources
    drmModeRes* res = drmModeGetResources(fd);
    if (!res) {
        printf("[DRM] Failed to get DRM resources: %s\n", strerror(errno));
        close(fd);
        return false;
    }

    printf("[DRM] Found %d connectors, %d CRTCs, %d encoders\n",
           res->count_connectors, res->count_crtcs, res->count_encoders);

    // Find a suitable connector (preferably Viture display)
    drmModeConnector* connector = NULL;
    drmModeConnector* viture_connector = NULL;
    drmModeConnector* any_connector = NULL;
    uint32_t taken_crtcs = 0;

    for (int i = 0; i < res->count_connectors; i++) {
        connector = drmModeGetConnector(fd, res->connectors[i]);
        if (!connector) continue;

        printf("[DRM] Connector %d: type=%d, connection=%s, modes=%d\n",
               connector->connector_id,
               connector->connector_type,
               connector->connection == DRM_MODE_CONNECTED ? "connected" : "disconnected",
               connector->count_modes);

        if (connector->connection == DRM_MODE_CONNECTED && connector->count_modes > 0) {
            // Print available modes
            for (int m = 0; m < connector->count_modes && m < 5; m++) {
                printf("[DRM]   Mode %d: %s %dx%d@%d\n", m,
                       connector->modes[m].name,
                       connector->modes[m].hdisplay,
                       connector->modes[m].vdisplay,
                       connector->modes[m].vrefresh);
            }

            if (is_viture_display(connector, fd)) {
                if (!viture_connector) {
                    viture_connector = connector;
                    connector = NULL;
                }
            }
            if (!any_connector && !viture_connector) {
                any_connector = connector;
                connector = NULL;
            }
        }

        if (connector) {
            drmModeFreeConnector(connector);
        }
    }

    // Prefer Viture display, fall back to any connected display
    connector = viture_connector ? viture_connector : any_connector;

    if (!connector) {
        printf("[DRM] No connected display found\n");
        drmModeFreeResources(res);
        close(fd);
        return false;
    }

    // Store connector info
    display->connector_id = connector->connector_id;
    snprintf(display->connector_name, sizeof(display->connector_name),
             "%s-%u", get_connector_type_name(connector->connector_type),
             connector->connector_type_id);

    printf("[DRM] Using connector: %s (id=%u)\n",
           display->connector_name, display->connector_id);

    // Select the best mode - prefer 3840x (3D SBS), then 1920x (2D)
    drmModeModeInfo* mode = &connector->modes[0];
    drmModeModeInfo* stereo_mode_1200 = NULL;  // 3840x1200 (3D SBS)
    drmModeModeInfo* stereo_mode_1080 = NULL;  // 3840x1080 (3D SBS)
    drmModeModeInfo* viture_mode = NULL;       // 1920x1200 (2D)
    drmModeModeInfo* fallback_mode = NULL;     // 1920x1080

    for (int i = 0; i < connector->count_modes; i++) {
        drmModeModeInfo* m = &connector->modes[i];

        // Prefer 3840x1200@60Hz (Viture 3D SBS mode - 1200p)
        if (m->hdisplay == 3840 && m->vdisplay == 1200 && m->vrefresh == 60) {
            stereo_mode_1200 = m;
        }
        // Also accept 3840x1080@60Hz (3D SBS mode - 1080p)
        if (m->hdisplay == 3840 && m->vdisplay == 1080 && m->vrefresh == 60) {
            stereo_mode_1080 = m;
        }
        // 1920x1200@60Hz (Viture 2D native)
        if (m->hdisplay == 1920 && m->vdisplay == 1200 && m->vrefresh == 60) {
            viture_mode = m;
        }
        // Fallback to 1920x1080@60Hz
        if (m->hdisplay == 1920 && m->vdisplay == 1080 && m->vrefresh == 60) {
            if (!fallback_mode) fallback_mode = m;
        }
    }

    // Priority: stereo_1200 > stereo_1080 > viture > fallback
    if (stereo_mode_1200) {
        mode = stereo_mode_1200;
        printf("[DRM] Using 3D SBS stereo mode (3840x1200)\n");
    } else if (stereo_mode_1080) {
        mode = stereo_mode_1080;
        printf("[DRM] Using 3D SBS stereo mode (3840x1080)\n");
    } else if (viture_mode) {
        mode = viture_mode;
    } else if (fallback_mode) {
        mode = fallback_mode;
    }

    display->width = mode->hdisplay;
    display->height = mode->vdisplay;
    strncpy(display->mode_name, mode->name, sizeof(display->mode_name) - 1);

    printf("[DRM] Selected mode: %s (%ux%u@%uHz)\n",
           display->mode_name, display->width, display->height, mode->vrefresh);

    // Find CRTC
    display->crtc_id = find_crtc(fd, res, connector, &taken_crtcs);
    if (!display->crtc_id) {
        printf("[DRM] Failed to find CRTC for connector\n");
        drmModeFreeConnector(connector);
        drmModeFreeResources(res);
        close(fd);
        return false;
    }

    printf("[DRM] Using CRTC %u for connector %u\n", display->crtc_id, display->connector_id);

    // Create framebuffer
    display->fd = fd;
    if (!create_dumb_buffer(fd, display)) {
        printf("[DRM] Failed to create framebuffer\n");
        drmModeFreeConnector(connector);
        drmModeFreeResources(res);
        close(fd);
        display->fd = -1;
        return false;
    }

    // Try to mark the display as non-desktop first
    // This may allow the display manager to release it
    printf("[DRM] Checking non-desktop property...\n");
    bool was_non_desktop = is_non_desktop(fd, display->connector_id);
    if (!was_non_desktop) {
        printf("[DRM] Attempting to set non-desktop property...\n");
        set_non_desktop(fd, display->connector_id, true);
    } else {
        printf("[DRM] Connector already marked as non-desktop\n");
    }

    // Try DRM leasing first (works with modern compositors)
    printf("[DRM] Attempting DRM lease...\n");
    uint32_t lease_objects[] = {display->connector_id, display->crtc_id};
    int lease_fd = try_drm_lease(fd, lease_objects, 2);

    int working_fd = fd;
    bool using_lease = false;

    if (lease_fd >= 0) {
        printf("[DRM] Using DRM lease for display access\n");
        working_fd = lease_fd;
        using_lease = true;
    } else {
        printf("[DRM] DRM lease not available: %s\n", strerror(errno));
        printf("[DRM] Falling back to DRM master...\n");

        // Try to become DRM master
        printf("[DRM] Attempting to acquire DRM master...\n");
        if (drmSetMaster(fd) != 0) {
            printf("[DRM] Warning: Could not acquire DRM master: %s\n", strerror(errno));
            printf("[DRM] This usually means a display manager is running.\n");
        }
    }

    // Recreate framebuffer on the working fd if using lease
    if (using_lease) {
        // Need to recreate the framebuffer on the lease fd
        destroy_dumb_buffer(fd, display);
        display->fd = lease_fd;
        if (!create_dumb_buffer(lease_fd, display)) {
            printf("[DRM] Failed to create framebuffer on lease fd\n");
            close(lease_fd);
            close(fd);
            drmModeFreeConnector(connector);
            drmModeFreeResources(res);
            return false;
        }
    }

    // Set the mode
    printf("[DRM] Setting mode on CRTC %u (fd=%d, fb=%u)...\n",
           display->crtc_id, working_fd, display->fb_id);
    if (drmModeSetCrtc(working_fd, display->crtc_id, display->fb_id, 0, 0,
                       &display->connector_id, 1, mode) != 0) {
        printf("[DRM] Failed to set CRTC mode: %s\n", strerror(errno));
        if (errno == EACCES || errno == EPERM) {
            printf("[DRM] Permission denied - display manager likely holds DRM master.\n");
            printf("[DRM] Solutions:\n");
            printf("[DRM]   1. Set non-desktop=1: xrandr --output %s --set non-desktop 1\n",
                   display->connector_name);
            printf("[DRM]   2. Run from a TTY (Ctrl+Alt+F3) without desktop\n");
            printf("[DRM]   3. Stop display manager: sudo systemctl stop gdm\n");
            printf("[DRM]   4. Use a compositor with DRM lease support (e.g., gamescope)\n");
        }
        if (!using_lease) {
            drmDropMaster(fd);
        }
        destroy_dumb_buffer(working_fd, display);
        drmModeFreeConnector(connector);
        drmModeFreeResources(res);
        if (using_lease) {
            close(lease_fd);
        }
        close(fd);
        display->fd = -1;
        return false;
    }

    printf("[DRM] Successfully initialized display: %ux%u\n", display->width, display->height);

    // Store the working fd and close the unused one
    if (using_lease) {
        display->fd = lease_fd;
        close(fd);  // Close original fd, we use lease_fd now
    } else {
        display->fd = fd;
    }

    // Free connector resources
    if (viture_connector && any_connector) {
        drmModeFreeConnector(any_connector);
    }
    drmModeFreeConnector(connector);
    drmModeFreeResources(res);

    return true;
}

void drm_cleanup(drm_display_t* display) {
    if (!display || display->fd < 0) return;

    printf("[DRM] Cleaning up...\n");

    destroy_dumb_buffer(display->fd, display);

    close(display->fd);
    display->fd = -1;

    printf("[DRM] Cleanup complete\n");
}

void drm_clear(drm_display_t* display, uint8_t r, uint8_t g, uint8_t b) {
    if (!display || !display->shadow_buffer) return;

    uint32_t color = (r << 16) | (g << 8) | b;
    uint32_t* pixels = (uint32_t*)display->shadow_buffer;
    size_t pixel_count = display->size / 4;

    for (size_t i = 0; i < pixel_count; i++) {
        pixels[i] = color;
    }
}

void drm_copy_front_to_back(drm_display_t* display) {
    // No-op with shadow buffer approach - shadow is always the active drawing target
    (void)display;
}

void drm_fill_rect(drm_display_t* display, int x, int y, int w, int h,
                   uint8_t r, uint8_t g, uint8_t b) {
    if (!display || !display->shadow_buffer) return;

    uint32_t color = (r << 16) | (g << 8) | b;
    uint32_t* pixels = (uint32_t*)display->shadow_buffer;
    uint32_t stride = display->pitch / 4;

    // Clamp to display bounds
    if (x < 0) { w += x; x = 0; }
    if (y < 0) { h += y; y = 0; }
    if (x + w > (int)display->width) w = display->width - x;
    if (y + h > (int)display->height) h = display->height - y;
    if (w <= 0 || h <= 0) return;

    for (int row = y; row < y + h; row++) {
        for (int col = x; col < x + w; col++) {
            pixels[row * stride + col] = color;
        }
    }
}

bool drm_present(drm_display_t* display) {
    if (!display || display->fd < 0) return false;

    if (display->shadow_buffer && display->framebuffer) {
        // Copy shadow buffer to DRM framebuffer in one fast operation
        // This minimizes the time the display shows partial content
        memcpy(display->framebuffer, display->shadow_buffer, display->size);
    }

    return true;
}

void drm_draw_image(drm_display_t* display, int x, int y, int width, int height,
                    const uint8_t* rgb_data) {
    if (!display || !display->shadow_buffer || !rgb_data) return;

    uint32_t* pixels = (uint32_t*)display->shadow_buffer;
    uint32_t stride = display->pitch / 4;

    for (int row = 0; row < height; row++) {
        int dst_y = y + row;
        if (dst_y < 0 || dst_y >= (int)display->height) continue;

        for (int col = 0; col < width; col++) {
            int dst_x = x + col;
            if (dst_x < 0 || dst_x >= (int)display->width) continue;

            int src_idx = (row * width + col) * 3;
            uint8_t r = rgb_data[src_idx + 0];
            uint8_t g = rgb_data[src_idx + 1];
            uint8_t b = rgb_data[src_idx + 2];

            uint32_t color = (r << 16) | (g << 8) | b;
            pixels[dst_y * stride + dst_x] = color;
        }
    }
}

void drm_draw_border(drm_display_t* display, int x, int y, int w, int h,
                     int thickness, uint8_t r, uint8_t g, uint8_t b) {
    if (!display || !display->shadow_buffer || thickness <= 0) return;

    // Top border
    drm_fill_rect(display, x, y, w, thickness, r, g, b);
    // Bottom border
    drm_fill_rect(display, x, y + h - thickness, w, thickness, r, g, b);
    // Left border
    drm_fill_rect(display, x, y, thickness, h, r, g, b);
    // Right border
    drm_fill_rect(display, x + w - thickness, y, thickness, h, r, g, b);
}

// Simple 8x8 bitmap font (uppercase letters, numbers, and some symbols)
// Each character is 8 bytes, each byte is a row (MSB = leftmost pixel)
static const uint8_t FONT_8X8[][8] = {
    // Space (32)
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    // ! (33)
    {0x18, 0x18, 0x18, 0x18, 0x18, 0x00, 0x18, 0x00},
    // " (34)
    {0x6C, 0x6C, 0x24, 0x00, 0x00, 0x00, 0x00, 0x00},
    // # (35)
    {0x6C, 0xFE, 0x6C, 0x6C, 0xFE, 0x6C, 0x00, 0x00},
    // $ (36)
    {0x18, 0x7E, 0xC0, 0x7C, 0x06, 0xFC, 0x18, 0x00},
    // % (37)
    {0xC6, 0xCC, 0x18, 0x30, 0x66, 0xC6, 0x00, 0x00},
    // & (38)
    {0x38, 0x6C, 0x38, 0x76, 0xDC, 0xCC, 0x76, 0x00},
    // ' (39)
    {0x18, 0x18, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00},
    // ( (40)
    {0x0C, 0x18, 0x30, 0x30, 0x30, 0x18, 0x0C, 0x00},
    // ) (41)
    {0x30, 0x18, 0x0C, 0x0C, 0x0C, 0x18, 0x30, 0x00},
    // * (42)
    {0x00, 0x66, 0x3C, 0xFF, 0x3C, 0x66, 0x00, 0x00},
    // + (43)
    {0x00, 0x18, 0x18, 0x7E, 0x18, 0x18, 0x00, 0x00},
    // , (44)
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x18, 0x30},
    // - (45)
    {0x00, 0x00, 0x00, 0x7E, 0x00, 0x00, 0x00, 0x00},
    // . (46)
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x18, 0x00},
    // / (47)
    {0x06, 0x0C, 0x18, 0x30, 0x60, 0xC0, 0x00, 0x00},
    // 0 (48)
    {0x7C, 0xC6, 0xCE, 0xD6, 0xE6, 0xC6, 0x7C, 0x00},
    // 1 (49)
    {0x18, 0x38, 0x18, 0x18, 0x18, 0x18, 0x7E, 0x00},
    // 2 (50)
    {0x7C, 0xC6, 0x06, 0x1C, 0x70, 0xC6, 0xFE, 0x00},
    // 3 (51)
    {0x7C, 0xC6, 0x06, 0x3C, 0x06, 0xC6, 0x7C, 0x00},
    // 4 (52)
    {0x1C, 0x3C, 0x6C, 0xCC, 0xFE, 0x0C, 0x1E, 0x00},
    // 5 (53)
    {0xFE, 0xC0, 0xFC, 0x06, 0x06, 0xC6, 0x7C, 0x00},
    // 6 (54)
    {0x38, 0x60, 0xC0, 0xFC, 0xC6, 0xC6, 0x7C, 0x00},
    // 7 (55)
    {0xFE, 0xC6, 0x0C, 0x18, 0x30, 0x30, 0x30, 0x00},
    // 8 (56)
    {0x7C, 0xC6, 0xC6, 0x7C, 0xC6, 0xC6, 0x7C, 0x00},
    // 9 (57)
    {0x7C, 0xC6, 0xC6, 0x7E, 0x06, 0x0C, 0x78, 0x00},
    // : (58)
    {0x00, 0x18, 0x18, 0x00, 0x00, 0x18, 0x18, 0x00},
    // ; (59)
    {0x00, 0x18, 0x18, 0x00, 0x00, 0x18, 0x18, 0x30},
    // < (60)
    {0x0C, 0x18, 0x30, 0x60, 0x30, 0x18, 0x0C, 0x00},
    // = (61)
    {0x00, 0x00, 0x7E, 0x00, 0x7E, 0x00, 0x00, 0x00},
    // > (62)
    {0x60, 0x30, 0x18, 0x0C, 0x18, 0x30, 0x60, 0x00},
    // ? (63)
    {0x7C, 0xC6, 0x0C, 0x18, 0x18, 0x00, 0x18, 0x00},
    // @ (64)
    {0x7C, 0xC6, 0xDE, 0xDE, 0xDC, 0xC0, 0x7C, 0x00},
    // A (65)
    {0x38, 0x6C, 0xC6, 0xC6, 0xFE, 0xC6, 0xC6, 0x00},
    // B (66)
    {0xFC, 0xC6, 0xC6, 0xFC, 0xC6, 0xC6, 0xFC, 0x00},
    // C (67)
    {0x7C, 0xC6, 0xC0, 0xC0, 0xC0, 0xC6, 0x7C, 0x00},
    // D (68)
    {0xF8, 0xCC, 0xC6, 0xC6, 0xC6, 0xCC, 0xF8, 0x00},
    // E (69)
    {0xFE, 0xC0, 0xC0, 0xFC, 0xC0, 0xC0, 0xFE, 0x00},
    // F (70)
    {0xFE, 0xC0, 0xC0, 0xFC, 0xC0, 0xC0, 0xC0, 0x00},
    // G (71)
    {0x7C, 0xC6, 0xC0, 0xCE, 0xC6, 0xC6, 0x7E, 0x00},
    // H (72)
    {0xC6, 0xC6, 0xC6, 0xFE, 0xC6, 0xC6, 0xC6, 0x00},
    // I (73)
    {0x7E, 0x18, 0x18, 0x18, 0x18, 0x18, 0x7E, 0x00},
    // J (74)
    {0x1E, 0x06, 0x06, 0x06, 0xC6, 0xC6, 0x7C, 0x00},
    // K (75)
    {0xC6, 0xCC, 0xD8, 0xF0, 0xD8, 0xCC, 0xC6, 0x00},
    // L (76)
    {0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xFE, 0x00},
    // M (77)
    {0xC6, 0xEE, 0xFE, 0xD6, 0xC6, 0xC6, 0xC6, 0x00},
    // N (78)
    {0xC6, 0xE6, 0xF6, 0xDE, 0xCE, 0xC6, 0xC6, 0x00},
    // O (79)
    {0x7C, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6, 0x7C, 0x00},
    // P (80)
    {0xFC, 0xC6, 0xC6, 0xFC, 0xC0, 0xC0, 0xC0, 0x00},
    // Q (81)
    {0x7C, 0xC6, 0xC6, 0xC6, 0xD6, 0xDE, 0x7C, 0x06},
    // R (82)
    {0xFC, 0xC6, 0xC6, 0xFC, 0xD8, 0xCC, 0xC6, 0x00},
    // S (83)
    {0x7C, 0xC6, 0xC0, 0x7C, 0x06, 0xC6, 0x7C, 0x00},
    // T (84)
    {0xFE, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x00},
    // U (85)
    {0xC6, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6, 0x7C, 0x00},
    // V (86)
    {0xC6, 0xC6, 0xC6, 0xC6, 0x6C, 0x38, 0x10, 0x00},
    // W (87)
    {0xC6, 0xC6, 0xC6, 0xD6, 0xFE, 0xEE, 0xC6, 0x00},
    // X (88)
    {0xC6, 0xC6, 0x6C, 0x38, 0x6C, 0xC6, 0xC6, 0x00},
    // Y (89)
    {0xC6, 0xC6, 0x6C, 0x38, 0x18, 0x18, 0x18, 0x00},
    // Z (90)
    {0xFE, 0x06, 0x0C, 0x18, 0x30, 0x60, 0xFE, 0x00},
};

#define FONT_FIRST_CHAR 32
#define FONT_CHAR_COUNT (sizeof(FONT_8X8) / sizeof(FONT_8X8[0]))

static const uint8_t* get_char_bitmap(char c) {
    // Convert lowercase to uppercase
    if (c >= 'a' && c <= 'z') {
        c = c - 'a' + 'A';
    }

    int index = c - FONT_FIRST_CHAR;
    if (index >= 0 && index < (int)FONT_CHAR_COUNT) {
        return FONT_8X8[index];
    }
    // Return '?' for unknown characters
    return FONT_8X8['?' - FONT_FIRST_CHAR];
}

void drm_draw_text(drm_display_t* display, int x, int y, const char* text,
                   int scale, uint8_t r, uint8_t g, uint8_t b) {
    if (!display || !display->shadow_buffer || !text || scale < 1) return;

    uint32_t color = (r << 16) | (g << 8) | b;
    uint32_t* pixels = (uint32_t*)display->shadow_buffer;
    uint32_t stride = display->pitch / 4;

    int cursor_x = x;

    while (*text) {
        const uint8_t* bitmap = get_char_bitmap(*text);

        // Draw 8x8 character scaled
        for (int row = 0; row < 8; row++) {
            uint8_t row_bits = bitmap[row];
            for (int col = 0; col < 8; col++) {
                if (row_bits & (0x80 >> col)) {
                    // Draw scaled pixel
                    for (int sy = 0; sy < scale; sy++) {
                        for (int sx = 0; sx < scale; sx++) {
                            int px = cursor_x + col * scale + sx;
                            int py = y + row * scale + sy;
                            if (px >= 0 && px < (int)display->width &&
                                py >= 0 && py < (int)display->height) {
                                pixels[py * stride + px] = color;
                            }
                        }
                    }
                }
            }
        }

        cursor_x += 8 * scale;
        text++;
    }
}

int drm_text_width(const char* text, int scale) {
    if (!text || scale < 1) return 0;
    int len = 0;
    while (*text++) len++;
    return len * 8 * scale;
}

bool drm_is_stereo_mode(drm_display_t* display) {
    if (!display) return false;
    // Stereo SBS mode has double width (3840x1200 instead of 1920x1200)
    return display->width > 1920;
}

void drm_draw_image_stereo(drm_display_t* display, int x, int y, int width, int height,
                           const uint8_t* rgb_data, int disparity) {
    if (!display || !display->shadow_buffer || !rgb_data) return;

    if (!drm_is_stereo_mode(display)) {
        // Fallback to regular rendering if not in stereo mode
        drm_draw_image(display, x, y, width, height, rgb_data);
        return;
    }

    // In SBS mode: left eye = 0-1919, right eye = 1920-3839
    int half_width = display->width / 2;  // 1920

    // Left eye: shift left by disparity/2 (crossed disparity for closer objects)
    int left_x = x - disparity / 2;

    // Right eye: shift right by disparity/2, plus offset to right half
    int right_x = x + disparity / 2 + half_width;

    uint32_t* pixels = (uint32_t*)display->shadow_buffer;
    uint32_t stride = display->pitch / 4;

    // Draw to left eye region
    for (int row = 0; row < height; row++) {
        int dst_y = y + row;
        if (dst_y < 0 || dst_y >= (int)display->height) continue;

        for (int col = 0; col < width; col++) {
            int dst_x = left_x + col;
            // Clip to left eye region (0 to half_width-1)
            if (dst_x >= 0 && dst_x < half_width) {
                int src_idx = (row * width + col) * 3;
                uint8_t r = rgb_data[src_idx + 0];
                uint8_t g = rgb_data[src_idx + 1];
                uint8_t b = rgb_data[src_idx + 2];
                uint32_t color = (r << 16) | (g << 8) | b;
                pixels[dst_y * stride + dst_x] = color;
            }
        }
    }

    // Draw to right eye region
    for (int row = 0; row < height; row++) {
        int dst_y = y + row;
        if (dst_y < 0 || dst_y >= (int)display->height) continue;

        for (int col = 0; col < width; col++) {
            int dst_x = right_x + col;
            // Clip to right eye region (half_width to width-1)
            if (dst_x >= half_width && dst_x < (int)display->width) {
                int src_idx = (row * width + col) * 3;
                uint8_t r = rgb_data[src_idx + 0];
                uint8_t g = rgb_data[src_idx + 1];
                uint8_t b = rgb_data[src_idx + 2];
                uint32_t color = (r << 16) | (g << 8) | b;
                pixels[dst_y * stride + dst_x] = color;
            }
        }
    }
}

// Helper for stereo fill rect
static void drm_fill_rect_region(drm_display_t* display, int x, int y, int w, int h,
                                  uint8_t r, uint8_t g, uint8_t b,
                                  int clip_left, int clip_right) {
    if (!display || !display->shadow_buffer) return;

    uint32_t color = (r << 16) | (g << 8) | b;
    uint32_t* pixels = (uint32_t*)display->shadow_buffer;
    uint32_t stride = display->pitch / 4;

    // Clamp to region bounds
    if (x < clip_left) { w -= (clip_left - x); x = clip_left; }
    if (y < 0) { h += y; y = 0; }
    if (x + w > clip_right) w = clip_right - x;
    if (y + h > (int)display->height) h = display->height - y;
    if (w <= 0 || h <= 0) return;

    for (int row = y; row < y + h; row++) {
        for (int col = x; col < x + w; col++) {
            pixels[row * stride + col] = color;
        }
    }
}

void drm_draw_border_stereo(drm_display_t* display, int x, int y, int w, int h,
                            int thickness, uint8_t r, uint8_t g, uint8_t b, int disparity) {
    if (!display || !display->shadow_buffer || thickness <= 0) return;

    if (!drm_is_stereo_mode(display)) {
        drm_draw_border(display, x, y, w, h, thickness, r, g, b);
        return;
    }

    int half_width = display->width / 2;

    // Left eye position
    int left_x = x - disparity / 2;
    // Right eye position
    int right_x = x + disparity / 2 + half_width;

    // Draw borders for left eye (clipped to left region)
    // Top
    drm_fill_rect_region(display, left_x, y, w, thickness, r, g, b, 0, half_width);
    // Bottom
    drm_fill_rect_region(display, left_x, y + h - thickness, w, thickness, r, g, b, 0, half_width);
    // Left
    drm_fill_rect_region(display, left_x, y, thickness, h, r, g, b, 0, half_width);
    // Right
    drm_fill_rect_region(display, left_x + w - thickness, y, thickness, h, r, g, b, 0, half_width);

    // Draw borders for right eye (clipped to right region)
    // Top
    drm_fill_rect_region(display, right_x, y, w, thickness, r, g, b, half_width, display->width);
    // Bottom
    drm_fill_rect_region(display, right_x, y + h - thickness, w, thickness, r, g, b, half_width, display->width);
    // Left
    drm_fill_rect_region(display, right_x, y, thickness, h, r, g, b, half_width, display->width);
    // Right
    drm_fill_rect_region(display, right_x + w - thickness, y, thickness, h, r, g, b, half_width, display->width);
}

void drm_draw_text_stereo(drm_display_t* display, int x, int y, const char* text,
                          int scale, uint8_t r, uint8_t g, uint8_t b, int disparity) {
    if (!display || !display->shadow_buffer || !text || scale < 1) return;

    if (!drm_is_stereo_mode(display)) {
        drm_draw_text(display, x, y, text, scale, r, g, b);
        return;
    }

    int half_width = display->width / 2;
    uint32_t color = (r << 16) | (g << 8) | b;
    uint32_t* pixels = (uint32_t*)display->shadow_buffer;
    uint32_t stride = display->pitch / 4;

    // Left eye position
    int left_base_x = x - disparity / 2;
    // Right eye position
    int right_base_x = x + disparity / 2 + half_width;

    const char* ptr = text;
    int char_offset = 0;

    while (*ptr) {
        const uint8_t* bitmap = get_char_bitmap(*ptr);

        // Draw character to both eyes
        for (int row = 0; row < 8; row++) {
            uint8_t row_bits = bitmap[row];
            for (int col = 0; col < 8; col++) {
                if (row_bits & (0x80 >> col)) {
                    for (int sy = 0; sy < scale; sy++) {
                        for (int sx = 0; sx < scale; sx++) {
                            int py = y + row * scale + sy;
                            if (py < 0 || py >= (int)display->height) continue;

                            // Left eye
                            int left_px = left_base_x + char_offset + col * scale + sx;
                            if (left_px >= 0 && left_px < half_width) {
                                pixels[py * stride + left_px] = color;
                            }

                            // Right eye
                            int right_px = right_base_x + char_offset + col * scale + sx;
                            if (right_px >= half_width && right_px < (int)display->width) {
                                pixels[py * stride + right_px] = color;
                            }
                        }
                    }
                }
            }
        }

        char_offset += 8 * scale;
        ptr++;
    }
}
