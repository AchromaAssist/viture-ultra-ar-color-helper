/**
 * V4L2 Camera Capture Implementation
 */

#include "v4l2_capture.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <jpeglib.h>
#include <setjmp.h>
#include <unistd.h>
#include <errno.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>

// Helper for ioctl with retry on EINTR
static int xioctl(int fd, int request, void* arg) {
    int r;
    do {
        r = ioctl(fd, request, arg);
    } while (r == -1 && errno == EINTR);
    return r;
}

// Convert YUYV to RGB24
static void yuyv_to_rgb(const uint8_t* yuyv, uint8_t* rgb, int width, int height) {
    int yuyv_stride = width * 2;
    int rgb_stride = width * 3;

    for (int y = 0; y < height; y++) {
        const uint8_t* yuyv_line = yuyv + y * yuyv_stride;
        uint8_t* rgb_line = rgb + y * rgb_stride;

        for (int x = 0; x < width; x += 2) {
            int y0 = yuyv_line[0];
            int u  = yuyv_line[1];
            int y1 = yuyv_line[2];
            int v  = yuyv_line[3];

            // Convert to RGB (BT.601)
            int c0 = y0 - 16;
            int c1 = y1 - 16;
            int d = u - 128;
            int e = v - 128;

            int r0 = (298 * c0 + 409 * e + 128) >> 8;
            int g0 = (298 * c0 - 100 * d - 208 * e + 128) >> 8;
            int b0 = (298 * c0 + 516 * d + 128) >> 8;

            int r1 = (298 * c1 + 409 * e + 128) >> 8;
            int g1 = (298 * c1 - 100 * d - 208 * e + 128) >> 8;
            int b1 = (298 * c1 + 516 * d + 128) >> 8;

            // Clamp values
            #define CLAMP(x) ((x) < 0 ? 0 : ((x) > 255 ? 255 : (x)))
            rgb_line[0] = CLAMP(r0);
            rgb_line[1] = CLAMP(g0);
            rgb_line[2] = CLAMP(b0);
            rgb_line[3] = CLAMP(r1);
            rgb_line[4] = CLAMP(g1);
            rgb_line[5] = CLAMP(b1);
            #undef CLAMP

            yuyv_line += 4;
            rgb_line += 6;
        }
    }
}

// Error handler for libjpeg that doesn't call exit()
struct jpeg_error_ctx {
    struct jpeg_error_mgr pub;
    jmp_buf setjmp_buf;
};

static void jpeg_error_exit(j_common_ptr cinfo) {
    struct jpeg_error_ctx* ctx = (struct jpeg_error_ctx*)cinfo->err;
    longjmp(ctx->setjmp_buf, 1);
}

// Convert MJPEG to RGB24 using libjpeg
static void mjpeg_to_rgb(const uint8_t* mjpeg, size_t size, uint8_t* rgb, int width, int height) {
    struct jpeg_decompress_struct cinfo;
    struct jpeg_error_ctx jerr;

    cinfo.err = jpeg_std_error(&jerr.pub);
    jerr.pub.error_exit = jpeg_error_exit;

    if (setjmp(jerr.setjmp_buf)) {
        jpeg_destroy_decompress(&cinfo);
        memset(rgb, 0, width * height * 3);
        return;
    }

    jpeg_create_decompress(&cinfo);
    jpeg_mem_src(&cinfo, mjpeg, size);
    jpeg_read_header(&cinfo, TRUE);

    cinfo.out_color_space = JCS_RGB;
    jpeg_start_decompress(&cinfo);

    int row_stride = cinfo.output_width * cinfo.output_components;
    while (cinfo.output_scanline < cinfo.output_height) {
        int y = cinfo.output_scanline;
        if (y < height) {
            uint8_t* row = rgb + y * width * 3;
            jpeg_read_scanlines(&cinfo, &row, 1);
        } else {
            uint8_t* dummy = rgb;  // Read into start (will be overwritten or already done)
            jpeg_read_scanlines(&cinfo, &dummy, 1);
        }
    }
    (void)row_stride;

    jpeg_finish_decompress(&cinfo);
    jpeg_destroy_decompress(&cinfo);
}

bool camera_init(camera_t* camera, const char* device, uint32_t req_width, uint32_t req_height) {
    memset(camera, 0, sizeof(*camera));
    camera->fd = -1;

    printf("[Camera] Opening %s...\n", device);

    // Open device
    camera->fd = open(device, O_RDWR | O_NONBLOCK);
    if (camera->fd < 0) {
        printf("[Camera] Failed to open %s: %s\n", device, strerror(errno));
        return false;
    }

    // Query capabilities
    struct v4l2_capability cap;
    if (xioctl(camera->fd, VIDIOC_QUERYCAP, &cap) < 0) {
        printf("[Camera] Failed to query capabilities: %s\n", strerror(errno));
        close(camera->fd);
        camera->fd = -1;
        return false;
    }

    printf("[Camera] Device: %s\n", cap.card);
    printf("[Camera] Driver: %s\n", cap.driver);

    if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
        printf("[Camera] Device does not support video capture\n");
        close(camera->fd);
        camera->fd = -1;
        return false;
    }

    if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
        printf("[Camera] Device does not support streaming\n");
        close(camera->fd);
        camera->fd = -1;
        return false;
    }

    // Try to set format (prefer YUYV, fallback to MJPEG)
    struct v4l2_format fmt = {0};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    // First, get current format
    if (xioctl(camera->fd, VIDIOC_G_FMT, &fmt) < 0) {
        printf("[Camera] Failed to get format: %s\n", strerror(errno));
    }

    printf("[Camera] Current format: %ux%u, fourcc=%c%c%c%c\n",
           fmt.fmt.pix.width, fmt.fmt.pix.height,
           (fmt.fmt.pix.pixelformat >> 0) & 0xFF,
           (fmt.fmt.pix.pixelformat >> 8) & 0xFF,
           (fmt.fmt.pix.pixelformat >> 16) & 0xFF,
           (fmt.fmt.pix.pixelformat >> 24) & 0xFF);

    // Set requested format - prefer MJPEG for better quality at high resolution
    fmt.fmt.pix.width = req_width > 0 ? req_width : 640;
    fmt.fmt.pix.height = req_height > 0 ? req_height : 480;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
    fmt.fmt.pix.field = V4L2_FIELD_NONE;

    if (xioctl(camera->fd, VIDIOC_S_FMT, &fmt) < 0) {
        printf("[Camera] MJPEG not supported, trying YUYV...\n");
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
        if (xioctl(camera->fd, VIDIOC_S_FMT, &fmt) < 0) {
            printf("[Camera] Failed to set format: %s\n", strerror(errno));
            close(camera->fd);
            camera->fd = -1;
            return false;
        }
    }

    camera->width = fmt.fmt.pix.width;
    camera->height = fmt.fmt.pix.height;
    camera->format = fmt.fmt.pix.pixelformat;
    camera->bytesperline = fmt.fmt.pix.bytesperline;
    camera->sizeimage = fmt.fmt.pix.sizeimage;

    printf("[Camera] Set format: %ux%u, fourcc=%c%c%c%c, size=%u\n",
           camera->width, camera->height,
           (camera->format >> 0) & 0xFF,
           (camera->format >> 8) & 0xFF,
           (camera->format >> 16) & 0xFF,
           (camera->format >> 24) & 0xFF,
           camera->sizeimage);

    // Request buffers
    struct v4l2_requestbuffers req = {0};
    req.count = CAMERA_BUFFER_COUNT;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    if (xioctl(camera->fd, VIDIOC_REQBUFS, &req) < 0) {
        printf("[Camera] Failed to request buffers: %s\n", strerror(errno));
        close(camera->fd);
        camera->fd = -1;
        return false;
    }

    if (req.count < 2) {
        printf("[Camera] Insufficient buffer memory\n");
        close(camera->fd);
        camera->fd = -1;
        return false;
    }

    camera->buffer_count = req.count;
    printf("[Camera] Allocated %d buffers\n", camera->buffer_count);

    // Map buffers
    for (int i = 0; i < camera->buffer_count; i++) {
        struct v4l2_buffer buf = {0};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;

        if (xioctl(camera->fd, VIDIOC_QUERYBUF, &buf) < 0) {
            printf("[Camera] Failed to query buffer %d: %s\n", i, strerror(errno));
            camera_cleanup(camera);
            return false;
        }

        camera->buffers[i].length = buf.length;
        camera->buffers[i].start = mmap(NULL, buf.length,
                                         PROT_READ | PROT_WRITE,
                                         MAP_SHARED,
                                         camera->fd, buf.m.offset);

        if (camera->buffers[i].start == MAP_FAILED) {
            printf("[Camera] Failed to mmap buffer %d: %s\n", i, strerror(errno));
            camera->buffers[i].start = NULL;
            camera_cleanup(camera);
            return false;
        }
    }

    // Optimize image quality via V4L2 controls
    struct v4l2_control ctrl;
    ctrl.id = V4L2_CID_SHARPNESS;
    ctrl.value = 6;  // Max sharpness
    if (xioctl(camera->fd, VIDIOC_S_CTRL, &ctrl) == 0) {
        printf("[Camera] Sharpness set to %d\n", ctrl.value);
    }
    ctrl.id = V4L2_CID_CONTRAST;
    ctrl.value = 40;  // Slightly above default (32)
    if (xioctl(camera->fd, VIDIOC_S_CTRL, &ctrl) == 0) {
        printf("[Camera] Contrast set to %d\n", ctrl.value);
    }

    // Allocate RGB frame buffer
    camera->rgb_size = camera->width * camera->height * 3;
    camera->rgb_frame = malloc(camera->rgb_size);
    if (!camera->rgb_frame) {
        printf("[Camera] Failed to allocate RGB buffer\n");
        camera_cleanup(camera);
        return false;
    }

    printf("[Camera] Initialized successfully\n");
    return true;
}

bool camera_start(camera_t* camera) {
    if (camera->fd < 0 || camera->streaming) {
        return false;
    }

    printf("[Camera] Starting streaming...\n");

    // Queue all buffers
    for (int i = 0; i < camera->buffer_count; i++) {
        struct v4l2_buffer buf = {0};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;

        if (xioctl(camera->fd, VIDIOC_QBUF, &buf) < 0) {
            printf("[Camera] Failed to queue buffer %d: %s\n", i, strerror(errno));
            return false;
        }
    }

    // Start streaming
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl(camera->fd, VIDIOC_STREAMON, &type) < 0) {
        printf("[Camera] Failed to start streaming: %s\n", strerror(errno));
        return false;
    }

    camera->streaming = true;
    printf("[Camera] Streaming started\n");
    return true;
}

void camera_stop(camera_t* camera) {
    if (camera->fd < 0 || !camera->streaming) {
        return;
    }

    printf("[Camera] Stopping streaming...\n");

    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    xioctl(camera->fd, VIDIOC_STREAMOFF, &type);

    camera->streaming = false;
    printf("[Camera] Streaming stopped\n");
}

uint8_t* camera_capture_frame(camera_t* camera) {
    if (camera->fd < 0 || !camera->streaming) {
        return NULL;
    }

    // Wait for frame using select
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(camera->fd, &fds);

    struct timeval tv;
    tv.tv_sec = 2;
    tv.tv_usec = 0;

    int r = select(camera->fd + 1, &fds, NULL, NULL, &tv);
    if (r < 0) {
        if (errno == EINTR) {
            return NULL;
        }
        printf("[Camera] Select error: %s\n", strerror(errno));
        return NULL;
    }
    if (r == 0) {
        printf("[Camera] Select timeout\n");
        return NULL;
    }

    // Dequeue buffer
    struct v4l2_buffer buf = {0};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;

    if (xioctl(camera->fd, VIDIOC_DQBUF, &buf) < 0) {
        if (errno == EAGAIN) {
            return NULL;
        }
        printf("[Camera] Failed to dequeue buffer: %s\n", strerror(errno));
        return NULL;
    }

    // Convert to RGB
    uint8_t* frame_data = camera->buffers[buf.index].start;
    if (camera->format == V4L2_PIX_FMT_YUYV) {
        yuyv_to_rgb(frame_data, camera->rgb_frame, camera->width, camera->height);
    } else if (camera->format == V4L2_PIX_FMT_MJPEG) {
        mjpeg_to_rgb(frame_data, buf.bytesused, camera->rgb_frame, camera->width, camera->height);
    }

    // Re-queue buffer
    if (xioctl(camera->fd, VIDIOC_QBUF, &buf) < 0) {
        printf("[Camera] Failed to re-queue buffer: %s\n", strerror(errno));
    }

    return camera->rgb_frame;
}

uint8_t* camera_capture_frame_fresh(camera_t* camera) {
    if (camera->fd < 0 || !camera->streaming) {
        return NULL;
    }

    // Drain all queued buffers to get the freshest frame
    struct v4l2_buffer buf = {0};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;

    int last_index = -1;
    int frames_drained = 0;

    // Dequeue all available buffers (non-blocking)
    while (1) {
        buf.index = 0;  // Reset for each attempt
        if (xioctl(camera->fd, VIDIOC_DQBUF, &buf) < 0) {
            if (errno == EAGAIN) {
                // No more frames available
                break;
            }
            // Real error
            if (frames_drained == 0) {
                // No frames at all, wait for one
                fd_set fds;
                FD_ZERO(&fds);
                FD_SET(camera->fd, &fds);

                struct timeval tv;
                tv.tv_sec = 2;
                tv.tv_usec = 0;

                int r = select(camera->fd + 1, &fds, NULL, NULL, &tv);
                if (r <= 0) {
                    return NULL;
                }
                continue;  // Try again after select
            }
            break;
        }

        // Re-queue old buffer if we have a new one
        if (last_index >= 0) {
            struct v4l2_buffer qbuf = {0};
            qbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            qbuf.memory = V4L2_MEMORY_MMAP;
            qbuf.index = last_index;
            xioctl(camera->fd, VIDIOC_QBUF, &qbuf);
        }

        last_index = buf.index;
        frames_drained++;
    }

    // If no frames were captured, try regular capture
    if (last_index < 0) {
        return camera_capture_frame(camera);
    }

    // Convert the freshest frame to RGB
    uint8_t* frame_data = camera->buffers[last_index].start;
    if (camera->format == V4L2_PIX_FMT_YUYV) {
        yuyv_to_rgb(frame_data, camera->rgb_frame, camera->width, camera->height);
    } else if (camera->format == V4L2_PIX_FMT_MJPEG) {
        mjpeg_to_rgb(frame_data, buf.bytesused, camera->rgb_frame, camera->width, camera->height);
    }

    // Re-queue the last buffer
    struct v4l2_buffer qbuf = {0};
    qbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    qbuf.memory = V4L2_MEMORY_MMAP;
    qbuf.index = last_index;
    xioctl(camera->fd, VIDIOC_QBUF, &qbuf);

    return camera->rgb_frame;
}

void camera_cleanup(camera_t* camera) {
    if (camera->streaming) {
        camera_stop(camera);
    }

    // Free RGB buffer
    if (camera->rgb_frame) {
        free(camera->rgb_frame);
        camera->rgb_frame = NULL;
    }

    // Unmap buffers
    for (int i = 0; i < camera->buffer_count; i++) {
        if (camera->buffers[i].start && camera->buffers[i].start != MAP_FAILED) {
            munmap(camera->buffers[i].start, camera->buffers[i].length);
            camera->buffers[i].start = NULL;
        }
    }

    // Close device
    if (camera->fd >= 0) {
        close(camera->fd);
        camera->fd = -1;
    }

    printf("[Camera] Cleanup complete\n");
}

void camera_get_pixel(camera_t* camera, int x, int y, uint8_t* r, uint8_t* g, uint8_t* b) {
    if (!camera->rgb_frame || x < 0 || y < 0 ||
        x >= (int)camera->width || y >= (int)camera->height) {
        *r = *g = *b = 0;
        return;
    }

    int offset = (y * camera->width + x) * 3;
    *r = camera->rgb_frame[offset + 0];
    *g = camera->rgb_frame[offset + 1];
    *b = camera->rgb_frame[offset + 2];
}
