#include "mjpeg_stream.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <errno.h>
#include <signal.h>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <jpeglib.h>

// Internal state
static int g_listen_fd = -1;
static pthread_t g_thread;
static volatile bool g_stream_running = false;

// Double-buffered RGB frame (caller writes, server thread reads)
static pthread_mutex_t g_frame_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t  g_frame_cond  = PTHREAD_COND_INITIALIZER;
static uint8_t* g_frame_rgb = NULL;
static int g_frame_w = 0;
static int g_frame_h = 0;
static bool g_frame_new = false;

// JPEG encode an RGB buffer into a malloc'd buffer. Returns size, or 0 on error.
static size_t jpeg_encode(const uint8_t* rgb, int w, int h, int quality,
                          uint8_t** out_buf) {
    struct jpeg_compress_struct cinfo;
    struct jpeg_error_mgr jerr;

    cinfo.err = jpeg_std_error(&jerr);
    jpeg_create_compress(&cinfo);

    unsigned char* mem_buf = NULL;
    unsigned long mem_size = 0;
    jpeg_mem_dest(&cinfo, &mem_buf, &mem_size);

    cinfo.image_width = w;
    cinfo.image_height = h;
    cinfo.input_components = 3;
    cinfo.in_color_space = JCS_RGB;
    jpeg_set_defaults(&cinfo);
    jpeg_set_quality(&cinfo, quality, TRUE);
    jpeg_start_compress(&cinfo, TRUE);

    while (cinfo.next_scanline < cinfo.image_height) {
        const uint8_t* row = rgb + cinfo.next_scanline * w * 3;
        jpeg_write_scanlines(&cinfo, (JSAMPARRAY)&row, 1);
    }

    jpeg_finish_compress(&cinfo);
    jpeg_destroy_compress(&cinfo);

    *out_buf = mem_buf;
    return (size_t)mem_size;
}

// Read and discard the HTTP request from client (we don't parse it)
static void consume_http_request(int fd) {
    char buf[2048];
    // Read until we see \r\n\r\n or error/timeout
    ssize_t total = 0;
    while (total < (ssize_t)sizeof(buf) - 1) {
        ssize_t n = read(fd, buf + total, sizeof(buf) - 1 - total);
        if (n <= 0) break;
        total += n;
        buf[total] = '\0';
        if (strstr(buf, "\r\n\r\n")) break;
    }
}

// Send MJPEG HTTP response headers
static bool send_mjpeg_headers(int fd) {
    const char* hdr =
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n"
        "Cache-Control: no-cache\r\n"
        "Connection: close\r\n"
        "\r\n";
    ssize_t len = strlen(hdr);
    return write(fd, hdr, len) == len;
}

// Send a single JPEG frame as an MJPEG boundary part
static bool send_frame(int fd, const uint8_t* jpeg, size_t jpeg_size) {
    char hdr[256];
    int hdr_len = snprintf(hdr, sizeof(hdr),
        "--frame\r\n"
        "Content-Type: image/jpeg\r\n"
        "Content-Length: %zu\r\n"
        "\r\n", jpeg_size);

    if (write(fd, hdr, hdr_len) != hdr_len) return false;

    // Send JPEG data in chunks
    size_t sent = 0;
    while (sent < jpeg_size) {
        size_t chunk = jpeg_size - sent;
        if (chunk > 65536) chunk = 65536;
        ssize_t n = write(fd, jpeg + sent, chunk);
        if (n <= 0) return false;
        sent += n;
    }

    // Trailing \r\n after data
    return write(fd, "\r\n", 2) == 2;
}

static void* server_thread(void* arg) {
    (void)arg;

    // Allocate local copy buffer for RGB frame
    uint8_t* local_rgb = NULL;
    int local_w = 0, local_h = 0;
    size_t local_alloc = 0;

    while (g_stream_running) {
        // Accept a client
        struct sockaddr_in client_addr;
        socklen_t addr_len = sizeof(client_addr);

        // Use a timeout on accept so we can check g_stream_running
        fd_set rfds;
        struct timeval tv;
        FD_ZERO(&rfds);
        FD_SET(g_listen_fd, &rfds);
        tv.tv_sec = 1;
        tv.tv_usec = 0;
        int sel = select(g_listen_fd + 1, &rfds, NULL, NULL, &tv);
        if (sel <= 0) continue;

        int client_fd = accept(g_listen_fd, (struct sockaddr*)&client_addr, &addr_len);
        if (client_fd < 0) continue;

        printf("[MJPEG] Client connected from %s\n", inet_ntoa(client_addr.sin_addr));

        consume_http_request(client_fd);

        if (!send_mjpeg_headers(client_fd)) {
            close(client_fd);
            continue;
        }

        // Stream frames to this client
        while (g_stream_running) {
            pthread_mutex_lock(&g_frame_mutex);
            // Wait for a new frame (with timeout to check g_stream_running)
            while (!g_frame_new && g_stream_running) {
                struct timespec ts;
                clock_gettime(CLOCK_REALTIME, &ts);
                ts.tv_sec += 1;
                pthread_cond_timedwait(&g_frame_cond, &g_frame_mutex, &ts);
            }
            if (!g_stream_running) {
                pthread_mutex_unlock(&g_frame_mutex);
                break;
            }

            // Copy frame data locally
            size_t needed = g_frame_w * g_frame_h * 3;
            if (needed > local_alloc) {
                free(local_rgb);
                local_rgb = malloc(needed);
                local_alloc = needed;
            }
            if (local_rgb) {
                memcpy(local_rgb, g_frame_rgb, needed);
                local_w = g_frame_w;
                local_h = g_frame_h;
            }
            g_frame_new = false;
            pthread_mutex_unlock(&g_frame_mutex);

            if (!local_rgb) continue;

            // JPEG encode
            uint8_t* jpeg_buf = NULL;
            size_t jpeg_size = jpeg_encode(local_rgb, local_w, local_h, 70, &jpeg_buf);
            if (jpeg_size == 0 || !jpeg_buf) continue;

            // Send to client
            bool ok = send_frame(client_fd, jpeg_buf, jpeg_size);
            free(jpeg_buf);

            if (!ok) {
                printf("[MJPEG] Client disconnected\n");
                break;
            }
        }

        close(client_fd);
    }

    free(local_rgb);
    return NULL;
}

bool mjpeg_stream_start(int port) {
    // Ignore SIGPIPE so writes to disconnected clients don't kill us
    signal(SIGPIPE, SIG_IGN);

    g_listen_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (g_listen_fd < 0) {
        perror("[MJPEG] socket");
        return false;
    }

    int opt = 1;
    setsockopt(g_listen_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in addr = {0};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(port);

    if (bind(g_listen_fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("[MJPEG] bind");
        close(g_listen_fd);
        g_listen_fd = -1;
        return false;
    }

    if (listen(g_listen_fd, 1) < 0) {
        perror("[MJPEG] listen");
        close(g_listen_fd);
        g_listen_fd = -1;
        return false;
    }

    g_stream_running = true;

    if (pthread_create(&g_thread, NULL, server_thread, NULL) != 0) {
        perror("[MJPEG] pthread_create");
        g_stream_running = false;
        close(g_listen_fd);
        g_listen_fd = -1;
        return false;
    }

    printf("[MJPEG] Streaming on http://0.0.0.0:%d\n", port);
    return true;
}

void mjpeg_stream_stop(void) {
    if (!g_stream_running) return;

    g_stream_running = false;

    // Wake up any waiting condvar
    pthread_cond_signal(&g_frame_cond);

    // Close listener to unblock accept
    if (g_listen_fd >= 0) {
        close(g_listen_fd);
        g_listen_fd = -1;
    }

    pthread_join(g_thread, NULL);

    // Free frame buffer
    free(g_frame_rgb);
    g_frame_rgb = NULL;
    g_frame_w = 0;
    g_frame_h = 0;
    g_frame_new = false;

    printf("[MJPEG] Server stopped\n");
}

void mjpeg_stream_push_frame(const uint8_t* rgb, int w, int h) {
    if (!g_stream_running || !rgb || w <= 0 || h <= 0) return;

    // Try to lock; if busy, skip this frame (non-blocking)
    if (pthread_mutex_trylock(&g_frame_mutex) != 0) return;

    // Allocate/reallocate frame buffer if needed
    size_t needed = (size_t)w * h * 3;
    if (g_frame_w != w || g_frame_h != h || !g_frame_rgb) {
        free(g_frame_rgb);
        g_frame_rgb = malloc(needed);
        if (!g_frame_rgb) {
            g_frame_w = 0;
            g_frame_h = 0;
            pthread_mutex_unlock(&g_frame_mutex);
            return;
        }
    }

    memcpy(g_frame_rgb, rgb, needed);
    g_frame_w = w;
    g_frame_h = h;
    g_frame_new = true;

    pthread_cond_signal(&g_frame_cond);
    pthread_mutex_unlock(&g_frame_mutex);
}
