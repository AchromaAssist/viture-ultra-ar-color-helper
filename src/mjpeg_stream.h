#ifndef MJPEG_STREAM_H
#define MJPEG_STREAM_H

#include <stdbool.h>
#include <stdint.h>

// Start MJPEG HTTP server on given port (spawns background thread)
bool mjpeg_stream_start(int port);

// Stop the server and join thread
void mjpeg_stream_stop(void);

// Push a new RGB frame to the stream (non-blocking, drops if busy)
// The frame is JPEG-encoded in the server thread.
void mjpeg_stream_push_frame(const uint8_t* rgb, int w, int h);

#endif // MJPEG_STREAM_H
