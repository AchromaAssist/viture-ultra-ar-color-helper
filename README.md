# Viture Ultra AR Glasses Color Helper App (Proof of Concept)

A CLI application for Viture AR glasses that demonstrates real-time color detection, stereo depth estimation, and 3D overlay rendering.

![Demo](output.gif)

**Demo:** The app detects dominant colors in real time — in this example, distinguishing green (unripe) from yellow (mature) bananas, which can be difficult for color blind people. After a head nod gesture, it captures a frame, identifies the color, and displays the result as a 3D label on the glasses. The video above shows both the RGB camera feed and the overlay image rendered on the glasses display.

## Features

- **Color detection** — dominant color recognition with configurable threshold
- **Stereo depth estimation** — real-time depth from dual cameras with periodic updates
- **IMU tracking** — roll, pitch, yaw from device IMU (Carina device used)
- **Nod detection** — head nod gesture triggers capture (two-phase: down + return)
- **Stereo 3D disparity** — labels rendered at depth-corrected position with manual +/- adjustment
- **Precomputed rectification maps** — stereo rectification maps stored locally for fast startup
- **DRM direct rendering** — side-by-side stereo output to glasses display at 1920x1200
- **MJPEG streaming** — live camera feed with overlay via HTTP for monitoring
- **SLAM correction** — quaternion-based head tracking stabilizes label position

## Supported Devices

| Model | Product IDs |
|-------|-------------|
| [Luma Ultra](https://www.viture.com/product/viture-luma-ultra-xr-glasses) | 0x1101, 0x1104 |

Luma Ultra is the only Viture model with a built-in RGB camera and stereo IR cameras.

## Requirements

- Linux (aarch64 or x86_64)
- CMake 3.16+
- GCC with C17 support
- libusb-1.0 development files
- libdrm development files
- libjpeg development files

## Building

```bash
# Install dependencies (Debian/Ubuntu)
sudo apt-get install -y cmake libusb-1.0-0-dev libdrm-dev libjpeg-dev

# Initialize submodules
git submodule update --init --recursive

# Build
mkdir build && cd build
cmake ..
make -j$(nproc)
```

## Running

```bash
# Recommended (DRM direct rendering with tuned parameters)
sudo bash ./run-drm.sh -- --crop 200 --display 300x300 --offset 0,-350 --opacity 25 --dominance 50

# Or using the basic run script
./run.sh

# Or manually
cd build
LD_LIBRARY_PATH="../external/XRLinuxDriver/lib/aarch64/viture:$LD_LIBRARY_PATH" ./viture_ar_demo
```

DRM direct rendering requires master access. The app tries multiple approaches:
1. Set `non-desktop` property on the Viture connector
2. Request a DRM lease
3. Acquire DRM master (requires no active display manager on the VT)

### USB Permissions (without root)

```bash
sudo tee /etc/udev/rules.d/99-viture.rules << 'EOF'
# Viture AR Glasses
SUBSYSTEM=="usb", ATTR{idVendor}=="35ca", MODE="0666"
EOF
sudo udevadm control --reload-rules
sudo udevadm trigger
```

### Command-line Options

Key parameters:

| Option | Description |
|--------|-------------|
| `--crop SIZE` | Crop region size from camera center |
| `--display WxH` | Display/scene buffer dimensions |
| `--offset X,Y` | Pixel offset from display center |
| `--opacity PCT` | Image opacity 0-100% |
| `--dominance PCT` | Color detection threshold |
| `--roi-y OFFSET` | Vertical offset for crop ROI |
| `--stereo-3d` | Enable stereo 3D rendering |
| `--rectify` | Enable stereo rectification |
| `--stream-port PORT` | MJPEG stream port (default: 8080) |

### Keyboard Controls

| Key | Action |
|-----|--------|
| `Space` | Force capture |
| `+`/`-` | Adjust stereo disparity |
| `d` | Debug depth detection |
| `q` | Quit |

## Architecture

- **libusb** — USB device enumeration and detection
- **VITURE SDK** (XRLinuxDriver) — device control, IMU streaming, stereo cameras
- **libdrm** — direct rendering to glasses display
- **libjpeg** — MJPEG stream encoding

## License

This project is licensed under GPL-3.0.

XRLinuxDriver submodule is also licensed under GPL-3.0.
