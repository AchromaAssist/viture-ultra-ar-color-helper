# Viture AR Glasses CLI Demo

A simple CLI application for interacting with Viture AR glasses using the XRLinuxDriver SDK.

## Features

- Detects and displays connected Viture devices with detailed information
- Handles device connection/disconnection events
- Streams IMU data (roll, pitch, yaw) with 1-second updates
- Supports all Viture models: One, One Lite, Pro, Luma, Luma Pro, Luma Ultra, Luma Cyber, Beast

## Requirements

- Linux (aarch64 or x86_64)
- CMake 3.16+
- libusb-1.0 development files
- GCC with C17 support

## Building

### 1. Set up remote repository (on target device)

```bash
# SSH into target device
ssh mini

# Create bare repository for pushing
mkdir -p ~/repos/viture_ar_demo.git
cd ~/repos/viture_ar_demo.git
git init --bare
```

### 2. Configure local repository and push

```bash
# Add remote
git remote add mini mini:~/repos/viture_ar_demo.git

# Create initial commit and push
git add -A
git commit -m "Initial commit: Viture AR CLI demo"
git push -u mini main
```

### 3. Clone and build on target device

```bash
# SSH into target device
ssh mini

# Clone the repository
git clone ~/repos/viture_ar_demo.git ~/viture_ar_demo
cd ~/viture_ar_demo

# Initialize submodules
git submodule update --init --recursive

# Install dependencies (Debian/Ubuntu)
sudo apt-get install -y cmake libusb-1.0-0-dev

# Build
mkdir build && cd build
cmake ..
make -j$(nproc)
```

### 4. Push updates

After making changes locally:
```bash
git add -A
git commit -m "Your commit message"
git push mini main
```

Then on the target device:
```bash
cd ~/viture_ar_demo
git pull
cd build
make -j$(nproc)
```

## Running

```bash
# Make sure you have permissions to access USB devices
# Either run as root or set up udev rules

# Run the demo
./viture_ar_demo
```

### USB Permissions (without root)

Create a udev rule for Viture devices:

```bash
sudo tee /etc/udev/rules.d/99-viture.rules << 'EOF'
# Viture AR Glasses
SUBSYSTEM=="usb", ATTR{idVendor}=="35ca", MODE="0666"
EOF

# Reload udev rules
sudo udevadm control --reload-rules
sudo udevadm trigger
```

## Output Example

```
===========================================
  Viture AR Glasses CLI Demo
===========================================

Scanning for Viture devices...

Found 1 Viture device(s):

  Device #1:
    Model:        VITURE Luma Ultra
    Vendor ID:    0x35ca
    Product ID:   0x1101
    Bus:          1
    Address:      5
    Manufacturer: VITURE
    Product:      VITURE Luma Ultra
    SDK Name:     Luma Ultra
    SDK Support:  Yes

-------------------------------------------
Connecting to first available device...
Device type: Carina (Luma Ultra/Cyber)
Firmware version: 1.2.3
Brightness level: 5
Connected successfully!

-------------------------------------------
Streaming IMU data (Ctrl+C to exit):
-------------------------------------------

  Roll:   +1.23   Pitch:  -0.45   Yaw:  +12.34
```

## Supported Devices

| Model | Product IDs |
|-------|-------------|
| One | 0x1011, 0x1013, 0x1017 |
| One Lite | 0x1015, 0x101b |
| Pro | 0x1019, 0x101d |
| Luma | 0x1131 |
| Luma Pro | 0x1121, 0x1141 |
| Luma Ultra | 0x1101, 0x1104 |
| Luma Cyber | 0x1151 |
| Beast | 0x1201 |

## Architecture

The application uses:
- **libusb** for USB device enumeration and detection
- **VITURE SDK** (via XRLinuxDriver) for device control and IMU data streaming

For Carina devices (Luma Ultra/Cyber), the SDK provides 6DOF tracking with position data.
For older devices (Gen1/Gen2), the SDK provides 3DOF orientation tracking.

## License

This project uses the XRLinuxDriver which is licensed under GPL-3.0.
