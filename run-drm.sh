#!/bin/bash
# Run viture_ar_demo with DRM direct mode from SSH
# This script switches to a text VT, runs the app, then switches back
#
# Usage: sudo ./run-drm.sh
#
# The desktop will temporarily blank while the app runs on the glasses.
# Press Ctrl+C to exit and restore the desktop.

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${SCRIPT_DIR}/build"
VITURE_LIB_DIR="${SCRIPT_DIR}/external/XRLinuxDriver/lib"

# Detect architecture
ARCH=$(uname -m)
if [ "$ARCH" = "aarch64" ] || [ "$ARCH" = "arm64" ]; then
    ARCH="aarch64"
elif [ "$ARCH" = "x86_64" ]; then
    ARCH="x86_64"
else
    echo "Unsupported architecture: $ARCH"
    exit 1
fi

LIB_PATH="${VITURE_LIB_DIR}/${ARCH}/viture"

if [ ! -d "$LIB_PATH" ]; then
    echo "Error: VITURE library directory not found: $LIB_PATH"
    exit 1
fi

if [ ! -f "$BUILD_DIR/viture_ar_demo" ]; then
    echo "Error: Executable not found. Please build first:"
    echo "  mkdir -p build && cd build && cmake .. && make"
    exit 1
fi

# Check if running as root
if [ "$EUID" -ne 0 ]; then
    echo "This script must be run as root (sudo)"
    exit 1
fi

# Get the current VT
CURRENT_VT=$(cat /sys/class/tty/tty0/active 2>/dev/null | sed 's/tty//' || echo "")
if [ -z "$CURRENT_VT" ]; then
    # Try fgconsole
    CURRENT_VT=$(fgconsole 2>/dev/null || echo "2")
fi

echo "Current VT: $CURRENT_VT"

# Find a free text VT (usually 3-6 are free)
TARGET_VT=3

echo "============================================"
echo "  Viture AR Demo - DRM Direct Mode"
echo "============================================"
echo ""
echo "This will:"
echo "  1. Switch to VT${TARGET_VT} (text console)"
echo "  2. Run the demo with DRM access"
echo "  3. Switch back to VT${CURRENT_VT} when done"
echo ""
echo "Your desktop will temporarily blank but is NOT closed."
echo "Press Ctrl+C to exit the demo."
echo ""
echo "Starting in 3 seconds..."
sleep 3

# Function to restore VT on exit
cleanup() {
    echo ""
    echo "Restoring VT${CURRENT_VT}..."
    chvt "$CURRENT_VT" 2>/dev/null || true
    echo "Done."
}
trap cleanup EXIT

# Switch to text VT
echo "Switching to VT${TARGET_VT}..."
chvt "$TARGET_VT"

# Small delay to let VT switch complete
sleep 1

# Run the application
echo "Running viture_ar_demo..."
export LD_LIBRARY_PATH="${LIB_PATH}:${LD_LIBRARY_PATH}"
"$BUILD_DIR/viture_ar_demo" "$@"

# cleanup trap will restore VT
