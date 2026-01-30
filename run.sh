#!/bin/bash
# Run script for viture_ar_demo
# Sets up the library path for VITURE SDK dependencies

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${SCRIPT_DIR}/build"

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

VITURE_LIB_DIR="${SCRIPT_DIR}/external/XRLinuxDriver/lib/${ARCH}/viture"

if [ ! -d "$VITURE_LIB_DIR" ]; then
    echo "Error: VITURE library directory not found: $VITURE_LIB_DIR"
    exit 1
fi

if [ ! -f "$BUILD_DIR/viture_ar_demo" ]; then
    echo "Error: Executable not found. Please build first:"
    echo "  mkdir -p build && cd build && cmake .. && make"
    exit 1
fi

# Export library path and run
export LD_LIBRARY_PATH="${VITURE_LIB_DIR}:${LD_LIBRARY_PATH}"
exec "$BUILD_DIR/viture_ar_demo" "$@"
