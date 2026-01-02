#!/bin/bash
# Build script for VolRover3 application

set -e  # Exit on error

echo "================================"
echo "VolRover3 Build Script"
echo "================================"
echo ""

# Check for Qt6
echo "Checking for Qt6..."
if pkg-config --exists Qt6Core Qt6Widgets Qt6OpenGL Qt6OpenGLWidgets 2>/dev/null; then
    echo "✓ Qt6 found"
    pkg-config --modversion Qt6Core
else
    echo "✗ Qt6 not found"
    echo "  Install with: sudo apt-get install qt6-base-dev qt6-opengl-dev"
    echo "  Or on macOS: brew install qt@6"
fi
echo ""

# Check for VTK
echo "Checking for VTK..."
if pkg-config --exists vtk 2>/dev/null; then
    echo "✓ VTK found"
    pkg-config --modversion vtk
elif [ -d "/usr/local/lib/cmake/vtk-9.0" ] || [ -d "/usr/local/lib/cmake/vtk-9.1" ] || [ -d "/usr/local/lib/cmake/vtk-9.2" ]; then
    echo "✓ VTK found (CMake installation)"
else
    echo "✗ VTK not found"
    echo "  Install with: sudo apt-get install libvtk9-dev"
    echo "  Or on macOS: brew install vtk"
fi
echo ""

# Build
echo "Building trans-cvc with VolRover3..."
echo ""

cd "$(dirname "$0")"

if [ ! -d "build" ]; then
    mkdir build
fi

cd build

cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DCVC_BUILD_VOLROVER3=ON \
    -DCVC_BUILD_TESTS=OFF

echo ""
echo "Building..."
make volrover3 -j$(nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 4)

echo ""
echo "================================"
echo "Build complete!"
echo "================================"
echo ""
echo "Run with: ./build/bin/volrover3"
echo ""
