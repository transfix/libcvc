#!/bin/bash
# Build verification script for trans-cvc

set -e  # Exit on error

echo "=========================================="
echo "Trans-CVC Build Verification Script"
echo "=========================================="
echo ""

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Check CMake version
echo "Checking CMake version..."
CMAKE_VERSION=$(cmake --version | head -n1 | cut -d' ' -f3)
CMAKE_MAJOR=$(echo $CMAKE_VERSION | cut -d'.' -f1)
CMAKE_MINOR=$(echo $CMAKE_VERSION | cut -d'.' -f2)

if [ "$CMAKE_MAJOR" -lt 3 ] || ([ "$CMAKE_MAJOR" -eq 3 ] && [ "$CMAKE_MINOR" -lt 15 ]); then
    echo -e "${RED}ERROR: CMake version 3.15 or higher required. Found: $CMAKE_VERSION${NC}"
    exit 1
fi
echo -e "${GREEN}✓ CMake version: $CMAKE_VERSION${NC}"

# Check for required dependencies
echo ""
echo "Checking for required dependencies..."

# Check for Boost
if pkg-config --exists boost 2>/dev/null || [ -n "$BOOST_ROOT" ]; then
    echo -e "${GREEN}✓ Boost found${NC}"
else
    echo -e "${YELLOW}⚠ Boost might not be found automatically${NC}"
fi

# Check for optional dependencies
echo ""
echo "Checking for optional dependencies..."

# HDF5
if pkg-config --exists hdf5 2>/dev/null; then
    echo -e "${GREEN}✓ HDF5 found${NC}"
    HDF5_OPTION="-DCVC_USING_HDF5=ON"
else
    echo -e "${YELLOW}⚠ HDF5 not found (optional)${NC}"
    HDF5_OPTION="-DCVC_USING_HDF5=OFF"
fi

# CGAL
if pkg-config --exists CGAL 2>/dev/null; then
    echo -e "${GREEN}✓ CGAL found${NC}"
    CGAL_OPTION="-DDISABLE_CGAL=OFF"
else
    echo -e "${YELLOW}⚠ CGAL not found (optional)${NC}"
    CGAL_OPTION="-DDISABLE_CGAL=ON"
fi

# FFTW
if pkg-config --exists fftw3 2>/dev/null; then
    echo -e "${GREEN}✓ FFTW found${NC}"
else
    echo -e "${YELLOW}⚠ FFTW not found (optional)${NC}"
fi

# GSL
if pkg-config --exists gsl 2>/dev/null; then
    echo -e "${GREEN}✓ GSL found${NC}"
else
    echo -e "${YELLOW}⚠ GSL not found (optional)${NC}"
fi

# Create build directory
echo ""
echo "Setting up build directory..."
BUILD_DIR="build"
if [ -d "$BUILD_DIR" ]; then
    echo "Removing existing build directory..."
    rm -rf "$BUILD_DIR"
fi
mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

# Configure
echo ""
echo "Configuring with CMake..."
cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CXX_STANDARD=17 \
    -DBUILD_SHARED_LIBS=ON \
    $HDF5_OPTION \
    $CGAL_OPTION \
    -DCVC_ENABLE_MESHER=ON \
    -DCVC_ENABLE_SDF=ON \
    -DCVC_USING_IMOD_MRC=ON \
    -DCVC_GEOMETRY_ENABLE_BUNNY=ON

if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ Configuration successful${NC}"
else
    echo -e "${RED}✗ Configuration failed${NC}"
    exit 1
fi

# Build
echo ""
echo "Building (this may take a while)..."
NPROC=$(nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 4)
echo "Using $NPROC parallel jobs..."

cmake --build . -j$NPROC

if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ Build successful${NC}"
else
    echo -e "${RED}✗ Build failed${NC}"
    exit 1
fi

# Check outputs
echo ""
echo "Verifying build outputs..."

if [ -f "bin/trans-cvc" ]; then
    echo -e "${GREEN}✓ Executable: bin/trans-cvc${NC}"
else
    echo -e "${RED}✗ Executable not found${NC}"
fi

if [ -f "lib/libcvc.so" ] || [ -f "lib/libcvc.dylib" ] || [ -f "lib/libcvc.dll" ]; then
    echo -e "${GREEN}✓ Library: libcvc${NC}"
else
    echo -e "${RED}✗ Library not found${NC}"
fi

echo ""
echo "=========================================="
echo -e "${GREEN}Build verification complete!${NC}"
echo "=========================================="
echo ""
echo "To install, run:"
echo "  cd $BUILD_DIR && sudo cmake --install ."
echo ""
echo "To run the executable:"
echo "  ./$BUILD_DIR/bin/trans-cvc"
echo ""
