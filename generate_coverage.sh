#!/bin/bash
#
# Code coverage generation script for libcvc
# Automatically configures, builds, runs tests, and generates coverage reports
#

set -e  # Exit on error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Configuration
BUILD_DIR="${BUILD_DIR:-build-coverage}"
SOURCE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo -e "${BLUE}=== libcvc Code Coverage Generator ===${NC}\n"

# Check for required tools
check_tool() {
    if ! command -v "$1" &> /dev/null; then
        echo -e "${RED}Error: $1 is not installed${NC}"
        echo -e "Install with: sudo apt-get install $2"
        return 1
    fi
    return 0
}

echo "Checking for required tools..."
TOOLS_OK=true
check_tool "lcov" "lcov" || TOOLS_OK=false
check_tool "genhtml" "lcov" || TOOLS_OK=false
check_tool "gcov" "gcc" || TOOLS_OK=false

if [ "$TOOLS_OK" = false ]; then
    echo -e "${RED}Missing required tools. Please install them first.${NC}"
    exit 1
fi

echo -e "${GREEN}All required tools found${NC}\n"

# Step 1: Configure with coverage enabled
echo -e "${BLUE}Step 1: Configuring build with coverage enabled...${NC}"
cmake -B "$BUILD_DIR" -S "$SOURCE_DIR" \
    -DCMAKE_BUILD_TYPE=Debug \
    -DCVC_BUILD_TESTS=ON \
    -DCVC_ENABLE_COVERAGE=ON

# Step 2: Build
echo -e "\n${BLUE}Step 2: Building project...${NC}"
cmake --build "$BUILD_DIR" -j$(nproc)

# Step 3: Generate coverage
echo -e "\n${BLUE}Step 3: Generating coverage report...${NC}"
cd "$BUILD_DIR"
cmake --build . --target coverage

# Step 4: Display summary
echo -e "\n${GREEN}=== Coverage Report Generated ===${NC}"
echo -e "HTML Report: ${BLUE}${BUILD_DIR}/coverage_html/index.html${NC}"
echo -e "\nTo view the report:"
echo -e "  ${YELLOW}xdg-open ${BUILD_DIR}/coverage_html/index.html${NC}"
echo -e "  or"
echo -e "  ${YELLOW}cmake --build ${BUILD_DIR} --target coverage-view${NC}"

# Optional: Show coverage summary
if [ -f "coverage_filtered.info" ]; then
    echo -e "\n${BLUE}Coverage Summary:${NC}"
    lcov --summary coverage_filtered.info 2>&1 | grep -E "lines\.\.\.\.\.\.|functions\.\.\.\."
fi

echo -e "\n${GREEN}Done!${NC}"
