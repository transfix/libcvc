#!/usr/bin/env bash
# recipes/libcvc/build.sh — build libcvc on Linux/macOS/BSD with CMake.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck source=recipes/_common/env-linux.sh
source "${SCRIPT_DIR}/../_common/env-${CVC_PLATFORM}.sh"

# Configure, build, and install with CMake. libcvc needs C++20 (the shared
# helper defaults to 17), and we ship the library only — tests and CUDA are
# off for the packaged build (CUDA needs a system toolkit, not a cvcpkg dep),
# while the CGAL mesher and SDF paths stay on.
cvc_cmake_build \
    -DCMAKE_CXX_STANDARD=20 \
    -DBUILD_SHARED_LIBS=ON \
    -DCVC_BUILD_TESTS=OFF \
    -DCVC_ENABLE_CUDA=OFF \
    -DDISABLE_CGAL=OFF \
    -DCVC_ENABLE_MESHER=ON \
    -DCVC_ENABLE_SDF=ON
