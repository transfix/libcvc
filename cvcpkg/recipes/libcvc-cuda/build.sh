#!/usr/bin/env bash
# cvcpkg/recipes/libcvc-cuda/build.sh - build libcvc with CUDA, through the libcvc recipe's script.
#
# The GPU variant is the CPU SDK with CVC_ENABLE_CUDA=ON and nothing else changed, so this runs
# cvcpkg/recipes/libcvc/build.sh instead of keeping a copy of it. The copy it replaces had drifted:
# it built without gRPC, WebSocket or XMLRPC, and did not keep arisrv out of the install. The
# publish workflow's CUDA jobs run this script too, so `cvcpkg build libcvc-cuda` and the published
# bundle come from the same flags.
set -euo pipefail

: "${CVC_BUILD_DIR:?CVC_BUILD_DIR must be set}"

export CVC_ENABLE_CUDA=ON
bash "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/../libcvc/build.sh"

# CVC_ENABLE_CUDA=ON is a request, not a guarantee. When check_language(CUDA) finds no nvcc, the
# top-level CMakeLists prints a STATUS line and sets CVC_ENABLE_CUDA OFF as a plain variable, so the
# build succeeds as a CPU-only libcvc and the cache still reads CVC_ENABLE_CUDA:BOOL=ON. Packed, that
# would be a CPU build under the libcvc-cuda name, which declares requires_capabilities: [cuda], so
# the resolver would hand it to GPU users as the better variant. CMAKE_CUDA_COMPILER is set only when
# nvcc was found, so check that instead.
cache="$CVC_BUILD_DIR/CMakeCache.txt"
cuda_cc="$(sed -n 's/^CMAKE_CUDA_COMPILER:[^=]*=//p' "$cache" | head -1)"
case "${cuda_cc:-}" in
  '' | *NOTFOUND*)
    echo "libcvc-cuda: CUDA did not enable (CMAKE_CUDA_COMPILER='${cuda_cc:-<unset>}'): this is a CPU-only build" >&2
    exit 1
    ;;
esac
echo "libcvc-cuda: CUDA compiler $cuda_cc"
"$cuda_cc" --version

# Device code for several GPU generations. A cubin runs only on its own major compute capability and
# the bundles carry no PTX to JIT from, so a single architecture would not launch on most of the
# fleet. An early dry run packed an sm_52-only bundle, because the project's list was set after
# enable_language(CUDA) had already defined CMAKE_CUDA_ARCHITECTURES (fixed in CMakeLists.txt).
arches="$(sed -n 's/^CMAKE_CUDA_ARCHITECTURES:[^=]*=//p' "$cache" | head -1)"
echo "libcvc-cuda: CUDA architectures $arches"
case "$arches" in
  *\;*) ;;
  *)
    echo "libcvc-cuda: CMAKE_CUDA_ARCHITECTURES='$arches' is a single architecture" >&2
    exit 1
    ;;
esac
