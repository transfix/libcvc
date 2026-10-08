#!/usr/bin/env bash
# Build and run a find_package(cvc CONFIG) consumer against an SDK archive, and
# fail if anything it used came from outside the archive.
#
#   sdk-smoke-test.sh ARCHIVE BUILD_TYPE
#
# ARCHIVE is a release.yml libcvc-*.tar.gz or libcvc-*.zip. Run it on a runner
# that has never seen a cvcpkg deps prefix: the archive is the only thing on
# CMAKE_PREFIX_PATH, and no LD_LIBRARY_PATH / DYLD_LIBRARY_PATH is set. On
# Windows, which has no rpath, PATH is cut down to the SDK's bin/ and the
# Windows directories. cmake and ninja must be cvcpkg's (the cvcpkg-build-tools
# action); the compiler is the runner's.
#
# Runner images ship plenty of libraries of their own (zlib, libpng, brotli
# under Homebrew, ...), and a consumer that silently picks one of those up
# proves nothing. So, beyond "it configured, built and ran", check that
#   1. every CMake package config the consumer loaded lives in the archive,
#   2. every library on its link line is in the archive or the toolchain
#      (compiler, platform SDK, CUDA),
#   3. every shared library the loader mapped is in the archive or the base
#      system.
set -euo pipefail

if [ $# -ne 2 ]; then
  echo "usage: $0 ARCHIVE BUILD_TYPE" >&2
  exit 2
fi
case "$(uname -s)" in
  Linux) os=linux ;;
  Darwin) os=macos ;;
  MINGW*|MSYS*|CYGWIN*) os=windows ;;
  *) echo "unsupported host: $(uname -s)" >&2; exit 2 ;;
esac
archive=$(cd "$(dirname "$1")" && pwd)/$(basename "$1")
build_type=$2
temp=${RUNNER_TEMP:-${TMPDIR:-/tmp}}
[ $os != windows ] || temp=$(cygpath -u "$temp")
work="$temp/cvc-sdk-smoke"
rm -rf "$work"
mkdir -p "$work/sdk" "$work/src"

case "$archive" in
  *.tar.gz) tar xzf "$archive" -C "$work/sdk" ;;
  *.zip)
    if [ $os = macos ]; then
      ditto -x -k "$archive" "$work/sdk"   # keeps the dylib symlinks
    else
      python=$(command -v python || command -v python3)
      "$python" -m zipfile -e "$(cygpath -m "$archive" 2>/dev/null || echo "$archive")" \
        "$(cygpath -m "$work/sdk" 2>/dev/null || echo "$work/sdk")"
    fi
    ;;
  *) echo "unknown archive type: $archive" >&2; exit 2 ;;
esac
# Linux/macOS archives have one top-level directory; the Windows zip is flat.
top=$(find "$work/sdk" -mindepth 1 -maxdepth 1)
if [ "$(echo "$top" | wc -l)" -eq 1 ] && [ -d "$top" ]; then sdk=$top; else sdk="$work/sdk"; fi
echo "SDK: $sdk"

cat > "$work/src/main.cpp" <<'CPP'
#include <cvc/core/app.h>
#include <cvc/volume/volume.h>
#include <cvc/volume/volume_file_io.h>
#ifdef _WIN32
#define NOMINMAX
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <psapi.h>
#include <cstdio>
#endif
int main() {
  cvc::app app;                      // registers the volume file handlers
  cvc::dimension dim(4, 4, 4);
  cvc::bounding_box box(0, 0, 0, 1, 1, 1);
  std::vector<float> buf(4 * 4 * 4, 1.0f);
  cvc::volume v(app, reinterpret_cast<const unsigned char *>(buf.data()), dim, cvc::Float, box);
  v.write("smoke.rawiv");
  cvc::volume r(app);
  r.read("smoke.rawiv");
#ifdef _WIN32
  // Windows has no ldd and no DYLD_PRINT_LIBRARIES: list the loaded modules.
  HMODULE mods[1024];
  DWORD bytes = 0;
  if (K32EnumProcessModules(GetCurrentProcess(), mods, sizeof(mods), &bytes)) {
    for (DWORD i = 0; i < bytes / sizeof(HMODULE); ++i) {
      char path[MAX_PATH];
      if (GetModuleFileNameA(mods[i], path, MAX_PATH)) std::printf("module: %s\n", path);
    }
  }
#endif
  return (r.XDim() == 4) ? 0 : 1;
}
CPP
cat > "$work/src/CMakeLists.txt" <<'CMK'
cmake_minimum_required(VERSION 3.16)
project(cvc_sdk_smoke LANGUAGES C CXX)
set(CMAKE_CXX_STANDARD 20)
find_package(cvc CONFIG REQUIRED)
add_executable(smoke main.cpp)
target_link_libraries(smoke PRIVATE cvc::cvc)
CMK

# Ninja where there is no rpath question about the generator's own tools; the
# Visual Studio generator on Windows, which needs no developer shell.
if [ $os = windows ]; then
  cmake -S "$work/src" -B "$work/build" -DCMAKE_PREFIX_PATH="$(cygpath -m "$sdk")"
  cmake --build "$work/build" --config "$build_type"
  exe="$work/build/$build_type/smoke.exe"
else
  cmake -S "$work/src" -B "$work/build" -G Ninja \
    -DCMAKE_PREFIX_PATH="$sdk" -DCMAKE_BUILD_TYPE="$build_type"
  cmake --build "$work/build"
  exe="$work/build/smoke"
fi

errors=0
fail() { echo "::error::$*"; errors=$((errors + 1)); }
# Compare in one spelling: forward slashes, lower case on Windows.
norm() {
  if [ $os = windows ]; then
    cygpath -m "$1" 2>/dev/null | tr '[:upper:]' '[:lower:]' || echo "$1"
  else
    echo "$1"
  fi
}
sdk_n=$(norm "$sdk")
inside_sdk() { case "$(norm "$1")" in "$sdk_n"/*) return 0 ;; esac; return 1; }

# Toolchain paths a link line may use: the compiler, the platform SDK and, for
# the CUDA-enabled Linux/Windows SDKs, the CUDA toolkit. No library directory of
# the host (/usr/lib, /usr/local/lib, /opt/homebrew, ...): a dependency found
# there is one the archive is missing.
case $os in
  linux)   link_ok='^(/usr/bin/|/usr/lib/gcc/|/usr/local/cuda[^/]*/)' ;;
  macos)   link_ok='^(/usr/bin/|/library/developer/|/applications/xcode[^/]*\.app/)' ;;
  windows) link_ok='^(c:/program files( \(x86\))?/(microsoft visual studio|windows kits|nvidia gpu computing toolkit)/)' ;;
esac
# Linux base-system libraries by name: glibc, the GCC runtimes and the loader.
# They may appear on the link line by absolute path too (FindOpenMP and
# FindCUDAToolkit name libpthread / librt that way), and they are all the loader
# may map from outside the archive.
linux_base='^(libc|libm|libdl|libpthread|librt|libutil|libresolv|libstdc\+\+|libgcc_s|libgomp|libquadmath|ld-linux-x86-64|ld-linux-aarch64|linux-vdso)\.(so|a)'

echo "== 1. package configs"
while IFS= read -r line; do
  var=${line%%:PATH=*}
  dir=${line#*:PATH=}
  [ -d "$dir" ] || continue
  ls "$dir"/*Config.cmake "$dir"/*-config.cmake >/dev/null 2>&1 || continue
  if inside_sdk "$dir"; then
    echo "ok   $var -> $dir"
  else
    fail "$var resolved outside the SDK: $dir"
  fi
done < <(tr -d '\r' < "$work/build/CMakeCache.txt" | grep -E '^[A-Za-z0-9_.+-]+_DIR:PATH=([A-Za-z]:)?/')

echo "== 2. link line"
if [ $os = windows ]; then
  # The VS generator keeps the link inputs in the project file.
  toks=$(tr -d '\r' < "$work/build/smoke.vcxproj" \
    | sed -n -e 's|.*<AdditionalDependencies>\(.*\)</AdditionalDependencies>.*|\1|p' \
             -e 's|.*<AdditionalLibraryDirectories>\(.*\)</AdditionalLibraryDirectories>.*|\1|p' \
    | tr ';' '\n' | tr '\\' '/' | grep -E '^[A-Za-z]:/' || true)
else
  # The last command ninja lists for smoke is its link.
  link_cmd=$(ninja -C "$work/build" -t commands smoke | tail -n 1)
  echo "$link_cmd"
  toks=$(echo "$link_cmd" | tr -s ' ' '\n' \
    | sed -e 's/^-Wl,-rpath-link,//' -e 's/^-Wl,-rpath,//' -e 's/^-L//' | tr ':' '\n' | grep '^/' || true)
fi
while IFS= read -r tok; do
  [ -n "$tok" ] || continue
  if inside_sdk "$tok"; then
    echo "ok   $tok"
  elif [[ $(norm "$tok" | tr '[:upper:]' '[:lower:]') =~ $link_ok ]]; then
    echo "tool $tok"
  elif [ $os = linux ] && [[ $(basename "$tok") =~ $linux_base ]]; then
    echo "os   $tok"
  else
    fail "link line uses $tok, outside the SDK"
  fi
done <<<"$toks"

echo "== 3. run"
unset LD_LIBRARY_PATH DYLD_LIBRARY_PATH DYLD_FALLBACK_LIBRARY_PATH
if [ $os = windows ]; then
  win=$(cygpath -u "${SYSTEMROOT:-C:\\Windows}")
  out=$(cd "$work/build" && PATH="$sdk/bin:$win/System32:$win" "$exe")
else
  # No loader path: RUNPATH/@rpath alone has to find libcvc and every dependency.
  out=$(cd "$work/build" && "$exe")
fi
echo "smoke ran OK"

echo "== 4. loaded libraries"
case $os in
  linux)
    loaded=$(cd "$work/build" && ldd "$exe")
    if grep -q 'not found' <<<"$loaded"; then
      fail "unresolved at load time: $(grep 'not found' <<<"$loaded" | tr -s ' \t' ' ')"
    fi
    paths=$(sed -n 's/.*=> \(\/[^ ]*\) (0x.*/\1/p' <<<"$loaded")
    ;;
  macos)
    paths=$(cd "$work/build" && DYLD_PRINT_LIBRARIES=1 "$exe" 2>&1 >/dev/null \
      | sed -n 's/^dyld\[[0-9]*\]: <[^>]*> //p; s/^dyld\[[0-9]*\]: //p' | grep '^/' | grep -v '/smoke$' || true)
    ;;
  windows)
    paths=$(tr -d '\r' <<<"$out" | sed -n 's/^module: //p' | tr '\\' '/' | grep -vi '/smoke\.exe$' || true)
    ;;
esac
[ -n "$paths" ] || fail "could not list the libraries the consumer loaded"
while IFS= read -r p; do
  [ -n "$p" ] || continue
  if inside_sdk "$p"; then
    echo "ok   $p"
    continue
  fi
  case $os in
    linux)   [[ $(basename "$p") =~ $linux_base ]] ;;
    macos)   [[ $p == /usr/lib/* || $p == /System/* ]] ;;
    windows) [[ $(norm "$p") == "$(norm "$win")"/* ]] ;;
  esac && { echo "os   $p"; continue; }
  fail "loaded $p, outside the SDK and the base system"
done <<<"$paths"

if [ "$errors" -gt 0 ]; then
  echo "::error::$errors self-containment violations for $(basename "$archive")"
  exit 1
fi
echo "$(basename "$archive"): self-contained"
