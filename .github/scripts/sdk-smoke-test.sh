#!/usr/bin/env bash
# Build and run a find_package(cvc CONFIG) consumer against an SDK archive, and
# fail if anything it used came from outside the archive.
#
#   sdk-smoke-test.sh ARCHIVE BUILD_TYPE
#
# ARCHIVE is a release.yml libcvc-*.tar.gz or libcvc-*.zip. Run it on a runner
# that has never seen a cvcpkg deps prefix: the archive is the only thing on
# CMAKE_PREFIX_PATH, and no LD_LIBRARY_PATH / DYLD_LIBRARY_PATH is set. cmake
# and ninja must be cvcpkg's (the cvcpkg-build-tools action); the compiler is
# the runner's.
#
# Runner images ship plenty of libraries of their own (zlib, libpng, ...), and
# a consumer that silently picks one of those up proves nothing. So, beyond
# "it configured, built and ran", check that
#   1. every CMake package config the consumer loaded lives in the archive,
#   2. every library on its link line is in the archive or the system/toolchain,
#   3. every shared library the loader mapped is in the archive or the base
#      system.
set -euo pipefail

if [ $# -ne 2 ]; then
  echo "usage: $0 ARCHIVE BUILD_TYPE" >&2
  exit 2
fi
archive=$(cd "$(dirname "$1")" && pwd)/$(basename "$1")
build_type=$2
work="${RUNNER_TEMP:-${TMPDIR:-/tmp}}/cvc-sdk-smoke"
rm -rf "$work"
mkdir -p "$work/sdk" "$work/src"

case "$archive" in
  *.tar.gz) tar xzf "$archive" -C "$work/sdk" ;;
  *.zip)    ditto -x -k "$archive" "$work/sdk" 2>/dev/null || unzip -q "$archive" -d "$work/sdk" ;;
  *) echo "unknown archive type: $archive" >&2; exit 2 ;;
esac
sdk=$(find "$work/sdk" -mindepth 1 -maxdepth 1 -type d)
[ "$(echo "$sdk" | wc -l)" -eq 1 ] || { echo "expected one top-level directory in $archive" >&2; exit 1; }
echo "SDK: $sdk"

cat > "$work/src/main.cpp" <<'CPP'
#include <cvc/core/app.h>
#include <cvc/volume/volume.h>
#include <cvc/volume/volume_file_io.h>
int main() {
  cvc::app app;                      // registers the volume file handlers
  cvc::dimension dim(4, 4, 4);
  cvc::bounding_box box(0, 0, 0, 1, 1, 1);
  std::vector<float> buf(4 * 4 * 4, 1.0f);
  cvc::volume v(app, reinterpret_cast<const unsigned char *>(buf.data()), dim, cvc::Float, box);
  v.write("smoke.rawiv");
  cvc::volume r(app);
  r.read("smoke.rawiv");
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

cmake -S "$work/src" -B "$work/build" -G Ninja \
  -DCMAKE_PREFIX_PATH="$sdk" \
  -DCMAKE_BUILD_TYPE="$build_type"
cmake --build "$work/build"

errors=0
fail() { echo "::error::$*"; errors=$((errors + 1)); }
inside_sdk() { case "$1" in "$sdk"/*) return 0 ;; esac; return 1; }

# Paths the consumer may legitimately take from outside the archive: the base
# system, the compiler and its SDK sysroot, and (Linux) the CUDA toolkit, which
# the Linux SDK's headers include. Anything else, e.g. /usr/local/lib or
# /opt/homebrew, means the archive is missing something the runner happened to
# provide.
case "$(uname -s)" in
  Linux)  host_ok='^(/usr/bin/|/usr/lib/|/lib/|/lib64/|/usr/lib64/|/usr/local/cuda[^/]*/)' ;;
  Darwin) host_ok='^(/usr/bin/|/usr/lib/|/System/|/Library/Developer/|/Applications/Xcode[^/]*\.app/)' ;;
esac

echo "== 1. package configs"
while IFS= read -r line; do
  var=${line%%:PATH=*}
  dir=${line#*:PATH=}
  [ -d "$dir" ] || continue
  ls "$dir"/*Config.cmake "$dir"/*-config.cmake >/dev/null 2>&1 || continue
  if inside_sdk "$dir"; then
    echo "ok   $var -> ${dir#"$sdk"/}"
  else
    fail "$var resolved outside the SDK: $dir"
  fi
done < <(grep -E '^[A-Za-z0-9_.+-]+_DIR:PATH=/' "$work/build/CMakeCache.txt")

echo "== 2. link line"
# The last command ninja lists for smoke is its link. Check every absolute path
# in it: the compiler driver, library paths, -L dirs and rpaths.
link_cmd=$(ninja -C "$work/build" -t commands smoke | tail -n 1)
echo "$link_cmd"
for tok in $(echo "$link_cmd" | tr -s ' ' '\n' \
               | sed -e 's/^-Wl,-rpath-link,//' -e 's/^-Wl,-rpath,//' -e 's/^-L//' | tr ':' '\n'); do
  case "$tok" in /*) ;; *) continue ;; esac
  if inside_sdk "$tok"; then
    echo "ok   ${tok#"$sdk"/}"
  elif [[ $tok =~ $host_ok ]]; then
    echo "host $tok"
  else
    fail "link line uses $tok, outside the SDK"
  fi
done

echo "== 3. run"
# No loader path: RUNPATH/@rpath alone has to find libcvc and every dependency.
unset LD_LIBRARY_PATH DYLD_LIBRARY_PATH DYLD_FALLBACK_LIBRARY_PATH
( cd "$work/build" && ./smoke )
echo "smoke ran OK"

echo "== 4. loaded libraries"
if [ "$(uname -s)" = Linux ]; then
  loaded=$(cd "$work/build" && ldd ./smoke)
  if grep -q 'not found' <<<"$loaded"; then
    fail "unresolved at load time: $(grep 'not found' <<<"$loaded" | tr -s ' \t' ' ')"
  fi
  paths=$(sed -n 's/.*=> \(\/[^ ]*\) (0x.*/\1/p' <<<"$loaded")
else
  paths=$(cd "$work/build" && DYLD_PRINT_LIBRARIES=1 ./smoke 2>&1 >/dev/null \
            | sed -n 's/^dyld\[[0-9]*\]: <[^>]*> //p; s/^dyld\[[0-9]*\]: //p' | grep '^/' | grep -v '/smoke$' || true)
  [ -n "$paths" ] || fail "DYLD_PRINT_LIBRARIES printed nothing; cannot check what was loaded"
fi
for p in $paths; do
  if inside_sdk "$p"; then
    echo "ok   ${p#"$sdk"/}"
  elif [[ $p =~ $host_ok ]]; then
    echo "host $p"
  else
    fail "loaded $p, outside the SDK"
  fi
done

if [ "$errors" -gt 0 ]; then
  echo "::error::$errors self-containment violations for $(basename "$archive")"
  exit 1
fi
echo "$(basename "$archive"): self-contained"
