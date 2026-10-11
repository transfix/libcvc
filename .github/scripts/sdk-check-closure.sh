#!/usr/bin/env bash
# Check that a staged SDK's shared libcvc loads with nothing but the SDK and
# the base system.
#
#   sdk-check-closure.sh SDK_DIR
#
# Walks the dynamic dependencies of the shared libcvc (lib/libcvc.so,
# lib/libcvc.dylib, bin/cvc.dll), following every library it pulls in from the
# SDK. Fails if a dependency is neither in the SDK nor part of the base system,
# or if an object in the walk cannot find its SDK siblings at run time ($ORIGIN
# RUNPATH on Linux, an @loader_path LC_RPATH for @rpath loads on macOS; Windows
# finds them because they sit beside cvc.dll in bin/).
#
# The walk covers only what libcvc reaches, not the whole SDK. Whole bundles
# ship (see sdk-bundle-deps.sh), so the SDK also holds libraries libcvc never
# loads, which this walk does not vouch for. (cvcpkg's boost before
# 1.86.0+cvc.5, for one, had a libboost_locale that needed the build machine's
# ICU.) A static SDK has no shared libcvc to start from; there the sdk-smoke
# job's consumer link and run check the same thing.
set -euo pipefail

if [ $# -ne 1 ]; then
  echo "usage: $0 SDK_DIR" >&2
  exit 2
fi
sdk=$1

case "$(uname -s)" in
  Linux)  os=linux;  dir="$sdk/lib"; roots=$(find "$dir" -maxdepth 1 -type f -name 'libcvc*.so*') ;;
  Darwin) os=macos;  dir="$sdk/lib"; roots=$(find "$dir" -maxdepth 1 -type f -name 'libcvc*.dylib') ;;
  MINGW*|MSYS*|CYGWIN*)
    os=windows
    sdk=$(cygpath -u "$sdk")
    dir="$sdk/bin"
    roots=$(find "$dir" -maxdepth 1 -type f -iname 'cvc*.dll')
    python=$(command -v python || command -v python3)
    system32="$(cygpath -u "${SYSTEMROOT:-C:\\Windows}")/System32"
    ;;
  *) echo "unsupported host: $(uname -s)" >&2; exit 2 ;;
esac
if [ -z "$roots" ]; then
  echo "No shared libcvc under $dir (static SDK); the sdk-smoke job checks this flavor."
  exit 0
fi

# Base-system libraries the SDK deliberately leaves to the host: glibc, the GCC
# runtimes (libstdc++, libgcc_s, libgomp, libquadmath) and the loader.
linux_system='^(libc|libm|libdl|libpthread|librt|libutil|libresolv|libstdc\+\+|libgcc_s|libgomp|libquadmath|ld-linux-x86-64|ld-linux-aarch64)\.so'
# The MSVC runtime is not part of Windows. The SDK ships it in bin/ (CMake's
# InstallRequiredSystemLibraries, plus libomp140 from the top-level
# CMakeLists.txt), so a copy that only exists in a build machine's System32 does
# not count.
msvc_runtime='^(vcruntime140|msvcp140|vcomp140|concrt140|vccorlib140|libomp140)'
# TEMPORARY exception, the one dependency the Windows SDK knowingly lacks:
# MSVC builds of cvc use -openmp:llvm, whose runtime (libomp140.<arch>.dll)
# Microsoft does not allow redistributing. Reported as a warning until cvc links
# cvcpkg's LLVM libomp instead, which needs a Windows build of cvcpkg's openmp
# recipe. Remove this then.
interim_windows='^libomp140\.'

errors=0
fail() { echo "::error::$*"; errors=$((errors + 1)); }

# Prints "NEEDED <name>" / "RUNPATH <path>" (Linux), "LOAD <name>" /
# "RPATH <path>" (macOS) or "NEEDED <dll>" (Windows) lines for one object.
deps_of() {
  case $os in
    linux)
      readelf -d "$1" | sed -n \
        -e 's/.*(NEEDED).*\[\(.*\)\]/NEEDED \1/p' \
        -e 's/.*(RUNPATH).*\[\(.*\)\]/RUNPATH \1/p' \
        -e 's/.*(RPATH).*\[\(.*\)\]/RUNPATH \1/p'
      ;;
    macos)
      local id
      id=$(otool -D "$1" | sed -n 2p)
      otool -L "$1" | sed -n 's/^[[:space:]]*\(.*\) (compatibility.*/\1/p' \
        | grep -vxF "$id" | sed 's/^/LOAD /'
      otool -l "$1" | awk '/cmd LC_RPATH/ { getline; getline; print "RPATH " $2 }'
      ;;
    windows)
      "$python" "$(dirname "$0")/pe_imports.py" "$1" | tr -d '\r' | sed 's/^/NEEDED /'
      ;;
  esac
}

seen=" "
queue=()
for r in $roots; do queue+=("$(basename "$r")"); done
checked=0
while [ ${#queue[@]} -gt 0 ]; do
  name=${queue[0]}
  queue=("${queue[@]:1}")
  key=$(echo "$name" | tr '[:upper:]' '[:lower:]')
  case "$seen" in *" $key "*) continue ;; esac
  seen="$seen$key "
  checked=$((checked + 1))
  depth=dep
  for r in $roots; do [ "$(basename "$r")" = "$name" ] && depth=root; done
  info=$(deps_of "$dir/$name")

  case $os in
    linux)
      rp=$(sed -n 's/^RUNPATH //p' <<<"$info")
      case ":$rp:" in
        *':$ORIGIN:'*) ;;
        *) fail "$name: RUNPATH is '$rp', needs \$ORIGIN to find its lib/ siblings" ;;
      esac
      for dep in $(sed -n 's/^NEEDED //p' <<<"$info"); do
        if [ -e "$dir/$dep" ]; then
          queue+=("$dep")
        elif [[ ! $dep =~ $linux_system ]]; then
          fail "$name needs $dep, which is neither in the SDK nor a base-system library"
        fi
      done
      ;;
    macos)
      rpaths=$(sed -n 's/^RPATH //p' <<<"$info")
      while IFS= read -r dep; do
        [ -n "$dep" ] || continue
        case "$dep" in
          /usr/lib/*|/System/*) ;;
          @rpath/*)
            # dyld expands @rpath with the LC_RPATHs of every image in the
            # load chain, so libtiff's @rpath loads resolve through libcvc's
            # @loader_path even when libtiff has no rpath of its own. Only the
            # entry point, libcvc itself, has to carry one.
            if [ "$depth" = root ] && ! grep -qE '^@loader_path(/|/\.\./lib/?)?$' <<<"$rpaths"; then
              fail "$name loads $dep but has no @loader_path LC_RPATH"
            fi
            if [ -e "$dir/${dep#@rpath/}" ]; then queue+=("${dep#@rpath/}"); else fail "$name needs $dep, not in the SDK"; fi
            ;;
          @loader_path/*)
            if [ -e "$dir/${dep#@loader_path/}" ]; then queue+=("${dep#@loader_path/}"); else fail "$name needs $dep, not in the SDK"; fi
            ;;
          *) fail "$name loads $dep, outside the SDK and the base system" ;;
        esac
      done < <(sed -n 's/^LOAD //p' <<<"$info")
      ;;
    windows)
      # DLL names are case-insensitive: an import of MSVCP140.dll is satisfied
      # by bin/msvcp140.dll.
      for dep in $(sed -n 's/^NEEDED //p' <<<"$info"); do
        lc=$(echo "$dep" | tr '[:upper:]' '[:lower:]')
        hit=$(ls "$dir" | grep -ixF "$dep" | head -n 1 || true)
        if [ -n "$hit" ]; then
          queue+=("$hit")
        elif [[ $lc =~ $interim_windows ]]; then
          echo "::warning::$name needs $dep, MSVC's LLVM OpenMP runtime, which is not redistributable; until cvc links cvcpkg's libomp, the SDK cannot ship it and consumers need Visual Studio"
        elif [[ $lc =~ $msvc_runtime ]]; then
          fail "$name needs the MSVC runtime's $dep, which bin/ does not ship"
        elif [[ $lc == api-ms-win-* || $lc == ext-ms-* ]] || [ -e "$system32/$dep" ]; then
          :
        else
          fail "$name needs $dep, which is neither in the SDK nor part of Windows"
        fi
      done
      ;;
  esac
done

echo "Checked $checked libraries reachable from $(echo "$roots" | xargs -n1 basename | tr '\n' ' ')"
if [ "$errors" -gt 0 ]; then
  echo "::error::$errors unresolved dependencies in the SDK's libcvc closure"
  exit 1
fi
echo "All resolve inside $dir or the base system."
