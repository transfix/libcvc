#!/usr/bin/env bash
# Check that a staged SDK's shared libcvc loads with nothing but SDK_DIR/lib and
# the base system.
#
#   sdk-check-closure.sh SDK_DIR
#
# Walks the dynamic dependencies of lib/libcvc.so / lib/libcvc.dylib, following
# every library it pulls in from lib/. Fails if a dependency is neither in lib/
# nor a base-system library, or if an object in the walk cannot find its lib/
# siblings at run time ($ORIGIN RUNPATH on Linux, an @loader_path LC_RPATH for
# @rpath loads on macOS).
#
# The walk covers only what libcvc reaches, not all of lib/. Whole bundles ship
# (see sdk-bundle-deps.sh), so lib/ also holds libraries libcvc never loads, and
# some of those have host deps of their own. cvcpkg's libboost_locale, for one,
# needs the ICU of the machine it was built on. A static SDK has no shared
# libcvc to start from; there the sdk-smoke job's consumer link and run check
# the same thing.
set -euo pipefail

if [ $# -ne 1 ]; then
  echo "usage: $0 SDK_DIR" >&2
  exit 2
fi
lib="$1/lib"

# Base-system libraries the SDK deliberately leaves to the host: glibc, the GCC
# runtimes (libstdc++, libgcc_s, libgomp, libquadmath) and the loader.
linux_system='^(libc|libm|libdl|libpthread|librt|libutil|libresolv|libstdc\+\+|libgcc_s|libgomp|libquadmath|ld-linux-x86-64|ld-linux-aarch64)\.so'

case "$(uname -s)" in
  Linux)  roots=$(find "$lib" -maxdepth 1 -type f -name 'libcvc*.so*') ;;
  Darwin) roots=$(find "$lib" -maxdepth 1 -type f -name 'libcvc*.dylib') ;;
  *) echo "unsupported host: $(uname -s)" >&2; exit 2 ;;
esac
if [ -z "$roots" ]; then
  echo "No shared libcvc under $lib (static SDK); the sdk-smoke job checks this flavor."
  exit 0
fi

errors=0
fail() { echo "::error::$*"; errors=$((errors + 1)); }

# Prints "NEEDED <name>" / "RUNPATH <path>" (Linux) or "LOAD <name>" /
# "RPATH <path>" (macOS) lines for one object.
deps_of() {
  if [ "$(uname -s)" = Linux ]; then
    readelf -d "$1" | sed -n \
      -e 's/.*(NEEDED).*\[\(.*\)\]/NEEDED \1/p' \
      -e 's/.*(RUNPATH).*\[\(.*\)\]/RUNPATH \1/p' \
      -e 's/.*(RPATH).*\[\(.*\)\]/RUNPATH \1/p'
  else
    local id
    id=$(otool -D "$1" | sed -n 2p)
    otool -L "$1" | sed -n 's/^[[:space:]]*\(.*\) (compatibility.*/\1/p' \
      | grep -vxF "$id" | sed 's/^/LOAD /'
    otool -l "$1" | awk '/cmd LC_RPATH/ { getline; getline; print "RPATH " $2 }'
  fi
}

seen=" "
queue=()
for r in $roots; do queue+=("$(basename "$r")"); done
checked=0
while [ ${#queue[@]} -gt 0 ]; do
  name=${queue[0]}
  queue=("${queue[@]:1}")
  case "$seen" in *" $name "*) continue ;; esac
  seen="$seen$name "
  checked=$((checked + 1))
  info=$(deps_of "$lib/$name")

  if [ "$(uname -s)" = Linux ]; then
    rp=$(sed -n 's/^RUNPATH //p' <<<"$info")
    case ":$rp:" in
      *':$ORIGIN:'*) ;;
      *) fail "$name: RUNPATH is '$rp', needs \$ORIGIN to find its lib/ siblings" ;;
    esac
    for dep in $(sed -n 's/^NEEDED //p' <<<"$info"); do
      if [ -e "$lib/$dep" ]; then
        queue+=("$dep")
      elif [[ ! $dep =~ $linux_system ]]; then
        fail "$name needs $dep, which is neither in the SDK nor a base-system library"
      fi
    done
  else
    rpaths=$(sed -n 's/^RPATH //p' <<<"$info")
    while IFS= read -r dep; do
      [ -n "$dep" ] || continue
      case "$dep" in
        /usr/lib/*|/System/*) ;;
        @rpath/*)
          if ! grep -qE '^@loader_path(/|/\.\./lib/?)?$' <<<"$rpaths"; then
            fail "$name loads $dep but has no @loader_path LC_RPATH"
          fi
          if [ -e "$lib/${dep#@rpath/}" ]; then queue+=("${dep#@rpath/}"); else fail "$name needs $dep, not in the SDK"; fi
          ;;
        @loader_path/*)
          if [ -e "$lib/${dep#@loader_path/}" ]; then queue+=("${dep#@loader_path/}"); else fail "$name needs $dep, not in the SDK"; fi
          ;;
        *) fail "$name loads $dep, outside the SDK and the base system" ;;
      esac
    done < <(sed -n 's/^LOAD //p' <<<"$info")
  fi
done

echo "Checked $checked libraries reachable from $(echo "$roots" | xargs -n1 basename | tr '\n' ' ')"
if [ "$errors" -gt 0 ]; then
  echo "::error::$errors unresolved dependencies in the SDK's libcvc closure"
  exit 1
fi
echo "All resolve inside $lib or the base system."
