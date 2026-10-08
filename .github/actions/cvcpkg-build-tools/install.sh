#!/usr/bin/env bash
# Install $TOOLS from cvcpkg into $RUNNER_TEMP/cvcpkg-tools and put exactly
# those executables first on PATH. Exports CVCPKG_TOOLS_PATH, the directory
# added to PATH. Kept bash-3.2-clean for macOS's /bin/bash.
set -euo pipefail

prefix="$RUNNER_TEMP/cvcpkg-tools"
read -r -a tools <<<"$TOOLS"
extra=""

case "$RUNNER_OS" in
  Linux)
    # Static: cvcpkg's shared cmake (3.31.7+cvc.6) has a builder-absolute
    # RUNPATH and cannot find its own libcurl outside the builder. The static
    # build needs nothing beyond the base system's libz, libstdc++ and glibc.
    link=static
    ;;
  macOS)
    # cmake is only published shared here. It loads @rpath/libcurl.4.dylib, but
    # its sole LC_RPATH is a path on the builder and the bundle does not
    # declare curl; libcurl in turn loads @rpath/libssl.3.dylib without
    # declaring openssl. Install both and point cmake at them below. Drop this
    # once cvcpkg republishes a relocatable cmake.
    link=shared
    extra="curl openssl"
    ;;
  Windows)
    # The shared builds; pkg-config has no static one. cmake.exe imports only
    # system DLLs either way.
    link=shared
    ;;
  *)
    echo "::error::unsupported runner OS: $RUNNER_OS"
    exit 1
    ;;
esac

rm -rf "$prefix"
# $extra is intentionally unquoted: zero or more component names.
# shellcheck disable=SC2086
cvcpkg install "${tools[@]}" $extra --prefix "$prefix" --config release --link "$link"

if [ "$RUNNER_OS" = macOS ]; then
  # An absolute rpath: these binaries live only in this job's tools prefix, and
  # they run through the symlinks below, which would leave @executable_path
  # ambiguous. Keyed on the rpath, not on whether cmake starts: dyld's fallback
  # search can quietly load the OS's /usr/lib/libcurl.4.dylib instead.
  for exe in cmake ctest cpack; do
    f="$prefix/bin/$exe"
    [ -f "$f" ] || continue
    rpaths=$(otool -l "$f" | awk '/cmd LC_RPATH/ { getline; getline; print $2 }')
    if ! grep -qxF "$prefix/lib" <<<"$rpaths"; then
      install_name_tool -add_rpath "$prefix/lib" "$f"
      codesign --force --sign - "$f"
    fi
  done
fi

if [ "$RUNNER_OS" = Windows ]; then
  # cmake.exe finds its modules relative to its own location, so it cannot be
  # linked from elsewhere. The tools prefix's bin/ goes on PATH whole, which
  # also exposes the curl.exe / openssl.exe cmake's bundle brought along.
  dir="$prefix/bin"
else
  # Only the requested tools, through symlinks: the prefix's bin/ also holds
  # perl, curl, openssl and nasm from cmake's dependency closure, which must
  # not shadow the runner's for the rest of the job. cmake resolves the
  # symlink to find its modules.
  dir="$prefix/shim"
  mkdir -p "$dir"
  for t in "${tools[@]}"; do
    if [ ! -x "$prefix/bin/$t" ]; then
      echo "::error::cvcpkg component '$t' installed no bin/$t"
      exit 1
    fi
    ln -sf "$prefix/bin/$t" "$dir/$t"
    if [ "$t" = cmake ]; then
      ln -sf "$prefix/bin/ctest" "$dir/ctest"
      ln -sf "$prefix/bin/cpack" "$dir/cpack"
    fi
  done
fi

echo "$dir" >> "$GITHUB_PATH"
echo "CVCPKG_TOOLS_PATH=$dir" >> "$GITHUB_ENV"
