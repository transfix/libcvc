#!/usr/bin/env bash
# Install $TOOLS from cvcpkg and put exactly those executables first on PATH.
# Exports CVCPKG_TOOLS_PATH, the directory added to PATH. Kept bash-3.2-clean
# for macOS's /bin/bash.
#
# Two directories:
#   $RUNNER_TEMP/cvcpkg-tools-prefix  what `cvcpkg install` produced: the tools
#                                     plus their dependency closure (curl,
#                                     openssl, perl, ...) with headers, static
#                                     libs, pkg-config files and CMake configs.
#   $RUNNER_TEMP/cvcpkg-tools         bin/ with only the requested tools, and
#                                     cmake's own share/cmake-X.Y.
# The second one is what goes on PATH, and cmake is copied into it rather than
# linked: CMake adds its own install prefix (the parent of share/cmake-X.Y,
# found through the real executable path) to every find_package/find_library
# search. Run from the cvcpkg prefix, cmake let libcvc's find_package(CURL)
# pick up the tools' static libcurl.a and link it without OpenSSL.
set -euo pipefail

# POSIX paths for the shell; native() for cvcpkg and Python, which on Windows
# are native programs.
temp=$RUNNER_TEMP
if [ "$RUNNER_OS" = Windows ]; then temp=$(cygpath -u "$RUNNER_TEMP"); fi
native() { if [ "$RUNNER_OS" = Windows ]; then cygpath -m "$1"; else echo "$1"; fi; }
prefix="$temp/cvcpkg-tools-prefix"
tree="$temp/cvcpkg-tools"
read -r -a tools <<<"$TOOLS"
extra=""
exe=""

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
    exe=.exe
    ;;
  *)
    echo "::error::unsupported runner OS: $RUNNER_OS"
    exit 1
    ;;
esac

rm -rf "$prefix" "$tree"
# $extra is intentionally unquoted: zero or more component names.
# shellcheck disable=SC2086
cvcpkg install "${tools[@]}" $extra --prefix "$(native "$prefix")" --config release --link "$link"

if [ "$RUNNER_OS" = Windows ]; then
  python=$(command -v python || command -v python3)
  pe_imports=$(native "$GITHUB_ACTION_PATH/../../scripts/pe_imports.py")
fi
mkdir -p "$tree/bin" "$tree/share"

# Copy one executable from the cvcpkg prefix's bin/ into the tree. On Windows
# also copy the DLLs it imports from that bin/ (pkg-config.exe needs
# pkgconf-8.dll), transitively.
copy_tool() {
  local src="$prefix/bin/$1$exe"
  if [ ! -f "$src" ]; then
    echo "::error::cvcpkg installed no bin/$1$exe"
    exit 1
  fi
  cp -L "$src" "$tree/bin/"
  if [ "$RUNNER_OS" = Windows ]; then
    local todo="$1$exe" f dll
    while [ -n "$todo" ]; do
      f=${todo%% *}
      todo=${todo#"$f"}; todo=${todo# }
      for dll in $("$python" "$pe_imports" "$(native "$prefix/bin/$f")" | tr -d '\r'); do
        if [ -f "$prefix/bin/$dll" ] && [ ! -f "$tree/bin/$dll" ]; then
          cp "$prefix/bin/$dll" "$tree/bin/"
          todo="$todo $dll"
        fi
      done
    done
  fi
}

for t in "${tools[@]}"; do
  copy_tool "$t"
  if [ "$t" = cmake ]; then
    copy_tool ctest
    copy_tool cpack
    [ "$RUNNER_OS" != Windows ] || copy_tool cmcldeps
    cp -R "$prefix"/share/cmake-* "$tree/share/"
  fi
done

if [ "$RUNNER_OS" = macOS ]; then
  # cmake's dylibs (libcurl, libssl, libcrypto) go in a directory no find_*
  # call searches, and the copied binaries get an absolute rpath to it. Keyed
  # on the rpath, not on whether cmake starts: dyld's fallback search can
  # quietly load the OS's /usr/lib/libcurl.4.dylib instead.
  rt="$tree/libexec/cmake-runtime"
  mkdir -p "$rt"
  cp -R "$prefix"/lib/*.dylib "$rt/"
  for b in cmake ctest cpack; do
    f="$tree/bin/$b"
    [ -f "$f" ] || continue
    rpaths=$(otool -l "$f" | awk '/cmd LC_RPATH/ { getline; getline; print $2 }')
    if ! grep -qxF "$rt" <<<"$rpaths"; then
      install_name_tool -add_rpath "$rt" "$f"
      codesign --force --sign - "$f"
    fi
  done
fi

if [ "$RUNNER_OS" = Windows ]; then
  echo "$(cygpath -w "$tree/bin")" >> "$GITHUB_PATH"
else
  echo "$tree/bin" >> "$GITHUB_PATH"
fi
echo "CVCPKG_TOOLS_PATH=$tree/bin" >> "$GITHUB_ENV"
ls -l "$tree/bin"
