#!/usr/bin/env bash
# Install $TOOLS from cvcpkg and put exactly those executables first on PATH.
# Exports CVCPKG_TOOLS_PATH, the directory added to PATH. Kept bash-3.2-clean
# for macOS's /bin/bash.
#
# Two directories:
#   $RUNNER_TEMP/cvcpkg-tools-prefix  what `cvcpkg install` produced: the tools
#                                     plus their dependency closure (curl,
#                                     openssl, ...) with headers, static libs,
#                                     pkg-config files and CMake configs.
#   $RUNNER_TEMP/cvcpkg-tools         bin/ with only the requested tools,
#                                     cmake's own share/cmake-X.Y, and lib/
#                                     with the shared libraries they load.
# The second one is what goes on PATH, and cmake is copied into it rather than
# linked: CMake adds its own install prefix (the parent of share/cmake-X.Y,
# found through the real executable path) to every find_package/find_library
# search. Run from the cvcpkg prefix, cmake let libcvc's find_package(CURL)
# pick up the tools' static libcurl.a and link it without OpenSSL.
#
# cvcpkg's tools are relocatable (cmake >= 3.31.7+cvc.8: RUNPATH $ORIGIN/../lib
# on Linux, LC_RPATH @executable_path/../lib on macOS, curl and openssl declared
# as runtime deps), so the copies find their libraries in the tree's lib/. Only
# the versioned runtime names go there (libcurl.so.4.8.0, libssl.3.dylib, ...):
# find_library() looks for libcurl.so / libcurl.dylib / libcurl.a, so the tree
# still offers nothing to libcvc's find_* calls.
set -euo pipefail

# POSIX paths for the shell; native() for cvcpkg and Python, which on Windows
# are native programs.
temp=$RUNNER_TEMP
if [ "$RUNNER_OS" = Windows ]; then temp=$(cygpath -u "$RUNNER_TEMP"); fi
native() { if [ "$RUNNER_OS" = Windows ]; then cygpath -m "$1"; else echo "$1"; fi; }
# The @rpath/<leaf> load commands of a Mach-O file, as leaf names.
rpath_leaves() { otool -L "$1" | awk 'NR > 1 && $1 ~ /^@rpath\// { sub(/^@rpath\//, "", $1); print $1 }'; }
prefix="$temp/cvcpkg-tools-prefix"
tree="$temp/cvcpkg-tools"
read -r -a tools <<<"$TOOLS"
exe=""

case "$RUNNER_OS" in
  Linux | macOS)
    link=shared
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
cvcpkg install "${tools[@]}" --prefix "$(native "$prefix")" --config release --link "$link"

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

if [ "$RUNNER_OS" != Windows ]; then
  # The runtime libraries the tools load through their relative rpath: the
  # versioned names only (see the top of this file).
  mkdir -p "$tree/lib"
  if [ "$RUNNER_OS" = macOS ]; then pattern='lib*.*.dylib'; else pattern='lib*.so.*'; fi
  find "$prefix/lib" -maxdepth 1 -name "$pattern" \( -type f -o -type l \) -exec cp -P {} "$tree/lib/" \;

  # What a tool loads from the tree must come from the tree, not the host.
  #   Linux: every library ldd resolves that the tree ships.
  #   macOS: every @rpath load of the tool, and of the tree's libraries it pulls
  #   in, must resolve into the tree. dyld's fallback search quietly takes the
  #   OS's /usr/lib/libcurl.4.dylib when an @rpath load misses. That same file
  #   is loaded anyway, by absolute path, by the system frameworks cmake links,
  #   so a copy outside the tree is only wrong when it is the ONLY copy.
  for t in "${tools[@]}" ctest cpack; do
    f="$tree/bin/$t"
    [ -f "$f" ] || continue
    if [ "$RUNNER_OS" = macOS ]; then
      loaded=$(DYLD_PRINT_LIBRARIES=1 "$f" --version 2>&1 >/dev/null | sed -n 's/^dyld\[[0-9]*\]: <[^>]*> //p')
      want=$(rpath_leaves "$f" | sort -u)
      while :; do
        more=$( (echo "$want"; for leaf in $want; do
          if [ -f "$tree/lib/$leaf" ]; then rpath_leaves "$tree/lib/$leaf"; fi
        done) | sed '/^$/d' | sort -u)
        [ "$more" = "$want" ] && break
        want=$more
      done
      for leaf in $want; do
        if [ ! -e "$tree/lib/$leaf" ]; then
          echo "::error::$t loads @rpath/$leaf, which cvcpkg did not install"
          exit 1
        fi
        if ! awk -v t="$tree/" -v n="/$leaf" 'index($0, t) == 1 && substr($0, length($0) - length(n) + 1) == n { found = 1 } END { exit !found }' <<<"$loaded"; then
          echo "::error::$t's @rpath/$leaf did not load from $tree/lib"
          exit 1
        fi
      done
    else
      loaded=$(ldd "$f" | awk '$2 == "=>" { print $3 }')
      for lib in "$tree"/lib/*; do
        [ -e "$lib" ] || continue
        name=$(basename "$lib")
        while read -r hit; do
          case "$hit" in
            "" | "$tree"/*) ;;
            *) echo "::error::$t loads $hit, not the tree's lib/$name"; exit 1 ;;
          esac
        done <<<"$(awk -v n="/$name" 'substr($0, length($0) - length(n) + 1) == n' <<<"$loaded")"
      done
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
