#!/usr/bin/env bash
# Install libcvc's third-party dependencies into an SDK tree, as the same cvcpkg
# bundles the build used.
#
#   sdk-bundle-deps.sh DEPS_PREFIX SDK_DIR
#
# Used by release.yml's stage steps (Linux, macOS, and Windows through Git
# Bash). Run it on an empty SDK_DIR BEFORE `cmake --install` puts libcvc itself
# there.
#
# Unit of bundling: the whole cvcpkg bundle, not individual files. A dep's CMake
# package config is only loadable when every file its export set names is
# present. hdf5-targets.cmake, for instance, FATAL_ERRORs on a missing bin/h5diff
# or lib/libhdf5.a, and Boost's per-component configs skip any variant whose
# library is gone. So per-file selection (an ldd closure, say) cannot also ship
# working configs.
#
# The SDK gets every bundle in DEPS_PREFIX's lockfile (the release/shared
# closure libcvc was built against) except the ones named below, which libcvc's
# release build never links. Each is installed at the exact version the build
# used, so cvcpkg serves it from the download cache this job already filled.
# cvcpkg also pulls in anything a kept bundle requires, so an exclusion libcvc
# still needs comes back by itself; the version check at the end then fails if
# what landed in the SDK differs from what libcvc was built against.
#
# (Copying the prefix and running `cvcpkg uninstall` on the unused bundles would
# be simpler, but uninstall cannot read the Windows bundles: they are zip files
# named .tar.zst, which install handles and uninstall's tar reader does not.)
#
# A component missing from the list is shipped, which costs only size. A
# component wrongly on it is caught by sdk-check-closure.sh and by release.yml's
# sdk-smoke job.
set -euo pipefail

if [ $# -ne 2 ]; then
  echo "usage: $0 DEPS_PREFIX SDK_DIR" >&2
  exit 2
fi
deps=$1
sdk=$2
cvcpkg_sdk=$sdk
# Windows (Git Bash): callers pass native paths. Use POSIX ones for the shell
# tools and a mixed D:/... one for cvcpkg, a native Python program.
if command -v cygpath >/dev/null 2>&1; then
  deps=$(cygpath -u "$deps")
  sdk=$(cygpath -u "$sdk")
  cvcpkg_sdk=$(cygpath -m "$sdk")
fi

lock="$deps/share/libcvc-deps/lockfile.yaml"
if [ ! -f "$lock" ]; then
  echo "::error::$deps is not a cvcpkg prefix (no $lock)" >&2
  exit 1
fi

# In the closure cvcpkg/recipes/libcvc resolves to, but not linked by the
# release configure in release.yml (CVC_ENABLE_GRPC and CVC_LOG4CPLUS_DEFAULT are
# OFF; nothing in libcvc's CMake finds GSL, levmar, OpenBLAS or libyaml).
# Together they are about half the prefix. cmake and ninja are build tools that
# the Windows closure carries along; an SDK has no use for them.
unused=" abseil c-ares grpc protobuf re2 openssl openblas gsl levmar log4cplus yaml cmake ninja "

# "name version" per bundle, and one header field, from a cvcpkg lockfile.
bundles() {
  tr -d '\r' <"$1" | awk '/^- name: /{n=$3} /^  version: /{print n, $2}'
}
field() { tr -d '\r' <"$1" | sed -n "s/^$2: //p"; }

pins=()
while read -r name version; do
  case "$unused" in *" $name "*) continue ;; esac
  pins+=("$name==$version")
done < <(bundles "$lock")

mkdir -p "$sdk"
cvcpkg install "${pins[@]}" --prefix "$cvcpkg_sdk" \
  --platform "$(field "$lock" platform)" --arch "$(field "$lock" arch)" \
  --config "$(field "$lock" config)" --link "$(field "$lock" link)"

# Every bundle in the SDK must be one the build used, at the same version.
# Anything else means cvcpkg resolved a dependency differently from the build.
sdk_lock="$sdk/share/libcvc-deps/lockfile.yaml"
drift=$(comm -23 <(bundles "$sdk_lock" | sort) <(bundles "$lock" | sort))
if [ -n "$drift" ]; then
  echo "::error::the SDK's bundles differ from the ones libcvc was built against:"
  echo "$drift"
  exit 1
fi
back=""
while read -r name _; do
  case "$unused" in *" $name "*) back="$back $name" ;; esac
done < <(bundles "$sdk_lock")
if [ -n "$back" ]; then
  echo "::notice::excluded but required by a kept bundle, so shipped anyway:$back"
fi

# cvcpkg's prefix-management files, as opposed to bundle content. bin/activate*
# hard-code the build runner's path. share/libcvc-deps holds a per-component
# manifest and recipe copy. lib/cmake/{cvcpkg,libcvc-deps} only point
# CMAKE_PREFIX_PATH at the prefix. Keep the lockfile: it records the exact
# bundle versions this SDK carries.
rm -f "$sdk"/bin/activate "$sdk"/bin/activate.*
rm -rf "$sdk/lib/cmake/cvcpkg" "$sdk/lib/cmake/libcvc-deps"
find "$sdk/share/libcvc-deps" -mindepth 1 -maxdepth 1 ! -name lockfile.yaml -exec rm -rf {} +

kept=$(bundles "$sdk_lock" | cut -d' ' -f1)
echo "Bundled $(echo "$kept" | wc -l | tr -d ' ') cvcpkg bundles into $sdk:"
echo "$kept" | tr '\n' ' '
echo
du -sh "$sdk"
