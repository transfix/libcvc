#!/usr/bin/env bash
# Copy libcvc's third-party dependencies from a cvcpkg prefix into an SDK tree.
#
#   sdk-bundle-deps.sh DEPS_PREFIX SDK_DIR
#
# Used by release.yml's Linux and macOS stage steps. Run it on an empty SDK_DIR
# BEFORE `cmake --install` puts libcvc itself there.
#
# Unit of bundling: the whole cvcpkg bundle, not individual files. A dep's CMake
# package config is only loadable when every file its export set names is
# present. hdf5-targets.cmake, for instance, FATAL_ERRORs on a missing bin/h5diff
# or lib/libhdf5.a, and Boost's per-component configs skip any variant whose
# library is gone. So per-file selection (an ldd closure, say) cannot also ship
# working configs. Instead the SDK gets the full prefix that libcvc was built
# against, minus the bundles named below, which libcvc's release build never
# links. `cvcpkg uninstall` removes exactly the files each bundle installed, and
# it refuses if a kept bundle depends on one being removed, so an entry that
# libcvc still needs fails here instead of shipping a broken SDK.
#
# A component missing from this list is shipped, which costs only size. A
# component wrongly on it is caught by sdk-check-closure.sh and by release.yml's
# sdk-smoke job.
set -euo pipefail

if [ $# -ne 2 ]; then
  echo "usage: $0 DEPS_PREFIX SDK_DIR" >&2
  exit 2
fi
deps=$1
sdk=$2

lock="$deps/share/libcvc-deps/lockfile.yaml"
if [ ! -f "$lock" ]; then
  echo "::error::$deps is not a cvcpkg prefix (no $lock)" >&2
  exit 1
fi

# In cvcpkg/recipes/libcvc's runtime closure, but not linked by the release
# configure in release.yml (CVC_ENABLE_GRPC and CVC_LOG4CPLUS_DEFAULT are OFF;
# nothing in libcvc's CMake finds GSL, levmar, OpenBLAS or libyaml). Together
# they are about half the prefix.
unused=(abseil c-ares grpc protobuf re2 openssl openblas gsl levmar log4cplus yaml)

mkdir -p "$sdk"
cp -a "$deps/." "$sdk/"

installed=$(sed -n 's/^- name: //p' "$lock")
drop=()
for c in "${unused[@]}"; do
  if grep -qx "$c" <<<"$installed"; then
    drop+=("$c")
  fi
done
if [ ${#drop[@]} -gt 0 ]; then
  cvcpkg uninstall --prefix "$sdk" "${drop[@]}"
fi

# cvcpkg's prefix-management files, as opposed to bundle content. bin/activate*
# hard-code the build runner's prefix path. share/libcvc-deps holds a per-
# component manifest and recipe copy for every bundle ever installed, including
# the ones just removed. lib/cmake/{cvcpkg,libcvc-deps} only point
# CMAKE_PREFIX_PATH at the prefix. Keep the lockfile: it records the exact
# bundle versions this SDK carries.
rm -f "$sdk"/bin/activate "$sdk"/bin/activate.*
rm -rf "$sdk/lib/cmake/cvcpkg" "$sdk/lib/cmake/libcvc-deps"
find "$sdk/share/libcvc-deps" -mindepth 1 -maxdepth 1 ! -name lockfile.yaml -exec rm -rf {} +

echo "Bundled $(sed -n 's/^- name: //p' "$sdk/share/libcvc-deps/lockfile.yaml" | wc -l | tr -d ' ') cvcpkg bundles into $sdk:"
sed -n 's/^- name: /  /p' "$sdk/share/libcvc-deps/lockfile.yaml" | tr '\n' ' '
echo
du -sh "$sdk"
