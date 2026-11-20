#!/usr/bin/env bash
# pin-wasm-pre-jx1.sh PREFIX PLATFORM -- put back the pre-JX1 wasm bundles in a CI deps prefix.
#
# cy-pca/cvcpkg#624 (JX1 of docs/roadmap/WASM_JSPI_EXCEPTIONS.md) republished these packages
# for wasm and wasm-mt built with -fwasm-exceptions. libcvc still builds with Emscripten's JS
# exceptions (-fexceptions, CMakeLists.txt) until JX2, and the two don't link together.
# Against libcvc, the republished boost fails with "undefined symbol: __cpp_exception"
# (libboost_thread.a, libboost_filesystem.a), and libpng with "__wasm_longjmp".
#
# So after a job installs its deps (the newest bundles), this re-installs, for each package
# below that the prefix holds, its last bundle built the old way. It uses --no-deps, so
# nothing else in the prefix moves. JX2 deletes this script and its calls.
#
# PLATFORM is wasm or wasm-mt. The pins are the same on both.
set -euo pipefail

PREFIX="${1:?usage: pin-wasm-pre-jx1.sh PREFIX PLATFORM}"
PLATFORM="${2:?usage: pin-wasm-pre-jx1.sh PREFIX PLATFORM}"

# The newest wasm/wasm-mt bundle of each package from before the republish (2026-11-19). vtk's is
# cvc.5, master's FinishEveryRender build. Its JS-exceptions wasm bundle is from 2026-11-17.
# imagemagick is pinned ahead of its republish (JX1 bumped it, but its wasm package.files fix
# came later): cvc.23 is the newest JS-exceptions bundle on wasm and wasm-mt.
PINS=(
  boost==1.86.0+cvc.8
  datachannel-wasm==0.4.0+cvc.1
  freetype==2.13.3+cvc.4
  imagemagick==7.1.2+cvc.23
  libjpeg-turbo==3.1.0+cvc.3
  libpng==1.6.44+cvc.5
  lua==5.4.9+cvc.1
  mujoco==4.0.0+cvc.3
  mujoco3==3.17.0+cvc.2
  qhull==8.0.2+cvc.2
  tiff==4.7.0+cvc.3
  vtk==9.5.0+cvc.5
)

LOCK="${PREFIX}/share/libcvc-deps/lockfile.yaml"
test -f "${LOCK}" || { echo "pin-wasm-pre-jx1: no lockfile at ${LOCK}" >&2; exit 1; }

want=()
for spec in "${PINS[@]}"; do
  name="${spec%%==*}"
  grep -qx -- "- name: ${name}" "${LOCK}" && want+=("${spec}")
done
if ((${#want[@]} == 0)); then
  echo "pin-wasm-pre-jx1: the prefix holds none of the pinned packages"
  exit 0
fi
echo "pin-wasm-pre-jx1: ${PLATFORM}: ${want[*]}"
cvcpkg install --no-deps "${want[@]}" --prefix "${PREFIX}" \
  --platform "${PLATFORM}" --arch wasm32 --config release --link static \
  --no-fallback-to-source
for spec in "${want[@]}"; do
  name="${spec%%==*}"
  version="${spec#*==}"
  grep -A1 -x -- "- name: ${name}" "${LOCK}" | grep -q "version: ${version}$" ||
    { echo "pin-wasm-pre-jx1: ${name} is not at ${version} after the pin" >&2; exit 1; }
done
