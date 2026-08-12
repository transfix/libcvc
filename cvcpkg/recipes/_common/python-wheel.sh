#!/usr/bin/env bash
# recipes/_common/python-wheel.sh — per-interpreter wheel helper (Phase 7).
#
# cvcpkg ships several CPython interpreters as recipes (python311, python312,
# python313, python313t).  A wheel package is a *matrix* of recipes — one
# column per interpreter ABI (<name>-cp311 ... -cp313t), each depending on its
# interpreter and installing into that interpreter's own site-packages inside
# the prefix.  A column never touches another interpreter's tree: the old
# cross-interpreter copy fan-out is gone (per-interpreter columns made it
# meaningless).
#
# The wheel itself is fetched and sha256-verified by cvcpkg core (source.type
# python_wheel) and handed to the build script already on disk, so the install
# below runs fully offline (--no-index) and never re-resolves a pin.
#
# Usage in a wheel recipe's build.sh:
#
#     #!/usr/bin/env bash
#     set -euo pipefail
#     . "$(dirname "$0")/../_common/python-wheel.sh"
#     cvc_pip_install_wheel
#     cvc_python_check "import numpy; numpy.add(1, 2)"
#
# The recipe's `python:` block supplies CVC_PYTHON_ABI / CVC_PYTHON_INTERPRETER
# (exported by the builder from the recipe); they can also be set directly.

set -euo pipefail

# Map a cvcpkg interpreter recipe name to the X.Y[t] version it reports.
#
#   python311  -> 3.11      python313t -> 3.13t
#
# This is authoritative even for an abi3 recipe, whose `abi` tag (abi3) carries
# no version: the interpreter a wheel is *installed under* is named by the
# recipe's `interpreter:` field, not by the ABI tag.
cvc_interp_version() {
  local interp="${1:?usage: cvc_interp_version <pythonNNN[t]>}"
  local digits="${interp#python}"    # python313t -> 313t
  local suffix=""
  case "${digits}" in
    *t) suffix="t"; digits="${digits%t}" ;;
  esac
  echo "${digits:0:1}.${digits:1}${suffix}"   # 313 -> 3.13(t)
}

# Echo the prefix interpreter executable for an X.Y[t] version.
cvc_python_exe_for() {
  local ver="${1:?usage: cvc_python_exe_for <X.Y[t]>}"
  : "${CVC_DEPS_PREFIX:?CVC_DEPS_PREFIX must be set}"
  local exe="${CVC_DEPS_PREFIX}/bin/python${ver}"
  if [ ! -x "${exe}" ]; then
    echo "cvc_python_exe_for: interpreter not found: ${exe}" >&2
    echo "  (does this recipe depend on python${ver//./}?)" >&2
    return 1
  fi
  echo "${exe}"
}

# Echo the target interpreter's executable inside $CVC_DEPS_PREFIX.
#
# The interpreter to install *under* is the recipe's `interpreter:` field
# (CVC_PYTHON_INTERPRETER) — resolved against the *prefix* interpreter, never
# the host's. For an ordinary cpNN column this equals the ABI's version; for a
# column installing a stable-ABI wheel (abi: abi3) the tag carries no version,
# so the interpreter field alone names the column's install target.
cvc_python_exe() {
  : "${CVC_PYTHON_INTERPRETER:?CVC_PYTHON_INTERPRETER must be set (recipe python.interpreter)}"
  cvc_python_exe_for "$(cvc_interp_version "${CVC_PYTHON_INTERPRETER}")"
}

# True when the target ABI is the GIL-disabled one.
cvc_python_is_free_threaded() {
  case "${CVC_PYTHON_ABI:-}" in
    *t) return 0 ;;
    *)  return 1 ;;
  esac
}

# Install the verified wheel into the prefix interpreter's site-packages.
#
# --no-deps:  transitive Python deps are themselves cvcpkg recipes, resolved by
#             the depends graph — pip must not go behind cvcpkg's back.
# --no-index: the artifact is already on disk and pinned; forbid any network
#             resolution, which is what makes air-gapped installs work.
cvc_pip_install_wheel() {
  : "${CVC_SOURCE_DIR:?CVC_SOURCE_DIR must be set}"
  : "${CVC_INSTALL_DIR:?CVC_INSTALL_DIR must be set}"

  local py wheel
  py="$(cvc_python_exe)"

  wheel="$(find "${CVC_SOURCE_DIR}" -maxdepth 1 -name '*.whl' -print -quit)"
  if [ -z "${wheel}" ]; then
    echo "cvc_pip_install_wheel: no .whl in ${CVC_SOURCE_DIR}" >&2
    return 1
  fi

  echo "installing $(basename "${wheel}") into ${CVC_INSTALL_DIR} using ${py}"
  "${py}" -m pip install \
    --no-deps \
    --no-index \
    --no-compile \
    --ignore-installed \
    --prefix "${CVC_INSTALL_DIR}" \
    "${wheel}"
}

# Run a Python snippet under the target interpreter, with the wheel we just
# installed importable.
#
# For a free-threaded ABI this asserts the GIL is actually *disabled* at
# runtime (-X gil=0 / PYTHON_GIL=0) before running the snippet.  That check is
# the substance of the no-GIL claim: a cp313t wheel that only imports under a
# re-enabled GIL has not been shown to work without one, and cvcpkg would be
# publishing an unproven guarantee.  If CPython refuses to disable the GIL
# (e.g. an extension on the prefix forced it back on), the build fails here.
cvc_python_check() {
  local snippet="${1:?usage: cvc_python_check <python-snippet>}"
  local py libdir
  py="$(cvc_python_exe)"

  # sysconfig would report the *interpreter's* own prefix, not the staging dir
  # we just installed into, so locate the staged site-packages directly.  The
  # find covers both layouts: lib/pythonX.Yt/site-packages and Lib/site-packages.
  libdir="$(find "${CVC_INSTALL_DIR}" -maxdepth 3 -type d -name 'site-packages' -print -quit)"
  if [ -z "${libdir}" ]; then
    echo "cvc_python_check: no site-packages found under ${CVC_INSTALL_DIR}" >&2
    return 1
  fi
  export PYTHONPATH="${libdir}${PYTHONPATH:+:${PYTHONPATH}}"

  if cvc_python_is_free_threaded; then
    echo "verifying ${CVC_PYTHON_ABI} with the GIL disabled"
    PYTHON_GIL=0 "${py}" -X gil=0 -c "
import sys, sysconfig
if not sysconfig.get_config_var('Py_GIL_DISABLED'):
    sys.exit('${CVC_PYTHON_ABI}: interpreter is not a free-threaded build')
if sys._is_gil_enabled():
    sys.exit('${CVC_PYTHON_ABI}: GIL was re-enabled at runtime; no-GIL support unproven')
print('GIL disabled:', not sys._is_gil_enabled())
${snippet}
print('${CVC_PYTHON_ABI} check OK (GIL disabled)')
"
  else
    "${py}" -c "
${snippet}
print('${CVC_PYTHON_ABI} check OK')
"
  fi
}
