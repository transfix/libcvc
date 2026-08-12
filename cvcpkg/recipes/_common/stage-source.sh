#!/usr/bin/env bash
# recipes/_common/stage-source.sh — source-recipe staging helper.
#
# A *source recipe* is an ordinary recipe whose deliverable is just the
# (patched) source files — no compilation.  It declares `platform: any`
# (so it is built once and valid everywhere; its arch is `noarch`), and
# its build script simply stages the source tree into the install dir
# under the canonical layout:
#
#     $CVC_INSTALL_DIR/src/$CVC_COMPONENT/
#
# Downstream platform/arch recipes then depend on the source recipe
# (`depends.build`), find the staged tree at
# `$CVC_BUILD_PREFIX/src/<name>/`, and compile it into a real binary.
#
# Placement follows the dependency edge: a source recipe consumed via
# `depends.build` is part of the build closure, so it is installed into the
# build prefix (stripped on install unless --keep-build-prefix) rather than the
# deliverable install prefix.  Nothing about being "source" routes it -- being a
# build dep does.
#
# Usage in a source recipe's build.sh:
#
#     #!/usr/bin/env bash
#     set -euo pipefail
#     . "$(dirname "$0")/../_common/stage-source.sh"
#     cvc_stage_source            # stages everything under $CVC_SOURCE_DIR
#     # optionally: cvc_stage_source include src   # only these subpaths
#
# Idempotent: re-running replaces the staged tree.

set -euo pipefail

# Stage the recipe's source tree into the canonical source layout.
#
# With no arguments, the entire $CVC_SOURCE_DIR is staged.  With
# arguments, only the named paths (relative to $CVC_SOURCE_DIR) are
# staged — useful to drop build cruft (e.g. `.git`, test fixtures).
cvc_stage_source() {
  : "${CVC_SOURCE_DIR:?CVC_SOURCE_DIR must be set}"
  : "${CVC_INSTALL_DIR:?CVC_INSTALL_DIR must be set}"
  : "${CVC_COMPONENT:?CVC_COMPONENT must be set}"

  local dest="${CVC_INSTALL_DIR}/src/${CVC_COMPONENT}"
  rm -rf "${dest}"
  mkdir -p "${dest}"

  if [ "$#" -eq 0 ]; then
    # Copy the whole tree (contents, not the parent dir itself).
    cp -R "${CVC_SOURCE_DIR}/." "${dest}/"
  else
    local item
    for item in "$@"; do
      if [ -e "${CVC_SOURCE_DIR}/${item}" ]; then
        mkdir -p "${dest}/$(dirname "${item}")"
        cp -R "${CVC_SOURCE_DIR}/${item}" "${dest}/${item}"
      else
        echo "cvc_stage_source: warning: '${item}' not found under CVC_SOURCE_DIR" >&2
      fi
    done
  fi

  echo "staged source for ${CVC_COMPONENT} -> ${dest}"
}

# Echo the staged-source directory for a dependency source recipe, so a
# downstream build.sh can locate it: SRC=$(cvc_source_dir_of mysource).
#
# Source packages are build dependencies, so they live in the build prefix
# ($CVC_BUILD_PREFIX).  Fall back to $CVC_DEPS_PREFIX when the build prefix is
# not separated (legacy layout, or --build-prefix == --prefix).
cvc_source_dir_of() {
  local name="${1:?usage: cvc_source_dir_of <source-recipe-name>}"
  local root="${CVC_BUILD_PREFIX:-${CVC_DEPS_PREFIX:-}}"
  if [ -z "${root}" ]; then
    echo "cvc_source_dir_of: CVC_BUILD_PREFIX or CVC_DEPS_PREFIX must be set" >&2
    return 1
  fi
  echo "${root}/src/${name}"
}
