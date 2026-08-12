#!/usr/bin/env bash
# recipes/_common/build-python.sh — shared CPython build logic.
#
# Sourced by recipes/python3{11,12,13}/build.sh after they export:
#   PYTHON_VERSION   e.g. "3.12.10"
#   PYTHON_MINOR     e.g. "3.12"
#
# Required cvcpkg env vars (set by builder):
#   CVC_INSTALL_DIR, CVC_SOURCE_DIR, CVC_BUILD_DIR (unused — CPython uses
#   in-source build), CVC_DEPS_PREFIX, CVC_JOBS, CVC_LINK, CVC_PLATFORM.
set -euo pipefail

: "${CVC_INSTALL_DIR:?}"
: "${CVC_SOURCE_DIR:?}"
: "${CVC_JOBS:=$(nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 4)}"
: "${CVC_DEPS_PREFIX:?CVC_DEPS_PREFIX must be set to the cvcpkg deps prefix}"
: "${PYTHON_VERSION:?PYTHON_VERSION must be exported before sourcing this script}"
: "${PYTHON_MINOR:?PYTHON_MINOR must be exported before sourcing this script}"
# PYTHON_LDVERSION — ABI suffix for the binary/lib/include names.
# Defaults to PYTHON_MINOR; free-threaded builds set this to e.g. "3.13t".
: "${PYTHON_LDVERSION:=${PYTHON_MINOR}}"
# PYTHON_DISABLE_GIL — set to "1" to pass --disable-gil (free-threaded build).
: "${PYTHON_DISABLE_GIL:=0}"

cd "${CVC_SOURCE_DIR}"

# Use gmake on BSDs (CPython's Makefile is GNU make).
MAKE=make
if command -v gmake >/dev/null 2>&1; then
    MAKE=gmake
fi

# --- Detect cross-compile targets ---
# wasm / wasi / cosmo: static builds with a cross host triple.
# Native platforms (linux, macos, freebsd, openbsd, netbsd, windows) use
# the native toolchain with shared libraries.
# NOTE: must be declared before the RPATH/flags section below which
# gates LDFLAGS on IS_CROSS — set -u would error if IS_CROSS were unset.
IS_CROSS=false
CROSS_HOST=""
case "${CVC_PLATFORM}" in
    wasm)   IS_CROSS=true; CROSS_HOST="wasm32-emscripten" ;;
    wasi)   IS_CROSS=true; CROSS_HOST="wasm32-wasi" ;;
    cosmo)  IS_CROSS=true; CROSS_HOST="x86_64-cosmo" ;;
esac

# --- RPATH ---
# Embed $ORIGIN/../lib so the installed python3.X binary finds:
#   • libpython3.X.so  (installed alongside it in lib/)
#   • libssl, libz, libffi, libsqlite3, etc. (merged into same prefix)
case "${CVC_PLATFORM}" in
    macos)
        RPATH_SELF="@loader_path/../lib"
        RPATH_DEPS="@loader_path/../lib"
        ;;
    *)
        RPATH_SELF="\$ORIGIN/../lib"
        RPATH_DEPS="\$ORIGIN/../lib"
        ;;
esac

export PKG_CONFIG_PATH="${CVC_DEPS_PREFIX}/lib/pkgconfig${PKG_CONFIG_PATH:+:$PKG_CONFIG_PATH}"
export CPPFLAGS="-I${CVC_DEPS_PREFIX}/include ${CPPFLAGS:-}"
if [ "$IS_CROSS" = false ]; then
    export LDFLAGS="-L${CVC_DEPS_PREFIX}/lib -Wl,-rpath,${RPATH_SELF} ${LDFLAGS:-}"
fi

# On macOS ensure the deployment target is propagated.
if [[ "${CVC_PLATFORM}" == "macos" ]]; then
    export MACOSX_DEPLOYMENT_TARGET="${MACOSX_DEPLOYMENT_TARGET:-13.0}"
fi

# --- Configure flags ---
CONFIGURE_ARGS=(
    --prefix="${CVC_INSTALL_DIR}"
    --enable-shared
    # pip is included in the stdlib; ensurepip bootstraps it at build time.
    --with-ensurepip=upgrade
    # Point at the cvcpkg OpenSSL so ssl/hashlib use our library, not
    # whatever happens to be on PATH.  openssldir=/etc/ssl (baked into our
    # OpenSSL build) means CA verification uses the host system trust store.
    --with-openssl="${CVC_DEPS_PREFIX}"
    --with-ssl-default-suites=openssl
    # Install to versioned paths: lib/python3.X/, bin/python3.X, etc.
    # Multiple Python minor versions coexist in the same prefix this way.
    --enable-ipv6
)

# Cross-compilation targets: static-only, explicit host, no readline.
if [ "$IS_CROSS" = true ]; then
    CONFIGURE_ARGS+=(--disable-shared --host="${CROSS_HOST}")
    # readline/ncurses not available on wasm/wasi/cosmo.
    CONFIGURE_ARGS+=(--with-readline=tkinter)
else
    # Use the wide-char ncurses (libncursesw) for curses + readline.
    CONFIGURE_ARGS+=(--with-readline=readline)
fi

# Enable PGO on platforms where it's supported and reliable.
# Disabled on BSD and cross-compile variants.
case "${CVC_PLATFORM}" in
    linux)
        CONFIGURE_ARGS+=(--enable-optimizations)
        ;;
    macos)
        CONFIGURE_ARGS+=(--enable-optimizations)
        ;;
esac

# Free-threaded (no-GIL) build.
if [ "${PYTHON_DISABLE_GIL}" = "1" ]; then
    CONFIGURE_ARGS+=(--disable-gil)
fi

./configure "${CONFIGURE_ARGS[@]}"

$MAKE -j "${CVC_JOBS}"
$MAKE install

# --- Relocatable RPATH post-fixup (native only) ---
# CPython's Makefile bakes the absolute build-time LDFLAGS rpath; patch
# the installed binary so it uses $ORIGIN-relative paths instead.
if [ "$IS_CROSS" = false ]; then
    PY_BIN="${CVC_INSTALL_DIR}/bin/python${PYTHON_LDVERSION}"
    if [[ "${CVC_PLATFORM}" != "macos" ]]; then
        # ELF (Linux/BSD): overwrite the rpath with an $ORIGIN-relative path so
        # the install is relocatable. CPython's Makefile otherwise bakes a
        # make-MANGLED rpath — the $O in $ORIGIN is a make variable, so it
        # expands to a broken "RIGIN/../lib" (RUNPATH). patchelf writes literal
        # bytes and sidesteps make/shell $ORIGIN escaping entirely, so it is a
        # REQUIRED build dependency here (declared per-recipe for linux/*bsd).
        if ! command -v patchelf >/dev/null 2>&1; then
            echo "build-python.sh: patchelf required on ${CVC_PLATFORM} but not found on PATH" >&2
            exit 1
        fi
        patchelf --set-rpath '$ORIGIN/../lib' "${PY_BIN}"
        # Extension modules (lib/pythonX.Y/**): point each back at the prefix
        # lib/ (where libssl/libcrypto/libffi/... live). The depth varies —
        # stdlib C extensions sit at lib/pythonX.Y/lib-dynload/ (needs
        # $ORIGIN/../..) while site-packages/<pkg>/**.so are deeper — so a single
        # fixed relative path is WRONG for lib-dynload (a uniform
        # $ORIGIN/../../.. lands on the prefix ROOT, not lib/, so _ssl.so then
        # loads the system libcrypto -> "OPENSSL_x.y.z not found"). Compute the
        # correct $ORIGIN-relative path to lib/ per file. (cvcpkg's own relocation
        # pass later PREPENDS $ORIGIN and preserves these $ORIGIN-relative entries.)
        # The FREE-THREADED build installs its stdlib under the LDVERSION name
        # (lib/python3.13t/), not lib/python3.13/ — so keying this off
        # PYTHON_MINOR pointed `find` at a directory that does not exist. Under
        # `set -o pipefail` that non-zero find failed the whole build, which is
        # why python313t could not be rebuilt (and cascade-cancelled every
        # -cp313t consumer); when it did not fail, the extension rpaths were
        # simply never patched. Resolve the real directory, and guard with -d so
        # a layout we do not expect degrades to "nothing to patch" instead of
        # killing the build.
        _STDLIB_DIR="${CVC_INSTALL_DIR}/lib/python${PYTHON_LDVERSION}"
        [ -d "${_STDLIB_DIR}" ] || _STDLIB_DIR="${CVC_INSTALL_DIR}/lib/python${PYTHON_MINOR}"
        if [ -d "${_STDLIB_DIR}" ]; then
            find "${_STDLIB_DIR}" -name '*.so' -print0 \
                | while IFS= read -r -d '' _so; do
                    _rel="$(realpath --relative-to="$(dirname "${_so}")" "${CVC_INSTALL_DIR}/lib")"
                    patchelf --set-rpath "\$ORIGIN/${_rel}" "${_so}" 2>/dev/null || true
                  done
        else
            echo "build-python.sh: no stdlib dir under ${CVC_INSTALL_DIR}/lib — skipping rpath pass" >&2
        fi
    fi

    if [[ "${CVC_PLATFORM}" == "macos" ]]; then
        # Fix install_name on the framework-less shared build.
        DYLIB="${CVC_INSTALL_DIR}/lib/libpython${PYTHON_LDVERSION}.dylib"
        if [[ -f "$DYLIB" ]]; then
            install_name_tool -id "@rpath/libpython${PYTHON_LDVERSION}.dylib" "$DYLIB"
            install_name_tool -change \
                "${CVC_INSTALL_DIR}/lib/libpython${PYTHON_LDVERSION}.dylib" \
                "@rpath/libpython${PYTHON_LDVERSION}.dylib" \
                "${PY_BIN}" 2>/dev/null || true
        fi
    fi
fi

# --- Alias hygiene: keep version-specific builds side-by-side safe ---
# Version-specific recipes must ship ONLY versioned binaries (python3.X,
# pip3.X, ...) so that several minor versions can be installed into the same
# prefix without fighting over the generic names. The generic
# python3/python/pip3/pip aliases are owned by the `python3` meta-recipe
# instead, which points them at a single default interpreter.
#
# CPython's `make install` and ensurepip create some of these generic
# aliases; strip them here so they never end up in the staged tree.
# (stage_bundle copies the whole install dir — package.files does not filter
# it — so removing the files here is what actually prevents the collision.)
cd "${CVC_INSTALL_DIR}/bin"
rm -f python3 python python3-config pip3 pip 2>/dev/null || true
if [ "${PYTHON_LDVERSION}" != "${PYTHON_MINOR}" ]; then
    # Free-threaded (t) build: keep a short pythonXt alias (e.g. python3t).
    MAJOR="${PYTHON_MINOR%%.*}"
    ln -sf "python${PYTHON_LDVERSION}" "python${MAJOR}t" 2>/dev/null || true
    # ensurepip names its console script pip3.X even in the free-threaded
    # build; give it the t-suffixed name (pip3.13t) so it can never collide
    # with the non-t interpreter's bin/pip3.X in a shared prefix. (Its
    # shebang already runs the t interpreter — ensurepip ran under it.)
    if [ -f "pip${PYTHON_MINOR}" ]; then
        mv -f "pip${PYTHON_MINOR}" "pip${PYTHON_LDVERSION}"
    fi
fi
