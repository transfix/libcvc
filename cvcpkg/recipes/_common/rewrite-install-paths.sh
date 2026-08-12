#!/usr/bin/env bash
# recipes/_common/rewrite-install-paths.sh — relocatability helper.
#
# Rewrites absolute $CVC_INSTALL_DIR paths inside installed .pc and
# .cmake files so downstream consumers keep working when the package
# is unpacked at a different prefix. .pc files anchor at ${pcfiledir}
# and .cmake files at ${CMAKE_CURRENT_LIST_DIR}; the ../ suffix is
# computed per-file from its depth under $CVC_INSTALL_DIR.
#
# Idempotent — running twice is a no-op because the second pass finds
# no absolute install-dir strings to rewrite.

cvc_rewrite_install_paths() {
    local root="${CVC_INSTALL_DIR%/}"
    [ -n "$root" ] || return 0
    [ -d "$root" ] || return 0

    # Also relocate the DEPS prefix. An installed .pc/.cmake can bake an absolute
    # path to a dependency that lived under $CVC_DEPS_PREFIX at build time — e.g. a
    # CMake target export whose INTERFACE_LINK_LIBRARIES carries ${ZLIB_LIBRARIES}
    # (an absolute path emitted by the classic FindZLIB module) instead of the
    # relocatable ZLIB::ZLIB target. That path exists on no consumer machine, so a
    # downstream find_package()+target_link fails everywhere. In cvcpkg's FLAT
    # install every dependency co-locates with the package at the consumer prefix,
    # so a $CVC_DEPS_PREFIX path resolves to the SAME per-file anchor as a
    # $CVC_INSTALL_DIR path. This only fires where such an absolute dep path
    # literally appears (a no-op for recipes whose exports are already clean —
    # CMake normally emits its OWN paths via ${_IMPORT_PREFIX}; the leak is
    # external-dep absolute paths it does not relocate).
    local deps="${CVC_DEPS_PREFIX:-}"
    deps="${deps%/}"

    local root_esc deps_esc
    root_esc=$(printf '%s' "$root" | sed 's/[]\/$*.^[]/\\&/g')
    [ -n "$deps" ] && deps_esc=$(printf '%s' "$deps" | sed 's/[]\/$*.^[]/\\&/g')

    local count=0
    local f dir remainder depth rel i anchor prefix has_root has_deps
    while IFS= read -r -d '' f; do
        has_root=0
        has_deps=0
        grep -q -F -- "$root" "$f" 2>/dev/null && has_root=1
        [ -n "$deps" ] && grep -q -F -- "$deps" "$f" 2>/dev/null && has_deps=1
        [ "$has_root" -eq 1 ] || [ "$has_deps" -eq 1 ] || continue
        dir="$(dirname "$f")"
        remainder="${dir#$root}"
        remainder="${remainder#/}"
        depth=0
        if [ -n "$remainder" ]; then
            local _IFS_saved="$IFS"
            IFS=/
            for _ in $remainder; do depth=$((depth + 1)); done
            IFS="$_IFS_saved"
        fi
        rel=""
        for ((i = 0; i < depth; i++)); do rel+="../"; done
        rel="${rel%/}"
        [ -z "$rel" ] && rel="."
        case "$f" in
            *.pc)    anchor='${pcfiledir}' ;;
            *.cmake) anchor='${CMAKE_CURRENT_LIST_DIR}' ;;
            *)       continue ;;
        esac
        prefix="${anchor}/${rel}"
        # -i.bak keeps us portable across GNU and BSD sed. Rewrite the install
        # prefix first (it is the more specific match when the two prefixes
        # overlap), then the deps prefix.
        [ "$has_root" -eq 1 ] && sed -i.bak "s|${root_esc}|${prefix}|g" "$f"
        [ "$has_deps" -eq 1 ] && sed -i.bak "s|${deps_esc}|${prefix}|g" "$f"
        rm -f "${f}.bak"
        count=$((count + 1))
    done < <(find "$root" \( -name '*.pc' -o -name '*.cmake' \) -type f -print0)

    if [ "$count" -gt 0 ]; then
        echo "── cvc_rewrite_install_paths: normalized ${count} file(s) under ${root} ──"
    fi
}

# cvc_relocate_macos_dylibs — rewrite autotools/libtool dylib install names to
# @rpath (macOS only; no-op elsewhere).
#
# cvc_cmake_build gives CMake-built libraries @loader_path rpaths, but GNU
# autotools bakes the absolute --prefix (a per-build temp $CVC_INSTALL_DIR) into
# every produced dylib: both its own id (LC_ID_DYLIB) and its load commands for
# sibling libraries. Once the bundle is unpacked at a different prefix those
# temp paths no longer exist, so dyld aborts at load time
# ("Library not loaded: <build-temp>/lib/libFoo.dylib"). Rewrite the id and any
# dependency that points back into our install prefix to @rpath/<name>;
# consumers already add the real libdir to their rpath (-Wl,-rpath,.../lib).
#
# Operates on versioned real files (e.g. libgmpxx.4.dylib); the unversioned
# symlinks (libgmpxx.dylib) resolve to them, so consumers that link the symlink
# record the fixed @rpath id. Idempotent: entries already @rpath don't match the
# temp prefix. No-op off macOS or when install_name_tool/otool are unavailable.
cvc_relocate_macos_dylibs() {
    [ "${CVC_PLATFORM:-}" = "macos" ] || return 0
    command -v install_name_tool >/dev/null 2>&1 || return 0
    command -v otool >/dev/null 2>&1 || return 0

    local root="${CVC_INSTALL_DIR%/}"
    local libdir="${root}/lib"
    [ -d "$libdir" ] || return 0

    local count=0 dylib base dep
    while IFS= read -r -d '' dylib; do
        base="$(basename "$dylib")"
        chmod u+w "$dylib" 2>/dev/null || true
        # Set the library's own id to @rpath first, so the otool pass below sees
        # (and skips) the already-relocated id.
        install_name_tool -id "@rpath/${base}" "$dylib" 2>/dev/null || true
        while IFS= read -r dep; do
            case "$dep" in
                "${root}"/*)
                    install_name_tool -change "$dep" "@rpath/$(basename "$dep")" "$dylib" 2>/dev/null || true
                    ;;
            esac
        done < <(otool -L "$dylib" | awk 'NR>1 {print $1}')
        count=$((count + 1))
    done < <(find "$libdir" -type f -name '*.dylib' -print0)

    if [ "$count" -gt 0 ]; then
        echo "── cvc_relocate_macos_dylibs: relocated ${count} dylib(s) to @rpath under ${libdir} ──"
    fi
}
