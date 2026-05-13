# ─────────────────────────────────────────────────────────────────────
# imagemagick (overlay port)
#
# vcpkg upstream does not ship an `imagemagick` port. ImageMagick's
# Windows source build is intricate (its own VisualMagick configure
# wizard, ~20 optional delegate libraries, Q-depth and HDRI matrices)
# and not amenable to a quick vcpkg port. For CI usage we instead
# repackage the official upstream Inno Setup installer:
#
#   * Download the installer + a pinned innoextract.exe
#   * Run innoextract to unpack the installer into a temp tree
#   * Copy headers / import libraries / DLLs into vcpkg's layout so
#     CMake's stock FindImageMagick.cmake can locate them via the
#     vcpkg toolchain's CMAKE_PREFIX_PATH.
#
# This intentionally tracks the Q16-HDRI x64 build, which matches the
# brew-installed flavor on macOS and keeps MAGICKCORE_QUANTUM_DEPTH=16
# / MAGICKCORE_HDRI_ENABLE=1 consistent across platforms.
# ─────────────────────────────────────────────────────────────────────

if(NOT VCPKG_TARGET_IS_WINDOWS)
  message(FATAL_ERROR "imagemagick overlay port is Windows-only.")
endif()

if(NOT VCPKG_TARGET_ARCHITECTURE STREQUAL "x64")
  message(FATAL_ERROR "imagemagick overlay port supports x64 only.")
endif()

set(IM_VERSION "7.1.2-22")

# Mirror order matters: GitHub Releases is the canonical permanent home for
# every published ImageMagick installer (assets are immutable once a tag
# ships), so we try it first. imagemagick.org/archive/binaries/ only keeps
# the *latest* point release live -- once a new -NN drops, the previous URL
# returns 404 -- so it is best-effort fallback in case GitHub is unreachable.
# vcpkg also resolves cached downloads (e.g. populated by a libcvc-deps
# prefetch step) by SHA512 in $VCPKG_ROOT/downloads regardless of URL, so
# this list never has to enumerate every conceivable mirror.
vcpkg_download_distfile(IM_INSTALLER
  URLS
    "https://github.com/ImageMagick/ImageMagick/releases/download/${IM_VERSION}/ImageMagick-${IM_VERSION}-Q16-HDRI-x64-dll.exe"
    "https://imagemagick.org/archive/binaries/ImageMagick-${IM_VERSION}-Q16-HDRI-x64-dll.exe"
  FILENAME "ImageMagick-${IM_VERSION}-Q16-HDRI-x64-dll.exe"
  SHA512 c3576a97380e172d6377a91831441d8f4f6bdf57819b8d6892a061355d80444bf68ef5cf08405396a5ca7cdff18eae65ab195319319d6d92026d9502b5de2fd7
)

# innoextract 1.9 supports Inno Setup 6.x (the format used by IM 7.1.2-NN).
vcpkg_download_distfile(INNOEXTRACT_ZIP
  URLS "https://github.com/dscharrer/innoextract/releases/download/1.9/innoextract-1.9-windows.zip"
  FILENAME "innoextract-1.9-windows.zip"
  SHA512 eea751bc021b8cb9a979875d7df5ef0438a8ec6157f75fa9d34b0471bb359daf15cf9cee51a21f9115cb8410bdb836a57f1cd6b2198c634cf108e6785f902488
)

# Stage everything under the buildtrees temp dir.
set(STAGE_DIR "${CURRENT_BUILDTREES_DIR}/${TARGET_TRIPLET}-stage")
file(REMOVE_RECURSE "${STAGE_DIR}")
file(MAKE_DIRECTORY "${STAGE_DIR}")

# Unpack innoextract.exe.
vcpkg_execute_required_process(
  COMMAND "${CMAKE_COMMAND}" -E tar xf "${INNOEXTRACT_ZIP}"
  WORKING_DIRECTORY "${STAGE_DIR}"
  LOGNAME "unzip-innoextract-${TARGET_TRIPLET}"
)
set(INNOEXTRACT_EXE "${STAGE_DIR}/innoextract.exe")
if(NOT EXISTS "${INNOEXTRACT_EXE}")
  message(FATAL_ERROR "innoextract.exe missing after unzip: ${INNOEXTRACT_EXE}")
endif()

# Run innoextract on the IM installer. -s strips the leading "{app}"
# prefix from extracted paths so the layout below is just "app/...".
vcpkg_execute_required_process(
  COMMAND "${INNOEXTRACT_EXE}" --silent --extract --output-dir "${STAGE_DIR}" "${IM_INSTALLER}"
  WORKING_DIRECTORY "${STAGE_DIR}"
  LOGNAME "innoextract-${TARGET_TRIPLET}"
)
set(APP_DIR "${STAGE_DIR}/app")
if(NOT EXISTS "${APP_DIR}/include")
  message(FATAL_ERROR "innoextract output missing headers: ${APP_DIR}/include")
endif()

# Headers: Magick++/, MagickCore/, MagickWand/ subtrees go straight into
# the vcpkg include root. CMake's FindImageMagick recognises this layout.
file(COPY "${APP_DIR}/include/" DESTINATION "${CURRENT_PACKAGES_DIR}/include")

# Import libraries.
file(GLOB IM_LIBS "${APP_DIR}/lib/CORE_RL_*.lib")
if(NOT IM_LIBS)
  message(FATAL_ERROR "No CORE_RL_*.lib files found under ${APP_DIR}/lib")
endif()
file(MAKE_DIRECTORY "${CURRENT_PACKAGES_DIR}/lib")
file(MAKE_DIRECTORY "${CURRENT_PACKAGES_DIR}/debug/lib")
file(COPY ${IM_LIBS} DESTINATION "${CURRENT_PACKAGES_DIR}/lib")
# Upstream ships only one (release) build; alias it for Debug so
# vcpkg's debug-config also resolves.
file(COPY ${IM_LIBS} DESTINATION "${CURRENT_PACKAGES_DIR}/debug/lib")

# CMake's FindImageMagick.cmake searches for Magick++/MagickCore/
# MagickWand/MagickWand-7.Q16HDRI/etc. and also CORE_RL_Magick++_,
# but only on recent versions. Older CMakes (< 3.27 or so) and some
# distro-patched FindImageMagick scripts only know the canonical
# unprefixed names. The IM Windows installer ships only
# 'CORE_RL_<component>_.lib', so create canonical-named hardlinks/
# copies (Magick++.lib, MagickCore.lib, MagickWand.lib) to make
# find_library() succeed regardless of FindImageMagick vintage.
foreach(_im_lib_dir
    "${CURRENT_PACKAGES_DIR}/lib" "${CURRENT_PACKAGES_DIR}/debug/lib")
  foreach(_im_component Magick++ MagickCore MagickWand)
    set(_src "${_im_lib_dir}/CORE_RL_${_im_component}_.lib")
    set(_dst "${_im_lib_dir}/${_im_component}.lib")
    if(EXISTS "${_src}" AND NOT EXISTS "${_dst}")
      configure_file("${_src}" "${_dst}" COPYONLY)
    endif()
  endforeach()
endforeach()

# Runtime DLLs (CORE_RL_*, IM_MOD_RL_*, FILTER_*) plus the C/C++ runtime
# bundled by the installer; ship them all so apps can resolve symbols.
file(GLOB IM_DLLS
  "${APP_DIR}/CORE_RL_*.dll"
  "${APP_DIR}/IM_MOD_RL_*.dll"
  "${APP_DIR}/FILTER_*.dll"
)
file(MAKE_DIRECTORY "${CURRENT_PACKAGES_DIR}/bin")
file(MAKE_DIRECTORY "${CURRENT_PACKAGES_DIR}/debug/bin")
if(IM_DLLS)
  file(COPY ${IM_DLLS} DESTINATION "${CURRENT_PACKAGES_DIR}/bin")
  file(COPY ${IM_DLLS} DESTINATION "${CURRENT_PACKAGES_DIR}/debug/bin")
endif()

# License.
if(EXISTS "${APP_DIR}/License.txt")
  file(INSTALL "${APP_DIR}/License.txt"
       DESTINATION "${CURRENT_PACKAGES_DIR}/share/${PORT}"
       RENAME copyright)
elseif(EXISTS "${APP_DIR}/LICENSE.txt")
  file(INSTALL "${APP_DIR}/LICENSE.txt"
       DESTINATION "${CURRENT_PACKAGES_DIR}/share/${PORT}"
       RENAME copyright)
else()
  file(WRITE "${CURRENT_PACKAGES_DIR}/share/${PORT}/copyright"
       "ImageMagick license: see https://imagemagick.org/script/license.php\n")
endif()

file(INSTALL "${CMAKE_CURRENT_LIST_DIR}/usage"
     DESTINATION "${CURRENT_PACKAGES_DIR}/share/${PORT}")
