# vcpkg overlay triplet: x64-windows-static
#
# See x64-windows.cmake for rationale. This is the static-CRT/static-lib
# variant used by the release.yml `link: static` matrix entries.
set(VCPKG_TARGET_ARCHITECTURE x64)
set(VCPKG_CRT_LINKAGE static)
set(VCPKG_LIBRARY_LINKAGE static)
if(DEFINED ENV{VCPKG_FORCE_BUILD_TYPE} AND NOT "$ENV{VCPKG_FORCE_BUILD_TYPE}" STREQUAL "")
    set(VCPKG_BUILD_TYPE "$ENV{VCPKG_FORCE_BUILD_TYPE}")
endif()
