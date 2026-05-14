# vcpkg overlay triplet: x64-windows
#
# Overrides the upstream x64-windows triplet so that we can restrict
# vcpkg to building only one of {Debug,Release} per CI job, instead of
# building both configurations (the upstream default). Each Windows CI
# job consumes only its matching matrix.build_type, so the other half
# is wasted ~equal-time work.
#
# Activated by setting the VCPKG_FORCE_BUILD_TYPE environment variable
# to "release" or "debug" before invoking vcpkg / cmake. When the env
# var is unset or empty, behavior matches the upstream default (both
# debug and release built).
set(VCPKG_TARGET_ARCHITECTURE x64)
set(VCPKG_CRT_LINKAGE dynamic)
set(VCPKG_LIBRARY_LINKAGE dynamic)
if(DEFINED ENV{VCPKG_FORCE_BUILD_TYPE} AND NOT "$ENV{VCPKG_FORCE_BUILD_TYPE}" STREQUAL "")
    set(VCPKG_BUILD_TYPE "$ENV{VCPKG_FORCE_BUILD_TYPE}")
endif()
