# cvc_set_install_rpath(<target> <install-dir>)
#
# Give an installed binary a loader-relative RUNPATH (Linux, BSD) or LC_RPATH
# (macOS) that points at the install prefix's library directory, so it finds
# libcvc and every dependency installed beside it -- a cvcpkg prefix, an
# extracted release SDK -- with no LD_LIBRARY_PATH / DYLD_LIBRARY_PATH and no
# cvcpkg `activate`. RUNPATH is not inherited: an executable's $ORIGIN/../lib
# finds libcvc.so, but libcvc.so's own dependencies (yaml-cpp, curl, Boost, ...)
# are only found through libcvc.so's RUNPATH, so every binary that loads
# something from lib/ needs its own.
#
# <install-dir> is where the target is installed, relative to the prefix: lib,
# bin, lib/python3.11/site-packages, ... The RUNPATH is that directory's path
# to CMAKE_INSTALL_LIBDIR from $ORIGIN / @loader_path. The build tree keeps
# CMake's absolute build RPATH; this only changes what `cmake --install` writes.
#
# A CMAKE_INSTALL_RPATH given by the caller wins (cvcpkg recipes may pass their
# own). Windows has no rpath, and wasm links statically.
include_guard(GLOBAL)
include(GNUInstallDirs)

function(cvc_set_install_rpath target install_dir)
  if(WIN32 OR EMSCRIPTEN OR CMAKE_INSTALL_RPATH OR IS_ABSOLUTE "${CMAKE_INSTALL_LIBDIR}")
    return()
  endif()
  if(APPLE)
    set(_origin "@loader_path")
  else()
    set(_origin "$ORIGIN")
  endif()
  file(RELATIVE_PATH _rel "/${install_dir}" "/${CMAKE_INSTALL_LIBDIR}")
  string(REGEX REPLACE "/+$" "" _rel "${_rel}")
  if(_rel STREQUAL "")
    set(_rpath "${_origin}")
  else()
    set(_rpath "${_origin}/${_rel}")
  endif()
  set_target_properties(${target} PROPERTIES INSTALL_RPATH "${_rpath}")
endfunction()
