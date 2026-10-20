# MSVC: compile each target's C/C++ sources in parallel (/MP).
#
# The Visual Studio generators give cl.exe all of a target's sources in one
# call, and without /MP cl compiles them one at a time. MSBuild's /m (what
# `cmake --build --parallel` passes) only builds separate projects side by
# side, and libcvc is one big target. On 2026-10-20 the v3.7.0 publish spent
# 50 min compiling the `cvc` target's ~300 files serially on the Windows CUDA
# runner, and 17 min on a hosted windows-2022 runner. Ninja already runs one
# cl per source, so /MP changes nothing there.
#
# CVC_MSVC_MP:
#   ON (default)  /MP: one cl process per logical CPU.
#   N             /MPN: at most N processes, to bound memory on a small or
#                 shared machine.
#   OFF           no /MP.
#
# C and C++ only, because nvcc does not take /MP. /MP implies /FS, so the
# target's shared PDB is written safely. The VS generator turns /MP[N] into
# the MultiProcessorCompilation and ProcessorNumber properties.
#
# Included from the top-level CMakeLists.txt and from src/cvcGL when it is
# built standalone.
if(NOT MSVC)
  return()
endif()

set(CVC_MSVC_MP ON CACHE STRING
    "MSVC parallel compile (/MP): ON, OFF, or a maximum number of cl processes")
if(CVC_MSVC_MP MATCHES "^[1-9][0-9]*$")
  add_compile_options("$<$<COMPILE_LANGUAGE:C,CXX>:/MP${CVC_MSVC_MP}>")
elseif(CVC_MSVC_MP)
  add_compile_options("$<$<COMPILE_LANGUAGE:C,CXX>:/MP>")
endif()
