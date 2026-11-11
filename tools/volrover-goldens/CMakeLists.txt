# The legacy VolumeRover libraries, built from a transfix/volrover checkout at the pinned sha
# (volrover.sha), to make the ports' T1 goldens (README.md). Never part of libcvc's build: make_goldens.py
# configures this directory on its own. GPL code (the legacy tree's own CGAL, for the libraries
# that need it) may be built here and only here; nothing of it is linked into libcvc.
cmake_minimum_required(VERSION 3.16)
project(volrover_goldens CXX)

set(VOLROVER_DIR "" CACHE PATH "A transfix/volrover checkout at the sha in volrover.sha")
if(NOT EXISTS "${VOLROVER_DIR}/src/Segmentation/GenSeg/segment.cpp")
  message(FATAL_ERROR "VOLROVER_DIR (${VOLROVER_DIR}) is not a volrover checkout with GenSeg")
endif()

# Legacy code as it is: built with its warnings off, and never with -Werror.
function(volrover_legacy target)
  # The shim comes first, so <VolMagick/...> and <XmlRPC/...> resolve to it (shim/*/*.h say why).
  target_include_directories(${target} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/shim ${VOLROVER_DIR}/inc)
  if(WIN32)
    target_compile_definitions(${target} PRIVATE __WINDOWS__ _CRT_SECURE_NO_WARNINGS)
  endif()
  if(MSVC)
    target_compile_options(${target} PRIVATE /w)
  else()
    target_compile_options(${target} PRIVATE -w)
  endif()
endfunction()

# GenSeg (src/Segmentation/GenSeg): the fast marching and its rawiv reader and writer, as they are.
# general_segmentation.cpp is its XML-RPC entry, which genseg_driver.cpp stands in for.
add_executable(genseg_legacy genseg_driver.cpp
  ${VOLROVER_DIR}/src/Segmentation/GenSeg/segment.cpp
  ${VOLROVER_DIR}/src/Segmentation/GenSeg/inout.cpp)
volrover_legacy(genseg_legacy)
