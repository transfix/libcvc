cmake_minimum_required(VERSION 3.15)

find_package(Boost 1.58 REQUIRED COMPONENTS program_options filesystem)

# ImageMagick support (enabled by default)
option(CVC_ENABLE_IMAGEMAGICK "Enable ImageMagick support for image I/O" ON)
if(CVC_ENABLE_IMAGEMAGICK)
  find_package(ImageMagick COMPONENTS Magick++ MagickCore)
  if(ImageMagick_FOUND)
    message(STATUS "ImageMagick found: ${ImageMagick_INCLUDE_DIRS}")
  else()
    message(STATUS "ImageMagick not found - disabling image I/O commands")
    set(CVC_ENABLE_IMAGEMAGICK OFF)
  endif()
endif()

# FFTW support (enabled by default)
option(CVC_ENABLE_FFTW "Enable FFTW support for filtered back-projection" ON)
if(CVC_ENABLE_FFTW)
  find_package(PkgConfig QUIET)
  if(PkgConfig_FOUND)
    pkg_check_modules(FFTW3 QUIET fftw3)
  endif()
  if(NOT FFTW3_FOUND)
    find_path(FFTW3_INCLUDE_DIRS fftw3.h)
    find_library(FFTW3_LIBRARIES NAMES fftw3)
    if(FFTW3_INCLUDE_DIRS AND FFTW3_LIBRARIES)
      set(FFTW3_FOUND TRUE)
    endif()
  endif()
  if(FFTW3_FOUND)
    message(STATUS "FFTW3 found: ${FFTW3_LIBRARIES}")
  else()
    message(STATUS "FFTW3 not found - disabling filtered back-projection")
    set(CVC_ENABLE_FFTW OFF)
  endif()
endif()

add_executable(volutils main.cpp)
target_link_libraries(volutils PRIVATE cvc Boost::program_options Boost::filesystem)
target_include_directories(volutils PRIVATE ${CMAKE_SOURCE_DIR}/inc)

if(CVC_ENABLE_IMAGEMAGICK)
  target_compile_definitions(volutils PRIVATE CVC_ENABLE_IMAGEMAGICK)
  target_include_directories(volutils PRIVATE ${ImageMagick_INCLUDE_DIRS})
  target_link_libraries(volutils PRIVATE ${ImageMagick_LIBRARIES})
  # ImageMagick 7 needs MAGICKCORE_QUANTUM_DEPTH and MAGICKCORE_HDRI_ENABLE
  target_compile_definitions(volutils PRIVATE
    MAGICKCORE_QUANTUM_DEPTH=16
    MAGICKCORE_HDRI_ENABLE=0)
  # Also add to cvc library for volume_ops.cpp. Wrap each include dir
  # in $<BUILD_INTERFACE:...> so a bundled toolchain doesn't leak a
  # build-dir path into the installed cvc target's
  # INTERFACE_INCLUDE_DIRECTORIES.
  target_compile_definitions(cvc PUBLIC CVC_ENABLE_IMAGEMAGICK)
  foreach(_im_inc IN LISTS ImageMagick_INCLUDE_DIRS)
    target_include_directories(cvc PUBLIC "$<BUILD_INTERFACE:${_im_inc}>")
  endforeach()
  target_link_libraries(cvc PUBLIC ${ImageMagick_LIBRARIES})
endif()

if(CVC_ENABLE_FFTW)
  target_compile_definitions(volutils PRIVATE CVC_ENABLE_FFTW)
  target_include_directories(volutils PRIVATE ${FFTW3_INCLUDE_DIRS})
  target_link_libraries(volutils PRIVATE ${FFTW3_LIBRARIES})
  target_compile_definitions(cvc PUBLIC CVC_ENABLE_FFTW)
  foreach(_fftw_inc IN LISTS FFTW3_INCLUDE_DIRS)
    target_include_directories(cvc PUBLIC "$<BUILD_INTERFACE:${_fftw_inc}>")
  endforeach()
  target_link_libraries(cvc PUBLIC ${FFTW3_LIBRARIES})
endif()

install(TARGETS volutils
  RUNTIME DESTINATION bin
)

# Windows: bundle runtime DLLs (Boost, FFTW, ImageMagick, etc.)
if(WIN32)
  install(FILES $<TARGET_RUNTIME_DLLS:volutils> TYPE BIN)
endif()
