cmake_minimum_required(VERSION 3.15)

# ---------------------------------------------------------------------------
# cvcMedia — libcvc's GL-free media library (docs/roadmap/ARIADNE_MEDIA.md §5). It holds the SDL3
# audio output that used to live in cvcGL (cvc::media::playback, cvc/media/playback/*.h), and is
# where the decoders behind Ariadne's video: and audio: widgets go. It links cvc and, where found,
# SDL3 for its AUDIO subsystem only: no GL, VTK, ImGui or SDL video, so a terminal host plays sound
# with no window or display (ari-media). Without SDL3 it still builds: open_audio_output() is then
# always empty, the graceful absence cvcGL had.
#
# Like cvcGL it is its own library with its own export set and package (find_package(cvcMedia)),
# so the cvc package's single-target contract (cvc::cvc) is untouched and a lean consumer of libcvc
# never links SDL. Dual-mode:
#   In-tree    (src/CMakeLists.txt adds it before ari, and cvcGL links it).
#   Standalone (the ari-media cvcpkg recipe builds it beside src/ari against an installed libcvc,
#              statically, so bin/ari-media carries it and nothing more is installed).
# ---------------------------------------------------------------------------
if(CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
  project(cvcMedia LANGUAGES C CXX)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(NOT TARGET cvc AND NOT TARGET cvc::cvc)
  find_package(cvc CONFIG REQUIRED)
endif()

get_filename_component(_CVCMEDIA_INC "${CMAKE_CURRENT_SOURCE_DIR}/../../inc" ABSOLUTE)

add_library(cvcMedia playback/audio_output.cpp playback/audio_stream_player.cpp
                     player/ffmpeg_player.cpp)
add_library(cvc::cvcMedia ALIAS cvcMedia)
set_target_properties(cvcMedia PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(cvcMedia PUBLIC $<BUILD_INTERFACE:${_CVCMEDIA_INC}>
                                           $<INSTALL_INTERFACE:include>)
if(TARGET cvc::cvc)
  target_link_libraries(cvcMedia PUBLIC cvc::cvc)
else()
  target_link_libraries(cvcMedia PUBLIC cvc)
endif()
if(MSVC)
  set_target_properties(cvcMedia PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS ON)
endif()

# SDL3 (optional): its audio subsystem only. The same SDL3::SDL3 cvcGL links for its window and
# input, so a process with both (arigl) has one SDL.
set(CVCMEDIA_SDL_DEP "")
find_package(SDL3 CONFIG QUIET)
if(TARGET SDL3::SDL3)
  target_link_libraries(cvcMedia PUBLIC SDL3::SDL3)
  target_compile_definitions(cvcMedia PRIVATE CVC_MEDIA_SDL=1)
  set(CVCMEDIA_SDL_DEP "find_dependency(SDL3 CONFIG)")
  message(STATUS "cvcMedia: SDL3 audio output ENABLED (SDL3::SDL3, audio subsystem only)")
else()
  message(STATUS "cvcMedia: SDL3 not found -- no audio output (open_audio_output is empty)")
endif()

# FFmpeg (optional): the decoders behind Ariadne's video: and audio: widgets (cvc/media/player.h).
# Linked as SHARED libraries only (cvcpkg's ffmpeg-lgpl: LGPL-2.1, decoders only), so the LGPL is
# kept by dynamic linking. Found through pkg-config (FFmpeg's own .pc files), else by name. Without
# it the player is absent and the widgets say `unsupported`.
set(CVCMEDIA_FFMPEG_DEP "")
set(_ffmpeg_libs "")
# Never in a browser build: the pages play through the browser's own <video> / <audio>
# (media_browser.cpp), and a cross build's pkg-config could find the host's FFmpeg.
if(NOT EMSCRIPTEN)
  find_package(PkgConfig QUIET)
endif()
if(PKG_CONFIG_FOUND AND NOT EMSCRIPTEN)
  pkg_check_modules(CVC_FFMPEG QUIET IMPORTED_TARGET libavformat libavcodec libswscale
                    libswresample libavutil)
  if(CVC_FFMPEG_FOUND)
    set(_ffmpeg_libs PkgConfig::CVC_FFMPEG)
  endif()
endif()
if(NOT _ffmpeg_libs AND NOT EMSCRIPTEN)
  find_path(CVC_FFMPEG_INCLUDE_DIR libavformat/avformat.h)
  set(_ffmpeg_all TRUE)
  foreach(_lib avformat avcodec swscale swresample avutil)
    find_library(CVC_FFMPEG_${_lib}_LIBRARY NAMES ${_lib})
    if(NOT CVC_FFMPEG_${_lib}_LIBRARY)
      set(_ffmpeg_all FALSE)
    endif()
  endforeach()
  if(CVC_FFMPEG_INCLUDE_DIR AND _ffmpeg_all)
    target_include_directories(cvcMedia PRIVATE "${CVC_FFMPEG_INCLUDE_DIR}")
    foreach(_lib avformat avcodec swscale swresample avutil)
      list(APPEND _ffmpeg_libs "${CVC_FFMPEG_${_lib}_LIBRARY}")
    endforeach()
  endif()
endif()
set(CVC_MEDIA_HAVE_FFMPEG OFF CACHE INTERNAL "cvcMedia was built with its FFmpeg player")
if(_ffmpeg_libs)
  target_link_libraries(cvcMedia PRIVATE ${_ffmpeg_libs})
  target_compile_definitions(cvcMedia PRIVATE CVC_MEDIA_FFMPEG=1)
  set(CVC_MEDIA_HAVE_FFMPEG ON CACHE INTERNAL "cvcMedia was built with its FFmpeg player")
  message(STATUS "cvcMedia: FFmpeg player ENABLED (${_ffmpeg_libs})")
else()
  message(STATUS "cvcMedia: FFmpeg not found -- no player for video: / audio: (they say unsupported)")
endif()

include(CMakePackageConfigHelpers)
include("${CMAKE_CURRENT_LIST_DIR}/../../CMake/CvcInstallRpath.cmake")
cvc_set_install_rpath(cvcMedia lib)
# Standalone (the ari-media recipe): linked into bin/ari-media, nothing to install of its own.
if(NOT CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR AND NOT CVC_MEDIA_NO_INSTALL)
  install(TARGETS cvcMedia EXPORT cvcMediaTargets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)
  install(DIRECTORY "${_CVCMEDIA_INC}/cvc/media/playback" DESTINATION include/cvc/media
          FILES_MATCHING PATTERN "*.h")
  install(FILES "${_CVCMEDIA_INC}/cvc/media/player.h" DESTINATION include/cvc/media)
  install(EXPORT cvcMediaTargets NAMESPACE cvc:: FILE cvcMediaTargets.cmake
          DESTINATION lib/cmake/cvcMedia)
  configure_package_config_file("${CMAKE_CURRENT_SOURCE_DIR}/cvcMediaConfig.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/cvcMediaConfig.cmake" INSTALL_DESTINATION lib/cmake/cvcMedia)
  write_basic_package_version_file("${CMAKE_CURRENT_BINARY_DIR}/cvcMediaConfigVersion.cmake"
    VERSION ${PROJECT_VERSION} COMPATIBILITY SameMajorVersion)
  install(FILES "${CMAKE_CURRENT_BINARY_DIR}/cvcMediaConfig.cmake"
                "${CMAKE_CURRENT_BINARY_DIR}/cvcMediaConfigVersion.cmake"
          DESTINATION lib/cmake/cvcMedia)
endif()

# A terminal program plays sound: SDL's audio subsystem with no video subsystem, no window and no
# display (SDL's dummy driver in CI). Linked to cvc and cvcMedia only, and the test checks it pulls
# in no GL.
if(CVC_BUILD_TESTS AND NOT EMSCRIPTEN AND NOT CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
  add_executable(cvcmedia_audio_output test/cvcmedia_audio_output.cpp)
  target_link_libraries(cvcmedia_audio_output PRIVATE cvcMedia)
  if(TARGET SDL3::SDL3)
    target_compile_definitions(cvcmedia_audio_output PRIVATE CVC_MEDIA_TEST_SDL=1)
  endif()
  add_test(NAME cvcmedia_audio_output COMMAND cvcmedia_audio_output)
  set_tests_properties(cvcmedia_audio_output PROPERTIES ENVIRONMENT
                       "SDL_AUDIO_DRIVER=dummy;SDL_VIDEO_DRIVER=dummy" TIMEOUT 60)
  # The FFmpeg player on a 2 s clip and a tone beside the ari tests (src/ari/tests/media).
  add_executable(cvcmedia_player test/cvcmedia_player.cpp)
  target_link_libraries(cvcmedia_player PRIVATE cvcMedia)
  target_compile_definitions(cvcmedia_player PRIVATE
    CVC_MEDIA_TEST_DATA="${CMAKE_CURRENT_SOURCE_DIR}/../ari/tests/media")
  add_test(NAME cvcmedia_player COMMAND cvcmedia_player)
  set_tests_properties(cvcmedia_player PROPERTIES TIMEOUT 60)
endif()
