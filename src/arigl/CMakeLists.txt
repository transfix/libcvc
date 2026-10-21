cmake_minimum_required(VERSION 3.15)

# ---------------------------------------------------------------------------
# `arigl` — the GL Ariadne runtime (cvcGL: VTK + ImGui). Runs a .ari document in a real window, or
# with --terminal in the terminal with its scene rendered offscreen behind the widgets. `ari` is
# the GL-free core runtime; this is its cvcGL sibling. Dual-mode, like src/ari:
#
#   In-tree    (the top-level CMakeLists adds it after cvcGL, with CVC_BUILD_CVCGL and
#              CVC_BUILD_ARIGL on): the cvc and cvcGL targets already exist.
#   Standalone (an arigl cvcpkg recipe builds ONLY this dir): find the installed libcvc and cvcGL
#              SDKs via find_package(... CONFIG).
# ---------------------------------------------------------------------------
if(CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
  project(arigl LANGUAGES CXX)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  find_package(cvc CONFIG REQUIRED)
  find_package(cvcGL CONFIG REQUIRED)
endif()
# What the host itself draws with, beyond what cvcGL links: VTK's window/interactor and image IO.
find_package(VTK REQUIRED COMPONENTS RenderingUI IOImage)

add_executable(arigl main.cpp)
target_link_libraries(arigl PRIVATE cvc::cvcGL ${VTK_LIBRARIES})
vtk_module_autoinit(TARGETS arigl MODULES ${VTK_LIBRARIES})
# Installed bin/arigl finds libcvc, cvcGL and their dependencies in ../lib.
include("${CMAKE_CURRENT_LIST_DIR}/../../CMake/CvcInstallRpath.cmake")
cvc_set_install_rpath(arigl bin)
install(TARGETS arigl RUNTIME DESTINATION bin COMPONENT arigl)

# ctest coverage of the binary itself, in-tree only (it runs the shipped example documents). Each
# renders offscreen, so it takes the GL-context lock the cvcGL tests share; a libcvc without FTXUI
# (the terminal tests) or yaml-cpp skips rather than fails.
if(NOT CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
  set(_arigl_examples "${CMAKE_CURRENT_SOURCE_DIR}/../cvcGL/examples")
  set(_arigl_tour "${CMAKE_CURRENT_SOURCE_DIR}/../cvc/ariadne/examples/terminal_tour.ari")
  add_test(NAME arigl_terminal_snapshot
           COMMAND arigl snapshot --terminal --width 60 --height 16
                   "${_arigl_examples}/terminal_scene.ari")
  set_tests_properties(arigl_terminal_snapshot PROPERTIES PASS_REGULAR_EXPRESSION "Show mesh")
  # The document's own arguments, after its path, reach its init: through args.
  add_test(NAME arigl_document_args
           COMMAND arigl snapshot --terminal --width 100 --height 26 "${_arigl_tour}"
                   --agents 77 --name ci)
  set_tests_properties(arigl_document_args PROPERTIES PASS_REGULAR_EXPRESSION "agents: 77")
  # An argument the document's args: block does not accept fails with exit status 2.
  add_test(NAME arigl_rejects_bad_document_args
           COMMAND arigl snapshot --terminal "${_arigl_tour}" --agents many)
  set_tests_properties(arigl_rejects_bad_document_args PROPERTIES WILL_FAIL TRUE)
  # stream_view in the terminal: the document's synthetic camera reaches its window as an image,
  # not the "no frame" placeholder (40 frames at 20 a second give the producer time to deliver).
  add_test(NAME arigl_terminal_stream_view
           COMMAND arigl snapshot --terminal --frames 40 --width 90 --height 30
                   "${_arigl_examples}/conference_media.ari")
  set_tests_properties(arigl_terminal_stream_view PROPERTIES FAIL_REGULAR_EXPRESSION "no frame\\]")
  add_test(NAME arigl_window_snapshot
           COMMAND arigl snapshot --frames 2 --png "${CMAKE_CURRENT_BINARY_DIR}/arigl_window.png"
                   "${_arigl_examples}/hello.ari")
  add_test(NAME arigl_check COMMAND arigl check "${_arigl_examples}/hello.ari")
  set_tests_properties(
    arigl_terminal_snapshot arigl_document_args arigl_rejects_bad_document_args
    arigl_terminal_stream_view arigl_window_snapshot arigl_check
    PROPERTIES RESOURCE_LOCK cvcgl_gl_context
               SKIP_REGULAR_EXPRESSION "built without FTXUI;built without yaml-cpp")
endif()
