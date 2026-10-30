cmake_minimum_required(VERSION 3.15)

# ---------------------------------------------------------------------------
# `ari` — run an Ariadne (.ari) document in the terminal (cvc::ariadne::FtxuiBackend). Dual-mode,
# like src/cvc-cli:
#
#   In-tree    (add_subdirectory(ari) from the libcvc build): the cvc target already exists.
#   Standalone (the `ari` cvcpkg recipe builds ONLY this dir): find the installed libcvc SDK via
#              find_package(cvc CONFIG); the recipe points CMAKE_PREFIX_PATH at the deps prefix.
#
# Without FTXUI or yaml-cpp in that libcvc it still builds, and says what is missing when run.
# ---------------------------------------------------------------------------
if(CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
  project(ari LANGUAGES CXX)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  find_package(cvc CONFIG REQUIRED)
else()
  # In-tree: re-import the Boost targets cvc links PUBLIC so its transitive link evaluates in
  # this directory scope (src/arisrv/CMakeLists.txt explains why).
  if(NOT TARGET Boost::filesystem)
    find_package(Boost REQUIRED COMPONENTS thread date_time regex filesystem program_options
                 OPTIONAL_COMPONENTS system chrono)
  endif()
endif()

add_executable(ari main.cpp)
if(TARGET cvc::cvc)
  target_link_libraries(ari PRIVATE cvc::cvc)
else()
  target_link_libraries(ari PRIVATE cvc)
endif()
# Installed bin/ari finds libcvc and its dependencies in ../lib (CMake/CvcInstallRpath.cmake).
include("${CMAKE_CURRENT_LIST_DIR}/../../CMake/CvcInstallRpath.cmake")
cvc_set_install_rpath(ari bin)
install(TARGETS ari RUNTIME DESTINATION bin COMPONENT ari)

# The network flags end to end (ari_net_test.cpp): the built ari, run as a launcher would, against
# localhost servers. In-tree only, beside the GTest of the libcvc build, and not in a browser build.
if(CVC_BUILD_TESTS AND NOT EMSCRIPTEN AND NOT CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
  add_executable(ari_net_test ari_net_test.cpp)
  target_link_libraries(ari_net_test PRIVATE cvc GTest::gtest GTest::gtest_main)
  add_dependencies(ari_net_test ari)
  # As src/cvc-cli/CMakeLists.txt does (its note explains): GoogleTest in this directory's scope,
  # discovery at test time on macOS and with a generous timeout on Windows.
  include(GoogleTest)
  if(APPLE)
    macro(gtest_discover_tests)
      _gtest_discover_tests(${ARGV} DISCOVERY_MODE PRE_TEST DISCOVERY_TIMEOUT 120)
    endmacro()
  elseif(WIN32)
    macro(gtest_discover_tests)
      _gtest_discover_tests(${ARGV} DISCOVERY_TIMEOUT 120)
    endmacro()
  endif()
  gtest_discover_tests(ari_net_test PROPERTIES ENVIRONMENT "ARI_BINARY=$<TARGET_FILE:ari>")
endif()

# A document as each terminal backend really draws it: `ari run` in a 100x30 tmux pane, the text
# inside its window's borders compared with the rows expected (tests/run_in_tmux_test.sh). `ari
# snapshot` always lays out with FTXUI, so this is what checks the tvision desktop itself, and the
# live FTXUI screen. json_feed.ari's rows include accents, CJK and an emoji, so a wide glyph that
# took the wrong number of columns moves its row's right border. Linux, where tmux is; a backend
# this ari lacks is skipped (77).
if(CVC_BUILD_TESTS AND NOT EMSCRIPTEN AND CMAKE_SYSTEM_NAME STREQUAL "Linux"
   AND NOT CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
  find_program(CVC_TMUX_EXECUTABLE tmux)
  if(CVC_TMUX_EXECUTABLE)
    foreach(_backend ftxui tvision)
      add_test(NAME ari_run_json_feed_${_backend}
        COMMAND bash "${CMAKE_CURRENT_SOURCE_DIR}/tests/run_in_tmux_test.sh" $<TARGET_FILE:ari>
                "${CMAKE_CURRENT_SOURCE_DIR}/../../examples/ariadne/documents/json_feed.ari"
                ${_backend} "${CMAKE_CURRENT_SOURCE_DIR}/tests/json_feed.rows")
      set_tests_properties(ari_run_json_feed_${_backend} PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 60)
    endforeach()
  endif()
endif()

if(EMSCRIPTEN)
  # The browser build: ari.js/ari.wasm plus ari.html, an xterm.js page wired to the module's
  # stdin/stdout, which is all FTXUI's Emscripten screen talks to. A page has no argv, so one
  # document is baked in as the default (CVC_ARI_WASM_DOC) with the shared component library
  # embedded in MEMFS at /components — the same shape as the cvcGL ariadne_hello demos.
  # FTXUI's run loop yields to the browser with emscripten_sleep, hence -sASYNCIFY (which
  # cvc::net's fetch backend needs as well, with -sFETCH). Single-threaded: it runs on the
  # page's main thread and needs no COOP/COEP headers. EXIT_RUNTIME so a document that quits
  # reaches the page's onExit.
  set(CVC_ARI_WASM_DOC "${CMAKE_CURRENT_SOURCE_DIR}/../../examples/ariadne/terminal/terminal_tour.ari"
      CACHE FILEPATH "The .ari document the ari browser build runs")
  get_filename_component(_ari_doc_name "${CVC_ARI_WASM_DOC}" NAME)
  # The page goes next to <target>.js, with its names filled in by a plain replace:
  # configure_file(@ONLY) would also eat the "@xterm/xterm@" in its CDN URLs.
  set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/web/ari.html")
  file(READ "${CMAKE_CURRENT_SOURCE_DIR}/web/ari.html" _ari_html_in)
  function(ari_wasm_page target label hint)
    target_compile_definitions(${target} PRIVATE CVC_ARIADNE_WASM_DOC="/${_ari_doc_name}")
    target_link_options(${target} PRIVATE
      "--embed-file=${CVC_ARI_WASM_DOC}@/${_ari_doc_name}"
      "--embed-file=${CMAKE_CURRENT_SOURCE_DIR}/../cvc/ariadne/components@/components"
      -sASYNCIFY=1
      -sASYNCIFY_STACK_SIZE=131072
      -sALLOW_MEMORY_GROWTH=1
      -sSTACK_SIZE=2097152
      -sFETCH=1
      -sEXIT_RUNTIME=1
      # The page reports the xterm.js size through FTXUI's resize hook, and tvision's.
      ${ARGN})
    set_property(TARGET ${target} APPEND PROPERTY LINK_DEPENDS "${CVC_ARI_WASM_DOC}")
    set(_html "${_ari_html_in}")
    string(REPLACE "@ARI_WASM_DOC_NAME@" "${_ari_doc_name}" _html "${_html}")
    string(REPLACE "@ARI_WASM_MODULE@" "${target}" _html "${_html}")
    string(REPLACE "@ARI_WASM_LABEL@" "${label}" _html "${_html}")
    string(REPLACE "@ARI_WASM_HINT@" "${hint}" _html "${_html}")
    file(GENERATE OUTPUT "$<TARGET_FILE_DIR:${target}>/${target}.html" CONTENT "${_html}")
  endfunction()
  ari_wasm_page(ari "ari"
    "Tab / Enter / arrows · mouse · F10 menu · Ctrl-C asks, twice quits"
    -sEXPORTED_FUNCTIONS=_main,_ftxui_on_resize)
  # ari_tvision: the same runner and document on the tvision desktop (movable, overlapping
  # windows, a menu bar and a status line), when libcvc has the tvision backend: on wasm that is
  # cvcpkg's tvision with its browser console (src/cvc/CMakeLists.txt). Out of ALL like the gallery
  # demos; examples/web/build-wasm-demo.sh builds it by name.
  if(CVC_ARIADNE_TVISION_ENABLED)
    add_executable(ari_tvision EXCLUDE_FROM_ALL main.cpp)
    if(TARGET cvc::cvc)
      target_link_libraries(ari_tvision PRIVATE cvc::cvc)
    else()
      target_link_libraries(ari_tvision PRIVATE cvc)
    endif()
    target_compile_definitions(ari_tvision PRIVATE CVC_ARI_WASM_BACKEND="tvision")
    ari_wasm_page(ari_tvision "ari · tvision"
      "drag a window by its title · F10 menu · F6 next window · Alt-X quits"
      -sEXPORTED_FUNCTIONS=_main,_ftxui_on_resize,_tvision_set_screen_size)
  endif()
endif()
