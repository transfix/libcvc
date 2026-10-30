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
    find_package(Boost REQUIRED COMPONENTS thread date_time regex filesystem
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

# ctest coverage of the binary, in-tree only. A document set as an app installs one
# (share/<app>/main.ari), run from another working directory: its relative imports, out of its own
# directory and its parent's, resolve against the importing file's directory, as in arigl
# (arigl_docset runs the same set). A libcvc without FTXUI or yaml-cpp skips rather than fails.
if(NOT CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR AND NOT EMSCRIPTEN)
  add_test(NAME ari_docset
           COMMAND ari snapshot --width 90 --height 12 --component-path
                   "${CMAKE_CURRENT_SOURCE_DIR}/../cvc/ariadne"
                   "${CMAKE_CURRENT_SOURCE_DIR}/../cvc/tests/data/ariadne_docset/app/main.ari"
           WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}")
  set_tests_properties(ari_docset PROPERTIES
                       PASS_REGULAR_EXPRESSION "from shared/note.ari"
                       FAIL_REGULAR_EXPRESSION "could not be resolved;unknown unit")
  # A URL names a document for the browser page; natively ari says so.
  add_test(NAME ari_url_document COMMAND ari check https://example.invalid/app/main.ari)
  set_tests_properties(ari_url_document PROPERTIES
                       PASS_REGULAR_EXPRESSION "ari runs a document from a path")
  set_tests_properties(ari_docset ari_url_document PROPERTIES
                       SKIP_REGULAR_EXPRESSION "built without FTXUI;built without yaml-cpp")
endif()

if(EMSCRIPTEN)
  # The browser build: ari.js/ari.wasm plus ari.html, an xterm.js page wired to the module's
  # stdin/stdout, which is all FTXUI's Emscripten screen talks to. Opened on ?src= (or with a page
  # config naming a document), the page runs that document by URL: ari fetches it and the files
  # it imports into MEMFS at their URL paths (cvc/ariadne/web_document.h). Otherwise it runs the
  # one baked in (CVC_ARI_WASM_DOC). The shared component library is embedded where an install
  # keeps it (<prefix>/share/libcvc/ariadne), so cvc://components/... resolves with no download.
  # FTXUI's run loop yields to the browser with emscripten_sleep, hence -sASYNCIFY (which
  # cvc::net's fetch backend needs as well, with -sFETCH). Single-threaded: it runs on the
  # page's main thread and needs no COOP/COEP headers. EXIT_RUNTIME so a document that quits
  # reaches the page's onExit.
  set(CVC_ARI_WASM_DOC "${CMAKE_CURRENT_SOURCE_DIR}/../../examples/ariadne/terminal/terminal_tour.ari"
      CACHE FILEPATH "The .ari document the ari browser build runs")
  get_filename_component(_ari_doc_name "${CVC_ARI_WASM_DOC}" NAME)
  # The page's document when its URL names none: empty for the baked one (the gallery's page);
  # the arigl-web package sets main.ari, so the page is a shell for the documents beside it.
  set(CVC_ARI_WASM_DEFAULT_SRC "" CACHE STRING
      "The document ari.html runs when its URL has no ?src= (relative to the page; empty: the baked one)")
  # The page goes next to <target>.js, with its names filled in by a plain replace:
  # configure_file(@ONLY) would also eat the "@xterm/xterm@" in its CDN URLs.
  set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/web/ari.html")
  file(READ "${CMAKE_CURRENT_SOURCE_DIR}/web/ari.html" _ari_html_in)
  function(ari_wasm_page target label hint)
    target_compile_definitions(${target} PRIVATE CVC_ARIADNE_WASM_DOC="/${_ari_doc_name}"
                                                 CVC_ARI_COMPONENT_DIR="/usr/share/libcvc/ariadne")
    target_link_options(${target} PRIVATE
      "--embed-file=${CVC_ARI_WASM_DOC}@/${_ari_doc_name}"
      "--embed-file=${CMAKE_CURRENT_SOURCE_DIR}/../cvc/ariadne/components@/usr/share/libcvc/ariadne/components"
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
    string(REPLACE "@ARI_WASM_DEFAULT_SRC@" "${CVC_ARI_WASM_DEFAULT_SRC}" _html "${_html}")
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
