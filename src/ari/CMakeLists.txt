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
  target_compile_definitions(ari PRIVATE CVC_ARIADNE_WASM_DOC="/${_ari_doc_name}")
  target_link_options(ari PRIVATE
    "--embed-file=${CVC_ARI_WASM_DOC}@/${_ari_doc_name}"
    "--embed-file=${CMAKE_CURRENT_SOURCE_DIR}/../cvc/ariadne/components@/components"
    -sASYNCIFY=1
    -sASYNCIFY_STACK_SIZE=131072
    -sALLOW_MEMORY_GROWTH=1
    -sSTACK_SIZE=2097152
    -sFETCH=1
    -sEXIT_RUNTIME=1
    # The page reports the xterm.js size through FTXUI's resize hook.
    -sEXPORTED_FUNCTIONS=_main,_ftxui_on_resize)
  set_property(TARGET ari APPEND PROPERTY LINK_DEPENDS "${CVC_ARI_WASM_DOC}")
  # The page goes next to ari.js, with the document name filled in by a plain replace:
  # configure_file(@ONLY) would also eat the "@xterm/xterm@" in its CDN URLs.
  set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/web/ari.html")
  file(READ "${CMAKE_CURRENT_SOURCE_DIR}/web/ari.html" _ari_html)
  string(REPLACE "@ARI_WASM_DOC_NAME@" "${_ari_doc_name}" _ari_html "${_ari_html}")
  file(GENERATE OUTPUT "$<TARGET_FILE_DIR:ari>/ari.html" CONTENT "${_ari_html}")
endif()
