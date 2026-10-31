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

# The network flags end to end (ari_net_test.cpp): the built ari, run as a launcher would, against
# localhost servers (ari_run.h runs it). In-tree only, beside the GTest of the libcvc build, and not in a browser build.
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
  # (quit code), assert and `ari test` end to end (ari_test_command_test.cpp).
  add_executable(ari_test_command_test ari_test_command_test.cpp)
  target_link_libraries(ari_test_command_test PRIVATE cvc GTest::gtest GTest::gtest_main)
  add_dependencies(ari_test_command_test ari)
  gtest_discover_tests(ari_test_command_test
                       PROPERTIES ENVIRONMENT "ARI_BINARY=$<TARGET_FILE:ari>")
endif()

# A spinner turns by the host's frame clock while the document's program is parked: `ari snapshot`
# frame 1 against frame 3, then `ari run` in tmux under each terminal backend this ari has
# (tests/spinner_test.sh). Where bash is; without tmux only the snapshots are checked.
if(CVC_BUILD_TESTS AND NOT EMSCRIPTEN AND NOT WIN32 AND NOT CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
  add_test(NAME ari_spinner
    COMMAND bash "${CMAKE_CURRENT_SOURCE_DIR}/tests/spinner_test.sh" $<TARGET_FILE:ari>
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/spinner.ari")
  set_tests_properties(ari_spinner PROPERTIES TIMEOUT 60)
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
      # Windows that close and open again in one run (tests/reopen_refit.ari, driven by its keys):
      # each fits the rows it gained while closed, and on tvision one the user resized (Ctrl-F5)
      # comes back at their size.
      add_test(NAME ari_run_reopen_refit_${_backend}
        COMMAND bash "${CMAKE_CURRENT_SOURCE_DIR}/tests/reopen_refit_test.sh" $<TARGET_FILE:ari>
                "${CMAKE_CURRENT_SOURCE_DIR}/tests/reopen_refit.ari" ${_backend})
      set_tests_properties(ari_run_reopen_refit_${_backend} PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 60)
    endforeach()
  endif()
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
  # xterm.js and its fit and WebGL addons, served beside the page (in xterm/) rather than from a CDN,
  # so the page needs no other origin and runs under a strict Content-Security-Policy. The first
  # found of:
  #   CVC_XTERM_DIR  a directory holding xterm.js, xterm.css, addon-fit.js, addon-webgl.js (and
  #                  LICENSE* files), for a build without the network
  #   the prefix     cvcpkg's xterm-js, xterm-js-addon-fit and xterm-js-addon-webgl packages, in
  #                  share/xterm.js/ (licences in share/doc/xterm-js*/) under CMAKE_PREFIX_PATH or
  #                  CMAKE_FIND_ROOT_PATH
  #   a download     the same pinned npm packages, checked by SHA-256, fetched once into the build
  #                  tree
  # All three are xterm.js 5.5.0, addon-fit 0.10.0 and addon-webgl 0.18.0: the files whose SRI
  # hashes the page pinned when it loaded them from jsDelivr.
  set(CVC_XTERM_DIR "" CACHE PATH
      "xterm.js 5.5.0 + addon-fit 0.10.0 + addon-webgl 0.18.0 for ari's page (empty: the prefix, else download)")
  set(_ari_xterm "${CMAKE_CURRENT_BINARY_DIR}/xterm")
  set(_ari_xterm_files xterm.js xterm.css addon-fit.js addon-webgl.js)
  set(_ari_xterm_from "${CVC_XTERM_DIR}")
  set(_ari_xterm_licenses)
  if(_ari_xterm_from)
    file(GLOB _ari_xterm_licenses "${CVC_XTERM_DIR}/LICENSE*")
  else()
    foreach(_p IN LISTS CMAKE_PREFIX_PATH CMAKE_FIND_ROOT_PATH)
      set(_ok TRUE)
      foreach(_f IN LISTS _ari_xterm_files)
        if(NOT EXISTS "${_p}/share/xterm.js/${_f}")
          set(_ok FALSE)
        endif()
      endforeach()
      if(_ok)
        set(_ari_xterm_from "${_p}/share/xterm.js")
        # share/doc/xterm-js/LICENSE -> LICENSE.xterm, .../xterm-js-addon-fit/LICENSE -> LICENSE.addon-fit
        file(GLOB _docs LIST_DIRECTORIES true "${_p}/share/doc/xterm-js*")
        foreach(_d IN LISTS _docs)
          get_filename_component(_n "${_d}" NAME)
          string(REGEX REPLACE "^xterm-js-?" "" _n "${_n}")
          if(_n STREQUAL "")
            set(_n xterm)
          endif()
          if(EXISTS "${_d}/LICENSE")
            configure_file("${_d}/LICENSE" "${_ari_xterm}/LICENSE.${_n}" COPYONLY)
          endif()
        endforeach()
        break()
      endif()
    endforeach()
  endif()
  if(_ari_xterm_from)
    foreach(_f IN LISTS _ari_xterm_files)
      configure_file("${_ari_xterm_from}/${_f}" "${_ari_xterm}/${_f}" COPYONLY)
    endforeach()
    if(_ari_xterm_licenses)
      file(COPY ${_ari_xterm_licenses} DESTINATION "${_ari_xterm}")
    endif()
    message(STATUS "ari: xterm.js for the browser page from ${_ari_xterm_from}")
  else()
    # name | version | tarball SHA-256 | files wanted from its package/ (-> the name in xterm/)
    set(_ari_xterm_pkgs
        "xterm|5.5.0|bd954fa721872170188cc5d7e83e88db3c83c9a18a4e8d24c2783d26491f59d2|lib/xterm.js>xterm.js,css/xterm.css>xterm.css,LICENSE>LICENSE.xterm"
        "addon-fit|0.10.0|917ac44972453d5eed52edc1e50260c76398ce48cf2290c2e60671102bba0b33|lib/addon-fit.js>addon-fit.js,LICENSE>LICENSE.addon-fit"
        "addon-webgl|0.18.0|6229e5c0b66a9de0e155602e90586605c53b91d6af657a6cbe2b8377d6019370|lib/addon-webgl.js>addon-webgl.js,LICENSE>LICENSE.addon-webgl")
    foreach(_pkg IN LISTS _ari_xterm_pkgs)
      string(REPLACE "|" ";" _pkg "${_pkg}")
      list(GET _pkg 0 _name)
      list(GET _pkg 1 _ver)
      list(GET _pkg 2 _sha)
      list(GET _pkg 3 _files)
      set(_tgz "${CMAKE_CURRENT_BINARY_DIR}/_xterm-dl/${_name}-${_ver}.tgz")
      set(_unpacked "${CMAKE_CURRENT_BINARY_DIR}/_xterm-dl/${_name}-${_ver}")
      if(NOT EXISTS "${_unpacked}/package")
        file(DOWNLOAD "https://registry.npmjs.org/@xterm/${_name}/-/${_name}-${_ver}.tgz" "${_tgz}"
             EXPECTED_HASH SHA256=${_sha} TLS_VERIFY ON STATUS _st)
        list(GET _st 0 _code)
        if(NOT _code EQUAL 0)
          message(FATAL_ERROR "ari: could not download @xterm/${_name} ${_ver} for ari's page (${_st}); "
                              "install cvcpkg's xterm-js packages in the prefix, or set CVC_XTERM_DIR")
        endif()
        file(MAKE_DIRECTORY "${_unpacked}")
        execute_process(COMMAND "${CMAKE_COMMAND}" -E tar xzf "${_tgz}"
                        WORKING_DIRECTORY "${_unpacked}" RESULT_VARIABLE _rc)
        if(NOT _rc EQUAL 0)
          message(FATAL_ERROR "ari: could not unpack ${_tgz}")
        endif()
      endif()
      string(REPLACE "," ";" _files "${_files}")
      foreach(_map IN LISTS _files)
        string(REPLACE ">" ";" _map "${_map}")
        list(GET _map 0 _from)
        list(GET _map 1 _to)
        configure_file("${_unpacked}/package/${_from}" "${_ari_xterm}/${_to}" COPYONLY)
      endforeach()
    endforeach()
    message(STATUS "ari: xterm.js for the browser page downloaded from registry.npmjs.org")
  endif()
  # The page goes next to <target>.js, with its names filled in by a plain replace (it has no
  # configure_file-style variables). Its loader and style sheet, and xterm/, go beside it; the
  # gallery's build-pages.py copies what <target>.page-files lists along with the page.
  set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/web/ari.html")
  file(READ "${CMAKE_CURRENT_SOURCE_DIR}/web/ari.html" _ari_html_in)
  # What every page shares, beside the modules: the loader, the style sheet and xterm/. The copy is a
  # target that always runs (cheap), so xterm/ is there whenever a page is built, even when its
  # module is up to date. (The directory is named outright: a $<TARGET_FILE_DIR> of a page here
  # would make the copy depend on the page, a cycle.)
  if(CMAKE_RUNTIME_OUTPUT_DIRECTORY)
    set(_ari_bin "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")
  else()
    set(_ari_bin "${CMAKE_CURRENT_BINARY_DIR}")
  endif()
  foreach(_f ari-web.js ari-web.css)
    file(GENERATE OUTPUT "${_ari_bin}/${_f}" INPUT "${CMAKE_CURRENT_SOURCE_DIR}/web/${_f}")
  endforeach()
  add_custom_target(ari_web_assets
    COMMAND "${CMAKE_COMMAND}" -E copy_directory "${_ari_xterm}" "${_ari_bin}/xterm"
    VERBATIM)
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
    add_dependencies(${target} ari_web_assets)
    file(GENERATE OUTPUT "$<TARGET_FILE_DIR:${target}>/${target}.page-files"
         CONTENT "ari-web.js\nari-web.css\nxterm\n")
  endfunction()
  ari_wasm_page(ari "ari"
    "Tab / Enter / arrows · mouse · F10 menu · Ctrl-C asks, twice quits"
    -sEXPORTED_FUNCTIONS=_main,_ftxui_on_resize)
  # CI's network checks (ci.yml, "ari terminal backend (wasm, node)") run tests/net_probe.ari and
  # tests/l4_frames.ari against tests/net_probe_server.py under node: the documents go into MEMFS
  # at /tests. Off by default, so the published page does not carry them.
  option(CVC_ARI_WASM_TEST_DOCS "Embed src/ari/tests at /tests in the wasm ari (CI)" OFF)
  if(CVC_ARI_WASM_TEST_DOCS)
    target_link_options(ari PRIVATE "--embed-file=${CMAKE_CURRENT_SOURCE_DIR}/tests@/tests")
    set_property(TARGET ari APPEND PROPERTY LINK_DEPENDS
      "${CMAKE_CURRENT_SOURCE_DIR}/tests/net_probe.ari"
      "${CMAKE_CURRENT_SOURCE_DIR}/tests/l4_frames.ari")
  endif()
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
