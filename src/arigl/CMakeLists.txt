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
else()
  # In-tree: re-import the Boost targets cvc links PUBLIC (through cvcGL) so its transitive link
  # evaluates in this directory scope (src/arisrv/CMakeLists.txt explains why).
  if(NOT TARGET Boost::filesystem)
    find_package(Boost REQUIRED COMPONENTS thread date_time regex filesystem program_options
                 OPTIONAL_COMPONENTS system chrono)
  endif()
endif()
# What the host itself draws with, beyond what cvcGL links: VTK's window/interactor and image IO.
find_package(VTK REQUIRED COMPONENTS RenderingUI IOImage)

add_executable(arigl main.cpp gallery.cpp)
target_link_libraries(arigl PRIVATE cvc::cvcGL ${VTK_LIBRARIES})
vtk_module_autoinit(TARGETS arigl MODULES ${VTK_LIBRARIES})
# Installed bin/arigl finds libcvc, cvcGL and their dependencies in ../lib.
include("${CMAKE_CURRENT_LIST_DIR}/../../CMake/CvcInstallRpath.cmake")
cvc_set_install_rpath(arigl bin)
# A deep document's load and draw: an 8 MB stack on Windows, as on Linux (CMake/CvcHostStack.cmake).
include("${CMAKE_CURRENT_LIST_DIR}/../../CMake/CvcHostStack.cmake")
cvc_document_host_stack(arigl)
install(TARGETS arigl RUNTIME DESTINATION bin COMPONENT arigl)

# ariglx: the scripting host, the same arigl with an embedded Python (src/ariadne_python): a
# document's python: block, a Python program in place of the document (ariglx run app.py, whose
# pycvc.ari.Host() adopts the window), and ariglx python
# (docs/roadmap/ARIADNE_SCRIPTING_HOSTS.md). Its Python drives the window through pycvc_gl, so
# in-tree it is built with the bindings (CVC_BUILD_PYCVC); standalone (the ariglx cvcpkg recipe),
# with -DARIGL_WITH_PYTHON=ON. The stock `arigl` above stays Python-free.
option(ARIGL_WITH_PYTHON "Also build ariglx, arigl with an embedded Python" OFF)
if(NOT EMSCRIPTEN AND (CVC_BUILD_PYCVC OR ARIGL_WITH_PYTHON))
  include("${CMAKE_CURRENT_LIST_DIR}/../ariadne_python/ariadne_python.cmake")
  cvc_find_ariadne_python(ARIGL_WITH_PYTHON)
  if(CVC_ARIADNE_PYTHON_FOUND)
    add_executable(ariglx main.cpp gallery.cpp)
    cvc_document_host_stack(ariglx)
    target_link_libraries(ariglx PRIVATE cvc::cvcGL ${VTK_LIBRARIES})
    vtk_module_autoinit(TARGETS ariglx MODULES ${VTK_LIBRARIES})
    cvc_ariadne_python_host(ariglx)
    # ... with WebAssembly guests where wasmtime's C API is found (src/ariadne_wasm); with
    # -DARIGL_WITH_WASM=ON a missing one is a configure error.
    option(ARIGL_WITH_WASM "ariglx requires WebAssembly guests (wasmtime's C API)" OFF)
    include("${CMAKE_CURRENT_LIST_DIR}/../ariadne_wasm/ariadne_wasm.cmake")
    cvc_find_ariadne_wasm(ARIGL_WITH_WASM)
    if(CVC_ARIADNE_WASM_FOUND)
      cvc_ariadne_wasm_host(ariglx INSTALL)
    endif()
    cvc_set_install_rpath(ariglx bin)
    install(TARGETS ariglx RUNTIME DESTINATION bin COMPONENT ariglx)

    # ctest, in-tree: a document's python: block loads in a window (offscreen), and a Python program
    # in place of the document adopts the window, gives its document a function and renders it.
    if(CVC_BUILD_TESTS AND NOT CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
      set(_pyt "${CVC_ARIADNE_PYTHON_TESTS}")
      add_test(NAME ariglx_python_check
               COMMAND ariglx check ${CVC_ARIADNE_PYTHON_TEST_PATH} "${_pyt}/python_block.ari")
      add_test(NAME ariglx_python_program
               COMMAND ariglx snapshot --frames 3 --png "${CMAKE_CURRENT_BINARY_DIR}/ariglx_app.png"
                       ${CVC_ARIADNE_PYTHON_TEST_PATH} "${_pyt}/app.py")
      set_tests_properties(ariglx_python_program PROPERTIES
                           PASS_REGULAR_EXPRESSION "app.py: result 14, frames [1-9]")
      cvc_ariadne_python_test_properties(ariglx_python_check ariglx_python_program)
      set_tests_properties(ariglx_python_check ariglx_python_program PROPERTIES
                           RESOURCE_LOCK cvcgl_gl_context
                           SKIP_REGULAR_EXPRESSION "built without FTXUI;built without yaml-cpp")
      # matplotlib (pycvc_gl.ariadne_mpl): the default backend in ariglx, a blocking plt.show()
      # from a program with no document (figures.ari's windows, the host's loop), the same ended by
      # its last window closing (figures.ari closes the host), a python: block's plt.show() during
      # its load, shown by the document's own image:, and a document function's plt.show() through
      # exec() and runpy from a frame, which returns with no loop inside the frame. They skip
      # without matplotlib; its font cache goes in the build tree.
      add_test(NAME ariglx_matplotlib_show
               COMMAND ariglx snapshot --frames 3 --png "${CMAKE_CURRENT_BINARY_DIR}/ariglx_mpl.png"
                       ${CVC_ARIADNE_PYTHON_TEST_PATH} "${_pyt}/mpl_show.py")
      add_test(NAME ariglx_matplotlib_close
               COMMAND ariglx snapshot --frames 60
                       --png "${CMAKE_CURRENT_BINARY_DIR}/ariglx_mpl_close.png"
                       ${CVC_ARIADNE_PYTHON_TEST_PATH} "${_pyt}/mpl_show.py" --close)
      add_test(NAME ariglx_matplotlib_block
               COMMAND ariglx snapshot --frames 3
                       --png "${CMAKE_CURRENT_BINARY_DIR}/ariglx_mpl_block.png"
                       ${CVC_ARIADNE_PYTHON_TEST_PATH} "${_pyt}/mpl_block.ari")
      add_test(NAME ariglx_matplotlib_nested
               COMMAND ariglx snapshot --frames 6
                       --png "${CMAKE_CURRENT_BINARY_DIR}/ariglx_mpl_nested.png"
                       ${CVC_ARIADNE_PYTHON_TEST_PATH} "${_pyt}/mpl_nested.ari")
      set_tests_properties(ariglx_matplotlib_show PROPERTIES PASS_REGULAR_EXPRESSION
                           "mpl_show.py: backend module://pycvc_gl.ariadne_mpl, figures 1, image 640x480")
      set_tests_properties(ariglx_matplotlib_close PROPERTIES PASS_REGULAR_EXPRESSION
                           "mpl_show.py: closed: plt.show\\(\\) returned after [0-9]+ frames, the figure gone")
      set_tests_properties(ariglx_matplotlib_block PROPERTIES PASS_REGULAR_EXPRESSION
                           "mpl_block.ari: figure-1 640x480, display printed 'text out'")
      set_tests_properties(ariglx_matplotlib_nested PROPERTIES PASS_REGULAR_EXPRESSION
                           "returned, 0 frames, and host.run\\(\\) refused.*the frames went on, to 4")
      set(_mpl_tests ariglx_matplotlib_show ariglx_matplotlib_close ariglx_matplotlib_block
                     ariglx_matplotlib_nested)
      cvc_ariadne_python_test_properties(${_mpl_tests})
      set_tests_properties(${_mpl_tests} PROPERTIES
                           RESOURCE_LOCK cvcgl_gl_context
                           SKIP_REGULAR_EXPRESSION
                           "SKIP: no matplotlib;built without FTXUI;built without yaml-cpp")
      set_property(TEST ${_mpl_tests} APPEND PROPERTY
                   ENVIRONMENT "MPLCONFIGDIR=${CMAKE_CURRENT_BINARY_DIR}/mplconfig")
    endif()
  else()
    message(STATUS "ariglx: not built (no Python embedding files: Python3 Development.Embed)")
  endif()
endif()

# ctest coverage of the binary itself, in-tree only (it runs the shipped example documents). Each
# renders offscreen, so it takes the GL-context lock the cvcGL tests share; a libcvc without FTXUI
# (the terminal tests) or yaml-cpp skips rather than fails.
if(NOT CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
  set(_arigl_examples "${CMAKE_CURRENT_SOURCE_DIR}/../../examples/ariadne/documents")
  set(_arigl_tour "${CMAKE_CURRENT_SOURCE_DIR}/../../examples/ariadne/terminal/terminal_tour.ari")
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
  # snapshot --mp4 records the scene on its clock (record_frames; cvcGL's cvcgl_ari_record checks
  # the video itself); it skips where there is no video encoder. --mp4 without --seconds, or with
  # --terminal, is refused before anything loads.
  add_test(NAME arigl_record_mp4
           COMMAND arigl snapshot --mp4 "${CMAKE_CURRENT_BINARY_DIR}/arigl_record.mp4" --seconds 0.5
                   --fps 10 --width 160 --height 120
                   --png "${CMAKE_CURRENT_BINARY_DIR}/arigl_record.png"
                   "${_arigl_examples}/hello.ari")
  set_tests_properties(arigl_record_mp4 PROPERTIES
                       PASS_REGULAR_EXPRESSION "recorded 5 frames \\(0.5 s at 10 fps, 160x120"
                       SKIP_REGULAR_EXPRESSION "no video encoder;built without yaml-cpp"
                       RESOURCE_LOCK cvcgl_gl_context)
  add_test(NAME arigl_record_needs_seconds
           COMMAND arigl snapshot --mp4 "${CMAKE_CURRENT_BINARY_DIR}/arigl_no.mp4"
                   "${_arigl_examples}/hello.ari")
  set_tests_properties(arigl_record_needs_seconds PROPERTIES
                       PASS_REGULAR_EXPRESSION "--mp4 needs --seconds")
  add_test(NAME arigl_record_not_in_terminal
           COMMAND arigl snapshot --terminal --mp4 "${CMAKE_CURRENT_BINARY_DIR}/arigl_no.mp4"
                   --seconds 1 "${_arigl_examples}/hello.ari")
  set_tests_properties(arigl_record_not_in_terminal PROPERTIES
                       PASS_REGULAR_EXPRESSION "drop --terminal")
  # A document set as an app installs one (share/<app>/main.ari), run from another working
  # directory: its relative imports, out of its own directory and its parent's, resolve against
  # the importing file's directory. The same set is what ari's test and the browser shell's check
  # load (src/cvc/tests/data/ariadne_docset).
  add_test(NAME arigl_docset
           COMMAND arigl snapshot --terminal --width 90 --height 12 --component-path
                   "${CMAKE_CURRENT_SOURCE_DIR}/../cvc/ariadne"
                   "${CMAKE_CURRENT_SOURCE_DIR}/../cvc/tests/data/ariadne_docset/app/main.ari"
           WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}")
  set_tests_properties(arigl_docset PROPERTIES
                       PASS_REGULAR_EXPRESSION "from shared/note.ari"
                       FAIL_REGULAR_EXPRESSION "could not be resolved;unknown unit")
  # Pose entities (an outside simulation places them, no plant integrates them), offscreen: at
  # physics tick 60 the document prints the world's state hash, the one ariadne_scene_world_test's
  # APoseDocumentsHashIsPinned pins headless (a cross-platform-deterministic Jolt's, every platform).
  add_test(NAME arigl_pose_entities
           COMMAND arigl snapshot --frames 60 --width 320 --height 240
                   --png "${CMAKE_CURRENT_BINARY_DIR}/arigl_pose_entities.png"
                   "${CMAKE_CURRENT_SOURCE_DIR}/../cvc/tests/data/ariadne_pose_entities.ari")
  set_tests_properties(arigl_pose_entities PROPERTIES
                       PASS_REGULAR_EXPRESSION "pose_entities: tick 60 hash cbe4b2596cd56330 count 3"
                       SKIP_REGULAR_EXPRESSION "built without yaml-cpp;this build has no Jolt"
                       RESOURCE_LOCK cvcgl_gl_context)
  # A URL names a document for the browser shell; natively arigl says so.
  add_test(NAME arigl_url_document COMMAND arigl check https://example.invalid/app/main.ari)
  set_tests_properties(arigl_url_document PROPERTIES
                       PASS_REGULAR_EXPRESSION "arigl runs a document from a path")
  set_tests_properties(
    arigl_terminal_snapshot arigl_document_args arigl_rejects_bad_document_args
    arigl_terminal_stream_view arigl_window_snapshot arigl_check arigl_docset arigl_url_document
    arigl_record_needs_seconds arigl_record_not_in_terminal
    PROPERTIES RESOURCE_LOCK cvcgl_gl_context
               SKIP_REGULAR_EXPRESSION "built without FTXUI;built without yaml-cpp")

  # arigl joins a relay from its document's relay: block, as ari does (attach_host_relay,
  # ARIADNE.md §9.8.3): an offscreen snapshot of src/arisrv/tests/relay_arigl.ari, against
  # arisrvx running tests/call_service.py beside tests/call_server.ari, mirrors the published svc
  # and calls echo, over gRPC and the WebSocket. src/arisrv/tests/relay_pair.py runs the pair.
  if(TARGET arisrvx AND Python3_EXECUTABLE)
    set(_srv_tests "${CMAKE_CURRENT_SOURCE_DIR}/../arisrv/tests")
    set(_relay --relay-arg=--python "--relay-arg=${_srv_tests}/call_service.py"
               --relay-arg=--doc "--relay-arg=${_srv_tests}/call_server.ari"
               --relay-arg=--room --relay-arg=ctest --relay-arg=--state-listen
               --relay-arg=127.0.0.1:0 --relay-arg=--media-listen --relay-arg=127.0.0.1:0
               --relay-arg=--ws-listen --relay-arg=127.0.0.1:0)
    set(_relay_python_path)
    foreach(_a IN LISTS CVC_ARIADNE_PYTHON_TEST_PATH)
      list(APPEND _relay "--relay-arg=${_a}")
      list(APPEND _relay_python_path "--relay-arg=${_a}")
    endforeach()
    set(_relay_tests)
    foreach(_scheme grpc ws)
      add_test(NAME arisrvx_room_call_arigl_${_scheme}
               COMMAND "${Python3_EXECUTABLE}" "${_srv_tests}/relay_pair.py"
                       --relay $<TARGET_FILE:arisrvx> --client $<TARGET_FILE:arigl>
                       --client-command snapshot --scheme ${_scheme} ${_relay} --client-only
                       "--client-expect=relay_arigl: echo answered arigl_[a-z0-9]+"
                       --client-arg=--relay-room --client-arg=ctest --client-arg=--frames
                       --client-arg=900 --client-arg=--png
                       "--client-arg=${CMAKE_CURRENT_BINARY_DIR}/relay_arigl_${_scheme}.png"
                       "--client-arg=${_srv_tests}/relay_arigl.ari")
      list(APPEND _relay_tests arisrvx_room_call_arigl_${_scheme})
      # Bridged users (ARIADNE_CHAT.md): arisrvx running tests/bridge_service.py puts another chat
      # system's users in the room; src/arisrv/tests/bridge_arigl.ari, a participant (arigl's
      # conference mode, CVC_ARIADNE_RELAY), finds them in the roster, reads and answers their
      # posts, and messages one of them.
      add_test(NAME arisrvx_bridge_arigl_${_scheme}
               COMMAND "${Python3_EXECUTABLE}" "${_srv_tests}/relay_pair.py"
                       --relay $<TARGET_FILE:arisrvx> --client $<TARGET_FILE:arigl>
                       --client-command snapshot --scheme ${_scheme}
                       --relay-arg=--python "--relay-arg=${_srv_tests}/bridge_service.py"
                       --relay-arg=--room --relay-arg=ctest --relay-arg=--state-listen
                       --relay-arg=127.0.0.1:0 --relay-arg=--media-listen --relay-arg=127.0.0.1:0
                       --relay-arg=--ws-listen --relay-arg=127.0.0.1:0 ${_relay_python_path}
                       --client-only "--client-expect=bridge_arigl: ok"
                       "--client-env=CVC_ARIADNE_RELAY={url}" --client-env=CVC_ARIADNE_ROOM=ctest
                       --client-env=CVC_ARIADNE_NAME=Alice --client-arg=--frames
                       --client-arg=1800 --client-arg=--png
                       "--client-arg=${CMAKE_CURRENT_BINARY_DIR}/bridge_arigl_${_scheme}.png"
                       "--client-arg=${_srv_tests}/bridge_arigl.ari")
      list(APPEND _relay_tests arisrvx_bridge_arigl_${_scheme})
    endforeach()
    cvc_ariadne_python_test_properties(${_relay_tests})
    set_tests_properties(${_relay_tests} PROPERTIES
                         RESOURCE_LOCK cvcgl_gl_context TIMEOUT 120
                         SKIP_REGULAR_EXPRESSION "No module named '(fastapi|uvicorn|starlette)';built without yaml-cpp;needs pycvc beside arisrvx;no pycvc to convert")
  endif()
endif()

if(EMSCRIPTEN)
  # cvcGL's fallback fonts (src/cvcGL/fonts): a browser has no system fonts for ImGui, so each
  # browser host embeds them where an install keeps them (<prefix>/share/libcvc/fonts), and the
  # overlay finds them there: DejaVu Sans Mono as the UI font, Twemoji's colour emoji under it.
  set(_arigl_font_embed "")
  if(CVCGL_FONT_DIR AND EXISTS "${CVCGL_FONT_DIR}/Twemoji.Mozilla.ttf")
    set(_arigl_font_embed "--embed-file=${CVCGL_FONT_DIR}@/usr/share/libcvc/fonts")
  endif()

  # cyprty — arigl's browser build: the conference client. arigl built to wasm, dialing the relay
  # over a WebSocket (cvc::state_transport_ws) and rendering conference_roster.ari (control plane
  # only: room-join + room-roster). Its page, web/cyprty.html, sets CVC_ARIADNE_RELAY to
  # wss://<page origin>/cyprty-ws and the run arguments; the build writes it next to cyprty.js. It
  # needs -DCVC_ENABLE_WS=ON (grpc off); Emscripten's WebSocket client needs -lwebsocket.js.
  # Deployed to cyberpcangel.com/cyprty, not a gallery card, so it builds into bin/extra/, which
  # examples/web/build-pages.py does not scan, and joins `wasm-extra-demos` when the examples are on.
  # Out of ALL, as it was as an example: the wasm SDK and pycvc-gl builds never link it.
  set(CVC_ARIGL_WASM_DOC "${CMAKE_CURRENT_SOURCE_DIR}/../../examples/ariadne/documents/conference_roster.ari"
      CACHE FILEPATH "The .ari document cyprty, arigl's browser build, embeds")
  get_filename_component(_arigl_doc_name "${CVC_ARIGL_WASM_DOC}" NAME)
  add_executable(cyprty EXCLUDE_FROM_ALL main.cpp gallery.cpp)
  cvc_document_host_stack(cyprty)
  target_link_libraries(cyprty PRIVATE cvc::cvcGL ${VTK_LIBRARIES})
  vtk_module_autoinit(TARGETS cyprty MODULES ${VTK_LIBRARIES})
  # The same link options as the gallery demos (examples/CMakeLists.txt): each keeps its own loop
  # and yields with emscripten_sleep (Asyncify), the heap grows, and cvc::net fetches with -sFETCH.
  target_link_options(cyprty PRIVATE
    -lwebsocket.js
    "--embed-file=${CVC_ARIGL_WASM_DOC}@/${_arigl_doc_name}"
    -sASYNCIFY=1
    -sASYNCIFY_STACK_SIZE=131072
    -sALLOW_MEMORY_GROWTH=1
    -sMAXIMUM_MEMORY=4GB
    -sSTACK_SIZE=2097152
    -sEXIT_RUNTIME=0
    -sFETCH=1)
  if(CVC_WASM_PTHREADS)
    target_link_options(cyprty PRIVATE -sPTHREAD_POOL_SIZE=4)
  endif()
  # The component library the document imports (cvc://components/...: its Users and Chat windows),
  # embedded where an install keeps it, as arigl-web does.
  set(_cyprty_components "${CMAKE_CURRENT_SOURCE_DIR}/../cvc/ariadne/components")
  file(GLOB _cyprty_component_files CONFIGURE_DEPENDS "${_cyprty_components}/*.ari")
  target_link_options(cyprty PRIVATE
                      "--embed-file=${_cyprty_components}@/usr/share/libcvc/ariadne/components"
                      ${_arigl_font_embed})
  target_compile_definitions(cyprty PRIVATE CVC_ARIGL_COMPONENT_DIR="/usr/share/libcvc/ariadne")
  # Relink (re-embed) whenever the baked document or a component changes; otherwise a .ari-only
  # edit leaves the stale document embedded.
  set_property(TARGET cyprty APPEND PROPERTY LINK_DEPENDS "${CVC_ARIGL_WASM_DOC}"
                                                          ${_cyprty_component_files})
  if(COMMAND cvcgl_wasm_app)
    cvcgl_wasm_app(cyprty)
  endif()
  if(CMAKE_RUNTIME_OUTPUT_DIRECTORY)
    set_target_properties(cyprty PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/extra")
  else()
    set_target_properties(cyprty PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/extra")
  endif()
  file(GENERATE OUTPUT "$<TARGET_FILE_DIR:cyprty>/cyprty.html"
       INPUT "${CMAKE_CURRENT_SOURCE_DIR}/web/cyprty.html")
  file(GENERATE OUTPUT "$<TARGET_FILE_DIR:cyprty>/cvc-audio-worklet.js"
       INPUT "${CMAKE_CURRENT_SOURCE_DIR}/../cvcMedia/web/cvc-audio-worklet.js")
  if(TARGET wasm-extra-demos)
    add_dependencies(wasm-extra-demos cyprty)
  endif()

  # arigl itself in a browser, linked as the gallery demos are: it keeps its own loop and yields
  # with emscripten_sleep (Asyncify), the heap grows, and cvc::net fetches with -sFETCH.
  set(_arigl_wasm_link
      -sASYNCIFY=1
      -sASYNCIFY_STACK_SIZE=131072
      -sALLOW_MEMORY_GROWTH=1
      -sMAXIMUM_MEMORY=4GB
      -sSTACK_SIZE=2097152
      -sEXIT_RUNTIME=0
      -sFETCH=1)
  if(CVC_WASM_PTHREADS)
    list(APPEND _arigl_wasm_link -sPTHREAD_POOL_SIZE=4)
  endif()
  set(_arigl_components "${CMAKE_CURRENT_SOURCE_DIR}/../cvc/ariadne/components")
  file(GLOB _arigl_component_files CONFIGURE_DEPENDS "${_arigl_components}/*.ari")

  # In the examples' browser gallery (examples/web), arigl is the gallery's Ariadne app: every
  # .ari demo there is arigl opened on its document (<gallery>/arigl/?doc=<name>), and its Demos
  # menu switches between them in place. So the documents, the component library and their data
  # go into its file system, under the paths examples/web/demos.json's "arigl" boot entry and
  # build-pages.py's gallery.json name, and it joins the gallery's wasm-demos.
  #
  # A wasm build without the examples makes arigl-web instead (the next branch): the stock
  # browser host for anyone's documents.
  set(_arigl_examples "${CMAKE_CURRENT_SOURCE_DIR}/../../examples")
  if(TARGET wasm-demos AND EXISTS "${_arigl_examples}/ariadne/documents")
    target_link_options(arigl PRIVATE
      # One arigl.data package beside it (build-pages.py ships it): Emscripten won't mix
      # --preload-file with --embed-file, and the datasets are too big to embed in the module.
      "--preload-file=${_arigl_examples}/ariadne/documents@/ariadne/documents"
      "--preload-file=${_arigl_components}@/components"
      ${_arigl_wasm_link})
    if(EXISTS "${_arigl_examples}/data") # the FEM gallery's datasets
      target_link_options(arigl PRIVATE "--preload-file=${_arigl_examples}/data@/data")
    endif()
    # Re-embed when a document or component changes.
    file(GLOB _arigl_embedded CONFIGURE_DEPENDS "${_arigl_examples}/ariadne/documents/*.ari"
         "${_arigl_components}/*.ari")
    set_property(TARGET arigl APPEND PROPERTY LINK_DEPENDS ${_arigl_embedded})
    if(COMMAND cvcgl_wasm_app)
      cvcgl_wasm_app(arigl)
    endif()
    add_dependencies(wasm-demos arigl)

    # arigl_term: `arigl run --terminal` in the browser, behind an xterm.js page (web/
    # arigl_term.html, from ari's): the widgets through FTXUI's Emscripten screen on the module's
    # stdio, the scene rendered by VTK into a hidden canvas and drawn behind them, as iTerm2 inline
    # images (xterm.js's image addon) or half-blocks. The page picks the document (?doc=) and the
    # drawing (?gfx=); the documents and components are preloaded as for arigl, without the
    # datasets. Built by name (not part of wasm-demos), and only defined where FTXUI is found: it
    # exports FTXUI's resize hook. (libcvc found FTXUI in its own directory; its targets aren't
    # visible here.)
    find_package(ftxui CONFIG QUIET)
    if(TARGET ftxui::screen)
      add_executable(arigl_term EXCLUDE_FROM_ALL main.cpp gallery.cpp)
      cvc_document_host_stack(arigl_term)
      target_link_libraries(arigl_term PRIVATE cvc::cvcGL ${VTK_LIBRARIES})
      vtk_module_autoinit(TARGETS arigl_term MODULES ${VTK_LIBRARIES})
      target_compile_definitions(arigl_term PRIVATE CVC_ARIGL_WASM_TERMINAL=1)
      target_link_options(arigl_term PRIVATE
        "--preload-file=${_arigl_examples}/ariadne/documents@/ariadne/documents"
        "--preload-file=${_arigl_examples}/ariadne/terminal@/ariadne/terminal"
        "--preload-file=${CMAKE_CURRENT_SOURCE_DIR}/../cvc/ariadne/components@/components"
        -sASYNCIFY=1
        -sASYNCIFY_STACK_SIZE=131072
        -sALLOW_MEMORY_GROWTH=1
        -sMAXIMUM_MEMORY=4GB
        -sSTACK_SIZE=2097152
        -sEXIT_RUNTIME=1
        -sFETCH=1
        -sEXPORTED_FUNCTIONS=_main,_ftxui_on_resize)
      set_property(TARGET arigl_term APPEND PROPERTY LINK_DEPENDS ${_arigl_embedded})
      if(COMMAND cvcgl_wasm_app)
        # The state shim patches every WebGL context on the page, xterm.js's renderer's too.
        cvcgl_wasm_app(arigl_term STATE_SHIM OFF)
      endif()
      file(GENERATE OUTPUT "$<TARGET_FILE_DIR:arigl_term>/arigl_term.html"
           INPUT "${CMAKE_CURRENT_SOURCE_DIR}/web/arigl_term.html")
    endif()
  else()
    # arigl-web: the stock browser host (cvcpkg/recipes/arigl-web). It carries no documents of its
    # own: its page, web/arigl-web.{html,js,css} (the html shipped as index.html), runs the one
    # ?src= names, which arigl fetches with the files it imports into MEMFS at their URL paths
    # (cvc/ariadne/web_document.h). Only the component library is built in, embedded where an
    # install keeps it (<prefix>/share/libcvc/ariadne), so cvc://components/... resolves with no
    # download.
    target_link_options(arigl PRIVATE
      "--embed-file=${_arigl_components}@/usr/share/libcvc/ariadne/components"
      ${_arigl_font_embed} ${_arigl_wasm_link})
    target_compile_definitions(arigl PRIVATE
                               CVC_ARIGL_COMPONENT_DIR="/usr/share/libcvc/ariadne")
    set_property(TARGET arigl APPEND PROPERTY LINK_DEPENDS ${_arigl_component_files})
    if(COMMAND cvcgl_wasm_app)
      cvcgl_wasm_app(arigl)
    endif()
    foreach(_f arigl-web.html arigl-web.js arigl-web.css)
      file(GENERATE OUTPUT "$<TARGET_FILE_DIR:arigl>/${_f}"
           INPUT "${CMAKE_CURRENT_SOURCE_DIR}/web/${_f}")
    endforeach()
    # cvcMedia's AudioWorklets (cvc/media/web_audio.h), served beside the page.
    file(GENERATE OUTPUT "$<TARGET_FILE_DIR:arigl>/cvc-audio-worklet.js"
         INPUT "${CMAKE_CURRENT_SOURCE_DIR}/../cvcMedia/web/cvc-audio-worklet.js")

    # ariglx-web: the same shell with an embedded Python (cvcpkg/recipes/ariglx-web;
    # docs/roadmap/ARIADNE_SCRIPTING_HOSTS.md §5): a document's python: block, and a Python program
    # in place of the document, run in the browser with pycvc, pycvc_gl and numpy. A browser has no
    # dlopen, so the interpreter and every extension are linked in (ariadne_python.cmake), and the
    # pure-Python side is python312.zip beside the page, which arigl-web.js fetches before the
    # module starts. Built with the bindings (CVC_BUILD_PYCVC, which builds the SWIG modules as
    # static archives on Emscripten); CVC_PYCVCGL_VTK_BRIDGE also links vtk-python (the full tier).
    if(CVC_BUILD_PYCVC)
      include("${CMAKE_CURRENT_LIST_DIR}/../ariadne_python/ariadne_python.cmake")
      add_executable(ariglx main.cpp gallery.cpp)
      cvc_document_host_stack(ariglx)
      target_link_libraries(ariglx PRIVATE cvc::cvcGL ${VTK_LIBRARIES})
      vtk_module_autoinit(TARGETS ariglx MODULES ${VTK_LIBRARIES})
      set(_ariglx_bridge "")
      if(CVC_PYCVCGL_VTK_BRIDGE)
        set(_ariglx_bridge VTK_BRIDGE)
      endif()
      # Beside ariglx.js (an OUTPUT can't name a target's directory).
      set(_ariglx_dir "${CMAKE_CURRENT_BINARY_DIR}")
      if(CMAKE_RUNTIME_OUTPUT_DIRECTORY)
        set(_ariglx_dir "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")
      endif()
      cvc_ariadne_python_wasm_host(ariglx ZIP "${_ariglx_dir}/python312.zip" ${_ariglx_bridge})
      # The shell's link, with a deeper stack for the interpreter (link-host.sh's 8 MB), and a
      # first heap that holds it and the module's static data (22 MB of it), which the default
      # 16 MB does not.
      list(TRANSFORM _arigl_wasm_link REPLACE "^-sSTACK_SIZE=.*" "-sSTACK_SIZE=8388608"
           OUTPUT_VARIABLE _ariglx_wasm_link)
      # Threaded (wasm-mt), a pool of 8 workers where arigl has 4, for Python's threads. Their
      # start() waits for the thread to run, and on the page's main thread (init:, an export, a
      # tick) a thread only runs on a worker the pool already has: a new one can't load while
      # that thread waits, so the page hangs. By init: the shell holds one, so 4 left room for 3
      # threads at once and 8 leaves 7. Each thread gets the 8 MB stack above (the default
      # pthread stack is STACK_SIZE).
      if(CVC_WASM_PTHREADS)
        list(TRANSFORM _ariglx_wasm_link REPLACE "^-sPTHREAD_POOL_SIZE=.*"
             "-sPTHREAD_POOL_SIZE=8")
      endif()
      target_link_options(ariglx PRIVATE
        "--embed-file=${_arigl_components}@/usr/share/libcvc/ariadne/components"
        ${_arigl_font_embed} ${_ariglx_wasm_link} -sINITIAL_MEMORY=67108864)
      target_compile_definitions(ariglx PRIVATE
                                 CVC_ARIGL_COMPONENT_DIR="/usr/share/libcvc/ariadne")
      set_property(TARGET ariglx APPEND PROPERTY LINK_DEPENDS ${_arigl_component_files})
      if(COMMAND cvcgl_wasm_app)
        cvcgl_wasm_app(ariglx)
      endif()
      # Its page: arigl-web's, told to start ariglx.js on ariglx.wasm and to fetch the zip first.
      file(READ "${CMAKE_CURRENT_SOURCE_DIR}/web/arigl-web.html" _ariglx_html)
      string(REPLACE "<title>arigl</title>" "<title>ariglx</title>" _ariglx_html "${_ariglx_html}")
      string(REPLACE "<script src=\"arigl-web.js\"></script>"
             "<script src=\"arigl-web.js\" data-module=\"ariglx\" data-python=\"python312.zip\"></script>"
             _ariglx_html "${_ariglx_html}")
      if(NOT _ariglx_html MATCHES "data-module=\"ariglx\"")
        message(FATAL_ERROR "ariglx: web/arigl-web.html no longer has the script tag to point at "
                            "ariglx.js")
      endif()
      file(GENERATE OUTPUT "$<TARGET_FILE_DIR:arigl>/ariglx.html" CONTENT "${_ariglx_html}")

      # ... with WebAssembly guests, as the native ariglx has them: a document's wasm: block runs
      # on the page's own WebAssembly (src/ariadne_wasm/browser_runtime.cpp), its guest transformed
      # ahead (cvc_ariadne_wasm_transform). The page fetches the guest with the document
      # (doc_fetch.cpp follows a wasm: block's module). The shell links Asyncify, so a guest the
      # page won't compile on its own thread (over Chrome's 8 MB) is compiled asynchronously.
      include("${CMAKE_CURRENT_LIST_DIR}/../ariadne_wasm/ariadne_wasm.cmake")
      cvc_ariadne_wasm_host(ariglx ASYNC_LOAD)
      # The browser smoke's guest documents (web/test/wasm) and the test guest they run, transformed:
      # the ariglx_wasm_guests target (built by name, not by the package) writes them to
      # ariglx-wasm-guests beside ariglx.js, the directory arigl_web_smoke.mjs --wasmdocs serves.
      cvc_find_wasm_opt()
      if(CVC_WASM_OPT)
        set(_ariglx_guests "${_ariglx_dir}/ariglx-wasm-guests")
        cvc_ariadne_wasm_transform("${_ariglx_guests}/guest.wasm"
                                   "${CVC_ARIADNE_WASM_TESTS}/guest.wat")
        set(_ariglx_guest_files "${_ariglx_guests}/guest.wasm")
        file(GLOB _ariglx_guest_docs CONFIGURE_DEPENDS
             "${CMAKE_CURRENT_SOURCE_DIR}/web/test/wasm/*.ari")
        foreach(_d ${_ariglx_guest_docs})
          get_filename_component(_n "${_d}" NAME)
          add_custom_command(OUTPUT "${_ariglx_guests}/${_n}"
                             COMMAND "${CMAKE_COMMAND}" -E make_directory "${_ariglx_guests}"
                             COMMAND "${CMAKE_COMMAND}" -E copy_if_different "${_d}"
                                     "${_ariglx_guests}/${_n}"
                             DEPENDS "${_d}" VERBATIM)
          list(APPEND _ariglx_guest_files "${_ariglx_guests}/${_n}")
        endforeach()
        add_custom_target(ariglx_wasm_guests DEPENDS ${_ariglx_guest_files})
      else()
        message(STATUS "ariglx: no wasm-opt (CVC_WASM_OPT), so no ariglx_wasm_guests for the "
                       "browser smoke")
      endif()
    endif()
  endif()
endif()
