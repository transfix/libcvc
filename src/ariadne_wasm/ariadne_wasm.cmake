# WebAssembly guests in Ariadne documents (the wasm engine): docs/roadmap/ARIADNE_SCRIPTING_HOSTS.md
# §8.2-8.7, phases G1, G2 and G3.
#
#   cvc_ariadne_wasm_host(<target> [INSTALL] [ASYNC_LOAD])
#
# compiles the engine (wasm_engine.cpp, the CBOR codec, wasm_binary.cpp, the WASI stubs, the gas
# pass and the runtime) into a target that already links cvc, and defines CVC_ARIADNE_WASM, so a
# host's main.cpp turns its guests on. Natively the runtime is the one cvc_find_ariadne_wasm chose
# (CVC_ARIADNE_WASM_BACKEND), linked here:
#   - wasmtime's C API (the cvcpkg `wasmtime` package: wasmtime::wasmtime), where it is found. On
#     Windows, with wasmtime.dll's import library, the DLL is copied beside the target; with the
#     static library the target carries it. INSTALL also installs that DLL into bin, beside the
#     host.
#   - WAMR's wasm-c-api (the cvcpkg `wamr` package's static libiwasm: iwasm::iwasm,
#     wamr_runtime.cpp), where there is no wasmtime: the BSDs and Haiku (G3). It runs guests
#     transformed for the browser. With binaryen (the cvcpkg `binaryen` package's static
#     libbinaryen: binaryen::binaryen) the host also transforms any other guest as it loads it, and
#     reads the text format (wasm_transform.cpp, CVC_ARIADNE_WASM_BINARYEN); without it, it runs
#     only guests transformed at install.
# Under Emscripten the runtime is the page's own WebAssembly (browser_runtime.cpp), which runs
# guests transformed ahead (cvc_ariadne_wasm_transform), and nothing is linked. ASYNC_LOAD is for a
# target linked with Asyncify (-sASYNCIFY), as ariglx is: a guest the page won't compile or
# instantiate on its own thread (Chrome refuses a module over 8 MB there) is then compiled and
# instantiated asynchronously, and the document's load waits for it. Without it such a guest fails
# to load.
set(_CVC_ARIADNE_WASM_DIR "${CMAKE_CURRENT_LIST_DIR}")

# Which runtime runs the guests natively: wasmtime where it is found, else WAMR (auto); or the one
# named, which a configure then requires.
set(CVC_ARIADNE_WASM_RUNTIME "auto" CACHE STRING
    "The wasm guests' native runtime: auto (wasmtime where found, else WAMR), wasmtime or wamr")
set_property(CACHE CVC_ARIADNE_WASM_RUNTIME PROPERTY STRINGS auto wasmtime wamr)

#   cvc_find_ariadne_wasm(<required>)
#
# Look for a runtime for the guests, as CVC_ARIADNE_WASM_RUNTIME says: wasmtime's C API, and where
# it misses (or with CVC_ARIADNE_WASM_RUNTIME=wamr) WAMR's, with binaryen if it is there. Sets
# CVC_ARIADNE_WASM_FOUND, and CVC_ARIADNE_WASM_BACKEND (wasmtime, wamr or browser). With
# <required> true a miss is an error; otherwise the guests are left out of the host. Under
# Emscripten there is nothing to find: the browser backend needs no library.
macro(cvc_find_ariadne_wasm required)
  set(CVC_ARIADNE_WASM_FOUND FALSE)
  set(CVC_ARIADNE_WASM_BACKEND "")
  if(EMSCRIPTEN)
    set(CVC_ARIADNE_WASM_FOUND TRUE)
    set(CVC_ARIADNE_WASM_BACKEND browser)
  else()
    if(NOT CVC_ARIADNE_WASM_RUNTIME MATCHES "^(auto|wasmtime|wamr)$")
      message(FATAL_ERROR "CVC_ARIADNE_WASM_RUNTIME is auto, wasmtime or wamr, not "
                          "'${CVC_ARIADNE_WASM_RUNTIME}'")
    endif()
    if(NOT CVC_ARIADNE_WASM_RUNTIME STREQUAL "wamr")
      if(NOT TARGET wasmtime::wasmtime)
        if(${required} AND CVC_ARIADNE_WASM_RUNTIME STREQUAL "wasmtime")
          find_package(wasmtime CONFIG REQUIRED)
        else()
          find_package(wasmtime CONFIG QUIET)
        endif()
      endif()
      if(TARGET wasmtime::wasmtime)
        set(CVC_ARIADNE_WASM_FOUND TRUE)
        set(CVC_ARIADNE_WASM_BACKEND wasmtime)
      endif()
    endif()
    if(NOT CVC_ARIADNE_WASM_FOUND AND NOT CVC_ARIADNE_WASM_RUNTIME STREQUAL "wasmtime")
      if(NOT TARGET iwasm::iwasm)
        find_package(iwasm CONFIG QUIET)
      endif()
      if(TARGET iwasm::iwasm)
        set(CVC_ARIADNE_WASM_FOUND TRUE)
        set(CVC_ARIADNE_WASM_BACKEND wamr)
        if(NOT TARGET binaryen::binaryen)
          find_package(binaryen CONFIG QUIET)
        endif()
      endif()
    endif()
    if(${required} AND NOT CVC_ARIADNE_WASM_FOUND)
      if(CVC_ARIADNE_WASM_RUNTIME STREQUAL "wamr")
        message(FATAL_ERROR "WebAssembly guests are required, and WAMR (the cvcpkg `wamr` "
                            "package: iwasmConfig.cmake) isn't found")
      else()
        message(FATAL_ERROR "WebAssembly guests are required, and neither wasmtime (the cvcpkg "
                            "`wasmtime` package) nor WAMR (`wamr`) is found")
      endif()
    endif()
  endif()
endmacro()

# The test documents the hosts' ctests run (wasm_block.ari, wasi_block.ari and their guests).
set(CVC_ARIADNE_WASM_TESTS "${CMAKE_CURRENT_LIST_DIR}/tests")

#   cvc_find_wasm_opt()
#
# Find binaryen's wasm-opt, which transforms guests for the browser: emsdk's (beside emcc's
# Emscripten, or under $EMSDK), or one on PATH. Sets the cache variable CVC_WASM_OPT (a -NOTFOUND
# without one); -DCVC_WASM_OPT=<path> names it.
macro(cvc_find_wasm_opt)
  find_program(CVC_WASM_OPT wasm-opt
    HINTS "${EMSCRIPTEN_ROOT_PATH}/../bin" "$ENV{EMSDK}/upstream/bin"
    NO_CMAKE_FIND_ROOT_PATH
    DOC "binaryen's wasm-opt, which transforms WebAssembly guests for the browser")
endmacro()

#   cvc_ariadne_wasm_meter_tool()
#
# ari_wasm_meter (meter_main.cpp): the gas pass as a command. Natively it is a host program. Under
# Emscripten it is a node one, on the host's files, which a custom command runs through the
# toolchain's CMAKE_CROSSCOMPILING_EMULATOR (node).
function(cvc_ariadne_wasm_meter_tool)
  if(TARGET ari_wasm_meter)
    return()
  endif()
  add_executable(ari_wasm_meter "${_CVC_ARIADNE_WASM_DIR}/meter_main.cpp"
                                "${_CVC_ARIADNE_WASM_DIR}/meter.cpp")
  if(EMSCRIPTEN)
    target_link_options(ari_wasm_meter PRIVATE -sNODERAWFS=1 -sENVIRONMENT=node -sEXIT_RUNTIME=1
                                               -sALLOW_MEMORY_GROWTH=1)
  endif()
endfunction()

#   cvc_ariadne_wasm_transform(<out> <in>)
#
# The install-time transform (guest_runtime.h; ARIADNE_SCRIPTING_HOSTS.md §8.5): <in>, a .wasm or
# .wat guest, becomes <out>, a .wasm the browser backend runs. It is metered first (meter.h: a
# charge of each block's cost against its ari_gas global, and a call of ari.gas where the slice
# can't pay), then Asyncify goes around every `ari` import, ari.gas among them, so the guest can
# pause at any of them. wasmtime runs <out> as it runs <in>, and counts the same. These are the
# commands a guest's author runs, with the features below:
#
#   wasm-opt <in> <features> -o <binary>          (a .wat to a binary; a .wasm as it is)
#   ari_wasm_meter <binary> <metered>
#   wasm-opt <metered> <features> --asyncify --pass-arg=asyncify-imports@ari.* -O1 -o <out>
#
# Binaryen reads only the MVP unless it is told more, and a module from clang or rustc uses bulk
# memory, sign extension and the rest. CVC_WASM_GUEST_FEATURES are those that wasmtime and the
# browsers run, and none that only some do (threads, GC, exception handling, memory64).
set(CVC_WASM_GUEST_FEATURES
    --enable-sign-ext --enable-mutable-globals --enable-nontrapping-float-to-int
    --enable-bulk-memory --enable-bulk-memory-opt --enable-multivalue --enable-reference-types
    --enable-simd --enable-extended-const --enable-tail-call)
function(cvc_ariadne_wasm_transform out in)
  cvc_find_wasm_opt()
  if(NOT CVC_WASM_OPT)
    message(FATAL_ERROR "cvc_ariadne_wasm_transform: no wasm-opt (binaryen, which emsdk ships): "
                        "set CVC_WASM_OPT")
  endif()
  cvc_ariadne_wasm_meter_tool()
  get_filename_component(_dir "${out}" DIRECTORY)
  add_custom_command(OUTPUT "${out}"
    COMMAND "${CMAKE_COMMAND}" -E make_directory "${_dir}"
    COMMAND "${CVC_WASM_OPT}" "${in}" ${CVC_WASM_GUEST_FEATURES} -o "${out}.binary"
    COMMAND ari_wasm_meter "${out}.binary" "${out}.metered"
    COMMAND "${CVC_WASM_OPT}" "${out}.metered" ${CVC_WASM_GUEST_FEATURES} --asyncify
            "--pass-arg=asyncify-imports@ari.*" -O1 -o "${out}"
    DEPENDS "${in}" ari_wasm_meter
    COMMENT "ari_wasm_meter and wasm-opt: ${in}, for the browser (metered, Asyncify)"
    VERBATIM)
endfunction()

#   cvc_ariadne_wasm_test_guests(<target>)
#
# The test guests, tests/guest.wat and tests/page_guest.wat, transformed into the binary dir's
# wasm_guests for <target>, which finds them at CVC_WASM_GUESTS_DIR (under Emscripten embedded, at
# /guests). Without wasm-opt there are none natively, and the target's cases that need them skip;
# under Emscripten (emsdk ships wasm-opt) that is an error.
function(cvc_ariadne_wasm_test_guests target)
  cvc_find_wasm_opt()
  if(NOT CVC_WASM_OPT)
    if(EMSCRIPTEN)
      message(FATAL_ERROR "${target}: no wasm-opt beside emcc: set CVC_WASM_OPT")
    endif()
    message(STATUS "${target}: no wasm-opt (CVC_WASM_OPT), so no transformed guests: the cases "
                   "that run them skip")
    return()
  endif()
  set(_dir "${CMAKE_CURRENT_BINARY_DIR}/wasm_guests")
  set(_guests)
  foreach(_g guest page_guest wasi_guest)
    cvc_ariadne_wasm_transform("${_dir}/${_g}.wasm" "${_CVC_ARIADNE_WASM_DIR}/tests/${_g}.wat")
    list(APPEND _guests "${_dir}/${_g}.wasm")
  endforeach()
  add_custom_target(${target}_guests DEPENDS ${_guests})
  add_dependencies(${target} ${target}_guests)
  if(EMSCRIPTEN)
    target_link_options(${target} PRIVATE "SHELL:--embed-file ${_dir}@/guests")
    set_property(TARGET ${target} APPEND PROPERTY LINK_DEPENDS ${_guests})
    target_compile_definitions(${target} PRIVATE CVC_WASM_GUESTS_DIR="/guests")
  else()
    target_compile_definitions(${target} PRIVATE CVC_WASM_GUESTS_DIR="${_dir}")
  endif()
  message(STATUS "${target}: the test guests, transformed by ${CVC_WASM_OPT}")
endfunction()

# The WAMR backend in <target> (cvc_ariadne_wasm_host, G3): wamr_runtime.cpp on WAMR's wasm-c-api
# and wamr_load.cpp on its native API, both in the static libiwasm, and wasm_transform.cpp where
# binaryen is found. binaryen's C++ headers (include/binaryen: "wasm.h", "ir/...", "support/...")
# are on wasm_transform.cpp's include path only, so their names shadow nothing in the host's
# other files, and outside MSVC that file is built without RTTI, as libbinaryen is.
function(_cvc_ariadne_wasm_wamr target)
  target_sources(${target} PRIVATE "${_CVC_ARIADNE_WASM_DIR}/wamr_runtime.cpp"
                                   "${_CVC_ARIADNE_WASM_DIR}/wamr_load.cpp")
  target_link_libraries(${target} PRIVATE iwasm::iwasm)
  # The backend pauses guests by Asyncify, as the browser's does (the tests' cases for that).
  target_compile_definitions(${target} PRIVATE CVC_ARIADNE_WASM_WAMR)
  if(WIN32)
    # The static libiwasm.lib: its declarations without dllimport (wasm_c_api.h, wasm_export.h).
    target_compile_definitions(${target} PRIVATE WASM_API_EXTERN= WASM_RUNTIME_API_EXTERN=)
    target_link_libraries(${target} PRIVATE ntdll ws2_32)
  endif()
  get_target_property(_lib iwasm::iwasm IMPORTED_LOCATION)
  if(TARGET binaryen::binaryen)
    set(_t "${_CVC_ARIADNE_WASM_DIR}/wasm_transform.cpp")
    target_sources(${target} PRIVATE "${_t}")
    target_compile_definitions(${target} PRIVATE CVC_ARIADNE_WASM_BINARYEN)
    get_target_property(_binc binaryen::binaryen INTERFACE_INCLUDE_DIRECTORIES)
    get_target_property(_bdefs binaryen::binaryen INTERFACE_COMPILE_DEFINITIONS)
    get_target_property(_blib binaryen::binaryen IMPORTED_LOCATION)
    get_target_property(_bdeps binaryen::binaryen INTERFACE_LINK_LIBRARIES)
    set_property(SOURCE "${_t}" TARGET_DIRECTORY ${target} APPEND PROPERTY INCLUDE_DIRECTORIES
                 ${_binc})
    if(_bdefs)
      set_property(SOURCE "${_t}" TARGET_DIRECTORY ${target} APPEND PROPERTY COMPILE_DEFINITIONS
                   ${_bdefs})
    endif()
    if(NOT MSVC)
      set_property(SOURCE "${_t}" TARGET_DIRECTORY ${target} APPEND PROPERTY COMPILE_OPTIONS
                   -fno-rtti)
    endif()
    # The library by its file, without the target's include directories.
    target_link_libraries(${target} PRIVATE "${_blib}")
    if(_bdeps)
      target_link_libraries(${target} PRIVATE ${_bdeps})
    endif()
    message(STATUS "${target}: WebAssembly guests (WAMR: ${_lib}; binaryen: ${_blib}, so any "
                   "guest, and the text format)")
  else()
    message(STATUS "${target}: WebAssembly guests (WAMR: ${_lib}; no binaryen, so only guests "
                   "transformed at install)")
  endif()
endfunction()

function(cvc_ariadne_wasm_host target)
  cmake_parse_arguments(_w "INSTALL;ASYNC_LOAD" "" "" ${ARGN})
  target_sources(${target} PRIVATE "${_CVC_ARIADNE_WASM_DIR}/wasm_engine.cpp"
                                   "${_CVC_ARIADNE_WASM_DIR}/wasm_binary.cpp"
                                   "${_CVC_ARIADNE_WASM_DIR}/meter.cpp"
                                   "${_CVC_ARIADNE_WASM_DIR}/wasi_preview1.cpp")
  target_include_directories(${target} PRIVATE "${_CVC_ARIADNE_WASM_DIR}")
  target_compile_definitions(${target} PRIVATE CVC_ARIADNE_WASM)
  if(EMSCRIPTEN)
    target_sources(${target} PRIVATE "${_CVC_ARIADNE_WASM_DIR}/browser_runtime.cpp")
    if(_w_ASYNC_LOAD)
      target_compile_definitions(${target} PRIVATE CVC_ARIADNE_WASM_ASYNC_LOAD)
      message(STATUS "${target}: WebAssembly guests (the browser's own WebAssembly, a large "
                     "guest compiled asynchronously)")
    else()
      message(STATUS "${target}: WebAssembly guests (the browser's own WebAssembly)")
    endif()
    return()
  endif()
  if(CVC_ARIADNE_WASM_BACKEND STREQUAL "wamr")
    _cvc_ariadne_wasm_wamr(${target})
    return()
  endif()
  target_sources(${target} PRIVATE "${_CVC_ARIADNE_WASM_DIR}/wasmtime_runtime.cpp")
  get_target_property(_lib wasmtime::wasmtime IMPORTED_LOCATION)
  set(_link wasmtime::wasmtime)
  if(UNIX AND NOT APPLE AND _lib MATCHES "\\.so$")
    # Upstream's libwasmtime.so has no SONAME, and the package's target is UNKNOWN, so CMake links
    # it by its path and the host records that path (relative to the build tree) as what it needs:
    # it would load from nowhere else. A SHARED imported target with IMPORTED_NO_SONAME links it as
    # -lwasmtime, so the host needs plain libwasmtime.so, found in its build tree's rpath, and
    # installed in ../lib, beside libcvc (the wasmtime package's, in a cvcpkg prefix).
    if(NOT TARGET cvc_wasmtime_shared)
      add_library(cvc_wasmtime_shared SHARED IMPORTED GLOBAL)
      get_target_property(_inc wasmtime::wasmtime INTERFACE_INCLUDE_DIRECTORIES)
      get_target_property(_deps wasmtime::wasmtime INTERFACE_LINK_LIBRARIES)
      set_target_properties(cvc_wasmtime_shared PROPERTIES IMPORTED_LOCATION "${_lib}"
                                                           IMPORTED_NO_SONAME TRUE
                                                           INTERFACE_INCLUDE_DIRECTORIES "${_inc}")
      if(_deps)
        set_target_properties(cvc_wasmtime_shared PROPERTIES INTERFACE_LINK_LIBRARIES "${_deps}")
      endif()
    endif()
    set(_link cvc_wasmtime_shared)
  endif()
  target_link_libraries(${target} PRIVATE ${_link})
  if(WIN32 AND _lib MATCHES "\\.dll\\.lib$")
    # wasmtime.dll's import library: the DLL goes beside the target, which runs (and its tests are
    # listed) from there, and is installed with it.
    get_filename_component(_libdir "${_lib}" DIRECTORY)
    find_file(_CVC_WASMTIME_DLL wasmtime.dll PATHS "${_libdir}" "${_libdir}/../bin" NO_DEFAULT_PATH)
    if(_CVC_WASMTIME_DLL)
      add_custom_command(TARGET ${target} POST_BUILD
                         COMMAND "${CMAKE_COMMAND}" -E copy_if_different "${_CVC_WASMTIME_DLL}"
                                 "$<TARGET_FILE_DIR:${target}>")
      if(_w_INSTALL)
        install(FILES "${_CVC_WASMTIME_DLL}" DESTINATION bin COMPONENT ${target})
      endif()
    endif()
  elseif(WIN32)
    # wasmtime's static library: its declarations without dllimport, and what it links on Windows.
    target_compile_definitions(${target} PRIVATE LIBWASM_STATIC)
    target_link_libraries(${target} PRIVATE ws2_32 advapi32 userenv ntdll shell32 ole32 bcrypt)
  elseif(UNIX AND NOT APPLE AND _lib MATCHES "\\.a$")
    target_link_libraries(${target} PRIVATE pthread dl m)
  endif()
  message(STATUS "${target}: WebAssembly guests (wasmtime: ${_lib})")
endfunction()
