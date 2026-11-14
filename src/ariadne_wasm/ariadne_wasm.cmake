# WebAssembly guests in Ariadne documents (the wasm engine): docs/roadmap/ARIADNE_SCRIPTING_HOSTS.md
# §8.2-8.7, phases G1 and G2.
#
#   cvc_ariadne_wasm_host(<target> [INSTALL])
#
# compiles the engine (wasm_engine.cpp, the CBOR codec, wasm_binary.cpp, the WASI stubs and the
# runtime) into a target that already links cvc, and defines CVC_ARIADNE_WASM, so a host's
# main.cpp turns its guests on. Natively the runtime is wasmtime's C API (the cvcpkg `wasmtime`
# package: wasmtime::wasmtime), linked here. On Windows, with wasmtime.dll's import library, the
# DLL is copied beside the target; with the static library the target carries it. INSTALL also
# installs that DLL into bin, beside the host. Under Emscripten the runtime is the page's own
# WebAssembly (browser_runtime.cpp), which runs guests transformed ahead
# (cvc_ariadne_wasm_transform), and nothing is linked.
set(_CVC_ARIADNE_WASM_DIR "${CMAKE_CURRENT_LIST_DIR}")

#   cvc_find_ariadne_wasm(<required>)
#
# Look for wasmtime's C API. Sets CVC_ARIADNE_WASM_FOUND. With <required> true a miss is an error;
# otherwise the guests are left out of the host. Under Emscripten there is nothing to find: the
# browser backend needs no library.
macro(cvc_find_ariadne_wasm required)
  if(EMSCRIPTEN)
    set(CVC_ARIADNE_WASM_FOUND TRUE)
  else()
    if(NOT TARGET wasmtime::wasmtime)
      if(${required})
        find_package(wasmtime CONFIG REQUIRED)
      else()
        find_package(wasmtime CONFIG QUIET)
      endif()
    endif()
    if(TARGET wasmtime::wasmtime)
      set(CVC_ARIADNE_WASM_FOUND TRUE)
    else()
      set(CVC_ARIADNE_WASM_FOUND FALSE)
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

#   cvc_ariadne_wasm_transform(<out> <in>)
#
# The install-time transform (guest_runtime.h; ARIADNE_SCRIPTING_HOSTS.md §8.5): <in>, a .wasm or
# .wat guest, becomes <out>, a .wasm the browser backend runs. It has a gas check at every
# function's entry, loop header and return (an import, ari.log_execution), and Asyncify around
# every `ari` import, so the guest can pause at either. wasmtime runs <out> as it runs <in>. This
# is the command a guest's author runs, with the features below:
#
#   wasm-opt <in> --log-execution=ari --asyncify --pass-arg=asyncify-imports@ari.* -O1 -o <out>
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
  get_filename_component(_dir "${out}" DIRECTORY)
  add_custom_command(OUTPUT "${out}"
    COMMAND "${CMAKE_COMMAND}" -E make_directory "${_dir}"
    COMMAND "${CVC_WASM_OPT}" "${in}" ${CVC_WASM_GUEST_FEATURES} --log-execution=ari --asyncify
            "--pass-arg=asyncify-imports@ari.*" -O1 -o "${out}"
    DEPENDS "${in}"
    COMMENT "wasm-opt: ${in}, for the browser (gas checks, Asyncify)"
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

function(cvc_ariadne_wasm_host target)
  cmake_parse_arguments(_w "INSTALL" "" "" ${ARGN})
  target_sources(${target} PRIVATE "${_CVC_ARIADNE_WASM_DIR}/wasm_engine.cpp"
                                   "${_CVC_ARIADNE_WASM_DIR}/wasm_binary.cpp"
                                   "${_CVC_ARIADNE_WASM_DIR}/wasi_preview1.cpp")
  target_include_directories(${target} PRIVATE "${_CVC_ARIADNE_WASM_DIR}")
  target_compile_definitions(${target} PRIVATE CVC_ARIADNE_WASM)
  if(EMSCRIPTEN)
    target_sources(${target} PRIVATE "${_CVC_ARIADNE_WASM_DIR}/browser_runtime.cpp")
    message(STATUS "${target}: WebAssembly guests (the browser's own WebAssembly)")
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
