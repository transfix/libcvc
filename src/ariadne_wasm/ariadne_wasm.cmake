# WebAssembly guests in Ariadne documents (the wasm engine): docs/roadmap/ARIADNE_SCRIPTING_HOSTS.md
# §8.2-8.7, phase G1.
#
#   cvc_ariadne_wasm_host(<target> [INSTALL])
#
# compiles the engine (wasm_engine.cpp, the CBOR codec and the wasmtime runtime) into a target that
# already links cvc, links wasmtime's C API (the cvcpkg `wasmtime` package: wasmtime::wasmtime) and
# defines CVC_ARIADNE_WASM, so a host's main.cpp turns its guests on. On Windows, with wasmtime.dll's
# import library, the DLL is copied beside the target; with the static library the target
# carries it. INSTALL also installs that DLL into bin, beside the host.
set(_CVC_ARIADNE_WASM_DIR "${CMAKE_CURRENT_LIST_DIR}")

#   cvc_find_ariadne_wasm(<required>)
#
# Look for wasmtime's C API. Sets CVC_ARIADNE_WASM_FOUND. With <required> true a miss is an error;
# otherwise the guests are left out of the host.
macro(cvc_find_ariadne_wasm required)
  if(NOT TARGET wasmtime::wasmtime)
    if(${required})
      find_package(wasmtime CONFIG REQUIRED)
    else()
      find_package(wasmtime CONFIG QUIET)
    endif()
  endif()
  if(TARGET wasmtime::wasmtime AND NOT EMSCRIPTEN)
    set(CVC_ARIADNE_WASM_FOUND TRUE)
  else()
    set(CVC_ARIADNE_WASM_FOUND FALSE)
  endif()
endmacro()

# The test documents the hosts' ctests run (wasm_block.ari and its guest).
set(CVC_ARIADNE_WASM_TESTS "${CMAKE_CURRENT_LIST_DIR}/tests")

function(cvc_ariadne_wasm_host target)
  cmake_parse_arguments(_w "INSTALL" "" "" ${ARGN})
  target_sources(${target} PRIVATE "${_CVC_ARIADNE_WASM_DIR}/wasm_engine.cpp"
                                   "${_CVC_ARIADNE_WASM_DIR}/wasmtime_runtime.cpp")
  target_include_directories(${target} PRIVATE "${_CVC_ARIADNE_WASM_DIR}")
  target_link_libraries(${target} PRIVATE wasmtime::wasmtime)
  target_compile_definitions(${target} PRIVATE CVC_ARIADNE_WASM)
  get_target_property(_lib wasmtime::wasmtime IMPORTED_LOCATION)
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
