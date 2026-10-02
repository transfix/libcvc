# cvcGLWasm.cmake -- cvcgl_wasm_app(): opt a cvcGL WebAssembly app into the browser-side speedups.
#
# Included by cvcGLConfig.cmake (installed next to it in lib/cmake/cvcGL/) and, in-tree, by
# src/cvcGL/CMakeLists.txt. One line per app:
#
#   cvcgl_wasm_app(<target>
#     [STATE_SHIM ON|OFF]               # default ON
#     [STATE_SHIM_MODE on|verify|norb]  # build-time default when the URL / page say nothing; default on
#     [MIMALLOC AUTO|ON|OFF]            # default AUTO = ON iff cvcGL was built -pthread (wasm-mt)
#     [FRAME_YIELD_LOCKED AUTO|ON|OFF]) # default AUTO = ON iff it links -sASYNCIFY_IGNORE_INDIRECT
#
# Outside Emscripten it does nothing, so an app can call it unconditionally. Feature-test it with
#   if(COMMAND cvcgl_wasm_app)   (and CVCGL_WASM_APP_FEATURES for later keywords).
#
#   STATE_SHIM  links webgl_state_shadow.js as a --pre-js: GL state queries ImGui's OpenGL3 backend
#               (ImGuiOverlay, Ariadne's ImGuiBackend) and VTK make every frame are answered from a
#               client-side shadow instead of a synchronous GPU-process round-trip. It patches
#               every WebGL context on the page; a page can still turn it off with ?glshim=0.
#   STATE_SHIM_MODE  the mode when neither the URL (?glshim=) nor the page (Module.glStateShadow)
#               chooses one. Not `on`: a generated <target>_glshim_default.js sets
#               Module.glStateShadowDefault ahead of the shim.
#   MIMALLOC    links -sMALLOC=mimalloc. dlmalloc serialises every malloc/free on one global lock,
#               which threads contend on; a single-threaded build has no lock to contend on, and
#               mimalloc is bigger and uses more memory, so AUTO turns it on for wasm-mt only --
#               and never under -fsanitize=address, which emcc refuses to combine with mimalloc.
#               If the target already links -sMALLOC=mimalloc, that counts as ON (nothing added);
#               any other -sMALLOC= of its own is kept (a warning says so). One the app adds AFTER
#               this call comes later on the link line, and emcc keeps the last -s value, so it
#               wins too.
#   FRAME_YIELD_LOCKED  links a generated <target>_frameyield_lock.js --pre-js that sets
#               Module.cvcglFrameYieldLocked = 1: every cvcGL window then keeps FrameYield::App for
#               good -- ?frameyield=vtk and setFrameYield(Vtk) are refused, and the watchdog
#               reports instead of turning VTK 9.5's in-render emscripten_sleep back on. That sleep
#               is reached through a virtual call, so under -sASYNCIFY_IGNORE_INDIRECT=1 it TRAPS.
#               AUTO locks exactly when the target's own link line carries that flag -- LINK_OPTIONS,
#               LINK_FLAGS[_<CFG>], -s... items in target_link_libraries, or the global linker flags (checked
#               once the calling directory is done, CMake >= 3.19; at the call before that). Pass
#               ON when the flag comes from somewhere the check cannot see (a dependency's
#               INTERFACE_LINK_OPTIONS, a generator expression). The C++ equivalent is
#               SceneRenderer / ViewportManager::lockFrameYield().
#
# Deliberately NOT options here (docs/CVCGL_WASM.md):
#   -sASYNCIFY_IGNORE_INDIRECT -- only safe after an audit of every sleep the app can reach; a
#     deferred lint (CMake >= 3.19) warns when a cvcgl_wasm_app target links it (and AUTO locks
#     FrameYield::App, above).
#   FrameYield (VTK's in-render emscripten_sleep off) -- a C++ setting next to the app's own loop:
#     cvc::gl::SceneRenderer / ViewportManager::setFrameYield(FrameYield::App). FRAME_YIELD_LOCKED
#     only makes App permanent; it does not switch the loop's contract on.
#
# Where the JS lives: the GLOBAL property CVCGL_WASM_DATA_DIR (src/cvcGL/wasm in-tree,
# <prefix>/share/cvcGL/wasm installed -- resolved from the installed config's own location when the
# consumer configures, so no absolute path is baked into the package). Whether cvcGL was built
# -pthread: the GLOBAL property CVCGL_WASM_PTHREADS.

# Keywords cvcgl_wasm_app understands, for consumers that need a newer one:
#   if("MIMALLOC" IN_LIST CVCGL_WASM_APP_FEATURES)
# Set on every include, ahead of the guard: it is a directory-scoped variable, so a second
# find_package(cvcGL) from a sibling directory must see it too (the functions are global).
set(CVCGL_WASM_APP_FEATURES STATE_SHIM STATE_SHIM_MODE MIMALLOC FRAME_YIELD_LOCKED)
include_guard(GLOBAL)

function(cvcgl_wasm_app target)
  if(NOT EMSCRIPTEN)
    return()
  endif()
  if(NOT TARGET ${target})
    message(FATAL_ERROR "cvcgl_wasm_app: '${target}' is not a target")
  endif()
  cmake_parse_arguments(PARSE_ARGV 1 _cwa ""
                        "STATE_SHIM;STATE_SHIM_MODE;MIMALLOC;FRAME_YIELD_LOCKED" "")
  if(_cwa_UNPARSED_ARGUMENTS)
    message(FATAL_ERROR "cvcgl_wasm_app(${target}): unknown arguments: ${_cwa_UNPARSED_ARGUMENTS}")
  endif()
  foreach(_k IN LISTS _cwa_KEYWORDS_MISSING_VALUES)
    message(FATAL_ERROR "cvcgl_wasm_app(${target}): ${_k} needs a value")
  endforeach()

  # STATE_SHIM: any CMake boolean, default ON.
  set(_shim ON)
  if(DEFINED _cwa_STATE_SHIM)
    if(_cwa_STATE_SHIM)
      set(_shim ON)
    else()
      set(_shim OFF)
    endif()
  endif()
  set(_mode on)
  if(DEFINED _cwa_STATE_SHIM_MODE)
    string(TOLOWER "${_cwa_STATE_SHIM_MODE}" _mode)
    if(NOT _mode MATCHES "^(on|verify|norb)$")
      message(FATAL_ERROR "cvcgl_wasm_app(${target}): STATE_SHIM_MODE must be on, verify or norb "
                          "(got '${_cwa_STATE_SHIM_MODE}'; use STATE_SHIM OFF to leave the shim out)")
    endif()
  endif()
  set(_malloc AUTO)
  if(DEFINED _cwa_MIMALLOC)
    string(TOUPPER "${_cwa_MIMALLOC}" _malloc)
    if(NOT _malloc STREQUAL "AUTO")
      if(_cwa_MIMALLOC)
        set(_malloc ON)
      else()
        set(_malloc OFF)
      endif()
    endif()
  endif()
  # FRAME_YIELD_LOCKED: AUTO (resolved against the final link options) or any CMake boolean.
  set(_lock AUTO)
  if(DEFINED _cwa_FRAME_YIELD_LOCKED)
    string(TOUPPER "${_cwa_FRAME_YIELD_LOCKED}" _lock)
    if(NOT _lock STREQUAL "AUTO")
      if(_cwa_FRAME_YIELD_LOCKED)
        set(_lock ON)
      else()
        set(_lock OFF)
      endif()
    endif()
  endif()

  # The flags this target is compiled and linked with that this call can see: the
  # target's own and the global ones for the build type. Not the directory's: a target's
  # LINK_OPTIONS / COMPILE_OPTIONS start as its directory's when it is created, and a
  # directory option added later (or the calling directory's) is not on its link line.
  string(TOUPPER "${CMAKE_BUILD_TYPE}" _cfg)
  _cvcgl_wasm_app_target_flags(${target} _flags)
  get_target_property(_v ${target} COMPILE_OPTIONS)
  if(_v)
    string(APPEND _flags ";${_v}")
  endif()
  foreach(_v CMAKE_C_FLAGS CMAKE_CXX_FLAGS CMAKE_EXE_LINKER_FLAGS)
    string(APPEND _flags " ${${_v}} ${${_v}_${_cfg}}")
  endforeach()
  set(_asan OFF)
  if(_flags MATCHES "(^|[; ])-fsanitize=[^ ;]*address")
    set(_asan ON)
  endif()

  if(_malloc STREQUAL "AUTO")
    get_property(_pthreads GLOBAL PROPERTY CVCGL_WASM_PTHREADS)
    if(_pthreads AND _asan)
      message(STATUS "cvcgl_wasm_app(${target}): MIMALLOC AUTO -> OFF: built with "
                     "-fsanitize=address, which emcc refuses to combine with mimalloc")
      set(_malloc OFF)
    elseif(_pthreads)
      set(_malloc ON)
    else()
      set(_malloc OFF)
    endif()
  elseif(_malloc AND _asan)
    message(WARNING "cvcgl_wasm_app(${target}): MIMALLOC ON with -fsanitize=address -- emcc "
                    "refuses mimalloc under ASan and will stop at the link. Pass MIMALLOC AUTO "
                    "(OFF under ASan) or OFF.")
  endif()

  if(_shim)
    get_property(_dir GLOBAL PROPERTY CVCGL_WASM_DATA_DIR)
    set(_js "${_dir}/webgl_state_shadow.js")
    if(NOT _dir OR NOT EXISTS "${_js}")
      message(FATAL_ERROR
        "cvcgl_wasm_app(${target}): this cvcGL has no webgl_state_shadow.js (looked in "
        "'${_dir}'). Install a cvcGL that ships share/cvcGL/wasm/, or pass STATE_SHIM OFF.")
    endif()
    if(NOT _mode STREQUAL "on")
      # Read by the shim when neither the URL nor the page chose a mode. Generated, so it lives in
      # the consumer's build tree; file(GENERATE) only rewrites it when the content changes.
      set(_def "${CMAKE_CURRENT_BINARY_DIR}/${target}_glshim_default.js")
      file(GENERATE OUTPUT "${_def}" CONTENT
        "// Generated by cvcgl_wasm_app(${target} STATE_SHIM_MODE ${_mode}): the build-time default\n// mode of webgl_state_shadow.js. ?glshim= and Module.glStateShadow still win.\nModule['glStateShadowDefault'] = '${_mode}';\n")
      target_link_options(${target} PRIVATE "SHELL:--pre-js \"${_def}\"")
      set_property(TARGET ${target} APPEND PROPERTY LINK_DEPENDS "${_def}")
    endif()
    target_link_options(${target} PRIVATE "SHELL:--pre-js \"${_js}\"")
    set_property(TARGET ${target} APPEND PROPERTY LINK_DEPENDS "${_js}")
  endif()

  # An allocator the app already chose (target or global flags) is kept. mimalloc already
  # there is simply ON: nothing to add, nothing to warn about.
  if(_flags MATCHES "(^|[; :])-s[ ]*MALLOC=([A-Za-z0-9_-]*)")
    set(_theirs "${CMAKE_MATCH_2}")
    if(_theirs STREQUAL "mimalloc")
      set(_malloc ON)
    else()
      if(_malloc)
        message(WARNING "cvcgl_wasm_app(${target}): the target already links -sMALLOC=${_theirs}; "
                        "keeping it instead of mimalloc (pass MIMALLOC OFF to silence this)")
      endif()
      set(_malloc OFF)
    endif()
  elseif(_malloc)
    target_link_options(${target} PRIVATE "-sMALLOC=mimalloc")
  endif()

  set_target_properties(${target} PROPERTIES
    CVCGL_WASM_APP ON
    CVCGL_WASM_APP_STATE_SHIM ${_shim}
    CVCGL_WASM_APP_STATE_SHIM_MODE ${_mode}
    CVCGL_WASM_APP_MIMALLOC ${_malloc}
    CVCGL_WASM_APP_FRAME_YIELD_LOCKED ${_lock}) # AUTO until the deferred check resolves it
  if(_lock STREQUAL "ON")
    _cvcgl_wasm_app_lock_frame_yield(${target} "FRAME_YIELD_LOCKED ON")
  endif()

  # Check the FINAL link options once the calling directory is done (the app may add flags after
  # this call): -sASYNCIFY_IGNORE_INDIRECT is app-specific and easy to get wrong, and it decides
  # FRAME_YIELD_LOCKED AUTO. Before CMake 3.19 there is no DEFER: check what is there now.
  if(NOT CMAKE_VERSION VERSION_LESS 3.19)
    cmake_language(EVAL CODE "cmake_language(DEFER CALL _cvcgl_wasm_app_finish [[${target}]])")
  else()
    _cvcgl_wasm_app_finish(${target})
  endif()
endfunction()

# FrameYield::App for good on every cvcGL window of <target>: a generated --pre-js the
# ViewportManager reads once per window (Module.cvcglFrameYieldLocked).
function(_cvcgl_wasm_app_lock_frame_yield target why)
  get_target_property(_done ${target} CVCGL_WASM_APP_FRAME_YIELD_LOCK_JS)
  if(_done)
    return()
  endif()
  set(_f "${CMAKE_CURRENT_BINARY_DIR}/${target}_frameyield_lock.js")
  file(GENERATE OUTPUT "${_f}" CONTENT
    "// Generated by cvcgl_wasm_app(${target}): ${why}.\n// Every cvcGL window keeps FrameYield::App: ?frameyield=vtk, setFrameYield(Vtk) and the watchdog\n// cannot turn VTK 9.5's in-render emscripten_sleep -- reached through a virtual call, so a trap\n// under -sASYNCIFY_IGNORE_INDIRECT=1 -- back on (cvc/gl/ViewportManager.h, lockFrameYield).\nModule['cvcglFrameYieldLocked'] = 1;\n")
  target_link_options(${target} PRIVATE "SHELL:--pre-js \"${_f}\"")
  set_property(TARGET ${target} APPEND PROPERTY LINK_DEPENDS "${_f}")
  set_target_properties(${target} PROPERTIES
    CVCGL_WASM_APP_FRAME_YIELD_LOCKED ON
    CVCGL_WASM_APP_FRAME_YIELD_LOCK_JS "${_f}")
endfunction()

# The link flags on ${target}'s own link line: LINK_OPTIONS, LINK_FLAGS[_<CFG>], and the
# -s... items given to target_link_libraries (a common emscripten idiom).
function(_cvcgl_wasm_app_target_flags target out)
  string(TOUPPER "${CMAKE_BUILD_TYPE}" _cfg)
  set(_r "")
  foreach(_p LINK_OPTIONS LINK_FLAGS LINK_FLAGS_${_cfg})
    get_target_property(_v ${target} ${_p})
    if(_v)
      string(APPEND _r ";${_v}")
    endif()
  endforeach()
  get_target_property(_v ${target} LINK_LIBRARIES)
  if(_v)
    foreach(_i IN LISTS _v)
      if(_i MATCHES "^-s")
        string(APPEND _r ";${_i}")
      endif()
    endforeach()
  endif()
  set(${out} "${_r}" PARENT_SCOPE)
endfunction()

function(_cvcgl_wasm_app_finish target)
  string(TOUPPER "${CMAKE_BUILD_TYPE}" _cfg)
  _cvcgl_wasm_app_target_flags(${target} _lo)
  # Whole tokens only: -sASYNCIFY_IGNORE_INDIRECT[=1] (also "-s X" and SHELL:), not =0.
  set(_all "${_lo};${CMAKE_EXE_LINKER_FLAGS} ${CMAKE_EXE_LINKER_FLAGS_${_cfg}}")
  set(_ii OFF)
  if(_all MATCHES "(^|[; :])-s[ ]*ASYNCIFY_IGNORE_INDIRECT(=1)?($|[; ])")
    set(_ii ON)
  endif()
  get_target_property(_lock ${target} CVCGL_WASM_APP_FRAME_YIELD_LOCKED)
  if(_lock STREQUAL "AUTO")
    if(_ii)
      _cvcgl_wasm_app_lock_frame_yield(${target}
        "it links -sASYNCIFY_IGNORE_INDIRECT=1 (FRAME_YIELD_LOCKED AUTO)")
      set(_lock ON)
    else()
      set_property(TARGET ${target} PROPERTY CVCGL_WASM_APP_FRAME_YIELD_LOCKED OFF)
      set(_lock OFF)
    endif()
  endif()
  if(NOT _ii)
    return()
  endif()
  if(_lock)
    string(CONCAT _lock_note
      "FrameYield::App is locked for it (Module.cvcglFrameYieldLocked, FRAME_YIELD_LOCKED), so "
      "neither ?frameyield=vtk nor the watchdog can bring that yield back; the app's loop must "
      "still call emscripten_sleep(0) once per frame on every path that renders.")
  else()
    string(CONCAT _lock_note
      "FRAME_YIELD_LOCKED is OFF, so ?frameyield=vtk or a watchdog trip turns that yield back on "
      "and the app TRAPS, unless its C++ calls lockFrameYield() on every window: prefer "
      "FRAME_YIELD_LOCKED AUTO or ON.")
  endif()
  message(WARNING
    "cvcgl_wasm_app(${target}): links -sASYNCIFY_IGNORE_INDIRECT=1. That is only correct while "
    "every emscripten_sleep (and every other async import) the app can reach on the main thread is "
    "reached through DIRECT calls -- a sleep reached through a virtual call, function pointer or "
    "std::function traps at its unwind. Known indirect sleepers in a cvcGL app: VTK 9.5's "
    "in-render yield (reached through the virtual Render(); off only in FrameYield::App). "
    "${_lock_note} Also cvc::net's fetch (Ariadne http verbs) and anything an ImGuiOverlay draw "
    "callback runs. Work through the checklist in docs/CVCGL_WASM.md "
    "(\"-sASYNCIFY_IGNORE_INDIRECT\") before shipping.")
  if(_all MATCHES "(^|[; :])-s[ ]*FETCH(=1)?($|[; ])")
    message(WARNING
      "cvcgl_wasm_app(${target}): -sASYNCIFY_IGNORE_INDIRECT=1 TOGETHER WITH -sFETCH=1. cvc::net's "
      "blocking fetch spins on emscripten_sleep and is reached from Ariadne's http verbs through "
      "std::function intrinsics -- an indirect call -- so any http(s):// load in this app will trap "
      "unless you have proven fetch unreachable (docs/CVCGL_WASM.md, checklist item 2).")
  endif()
endfunction()
