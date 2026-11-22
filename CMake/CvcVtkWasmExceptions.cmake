include_guard(GLOBAL)

# cvc_vtk_wasm_exceptions(): call it after each find_package(VTK). In a browser build it takes
# Emscripten's JS exceptions back off the programs that link VTK.
#
# VTK 9.5's Common/Misc/CMakeLists.txt compiles its exprtk parser with -fexceptions. It then gives
# every program that links VTK::CommonMisc two link options: -sDISABLE_EXCEPTION_CATCHING=0 and
# -fexceptions.
#
# libcvc links with wasm exceptions (-fwasm-exceptions), and emcc refuses
# DISABLE_EXCEPTION_CATCHING=0 alongside them ("not compatible with -fwasm-exceptions").
#
# cvcpkg builds its wasm vtk with wasm exceptions (cy-pca/cvcpkg#624), exprtk included: EMCC_CFLAGS'
# -fwasm-exceptions overrides that -fexceptions. So the two options describe a VTK we don't have,
# and they go.
#
# Imported targets belong to the directory that found them, which is why every directory that runs
# find_package(VTK) calls this.
function(cvc_vtk_wasm_exceptions)
  if(NOT EMSCRIPTEN OR NOT TARGET VTK::CommonMisc)
    return()
  endif()
  get_target_property(_opts VTK::CommonMisc INTERFACE_LINK_OPTIONS)
  if(NOT _opts)
    return()
  endif()
  list(REMOVE_ITEM _opts "-sDISABLE_EXCEPTION_CATCHING=0" "-fexceptions")
  set_property(TARGET VTK::CommonMisc PROPERTY INTERFACE_LINK_OPTIONS "${_opts}")
endfunction()
