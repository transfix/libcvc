# MSVC: a DLL's export table, without the std:: symbols.
#
# libcvc's sources carry no __declspec(dllexport), so a Windows DLL needs a
# generated export table. WINDOWS_EXPORT_ALL_SYMBOLS exports every external
# symbol in the target's objects, and a PE export table holds at most 65,535
# names. On 2026-10-24 the gRPC build's cvc.dll exported 63,073 of them on master
# and 65,122 with S7 (transfix/libcvc#789), and 44,412 of master's were in
# namespace std: std::vector<cvc::...>::_Emplace_reallocate,
# std::_Tree<...>::_Insert_node, std::function's _Func_impl for cvc's lambdas,
# the STL's inline functions and RTTI. That table kept the WebSocket transport
# off on Windows. Without them master's cvc.dll exports 18,661 names.
#
# No consumer imports those symbols. Each is an implicit template instantiation
# or an inline function, so a consumer that uses one compiles its own copy from
# the same header, and the linker takes it from the consumer's own objects
# without searching cvc.lib: of the 2,194 names that S7's 286 binaries import
# from cvc.dll, none is in std. The one way a std:: symbol has to come from the
# DLL is a std template specialized out of line in a .cpp (a std::hash<cvc::X>
# whose operator() is defined in the .cpp). libcvc has none: keep it that way,
# or define the member in the header.
#
# cvc_msvc_export_def(<target>) builds the table as WINDOWS_EXPORT_ALL_SYMBOLS
# does, as a pre-link step: `cmake -E __create_def` over the target's own objects,
# which yields the same file byte for byte. cvc_def_filter (cvc_def_filter.cpp,
# built here) then leaves out every symbol whose undecorated name is in namespace
# std, and the DLL links with /DEF: on the result. As before, a static library
# linked into the target (cvc_igl, cvc_physics_jolt) is not scanned, so its
# symbols stay private.
#
# The target keeps WINDOWS_EXPORT_ALL_SYMBOLS instead when cross-compiling (the
# filter would not run on the build machine) or with CMake older than 3.18
# ($<HOST_LINK>, which keeps /DEF: off a CUDA device link).
set(_cvc_export_def_dir "${CMAKE_CURRENT_LIST_DIR}")

function(cvc_msvc_export_def target)
  get_target_property(_type ${target} TYPE)
  if(NOT _type STREQUAL "SHARED_LIBRARY")
    return()
  endif()
  if(CMAKE_CROSSCOMPILING OR CMAKE_VERSION VERSION_LESS 3.18)
    set_target_properties(${target} PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS ON)
    return()
  endif()
  if(NOT TARGET cvc_def_filter)
    add_executable(cvc_def_filter "${_cvc_export_def_dir}/cvc_def_filter.cpp")
    target_link_libraries(cvc_def_filter PRIVATE dbghelp)
    # A build tool: keep it out of bin/, which packaging copies wholesale.
    set_target_properties(cvc_def_filter PROPERTIES RUNTIME_OUTPUT_DIRECTORY
                                                    "${CMAKE_CURRENT_BINARY_DIR}")
  endif()

  set(_dir "${CMAKE_CURRENT_BINARY_DIR}/${target}_exports/$<CONFIG>")
  file(GENERATE OUTPUT "${_dir}/objects.txt"
    CONTENT "$<JOIN:$<FILTER:$<TARGET_OBJECTS:${target}>,INCLUDE,[.]obj$>,\n>\n")
  add_custom_command(TARGET ${target} PRE_LINK
    COMMAND "${CMAKE_COMMAND}" -E __create_def "${_dir}/all.def" "${_dir}/objects.txt"
    COMMAND cvc_def_filter "${_dir}/all.def" "${_dir}/exports.def"
    VERBATIM)
  target_link_options(${target} PRIVATE "$<HOST_LINK:/DEF:${_dir}/exports.def>")
  set_property(TARGET ${target} PROPERTY CVC_EXPORT_DEF "${_dir}/exports.def")
  # Relink, and so filter again, when the filter changes.
  set_property(TARGET ${target} APPEND PROPERTY LINK_DEPENDS "$<TARGET_FILE:cvc_def_filter>")
endfunction()
