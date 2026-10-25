# The gallery's demo list, examples/web/demos.json, made into what the native programs read:
#
#   demos_catalog.inc  cvc_demos_menu's table, the C++ demos' Demos menu: slug, title, desc, doc
#   gallery.json       arigl's catalog ($CVC_ARIGL_GALLERY, see src/arigl/gallery.h) for this build
#                      tree: each .ari demo's document from the sources, every other demo as the
#                      program beside arigl
#   share/libcvc-examples/gallery.json, installed: the same, its documents relative to it
#
# The browser gallery writes its own, from what it deployed (examples/web/build-pages.py). The
# terminal pages (ari, ari_tvision) are browser-only, so the native catalogs leave them out.
# Sets _demos_gallery_json: the build tree's gallery.json.

file(READ "${CMAKE_CURRENT_SOURCE_DIR}/web/demos.json" _demos_json)
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
             "${CMAKE_CURRENT_SOURCE_DIR}/web/demos.json")

# The body of a JSON or C string literal: backslashes and quotes escaped.
function(_demos_quote out text)
  string(REPLACE "\\" "\\\\" text "${text}")
  string(REPLACE "\"" "\\\"" text "${text}")
  set(${out} "${text}" PARENT_SCOPE)
endfunction()

# Write `content` to `path` only when it changed, so an unchanged list rebuilds nothing.
function(_demos_write path content)
  if(EXISTS "${path}")
    file(READ "${path}" _old)
    if(_old STREQUAL content)
      return()
    endif()
  endif()
  file(WRITE "${path}" "${content}")
endfunction()

set(_demos_inc "// Generated from examples/web/demos.json by examples/common/demos.cmake.\n")
set(_demos_tree "")
set(_demos_installed "")
set(_demos_sep "")
string(JSON _demos_n LENGTH "${_demos_json}" order)
math(EXPR _demos_last "${_demos_n} - 1")
foreach(_i RANGE ${_demos_last})
  string(JSON _slug GET "${_demos_json}" order ${_i})
  foreach(_field title desc doc app)
    string(JSON _${_field} ERROR_VARIABLE _err GET "${_demos_json}" demos ${_slug} ${_field})
    if(_err)
      set(_${_field} "")
    endif()
  endforeach()
  if(_title STREQUAL "")
    set(_title "${_slug}")
  endif()
  if(_app STREQUAL "")
    set(_doc "") # a doc means something only with the app that opens it
  elseif(NOT _app STREQUAL "arigl")
    continue() # another app's document (arigl_term's, in the browser) is not native arigl's
  endif()

  # The C table: non-ASCII folded to '-' (ImGui's UI font, and MSVC without /utf-8).
  string(REGEX REPLACE "[^ -~]+" "-" _ctitle "${_title}")
  string(REGEX REPLACE "[^ -~]+" "-" _cdesc "${_desc}")
  _demos_quote(_ctitle "${_ctitle}")
  _demos_quote(_cdesc "${_cdesc}")
  string(APPEND _demos_inc "{\"${_slug}\", \"${_ctitle}\", \"${_cdesc}\", \"${_doc}\"},\n")

  # arigl's catalog.
  _demos_quote(_jtitle "${_title}")
  _demos_quote(_jdesc "${_desc}")
  set(_head "{\"slug\": \"${_slug}\", \"title\": \"${_jtitle}\", \"desc\": \"${_jdesc}\"")
  if(NOT _doc STREQUAL "")
    string(APPEND _demos_tree
           "${_demos_sep}  ${_head}, \"doc\": \"${CMAKE_CURRENT_SOURCE_DIR}/ariadne/documents/${_doc}.ari\"}")
    string(APPEND _demos_installed "${_demos_sep}  ${_head}, \"doc\": \"ariadne/documents/${_doc}.ari\"}")
    set(_demos_sep ",\n")
  elseif(NOT _slug MATCHES "^ari(_tvision)?$")
    string(APPEND _demos_tree "${_demos_sep}  ${_head}, \"program\": \"${_slug}\"}")
    string(APPEND _demos_installed "${_demos_sep}  ${_head}, \"program\": \"${_slug}\"}")
    set(_demos_sep ",\n")
  endif()
endforeach()

_demos_write("${CMAKE_CURRENT_BINARY_DIR}/demos_catalog.inc" "${_demos_inc}")
set(_demos_gallery_json "${CMAKE_CURRENT_BINARY_DIR}/gallery.json")
_demos_write("${_demos_gallery_json}" "{\"demos\": [\n${_demos_tree}\n]}\n")
_demos_write("${CMAKE_CURRENT_BINARY_DIR}/gallery.installed.json"
             "{\"demos\": [\n${_demos_installed}\n]}\n")
