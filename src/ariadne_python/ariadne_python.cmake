# The scripting hosts' embedded Python (arix, ariglx): docs/roadmap/ARIADNE_SCRIPTING_HOSTS.md §3.
#
#   cvc_ariadne_python_host(<target>)
#
# compiles the engine (python_engine.cpp, with ariadne_python.py built in) into an executable that
# already links cvc, links libpython (Python3 Development.Embed) and defines CVC_ARIADNE_PYTHON, so
# the host's main.cpp turns on its Python. src/ari and src/arigl include this file, in-tree and in a
# standalone recipe build alike (the arix and ariglx cvcpkg recipes). The engine finds the Python it
# runs: PYTHONHOME; an installed prefix beside the executable; else, in-tree, the one found here,
# compiled in as CVC_ARIADNE_PYTHON_HOME.
set(_CVC_ARIADNE_PYTHON_DIR "${CMAKE_CURRENT_LIST_DIR}")

#   _cvc_ariadne_python_embed(<file.py> <name> <out.h>)
#
# <file.py> as `static const char <name>[]` in <out.h>, written when it changes. MSVC takes at most
# about 16 KB in one string literal, so the text goes in as adjacent raw literals of 8000 bytes each,
# which the compiler joins. Keep the file ASCII: a chunk could otherwise split a character.
function(_cvc_ariadne_python_embed file name out)
  file(READ "${file}" _text)
  set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${file}")
  string(LENGTH "${_text}" _n)
  set(_body "")
  set(_at 0)
  while(_at LESS _n)
    string(SUBSTRING "${_text}" ${_at} 8000 _chunk)
    string(APPEND _body "    R\"ARIPY(${_chunk})ARIPY\"\n")
    math(EXPR _at "${_at} + 8000")
  endwhile()
  get_filename_component(_base "${file}" NAME)
  # Built up as one quoted string, never as a list: the Python has semicolons.
  set(_header "// Generated from ${_base} by ariadne_python.cmake: edit that file, not this one.\n")
  string(APPEND _header "#pragma once\nstatic const char ${name}[] =\n${_body}    \"\";\n")
  if(EXISTS "${out}")
    file(READ "${out}" _old)
  else()
    set(_old "")
  endif()
  if(NOT _old STREQUAL _header)
    file(WRITE "${out}" "${_header}")
  endif()
endfunction()

#   cvc_find_ariadne_python(<required>)
#
# Look for the Python to embed: the interpreter and its embedding files (Python3 Development.Embed).
# Sets CVC_ARIADNE_PYTHON_FOUND. With <required> true (a recipe's -D..._WITH_PYTHON=ON) a miss is
# an error; otherwise the scripting host is left out, as an in-tree build with a Python that cannot
# be embedded does.
macro(cvc_find_ariadne_python required)
  if(${required})
    find_package(Python3 REQUIRED COMPONENTS Interpreter Development.Embed)
  else()
    find_package(Python3 QUIET COMPONENTS Interpreter Development.Embed)
  endif()
  set(CVC_ARIADNE_PYTHON_FOUND "${Python3_Development.Embed_FOUND}")
endmacro()

#   cvc_ariadne_python_host(<target> [SERVER])
#
# SERVER: a server host (arisrvx, src/arisrv): it also builds in pycvc.ariadne (ariadne_server.py)
# and the ServerLink its script thread and the relay share (server_link.cpp), and defines
# CVC_ARIADNE_PYTHON_SERVER.
function(cvc_ariadne_python_host target)
  cmake_parse_arguments(PARSE_ARGV 1 _arg "SERVER" "" "")
  set(_py "${_CVC_ARIADNE_PYTHON_DIR}/ariadne_python.py")
  file(READ "${_py}" ARIADNE_PYTHON_SOURCE)
  set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${_py}")
  set(_gen "${CMAKE_CURRENT_BINARY_DIR}/ariadne_python/${target}")
  configure_file("${_CVC_ARIADNE_PYTHON_DIR}/ariadne_python_source.h.in"
                 "${_gen}/ariadne_python_source.h" @ONLY)
  # The console (_ariadne_console) is a module of its own: ariadne_python.py goes in as a single
  # literal (configure_file), already half of MSVC's 16 KB cap.
  _cvc_ariadne_python_embed("${_CVC_ARIADNE_PYTHON_DIR}/ariadne_console.py" kAriadneConsoleSource
                            "${_gen}/ariadne_console_source.h")
  execute_process(COMMAND "${Python3_EXECUTABLE}" -c "import sys; print(sys.base_prefix)"
                  OUTPUT_VARIABLE _home OUTPUT_STRIP_TRAILING_WHITESPACE RESULT_VARIABLE _rc)
  target_sources(${target} PRIVATE "${_CVC_ARIADNE_PYTHON_DIR}/python_engine.cpp")
  target_include_directories(${target} PRIVATE "${_CVC_ARIADNE_PYTHON_DIR}" "${_gen}")
  target_link_libraries(${target} PRIVATE Python3::Python)
  target_compile_definitions(${target} PRIVATE CVC_ARIADNE_PYTHON)
  if(_arg_SERVER)
    _cvc_ariadne_python_embed("${_CVC_ARIADNE_PYTHON_DIR}/ariadne_server.py" kAriadneServerSource
                              "${_gen}/ariadne_server_source.h")
    target_sources(${target} PRIVATE "${_CVC_ARIADNE_PYTHON_DIR}/server_link.cpp")
    target_compile_definitions(${target} PRIVATE CVC_ARIADNE_PYTHON_SERVER)
  endif()
  # In-tree only: a recipe's build prefix is gone once it packs, and the installed host finds the
  # prefix's Python beside it instead (python_engine.cpp, python_home).
  if(_rc EQUAL 0 AND _home AND NOT CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
    file(TO_CMAKE_PATH "${_home}" _home)
    target_compile_definitions(${target} PRIVATE "CVC_ARIADNE_PYTHON_HOME=\"${_home}\"")
  endif()
  message(STATUS "${target}: embedded Python ${Python3_VERSION} (${Python3_EXECUTABLE})")
endfunction()

#   cvc_ariadne_python_wasm_host(<target> ZIP <out.zip> [VTK_BRIDGE])
#
# The browser's scripting host (ariglx-web, roadmap §5.1): the engine as above, but nothing is
# loaded at run time. A browser has no dlopen, so libpython3.12.a, the pycvc / pycvc_gl SWIG
# archives (bindings/pycvc, built in-tree) and every Python C extension in the deps prefix are
# linked in, and a generated registrar (cvc_ariadne_python_register_builtins) puts each in the
# inittab before the interpreter boots. The extensions come from the prefix's
# share/python-wasm/*.inittab manifests, each object with a namespace of its own
# (cvcpkg/recipes/_common/wasm_pyext.py); a numpy installed before it shipped one (cvc.5) is
# localized here, into copies under the build tree. VTK_BRIDGE also links vtk-python's
# _vtkmodules_static builtin.
#
# The pure-Python side goes into <out.zip>, which the page fetches and writes to
# /py/lib/python312.zip before the module starts (arigl-web.js). It is on sys.path from the start
# (CPython puts <home>/lib/python312.zip there), so the stdlib, pycvc, pycvc_gl and the packages'
# own .py files import through zipimport. The deps prefix is CMAKE_FIND_ROOT_PATH; the native
# Python3_EXECUTABLE runs the manifest and zip steps.
function(cvc_ariadne_python_wasm_host target)
  cmake_parse_arguments(PARSE_ARGV 1 _arg "VTK_BRIDGE" "ZIP" "")
  set(_py "${_CVC_ARIADNE_PYTHON_DIR}/ariadne_python.py")
  file(READ "${_py}" ARIADNE_PYTHON_SOURCE)
  set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${_py}")
  set(_gen "${CMAKE_CURRENT_BINARY_DIR}/ariadne_python/${target}")
  configure_file("${_CVC_ARIADNE_PYTHON_DIR}/ariadne_python_source.h.in"
                 "${_gen}/ariadne_python_source.h" @ONLY)
  _cvc_ariadne_python_embed("${_CVC_ARIADNE_PYTHON_DIR}/ariadne_console.py" kAriadneConsoleSource
                            "${_gen}/ariadne_console_source.h")

  # The deps prefix: libpython, its headers, its stdlib, and the packages' site-packages.
  set(_deps "")
  foreach(_p IN LISTS CMAKE_FIND_ROOT_PATH)
    if(EXISTS "${_p}/lib/libpython3.12.a")
      set(_deps "${_p}")
      break()
    endif()
  endforeach()
  if(NOT _deps)
    message(FATAL_ERROR "${target}: no libpython3.12.a in CMAKE_FIND_ROOT_PATH "
                        "(${CMAKE_FIND_ROOT_PATH}): install python312 for wasm there")
  endif()
  # The link takes undefined symbols as errors, so libpython3.12.a must be python312
  # 3.12.10+cvc.23 or later: the wasm bundles before it archive pyexpat and _sha2 without their
  # expat and HACL SHA-2 objects (cy-pca/cvcpkg#565). With the VTK bridge, cvc.25 or later:
  # vtk-python's wrappers bring in VTK's bundled expat, and cvc.23 and cvc.24 leave two expat
  # internals unrenamed that VTK's copy defines too (duplicate symbols; cy-pca/cvcpkg#581). A
  # recipe can't ask for a revision (cvcpkg's version ranges ignore +cvc.N), so the bundle's
  # manifest is read here. A prefix whose Python cvcpkg didn't install has none, and is left to
  # the link.
  set(_pyman "${_deps}/share/libcvc-deps/python312/manifest.yaml")
  if(EXISTS "${_pyman}")
    file(STRINGS "${_pyman}" _pyup REGEX "^  upstream_version: " LIMIT_COUNT 1)
    file(STRINGS "${_pyman}" _pyrev REGEX "^  cvc_revision: " LIMIT_COUNT 1)
    string(REGEX REPLACE "^  [a-z_]+: *" "" _pyup "${_pyup}")
    string(REGEX REPLACE "^  [a-z_]+: *" "" _pyrev "${_pyrev}")
    set(_pymin 23)
    set(_pywhy "lacks pyexpat's expat and _sha2's HACL objects")
    if(_arg_VTK_BRIDGE)
      set(_pymin 25)
      set(_pywhy "${_pywhy}, or (cvc.23, cvc.24) clashes with VTK's expat")
    endif()
    if(_pyup VERSION_LESS 3.12.10 OR (_pyup VERSION_EQUAL 3.12.10 AND _pyrev LESS _pymin))
      message(FATAL_ERROR "${target}: ${_deps} has python312 ${_pyup}+cvc.${_pyrev}, whose "
                          "libpython3.12.a ${_pywhy}: install 3.12.10+cvc.${_pymin} or later for "
                          "wasm there")
    endif()
  endif()
  set(_stdlib "${_deps}/lib/python3.12")
  set(_site "${_stdlib}/site-packages")
  find_package(Python3 REQUIRED COMPONENTS Interpreter Development.Module)
  if(NOT Python3_EXECUTABLE)
    message(FATAL_ERROR "${target}: set Python3_EXECUTABLE to a native Python 3 for the build's "
                        "Python steps")
  endif()

  # The extensions: the manifests the prefix has, plus numpy localized here when none lists it.
  set(_pyext "${_gen}/pyext")
  set(_tool "${_CVC_ARIADNE_PYTHON_DIR}/../../cvcpkg/recipes/_common/wasm_pyext.py")
  file(GLOB _manifests "${_deps}/share/python-wasm/*.inittab")
  set(_lists_numpy FALSE)
  foreach(_m IN LISTS _manifests)
    file(STRINGS "${_m}" _numpy_lines REGEX "^numpy[.]")
    if(_numpy_lines)
      set(_lists_numpy TRUE)
    endif()
  endforeach()
  if(IS_DIRECTORY "${_site}/numpy" AND NOT _lists_numpy)
    message(STATUS "${target}: numpy ships no inittab manifest; localizing copies of its extensions")
    execute_process(COMMAND "${Python3_EXECUTABLE}" "${_tool}" package "${_deps}" numpy
                            --site-packages "${_site}" --out "${_pyext}"
                    OUTPUT_QUIET RESULT_VARIABLE _rc ERROR_VARIABLE _err)
    if(NOT _rc EQUAL 0)
      message(FATAL_ERROR "${target}: localizing numpy's extensions failed:\n${_err}")
    endif()
    list(APPEND _manifests "${_pyext}/share/python-wasm/numpy.inittab")
  endif()
  set(_registrar "${_gen}/pyext_inittab.cpp")
  if(_manifests)
    execute_process(COMMAND "${Python3_EXECUTABLE}" "${_tool}" check ${_manifests}
                    RESULT_VARIABLE _rc OUTPUT_VARIABLE _out ERROR_VARIABLE _err)
    if(NOT _rc EQUAL 0)
      message(FATAL_ERROR "${target}: the Python extensions would clash:\n${_out}${_err}")
    endif()
    execute_process(COMMAND "${Python3_EXECUTABLE}" "${_tool}" registrar
                            --function cvc_ariadne_python_register_pyext --cpp "${_registrar}"
                            --objects "${_gen}/pyext_objects.txt" ${_manifests}
                    RESULT_VARIABLE _rc ERROR_VARIABLE _err)
    if(NOT _rc EQUAL 0)
      message(FATAL_ERROR "${target}: the extension registrar failed:\n${_err}")
    endif()
    file(STRINGS "${_gen}/pyext_objects.txt" _objects)
  else()
    file(WRITE "${_registrar}"
         "extern \"C\" int cvc_ariadne_python_register_pyext(void) { return 0; }\n")
    set(_objects "")
  endif()
  list(LENGTH _objects _n)
  set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${_manifests})

  # The registrar the engine calls (python_engine.cpp): pycvc's two modules, vtk-python's
  # builtin, then the extensions'.
  set(_decl "")
  set(_reg "")
  if(_arg_VTK_BRIDGE)
    string(APPEND _decl "PyObject *PyInit__vtkmodules_static(void);\n")
    string(APPEND _reg
           "  if (PyImport_AppendInittab(\"_vtkmodules_static\", PyInit__vtkmodules_static) != 0)\n"
           "    return 1;\n")
  endif()
  file(WRITE "${_gen}/builtins.cpp.in"
       "// Generated by cvc_ariadne_python_wasm_host (src/ariadne_python/ariadne_python.cmake).\n"
       "#include <Python.h>\n"
       "extern \"C\" {\n"
       "PyObject *PyInit__pycvc(void);\n"
       "PyObject *PyInit__pycvc_gl(void);\n"
       "${_decl}"
       "int cvc_ariadne_python_register_pyext(void);\n"
       "int cvc_ariadne_python_register_builtins(void) {\n"
       "  if (PyImport_AppendInittab(\"_pycvc\", PyInit__pycvc) != 0)\n"
       "    return 1;\n"
       "  // pycvc_gl is a package: its __init__.py does `from . import _pycvc_gl`.\n"
       "  if (PyImport_AppendInittab(\"pycvc_gl._pycvc_gl\", PyInit__pycvc_gl) != 0)\n"
       "    return 1;\n"
       "${_reg}"
       "  return cvc_ariadne_python_register_pyext();\n"
       "}\n"
       "}\n")
  configure_file("${_gen}/builtins.cpp.in" "${_gen}/builtins.cpp" COPYONLY)

  target_sources(${target} PRIVATE "${_CVC_ARIADNE_PYTHON_DIR}/python_engine.cpp"
                                   "${_gen}/builtins.cpp" "${_registrar}" ${_objects})
  set_source_files_properties(${_objects} PROPERTIES EXTERNAL_OBJECT TRUE)
  target_include_directories(${target} PRIVATE "${_CVC_ARIADNE_PYTHON_DIR}" "${_gen}"
                                               ${Python3_INCLUDE_DIRS})
  target_compile_definitions(${target} PRIVATE CVC_ARIADNE_PYTHON CVC_ARIADNE_PYTHON_BUILTINS
                                               "CVC_ARIADNE_PYTHON_HOME=\"/py\"")
  # The SWIG archives whole, as bindings/pycvc/wasm/link-host.sh links them; libpython and the
  # libraries its modules need (ctypes' libffi, sqlite3, ssl and hashlib's OpenSSL, the codecs').
  target_link_libraries(${target} PRIVATE "$<LINK_LIBRARY:WHOLE_ARCHIVE,pycvc,pycvc_gl>"
                                          "${_deps}/lib/libpython3.12.a")
  foreach(_lib ffi sqlite3 ssl crypto bz2 lzma z mpdec expat)
    if(EXISTS "${_deps}/lib/lib${_lib}.a")
      target_link_libraries(${target} PRIVATE "${_deps}/lib/lib${_lib}.a")
    endif()
  endforeach()
  if(_arg_VTK_BRIDGE)
    file(GLOB _vtkstatic "${_deps}/lib/_vtkmodules_static.a")
    if(NOT _vtkstatic)
      message(FATAL_ERROR "${target}: VTK_BRIDGE, but no _vtkmodules_static.a in ${_deps}/lib "
                          "(vtk-python-cp312 for wasm)")
    endif()
    # The builtin only registers vtkmodules' submodules: their wrappers are the prefix's
    # libvtk*Python.a, and those take VTK modules the host itself doesn't link (IONetCDF, ...). So
    # every archive in the prefix goes on the line, as link-host.sh links them; wasm-ld takes from
    # an archive only what something references. The builtin itself is a plain archive: the
    # registrar references its init function, and CMake repeats the archives of a link cycle, which
    # would whole-archive it twice.
    file(GLOB _deps_archives "${_deps}/lib/lib*.a")
    list(REMOVE_ITEM _deps_archives "${_deps}/lib/libpython3.12.a")
    target_link_libraries(${target} PRIVATE ${_vtkstatic} ${_deps_archives})
  endif()

  # The zip: the stdlib (less its tests and the GUI and build tools), the prefix's pure-Python
  # packages with no extension objects or headers, pycvc.py and _pycvc_ari.py (pycvc.ari, the
  # Host pycvc_gl.ariadne is), the pycvc_gl package as staged for the build-tree tests
  # (bindings/pycvc/pkg), and vtk-python's _vtk.zip contents with the bridge.
  set(_zip_args --stdlib "${_stdlib}" --site-packages "${_site}"
                --add "${CMAKE_BINARY_DIR}/bindings/pycvc/pycvc.py"
                --add "${CMAKE_BINARY_DIR}/bindings/pycvc/_pycvc_ari.py"
                --add "${CMAKE_BINARY_DIR}/bindings/pycvc/pkg/pycvc_gl")
  if(_arg_VTK_BRIDGE)
    list(APPEND _zip_args --merge-zip "${_site}/_vtk.zip")
  else()
    list(APPEND _zip_args --exclude _vtk.zip)
  endif()
  add_custom_command(
    OUTPUT "${_arg_ZIP}"
    COMMAND "${Python3_EXECUTABLE}" "${_CVC_ARIADNE_PYTHON_DIR}/make_python_zip.py"
            --out "${_arg_ZIP}" ${_zip_args}
    DEPENDS "${_CVC_ARIADNE_PYTHON_DIR}/make_python_zip.py" pycvc pycvc_gl_pkg
            "${CMAKE_BINARY_DIR}/bindings/pycvc/_pycvc_ari.py"
    COMMENT "Zipping ${target}'s Python: the stdlib, pycvc, pycvc_gl and the prefix's packages"
    VERBATIM)
  add_custom_target(${target}_python_zip DEPENDS "${_arg_ZIP}")
  add_dependencies(${target} ${target}_python_zip)
  message(STATUS "${target}: embedded Python ${Python3_VERSION} for the browser, ${_n} extension "
                 "object(s) linked in, the zip at ${_arg_ZIP}")
endfunction()

# The in-tree ctest entries' fixtures (tests/) and the pycvc modules they name with --python-path
# (the build tree's pycvc, and its pycvc_gl package layout).
set(CVC_ARIADNE_PYTHON_TESTS "${_CVC_ARIADNE_PYTHON_DIR}/tests")
set(CVC_ARIADNE_PYTHON_TEST_PATH --python-path "${CMAKE_BINARY_DIR}/bindings/pycvc/pkg"
    --python-path "${CMAKE_BINARY_DIR}/bindings/pycvc")

#   cvc_ariadne_python_test_properties(<test>...)
#
# The environment of a scripting host's ctest entries. On Windows, Python 3.8+ loads an extension
# module's DLLs without PATH: pycvc's ctest_site goes first on PYTHONPATH, and its sitecustomize
# adds CVC_DLL_DIRS (the build's bin/ and each CMAKE_PREFIX_PATH entry's) with os.add_dll_directory;
# the embedded Python's own DLL, beside python.exe, goes on PATH. Elsewhere RUNPATH finds them all.
function(cvc_ariadne_python_test_properties)
  set(_env "PYTHONIOENCODING=utf-8")
  if(WIN32)
    set(_dirs "${CMAKE_BINARY_DIR}/bin")
    foreach(_p IN LISTS CMAKE_PREFIX_PATH)
      file(TO_CMAKE_PATH "${_p}" _p)
      if(IS_DIRECTORY "${_p}/bin")
        list(APPEND _dirs "${_p}/bin")
      endif()
    endforeach()
    list(JOIN _dirs "\\;" _dirs)
    list(APPEND _env "PYTHONPATH=${_CVC_ARIADNE_PYTHON_DIR}/../../bindings/pycvc/ctest_site"
         "CVC_DLL_DIRS=${_dirs}")
    get_filename_component(_python_dir "${Python3_EXECUTABLE}" DIRECTORY)
    set_tests_properties(${ARGN} PROPERTIES ENVIRONMENT_MODIFICATION
                         "PATH=path_list_prepend:${_python_dir}")
  endif()
  set_tests_properties(${ARGN} PROPERTIES ENVIRONMENT "${_env}")
endfunction()
