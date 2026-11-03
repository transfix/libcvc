# The scripting hosts' embedded Python (arix, ariglx): docs/roadmap/ARIADNE_SCRIPTING_HOSTS.md §3.
#
#   cvc_ariadne_python_host(<target>)
#
# compiles the engine (python_engine.cpp, with ariadne_python.py built in) into an executable that
# already links cvc, links libpython (Python3 Development.Embed) and defines CVC_ARIADNE_PYTHON, so
# the host's main.cpp turns on its Python. src/ari and src/arigl include this file, in-tree and in a
# standalone recipe build alike. The engine finds the Python it runs: PYTHONHOME; an installed
# prefix beside the executable; else the one found here, compiled in as CVC_ARIADNE_PYTHON_HOME.
set(_CVC_ARIADNE_PYTHON_DIR "${CMAKE_CURRENT_LIST_DIR}")

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

function(cvc_ariadne_python_host target)
  set(_py "${_CVC_ARIADNE_PYTHON_DIR}/ariadne_python.py")
  file(READ "${_py}" ARIADNE_PYTHON_SOURCE)
  set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${_py}")
  set(_gen "${CMAKE_CURRENT_BINARY_DIR}/ariadne_python/${target}")
  configure_file("${_CVC_ARIADNE_PYTHON_DIR}/ariadne_python_source.h.in"
                 "${_gen}/ariadne_python_source.h" @ONLY)
  execute_process(COMMAND "${Python3_EXECUTABLE}" -c "import sys; print(sys.base_prefix)"
                  OUTPUT_VARIABLE _home OUTPUT_STRIP_TRAILING_WHITESPACE RESULT_VARIABLE _rc)
  target_sources(${target} PRIVATE "${_CVC_ARIADNE_PYTHON_DIR}/python_engine.cpp")
  target_include_directories(${target} PRIVATE "${_CVC_ARIADNE_PYTHON_DIR}" "${_gen}")
  target_link_libraries(${target} PRIVATE Python3::Python)
  target_compile_definitions(${target} PRIVATE CVC_ARIADNE_PYTHON)
  if(_rc EQUAL 0 AND _home)
    file(TO_CMAKE_PATH "${_home}" _home)
    target_compile_definitions(${target} PRIVATE "CVC_ARIADNE_PYTHON_HOME=\"${_home}\"")
  endif()
  message(STATUS "${target}: embedded Python ${Python3_VERSION} (${Python3_EXECUTABLE})")
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
