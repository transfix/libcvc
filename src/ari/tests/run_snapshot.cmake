# Run `ari snapshot` as a ctest checks it (src/ari/CMakeLists.txt): `cmake -DARI=<binary>
# -DARGS=<a@@b@@...> [-DEXPECT1=<text> ... -DEXPECT8=<text>] [-DREJECT1=<text> -DREJECT2=<text>]
# -P run_snapshot.cmake`. The run must exit 0, and its output hold each EXPECTn as written (not a
# regex: a screen is full of brackets and dots) and no REJECTn. The output is printed either way,
# so the test's SKIP_REGULAR_EXPRESSION sees it. ARGS separates arguments with "@@", since a ";"
# would split the -D value.
string(REPLACE "@@" ";" _args "${ARGS}")
execute_process(COMMAND "${ARI}" ${_args}
                RESULT_VARIABLE _status OUTPUT_VARIABLE _out ERROR_VARIABLE _err TIMEOUT 60)
set(_all "${_out}${_err}")
message("${_all}")
if(NOT "${_status}" STREQUAL "0")
  message(FATAL_ERROR "ari exited with '${_status}'")
endif()
foreach(_n RANGE 1 8)
  if(DEFINED EXPECT${_n})
    string(FIND "${_all}" "${EXPECT${_n}}" _at)
    if(_at EQUAL -1)
      message(FATAL_ERROR "the snapshot has no '${EXPECT${_n}}'")
    endif()
  endif()
endforeach()
foreach(_n RANGE 1 2)
  if(DEFINED REJECT${_n})
    string(FIND "${_all}" "${REJECT${_n}}" _at)
    if(NOT _at EQUAL -1)
      message(FATAL_ERROR "the snapshot has '${REJECT${_n}}', which it must not")
    endif()
  endif()
endforeach()
