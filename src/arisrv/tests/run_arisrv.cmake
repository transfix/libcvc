# Run arisrv as a ctest checks it (src/arisrv/CMakeLists.txt): `cmake -DARISRV=<binary>
# -DARGS=<a@@b@@...> -DSTATUS=<n> [-DEXPECT1=<regex> ...] -P run_arisrv.cmake`. The run must end
# with exit status STATUS, and its output (stdout, then stderr) match each EXPECTn. The output is
# printed either way, so the test's SKIP_REGULAR_EXPRESSION sees it. ARGS separates arguments with
# "@@", since a ";" would split the -D value.
string(REPLACE "@@" ";" _args "${ARGS}")
execute_process(COMMAND "${ARISRV}" ${_args}
                RESULT_VARIABLE _status OUTPUT_VARIABLE _out ERROR_VARIABLE _err TIMEOUT 60)
set(_all "${_out}${_err}")
message("${_all}")
if(NOT "${_status}" STREQUAL "${STATUS}")
  message(FATAL_ERROR "arisrv exited with '${_status}', expected ${STATUS}")
endif()
foreach(_n RANGE 1 3)
  if(DEFINED EXPECT${_n} AND NOT _all MATCHES "${EXPECT${_n}}")
    message(FATAL_ERROR "arisrv's output does not match: ${EXPECT${_n}}")
  endif()
endforeach()
