// Threaded execution, Stage 0 (docs/STATE_EXEC_THREADED.md 7, 8.1): the process-global hazards a
// second thread would hit today, each case one HZ test of td 8.1 that needs threads or a whole
// host rather than one intrinsic. The sanitizer jobs run this binary (ci.yml's SAN_CXX_TESTS) and
// require each case's line. No case name contains "Stress": run.sh filters those out.

#include <gtest/gtest.h>
