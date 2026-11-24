// Force-included into legacy SecondaryStructures: its CMakeLists defined HUGE as 1000000 on the
// command line, which the platform's math.h then redefines (glibc, to FLT_MAX) or chokes on (the
// UCRT). Here it is 1000000 after math.h on every platform. Every use of it is a "larger than any
// distance" start, so the value does not change the output at the inputs' scale. Part of
// tools/volrover-goldens: never built into libcvc.

#include <cmath>
#include <math.h>
#undef HUGE
#define HUGE 1000000
