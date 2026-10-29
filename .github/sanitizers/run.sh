#!/usr/bin/env bash
# Runs the suites of ci.yml's `sanitizers` jobs, from the repository root, after the build:
#   - every executable named in SAN_CXX_TESTS, from build/bin, one process each, without its
#     Stress suites (locks-deep.yml runs LocksStress and LocksModelStress; ci.yml's STRESS_REGEX
#     keeps them off the PR lanes too, and under TSan LocksStress alone outlasts run_one's hour);
#   - every pycvc script named in SAN_PY_TESTS, with the sanitizer runtime the build linked
#     preloaded into the interpreter, which was not built with it, except those py_skip (below)
#     names for the sanitizer.
# Each run's output goes to san-logs/<name>.log, and a table of the runs to the job summary.
# Exits 1 when any run failed or printed a sanitizer report.
#
# Usage: run.sh tsan|asan-ubsan
set -uo pipefail

kind=${1:?usage: run.sh tsan|asan-ubsan}
here=$(cd "$(dirname "$0")" && pwd)
bin=build/bin
py="${PY_PREFIX:?PY_PREFIX is not set}/bin/python3.12"
mkdir -p san-logs

case "$kind" in
  tsan)
    lib=libtsan
    export TSAN_OPTIONS="suppressions=$here/tsan.supp:second_deadlock_stack=1:history_size=4:exitcode=66"
    py_skip=""
    ;;
  asan-ubsan)
    lib=libasan
    export ASAN_OPTIONS="detect_leaks=1:exitcode=66"
    export LSAN_OPTIONS="suppressions=$here/lsan.supp:print_suppressions=0"
    export UBSAN_OPTIONS="print_stacktrace=1:halt_on_error=1:exitcode=66"
    # pycvc scripts not run under ASan, which can't suppress a report and stops a process at its
    # first, space-separated. Give each a comment naming what it would report and the PR that
    # fixes it, and take it out when that lands. (#864 needed one until it did.)
    py_skip=""
    ;;
  *)
    echo "unknown sanitizer '$kind' (tsan or asan-ubsan)" >&2
    exit 2
    ;;
esac

set -- $SAN_CXX_TESTS
rt=$(ldd "$bin/$1" | awk -v l="$lib" 'index($1, l) == 1 { print $3; exit }')
if [ -z "$rt" ] || [ ! -f "$rt" ]; then
  echo "::error::$bin/$1 does not link $lib"
  exit 1
fi
echo "sanitizer runtime: $rt"
# ASan intercepts __cxa_throw, and finds the real one when it starts. The interpreter is C, so
# libstdc++ isn't loaded then, and pycvc's first throw failed ASan's CHECK on it: preload it too.
cxxrt=$(ldd "$bin/$1" | awk 'index($1, "libstdc++.so") == 1 { print $3; exit }')
if [ -z "$cxxrt" ] || [ ! -f "$cxxrt" ]; then
  echo "::error::$bin/$1 does not link libstdc++"
  exit 1
fi

# A report's first line, for each sanitizer. UBSan's carries the source position, so a test's
# own output that happens to say "runtime error" is not counted.
pattern='WARNING: ThreadSanitizer|ERROR: AddressSanitizer|ERROR: LeakSanitizer|\.(c|cc|cpp|cxx|h|hh|hpp|hxx|ipp|inl):[0-9]+:[0-9]+: runtime error:'
failed=0
rows=()

run_one() { # name command...
  local name=$1 log="san-logs/$1.log" t0=$SECONDS st reports
  shift
  echo "::group::$name"
  timeout 3600 "$@" 2>&1 | tee "$log"
  st=${PIPESTATUS[0]}
  echo "::endgroup::"
  reports=$(grep -cE "$pattern" "$log" || true)
  if [ "$st" -ne 0 ] || [ "$reports" -ne 0 ]; then
    failed=1
    echo "::error::$name: exit $st, $reports sanitizer report(s); the log is in the sanitizers-$kind-logs artifact"
  fi
  rows+=("| $name | $st | $reports | $((SECONDS - t0)) |")
}

for t in $SAN_CXX_TESTS; do
  run_one "$t" "$bin/$t" --gtest_filter='-*Stress*'
done

# CPython frees little at exit, so the leak check is off under the interpreter (the C++ suites
# keep it), and PYTHONMALLOC=malloc lets ASan see the interpreter's own allocations.
for s in $SAN_PY_TESTS; do
  case " $py_skip " in
    *" $s "*)
      echo "::warning::$s is not run under $kind (see py_skip in .github/sanitizers/run.sh)"
      rows+=("| $s | skipped | | 0 |")
      continue
      ;;
  esac
  case "$kind" in
    tsan)
      run_one "$s" env LD_PRELOAD="$rt" PYTHONPATH=build/bindings/pycvc \
        "$py" "bindings/pycvc/$s"
      ;;
    asan-ubsan)
      run_one "$s" env LD_PRELOAD="$rt $cxxrt" PYTHONPATH=build/bindings/pycvc PYTHONMALLOC=malloc \
        ASAN_OPTIONS="$ASAN_OPTIONS:detect_leaks=0" "$py" "bindings/pycvc/$s"
      ;;
  esac
done

{
  echo "### sanitizers / $kind"
  echo
  echo "| run | exit | reports | seconds |"
  echo "|---|---|---|---|"
  printf '%s\n' "${rows[@]}"
} >> "${GITHUB_STEP_SUMMARY:-/dev/stdout}"
exit "$failed"
