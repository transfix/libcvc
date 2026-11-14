#!/usr/bin/env bash
# Runs the suites of ci.yml's `sanitizers` jobs, from the repository root, after the build:
#   - every executable named in SAN_CXX_TESTS, from build/bin, one process each, without its
#     Stress suites (locks-deep.yml runs LocksStress and LocksModelStress; ci.yml's STRESS_REGEX
#     keeps them off the PR lanes too, and under TSan LocksStress alone outlasts run_one's hour).
#     SAN_CXX_RUNS (ci.yml's matrix `cxx_runs`) gives some of them an environment and arguments of
#     their own, one line a binary: `<name>: [VAR=VALUE ...] [--option ...]`. That binary runs with
#     the variables set and the options passed; a --gtest_filter among them gets *Stress* added to
#     its negative patterns, so the Stress suites stay out. The others run bare. A line naming a
#     binary SAN_CXX_TESTS does not run is an error;
#   - every pycvc script named in SAN_PY_TESTS, with the sanitizer runtime the build linked
#     preloaded into the interpreter, which was not built with it, except those py_skip (below)
#     names for the sanitizer. gcc links its runtimes as shared libraries, found with ldd; clang
#     links TSan's into each executable, so clang-tsan preloads clang's shared one;
#   - every state_exec_bench run in SAN_BENCH, one a line: a scenario and its --size options. It
#     runs once (--reps 1 --warmup 0, --timeout 900 a rep), named after the scenario, with its
#     JSON records in san-logs/<scenario>.jsonl. The bench takes no --gtest_filter.
# Each run's output goes to san-logs/<name>.log, and a table of the runs to the job summary.
# SAN_REQUIRED (ci.yml's matrix `required`) names, one per line as `<run>: <text>`, lines a run's
# log must contain: the cases the job is there for, which must not drop out unseen.
# Exits 1 when any run failed or printed a sanitizer report, or a required line is missing.
#
# Usage: run.sh tsan|clang-tsan|asan-ubsan
set -uo pipefail

kind=${1:?usage: run.sh tsan|clang-tsan|asan-ubsan}
here=$(cd "$(dirname "$0")" && pwd)
bin=build/bin
py="${PY_PREFIX:?PY_PREFIX is not set}/bin/python3.12"
mkdir -p san-logs

case "$kind" in
  tsan | clang-tsan)
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
    echo "unknown sanitizer '$kind' (tsan, clang-tsan or asan-ubsan)" >&2
    exit 2
    ;;
esac

set -- $SAN_CXX_TESTS
if [ "$kind" = clang-tsan ]; then
  # clang links TSan's runtime into each executable, which ldd cannot see; the uninstrumented
  # interpreter needs it as a shared library, clang's libclang_rt.tsan: in its runtime directory
  # (named libclang_rt.tsan.so, or libclang_rt.tsan-<arch>.so in the older layout), else anywhere
  # under its resource directory. Every pipeline here reads its input to the end: under pipefail
  # a reader that stops early (grep -q, awk's exit, head) can kill the writer with SIGPIPE and
  # fail the pipeline though it found what it looked for.
  if ! nm "$bin/$1" 2>/dev/null | awk '$NF == "__tsan_init" { f = 1 } END { exit !f }'; then
    echo "::error::$bin/$1 has no TSan runtime linked in (not built with clang -fsanitize=thread?)"
    exit 1
  fi
  dir=$(clang -print-runtime-dir 2>/dev/null)
  rt=""
  for c in "$dir/libclang_rt.tsan.so" "$dir/libclang_rt.tsan-$(uname -m).so"; do
    if [ -f "$c" ]; then
      rt=$c
      break
    fi
  done
  if [ -z "$rt" ]; then
    res=$(clang -print-resource-dir 2>/dev/null)
    rt=$(find "$res" -name 'libclang_rt.tsan*.so' ! -name '*cxx*' -print -quit 2>/dev/null)
  fi
  if [ -z "$rt" ] || [ ! -f "$rt" ]; then
    echo "::error::no shared libclang_rt.tsan for the interpreter (clang's runtime dir: $dir)"
    exit 1
  fi
else
  rt=$(ldd "$bin/$1" | awk -v l="$lib" 'index($1, l) == 1 && !f { print $3; f = 1 }')
  if [ -z "$rt" ] || [ ! -f "$rt" ]; then
    echo "::error::$bin/$1 does not link $lib"
    exit 1
  fi
fi
echo "sanitizer runtime: $rt"
# ASan intercepts __cxa_throw, and finds the real one when it starts. The interpreter is C, so
# libstdc++ isn't loaded then, and pycvc's first throw failed ASan's CHECK on it: preload it too.
cxxrt=$(ldd "$bin/$1" | awk 'index($1, "libstdc++.so") == 1 && !f { print $3; f = 1 }')
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

# SAN_CXX_RUNS: a binary's own variables and options, its line's text after the name.
declare -A cxx_run=()
while IFS= read -r line; do
  [ -n "${line// /}" ] || continue
  name=${line%%:*}
  name=${name//[[:space:]]/}
  known=0
  for t in $SAN_CXX_TESTS; do
    [ "$t" = "$name" ] && known=1
  done
  if [ "$known" -eq 0 ]; then
    failed=1
    echo "::error::SAN_CXX_RUNS names '$name', which SAN_CXX_TESTS does not run"
    continue
  fi
  cxx_run[$name]=${line#*:}
done <<< "${SAN_CXX_RUNS:-}"

for t in $SAN_CXX_TESTS; do
  if [ -z "${cxx_run[$t]+set}" ]; then
    run_one "$t" "$bin/$t" --gtest_filter='-*Stress*'
    continue
  fi
  # read -a splits on whitespace without expanding globs (a filter's *).
  read -ra words <<< "${cxx_run[$t]}"
  vars=()
  opts=()
  filter='-*Stress*'
  for w in "${words[@]}"; do
    if [[ $w =~ ^[A-Za-z_][A-Za-z0-9_]*= ]]; then
      vars+=("$w")
    elif [[ $w == --gtest_filter=* ]]; then
      f=${w#--gtest_filter=}
      if [[ $f == *-* ]]; then
        filter="$f:*Stress*" # more negative patterns
      else
        filter="$f-*Stress*" # positive patterns only
      fi
    else
      opts+=("$w")
    fi
  done
  echo "$t: ${vars[*]} --gtest_filter=$filter ${opts[*]}"
  run_one "$t" env "${vars[@]}" "$bin/$t" --gtest_filter="$filter" "${opts[@]}"
done

# A run whose points were all skipped exits 0: SAN_REQUIRED names the points' lines. The list is
# read on fd 3, so nothing the runs read from stdin takes its lines.
while read -r scenario sizes <&3; do
  [ -n "$scenario" ] || continue
  # shellcheck disable=SC2086 # the --size options, split
  run_one "$scenario" "$bin/state_exec_bench" --scenario "$scenario" $sizes --reps 1 \
    --warmup 0 --timeout 900 --json "san-logs/$scenario.jsonl"
done 3<<< "${SAN_BENCH:-}"

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
    tsan | clang-tsan)
      run_one "$s" env LD_PRELOAD="$rt" PYTHONPATH=build/bindings/pycvc \
        "$py" "bindings/pycvc/$s"
      ;;
    asan-ubsan)
      run_one "$s" env LD_PRELOAD="$rt $cxxrt" PYTHONPATH=build/bindings/pycvc PYTHONMALLOC=malloc \
        ASAN_OPTIONS="$ASAN_OPTIONS:detect_leaks=0" "$py" "bindings/pycvc/$s"
      ;;
  esac
done

missing=()
while IFS= read -r line; do
  [ -n "${line// /}" ] || continue
  run=${line%%:*}
  text=${line#*:}
  text=${text# }
  if ! grep -qF -- "$text" "san-logs/$run.log" 2>/dev/null; then
    failed=1
    missing+=("| $run | $text |")
    echo "::error::$run: its log has no '$text' (a case this job requires did not run or pass)"
  fi
done <<< "${SAN_REQUIRED:-}"

{
  echo "### sanitizers / $kind"
  echo
  echo "| run | exit | reports | seconds |"
  echo "|---|---|---|---|"
  printf '%s\n' "${rows[@]}"
  if [ "${#missing[@]}" -ne 0 ]; then
    echo
    echo "Required lines missing:"
    echo
    echo "| run | line |"
    echo "|---|---|"
    printf '%s\n' "${missing[@]}"
  fi
} >> "${GITHUB_STEP_SUMMARY:-/dev/stdout}"
exit "$failed"
