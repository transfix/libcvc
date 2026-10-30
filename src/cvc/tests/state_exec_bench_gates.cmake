# state_exec_bench_gates: the deterministic state_exec bench gates, as a CTest
# (docs/STATE_EXEC_BENCH.md, "Gates in CTest"; docs/STATE_EXEC_LOCKS.md 11.2). Counts, not times,
# so they hold on any machine and under any load:
#
#   - lock.uncontended, form=single: an uncontended with-lock adds 0 evaluator steps and 0
#     allocations over (begin ...), at every depth, mode, chroot and owner, on both schedulers
#     (design 8.8);
#   - sched.single.arith_loop and sched.single.tail_rec: steps_total pinned, so a change to the
#     evaluator's or a scheduler's step counts shows here.
#
# Run by CTest as
#   cmake -DBENCH=<state_exec_bench> -DPYTHON=<python3> -DRUNNER=<tools/bench/state_exec_bench.py>
#         -DWORK_DIR=<dir> -P state_exec_bench_gates.cmake
# Every command must exit 0: a gate that fails (or matches no point) fails the test.
#
# A deliberate change to a pinned steps_total updates the number below in the same commit, with
# the reason.

foreach(var BENCH PYTHON RUNNER WORK_DIR)
  if(NOT DEFINED ${var} OR "${${var}}" STREQUAL "")
    message(FATAL_ERROR "state_exec_bench_gates: -D${var}=... is required")
  endif()
endforeach()

file(MAKE_DIRECTORY "${WORK_DIR}")
set(LOCKS_RUN "${WORK_DIR}/lock_uncontended.jsonl")
set(SCHED_RUN "${WORK_DIR}/sched_single.jsonl")
file(REMOVE "${LOCKS_RUN}" "${SCHED_RUN}") # --json appends

set(failed "")

# Run a command; on a non-zero exit, record it (every command still runs, so one CTest log shows
# every gate).
function(gate_step what)
  execute_process(COMMAND ${ARGN} RESULT_VARIABLE rc)
  if(NOT rc EQUAL 0)
    set(failed "${failed}\n  ${what} (exit ${rc})" PARENT_SCOPE)
  endif()
endfunction()

# The runs. If one fails, its gates have nothing to read and fail too. One rep, no warmup and one
# timing round: the counts are the same in every rep and round, so more only costs time.
gate_step("state_exec_bench lock.uncontended"
  "${BENCH}" --scenario lock.uncontended --size n=2000 --size rounds=1 --reps 1 --warmup 0
  --json "${LOCKS_RUN}")
gate_step("state_exec_bench sched.single.*"
  "${BENCH}" --scenario "sched.single.*" --reps 1 --warmup 0 --json "${SCHED_RUN}")

# 0 steps and 0 allocations per uncontended lock, for a one-form body (design 8.8, 11.2).
foreach(metric extra.steps_per_lock extra.allocs_per_lock)
  gate_step("gate lock.uncontended form=single ${metric} <= 0"
    "${PYTHON}" -I "${RUNNER}" gate "${LOCKS_RUN}" --scenario lock.uncontended
    --where form=single --metric ${metric} --max 0)
endforeach()

# Pinned steps_total, the same on both schedulers (the programs are submitted pre-parsed, so the
# async scheduler takes no read steps). Measured on the L1 lock branch (locks/l1-bench, cut from
# 769f47aa), Windows MSVC Release, the same on sync and async: 27 steps an iteration of the
# arithmetic loop, 25 a level of the tail call, and 20 to set up and return.
#   sched.single.arith_loop n=40000:   1080020
#   sched.single.tail_rec   n=1000:      25020
#   sched.single.tail_rec   n=10000:    250020
#   sched.single.tail_rec   n=100000:  2500020
foreach(point "arith_loop;40000;1080020" "tail_rec;1000;25020" "tail_rec;10000;250020"
              "tail_rec;100000;2500020")
  list(GET point 0 program)
  list(GET point 1 n)
  list(GET point 2 steps)
  gate_step("gate sched.single.${program} n=${n} steps_total == ${steps}"
    "${PYTHON}" -I "${RUNNER}" gate "${SCHED_RUN}" --scenario "sched.single.${program}"
    --where n=${n} --metric steps_total --min ${steps} --max ${steps})
endforeach()

if(failed)
  message(FATAL_ERROR "state_exec_bench_gates failed:${failed}")
endif()
message(STATUS "state_exec_bench_gates: every gate passed")
