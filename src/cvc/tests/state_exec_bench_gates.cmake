# state_exec_bench_gates: the deterministic state_exec bench gates, as a CTest
# (docs/STATE_EXEC_BENCH.md, "Gates in CTest"; docs/STATE_EXEC_LOCKS.md 11.2). Counts, not times,
# so they hold on any machine and under any load:
#
#   - lock.uncontended, form=single: an uncontended with-lock adds 0 evaluator steps and 0
#     allocations over (begin ...), at every depth, mode, chroot and owner, on both schedulers,
#     and on both kinds of manager: threading=single and threading=shared, whose mutex (phase L3)
#     adds neither (design 8.8, 11.2);
#   - sched.single.arith_loop and sched.single.tail_rec: steps_total pinned, so a change to the
#     evaluator's or a scheduler's step counts shows here; on the async scheduler also through an
#     exec_group at threads 0 (driver=group, over its own and over the frozen base), which takes
#     the same steps (threaded execution's Stage 1, rule 1);
#   - the same points on the async scheduler at slice_steps 256 (threaded execution's Stage S): a
#     pick that runs up to 256 steps of its process takes the same steps as one that runs one, so
#     the pins are the same numbers (S2.md: step counts unchanged at any slice_steps);
#   - mt.groups at its smoke sizes, continuous against fork-join (Stage 2a): attached groups,
#     pumped whenever they have work in pumps of the pool's budget, take the same steps as groups
#     pumped by pump_all rounds, on the caller (t=1) and on workers (t=2), for even and uneven
#     groups;
#   - mt.cross_group_msg at its smoke sizes, across groups against one group (Stage 2b): the same
#     programs exchanging messages through the router (pingpong, fan-out and broadcast between
#     groups with channel prefixes) take the steps they take in one group with the router off, in
#     both pump modes. Until the router routes (S2b.0's stubs) its cross-group points fail their
#     routing probe, and so does this gate.
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
set(SLICE_RUN "${WORK_DIR}/sched_single_slice.jsonl")
set(GROUPS_RUN "${WORK_DIR}/mt_groups.jsonl")
set(CROSS_RUN "${WORK_DIR}/mt_cross_group_msg.jsonl")
# --json appends.
file(REMOVE "${LOCKS_RUN}" "${SCHED_RUN}" "${SLICE_RUN}" "${GROUPS_RUN}" "${CROSS_RUN}")

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
  "${BENCH}" --scenario lock.uncontended --size n=2000 --size rounds=1
  --size threading=single,shared --reps 1 --warmup 0 --json "${LOCKS_RUN}")
gate_step("state_exec_bench sched.single.*"
  "${BENCH}" --scenario "sched.single.*" --size driver=spin,group --size base=own,frozen
  --reps 1 --warmup 0 --json "${SCHED_RUN}")
# Stage S: the pinned programs alone at slice_steps 256 (async only: the sync scheduler has no
# runs), through a group too.
gate_step("state_exec_bench sched.single.* slice_steps=256"
  "${BENCH}" --scenario sched.single.arith_loop --scenario sched.single.tail_rec
  --executor async --size slice_steps=256 --size driver=spin,group --size base=own,frozen
  --reps 1 --warmup 0 --json "${SLICE_RUN}")
# Stage 2a: mt.groups in both pump modes, at its smoke sizes.
gate_step("state_exec_bench mt.groups pump=fork_join,continuous"
  "${BENCH}" --scenario mt.groups --smoke --size t=1,2 --size pump=fork_join,continuous
  --size shape=even,uneven --reps 1 --warmup 0 --json "${GROUPS_RUN}")
# Stage 2b: mt.cross_group_msg in one group and across groups, in both pump modes, at its smoke
# sizes.
gate_step("state_exec_bench mt.cross_group_msg span=one,cross"
  "${BENCH}" --scenario mt.cross_group_msg --smoke --size span=one,cross --size t=1,2
  --size pump=fork_join,continuous --reps 1 --warmup 0 --json "${CROSS_RUN}")

# 0 steps and 0 allocations per uncontended lock, for a one-form body (design 8.8, 11.2).
foreach(metric extra.steps_per_lock extra.allocs_per_lock)
  gate_step("gate lock.uncontended form=single ${metric} <= 0"
    "${PYTHON}" -I "${RUNNER}" gate "${LOCKS_RUN}" --scenario lock.uncontended
    --where form=single --metric ${metric} --max 0)
endforeach()

# Pinned steps_total, the same on both schedulers and through a group (the programs are submitted
# pre-parsed, so the async scheduler takes no read steps; every matching point is gated, the
# driver=group ones included). Measured on the L1 lock branch (locks/l1-bench, cut from 769f47aa),
# Windows MSVC Release, the same on sync and async: 27 steps an iteration of the arithmetic loop,
# 25 a level of the tail call, and 20 to set up and return.
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
  # The same number at slice_steps 256: runs change which process steps when, not how many steps
  # a program takes. --where makes a run without the 256 points fail rather than pass vacuously.
  gate_step("gate sched.single.${program} n=${n} slice_steps=256 steps_total == ${steps}"
    "${PYTHON}" -I "${RUNNER}" gate "${SLICE_RUN}" --scenario "sched.single.${program}"
    --where n=${n} --where slice_steps=256 --metric steps_total --min ${steps} --max ${steps})
endforeach()

# Continuous mode takes fork-join's steps, point by point (one run, paired on pump: a fork-join
# point without its continuous twin fails, as does a continuous point that failed its checks).
gate_step("compare mt.groups pump=fork_join:continuous steps_total unchanged"
  "${PYTHON}" -I "${RUNNER}" compare "${GROUPS_RUN}" "${GROUPS_RUN}"
  --pair pump=fork_join:continuous --metric steps_total --threshold 0 --fail-on-change)

# Across groups, the programs take the steps they take in one group, point by point (paired on
# span: a cross-group point that failed its checks, routing off included, fails the pin): a
# routed send or receive takes the steps a local one does.
gate_step("compare mt.cross_group_msg span=one:cross steps_total unchanged"
  "${PYTHON}" -I "${RUNNER}" compare "${CROSS_RUN}" "${CROSS_RUN}"
  --pair span=one:cross --metric steps_total --threshold 0 --fail-on-change)

if(failed)
  message(FATAL_ERROR "state_exec_bench_gates failed:${failed}")
endif()
message(STATUS "state_exec_bench_gates: every gate passed")
