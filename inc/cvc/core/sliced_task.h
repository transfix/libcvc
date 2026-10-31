/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// Sliced tasks: a long computation run as a state_exec process on the app's scheduler, a slice at
// a time. cvc::tf's overlay tasks (cvc/tf/overlay_tasks.h) and fe_lab's loading, meshing and
// solves are such tasks. The computation is a cvc::sliced_job, which steps in budgets of work
// units; the process steps it:
//
//   (let ((r (job-step)))
//     (while (not (= r #t))
//       (await r)                      ; #f: yield one frame; a future: sleep until it is posted
//       (set r (job-step))))
//   (job-finish)
//
// (job-step) and (job-finish) are natives bound in the task's own environment (a child of a
// default environment with the intrinsics, for (await)); they close over the task, so the program
// names no handle. The process is named "<kind>/cooperative <subject>" or "<kind>/threaded
// <subject>" (as (ps) shows it), carries the owner tag opt.owner, and runs with no step, time or
// memory limit: a task runs for seconds and thousands of frames, each frame's share is bounded by
// slice_ms (cooperative) or costs the scheduler nothing (threaded), and the job's memory is
// C++-side, not DSL heap.
//
//   cooperative  (single-threaded wasm, or threading = cooperative): each (job-step) steps the job
//                for about slice_ms of wall time (steady_clock; the step budget adapts to the
//                measured cost, restarting small at every phase change) and returns #f, so the
//                process yields one frame and the scheduler interleaves it with every other
//                process: one slice per task per frame. The call after the slice that finishes the
//                job returns #t, so finishing gets a frame of its own.
//   threaded     (native, or wasm built with -pthread and served cross-origin isolated): each
//                (job-step) launches about chunk_ms of the job on a background worker
//                (app::compute_async) and returns the future of its reply; the process parks on it
//                (it costs the scheduler nothing meanwhile) and the step after the last chunk
//                returns #t. A chunk steps the job in calls of about a quarter of chunk_ms (at
//                most 50 ms), so a job's per-call costs vanish. Chunking keeps the progress live,
//                and a cancel lands within one call. The chunk touches only the job, never state,
//                and reaches the scheduler only through post_message, its thread-safe ingress
//                (and only while the task still waits for the reply). If no thread can be
//                started, the task carries on cooperatively.
//
// Either way opt.on_progress (at most every 100 ms while it runs) and opt.on_done (once, when the
// job is done and not cancelled) run on the scheduler thread -- never on a worker -- so they may
// read and write cvc::state. Something must pump app.exec_scheduler(): every Ariadne Runtime does,
// whoever submitted the process; a host with no Runtime calls cvc::pump_exec_frame once a frame,
// or blocks in wait_sliced_job(). The functions below are for the thread that pumps the scheduler,
// and none may be called from inside a task's own steps or hooks.
//
// Lifetime. The task (job, options, hooks) is owned by its process's environment: when the
// process ends -- finished, cancelled, killed by kill_owner, or the scheduler destroyed with its
// app -- the job is cancelled and let go of at once (a running pool chunk keeps it until its next
// check). The per-app bookkeeping (pid -> task) lives in the app's own data map, so it is
// destroyed with the app.

#ifndef CVC_CORE_SLICED_TASK_H
#define CVC_CORE_SLICED_TASK_H

#include <cstddef>
#include <functional>
#include <memory>
#include <string>

namespace cvc {
class app;

// A computation done a slice at a time. step(budget) does about `budget` units of work (a step may
// run past it by one indivisible piece) and returns true once the job is done; budget 0 runs to
// the end. A unit should cost about the same within one phase: phase() names the kind of work the
// next step does (a static string, compared by pointer), and a caller pacing its budget by the
// clock restarts its sizing when it changes. cancel() is the one thread-safe member: the next step
// (or the running one, at its next check) returns true with nothing more done, and done() is true
// from then on. step() reports a failure through status() rather than throwing; a throw counts as
// one too.
class sliced_job {
public:
  virtual ~sliced_job() = default;
  virtual bool step(std::size_t budget) = 0;
  virtual bool done() const = 0;
  virtual void cancel() = 0;
  virtual bool cancelled() const = 0;
  virtual const char *phase() const = 0;
  // In [0,1], monotone; 1 once done.
  virtual double progress() const = 0;
  // While running, a progress note ("meshing: collapse 42%"); once done, the job's own summary.
  virtual std::string status() const = 0;
};

enum class sliced_threading {
  // threaded where threads exist (native, wasm-mt), else cooperative; cooperative everywhere
  // with CVC_SLICED_THREADING=cooperative in the environment (a native build run the way
  // single-threaded wasm runs)
  automatic,
  cooperative, // always sliced on the scheduler thread
  threaded     // always on the background pool (cooperative where no threads exist or start)
};

// Whether this build can run work on other threads: false exactly under Emscripten without
// pthreads (__EMSCRIPTEN__ && !__EMSCRIPTEN_PTHREADS__); true in every native build and in
// wasm-mt.
bool threads_available();

struct sliced_task_options {
  std::string kind = "sliced"; // the process name: "<kind>/<mode> <subject>"
  std::string subject;
  sliced_threading threading = sliced_threading::automatic;
  double slice_ms = 4.0;   // cooperative: wall time of work per frame (<= 0 or NaN: 4)
  double chunk_ms = 250.0; // threaded: wall time of work per pool chunk (<= 0 or NaN: 250)
  std::string owner;       // the scheduler owner tag (kill_owner stops the task); "" = unowned
  // On the scheduler thread. on_progress: while the job runs, at most every 100 ms (and once
  // before its first step in threaded mode). on_done: once, after the job is done, unless it was
  // cancelled. An exception either throws ends the process (killed, with the message).
  std::function<void(const sliced_job &)> on_progress;
  std::function<void(sliced_job &)> on_done;
};

// Submit `job` as a process to app.exec_scheduler() and return its pid; the work starts at the
// next pump. Tasks are independent. Reaps finished tasks first. Never throws: a task that cannot
// start returns -1 and, when `why` is given, says why there.
int schedule_sliced_job(app &app, std::shared_ptr<sliced_job> job, const sliced_task_options &opt,
                        std::string *why = nullptr);

// Stop a task and reap its process: its job is cancelled and let go of, on_done never runs, and a
// chunk running on the pool stops at its next check and posts no reply. Safe on a finished or
// unknown pid; a pid that is not a sliced task's is left alone.
void cancel_sliced_job(app &app, int pid);

// Whether the task is still running (finished tasks' processes are reaped here).
bool sliced_job_running(app &app, int pid);

// Pump app.exec_scheduler() a frame at a time until the task finishes or `timeout_s` passes; true
// when it finished (or was not running). For tests and headless tools with no Ariadne Runtime
// driving the scheduler; never call it from inside a scheduler step.
bool wait_sliced_job(app &app, int pid, double timeout_s = 60.0);

// How many replies of stopped tasks' pool chunks the app's bookkeeping holds for a sweep to
// collect from the scheduler (0 when none). For tests and leak checks.
std::size_t sliced_job_orphaned_replies(app &app);

} // namespace cvc

#endif // CVC_CORE_SLICED_TASK_H
