/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// Computing a transfer function's overlays (histogram, contour spectrum, contour tree) ON THE
// STATE_EXEC SCHEDULER, without a thread of its own.
//
// A contour tree or spectrum of a 128^3 volume takes seconds. Running it inline would freeze the
// frame; a dedicated std::thread is not an option on single-threaded wasm (starting one aborts).
// So the work is a resumable cvc::tf::overlay_job driven by an ordinary state_exec process on the
// app's scheduler (app.exec_scheduler(); who pumps it is below):
//
//   (let ((r (tf-overlay-step)))
//     (while (not (= r #t))
//       (await r)                      ; #f: yield one frame; a future: sleep until it is posted
//       (set r (tf-overlay-step))))
//   (tf-overlay-publish)
//
// (tf-overlay-step) and (tf-overlay-publish) are natives bound in the task's own environment (a
// child of a default environment with the intrinsics, for (await)); they close over the task, so
// the program names no handle. The process is named "tf-overlays/cooperative <tf>" or
// "tf-overlays/threaded <tf>" (as (ps) shows it), carries the owner tag opt.owner, and runs with
// no step, time or memory limit: a task runs for seconds and thousands of frames, each frame's
// share is bounded by slice_ms (cooperative) or costs the scheduler nothing (threaded), and the
// job's memory is C++-side, not DSL heap.
//
//   cooperative  (single-threaded wasm, or threading = cooperative): each (tf-overlay-step) steps
//                the job for about slice_ms of wall time (steady_clock; the step budget adapts to
//                the measured cost, restarting small at every phase change) and returns #f, so the
//                process yields one frame and the scheduler interleaves it with every other
//                process: one slice per task per frame. The call after the slice that finishes
//                the job returns #t, so publishing gets a frame of its own.
//   threaded     (native, or wasm built with -pthread and served cross-origin isolated): each
//                (tf-overlay-step) launches about chunk_ms of the job on a background worker
//                (app::compute_async, an app background-pool task as cvc::future_pool_task's is)
//                and returns the future of its reply; the process parks on it (it costs the
//                scheduler nothing meanwhile) and the step after the last chunk returns #t.
//                Chunking keeps the progress note live, and a cancel lands within one job step
//                (at most ~50 ms). The chunk touches only the job, never state, and reaches the
//                scheduler only through post_message, its thread-safe ingress (and only while the
//                task still waits for the reply). (app::compute_async runs each chunk on one of the
//                app's background-pool workers, not on the compute pool, so this works whatever
//                the size of the app's compute pool, even with no workers. If no thread can be
//                started -- compute_async throws when the background pool has no worker and
//                cannot create one, or drops a chunk it queued once the pool is left with no
//                worker, as when a keyed task held the only slot of a pool with none and the
//                worker asked for as it ended couldn't be created -- the task carries on
//                cooperatively.)
//
// Either way the results reach state through (tf-overlay-publish), on the scheduler thread --
// never from a worker -- and between steps the task writes its progress to <tf>.overlay_status
// (the job's status(), e.g. "contour tree: sorting 42%"; at most every 100 ms). Something must pump
// app.exec_scheduler() for any of this to advance. Every Ariadne Runtime does, in its drain(),
// whenever a process is live on the scheduler, whoever submitted it (drain_ingress, wake_awaiting,
// sync_run), so a task advances in any Ariadne host; an app's Runtimes share one frame boundary,
// so a cooperative task gets one slice per app frame however many Runtimes it has. A host with no
// Runtime pumps the scheduler itself, once a frame on the thread that owns it
// (cvc::pump_exec_frame, cvc/core/async_lane.h, is one frame), or blocks in wait_overlays(). The
// functions below are for the thread that pumps the scheduler (the render thread in an Ariadne
// host), and none may be called from inside the task's own steps (its deliver hook included).
//
// Lifetime. The task (job, options, deliver hook) is owned by its process's environment: when the
// process ends -- published, cancelled, killed by kill_owner, or the scheduler destroyed with its
// app -- the job is let go of at once (a running pool chunk keeps it until its next check). The
// per-app bookkeeping (pid -> task, for cancel_overlays, overlays_running and reaping) lives in
// the app's own data map ("__tf_overlay_tasks"), so it is destroyed with the app.

#ifndef CVC_TF_OVERLAY_TASKS_H
#define CVC_TF_OVERLAY_TASKS_H

#include <cstddef>
#include <cvc/tf/overlays.h>
#include <functional>
#include <memory>
#include <string>

namespace cvc {
class app;
class volume;
namespace tf {

enum class overlay_threading {
  automatic,   // threaded where threads exist (native, wasm-mt), else cooperative
  cooperative, // always sliced on the scheduler thread
  threaded     // always on the background pool (cooperative where no threads exist or start)
};

// Whether this build can run work on other threads: false exactly under Emscripten without
// pthreads (__EMSCRIPTEN__ && !__EMSCRIPTEN_PTHREADS__); true in every native build and in
// wasm-mt.
bool threads_available();

struct overlay_task_options {
  overlay_options overlays;
  overlay_threading threading = overlay_threading::automatic;
  double slice_ms = 4.0;   // cooperative: wall time of work per frame (<= 0 or NaN: 4)
  double chunk_ms = 250.0; // threaded: wall time of work per pool chunk (<= 0 or NaN: 250)
  std::string owner;       // the scheduler owner tag (kill_owner stops the task); "" = unowned
  // When set, the finished overlays are handed to this (on the scheduler thread) INSTEAD of
  // publish_overlays(<tf>): a producer that post-processes them (re-binning onto a window)
  // publishes them itself, the final status included (the task writes only its progress notes to
  // <tf>.overlay_status). An exception it throws ends the process (killed, with the message).
  std::function<void(cvc::tf::overlays &&)> deliver;
};

// Start computing `bits` overlays of `vol` for the model key `tf_path` (an absolute state path):
// submits the process above to app.exec_scheduler() and returns its pid; the work starts at the
// next pump. The job keeps a shallow copy of `vol` (the voxels are shared copy-on-write: do not
// write them while the task runs). The task writes tf_path.overlay_status while it runs, then
// publish_overlays() (or opt.deliver) the results: exactly what compute_overlays(vol, bits,
// opt.overlays) gives. Tasks are independent: one for a key that already has one runs beside it
// (a producer restarting a key's task cancels the old one first). Reaps finished tasks first.
// Never throws; a task that cannot start returns -1 with the reason in overlay_status (an empty
// tf_path returns -1 and writes nothing).
int schedule_overlays(cvc::app &app, const std::string &tf_path, const cvc::volume &vol,
                      unsigned bits, const overlay_task_options &opt = {});
// The same for a scalar field on a mesh (an overlay_job on a contour_mesh): the task holds the
// shared mesh until its job lets go of it.
int schedule_overlays(cvc::app &app, const std::string &tf_path,
                      std::shared_ptr<const contour_mesh> mesh, unsigned bits,
                      const overlay_task_options &opt = {});

// Stop a task started by schedule_overlays and reap its process: it publishes nothing more (its
// last progress note stays in overlay_status: a producer that gives up for good clears it), the
// job is let go of, and a chunk running on the pool stops at its next check and posts no reply. (A
// reply a chunk had already posted, which no process will take now, is collected from the
// scheduler by this call's sweep -- or, for a task stopped by kill_owner, by the next sweep.) Safe
// on a finished or unknown pid; a pid that is not an overlay task's is left alone.
void cancel_overlays(cvc::app &app, int pid);

// Whether the task is still running. Finished tasks' processes (published, failed, or killed by
// kill_owner) are reaped here, and by schedule_overlays and cancel_overlays; their jobs were
// already let go of when they ended, so a finished task holds nothing. The same sweep collects
// the replies stopped tasks' chunks had posted (when there is one, it drains the scheduler's
// ingress first, as a pump would).
bool overlays_running(cvc::app &app, int pid);

// Pump app.exec_scheduler() a frame at a time (drain_ingress, wake_awaiting, sync_step, sync_run;
// a 1 ms sleep when nothing ran, e.g. while a pool chunk runs) until the task finishes or
// `timeout_s` passes; true when it finished (or was not running). For tests and headless tools
// that have no Ariadne Runtime driving the scheduler: it runs every process on the scheduler, not
// just this one, and must never be called from inside a scheduler step.
bool wait_overlays(cvc::app &app, int pid, double timeout_s = 60.0);

// How many replies of stopped tasks' pool chunks the app's bookkeeping holds for a sweep to
// collect from the scheduler (0 when none, or before the app's first task). For tests and leak
// checks.
std::size_t overlay_orphaned_replies(cvc::app &app);

} // namespace tf
} // namespace cvc

#endif // CVC_TF_OVERLAY_TASKS_H
