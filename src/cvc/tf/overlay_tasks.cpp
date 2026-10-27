/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::tf overlay tasks: an overlay_job driven by a state_exec process on the app's scheduler. See
// cvc/tf/overlay_tasks.h for the program, the two modes and the contract.
//
// Ownership. A task is a task_record: the job (in a job_box, shared with a pool chunk while one
// runs), the options, the progress clock and the process's intrinsics context. The process's
// environment owns it -- the two natives close over a task_anchor holding the record -- so it
// cannot outlive the scheduler, and when the process ends for ANY reason (published, cancelled,
// kill_owner, the scheduler destroyed with its app) the anchor's destructor lets go of the job at
// once. A per-app registry (pid -> record, for cancel_overlays / overlays_running / reaping) lives
// in the app's own data map, like the app's state root, so it is created and destroyed with the
// app and a later app at the same address never sees it.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cvc/core/app.h>
#include <cvc/state/state.h>
#include <cvc/state/state_exec/async_scheduler.h>
#include <cvc/state/state_exec/builtins.h>
#include <cvc/state/state_exec/intrinsics.h>
#include <cvc/state/state_exec/memory_tracker.h>
#include <cvc/state/state_exec/process.h>
#include <cvc/tf/overlay_tasks.h>
#include <cvc/volume/volume.h>
#include <exception>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <span>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

namespace cvc {
namespace tf {

namespace {

namespace se = cvc::state_exec;
using steady = std::chrono::steady_clock;

// The task's program (overlay_tasks.h). `#f` reads as nil and `=` is the DSL's equality.
const char kProgram[] = "(let ((r (tf-overlay-step)))\n"
                        "  (while (not (= r #t))\n"
                        "    (await r)\n"
                        "    (set r (tf-overlay-step))))\n"
                        "(tf-overlay-publish)\n";

// The app data key (and app mutex name) of the per-app registry.
const char kRegistryKey[] = "__tf_overlay_tasks";

constexpr double kStatusPeriodMs = 100.0; // progress notes at most ~10 Hz
constexpr std::size_t kFirstChunk = 256;  // units; a sweep unit is ~0.5-1 us, a scan unit ~2 ns
constexpr std::size_t kMinChunk = 16;
constexpr std::size_t kMaxChunk = std::size_t(1) << 24;
// The dearest unit there is, in microseconds: a vertex swept in a contour tree's join or split
// tree, ~0.3-0.6 us on a desktop core with a 128^3 volume's cache misses (contour_tree.h), with
// headroom. No call asks for more units than the slice's remaining time pays for at this cost.
constexpr double kWorstUnitUs = 1.0;

double ms_between(steady::time_point a, steady::time_point b) {
  return std::chrono::duration<double, std::milli>(b - a).count();
}

// Sizes job.step() budgets against the clock. A unit's cost differs by up to ~500x between phases
// (a voxel scanned vs a vertex swept), and by ~30x within one (a sweep's first stretch is cheap,
// then each vertex misses the cache), so the size restarts small at every phase change, adapts
// (at most doubling per call) to about `target` ms a call, and is capped by the time left.
struct pacer {
  std::size_t chunk = kFirstChunk;
  const char *phase = nullptr;
};

// Step `job` until about `ms` of wall time (steady_clock) has passed; true once it is done. Each
// call aims at an eighth of `ms` (50 us .. 2 ms) and never asks for more units than the time left
// pays for at kWorstUnitUs, so the slice overruns its budget by at most ~one call's estimate even
// when the cost jumps mid-phase, and a cancel lands within one call. The cap costs ~1-2% in the
// cheapest phases (a few thousand ~2-5 ns units a call, against ~0.2 us of overhead).
bool run_for(overlay_job &job, double ms, pacer &p) {
  const steady::time_point start = steady::now();
  const double target = std::clamp(ms / 8.0, 0.05, 2.0);
  for (;;) {
    const char *ph = job.phase();
    if (ph != p.phase) {
      p.phase = ph;
      p.chunk = kFirstChunk;
    }
    const steady::time_point t0 = steady::now();
    const double left_us = std::max(0.0, (ms - ms_between(start, t0)) * 1000.0);
    const std::size_t cap =
        std::max(kMinChunk, static_cast<std::size_t>(std::min(left_us / kWorstUnitUs, 1e9)));
    const std::size_t n = std::min(p.chunk, cap);
    if (job.step(n))
      return true;
    const steady::time_point t1 = steady::now();
    if (job.phase() == ph) { // a call that crossed a phase timed a mix of costs: learn nothing
      const double took = ms_between(t0, t1);
      const double ideal =
          took > 0.0 ? static_cast<double>(n) * target / took : 2.0 * static_cast<double>(p.chunk);
      const double next = std::clamp(ideal, 0.25 * static_cast<double>(p.chunk),
                                     2.0 * static_cast<double>(p.chunk));
      p.chunk = static_cast<std::size_t>(
          std::clamp(next, static_cast<double>(kMinChunk), static_cast<double>(kMaxChunk)));
    }
    if (ms_between(start, t1) >= ms)
      return false;
  }
}

// The job and its pacing state: shared with the pool chunk running it (threaded mode), so a
// cancelled task's chunk keeps them alive until it stops.
struct job_box {
  job_box(const volume &vol, unsigned bits, const overlay_options &opt) : job(vol, bits, opt) {}
  overlay_job job;
  pacer pace;
};

// A pool chunk's reply (threaded mode), shared by the chunk and its task. The chunk posts it only
// while the task still waits for it, and the task, when it stops, learns under the same lock
// whether it was posted: a chunk stopped before it posts never leaves a message behind, and one
// that had posted is the only kind the registry has to collect.
struct chunk_reply {
  std::mutex mu;
  bool abandoned = false; // the task let go of the chunk (release_job)
  bool posted = false;    // the chunk posted its reply to `channel`
  std::string channel;
};

struct overlay_registry;

struct task_record {
  cvc::app *app = nullptr;
  se::async_scheduler *sched = nullptr;
  std::string tf_path;
  overlay_task_options opt;
  bool threaded = false;
  int pid = -1;
  std::shared_ptr<job_box> box;       // let go of once published, cancelled, or the process ends
  std::shared_ptr<chunk_reply> chunk; // the chunk in flight (threaded), until the process steps on
  bool status_written = false;
  steady::time_point last_status;
  std::weak_ptr<overlay_registry> registry;
  // The process's intrinsics context ((await ...) is an intrinsic): it must outlive every
  // evaluation in the environment, which the record does (the environment owns the record).
  se::memory_tracker tracker;
  se::intrinsics_context ictx;
};

struct overlay_registry {
  std::mutex mu;
  std::unordered_map<int, std::shared_ptr<task_record>> tasks;
  // Reply channels of pool chunks that had posted their reply when their task was stopped. Each
  // reply is then in the scheduler's ingress, in its pending queue (the process no longer waits
  // for it), or already taken by the process; the next sweep drains the ingress, pops what is
  // queued and forgets the channel either way.
  std::vector<std::string> orphans;
};

std::shared_ptr<overlay_registry> registry_of(cvc::app &app, bool create) {
  // The app's own named mutex guards check-then-create, as for the app's state root.
  const cvc::mutex_ptr guard = app.mutex(kRegistryKey);
  boost::mutex::scoped_lock lock(*guard);
  std::shared_ptr<overlay_registry> reg;
  try {
    reg = app.data<std::shared_ptr<overlay_registry>>(kRegistryKey);
  } catch (const std::exception &) {
    // not yet present
  }
  if (!reg && create) {
    reg = std::make_shared<overlay_registry>();
    app.data(kRegistryKey, reg);
  }
  return reg;
}

// Let go of the job (cancelling a chunk that runs it on the pool), of the chunk's reply and of the
// deliver hook. Touches no scheduler: it also runs from the anchor's destructor, inside the
// scheduler (kill_owner, reap, its destructor).
void release_job(task_record &rec) {
  if (rec.box) {
    rec.box->job.cancel(); // a chunk on the pool stops at its next check
    rec.box.reset();
  }
  if (rec.chunk) {
    bool posted = false;
    {
      std::lock_guard<std::mutex> lk(rec.chunk->mu);
      rec.chunk->abandoned = true; // from now on the chunk posts nothing
      posted = rec.chunk->posted;
    }
    if (posted)
      if (std::shared_ptr<overlay_registry> reg = rec.registry.lock()) {
        std::lock_guard<std::mutex> lk(reg->mu);
        reg->orphans.push_back(rec.chunk->channel);
      }
    rec.chunk.reset();
  }
  rec.opt.deliver = nullptr;
}

// What the natives close over: when the environment lets go of it, the process has ended.
struct task_anchor {
  std::shared_ptr<task_record> rec;
  ~task_anchor() {
    try {
      if (rec)
        release_job(*rec);
    } catch (...) {
    }
  }
};

void write_status(task_record &rec, const std::string &text) {
  try {
    cvc::state &s = cvc::state::instance(*rec.app)(rec.tf_path)("overlay_status");
    if (s.value() != text)
      s.value(text);
  } catch (...) {
    // a progress note is best effort
  }
}

void write_progress(task_record &rec) {
  const steady::time_point now = steady::now();
  if (rec.status_written && ms_between(rec.last_status, now) < kStatusPeriodMs)
    return;
  rec.status_written = true;
  rec.last_status = now;
  write_status(rec, rec.box->job.status());
}

// Threaded mode: run the next ~chunk_ms of the job on a background worker (app::compute_async, an
// app background-pool task as cvc::future_pool_task's is) and return the future the process parks
// on. The chunk touches only the job_box and its chunk_reply -- never state or the record -- and
// reaches the scheduler only through post_message, the thread-safe ingress. Unlike
// future_pool_task's, its reply is posted only while the task still waits for it (chunk_reply). It
// holds the box only while it runs: the closure lives from the launch until its pool task ends
// (the pool destroys it on the worker as soon as it returns; compute_async jobs never enter the
// app's thread map, so nothing of them is left to reap), and a chunk may wait in the pool's queue
// behind other jobs first, so a strong capture would keep a stopped task's job alive until its
// chunk got a worker and ran.
se::value_t launch_chunk(task_record &rec) {
  static std::atomic<std::uint64_t> seq{0};
  auto reply = std::make_shared<chunk_reply>();
  // A unique '#' channel, as cvc::launch_pool_task makes one (policy-exempt, kept verbatim).
  reply->channel = se::resolve_channel_key(
      std::string(), "tf.overlay#" + std::to_string(seq.fetch_add(1, std::memory_order_relaxed)));
  const std::weak_ptr<job_box> weak = rec.box;
  const double ms = rec.opt.chunk_ms;
  se::async_scheduler *const sched = rec.sched;
  rec.app->compute_async(
      1, [](int) {},
      [weak, ms, reply, sched] {
        bool ok = true;
        try {
          if (const std::shared_ptr<job_box> box = weak.lock()) // gone: the task was stopped
            run_for(box->job, ms, box->pace);
        } catch (...) {
          ok = false; // the next step finds the job as this left it
        }
        std::lock_guard<std::mutex> lk(reply->mu);
        if (!reply->abandoned) {
          sched->post_message(reply->channel, se::value_t(ok));
          reply->posted = true;
        }
      });
  rec.chunk = reply;
  return se::make_future(reply->channel);
}

// (tf-overlay-step): #t once the job is done (or cancelled) -- the call after the one that
// finished it; else #f after a cooperative slice, or the future of the pool chunk just launched.
se::value_t task_step(task_record &rec) {
  // The process calls again only once the last chunk's reply was delivered to it: that chunk is
  // over, and its reply taken.
  rec.chunk.reset();
  if (!rec.box || rec.box->job.done())
    return se::value_t(true);
  if (rec.threaded) {
    write_progress(rec);
    try {
      return launch_chunk(rec);
    } catch (const std::exception &) {
      // No worker could be started (compute_async throws when the background pool has none and
      // can't create one; a pool with a worker queues the chunk instead): carry on cooperatively.
      rec.threaded = false;
    }
  }
  if (!run_for(rec.box->job, rec.opt.slice_ms, rec.box->pace))
    write_progress(rec);
  // Yield the frame even when this slice finished the job: the next call returns #t at once, so
  // publishing (a few ms of number formatting and state writes) gets a frame of its own.
  return se::value_t(false);
}

// (tf-overlay-publish): hand the overlays over and let go of the job.
se::value_t task_publish(task_record &rec) {
  if (!rec.box || rec.box->job.cancelled()) {
    release_job(rec);
    return se::value_t(false);
  }
  overlays ov = rec.box->job.take();
  rec.box.reset(); // the volume copy, the builders, the partial layers
  std::function<void(overlays &&)> deliver = std::move(rec.opt.deliver);
  rec.opt.deliver = nullptr;
  if (deliver)
    deliver(std::move(ov));
  else
    publish_overlays(cvc::state::instance(*rec.app)(rec.tf_path), ov);
  return se::value_t(true);
}

// Reap the registry's finished processes, then collect orphaned chunk replies. Never holds the
// registry's lock while calling the scheduler: reaping a process destroys its anchor, which may
// take the lock (and record an orphan, which is why the orphans are read after the reaping).
void sweep(se::async_scheduler &sched, overlay_registry &reg) {
  std::vector<int> pids;
  {
    std::lock_guard<std::mutex> lk(reg.mu);
    if (reg.tasks.empty() && reg.orphans.empty())
      return;
    pids.reserve(reg.tasks.size());
    for (const auto &kv : reg.tasks)
      pids.push_back(kv.first);
  }
  std::vector<int> gone;
  for (const int pid : pids) {
    const std::optional<se::process_info> info = sched.get_process_info(pid);
    if (!info || se::is_finished(info->status)) {
      sched.reap(pid);
      gone.push_back(pid);
    }
  }
  std::vector<std::string> orphans;
  {
    std::lock_guard<std::mutex> lk(reg.mu);
    orphans = reg.orphans;
  }
  if (!orphans.empty()) {
    // Each orphan's reply was posted before its task stopped waiting for it, so once the ingress is
    // drained it is in the pending queue (popped here) or was delivered to the process before it
    // ended: either way nothing more of it is to come.
    sched.drain_ingress();
    for (const std::string &ch : orphans)
      sched.pop_pending_message(ch);
  }
  std::vector<std::shared_ptr<task_record>> dead; // destroyed once the lock is let go
  std::lock_guard<std::mutex> lk(reg.mu);
  for (const int pid : gone) {
    auto it = reg.tasks.find(pid);
    if (it != reg.tasks.end()) {
      dead.push_back(std::move(it->second));
      reg.tasks.erase(it);
    }
  }
  for (const std::string &ch : orphans) {
    auto it = std::find(reg.orphans.begin(), reg.orphans.end(), ch);
    if (it != reg.orphans.end())
      reg.orphans.erase(it);
  }
}

double positive_or(double v, double fallback, double floor) {
  return std::isfinite(v) && v > 0.0 ? std::max(v, floor) : fallback;
}

} // namespace

bool threads_available() {
#if defined(__EMSCRIPTEN__) && !defined(__EMSCRIPTEN_PTHREADS__)
  return false;
#else
  return true;
#endif
}

int schedule_overlays(cvc::app &app, const std::string &tf_path, const cvc::volume &vol,
                      unsigned bits, const overlay_task_options &opt) {
  if (tf_path.empty())
    return -1; // nowhere to publish (or to say why)
  auto rec = std::make_shared<task_record>();
  try {
    se::async_scheduler &sched = app.exec_scheduler();
    std::shared_ptr<overlay_registry> reg = registry_of(app, true);
    sweep(sched, *reg);

    rec->app = &app;
    rec->sched = &sched;
    rec->tf_path = tf_path;
    rec->opt = opt;
    rec->opt.slice_ms = positive_or(opt.slice_ms, 4.0, 0.05);
    rec->opt.chunk_ms = positive_or(opt.chunk_ms, 250.0, 0.05);
    rec->threaded = opt.threading != overlay_threading::cooperative && threads_available();
    rec->registry = reg;
    rec->box = std::make_shared<job_box>(vol, bits, opt.overlays);

    cvc::state &root = cvc::state::instance(app);
    rec->ictx.sched = &sched;
    rec->ictx.root = &root;
    rec->ictx.tracker = &rec->tracker;
    rec->ictx.proc = se::make_process(); // a placeholder: intrinsics reach the running process
    rec->ictx.proc->status = se::process_status::ready; // through sched.current_process()
    se::environment_ptr base = se::builtins::make_default_environment();
    se::register_intrinsics(base, &rec->ictx);
    // The task's private scope: the two natives, closing over the anchor (so over the record).
    se::environment_ptr env = se::environment::extend(base);
    {
      auto anchor = std::make_shared<task_anchor>();
      anchor->rec = rec;
      se::builtins::register_fn(env, "tf-overlay-step",
                                [anchor](std::span<const se::value_t>) -> se::value_t {
                                  return task_step(*anchor->rec);
                                });
      se::builtins::register_fn(env, "tf-overlay-publish",
                                [anchor](std::span<const se::value_t>) -> se::value_t {
                                  return task_publish(*anchor->rec);
                                });
    }

    se::execute_options eo;
    eo.name =
        std::string("tf-overlays/") + (rec->threaded ? "threaded " : "cooperative ") + tf_path;
    eo.owner = opt.owner;
    eo.env = env;
    // A task runs for seconds and thousands of frames, so no step or time limit: each frame's work
    // is bounded by slice_ms (cooperative) or costs the scheduler nothing (threaded), and the
    // pump's own per-drain budget bounds the rest. The job's memory is C++-side, not DSL heap
    // (the DSL side is one variable), so no memory budget either.
    eo.max_steps = 0;
    eo.max_time = 0.0;
    eo.max_step_time = 0.0;
    eo.max_memory = 0;
    const int pid = sched.execute(std::string(kProgram), eo);
    rec->pid = pid;
    {
      std::lock_guard<std::mutex> lk(reg->mu);
      reg->tasks[pid] = rec;
    }
    return pid;
  } catch (const std::exception &e) {
    rec->box.reset();
    rec->app = &app;
    rec->tf_path = tf_path;
    write_status(*rec, std::string("unavailable: ") + e.what());
  } catch (...) {
    rec->box.reset();
    rec->app = &app;
    rec->tf_path = tf_path;
    write_status(*rec, "unavailable: an unknown error");
  }
  return -1;
}

void cancel_overlays(cvc::app &app, int pid) {
  std::shared_ptr<overlay_registry> reg = registry_of(app, false);
  if (!reg)
    return;
  std::shared_ptr<task_record> rec;
  {
    std::lock_guard<std::mutex> lk(reg->mu);
    auto it = reg->tasks.find(pid);
    if (it == reg->tasks.end())
      return; // finished and reaped, or not an overlay task: leave it alone
    rec = it->second;
  }
  se::async_scheduler &sched = app.exec_scheduler();
  release_job(*rec); // nothing more is published; a pool chunk stops at its next check
  sched.kill(pid);
  sched.reap(pid);
  {
    std::lock_guard<std::mutex> lk(reg->mu);
    reg->tasks.erase(pid);
  }
  rec.reset();
  sweep(sched, *reg);
}

bool overlays_running(cvc::app &app, int pid) {
  std::shared_ptr<overlay_registry> reg = registry_of(app, false);
  if (!reg)
    return false;
  sweep(app.exec_scheduler(), *reg);
  std::lock_guard<std::mutex> lk(reg->mu);
  return reg->tasks.count(pid) != 0;
}

bool wait_overlays(cvc::app &app, int pid, double timeout_s) {
  se::async_scheduler &sched = app.exec_scheduler();
  const steady::time_point start = steady::now();
  for (;;) {
    if (!overlays_running(app, pid))
      return true;
    if (ms_between(start, steady::now()) >= timeout_s * 1000.0)
      return false;
    // One frame, as an Ariadne Runtime pumps it. sync_step also wakes due sleepers (step() does,
    // sync_run alone does not while nothing is runnable); it steps nothing when everything waits.
    sched.drain_ingress();
    sched.wake_awaiting();
    const int stepped = sched.sync_step();
    sched.sync_run(200000, 0.1);
    if (stepped == 0)
      std::this_thread::sleep_for(std::chrono::milliseconds(1)); // e.g. a pool chunk is running
  }
}

std::size_t overlay_orphaned_replies(cvc::app &app) {
  std::shared_ptr<overlay_registry> reg = registry_of(app, false);
  if (!reg)
    return 0;
  std::lock_guard<std::mutex> lk(reg->mu);
  return reg->orphans.size();
}

} // namespace tf
} // namespace cvc
