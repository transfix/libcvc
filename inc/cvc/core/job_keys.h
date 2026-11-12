/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// The keys a node publishes for its job (docs/roadmap/VOLROVER-LIBRARY-PORTS-ROADMAP.md §1.9), as
// fe_lab does, under a base path:
//
//   busy            1 while a job runs, else 0
//   stoppable       1 while the running job can be stopped, else 0
//   status          the job's note while it runs ("meshing..."), then its summary, its failure or
//                   "cancelled"
//   progress        0..100, an int
//   phase           "<phase> 42%" while it runs, "" when idle
//   stats.work_ms   the last job's wall time in ms, rounded (unless the node times its own work)
//   pid             the job's task (the DSL's -start forms write it)
//
// A component binds a `progress:` widget to `progress`, labelled by `phase`, with a Stop button
// shown while `stoppable`, as fe_controls does. The DSL's `<op>-start` forms publish the same keys
// under their target (cvc/core/job_intrinsics.h).
//
//   auto keys = std::make_shared<cvc::job_keys>(cvc::state::instance(app), "doc.grid.segment");
//   int pid = cvc::schedule_published_job(app, job, opt, keys, "segmenting...",
//                                         [&](cvc::sliced_job &j) { take the result });
//
// Every write goes through cvc::state::childThroughLinks (as Ariadne's binds write), never throws,
// and is a no-op when the value is unchanged. The writes are the scheduler thread's: call these
// from the thread that pumps it (a node's tick, a task's hooks).

#ifndef CVC_CORE_JOB_KEYS_H
#define CVC_CORE_JOB_KEYS_H

#include <boost/any.hpp>
#include <chrono>
#include <cvc/core/sliced_task.h>
#include <cvc/state/state.h>
#include <exception>
#include <functional>
#include <memory>
#include <string>
#include <string_view>

namespace cvc {
class app;

class job_keys {
public:
  // The keys under `base` below `root` ("" = under root itself). `pins` are held while this
  // lives: the chain down to a chrooted root, so a document's node outlives a task it started.
  // `time_jobs`: write stats.work_ms when a job finishes (a node that times a whole chain of
  // commands itself, as fe_lab does, passes false).
  job_keys(cvc::state &root, std::string base, cvc::pin_vector pins = {}, bool time_jobs = true);

  // The absolute path of a key under the base ("busy" -> "<base>.busy").
  std::string path(std::string_view key) const;

  // Idle: busy 0, stoppable 0, progress 0, phase "" (a node's keys at realize time).
  void idle();
  // A job starts: busy 1, stoppable, progress 0, status, phase "". Starts the work clock.
  void started(const std::string &status, bool stoppable = true);
  // While it runs: progress (0..100) and phase ("<phase> 42%"; "" for a null phase).
  void progress(int percent, const char *phase);
  void progress(const sliced_job &job);
  // A stop was asked for: status "stopping...".
  void stopping();
  // The job ended: status, progress 100, stoppable 0, busy 0, phase "", and stats.work_ms.
  void finished(const std::string &status);
  // The same with the job's own status (its summary, its failure, or "cancelled").
  void finished(const sliced_job &job);
  // The task's pid ("pid").
  void pid(int pid);

  // One key, never throwing (a read-only key through a link keeps its value).
  template <class T> void write(std::string_view key, const T &v) {
    try {
      _root->childThroughLinks(path(key)).value(v);
    } catch (const std::exception &) {
    }
  }
  void write(std::string_view key, const std::string &v);
  void write(std::string_view key, const char *v) { write(key, std::string(v)); }
  // One key's data channel (the DSL's -start forms write the result to "result").
  void write_data(std::string_view key, const boost::any &v);

  // ms since started() (0 before it).
  double work_ms() const;

private:
  cvc::state *_root;
  std::string _base;
  cvc::pin_vector _pins;
  bool _time_jobs;
  bool _running = false;
  std::chrono::steady_clock::time_point _t0{};
};

// Submit `job` as schedule_sliced_job does, with `keys` published from its hooks: started(status)
// now (and pid once it has one), progress from on_progress, and finished() once it ends, from
// on_done or on_cancelled. Before finished(), `on_result` (if given) gets a job that ended
// normally (not failed or cancelled, for a result_job: failed() false) so the caller can take
// its result; an exception it throws is reported in status. opt's own hooks run too, first. A task
// that cannot start returns -1 and publishes finished("error: the job could not start: <why>").
int schedule_published_job(cvc::app &app, std::shared_ptr<sliced_job> job, sliced_task_options opt,
                           std::shared_ptr<job_keys> keys, const std::string &status,
                           std::function<void(sliced_job &)> on_result = {},
                           std::string *why = nullptr);

// "<phase> 42%" (phase null or empty: "").
std::string job_phase_text(const char *phase, int percent);

} // namespace cvc

#endif // CVC_CORE_JOB_KEYS_H
