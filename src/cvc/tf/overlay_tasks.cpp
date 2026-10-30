/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::tf overlay tasks: an overlay_job run as a sliced task (cvc/core/sliced_task.h) on the app's
// scheduler, which writes its progress notes to <tf>.overlay_status and publishes the overlays (or
// hands them to the deliver hook) when it is done. See cvc/tf/overlay_tasks.h for the contract.

#include <cvc/core/app.h>
#include <cvc/core/sliced_task.h>
#include <cvc/state/state.h>
#include <cvc/tf/overlay_tasks.h>
#include <cvc/volume/volume.h>
#include <exception>
#include <functional>
#include <memory>
#include <string>
#include <utility>

namespace cvc {
namespace tf {

namespace {

// An overlay_job as a sliced_job.
class overlay_task_job : public cvc::sliced_job {
public:
  template <class Source>
  overlay_task_job(Source &&source, unsigned bits, const overlay_options &opt)
      : job(std::forward<Source>(source), bits, opt) {}
  bool step(std::size_t budget) override { return job.step(budget); }
  bool done() const override { return job.done(); }
  void cancel() override { job.cancel(); }
  bool cancelled() const override { return job.cancelled(); }
  const char *phase() const override { return job.phase(); }
  double progress() const override { return job.progress(); }
  std::string status() const override { return job.status(); }
  overlay_job job;
};

void write_status(cvc::app &app, const std::string &tf_path, const std::string &text) {
  try {
    cvc::state &s = cvc::state::instance(app)(tf_path)("overlay_status");
    if (s.value() != text)
      s.value(text);
  } catch (...) {
    // a progress note is best effort
  }
}

sliced_threading threading_of(overlay_threading t) {
  switch (t) {
  case overlay_threading::cooperative:
    return sliced_threading::cooperative;
  case overlay_threading::threaded:
    return sliced_threading::threaded;
  case overlay_threading::automatic:
    break;
  }
  return sliced_threading::automatic;
}

template <class Source>
int schedule_task(cvc::app &app, const std::string &tf_path, Source &&source, unsigned bits,
                  const overlay_task_options &opt) {
  if (tf_path.empty())
    return -1; // nowhere to publish (or to say why)
  std::shared_ptr<overlay_task_job> job;
  try {
    job = std::make_shared<overlay_task_job>(std::forward<Source>(source), bits, opt.overlays);
  } catch (const std::exception &e) {
    write_status(app, tf_path, std::string("unavailable: ") + e.what());
    return -1;
  }
  sliced_task_options so;
  so.kind = "tf-overlays";
  so.subject = tf_path;
  so.threading = threading_of(opt.threading);
  so.slice_ms = opt.slice_ms;
  so.chunk_ms = opt.chunk_ms;
  so.owner = opt.owner;
  cvc::app *const a = &app;
  so.on_progress = [a, tf_path](const sliced_job &j) { write_status(*a, tf_path, j.status()); };
  so.on_done = [a, tf_path, deliver = opt.deliver](sliced_job &j) {
    overlays ov = static_cast<overlay_task_job &>(j).job.take();
    if (deliver)
      deliver(std::move(ov));
    else
      publish_overlays(cvc::state::instance(*a)(tf_path), ov);
  };
  std::string why;
  const int pid = schedule_sliced_job(app, std::move(job), so, &why);
  if (pid < 0)
    write_status(app, tf_path, "unavailable: " + why);
  return pid;
}

} // namespace

bool threads_available() { return cvc::threads_available(); }

int schedule_overlays(cvc::app &app, const std::string &tf_path, const cvc::volume &vol,
                      unsigned bits, const overlay_task_options &opt) {
  return schedule_task(app, tf_path, vol, bits, opt);
}

int schedule_overlays(cvc::app &app, const std::string &tf_path,
                      std::shared_ptr<const contour_mesh> mesh, unsigned bits,
                      const overlay_task_options &opt) {
  return schedule_task(app, tf_path, std::move(mesh), bits, opt);
}

void cancel_overlays(cvc::app &app, int pid) { cancel_sliced_job(app, pid); }

bool overlays_running(cvc::app &app, int pid) { return sliced_job_running(app, pid); }

bool wait_overlays(cvc::app &app, int pid, double timeout_s) {
  return wait_sliced_job(app, pid, timeout_s);
}

std::size_t overlay_orphaned_replies(cvc::app &app) { return sliced_job_orphaned_replies(app); }

} // namespace tf
} // namespace cvc
