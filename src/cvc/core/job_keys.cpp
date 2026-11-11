/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// The node keys of a job (cvc/core/job_keys.h): what fe_lab publishes, for any sliced task.

#include <algorithm>
#include <cmath>
#include <cvc/core/job_keys.h>
#include <cvc/core/result_job.h>
#include <exception>
#include <memory>
#include <string>
#include <utility>

namespace cvc {

std::string job_phase_text(const char *phase, int percent) {
  if (!phase || !*phase)
    return std::string();
  return std::string(phase) + " " + std::to_string(percent) + "%";
}

job_keys::job_keys(cvc::state &root, std::string base, cvc::pin_vector pins, bool time_jobs)
    : _root(&root), _base(std::move(base)), _pins(std::move(pins)), _time_jobs(time_jobs) {}

std::string job_keys::path(std::string_view key) const {
  if (_base.empty())
    return std::string(key);
  return _base + cvc::state::SEPARATOR + std::string(key);
}

void job_keys::write(std::string_view key, const std::string &v) {
  try {
    _root->childThroughLinks(path(key)).value(v);
  } catch (const std::exception &) {
  }
}

void job_keys::write_data(std::string_view key, const boost::any &v) {
  try {
    _root->childThroughLinks(path(key)).data(v);
  } catch (const std::exception &) {
  }
}

void job_keys::idle() {
  write<int>("busy", 0);
  write<int>("stoppable", 0);
  write<int>("progress", 0);
  write("phase", std::string());
}

void job_keys::started(const std::string &status, bool stoppable) {
  _running = true;
  _t0 = std::chrono::steady_clock::now();
  write<int>("busy", 1);
  write<int>("stoppable", stoppable ? 1 : 0);
  write<int>("progress", 0);
  write("status", status);
  write("phase", std::string());
}

void job_keys::progress(int percent, const char *phase) {
  percent = std::clamp(percent, 0, 100);
  write<int>("progress", percent);
  write("phase", job_phase_text(phase, percent));
}

void job_keys::progress(const sliced_job &job) {
  const double p = job.progress();
  const int pct =
      std::isfinite(p) ? static_cast<int>(std::floor(100.0 * std::clamp(p, 0.0, 1.0))) : 0;
  progress(pct, job.phase());
}

void job_keys::stopping() { write("status", std::string("stopping...")); }

void job_keys::finished(const std::string &status) {
  write("status", status);
  write<int>("progress", 100);
  write<int>("stoppable", 0);
  write<int>("busy", 0);
  write("phase", std::string());
  if (_time_jobs && _running)
    write<double>("stats.work_ms", std::round(work_ms()));
  _running = false;
}

void job_keys::finished(const sliced_job &job) { finished(job.status()); }

void job_keys::pid(int pid) { write<int>("pid", pid); }

double job_keys::work_ms() const {
  if (!_running)
    return 0.0;
  return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - _t0).count();
}

int schedule_published_job(cvc::app &app, std::shared_ptr<sliced_job> job, sliced_task_options opt,
                           std::shared_ptr<job_keys> keys, const std::string &status,
                           std::function<void(sliced_job &)> on_result, std::string *why) {
  if (!keys) {
    if (why)
      *why = "no keys";
    return -1;
  }
  keys->started(status);
  std::function<void(const sliced_job &)> user_progress = std::move(opt.on_progress);
  std::function<void(sliced_job &)> user_done = std::move(opt.on_done);
  std::function<void(sliced_job &)> user_cancelled = std::move(opt.on_cancelled);
  opt.on_progress = [keys, user_progress](const sliced_job &j) {
    if (user_progress)
      user_progress(j);
    keys->progress(j);
  };
  opt.on_done = [keys, user_done, on_result](sliced_job &j) {
    if (user_done)
      user_done(j);
    const auto *rj = dynamic_cast<const result_job_base *>(&j);
    std::string fail;
    if (on_result && !(rj && rj->failed())) {
      try {
        on_result(j);
      } catch (const std::exception &e) {
        fail = std::string("failed: ") + e.what();
      } catch (...) {
        fail = "failed: an unknown exception";
      }
    }
    keys->finished(fail.empty() ? j.status() : fail);
  };
  opt.on_cancelled = [keys, user_cancelled](sliced_job &j) {
    if (user_cancelled)
      user_cancelled(j);
    keys->finished(std::string("cancelled"));
  };
  std::string w;
  const int pid = schedule_sliced_job(app, std::move(job), opt, &w);
  if (pid < 0) {
    keys->finished("error: the job could not start: " + w);
    if (why)
      *why = w;
    return -1;
  }
  keys->pid(pid);
  return pid;
}

} // namespace cvc
