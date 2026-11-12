/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// The generic intrinsic forms of a port's operation (cvc/core/job_intrinsics.h).
//
// The waiting and -async forms are one async function whose work runs by itself
// (docs/STATE_EXEC_EXCEPTIONS.md 3.12): its start schedules the job as a sliced task, and the
// task's on_done settles the call (on the scheduler thread). A cancelled call (its process killed,
// the host shut down) stops the job through the call's on_cancel hook, and from then on the task
// posts nothing for it: a mutex orders the two, since the hook may run on another thread. Where the
// calling process cannot park, the call's kernel runs the job to the end instead.
//
// The -start form is a plain native: it schedules the job with the node keys published under its
// target (job_keys, schedule_published_job) and returns the pid.

#include <cstdint>
#include <cvc/core/app.h>
#include <cvc/core/job_intrinsics.h>
#include <cvc/core/job_keys.h>
#include <cvc/core/sliced_task.h>
#include <cvc/state/state.h>
#include <cvc/state/state_exec/async_host.h>
#include <cvc/state/state_exec/builtins.h>
#include <cvc/state/state_exec/exceptions.h>
#include <cvc/state/state_exec/intrinsics.h>
#include <cvc/state/state_exec/process.h>
#include <cvc/state/state_exec/scheduler_base.h>
#include <exception>
#include <memory>
#include <mutex>
#include <span>
#include <stdexcept>
#include <string>
#include <utility>
#include <variant>
#include <vector>

namespace cvc {

namespace se = cvc::state_exec;

job_call::job_call(cvc::app &app, se::intrinsics_context &ictx, std::string op,
                   std::vector<se::value_t> inputs, se::value_t params)
    : _app(&app), _ictx(&ictx), _op(std::move(op)), _inputs(std::move(inputs)),
      _params(std::move(params)) {}

std::string job_call::where(std::size_t i) const {
  return _op + ": argument " + std::to_string(i + 1);
}

const se::value_t &job_call::input(std::size_t i) const {
  if (i >= _inputs.size())
    throw std::invalid_argument(where(i) + " is missing");
  return _inputs[i];
}

boost::any job_call::input_any(std::size_t i) const {
  const se::value_t &v = input(i);
  if (const se::data_object_ptr *d = std::get_if<se::data_object_ptr>(&v.v)) {
    if (!*d || (*d)->payload.empty())
      throw std::invalid_argument(where(i) + " is an empty data object");
    return (*d)->payload;
  }
  if (const std::string *path = std::get_if<std::string>(&v.v)) {
    cvc::state *node = se::pinned_root(_ictx).findThroughLinks(*path);
    if (!node)
      throw std::invalid_argument(where(i) + ": no state node at '" + *path + "'");
    boost::any a = node->data();
    // What a program stored with state-data-set: a data object kept as the DSL value holding it.
    if (const se::value_t *dv = boost::any_cast<se::value_t>(&a))
      if (const se::data_object_ptr *d = std::get_if<se::data_object_ptr>(&dv->v))
        if (*d)
          a = (*d)->payload;
    if (a.empty())
      throw std::invalid_argument(where(i) + ": '" + *path + "' holds no data");
    return a;
  }
  throw se::dsl_error(
      "TypeError", where(i) + " is a data object or a state path, not " + param_detail::dsl_kind(v),
      "native");
}

namespace {

// The owner tag of the process making the call (a document's), so its tasks die with it.
std::string caller_owner(se::intrinsics_context &ictx) {
  if (ictx.sched) {
    const se::process_ptr &p = ictx.sched->current_process();
    if (p)
      return p->owner;
  }
  return ictx.proc ? ictx.proc->owner : std::string();
}

// The op's job from a call's arguments (the target, for -start, already taken off the end).
job_start start_from_args(const job_op &op, se::intrinsics_context &ictx, cvc::app &app,
                          std::span<const se::value_t> args, const char *form) {
  if (args.size() != op.inputs && args.size() != op.inputs + 1)
    throw se::dsl_error("TypeError",
                        op.name + form + ": takes " + std::to_string(op.inputs) + " input" +
                            (op.inputs == 1 ? "" : "s") +
                            (op.params ? " and an optional dict of parameters" : std::string()) +
                            (std::string(form) == "-start" ? ", then a target path" : "") +
                            ", not " + std::to_string(args.size()) + " argument" +
                            (args.size() == 1 ? "" : "s"),
                        "native");
  std::vector<se::value_t> inputs(args.begin(), args.begin() + op.inputs);
  se::value_t params = args.size() > op.inputs ? args[op.inputs] : se::nil_value;
  const job_call call(app, ictx, op.name, std::move(inputs), std::move(params));
  job_start js = op.make(call);
  if (!js.job || !js.result)
    throw std::runtime_error(op.name + ": its make gave no job");
  return js;
}

// What the call and its task share: whether the call was cancelled (its on_cancel hook ran), under
// the lock the task holds while it settles the call.
struct call_link {
  std::mutex mu;
  bool abandoned = false;
};

void settle(se::async_host &host, const std::shared_ptr<se::async_call> &call,
            const std::shared_ptr<call_link> &link, se::value_t value, std::exception_ptr error) {
  std::lock_guard<std::mutex> lk(link->mu);
  if (link->abandoned)
    return; // cancelled first: its host may be gone, and nothing more is posted
  call->settle(std::move(value), error);
  host.finish(call);
}

} // namespace

void register_job_op(se::environment_ptr env, se::intrinsics_context &ictx, cvc::app &app,
                     const job_op &op) {
  if (op.name.empty() || !op.make)
    throw std::invalid_argument("register_job_op: an op needs a name and a make");
  if (op.params)
    register_param_table(app, *op.params);
  auto shared_op = std::make_shared<const job_op>(op);
  se::intrinsics_context *ctx = &ictx;
  cvc::app *a = &app;
  se::async_host *host = ictx.host ? ictx.host : &app.exec_async_host();

  // (<op> ...) and (<op>-async ...).
  se::register_async_fn(
      env, *host, op.name,
      [shared_op, ctx, a, host](std::span<const se::value_t> args) -> se::async_work {
        job_start js = start_from_args(*shared_op, *ctx, *a, args, "");
        const std::string owner = caller_owner(*ctx);
        se::async_work work;
        // Where the process cannot park: the job to the end, in the call, a slice at a time so a
        // cancel (the process killed) stops it.
        work.kernel = [js](se::async_call &call) -> se::value_t {
          while (!js.job->step(4096))
            if (call.cancelled()) {
              js.job->cancel();
              js.job->step(1);
              throw std::runtime_error("cancelled");
            }
          if (js.job->failed())
            std::rethrow_exception(js.job->exception());
          return js.result();
        };
        work.start = [js, owner, shared_op, a, host](const std::shared_ptr<se::async_call> &call) {
          auto link = std::make_shared<call_link>();
          sliced_task_options o;
          o.kind = shared_op->name;
          o.subject = js.subject;
          o.owner = owner;
          o.threading = shared_op->threading;
          o.on_done = [js, link, call, host](sliced_job &) {
            se::value_t value;
            std::exception_ptr error;
            if (js.job->failed()) {
              error = js.job->exception();
            } else {
              try {
                value = js.result();
              } catch (...) {
                error = std::current_exception();
              }
            }
            settle(*host, call, link, std::move(value), error);
          };
          o.on_cancelled = [link, call, host, name = shared_op->name](sliced_job &) {
            settle(*host, call, link, se::nil_value,
                   std::make_exception_ptr(std::runtime_error(name + ": cancelled")));
          };
          std::string why;
          const int pid = schedule_sliced_job(*a, js.job, o, &why);
          if (pid < 0)
            throw std::runtime_error(shared_op->name + ": the job could not start: " + why);
          std::weak_ptr<sliced_job> weak = js.job;
          call->on_cancel([link, weak] {
            {
              std::lock_guard<std::mutex> lk(link->mu);
              link->abandoned = true;
            }
            if (std::shared_ptr<sliced_job> j = weak.lock())
              j->cancel(); // thread-safe; the task ends at its next step
          });
        };
        return work;
      });

  // (<op>-start ... target).
  se::builtins::register_fn(
      env, op.name + "-start",
      [shared_op, ctx, a](std::span<const se::value_t> args) -> se::value_t {
        const std::string name = shared_op->name + "-start";
        if (args.empty())
          throw se::dsl_error("TypeError", name + ": the last argument is the target path",
                              "native");
        const std::string *target = std::get_if<std::string>(&args.back().v);
        if (!target || target->empty())
          throw se::dsl_error("TypeError",
                              name + ": the last argument is the target path (a string), not " +
                                  param_detail::dsl_kind(args.back()),
                              "native");
        job_start js = start_from_args(*shared_op, *ctx, *a, args.first(args.size() - 1), "-start");
        // The keys live under the calling program's root, held with its chain while the task runs.
        cvc::state &root = se::pinned_root(ctx);
        auto keys = std::make_shared<job_keys>(root, *target, ctx->root_pins);
        keys->write_data("result", boost::any()); // no stale result while this one runs
        sliced_task_options o;
        o.kind = shared_op->name;
        o.subject = js.subject.empty() ? *target : js.subject;
        o.owner = caller_owner(*ctx);
        o.threading = shared_op->threading;
        auto on_result = [js, keys](sliced_job &) {
          keys->write_data("result", boost::any(js.result()));
        };
        std::string why;
        const int pid =
            schedule_published_job(*a, js.job, o, keys, shared_op->name + "...", on_result, &why);
        if (pid < 0)
          throw std::runtime_error(name + ": the job could not start: " + why);
        return se::value_t(static_cast<std::int64_t>(pid));
      });
}

void register_job_intrinsics(se::environment_ptr env, cvc::app &app) {
  cvc::app *a = &app;
  se::builtins::register_fn(
      env, "job.cancel", [a](std::span<const se::value_t> args) -> se::value_t {
        const std::int64_t *pid =
            args.size() == 1 ? std::get_if<std::int64_t>(&args[0].v) : nullptr;
        if (!pid)
          throw se::dsl_error("TypeError", "job.cancel: takes a task's pid (an integer)", "native");
        return se::value_t(stop_sliced_job(*a, static_cast<int>(*pid)));
      });
  register_param_intrinsics(env, app);
}

} // namespace cvc
