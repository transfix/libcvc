/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// A port's operation in the DSL, in the three forms of
// docs/roadmap/VOLROVER-LIBRARY-PORTS-ROADMAP.md §1.6, from its stepper factory and its parameter
// table alone:
//
//   (<op> inputs... [params])         waits: the calling process parks on the task's completion
//                                     (costing the scheduler nothing) and gets the result, or the
//                                     job's exception raises at the call
//   (<op>-async inputs... [params])   returns a future at once, for (await f)
//   (<op>-start inputs... [params] target)
//                                     returns the task's pid at once; under the state path `target`
//                                     (below the calling program's root) the task publishes the
//                                     node keys (busy, stoppable, status, progress, phase,
//                                     stats.work_ms, pid: cvc/core/job_keys.h) and writes the
//                                     result to `target.result` on the data channel. The form for
//                                     Ariadne actions: it returns in microseconds, under the
//                                     default action_max_time 0.1 s and action_max_steps 5000
//   (job.cancel pid)                  stops a task at its next step (#f: no such task); a -start
//                                     target then reads status "cancelled"
//   (params.defaults name), (params.check name dict)   the op's table (cvc/core/param_dsl.h)
//
// Every form runs the op's job as a sliced task on the app's scheduler (schedule_sliced_job,
// cvc/core/sliced_task.h), named "<op>/cooperative <subject>" or "<op>/threaded <subject>" in (ps),
// with the calling process's owner tag, so closing the document that started it stops it. Killing
// the waiting process cancels its job; so does killing the task's own process.
//
// Arguments. An input is a data object (from state-data-get) or a state path naming a node whose
// data channel holds the object (resolved under the calling program's root, in the calling step).
// The parameters are a dict, read through the op's table (an unknown key, a value of the wrong kind
// or one out of range raises ValueError naming every problem), or nil for the defaults.
//
// A port registers its op once per environment, at setup:
//
//   cvc::job_op op = cvc::make_job_op<seeded_segmentation_params, segmentation_result>(
//       "segment.seeded", 2,
//       [](const cvc::job_call &c, const seeded_segmentation_params &p) {
//         return cvc::segment_seeded_job(*c.input_as<cvc::volume>(0),
//                                        *c.input_as<cvc::point_classes>(1), p);
//       },
//       [](segmentation_result &&r) { return to_dict(std::move(r)); });
//   cvc::register_job_op(env, ictx, app, op);
//   cvc::register_job_intrinsics(env, app);  // job.cancel, params.*
//
// `ictx` is the environment's intrinsics context (its root is the calling program's, its host the
// async host its processes park with; null host: app.exec_async_host()). It and the app must
// outlive the environment's evaluations. Something must pump app.exec_scheduler(), as every Ariadne
// Runtime does. Where a process cannot park (Ariadne's reactive lane, a sync scheduler), the
// waiting form runs the job to the end in the call instead.

#ifndef CVC_CORE_JOB_INTRINSICS_H
#define CVC_CORE_JOB_INTRINSICS_H

#include <boost/any.hpp>
#include <cstddef>
#include <cvc/core/param_dsl.h>
#include <cvc/core/param_table.h>
#include <cvc/core/result_job.h>
#include <cvc/state/state_exec/types.h>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <tuple>
#include <typeinfo>
#include <utility>
#include <vector>

namespace cvc {
class app;
namespace state_exec {
struct intrinsics_context;
}

// One call of an op, as its make function sees it: on the scheduler thread, in the calling step.
class job_call {
public:
  job_call(cvc::app &app, state_exec::intrinsics_context &ictx, std::string op,
           std::vector<state_exec::value_t> inputs, state_exec::value_t params);

  cvc::app &app() const { return *_app; }
  state_exec::intrinsics_context &context() const { return *_ictx; }
  const std::string &op() const { return _op; }
  std::size_t inputs() const { return _inputs.size(); }
  const state_exec::value_t &input(std::size_t i) const;
  const state_exec::value_t &params() const { return _params; } // nil when not given

  // What input i holds: a data object's payload, or the data channel of the node a state path
  // names (a data object a program stored there with state-data-set gives its payload). Throws
  // std::invalid_argument naming the op and the argument.
  boost::any input_any(std::size_t i) const;

  // The object input i names: a T (copied: cvc::volume and cvc::geometry copies share their data),
  // a std::shared_ptr<T> or a std::shared_ptr<const T>. Throws std::invalid_argument naming the op,
  // the argument and the type found.
  template <class T> std::shared_ptr<const T> input_as(std::size_t i) const {
    const boost::any a = input_any(i);
    if (const T *p = boost::any_cast<T>(&a))
      return std::make_shared<const T>(*p);
    if (const std::shared_ptr<T> *p = boost::any_cast<std::shared_ptr<T>>(&a))
      if (*p)
        return *p;
    if (const std::shared_ptr<const T> *p = boost::any_cast<std::shared_ptr<const T>>(&a))
      if (*p)
        return *p;
    throw std::invalid_argument(where(i) + " holds " + a.type().name() + ", not a " +
                                typeid(T).name());
  }

  // "<op>: argument <i+1>", for messages.
  std::string where(std::size_t i) const;

private:
  cvc::app *_app;
  state_exec::intrinsics_context *_ictx;
  std::string _op;
  std::vector<state_exec::value_t> _inputs;
  state_exec::value_t _params;
};

// What an op's make returns: the job, how its result becomes a DSL value (on the scheduler thread,
// once, after the job is done and did not fail), and the subject the process is named by.
struct job_start {
  std::shared_ptr<result_job_base> job;
  std::function<state_exec::value_t()> result;
  std::string subject;
};

// An op: its name, how many inputs come before the optional parameters, its make, and its
// parameter table (registered in the app under the op's name, for params.defaults / params.check).
struct job_op {
  std::string name;
  std::size_t inputs = 0;
  std::function<job_start(const job_call &)> make;
  std::shared_ptr<const param_table_entry> params; // null: the op takes no parameters
  // How its tasks run (sliced_task.h): automatic (threaded where there are threads, cooperative
  // with CVC_SLICED_THREADING=cooperative), or forced either way.
  sliced_threading threading = sliced_threading::automatic;
};

// The parameters of an op that has none: an empty table.
struct no_params {};
template <> struct param_table<no_params> : param_fields<no_params> {
  static constexpr std::tuple<> fields{};
};

// An op from a typed make: the parameters read through P's table (any problem raises ValueError
// listing them all), the job from `make`, and the result made a DSL value by `to_value`.
template <class P, class R>
job_op make_job_op(std::string name, std::size_t inputs,
                   std::function<std::unique_ptr<result_job<R>>(const job_call &, const P &)> make,
                   std::function<state_exec::value_t(R &&)> to_value,
                   std::function<std::string(const job_call &, const P &)> subject = {}) {
  job_op op;
  op.name = name;
  op.inputs = inputs;
  if constexpr (!std::is_same_v<P, no_params>)
    op.params = std::make_shared<const param_table_entry>(make_param_table_entry<P>(name));
  op.make = [name, make = std::move(make), to_value = std::move(to_value),
             subject = std::move(subject)](const job_call &c) -> job_start {
    if constexpr (std::is_same_v<P, no_params>) {
      if (!c.params().is_nil())
        throw std::invalid_argument(name + ": takes no parameters");
    }
    const param_result<P> pr = params_from_value<P>(c.params());
    if (!pr.ok())
      throw std::invalid_argument(name + ": " + pr.message());
    std::shared_ptr<result_job<R>> job(make(c, pr.value));
    if (!job)
      throw std::runtime_error(name + ": no job");
    job_start js;
    js.subject = subject ? subject(c, pr.value) : std::string();
    js.result = [job, to_value]() { return to_value(job->take_result()); };
    js.job = std::move(job);
    return js;
  };
  return op;
}

// Register the op's three forms in `env` (and its table in the app's registry).
void register_job_op(state_exec::environment_ptr env, state_exec::intrinsics_context &ictx,
                     cvc::app &app, const job_op &op);

// (job.cancel pid) and (params.defaults name) / (params.check name dict) in `env`.
void register_job_intrinsics(state_exec::environment_ptr env, cvc::app &app);

} // namespace cvc

#endif // CVC_CORE_JOB_INTRINSICS_H
