/**
 * @file async_task.h
 * @brief Offload a blocking C++ kernel to the compute pool and resume a state_exec process when it
 *        finishes: the substrate behind (http-get)/(fetch) and any long-running intrinsic.
 *
 * A host intrinsic that must do slow work (an HTTP request, a volume filter, a mesh decimation)
 * must NOT run it inline: the state_exec evaluator deadline is cooperative, so a blocking builtin
 * freezes the drain/render frame for its whole duration. Instead it offloads the work to a
 * compute-pool worker and PARKS the calling process until the worker posts the result back, the
 * same shape that (http-get)/(fetch) already use. These helpers factor it out so a heavy intrinsic
 * is a thin wrapper:
 *
 *   register_fn(env, "volume-blur",       // transparent: self-parks, yields the result dict
 *     [&app, root](args) {
 *       auto in = marshal_in(args);
 *       return cvc::park_on_pool_task(app, root, "vol.reply",
 *         [in] { return marshal_out(run_blur(in)); },
 *         [](const std::string &e) { return error_value(e); });
 *     });
 *   register_fn(env, "volume-blur-async", // future: returns a handle you (await ...)
 *     [&app, root](args) {
 *       auto in = marshal_in(args);
 *       return cvc::future_pool_task(app, root, "vol.reply",
 *         [in] { return marshal_out(run_blur(in)); },
 *         [](const std::string &e) { return error_value(e); });
 *     });
 *
 * TWO RULES the caller MUST honor:
 *   1. Register the verb through the ACTION intrinsics seam only, never the reactive/read lane. A
 *      render frame then cannot reach a parking verb (the render lane forbids parking by design).
 *   2. The `work` kernel runs on a pool worker, OFF the scheduler thread, so it must be
 *      self-contained and thread-safe. Marshal every input in before the offload and the result
 * out; never touch the DSL evaluator or the state tree from `work` (that races the one scheduler
 *      timeline). A pure C++ kernel is the intended body. A Python callable is allowed only with
 *      correct GIL handling and the same no-re-entry rule.
 */
#ifndef CVC_CORE_ASYNC_TASK_H
#define CVC_CORE_ASYNC_TASK_H

#include <cvc/state/state_exec/types.h>
#include <functional>
#include <string>

namespace cvc {

class app;
namespace state_exec {
class async_scheduler;
}

/// The work kernel: runs on a compute-pool worker, returns the DSL value to deliver.
using pool_task_work = std::function<state_exec::value_t()>;
/// Builds the DSL value to deliver when `work` throws (given the exception message), so a waiter
/// never hangs — every path posts exactly one value.
using pool_task_on_error = std::function<state_exec::value_t(const std::string &)>;

/// Kick `work` on one of `a`'s compute-pool workers (OFF the scheduler thread); when it finishes,
/// post its result — or `on_error(message)` if it throws — to a UNIQUE reply channel scoped under
/// `root` ON `sched`, and return that channel key.  `sched` is the scheduler the CALLER drives (so
/// the delivery lands where the parked process will be resumed): the app-wide `a.exec_scheduler()`
/// for an Ariadne host, or a private per-Exec scheduler for pycvc.  Returns immediately.
std::string launch_pool_task(app &a, state_exec::async_scheduler &sched, const std::string &root,
                             const std::string &chan_prefix, pool_task_work work,
                             pool_task_on_error on_error);

/// Transparent form: launch the task and SELF-PARK the current process ON `sched`, resuming with
/// the result value threaded into the enclosing expression.  Call only from a native_fn running
/// under `sched` (sched.current_pid()/current_process() must be valid — i.e. `sched` is the
/// driver).
state_exec::value_t park_on_pool_task(app &a, state_exec::async_scheduler &sched,
                                      const std::string &root, const std::string &chan_prefix,
                                      pool_task_work work, pool_task_on_error on_error);

/// Future form: launch the task and return a future handle you (await ...) or (msg-recv ...) on
/// `sched`.
state_exec::value_t future_pool_task(app &a, state_exec::async_scheduler &sched,
                                     const std::string &root, const std::string &chan_prefix,
                                     pool_task_work work, pool_task_on_error on_error);

} // namespace cvc

#endif // CVC_CORE_ASYNC_TASK_H
