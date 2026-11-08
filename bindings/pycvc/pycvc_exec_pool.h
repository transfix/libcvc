// pycvc_exec_pool.h — run many DSL programs at once on an app's exec_pool
// (docs/STATE_EXEC_THREADED.md 4.12; S1-Q5).
//
// ExecPool is a HANDLE bound to an explicit app, like Exec: it owns `size` pooled Execs, each its
// own execution group, and runs a batch of programs across them on the app's pool, fork-join.
#pragma once

#include <memory>
#include <string>

namespace cvc {
class app;
} // namespace cvc

// Forward-declare PyObject so this SWIG-visible header needs no <Python.h>.
struct _object;
typedef _object PyObject;

namespace pycvc {

class ExecPool {
public:
  // Python takes keywords: ExecPool(app, threads=None, size=None, prefix="exec_pool",
  // fresh_env=False, idle_timeout=60.0).
  //
  // The app has one pool (app::exec_pool()): the first caller that builds it sets its size, from
  // `threads` (an int, 0 or more), else the tree's state_exec.threads. A later different request
  // keeps the built size and logs a warning; `threads` reports the real one. `size` (an int, 1 or
  // more) is the number of pooled Execs, `threads` by default, at least 1. Pooled Exec i is
  // chrooted at "<prefix>.<i>", so its programs' state keys resolve under it. Top-level
  // definitions persist per pooled Exec, unless `fresh_env` gives each program a fresh env. A
  // program parked for `idle_timeout` seconds with nothing pending is killed, as in Exec.run.
  // Anything else, or a negative or NaN idle_timeout, raises ValueError (std::invalid_argument).
  //
  // The pooled Execs are not exposed: no Python thread reaches a pooled scheduler.
  explicit ExecPool(const std::shared_ptr<cvc::app> &app, PyObject *threads = nullptr,
                    PyObject *size = nullptr, const std::string &prefix = "exec_pool",
                    bool fresh_env = false, double idle_timeout = 60.0);
  ~ExecPool();

  // Run each program of `programs` (a list of str) on a pooled Exec, all at once, fork-join on
  // the app's pool, with the GIL released for the whole batch, until every program has ended or
  // passed its idle limit. Returns a list of their results in program order, each rendered as
  // Exec.run renders it. A program that fails makes run_all raise its error, as Exec.run would,
  // once every program has ended (the first failing program's, in program order).
  //
  // Program i runs on pooled Exec i % size; each pooled Exec runs its programs one at a time, in
  // order, so the programs sharing its top-level definitions never interleave. A run_all or
  // register_* call while another is in progress (on another thread, or from a function the
  // batch calls) raises RuntimeError.
  PyObject *run_all(PyObject *programs);

  // Register a Python callable as a DSL function in every pooled Exec, as Exec.register_fn does.
  // It takes the GIL for each call, so the calls serialize on it.
  void register_fn(const std::string &name, PyObject *callable);
  // Register a Python callable as an async DSL function in every pooled Exec, as
  // Exec.register_async_fn does.
  void register_async_fn(const std::string &name, PyObject *callable, bool with_call = false);

  // The app pool's worker count, the real one (in Python, the read-only property `threads`).
  unsigned threads() const;

  // Kill what is still running, wait for its kernels and a batch in progress (with the GIL
  // released), and destroy the pooled Execs. Idempotent; afterwards run_all and register_* raise
  // RuntimeError. From inside its own batch (a function a program calls) it raises RuntimeError.
  void close();

private:
  struct ExecPoolImpl;
  std::shared_ptr<ExecPoolImpl> impl_;
};

} // namespace pycvc
