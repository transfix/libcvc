// pycvc_exec.h — run state_exec DSL programs in an app's context, and register
// Python callables as DSL functions.
//
// Exec is a HANDLE bound to an explicit app (no module-global): it owns the DSL
// environment + scheduler over that app's state root, so registered Python
// functions and run() programs all see the same shared tree. A Python function
// registered here becomes callable from DSL source like any builtin.
//
// Guarded by CVC_STATE_EXEC (a PUBLIC compile def on the cvc target); when the
// build lacks state_exec, the ctor throws.
#pragma once

#include <memory>
#include <string>

namespace cvc {
class app;
}

// Forward-declare PyObject so this SWIG-visible header needs no <Python.h>.
struct _object;
typedef _object PyObject;

namespace pycvc {

class Exec {
public:
  // Build a DSL environment + scheduler over `app`'s state root.
  explicit Exec(const std::shared_ptr<cvc::app> &app);
  ~Exec();

  // Register a Python callable as a DSL function named `name`. DSL programs run
  // later can call it; args arrive as DSL values converted to Python
  // (int/float/bool/str/None/list/dict) and the return value converts back. A
  // Python exception inside it is contained and surfaced as a run() error, not
  // a crash. The callable is kept alive for this Exec's lifetime.
  void register_fn(const std::string &name, PyObject *callable);

  // Execute a DSL program in this app's context; returns the rendered result
  // (strings raw, nil as "", others via the DSL's printed form). Throws on a
  // parse/eval error (surfaced as a Python exception).
  std::string run(const std::string &src);

private:
  struct ExecImpl;
  std::shared_ptr<ExecImpl> impl_;
};

} // namespace pycvc
