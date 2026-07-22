// pycvc_exec.h — Python-facing facade over the state_exec DSL runtime
// (cvc::state_exec: a Lisp-like program evaluator + round-robin process
// scheduler over the cvc::state tree).
//
// SWIG-safe like the other facades: forward-declared pimpl only, so the SWIG
// parser sees just std::string signatures. ONLY pycvc_exec.cpp includes
// libcvc's state_exec headers, and that whole TU is guarded by CVC_STATE_EXEC
// (a PUBLIC compile def carried by the `cvc` target, like CVC_USING_CUDA) —
// when libcvc is built without state_exec, the methods throw a clear error.
//
// An Exec builds a scheduler + intrinsics environment bound to the SAME
// process state-tree root as pycvc.State (cvc::state::instance(process_ctx())),
// so a DSL program can read state written from Python via (state-get "...")
// and write state visible to Python via (state-set "..." "...").
#pragma once

#include <memory>
#include <string>

namespace pycvc {

// Opaque implementation holder (scheduler, intrinsics context, host process,
// environment); defined only in pycvc_exec.cpp so no state_exec types cross
// the SWIG parser.
struct ExecImpl;

class Exec {
public:
  // Build a scheduler + intrinsics env on the process state-tree root (the
  // same root pycvc.State binds to). Throws if libcvc was built without
  // state_exec.
  Exec();
  ~Exec();

  // Parse and run `src` to completion, returning the program's result rendered
  // as a string:
  //   string      -> the raw string (no surrounding quotes)
  //   int / float -> its decimal form ("5", "3.14")
  //   bool        -> "#t" / "#f"   (the DSL's own literals)
  //   nil         -> ""            (no value)
  //   list / dict -> the DSL's printed form ("(1 2 3)", "{\"k\": 1}")
  // Throws on a parse/runtime error (surfaced as a Python exception) or if the
  // program produced no result.
  std::string run_program(const std::string &src);

private:
  std::shared_ptr<ExecImpl> impl_;
};

} // namespace pycvc
