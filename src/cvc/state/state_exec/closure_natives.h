// Private: the native functions defclass makes, as named types rather than lambdas, so the cycle
// collector (heap.cpp) can see what they hold through std::function::target<T>(). A lambda's
// captures are invisible to it, and a class's methods capture the environment the class is bound
// in, which closes a cycle (env -> class dict -> method -> closure -> env).
#ifndef CVC_STATE_EXEC_CLOSURE_NATIVES_H
#define CVC_STATE_EXEC_CLOSURE_NATIVES_H

#include <cvc/state/state_exec/types.h>
#include <span>
#include <string>

namespace cvc::state_exec::detail {

/// A defclass method: runs `method` (self first, then the call's arguments) on a fresh evaluator
/// over `global_env`.
struct class_method {
  closure_ptr method;
  environment_ptr global_env;
  value_t operator()(std::span<const value_t> args) const;
};

/// A defclass constructor: a new instance dict whose __class__ is `cls_dict`, passed to the
/// class's init method if it has one.
struct class_constructor {
  dict_ptr cls_dict;
  value_t operator()(std::span<const value_t> init_args) const;
};

/// The init or finalize a class gets when it defines none: the superclass's `name` method, if
/// `super_cls` has one; otherwise init returns self (the first argument) and finalize nil.
struct class_inherited_method {
  dict_ptr super_cls; // may be null: no superclass
  std::string name;   // "init" or "finalize"
  value_t operator()(std::span<const value_t> args) const;
};

} // namespace cvc::state_exec::detail

#endif // CVC_STATE_EXEC_CLOSURE_NATIVES_H
