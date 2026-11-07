// Private: whether a dict is one of the built-in exception class dicts, which are read-only
// (docs/STATE_EXEC_THREADED.md 5 item 1). The shape test keeps the lookup off the hot paths: a
// built-in class dict has 2-3 entries, `__name__` first (exceptions.cpp, classes()), and can no
// longer grow, while an instance starts with `__class__`, so an instance fails on its first key.
#ifndef CVC_STATE_EXEC_BUILTIN_CLASS_H
#define CVC_STATE_EXEC_BUILTIN_CLASS_H

#include <cvc/state/state_exec/exceptions.h>
#include <cvc/state/state_exec/types.h>
#include <variant>

namespace cvc::state_exec::detail {

inline bool is_class_dict_shape(const ordered_map &d) noexcept {
  return d.size() <= 3 && !d.empty() && d.front().first == "__name__";
}

inline bool is_builtin_class(const ordered_map &d) noexcept {
  return is_class_dict_shape(d) && is_builtin_class_dict(&d);
}

inline bool is_builtin_class(const value_t &v) noexcept {
  const auto *d = std::get_if<dict_ptr>(&v.v);
  return d && *d && is_builtin_class(**d);
}

} // namespace cvc::state_exec::detail

#endif // CVC_STATE_EXEC_BUILTIN_CLASS_H
