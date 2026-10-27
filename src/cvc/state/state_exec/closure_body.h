// closure_body.h — a closure's body as the one expression a call evaluates (private to the
// evaluators and builtins).
#ifndef CVC_STATE_EXEC_CLOSURE_BODY_H
#define CVC_STATE_EXEC_CLOSURE_BODY_H

#include <cvc/state/state_exec/types.h>
#include <memory>
#include <type_traits>
#include <variant>
#include <vector>

namespace cvc::state_exec::detail {

/// Whether `a` is `b` as a body form: the same object for a list (or anything else held by
/// pointer), an equal atom otherwise. (A native function can't be compared: never.)
inline bool same_form(const value_t &a, const value_t &b) {
  if (a.v.index() != b.v.index())
    return false;
  return std::visit(
      [&](const auto &x) -> bool {
        using T = std::decay_t<decltype(x)>;
        const T &y = std::get<T>(b.v);
        if constexpr (std::is_same_v<T, native_fn>)
          return false;
        else if constexpr (std::is_same_v<T, std::monostate>)
          return true;
        else if constexpr (std::is_same_v<T, list_ptr> || std::is_same_v<T, dict_ptr> ||
                           std::is_same_v<T, closure_ptr> ||
                           std::is_same_v<T, data_object_ptr> ||
                           std::is_same_v<T, generator_ptr>)
          return x.get() == y.get();
        else
          return x == y;
      },
      a.v);
}

/// (begin forms...): a body of several forms as one expression.
inline value_t begin_of(const std::vector<value_t> &forms) {
  std::vector<value_t> exprs;
  exprs.reserve(1 + forms.size());
  exprs.emplace_back(symbol{"begin"});
  exprs.insert(exprs.end(), forms.begin(), forms.end());
  return make_list(std::move(exprs));
}

/// Make `cls.body_expr` for a body of several forms (when the evaluator makes the closure,
/// before anything else can see it).
inline void cache_body_expr(closure &cls) {
  if (cls.body.size() > 1)
    cls.body_expr = begin_of(cls.body);
}

/// The expression a call of `cls` evaluates: its one form, or (begin body...), the one made with
/// the closure while it still matches the body (else a new one).
inline value_t body_expr_of(const closure &cls) {
  if (cls.body.size() == 1)
    return cls.body[0];
  if (const auto *lp = std::get_if<list_ptr>(&cls.body_expr.v);
      lp && *lp && (*lp)->size() == cls.body.size() + 1) {
    const std::vector<value_t> &cached = **lp;
    const auto *head = std::get_if<symbol>(&cached[0].v);
    bool current = head && head->name == "begin";
    for (std::size_t i = 0; current && i < cls.body.size(); ++i)
      current = same_form(cached[i + 1], cls.body[i]);
    if (current)
      return cls.body_expr;
  }
  return begin_of(cls.body);
}

} // namespace cvc::state_exec::detail

#endif // CVC_STATE_EXEC_CLOSURE_BODY_H
