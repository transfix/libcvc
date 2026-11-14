// closure_body.h — a closure's body as the one expression a call evaluates (private to the
// evaluators and builtins).
#ifndef CVC_STATE_EXEC_CLOSURE_BODY_H
#define CVC_STATE_EXEC_CLOSURE_BODY_H

#include <cstdint>
#include <cstring>
#include <cvc/state/state_exec/types.h>
#include <memory>
#include <type_traits>
#include <variant>
#include <vector>

namespace cvc::state_exec::detail {

/// Whether two numbers are the same bits: 0.0 and -0.0 are different forms (they evaluate to
/// different numbers), and a NaN is the same form as itself.
inline bool same_bits(double x, double y) noexcept {
  std::uint64_t a = 0, b = 0;
  std::memcpy(&a, &x, sizeof a);
  std::memcpy(&b, &y, sizeof b);
  return a == b;
}

/// Whether `a` is `b` as a body form: the same object for a list (or anything else held by
/// pointer), an atom that evaluates to the same value otherwise (a number of the same bits, an
/// equal string or symbol). (A native function can't be compared: never.)
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
                           std::is_same_v<T, closure_ptr> || std::is_same_v<T, data_object_ptr> ||
                           std::is_same_v<T, generator_ptr>)
          return x.get() == y.get();
        else if constexpr (std::is_same_v<T, double>)
          return same_bits(x, y); // not ==, which takes -0.0 for 0.0
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

/// Whether `expr` is (begin forms...), its forms the same as `forms` (same_form).
inline bool is_begin_of(const std::vector<value_t> &expr, const std::vector<value_t> &forms) {
  if (expr.size() != forms.size() + 1)
    return false;
  const auto *head = std::get_if<symbol>(&expr[0].v);
  if (!head || head->name != "begin")
    return false;
  for (std::size_t i = 0; i < forms.size(); ++i)
    if (!same_form(expr[i + 1], forms[i]))
      return false;
  return true;
}

/// The (begin ...) bodies shared by closures (shared_body_expr): one slot per lambda form, by its
/// address, holding (weakly) the (begin ...) the last closure it made shares. A slot whose form
/// was freed and its address reused is harmless: what a slot holds is used only while it is
/// exactly (begin body...) for the new closure's body, each form the same object or an atom that
/// evaluates the same (same_form: a number bit for bit, so -0.0 is not 0.0). Each cycle registry
/// keeps its own (a group's, or a thread's own registry), bounded (64 slots, each a weak
/// reference: the list's block stays allocated while the slot holds it, about 40 bytes, until the
/// slot is reused), so two groups taking turns on one thread never share a list (td 4.7: every
/// node a group's collection walks is the group's own), and nothing is shared between apps.
struct body_slots {
  static constexpr std::size_t kSlots = 64;
  struct slot {
    const void *form = nullptr;
    std::weak_ptr<std::vector<value_t>> expr;
  };
  slot slots[kSlots];
};

/// The slots of `ctx`'s cycle registry (heap.h), or null when it has none.
body_slots *body_slots_of(exec_context &ctx) noexcept;

/// The (begin ...) for `cls.body_expr`, when the stackless evaluator running in `ctx` makes a
/// closure of several body forms from the lambda form `form` (a defun's, a lambda's or a class
/// method's list): one shared by the closures that form makes, kept in the slots of `ctx`'s
/// registry while one of them lives (so a loop that makes closures adds no list per closure),
/// else a new one. Null for one form.
list_ptr shared_body_expr(exec_context &ctx, const void *form, const std::vector<value_t> &body);

/// The expression a call of `cls` evaluates: its one form, or (begin body...), the one made with
/// the closure while it still matches the body (else a new one).
inline value_t body_expr_of(const closure &cls) {
  if (cls.body.size() == 1)
    return cls.body[0];
  if (cls.body_expr && is_begin_of(*cls.body_expr, cls.body))
    return value_t(cls.body_expr);
  return begin_of(cls.body);
}

} // namespace cvc::state_exec::detail

#endif // CVC_STATE_EXEC_CLOSURE_BODY_H
