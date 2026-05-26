/**
 * @file generator.h
 * @brief Lazy sequence (generator) for the state_exec DSL.
 *
 * A generator produces values one at a time, either from a native
 * C++ step function (used by `range`) or from a DSL closure that
 * uses the `yield` special form.  Generators are first-class values
 * in the DSL and can be consumed with `(next gen)`, iterated with
 * `(for x gen body)`, or materialised with `(collect gen)`.
 */
#ifndef CVC_STATE_EXEC_GENERATOR_H
#define CVC_STATE_EXEC_GENERATOR_H

#include <cvc/core/state_exec/stackless_evaluator.h>
#include <cvc/core/state_exec/types.h>
#include <functional>
#include <optional>

namespace cvc::state_exec {

/// A lazy sequence that produces values on demand.
///
/// Two flavours:
///   1. **Native** — `native_step` is set.  Each call returns the next
///      value, or std::nullopt when exhausted.  Used by `range`.
///   2. **DSL closure** — `native_step` is empty.  The generator owns
///      its own evaluator + evaluator_state whose body uses `(yield v)`
///      to produce values.  `next` steps the evaluator until it yields
///      or completes.
struct generator {
  /// Evaluator instance for closure-based generators.
  stackless_evaluator evaluator;

  /// Execution snapshot for closure-based generators.
  evaluator_state state;

  /// True after the generator has produced its last value.
  bool exhausted = false;

  /// Native step function (optional — used by `range` and friends).
  std::function<std::optional<value_t>()> native_step;

  /// Construct a closure-based generator (evaluator takes a default env).
  explicit generator(environment_ptr env)
      : evaluator(std::move(env)) {}
};

/// Advance a generator by one value.
///
/// For native generators, calls `native_step`.  For closure-based
/// generators, steps the evaluator until `state.yielded` or `state.done`.
/// Returns std::nullopt when the generator is exhausted.
inline std::optional<value_t> generator_next(generator &gen) {
  if (gen.exhausted)
    return std::nullopt;

  // Native path (range, etc.)
  if (gen.native_step) {
    auto result = gen.native_step();
    if (!result)
      gen.exhausted = true;
    return result;
  }

  // Closure path — step until yield or done
  gen.state.yielded = false;
  while (!gen.state.done && !gen.state.yielded)
    gen.evaluator.step(gen.state);

  // If the body yielded a value, return it even if the evaluator
  // simultaneously reached done (e.g. yield was the last expression).
  if (gen.state.yielded)
    return gen.state.result;

  // Evaluator finished without yielding — generator is exhausted.
  gen.exhausted = true;
  return std::nullopt;
}

} // namespace cvc::state_exec

#endif // CVC_STATE_EXEC_GENERATOR_H
