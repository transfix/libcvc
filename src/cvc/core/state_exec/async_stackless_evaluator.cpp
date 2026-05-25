/*
  Copyright 2026 The University of Texas at Austin

  async_stackless_evaluator — coroutine wrapper around the synchronous
  stackless evaluator.  Each step() / run() is a coroutine that yields
  between evaluation steps, letting the scheduler interleave processes.
*/

#include <chrono>
#include <cvc/state_exec/async_stackless_evaluator.h>
#include <cvc/state_exec/parser.h>

namespace cvc::state_exec {

// ---------------------------------------------------------------------------
// construction
// ---------------------------------------------------------------------------

async_stackless_evaluator::async_stackless_evaluator(environment_ptr global_env)
    : inner_(std::move(global_env)) {}

// ---------------------------------------------------------------------------
// state creation (forwarded)
// ---------------------------------------------------------------------------

evaluator_state async_stackless_evaluator::create_state(const value_t &expr, environment_ptr env) {
  return inner_.create_state(expr, env);
}

evaluator_state async_stackless_evaluator::create_state(const std::string &script,
                                                        environment_ptr env) {
  return inner_.create_state(script, env);
}

// ---------------------------------------------------------------------------
// step — single step as a coroutine
// ---------------------------------------------------------------------------

task<bool> async_stackless_evaluator::step(evaluator_state &state) {
  bool done = inner_.step(state);
  co_await suspend_point{};
  co_return done;
}

// ---------------------------------------------------------------------------
// run — run until done/limit, yielding between steps
// ---------------------------------------------------------------------------

task<value_t> async_stackless_evaluator::run(evaluator_state &state,
                                             std::optional<uint64_t> max_steps,
                                             std::optional<double> timeout_sec,
                                             std::function<void(const value_t &)> on_complete) {
  auto start = timeout_sec ? std::optional{std::chrono::steady_clock::now()} : std::nullopt;

  state.stats.start();
  try {
    while (!state.done) {
      inner_.step(state);
      co_await suspend_point{};

      if (max_steps && state.stats.get_step_count() >= *max_steps)
        break;
      if (start) {
        auto elapsed = std::chrono::steady_clock::now() - *start;
        if (std::chrono::duration<double>(elapsed).count() > *timeout_sec) {
          state.stats.mark_complete();
          throw evaluation_timeout("evaluation exceeded timeout");
        }
      }
    }
    if (state.done) {
      state.stats.mark_complete();
      if (on_complete)
        on_complete(state.result);
    }
    co_return state.done ? state.result : nil_value;
  } catch (...) {
    state.stats.mark_complete();
    throw;
  }
}

// ---------------------------------------------------------------------------
// synchronous convenience wrappers
// ---------------------------------------------------------------------------

value_t async_stackless_evaluator::sync_evaluate(const value_t &expr, environment_ptr env,
                                                 std::optional<double> timeout_sec,
                                                 std::function<void(const value_t &)> on_complete) {
  auto st = create_state(expr, env);
  auto t = run(st, std::nullopt, timeout_sec, std::move(on_complete));
  return t.sync_wait();
}

value_t
async_stackless_evaluator::sync_evaluate_script(const std::string &script, environment_ptr env,
                                                std::optional<double> timeout_sec,
                                                std::function<void(const value_t &)> on_complete) {
  auto st = create_state(script, env);
  auto t = run(st, std::nullopt, timeout_sec, std::move(on_complete));
  return t.sync_wait();
}

// ---------------------------------------------------------------------------
// interrupt / pause / resume  (forwarded)
// ---------------------------------------------------------------------------

void async_stackless_evaluator::interrupt() { inner_.interrupt(); }
void async_stackless_evaluator::reset_interrupt() { inner_.reset_interrupt(); }
void async_stackless_evaluator::pause() { inner_.pause(); }
void async_stackless_evaluator::resume() { inner_.resume(); }
bool async_stackless_evaluator::is_paused() const { return inner_.is_paused(); }

} // namespace cvc::state_exec
