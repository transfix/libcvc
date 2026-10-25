/// @file state_exec_exceptions_test.cpp
/// @brief DSL exceptions (docs/STATE_EXEC_EXCEPTIONS.md): raising and catching (EX), the
/// evaluator's, builtins' and natives' errors as exceptions (UN), and what a program can't catch
/// (KL), and leaving the program (CX), in both evaluators and on both schedulers. Case IDs are the
/// spec's (section 6).

#include <cvc/core/app.h>
#include <cvc/state/state.h>
#include <cvc/state/state_exec/async_scheduler.h>
#include <cvc/state/state_exec/builtins.h>
#include <cvc/state/state_exec/evaluator.h>
#include <cvc/state/state_exec/exceptions.h>
#include <cvc/state/state_exec/parser.h>
#include <cvc/state/state_exec/scheduler.h>
#include <cvc/state/state_exec/stackless_evaluator.h>
#include <cvc/state/state_exec/state_value_codec.h>
#include <gtest/gtest.h>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

using namespace cvc::state_exec;

namespace exceptions_test {
// A C++ exception type of the test's own, for UN-9: a native error becomes NativeError, its
// native_type naming this.
struct custom_failure : std::runtime_error {
  using std::runtime_error::runtime_error;
};
} // namespace exceptions_test

namespace {

enum engine : unsigned {
  recursive = 1,  // evaluator
  stackless = 2,  // stackless_evaluator
  sync_sched = 4, // scheduler
  async_sched = 8 // async_scheduler
};
constexpr unsigned kAll = recursive | stackless | sync_sched | async_sched;
constexpr unsigned kStackless = stackless | sync_sched | async_sched; // forms recursive lacks

const char *engine_name(unsigned e) {
  switch (e) {
  case recursive:
    return "recursive";
  case stackless:
    return "stackless";
  case sync_sched:
    return "scheduler";
  default:
    return "async_scheduler";
  }
}

// A value as text: a string bare, anything else as to_string() prints it.
std::string show(const value_t &v) {
  if (const auto *s = std::get_if<std::string>(&v.v))
    return *s;
  return to_string(v);
}

// What a program came to: its value, or the exception nothing caught.
struct outcome {
  bool raised = false;
  std::string value;   // the program's value, shown
  std::string cls;     // the uncaught exception's class
  std::string message; // its message
};

// Record the exception a program ended with in `o`.
void record(outcome &o, const value_t &exc) {
  o.raised = true;
  o.cls = class_name(class_of_instance(exc));
  o.message = exception_message(exc);
}

// What a scheduler's process came to: its result, or the exception it failed with.
template <class Scheduler> void finish(outcome &o, const Scheduler &sched, int pid) {
  if (const auto exc = sched.get_exit_exception(pid)) {
    EXPECT_EQ(sched.get_process_info(pid)->status, process_status::failed);
    EXPECT_EQ(sched.get_exit_error(pid).value_or(""), exception_summary(*exc));
    record(o, *exc);
    return;
  }
  const auto r = sched.get_result(pid);
  o.value = r ? show(*r) : "<no result>";
}

// The default environment plus natives that fail the ways a C++ function can.
environment_ptr test_env() {
  auto env = builtins::make_default_environment();
  env->set("throw-invalid", value_t{native_fn{[](std::span<const value_t>) -> value_t {
             throw std::invalid_argument("bad argument");
           }}});
  env->set("throw-custom", value_t{native_fn{[](std::span<const value_t>) -> value_t {
             throw exceptions_test::custom_failure("custom");
           }}});
  env->set("parse-src", value_t{native_fn{[](std::span<const value_t> a) -> value_t {
             return parse(std::get<std::string>(a[0].v));
           }}});
  return env;
}

// Run `script` on engine `e`. An exception that reaches the C++ caller, or that a process failed
// with, is recorded as the DSL exception it is.
outcome run(unsigned e, const std::string &script) {
  outcome o;
  try {
    switch (e) {
    case recursive: {
      evaluator ev(test_env());
      o.value = show(ev.evaluate_script(script));
      break;
    }
    case stackless: {
      stackless_evaluator ev(test_env());
      o.value = show(ev.evaluate_script(script));
      break;
    }
    case sync_sched: {
      scheduler sched;
      execute_options opts;
      opts.env = test_env();
      const int pid = sched.execute(script, opts);
      sched.run(); // a program's error ends its process failed; run() doesn't throw it (CX-2)
      finish(o, sched, pid);
      break;
    }
    default: {
      async_scheduler sched;
      execute_options opts;
      opts.env = test_env();
      const int pid = sched.execute(script, opts);
      sched.sync_run();
      finish(o, sched, pid);
      break;
    }
    }
  } catch (const std::exception &ex) {
    record(o, exception_from(ex));
  }
  return o;
}

struct program_case {
  const char *id;
  const char *script;
  const char *value;             // the program's value; null when it raises
  const char *cls = nullptr;     // the class of the exception it raises
  const char *message = nullptr; // and its message (null: not checked)
  unsigned engines = kAll;
};

void expect_case(const program_case &c) {
  for (unsigned e : {recursive, stackless, sync_sched, async_sched}) {
    if (!(c.engines & e))
      continue;
    SCOPED_TRACE(std::string(c.id) + " on " + engine_name(e) + ": " + c.script);
    const outcome o = run(e, c.script);
    if (c.value) {
      EXPECT_FALSE(o.raised) << o.cls << ": " << o.message;
      EXPECT_EQ(o.value, c.value);
      continue;
    }
    ASSERT_TRUE(o.raised) << "value " << o.value;
    EXPECT_EQ(o.cls, c.cls) << o.message;
    if (c.message)
      EXPECT_EQ(o.message, c.message);
  }
}

} // namespace

// ===========================================================================
// 6.1 Raising and catching (EX)
// ===========================================================================

TEST(ExceptionsRaiseCatch, SpecCases) {
  // EX-6 and EX-11 record their effects in a list and a dict: `set` binds in the scope it runs
  // in, and a handler and a function body each have their own.
  const program_case cases[] = {
      {"EX-1", "(try (error \"boom\") (catch (e) (get-attr e \"message\")))", "boom"},
      {"EX-2", "(try (raise (ValueError \"v\")) (catch (TypeError e) 1) (catch (ValueError e) 2))",
       "2"},
      {"EX-3",
       "(begin (defclass MyError ValueError) (try (raise (MyError \"m\")) (catch (ValueError e) "
       "(get-attr (get-attr e \"__class__\") \"__name__\"))))",
       "MyError"},
      {"EX-4", "(try (raise (ValueError \"v\")) (catch (TypeError e) 1))", nullptr, "ValueError",
       "v"},
      {"EX-5", "(try 7 (catch (e) 1))", "7"},
      {"EX-6",
       "(begin (set log (list)) (try (error \"x\") (catch (e) (append log \"c\")) (finally "
       "(append log \"f\"))) log)",
       "(\"c\" \"f\")"},
      {"EX-7", "(begin (set n 0) (try (try (error \"in\") (finally (set n 1))) (catch (e) n)))",
       "1"},
      {"EX-8", "(try (try (error \"a\") (catch (e) (raise))) (catch (e) (get-attr e \"message\")))",
       "a"},
      {"EX-9",
       "(try (try (error \"a\") (catch (e) (error \"b\"))) (catch (e) (get-attr (get-attr e "
       "\"cause\") \"message\")))",
       "a"},
      {"EX-10", "(try (error \"a\") (finally (error \"b\")))", nullptr, "RuntimeError", "b"},
      {"EX-11",
       "(begin (set box (dict \"g\" 0)) (defun f () (try (return 1) (finally (set-attr box \"g\" "
       "2)))) (list (f) (get-attr box \"g\")))",
       "(1 2)"},
      {"EX-12", "(begin (set g 0) (while #t (try (break 5) (finally (set g 9)))) g)", "9", nullptr,
       nullptr, kStackless}, // the recursive evaluator has no break
      {"EX-13", "(raise 5)", nullptr, "TypeError"},
      {"EX-14",
       "(try (raise (Exception \"m\" (dict \"k\" 1))) (catch (e) (get-attr (get-attr e \"data\") "
       "\"k\")))",
       "1"},
      {"EX-15a", "(if (isinstance (ValueError \"v\") Exception) \"yes\" \"no\")", "yes"},
      {"EX-15b", "(if (isinstance 5 Exception) \"yes\" \"no\")", "no"},
      {"EX-16", "(try (error \"x\") (catch (NoSuchClass e) 1))", nullptr, "NameError",
       "catch: not a class: NoSuchClass"},
      {"EX-17", "(try (error \"x\"))", nullptr, "TypeError",
       "try: expected a catch or finally clause"},
      {"EX-18", "(raise)", nullptr, "RuntimeError", "raise: no exception is being handled"},
      {"EX-19", "(begin (try (error \"x\") (catch (caught_here) 1)) caught_here)", nullptr,
       "NameError"},
  };
  for (const auto &c : cases)
    expect_case(c);
}

TEST(ExceptionsRaiseCatch, MoreForms) {
  const program_case cases[] = {
      // A user class with fields of its own, raised and caught as its parent (spec 2.2).
      {"user-init",
       "(begin (defclass VolumeError RuntimeError (init (self message volume) (super self init "
       "message) (set-attr self \"volume\" volume) self)) (try (raise (VolumeError \"bad\" 7)) "
       "(catch (RuntimeError e) (list (get-attr e \"message\") (get-attr e \"volume\")))))",
       "(\"bad\" 7)"},
      {"type-only", "(try (raise (ValueError \"v\")) (catch (TypeError) 1) (catch (ValueError) 2))",
       "2"},
      {"catch-all",
       "(try (raise (KeyError \"k\")) (catch (err) (get-attr (get-attr err \"__class__\") "
       "\"__name__\")))",
       "KeyError"},
      {"empty-spec", "(try (error \"x\") (catch () 3))", "3"},
      {"error-data", "(try (error \"m\" 5) (catch (e) (get-attr e \"data\")))", "5"},
      {"origin-dsl", "(try (error \"m\") (catch (e) (get-attr e \"origin\")))", "dsl"},
      {"from-callee",
       "(begin (defun f () (error \"inner\")) (try (f) (catch (e) (get-attr e \"message\"))))",
       "inner"},
      {"finally-normal", "(begin (set g 0) (list (try 5 (finally (set g 1))) g))", "(5 1)"},
      {"finally-dropped", "(try 5 (finally 9))", "5"},
      {"return-in-handler",
       "(begin (set box (dict \"g\" 0)) (defun f () (try (error \"x\") (catch (e) (return 3)) "
       "(finally (set-attr box \"g\" 4)))) (list (f) (get-attr box \"g\")))",
       "(3 4)"},
      {"no-match-runs-finally",
       "(begin (set log (list)) (try (try (raise (KeyError \"k\")) (catch (TypeError e) 1) "
       "(finally (append log \"f\"))) (catch (KeyError e) (append log \"k\"))) log)",
       "(\"f\" \"k\")"},
      {"handler-raises-finally-runs",
       "(begin (set log (list)) (try (try (error \"a\") (catch (e) (error \"b\")) (finally (append "
       "log \"f\"))) (catch (e) (append log (get-attr e \"message\")))) log)",
       "(\"f\" \"b\")"},
      // Re-raising the exception being handled doesn't make it its own cause.
      {"reraise-no-cycle",
       "(try (try (error \"a\") (catch (e) (raise e))) (catch (e) (if (get-attr e \"cause\") "
       "\"cause\" \"none\")))",
       "none"},
      {"isinstance-user-class",
       "(begin (defclass A) (defclass B A) (list (if (isinstance (B) A) 1 0) (if (isinstance (A) "
       "B) 1 0)))",
       "(1 0)"},
      {"class-name", "(begin (defclass Thing) (get-attr Thing.__cls_dict__ \"__name__\"))",
       "Thing"},
      {"raise-arity", "(raise 1 2)", nullptr, "TypeError"},
      {"bad-catch", "(try 1 (catch x 2))", nullptr, "TypeError"},
      {"finally-not-last", "(try 1 (finally 2) (catch (e) 3))", nullptr, "TypeError",
       "try: finally must be the last clause"},
      // A return leaves only the function it is in (it used to unwind the stackless stack).
      {"return-boundary", "(begin (defun f () (return 5)) (+ (f) 1))", "6"},
  };
  for (const auto &c : cases)
    expect_case(c);
}

TEST(ExceptionsRaiseCatch, TracebackListsFramesInnermostLast) {
  // EX-20, and the cap of 64 frames.
  for (unsigned e : {recursive, stackless}) {
    SCOPED_TRACE(engine_name(e));
    const outcome o = run(e, "(try (error \"x\") (catch (e) (get-attr e \"traceback\")))");
    ASSERT_FALSE(o.raised) << o.message;
    ASSERT_FALSE(o.value.empty());
    EXPECT_EQ(o.value.front(), '(') << o.value;
    EXPECT_NE(o.value.find("error"), std::string::npos) << o.value;

    stackless_evaluator s(test_env());
    evaluator r(test_env());
    const char *deep = "(begin (defun f (n) (if (= n 0) (error \"deep\") (+ 1 (f (- n 1))))) "
                       "(try (f 100) (catch (e) (get-attr e \"traceback\"))))";
    const value_t lines = e == recursive ? r.evaluate_script(deep) : s.evaluate_script(deep);
    const auto &list = *std::get<list_ptr>(lines.v);
    EXPECT_EQ(list.size(), kTracebackFrames);
    for (const auto &line : list)
      ASSERT_TRUE(std::holds_alternative<std::string>(line.v));
    EXPECT_NE(std::get<std::string>(list.back().v).find("error"), std::string::npos)
        << "innermost last: " << std::get<std::string>(list.back().v);
  }
}

// ===========================================================================
// 6.2 Errors from the evaluator, builtins and natives (UN)
// ===========================================================================

TEST(ExceptionsFromErrors, SpecCases) {
  const program_case cases[] = {
      {"UN-1", "(try undefined-name (catch (NameError e) (get-attr e \"origin\")))", "builtin"},
      {"UN-2",
       "(begin (set f (lambda (x) x)) (try (f) (catch (TypeError e) (get-attr e \"message\"))))",
       "closure: expected 1 args, got 0"},
      {"UN-2b", "(try ((lambda (x) x)) (catch (TypeError e) \"type\"))", "type"},
      {"UN-3", "(try (5 1) (catch (TypeError e) \"type\"))", "type"},
      {"UN-4", "(try (int \"abc\") (catch (ValueError e) \"value\"))", "value"},
      {"UN-5", "(try (nth (list 1) 5) (catch (IndexError e) \"index\"))", "index"},
      {"UN-6", "(try (/ 1 0) (catch (ArithmeticError e) \"arith\"))", "arith"},
      {"UN-8",
       "(try (throw-invalid) (catch (ValueError e) (list (get-attr e \"origin\") (get-attr e "
       "\"function\") (get-attr e \"message\"))))",
       "(\"native\" \"throw-invalid\" \"bad argument\")"},
      {"UN-9", "(try (throw-custom) (catch (NativeError e) (get-attr e \"native_type\")))",
       "exceptions_test::custom_failure"},
      {"UN-10", "(try (collections.range 1000000000) (catch (MemoryError e) \"memory\"))",
       "memory"},
      {"UN-11",
       "(try (apply (lambda (x) (error \"in\")) (list 1)) (catch (e) (get-attr e "
       "\"message\")))",
       "in", nullptr, nullptr, kStackless}, // the recursive evaluator's natives call no lambdas
      {"UN-13",
       "(begin (set g (generator (lambda () (begin (yield 1) (error \"gen\") (yield 2))))) (list "
       "(next g) (try (next g) (catch (e) (get-attr e \"message\"))) (if (gen-done? g) \"done\" "
       "\"live\")))",
       "(1 \"gen\" \"done\")"},
      {"UN-14",
       "(begin (defclass A (boom (self) (error \"a-boom\"))) (defclass B A (boom (self) (super "
       "self boom))) (list (try (send (A) \"boom\") (catch (e) (get-attr e \"message\"))) (try "
       "(send (B) \"boom\") (catch (e) (get-attr e \"message\")))))",
       "(\"a-boom\" \"a-boom\")"},
      {"UN-16",
       "(begin (defmacro m (x) (error x)) (try (m \"mac\") (catch (e) (get-attr e \"message\"))))",
       "mac"},
  };
  for (const auto &c : cases)
    expect_case(c);

  // UN-7: the position is the parser's; that it is there is what matters here.
  for (unsigned e : {recursive, stackless, sync_sched}) {
    SCOPED_TRACE(std::string("UN-7 on ") + engine_name(e));
    const outcome o = run(e, "(try (parse-src \"(+ 1\") (catch (ParseError e) (list (get-attr e "
                             "\"line\") (get-attr e \"col\"))))");
    ASSERT_FALSE(o.raised) << o.cls << ": " << o.message;
    ASSERT_FALSE(o.value.empty());
    EXPECT_EQ(o.value.front(), '(') << o.value;
    EXPECT_EQ(o.value.find("nil"), std::string::npos) << o.value;
  }
}

TEST(ExceptionsFromErrors, SlicedJobIsAbandoned) {
  // UN-12: a lambda raising inside collections.map while the work runs a slice per step.
  for (const char *slice : {"1", "3", "0"}) {
    SCOPED_TRACE(std::string("step_nodes ") + slice);
    cvc::app app;
    cvc::state &root = cvc::state::instance(app);
    root("state_exec.defaults.data.step_nodes").value(std::string(slice));
    scheduler sched;
    sched.set_watch_root(&root);
    sched.load_settings();
    const int pid = sched.execute(std::string(
        "(begin (set r (try (collections.map (collections.range 50) (lambda (x) (if (= x 20) "
        "(error \"twenty\") x))) (catch (e) (get-attr e \"message\")))) (list r "
        "(collections.map (list 1 2) (lambda (x) (* x 10)))))"));
    sched.run();
    const auto r = sched.get_result(pid);
    ASSERT_TRUE(r);
    EXPECT_EQ(show(*r), "(\"twenty\" (10 20))");
  }
}

TEST(ExceptionsFromErrors, SignalHandlerRaisesInTheProgram) {
  // UN-15: the handler's exception is raised where it interrupted the program, inside its try.
  const std::string program = "(try (begin (set i 0) (while (< i 100000) (set i (+ i 1))) "
                              "\"done\") (catch (e) (get-attr e \"message\")))";
  {
    scheduler sched;
    execute_options opts;
    opts.signal_handlers["SIGUSR1"] = parse("(error \"sig\")");
    const int pid = sched.execute(program, opts);
    sched.run(40); // into the loop
    ASSERT_TRUE(sched.send_signal(pid, "SIGUSR1"));
    sched.run();
    const auto r = sched.get_result(pid);
    ASSERT_TRUE(r);
    EXPECT_EQ(show(*r), "sig");
  }
  {
    async_scheduler sched;
    execute_options opts;
    opts.signal_handlers["SIGUSR1"] = parse("(error \"sig\")");
    const int pid = sched.execute(program, opts);
    sched.sync_run(40);
    ASSERT_TRUE(sched.send_signal(pid, "SIGUSR1"));
    sched.sync_run();
    EXPECT_FALSE(sched.get_exit_error(pid)) << *sched.get_exit_error(pid);
    const auto r = sched.get_result(pid);
    ASSERT_TRUE(r);
    EXPECT_EQ(show(*r), "sig");
  }
  {
    // Raised where the program has no try, it ends the program as any error does.
    async_scheduler sched;
    execute_options opts;
    opts.signal_handlers["SIGUSR1"] = parse("(error \"sig\")");
    const int pid = sched.execute(std::string("(begin (set i 0) (while (< i 100000) (set i (+ i "
                                              "1))) \"done\")"),
                                  opts);
    sched.sync_run(40);
    ASSERT_TRUE(sched.send_signal(pid, "SIGUSR1"));
    sched.sync_run();
    EXPECT_EQ(sched.get_process_info(pid)->status, process_status::failed);
    EXPECT_EQ(sched.get_exit_error(pid).value_or(""), "RuntimeError: sig");
  }
}

TEST(ExceptionsFromErrors, UncaughtReachesTheCallerAsDslException) {
  // CX-3 and CX-4: the caller gets a dsl_exception with the value and the class chain. One made
  // from a C++ exception keeps it as original(), through a finally and a re-raise too.
  stackless_evaluator s(test_env());
  evaluator r(test_env());
  const auto original_of = [](const auto &evaluate) -> std::exception_ptr {
    try {
      evaluate();
    } catch (const dsl_exception &ex) {
      return ex.original();
    }
    return nullptr;
  };
  for (unsigned e : {recursive, stackless}) {
    SCOPED_TRACE(engine_name(e));
    const auto eval = [&](const char *script) {
      return [&, script] {
        if (e == recursive)
          r.evaluate_script(script);
        else
          s.evaluate_script(script);
      };
    };
    for (const char *script : {"(throw-invalid)", "(try (throw-invalid) (catch (e) (raise)))",
                               "(try (throw-invalid) (finally 1))"}) {
      SCOPED_TRACE(script);
      const std::exception_ptr original = original_of(eval(script));
      ASSERT_TRUE(original);
      EXPECT_THROW(std::rethrow_exception(original), std::invalid_argument);
    }
    const std::exception_ptr builtin = original_of(eval("(nth (list 1) 5)"));
    ASSERT_TRUE(builtin);
    EXPECT_THROW(std::rethrow_exception(builtin), dsl_error);
    EXPECT_FALSE(original_of(eval("(error \"raised\")"))); // a DSL raise has none

    try {
      eval("(begin (defclass MyError ValueError) (raise (MyError \"m\")))")();
      ADD_FAILURE() << "no exception";
    } catch (const dsl_exception &ex) {
      EXPECT_STREQ(ex.what(), "MyError: m");
      EXPECT_EQ(ex.class_name(), "MyError");
      EXPECT_EQ(ex.class_chain(), (std::vector<std::string>{"MyError", "ValueError", "Exception"}));
      EXPECT_EQ(exception_message(ex.value()), "m");
    }
  }
}

TEST(ExceptionsLeaving, SchedulersEndTheProcessFailed) {
  // CX-1 and CX-2: the process ends failed with its exception, and run() goes on (the sync
  // scheduler used to throw it). raise_exit_exception() throws it for a host that wants that.
  const char *program = "(begin (defclass MyError ValueError) (raise (MyError \"bad\" 7)))";
  const auto check = [](const auto &sched, int pid) {
    const auto info = sched.get_process_info(pid);
    ASSERT_TRUE(info);
    EXPECT_EQ(info->status, process_status::failed);
    ASSERT_TRUE(info->exit_exception);
    EXPECT_EQ(class_name(class_of_instance(*info->exit_exception)), "MyError");
    EXPECT_EQ(show(*field(*info->exit_exception, "data")), "7");
    EXPECT_EQ(info->exit_error.value_or(""), "MyError: bad");
    EXPECT_FALSE(sched.get_result(pid));
    try {
      sched.raise_exit_exception(pid);
      ADD_FAILURE() << "no exception";
    } catch (const dsl_exception &ex) {
      EXPECT_EQ(ex.class_chain(), (std::vector<std::string>{"MyError", "ValueError", "Exception"}));
    }
  };
  {
    scheduler sched;
    const int pid = sched.execute(std::string(program));
    const int good = sched.execute(std::string("(+ 1 2)"));
    EXPECT_NO_THROW(sched.run());
    check(sched, pid);
    EXPECT_EQ(show(*sched.get_result(good)), "3");
    EXPECT_NO_THROW(sched.raise_exit_exception(good)); // not failed: nothing to throw
  }
  {
    async_scheduler sched;
    const int pid = sched.execute(std::string(program));
    sched.sync_run();
    check(sched, pid);
  }
}

TEST(ExceptionsLeaving, FailedAndKilledAreCountedApart) {
  // CX-7
  for (int which : {0, 1}) {
    SCOPED_TRACE(which ? "async_scheduler" : "scheduler");
    // One process that fails, one killed at its step limit, one that terminates.
    const auto submit = [](auto &sched) {
      execute_options limited;
      limited.max_steps = 50;
      sched.execute(std::string("(error \"one\")"));
      sched.execute(std::string("(while t 1)"), limited);
      sched.execute(std::string("(+ 1 1)"));
    };
    std::vector<process_info> procs;
    scheduler_stats stats;
    if (which) {
      async_scheduler sched;
      submit(sched);
      sched.sync_run();
      procs = sched.list_processes();
      stats = sched.get_stats();
    } else {
      scheduler sched;
      submit(sched);
      sched.run();
      procs = sched.list_processes();
      stats = sched.get_stats();
    }
    EXPECT_EQ(stats.failed, 1);
    EXPECT_EQ(stats.killed, 1);
    EXPECT_EQ(stats.terminated, 1);
    ASSERT_EQ(procs.size(), 3u);
    EXPECT_EQ(procs[0].status, process_status::failed);
    EXPECT_EQ(procs[1].status, process_status::killed);
    EXPECT_EQ(procs[2].status, process_status::terminated);
    EXPECT_STREQ(to_string(process_status::failed), "failed");
    EXPECT_EQ(parse_process_status("failed"), process_status::failed);
  }
}

TEST(ExceptionsLeaving, AnExceptionFromBeforeASnapshotKeepsItsClass) {
  // A snapshot writes a class dict once for each value holding it, so a restored exception's
  // class is a copy: catch matches it by its chain of names. A handler is running when the
  // snapshot is taken; after the restore it re-raises its exception to an outer catch by class.
  cvc::app app;
  stackless_evaluator ev(builtins::make_default_environment());
  auto st = ev.create_state(parse(
      "(begin (defclass MyError ValueError) (try (try (raise (MyError \"m\")) (catch (e) (begin "
      "(set i 0) (while (< i 20) (set i (+ i 1))) (raise e)))) (catch (ValueError e2) (get-attr "
      "(get-attr e2 \"__class__\") \"__name__\"))))"));
  ev.run(st, 40);
  ASSERT_FALSE(st.done);
  bool handling = false;
  for (const auto &fr : st.stack)
    handling = handling || fr.phase == eval_phase::try_handler;
  ASSERT_TRUE(handling);

  cvc::state &node = cvc::state::instance(app)("__exceptions_class_snapshot__");
  encode_evaluator_state(node, st);
  evaluator_state restored = decode_evaluator_state(node);
  const auto builtins_env = builtins::make_default_environment();
  for (auto &fr : restored.stack) {
    environment_ptr env = fr.env;
    while (env && env->outer)
      env = env->outer;
    if (env)
      for (auto &[name, val] : builtins_env->bindings)
        env->set(name, val);
  }
  EXPECT_EQ(show(ev.run(restored)), "MyError");
}

TEST(ExceptionsFromErrors, StateRaisesAgainWhenSteppedAgain) {
  stackless_evaluator ev(test_env());
  auto st = ev.create_state(parse("(error \"once\")"));
  EXPECT_THROW(ev.run(st), dsl_exception);
  EXPECT_TRUE(st.done);
  ASSERT_TRUE(st.uncaught);
  EXPECT_THROW(ev.step(st), dsl_exception);
}

TEST(ExceptionsFromErrors, RaiseApiRaisesWhereTheStateIs) {
  stackless_evaluator ev(test_env());
  auto st = ev.create_state(parse("(try (begin (set i 0) (while (< i 1000) (set i (+ i 1))) "
                                  "\"done\") (catch (KeyError e) (get-attr e \"message\")))"));
  ev.run(st, 30);
  ASSERT_FALSE(st.done);
  ev.raise(st, make_exception(exception_class("KeyError"), "from C++"));
  EXPECT_EQ(show(ev.run(st)), "from C++");
  EXPECT_THROW(ev.raise(st, value_t{int64_t{5}}), std::invalid_argument);
}

TEST(ExceptionsFromErrors, SnapshotInsideTryResumesTheTry) {
  // The try frame's phase and slots survive the codec: the restored try catches what its body
  // raises after the restore (CX-5).
  cvc::app app;
  stackless_evaluator ev(builtins::make_default_environment());
  auto st = ev.create_state(parse("(try (begin (set i 0) (while (< i 5) (set i (+ i 1))) (error "
                                  "\"late\")) (catch (e) (get-attr e \"message\")) (finally (set "
                                  "i 99)))"));
  ev.run(st, 8);
  ASSERT_FALSE(st.done);
  bool in_try = false;
  for (const auto &f : st.stack)
    in_try = in_try || f.phase == eval_phase::try_body;
  ASSERT_TRUE(in_try);

  cvc::state &node = cvc::state::instance(app)("__exceptions_snapshot__");
  encode_evaluator_state(node, st);
  evaluator_state restored = decode_evaluator_state(node);
  // Natives aren't serialized: bind the builtins again at the root of every frame's scope.
  const auto builtins_env = builtins::make_default_environment();
  for (auto &f : restored.stack) {
    environment_ptr env = f.env;
    while (env && env->outer)
      env = env->outer;
    if (env)
      for (auto &[name, val] : builtins_env->bindings)
        env->set(name, val);
  }
  EXPECT_EQ(show(ev.run(restored)), "late");
}

// ===========================================================================
// 6.3 Not catchable (KL)
// ===========================================================================

TEST(ExceptionsNotCatchable, StepLimitKills) {
  // KL-1
  execute_options opts;
  opts.max_steps = 100;
  const std::string program = "(try (while t 1) (catch (e) \"caught\"))";
  {
    scheduler sched;
    const int pid = sched.execute(program, opts);
    sched.run();
    EXPECT_EQ(sched.get_process_info(pid)->status, process_status::killed);
    EXPECT_FALSE(sched.get_result(pid));
  }
  {
    async_scheduler sched;
    const int pid = sched.execute(program, opts);
    sched.sync_run();
    EXPECT_EQ(sched.get_process_info(pid)->status, process_status::killed);
    EXPECT_NE(sched.get_exit_error(pid).value_or("").find("step"), std::string::npos)
        << sched.get_exit_error(pid).value_or("");
    EXPECT_FALSE(sched.get_result(pid));
  }
}

TEST(ExceptionsNotCatchable, TimeLimitKills) {
  // KL-2
  execute_options opts;
  opts.max_time = 0.05;
  const std::string program = "(try (while t 1) (catch (e) \"caught\"))";
  {
    scheduler sched;
    const int pid = sched.execute(program, opts);
    sched.run();
    EXPECT_EQ(sched.get_process_info(pid)->status, process_status::killed);
    EXPECT_FALSE(sched.get_result(pid));
  }
  {
    async_scheduler sched;
    const int pid = sched.execute(program, opts);
    sched.sync_run();
    EXPECT_EQ(sched.get_process_info(pid)->status, process_status::killed);
    EXPECT_FALSE(sched.get_result(pid));
  }
}

TEST(ExceptionsNotCatchable, KillSkipsFinally) {
  // KL-3
  auto env = builtins::make_default_environment();
  auto box = make_dict({{"cleaned", value_t{std::string("no")}}});
  env->set("box", box);
  execute_options opts;
  opts.env = env;
  scheduler sched;
  const int pid = sched.execute(std::string("(try (while t 1) (finally (set-attr box \"cleaned\" "
                                            "\"yes\")))"),
                                opts);
  sched.run(50);
  ASSERT_TRUE(sched.kill(pid));
  sched.run();
  EXPECT_EQ(sched.get_process_info(pid)->status, process_status::killed);
  EXPECT_EQ(show(*field(box, "cleaned")), "no");
}

TEST(ExceptionsNotCatchable, MemoryLimitKills) {
  // KL-4: over max_memory the process is killed, and no catch runs. A call the per-call limit
  // refuses first is a MemoryError, which a program may catch (spec 2.5).
  execute_options opts;
  opts.max_memory = 256 * 1024;
  scheduler sched;
  const int pid = sched.execute(
      std::string("(try (begin (set keep (list)) (set i 0) (while (< i 60) (begin (append keep "
                  "(collections.range 200)) (set i (+ i 1)))) (length keep)) (catch (MemoryError "
                  "e) \"refused\") (catch (e) \"caught\"))"),
      opts);
  sched.run();
  const auto info = sched.get_process_info(pid);
  ASSERT_TRUE(info);
  const auto r = sched.get_result(pid);
  if (info->status == process_status::killed)
    EXPECT_FALSE(r);
  else
    EXPECT_EQ(r ? show(*r) : "", "refused");
}

TEST(ExceptionsNotCatchable, TimeoutAndInterruptReachTheCaller) {
  // KL-5
  {
    stackless_evaluator s(builtins::make_default_environment());
    evaluator r(builtins::make_default_environment());
    const value_t loop = parse("(try (while t 1) (catch (e) \"caught\") (finally \"cleanup\"))");
    EXPECT_THROW(s.evaluate(loop, nullptr, 0.05), evaluation_timeout);
    EXPECT_THROW(r.evaluate(loop, nullptr, 0.05), evaluation_timeout);
  }
  {
    auto env = builtins::make_default_environment();
    stackless_evaluator s(env);
    env->set("stop", value_t{native_fn{[&s](std::span<const value_t>) -> value_t {
               s.interrupt();
               return nil_value;
             }}});
    EXPECT_THROW(s.evaluate_script("(try (begin (stop) (while t 1)) (catch (e) \"caught\"))"),
                 evaluation_interrupted);
  }
  {
    auto env = builtins::make_default_environment();
    evaluator r(env);
    env->set("stop", value_t{native_fn{[&r](std::span<const value_t>) -> value_t {
               r.interrupt();
               return nil_value;
             }}});
    EXPECT_THROW(r.evaluate_script("(try (begin (stop) (while t 1)) (catch (e) \"caught\"))"),
                 evaluation_interrupted);
  }
}
