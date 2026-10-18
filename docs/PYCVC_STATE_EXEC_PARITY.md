# pycvc ↔ C++ API parity for state_exec

Goal: pycvc's state_exec API is the C++ API, under the same names, so Python code reads like the
C++ it drives. pycvc already wraps libcvc this way: `pycvc.i` wraps the real types and adds no
facade layer. This doc records the audit of `inc/cvc/state/state_exec/*.h` against pycvc, the
approach agreed for closing the gap, and the plan.

## Approach

1. **Real types and functions, C++ names.** SWIG wraps the actual declarations:
   `cvc::state_exec::parse` is `pycvc.parse`, `symbol` is `pycvc.symbol`, and so on. Members
   that can't cross are listed below as exceptions, with the reason.
2. **Tree arguments are `State`s.** Where C++ takes a `cvc::state&`, Python passes a
   `pycvc.State` (`pycvc.State.instance(app)` is the root, `root.child("a.b")` a node). The
   older `state_set(app, …)` functions stay as a convenience layer.
3. **Values convert to native Python types**, with two small types for what Python lacks
   (`bindings/pycvc/pycvc_value.h`):

   | `value_t` | Python |
   |---|---|
   | nil, bool, int64, double | `None`, `bool`, `int`, `float` |
   | string | `str`. The text must be valid UTF-8, or `UnicodeDecodeError` is raised |
   | `bytes_value` | `bytes`; `bytes` or `bytearray` coming in |
   | list | `list`; a `list` or `tuple` coming in |
   | dict | `dict`; keys must be `str` coming in |
   | symbol | `pycvc.symbol`: the real struct, with `.name`, equality and hashing |
   | closure, native_fn, data_object, generator | `pycvc.value`: the real `value_t` as an opaque handle that converts back to the same value |

   Conversion is strict:
   - an int outside 64 bits raises `OverflowError`;
   - a non-`str` dict key or any other Python type raises `TypeError`;
   - a Python value that contains itself raises `ValueError`, since only the evaluator's own
     containers are tracked for cycle collection.

   It takes any depth: it walks with an explicit stack, not recursion (PR 4), as the parser and
   the evaluator do. A list or dict shared within a value converts to one object, both ways, so
   sharing survives the trip, and a DSL list that holds itself comes out as a Python list that
   holds itself. (PR 1's converter recursed and stopped at 1000 levels.)

   Before this, symbols became plain strings, closures became the text `"<closure>"`, and any
   object became its `str()`. The old converter could also pass a null object into a list when a
   string wasn't valid UTF-8, and it left an `OverflowError` pending on a big int.
4. **Errors keep their type.** `parse_error` is `pycvc.parse_error`, a `RuntimeError` subclass
   with `.line` and `.col`. Other C++ exceptions stay `RuntimeError`.
5. **`Exec` stays** as the Python-integration layer on top of `async_scheduler`. It handles the
   GIL around Python callbacks, an asyncio loop for coroutines, and compute-pool offload. It uses
   the same value conversion as everything else. `Exec.run` raises `pycvc.parse_error` for a
   syntax error whether the script was parsed at submit or read inside the process.
   `Exec.scheduler()` returns its `async_scheduler`, which keeps the `Exec` alive.
6. **Enums are `enum.IntEnum` classes** with the C++ names and values:
   `pycvc.process_status.killed` is `process_status::killed`. An enum argument must be a member
   of its enum, so a plain int stays a value: `to_string(process_status)` and `to_string(value_t)`
   keep their shared C++ name.
7. **Empty results.** Where C++ returns an empty `std::optional` or a null pointer, Python gets
   `None`, except for a value: `None` is nil, so `get_result`, `pop_pending_message` and
   `environment::lookup` raise `KeyError(pid / path / name)` instead.
8. **Callables cross as Python callables.** Where C++ takes a `native_fn` or a
   `std::function` (`builtins::register_fn`, the `on_complete` of `execute_options`, `process`
   and the evaluators), Python passes a callable. It runs on the thread stepping the scheduler
   or evaluator and takes the GIL for its call. A Python exception becomes the C++ error a
   throwing callback would be. An `on_complete` member reads back as the callable that was set.
9. **The GIL is released while programs run:** `scheduler.run`/`step`,
   `async_scheduler.sync_run`/`sync_step`, `stackless_evaluator.step`/`run`/`evaluate`/
   `evaluate_script` and `evaluator.evaluate`/`evaluate_script`. Other Python threads keep going,
   and can call an evaluator's thread-safe `interrupt`, `pause` and `resume`. The schedulers'
   thread safety is the C++ one: while a scheduler runs, another thread may call only
   `async_scheduler.post_message`.
10. **References keep their owner alive.** A member or call that hands out a reference into its
    object (`evaluator_state.stats`, `evaluator.stats()`, `process.state`, `Exec.scheduler()`)
    returns a Python object that holds the owner. Every other reference return, such as a
    struct's list or map member, comes back as a copy.
11. **Intrinsics keep their context alive.** C++'s `register_intrinsics(env, ctx)` requires the
    caller to keep `ctx` valid for as long as `env` is used. In Python, every function it adds
    holds the context, and the context holds the scheduler and channel policy assigned to it.
    The context's `root` is a tree node, which lives as long as its app, so keep the app alive
    as in C++.

## Inventory

The audit (2026-10-17) found 97 public items in the 20 headers. Counted individually, with the 38
`builtin_*` functions and the 7 `MIME_*` constants, it is 140. Items added to master since then
are listed too. Status after PR 4:

| Header | Item | Python | Status |
|---|---|---|---|
| parser.h | `parse_options` | `pycvc.parse_options` | wrapped (#653) |
| | `parse_options_from_state(state&, id, base)` | `pycvc.parse_options_from_state(State, …)` | wrapped (#653) |
| | `parse`, `parse_all` | `pycvc.parse`, `pycvc.parse_all` | wrapped (PR 1) |
| | `parse_error` (line, col) | `pycvc.parse_error` | wrapped (PR 1) |
| | `script_reader` | `pycvc.script_reader` | wrapped (PR 1). `forms()`, `source()` and `options()` return copies |
| | `parser` (`read`, `parse_expr`) | none | exception: it holds a `string_view` of its input, which Python can't keep alive. `script_reader` owns its source and gives the same slice-by-slice reading |
| data_job.h (#658) | `data_options`, `data_options_from_state(state&, id, base)` | `pycvc.data_options`, `pycvc.data_options_from_state(State, …)` | wrapped (#658) |
| | `data_job` (`step`, `result`, `nodes`), `make_equal_job`, `make_print_job`, `make_concat_job`, `make_copy_job`, `make_substitute_job`, `run_to_completion`, `run_data_job`, `call_function` (with or without #683's `then`), `make_generator_job` (#683; the generator is a `pycvc.value`), `is_compound`, `count_data_nodes`, `kDataNodeBytes`, `kMaxNativeNesting`, `kNativeStackReserve` (#682) (#658, #665) | the same names under `pycvc.` | wrapped (PR 4). A factory hands Python an owned job, which `run_data_job` takes over. `who` (C++ keeps the pointer for its errors) is kept for good. `run_data_job`'s `then` is a Python callable. From a Python function a program calls, `run_data_job` and `call_function` work as for a C++ native: return what they return at once |
| | the call protocol (`data_job::call_request`, `take_call`, `pending_call`, `call_returned`, `own_inputs`), `detail::deferred_data_job`, `detail::data_step_scope`, `detail::native_nesting` (#682) | none | exception: for C++ job implementations and the evaluator's own bookkeeping; Python can't subclass a data job |
| types.h | `symbol` | `pycvc.symbol` | wrapped (PR 1) |
| | `value_t` | native types, or `pycvc.value` | wrapped (PR 1), see the table above |
| | `to_string`, `values_equal` | `pycvc.to_string`, `pycvc.values_equal` | wrapped (PR 1) |
| | `make_list`, `make_dict`, `make_bytes`, `nil_value`, `true_value`, `false_value` | `list`, `dict`, `bytes`, `None`, `True`, `False` | exception: the native Python values are the equivalents |
| | `environment` (`bindings`, `outer`, `frozen`, `lookup`, `set`, `set_existing`, `extend`) | `pycvc.environment` | wrapped (PR 2), shared like `environment_ptr`. `bindings` reads and writes as a dict (a copy); `lookup` raises `KeyError` where C++ returns nullptr; `set` on a frozen scope (#660) raises `RuntimeError` |
| | `closure`, `data_object`, `generator` | `pycvc.value` (opaque) | wrapped as opaque handles; no member access |
| | `time_budget`, `eval_deadline_guard`, `eval_deadline_expired`, `eval_deadline_check`, `thread_cpu_seconds` | none | exception: thread-local evaluator internals |
| | `evaluation_interrupted`, `evaluation_timeout` (in types.h since 2026-10) | listed with evaluator.h below | |
| | `kDefaultMaxAllocBytes`, `eval_alloc_limit`, `eval_alloc_check`, `alloc_bytes` (#660) | the same names under `pycvc.` | wrapped (PR 3). A Python function a program calls runs on the stepping thread, where the step's limit is armed, so it can check the limit as a builtin does |
| | `eval_alloc_guard` (#660) | none | exception: RAII that must nest last-in first-out, which a Python object's lifetime doesn't promise |
| | `detail::release_nested`, `detail::drop_owner` (#657) | none | exception: internals of `value_t`'s destructor |
| state_value_codec.h | `encode_value`, `decode_value` | `pycvc.encode_value(State, v)`, `pycvc.decode_value(State)` | wrapped (PR 1) |
| | `encode_environment`/`decode_environment`, `encode_evaluator_state`/`decode_evaluator_state` | `pycvc.encode_environment(State, env)`, ... | wrapped (PR 3) |
| scheduler_base.h | `execute_options`, `process_info`, `scheduler_stats` | `pycvc.execute_options`, `pycvc.process_info`, `pycvc.scheduler_stats` | wrapped (PR 2). `on_complete` is a Python callable, `signal_handlers` a dict, the `uint64_t` fields Python ints. `process_info` includes #658's steps and times by kind (`read_steps`, `eval_steps`, `data_steps`, `data_nodes`, `read_time`, `eval_time`, `data_time`) |
| | `fill_budget_usage` (#658) | none | exception: how the schedulers fill `process_info`, from the evaluator's stats |
| | `scheduling_policy` | `pycvc.scheduling_policy` | wrapped (PR 2), an `IntEnum` |
| | `scheduler_base` | `pycvc.scheduler_base` | wrapped (PR 3) as the common base of the two classes (what `intrinsics_context.sched` holds). Its members are on the concrete classes |
| scheduler.h | `scheduler` (about 45 members) | `pycvc.scheduler` | wrapped (PR 2). `run`/`step` release the GIL. `execute(str)` is the script overload, so a string can't be passed as a `value_t` expression (write it as the script `"\"text\""`). `id()`, `parser_settings()` and `data_settings()` (#658) return copies. `current_process()` (shares the process), `queue_watch_event()` and `max_alloc_bytes()` (#660) are wrapped in PR 3. Not wrapped: `mem_tracker()` (see memory_tracker.h) |
| | `max_alloc_bytes_from_state(state&, id, base)` (#660) | `pycvc.max_alloc_bytes_from_state(State, …)` | wrapped (PR 3) |
| async_scheduler.h | `async_scheduler` (about 55 members) | `pycvc.async_scheduler` | wrapped (PR 2), with the same notes as `scheduler`; `sync_run`/`sync_step` release the GIL, and `parser_defaults()` returns a copy. #658's `set_data_defaults`/`data_defaults`/`data_settings`/`host_data_defaults` sit beside the parser ones. Coroutine `step()`/`run()` are an exception: Python gets `sync_step()`/`sync_run()` |
| process.h | `process_status`, `to_string(process_status)`, `parse_process_status` | `pycvc.process_status` (an `IntEnum`), `pycvc.to_string`, `pycvc.parse_process_status` | wrapped (PR 2) |
| | `process`, `process::watch_event`, `process::watch_handler_entry`, `make_process` | `pycvc.process` (shared, as `process_ptr`), `pycvc.watch_event`, `pycvc.watch_handler_entry` (also `process.watch_event`, ...), `pycvc.make_process()` | wrapped (PR 3). `state` is a reference into the process; `inbox` reads as a list. The `steady_clock` time points (`create_time`, `last_run_start`, `wake_time`) are an exception: `elapsed_time()` and `accumulated_time` carry the same information |
| builtins.h | `builtins::make_default_environment`, `builtins::register_fn` | `pycvc.builtins.make_default_environment()`, `pycvc.builtins.register_fn(env, name, fn)` | wrapped (PR 2). `fn` is a Python callable, or a `pycvc.value` holding a native fn |
| | 38 `builtin_*` functions | none | exception: DSL builtins, called from DSL source |
| | `percent_encode`, `percent_decode` | `pycvc.percent_encode`, `pycvc.percent_decode` | wrapped (PR 2) |
| stackless_evaluator.h, evaluator.h | `stackless_evaluator`, `evaluator`, `evaluator_state`, `eval_frame`, `eval_phase`, `evaluation_stats`, `step_kind` (#658), `evaluation_interrupted`, `evaluation_timeout` | the same names under `pycvc.` | wrapped (PR 3). `eval_phase` and `step_kind` are `IntEnum`s, and `evaluation_stats` has #658's counts by kind (`steps`, `step_time`, `data_nodes`, ...); the two exceptions are `RuntimeError` subclasses. `on_complete` is a Python callable. `evaluator.user_macros()` returns a copy, a dict of name -> (params, body). `restrict_special_forms` takes any iterable of str, or None |
| | `evaluator_state.data_work`, `evaluator_state.backlog`, `pending_data_work` (#658), `waits_on_data_work` | none | exception: deep-data work in flight between steps, internal to the evaluator; `data_step_nodes` is wrapped |
| | `return_exception` | none | exception: the internal unwinding of `(return ...)`, never seen outside an evaluation |
| async_evaluator.h, async_stackless_evaluator.h | the async evaluators | none | exception: coroutine API |
| resource_policy.h | `resource_policy`, `process_limits`, `validate_limits` | `pycvc.resource_policy` (its `mode` is `pycvc.resource_policy.mode`, an `IntEnum`), `pycvc.process_limits`, `pycvc.validate_limits` | wrapped (PR 3) |
| intrinsics.h | `register_intrinsics`, `intrinsics_context`, `apply_chroot`, `channel_policy`, `resolve_channel_key`, `make_future`, `future_channel_of` | the same names under `pycvc.` | wrapped (PR 3). See approach item 11 for the context's lifetime. `channel_policy.global` is `global_` (a Python keyword). The context's `tracker`, `document` and `watches` aren't wrapped: `memory_tracker` is internal, `document_scope`'s slots are C++ templates, and `watches` is the intrinsics' own registry |
| | `document_scope`, `park_on_channel` | none | exception: Ariadne/runtime internals. `document_scope` is typed C++ template slots; `park_on_channel` takes raw process pointers and is valid only inside a running native function |
| stdlib.h | `stdlib_registry` (`standard`, `shared_scope`, `register_function`, `list_functions`, `list_modules`, `import_module`, `lookup_qualified`) | `pycvc.stdlib_registry` | wrapped (PR 4). `standard()` returns a copy, so Python can't add to the shared registry; `register_function` takes a Python callable or a `pycvc.value` native; `lookup_qualified` raises `KeyError` where C++ returns nullptr |
| heap.h (#667) | `heap_options`, `heap_options_from_state`, `measure_heap_share`, `make_heap_share_job`, `heap_remeasure_bytes`, `cycle_stats`, `collect_cycles`, `cycle_collection_due`, `note_process_exit`, `tracked_objects` | the same names under `pycvc.` | wrapped (PR 4). The collector is per thread: run it on the thread that ran the programs. `measure_heap_share` and `collect_cycles` release the GIL |
| | `heap_after_step`, `track_captured_env`, `track_mutated` | none | exception: the schedulers' per-step accounting, and tracking that only means something for the evaluator's own containers (a value from Python is a fresh copy) |
| | #667's fields: `process.heap_*`, `heap_measure_due`, `release_state`; `process_info.heap_memory`/`heap_measurements`; `evaluator_state.heap_measured`; `stackless_evaluator::global_env()`; `eval_alloc_note` | the same names | wrapped (PR 4). `heap_measured` is read-only (the result of a measurement in flight) |
| | `eval_alloc_guard::allocated()` (#667) | none | exception: the guard isn't wrapped (see types.h) |
| exec_coordinator.h | `exec_coordinator` and its result types | none | deferred: it needs the cluster shard, membership and message-bus objects, which pycvc doesn't wrap |
| memory_tracker.h | `memory_tracker` | none | exception: per-node byte accounting, internal to the schedulers |
| generator.h, task.h | `generator`, `generator_next`, `task<>` | none | exception: coroutine and evaluator internals; a generator crosses as a `pycvc.value` |
| utf8.h | `decode`, `count`, `byte_offset`, `encode`, `is_valid` | none | exception: Python's `str` already counts and indexes code points |

## Plan

Each PR is stacked on the previous one and includes pytest checks against the C++ behaviour.

1. **PR 1 (#661):** values, parsing and the value codec.
   - `pycvc_value.h`: the strict shared converter, used by `Exec` too.
   - `pycvc.symbol` and `pycvc.value`.
   - `parse`, `parse_all`, `parse_error` with position, `script_reader`, `to_string`,
     `values_equal`, `encode_value`, `decode_value`.
   - `bindings/pycvc/test_pycvc_state_exec.py`.
   - The pycvc README's outdated Status and Design sections are rewritten.
2. **PR 2 (#671):** `environment` and `builtins`; `execute_options`, `process_info`,
   `scheduler_stats` and the enums; `scheduler` and `async_scheduler` as real classes, with their
   non-coroutine methods. `Exec.scheduler()` returns its scheduler.
   `bindings/pycvc/test_pycvc_state_exec_sched.py` repeats the C++ scheduler tests on both
   classes.
3. **PR 3 (this):** the evaluators, `evaluator_state`, `eval_frame` and `evaluation_stats`; the
   `process` struct and the scheduler calls that take it; the environment and evaluator-state
   codec; `resource_policy`; `register_intrinsics` and `intrinsics_context`, so an environment
   built in Python can have the intrinsics `Exec.run` programs have; and master's
   `max_alloc_bytes` additions (#660). `bindings/pycvc/test_pycvc_state_exec_eval.py` repeats the
   C++ tests.
4. **PR 4 (this):** `stdlib_registry`, the deep-data jobs (#658, #665) and the heap (#667); and the value conversion without a depth limit (it walks with an explicit stack, as the parser and evaluator do, and keeps shared structure). `bindings/pycvc/test_pycvc_state_exec_data.py` covers them.

Known gaps:
- Invalid UTF-8 in a DSL string raises when it crosses to Python, but no Python test produces such
  a string. Python can't create one, and the stdlib's `bytes.decode` isn't reachable from DSL code
  yet.
- `Exec.run` finds a syntax error in a script read inside its process by parsing it again after
  the process is killed. That costs one extra parse, and only on failure.
