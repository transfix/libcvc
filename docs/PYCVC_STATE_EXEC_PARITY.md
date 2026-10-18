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
   - a value nested deeper than 1000 levels, or one that contains itself, raises `ValueError`.

   Before this, symbols became plain strings, closures became the text `"<closure>"`, and any
   object became its `str()`. The old converter could also pass a null object into a list when a
   string wasn't valid UTF-8, and it left an `OverflowError` pending on a big int.
4. **Errors keep their type.** `parse_error` is `pycvc.parse_error`, a `RuntimeError` subclass
   with `.line` and `.col`. Other C++ exceptions stay `RuntimeError`.
5. **`Exec` stays** as the Python-integration layer on top of `async_scheduler`. It handles the
   GIL around Python callbacks, an asyncio loop for coroutines, and compute-pool offload. It uses
   the same value conversion as everything else. `Exec.run` raises `pycvc.parse_error` for a
   syntax error whether the script was parsed at submit or read inside the process.

## Inventory

The audit (2026-10-17) found 97 public items in the 20 headers. Counted individually, with the 38
`builtin_*` functions and the 7 `MIME_*` constants, it is 140. Status after PR 1:

| Header | Item | Python | Status |
|---|---|---|---|
| parser.h | `parse_options` | `pycvc.parse_options` | wrapped (#653) |
| | `parse_options_from_state(state&, id, base)` | `pycvc.parse_options_from_state(State, …)` | wrapped (#653) |
| | `parse`, `parse_all` | `pycvc.parse`, `pycvc.parse_all` | wrapped (PR 1) |
| | `parse_error` (line, col) | `pycvc.parse_error` | wrapped (PR 1) |
| | `script_reader` | `pycvc.script_reader` | wrapped (PR 1). `forms()`, `source()` and `options()` return copies |
| | `parser` (`read`, `parse_expr`) | none | exception: it holds a `string_view` of its input, which Python can't keep alive. `script_reader` owns its source and gives the same slice-by-slice reading |
| types.h | `symbol` | `pycvc.symbol` | wrapped (PR 1) |
| | `value_t` | native types, or `pycvc.value` | wrapped (PR 1), see the table above |
| | `to_string`, `values_equal` | `pycvc.to_string`, `pycvc.values_equal` | wrapped (PR 1) |
| | `make_list`, `make_dict`, `make_bytes`, `nil_value`, `true_value`, `false_value` | `list`, `dict`, `bytes`, `None`, `True`, `False` | exception: the native Python values are the equivalents |
| | `environment` | | PR 2 |
| | `closure`, `data_object`, `generator` | `pycvc.value` (opaque) | wrapped as opaque handles; no member access |
| | `time_budget`, `eval_deadline_guard`, `eval_deadline_expired`, `thread_cpu_seconds` | none | exception: thread-local evaluator internals |
| state_value_codec.h | `encode_value`, `decode_value` | `pycvc.encode_value(State, v)`, `pycvc.decode_value(State)` | wrapped (PR 1) |
| | `encode_environment`/`decode_environment`, `encode_evaluator_state`/`decode_evaluator_state` | | PR 3, with the environment and evaluator types |
| scheduler_base.h | `execute_options`, `process_info`, `scheduler_stats`, `scheduling_policy` | | PR 2 |
| scheduler.h | `scheduler` (about 45 members) | | PR 2 |
| async_scheduler.h | `async_scheduler` (about 55 members) | `pycvc.async_scheduler.host_parser_defaults()` only | PR 2. Coroutine `step()`/`run()` are an exception: Python gets the existing `sync_step()`/`sync_run()` |
| process.h | `process_status`, `to_string(process_status)`, `parse_process_status` | | PR 2 |
| | `process` (about 35 fields), `make_process` | | PR 2, read-only where safe |
| builtins.h | `builtins::make_default_environment`, `builtins::register_fn` | `Exec.register_fn` | PR 2. `register_fn` takes a Python callable |
| | 38 `builtin_*` functions | none | exception: DSL builtins, called from DSL source |
| | `percent_encode`, `percent_decode` | | PR 2 |
| stackless_evaluator.h, evaluator.h | `stackless_evaluator`, `evaluator`, `evaluator_state`, `evaluation_stats`, the evaluation exceptions | | PR 3 |
| async_evaluator.h, async_stackless_evaluator.h | the async evaluators | none | exception: coroutine API |
| resource_policy.h | `resource_policy`, `process_limits`, `validate_limits` | | PR 3 |
| intrinsics.h | `register_intrinsics`, `intrinsics_context`, `apply_chroot`, `channel_policy` | `Exec` uses them internally | PR 3, with the intrinsics context's raw-pointer fields as `State`/scheduler references |
| | `document_scope`, `park_on_channel`, `make_future`, `future_channel_of` | none | exception: Ariadne/runtime internals (templates, raw process pointers) |
| stdlib.h | `stdlib_registry` | | after the stdlib is wired into the DSL (separate work: today only tests import it) |
| exec_coordinator.h | `exec_coordinator` and its result types | none | deferred: it needs the cluster shard, membership and message-bus objects, which pycvc doesn't wrap |
| memory_tracker.h | `memory_tracker` | none | exception: per-node byte accounting, internal to the schedulers |
| generator.h, task.h | `generator`, `generator_next`, `task<>` | none | exception: coroutine and evaluator internals; a generator crosses as a `pycvc.value` |
| utf8.h | `decode`, `count`, `byte_offset`, `encode`, `is_valid` | none | exception: Python's `str` already counts and indexes code points |

## Plan

Each PR is stacked on the previous one and includes pytest checks against the C++ behaviour.

1. **PR 1 (this):** values, parsing and the value codec.
   - `pycvc_value.h`: the strict shared converter, used by `Exec` too.
   - `pycvc.symbol` and `pycvc.value`.
   - `parse`, `parse_all`, `parse_error` with position, `script_reader`, `to_string`,
     `values_equal`, `encode_value`, `decode_value`.
   - `bindings/pycvc/test_pycvc_state_exec.py`.
   - The pycvc README's outdated Status and Design sections are rewritten.
2. **PR 2:** `environment` and `builtins`; `execute_options`, `process_info`, `scheduler_stats` and
   the enums; `scheduler` and `async_scheduler` as real classes, with their non-coroutine
   methods. `Exec` exposes its scheduler.
3. **PR 3:** the evaluators and `evaluator_state`, the environment and evaluator-state codec,
   `resource_policy`, and `register_intrinsics` / `intrinsics_context`.

Known gaps after PR 1:
- Invalid UTF-8 in a DSL string raises when it crosses to Python, but no Python test produces such
  a string. Python can't create one, and the stdlib's `bytes.decode` isn't reachable from DSL code
  yet.
- `Exec.run` finds a syntax error in a script read inside its process by parsing it again after
  the process is killed. That costs one extra parse, and only on failure.
