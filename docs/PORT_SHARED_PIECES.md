# The ports' shared pieces (P0b)

What every VolumeRover library port reuses, built in phase P0b of
[VOLROVER-LIBRARY-PORTS-ROADMAP.md](roadmap/VOLROVER-LIBRARY-PORTS-ROADMAP.md) (§1.4–§1.9, §7).
A port declares its params struct's table and registers its stepper; everything here does the
rest.

## Parameter tables (`cvc/core/param_table.h`)

A params struct is declared once, as a table of its fields:

```cpp
struct blur_params {
  double sigma = 1.5;
  int passes = 2;
  edge_mode edges = edge_mode::clamp; // BOOST_DESCRIBE_ENUM(edge_mode, clamp, wrap, zero)
  std::vector<double> weights{0.25, 0.5, 0.25};
};

template <> struct cvc::param_table<blur_params> : cvc::param_fields<blur_params> {
  using P = blur_params;
  static constexpr auto fields = std::tuple{
      field("sigma", &P::sigma, unit("voxel"), range(0.0, 64.0), doc("the Gaussian's sigma")),
      field("passes", &P::passes, range(1, 16), doc("how many times it runs")),
      field("edges", &P::edges, doc("what lies past the edge")), // choices: its enumerators
      field("weights", &P::weights, range(0.0, 1.0), doc("the kernel's taps")),
  };
  // Optional: checks across fields, appended to params_validate()'s list.
  static void check(const P &p, std::vector<std::string> &problems);
};
```

A field is a bool, an integer, `float`, `double`, `std::string`, an enum described with
`BOOST_DESCRIBE_ENUM`, or a `std::vector` of one of those. Its name is its state key under the
node; dots make nested keys (`smooth.iterations`). The name with dots as underscores should be its
member's name, which is what pycvc's keywords set.

From the table:

| What | Where | Does |
|---|---|---|
| `params_validate(p)` | `param_table.h` | A list of what is wrong: out of range, NaN, no such choice, then the table's `check`. Empty when nothing is |
| `params_to_state(p, node)` | `param_table.h` | One key per field under `node`, with its properties |
| `params_from_state<P>(node)` | `param_table.h` | Reads the keys back. A missing key is seeded with its default (and its properties). A value that does not parse is an error, not replaced by the default. The result is validated, so `errors` lists all a caller must report |
| `params_describe_state<P>(node)` | `param_table.h` | Writes the properties only |
| `param_field_infos<P>()`, `params_markdown<P>()` | `param_table.h` | The table as data, and as a Markdown table for a port's docs |
| `add_param_options<P>(desc)`, `params_from_options<P>(vm)` | `param_cli.h` | The CLI flags (`--smooth-iterations`; a bool flag alone is true) with their help, for a `cvc` subcommand on boost::program_options. Legacy aliases are added by hand |
| `params_from_value<P>(v)`, `params_to_dict(p)` | `param_dsl.h` | The dict an intrinsic takes as its parameters (nil: the defaults; an unknown key names the valid ones) |
| `register_params<P>(app, name)`, `register_param_intrinsics(env, app)` | `param_dsl.h` | The app's registry of tables, behind `(params.defaults name)` and `(params.check name dict)` |
| `PYCVC_PARAM_TABLE(pyname, P)` | `bindings/pycvc/pycvc_params.i` | pycvc keywords (an unknown one raises `TypeError` naming the valid ones), `fields()`, `from_dict` / `to_dict`, `validate()`, `from_state` / `to_state` on a `pycvc.State`, `set_text` / `get_text`, `markdown()` |

**Values as text** (state keys and CLI flags): numbers in their shortest round-trip form, so a
struct read back from the tree is the same bit for bit; bools as `1` / `0` (`true`, `yes`, `on`
and their opposites are read too); enums by name; lists as JSON arrays (`[1, 2.5]`,
`["a", "b"]`), with a DSL-printed `(1 2.5)` and bare `1,2.5` read too.

**Properties on each key** follow `cvc/state/state_key_meta.h`: the comment and `doc`, `unit`,
`min`, `max`, `choices` (comma-separated), `default`, `type` (`bool`, `int`, `number`, `string`,
`enum` or `list`) and `access` = `build` (read when a job starts; a change applies to the next
run). A component or a generic inspector labels and bounds a widget from them.

**No process-wide state.** The table is constexpr data. The registry behind `params.*` lives in
the app's data map, so it is created and destroyed with the app, and another app never sees it.

The first real table is `compute_contour_tree`'s options (`cvc/volume/contour_tree_params.h`),
which pycvc wraps as `pycvc.contour_tree_options(max_dim=64, quantize_u8=True)`.

## Jobs: a stepper that owns its result (`cvc/core/result_job.h`)

A port's routine is a resumable stepper beside its blocking function (§1.9):

- **Required:** `bool step(std::size_t budget)` (budget 0 runs to the end) and
  `double progress() const`.
- **Optional:** `phase()`, `status()`, `done()`, a thread-safe `cancel()`, and `take()`.

`make_result_job(stepper)`, or `make_result_job<R>(stepper, taker)`, wraps it as a
`cvc::result_job<R>`. That is a `cvc::sliced_job`, so `schedule_sliced_job` runs it.

| Member | Does |
|---|---|
| `step(budget)` | Steps the stepper. Once it is done, takes the result and destroys the stepper, so its working memory goes back at once |
| `cancel()` | From any thread. The next step ends the job with nothing and destroys the stepper |
| `phase()`, `progress()`, `status()` | The stepper's (`"running"` for one with no phase), then `"done"`, `"cancelled"` or `"failed"`. Progress is clamped to [0, 1], never falls, and is 1 once done |
| `failed()`, `error()`, `exception()` | A step that threw: the job ends with the failure in `status()`, as `sliced_job` reports failures |
| `result()`, `take_result()` | The result once done. Otherwise they throw, and after a failure they rethrow the stepper's own exception |
| `work_ms()`, `max_step_ms()`, `steps()` | Step timings |

`cvc/volume/volume_jobs.h` has the first two: `contour_tree_job(vol, opt)` and
`resize_job(vol, dim)`.

## The node keys (`cvc/core/job_keys.h`)

`cvc::job_keys` publishes, under a base path, what fe_lab publishes for its jobs:

- `busy`, `stoppable`, `status`;
- `progress` (0–100), `phase` (`"sorting 42%"`, or `""` when idle);
- `stats.work_ms`;
- `pid`, for the DSL's `-start` forms.

`schedule_published_job(app, job, opt, keys, status, on_result)` submits a job with the keys
published from its hooks:
- `started` now;
- progress from `on_progress`;
- `finished` from `on_done`, or from `on_cancelled` (a new `sliced_task_options` hook for a job
  stopped through its own `cancel()`).

`on_result` takes a result from a job that did not fail. fe_lab writes its keys through
`job_keys` now. It keeps its own `stats.work_ms`, because that times a chain of commands.

`stop_sliced_job(app, pid)` (`sliced_task.h`) asks a task's job to stop. It touches no scheduler,
so another process's step may call it.

## DSL intrinsics (`cvc/core/job_intrinsics.h`)

A port registers each operation once, from its stepper factory and its parameter table:

```cpp
cvc::job_op op = cvc::make_job_op<seeded_segmentation_params, segmentation_result>(
    "segment.seeded", 2,
    [](const cvc::job_call &c, const seeded_segmentation_params &p) {
      return segment_seeded_job(*c.input_as<cvc::volume>(0), *c.input_as<point_classes>(1), p);
    },
    [](segmentation_result &&r) { return to_dict(std::move(r)); });
cvc::register_job_op(env, ictx, app, op);
cvc::register_job_intrinsics(env, app); // job.cancel, params.defaults, params.check
```

That gives the three forms of §1.6, plus `job.cancel`:

| Form | Does |
|---|---|
| `(segment.seeded vol classes [params])` | Waits for the result. The calling process parks on the task (an async function whose work runs by itself, STATE_EXEC_EXCEPTIONS.md 3.12), and the job's exception raises at the call |
| `(segment.seeded-async ...)` | Returns a future |
| `(segment.seeded-start ... target)` | Returns the pid at once. The node keys are published under `target` (below the calling program's root), and the result goes to `target.result` on the data channel. This is the form for Ariadne actions: it runs under the default `action_max_time` (0.1 s) and `action_max_steps` (5000) |
| `(job.cancel pid)` | Stops the task at its next step. A `-start` target then reads `status` `cancelled` |

- **Inputs:** each is a data object, or a state path whose data channel holds one.
- **Parameters:** a dict, read through the table. Every problem raises `ValueError`.
- **Tasks:** they carry the caller's owner tag, so closing a document stops what it started.
  Killing the waiting process, or the task's own process, cancels the job.
- **Hosts:** every host should get the ops through the app's provider list, which P0a builds
  (§1.3). Until then, a host registers them itself; the tests use Ariadne's
  `register_action_intrinsics`.

## The T7 harness (`src/cvc/tests/resumable_harness.h`)

A port's T7 test names its stepper, a taker, a bit-for-bit comparison and, optionally, its
blocking function. Then `cvc::test::run_t7` checks:

1. Budgets 1, 64, a seeded random budget per step, and 0 all give the blocking result, bit for
   bit.
2. Progress stays in [0, 1] and never falls, and every phase is named.
3. A cancel in each phase leaves no result, and the stepper is destroyed at the next step.
4. It records the slowest step at the default budget (4096), and stepping's overhead over
   budget 0.
5. It runs the job as a cooperative sliced task, frame by frame, as single-threaded wasm and
   `CVC_SLICED_THREADING=cooperative` run it. The frames keep coming, the slowest frame is
   recorded, and the result is the same.

Timing bounds are off unless the port sets them, because sanitizer and Debug builds are many times
slower.

The proofs (`result_job_test`) run it on:
- `contour_tree_builder`, on noise and on resampled, quantized blobs;
- `resize_stepper`;
- `fem::stepped_solve`'s Poisson solve, where libigl is built in.

## pycvc (`bindings/pycvc/pycvc_jobs.i`)

`PYCVC_RESULT_JOB(pyname, R)` wraps `result_job<R>` once, generically. It has `step`, `done`,
`cancel`, `cancelled`, `progress`, `phase`, `status`, `failed`, `error`, `has_result`, `result`,
`take_result`, the timings, and `run(budget, progress)`. The GIL is released inside each step.

The first two wrapped are:
- `pycvc.contour_tree_job(vol, opts)`, which returns `result_job_contour_tree`;
- `pycvc.resize_job(vol, nx, ny, nz)`, which returns `result_job_volume`.

pycvc now also wraps `pycvc.compute_contour_tree` and the tree itself.

## Tests under node

`src/cvc/tests/port_tests.cmake` lists the ports' googletest binaries (`CVC_PORT_TESTS`). They
build with the rest of the suite, and a wasm-mt build with `-DCVC_BUILD_PORT_TESTS=ON
-DCVC_BUILD_TESTS=OFF` builds only them. ci.yml's `ports (wasm-mt, node)` lane does that and runs
each one under the emsdk's node (T14). googletest comes from source there: the catalog has no
wasm-mt googletest. A port adds its tests to that list.
