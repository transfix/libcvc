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

## Tests under node

`src/cvc/tests/port_tests.cmake` lists the ports' googletest binaries (`CVC_PORT_TESTS`). They
build with the rest of the suite, and a wasm-mt build with `-DCVC_BUILD_PORT_TESTS=ON
-DCVC_BUILD_TESTS=OFF` builds only them. ci.yml's `ports (wasm-mt, node)` lane does that and runs
each one under the emsdk's node (T14). googletest comes from source there: the catalog has no
wasm-mt googletest. A port adds its tests to that list.
