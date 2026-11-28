#ifndef CVC_ARIADNE_LOADER_H
#define CVC_ARIADNE_LOADER_H

// Ariadne — the .ari YAML loader (cvc::ariadne). Parses a .ari document into the
// backend-neutral Widget tree (widget.h) that Runtime renders. This is P0
// slice 1: the useful widget subset (menubar/menus, windows, and the common
// leaves — slider_int/slider_float/checkbox/combo/button/text/separator), in the
// syntax the roadmap documents (docs/roadmap/CVCGL-UI-DSL-ROADMAP.md §3.2–3.5):
// the widget TYPE is the map key and its value is the label, e.g.
//
//   menubar:
//     - menu: Sim
//       items:
//         - menu_item: Paused
//           bind: demo.paused
//         - separator
//         - menu_item: Quit
//           on: quit
//   windows:
//     - window: Controls
//       children:
//         - slider_int: Agents
//           bind: demo.agents
//           lo: 1
//           hi: 512
//         - combo: Belief
//           bind: demo.belief
//           options: [shared, grouped, private]
//         - button: Reset
//           on: reset
//
// It builds the SAME Widget tree the programmatic builders (widget.h) produce, so
// a .ari document and a hand-built tree are interchangeable. The loader is lenient: it
// loads what it understands and warns about the rest. The document's structure is
// defined by the .ari schema (src/cvc/ariadne/schema/ari.v1.schema.json, docs/ARIADNE.md
// §14); when this build has the validator, every load checks the document against it and
// reports each problem (an unknown key, a wrong type) as a warning and in
// LoadResult::schema_issues.

#include <cstdint>
#include <cvc/ariadne/data.h> // DataSpec
#include <cvc/ariadne/doc_args.h>
#include <cvc/ariadne/grants.h>
#include <cvc/ariadne/keys.h>
#include <cvc/ariadne/scene.h>
#include <cvc/ariadne/value.h>
#include <cvc/ariadne/widget.h>
#include <functional>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace cvc {
class app;
namespace ariadne {

// Document provenance (roadmap §3.1a). `min_libcvc` is the load GATE: a document
// declares the minimum libcvc it needs, checked FIRST, before the tree is used.
struct Meta {
  std::string name;
  std::string author;
  std::string description;
  std::string version;    // the .ari document's own version (informational)
  std::string min_libcvc; // minimum libcvc semver required to load this document
};

// A custom type the document declares it USES, in the `customs:` block — so a load on
// a system missing that custom either warns (the default) or fails hard (`required:
// true`). `kind` picks which registry checks it: Widget -> register_widget_type,
// Block -> register_ari_block (both checked by the loader at load time), Node ->
// register_scene_node_type (checked by cvc::gl::ariadne::verify_scene_customs at
// realize time, since node types are cvcGL-side).
struct CustomRequirement {
  enum class Kind { Widget, Node, Block };
  Kind kind = Kind::Widget;
  std::string name;
  // Default false: a missing NON-required custom is a WARNING (placeholder / skip).
  // `required: true` makes a missing custom a hard ERROR (fail fast).
  bool required = false;
};

// A channel declared in the document's `channels:` block (§12 channel scoping). `shared` marks a
// channel that a mount may be granted (documentation/intent). `global` puts the channel at app-root
// and adds it to the default-deny GLOBALS allowlist, so a '/'-prefixed reference to it is permitted
// (a chrooted sub-document may only reach an app-root-global the enclosing scope declared). Purely
// declarative — the enforcement layer (the lint pass + the runtime policy, PR-C) consumes these;
// channel RESOLUTION (private-by-prefix + link-hole grants) works with or without a declaration.
struct ChannelDecl {
  std::string name;
  bool shared = false;
  bool global = false;
};

// §12 lint / correctness policy (a document-level `lint:` block — part of a broader lint effort,
// not buried in `channels:`). `channels` decides how an UNDECLARED channel reference is treated
// once the document declares a `channels:` block: Strict = a hard load error, Warn = a warning, Off
// = no check. Default Strict when a `channels:` block is present; a document with NO `channels:`
// block is never channel-linted (existing/undeclared docs keep working). `quiet` suppresses the
// one-time "channel enforcement relaxed" notice emitted under Warn/Off.
struct LintConfig {
  enum class Mode { Strict, Warn, Off };
  Mode channels = Mode::Strict;
  bool quiet = false;
};

// A place in a document where it breaks the .ari schema, and what is wrong there.
struct SchemaIssue {
  // The file it is in (a load's own file, an import:ed library, a load: fragment), or "" for a
  // document given as a string.
  std::string source;
  // When it is in an include: expansion: "include <unit>", and `pointer`, `line` and `column`
  // are the unit's, where the loader could tell (the include site's otherwise).
  std::string context;
  std::string pointer;      // a JSON Pointer to the value: /windows/0/children/2
  int line = 0, column = 0; // 1-based, in `source`; 0 when unknown
  std::string message;      // "unknown key 'bnd'", "'abc' is not a number", ...
  // "<source>:<line>:<column>: <context>: <pointer>: <message>", leaving out what is unknown.
  std::string text() const;
};

// §6.7 the deepest a document's widgets nest: a level for each widget, and one for each include:
// and load: entry, across every unit and mount. A load that goes deeper fails, and says where
// (LoadResult::error, error_source, error_line). The loader parses a level at a time on the
// native stack, so this keeps a load inside the smallest stack a host loads on (wasm's 2 MiB).
constexpr std::size_t kMaxNesting = 256;

// §6.7 the deepest the schema check (§14.2) goes: levels of YAML, the document (or include:
// expansion) itself one. The check goes down a level of YAML at a time on the native stack, so a
// tree deeper than this is not checked: it gets one SchemaIssue, at its first value past the
// limit, that says so. The deepest shipped document is 13 levels of YAML.
constexpr std::size_t kMaxSchemaDepth = 128;

// What validate_string / validate_file found.
struct ValidationResult {
  // False when nothing was checked: this build has no validator (have_jsonschema()), or the
  // YAML did not parse (`error` says why).
  bool checked = false;
  std::string error;               // the YAML could not be read or parsed
  std::vector<SchemaIssue> issues; // where the document breaks the schema, in document order
  bool ok() const { return checked && error.empty() && issues.empty(); }
};

struct LoadResult {
  bool ok = false;                        // false if the load failed (see `error`)
  Widget root;                            // a Group of the document's widgets (empty on failure)
  std::vector<Scene> scenes; // the `scenes:` entries, each with its id (viewports name them)
  Scene scene;                            // the parsed `scene:` block (§9; empty if none)
  Meta meta;                              // parsed provenance (may be empty)
  std::vector<CustomRequirement> customs; // declared `customs:` (widget/node/block)
  std::vector<HostRequirement> requirements; // the top-level `requires:` (empty if none)
  // §12 channel scoping: the document's declared `channels:` (empty if none). `has_channels_block`
  // records whether the block was PRESENT (even if empty) — the enforcement gate: enforcement runs
  // only for a document that declares channels:. `lint` is the doc-level `lint:` policy (strictness
  // of the channel check). The loader CAPTURES these; the enforcement layer (PR-C) consumes them.
  std::vector<ChannelDecl> channels;
  bool has_channels_block = false;
  LintConfig lint;
  // The `init:` block's state_exec script (verbatim text; empty if none). The loader
  // only CAPTURES it (it has no cvc::app and never runs the DSL); the host runs it once
  // at load via cvc::ariadne::run_init, scoped to the document prefix.
  std::string init_script;
  // §5.3 `state:`: literal values the host writes under the document prefix (seed_state) just
  // before init: runs, so init: reads them and may override them. In document order.
  StateSeeds state_seeds;
  // §7.1/§4.6 document-level resident handlers (verbatim scripts; empty if none). Like init_script
  // the loader only CAPTURES these — the host hands each to the matching Runtime setter, which
  // submits ONE long-lived resident process (parking between activations): on_tick_script ->
  // set_tick_program (runs each frame); on_key_script -> set_key_program (runs per keyboard event);
  // on_pointer_script -> set_pointer_program (runs per mouse event). The on_key/on_pointer bodies
  // read the event with (get-attr event "key"/"x"/…). The host feeds events via Runtime::post_input
  // from its input source (e.g. cvc::gl::SdlInput). on_signal_script -> set_signal_program (runs
  // per signal the host routes through Runtime::post_signal — a terminal's Ctrl-C is SIGINT).
  std::string on_tick_script;
  std::string on_key_script;
  std::string on_pointer_script;
  std::string on_signal_script;
  // The `tests:` block (docs/ARIADNE.md "Testing a document"): name -> program, in document
  // order. Only `ari test` runs them, one at a time on the action lane, after init: and the first
  // frame; every other host ignores them.
  std::vector<std::pair<std::string, std::string>> tests;
  // The document's functions (docs/ARIADNE.md §6.6): the ones its imports declare, then its own
  // `functions:` block's (defun ...) forms, a later one of the same name replacing an earlier one.
  // root.functions holds the same as a FunctionScope; a mount's are on its wrapper widget.
  std::vector<FunctionDef> functions;
  // The document's `keys:` block (keys.h): its rebindable key actions, in document order. The host
  // hands them to AppRuntime, which binds them (Runtime::set_key_bindings) and lists them in the
  // Keys window.
  std::vector<KeyBinding> keys;
  // The document's `args:` block (doc_args.h): what it accepts on the command line after its path.
  // `has_args_block` records presence; without the block the runtime checks nothing and the
  // document reads its raw argv. Only the outermost document's block counts.
  std::vector<ArgSpec> args;
  bool has_args_block = false;
  // The `data:` block (data.h): the document's datasets, in document order, each with the
  // document's directory as its base. The host binds and loads them (DataStore::bind).
  std::vector<DataSpec> data;
  // Custom top-level blocks (register_ari_block), keyed by block name — whatever the
  // registered parser stored. Empty unless a document uses a registered custom block.
  std::vector<std::pair<std::string, Value>> extras;
  std::vector<std::string> warnings; // non-fatal issues (unknown types, empty binds, …)
  std::string error;                 // human-readable message when !ok
  // Where `error` is, when the load can say (a document nested deeper than kMaxNesting: its
  // deepest widget): the file ("" for a document given as a string), and the 1-based line and
  // column (0 when unknown). `ari validate` reports the error there.
  std::string error_source;
  int error_line = 0, error_column = 0;
  // Where the document, its import:ed libraries, its load: fragments and its include:
  // expansions break the .ari schema (each also in `warnings`, as "ari: schema: " + text()).
  // Empty when this build has no validator.
  std::vector<SchemaIssue> schema_issues;
  // §12.5 hot-reload: the local FILE paths read during this load — the main document plus every
  // file:// import: and load: fragment (state/http sources are not file-watchable, so omitted).
  // A host watches these for mtime changes and re-loads on change; sources_changed() does the
  // polling. De-duplicated, in first-seen order.
  std::vector<std::string> sources;
  // §12.5 hot-reload: each source's mtime captured AT READ TIME during this load. Seed
  // sources_changed()'s `stamps` with this so a change in the window between the load reading a
  // file and the first poll is detected, not silently adopted as the baseline. Keys match
  // `sources`.
  std::map<std::string, std::int64_t> source_stamps;
};

// Parse a .ari document from an in-memory string. Never throws. Enforces the
// meta.min_libcvc gate (fatal if this libcvc is too old); collects semantic
// warnings; a missing meta block is a warning, not an error. The document is checked against
// `app`'s registries: its widget types, .ari blocks (whose parsers run here) and data decoders.
LoadResult load_string(cvc::app &app, const std::string &yaml);

// Parse a .ari document from a file path, against `app`'s registries. Never throws.
LoadResult load_file(cvc::app &app, const std::string &path);

// A `mounts:` entry's document (docs/roadmap/ARIADNE_SHELL.md 4.2): mount `uri` as a static
// `load: <uri>` with `as: <as>` and `args:` would be, its relative URI resolved against `base_dir`
// (the directory of the document that holds the slot).
struct MountRequest {
  std::string uri;
  std::string as;
  std::vector<std::pair<std::string, std::string>> args;
  std::string base_dir;
  // ARIADNE_SHELL.md 4.5: what the entry is granted (its slot's grants:, picked by the entry's own
  // grants, its deny: applied; null: the defaults), which the loader bounds by `held_by`, what the
  // document that holds the slot holds (null: it is the outermost document, alone).
  std::shared_ptr<const GrantSet> grants;
  std::shared_ptr<const MountGrants> held_by;
  // 4.6 the entry's argv, when it has one (has_argv), parsed by the document's args: block.
  bool has_argv = false;
  std::vector<std::string> argv;
};
struct MountResult {
  bool ok = false;
  std::string error; // why not mounted (unresolved, a parse error, a guard, min_libcvc, too deep)
  // The mount: a Group with scope `includes.<as>`, the fragment its child, carrying its init:,
  // on_tick:, functions, requires: and application name, as expand_load builds a load:'s.
  Widget wrapper;
  std::string canonical;  // where its bytes came from (the resolver's canonical: a file's path)
  bool has_scene = false; // it declares scene: or scenes:, which a mount does not carry yet (SH3)
  std::vector<std::string> warnings;
  std::vector<SchemaIssue> schema_issues; // its own and its imports' and loads' (each in warnings)
  // The local files read, its own first, and each one's mtime when it was read (hot reload).
  std::vector<std::string> sources;
  std::map<std::string, std::int64_t> source_stamps;
  // Its `tests:` block, as LoadResult::tests: `ari test --mounted` runs them in its scope.
  std::vector<std::pair<std::string, std::string>> tests;
};
// Resolve and parse a mounts: entry's document, against `app`'s registries and resolver, through
// the path a static load: takes (the URI resolver, the cycle, depth and total guards, the
// component paths). Stateless: the Runtime keeps what it mounts. Never throws.
MountResult load_mount(cvc::app &app, const MountRequest &req);

// A YAML text (JSON is YAML too) as a Value, as a document's own maps are read: false, and why,
// when it does not parse or this build has no YAML. Never throws. A vehicle rig's file is read
// with it (vehicle_rig).
bool parse_value(const std::string &text, Value &out, std::string *why = nullptr);

// §12.5 hot-reload: poll whether any of `sources` (e.g. LoadResult::sources) has changed on disk
// since the last call. `stamps` is caller-owned state carried across calls (path -> last mtime);
// pass the same map each poll. SEED it with LoadResult::source_stamps right after a load, so a
// change between the load reading a file and the first poll is caught (a first-seen path with no
// prior stamp is adopted as a baseline and is not itself a change). Returns true when any source's
// mtime differs from its recorded stamp, updating `stamps` and pruning entries for paths no longer
// in `sources` (so the map stays bounded and a re-added path is re-baselined). A missing/unreadable
// file is treated as unchanged (a mid-write flicker should not thrash a reload). On true, the host
// re-runs load_file + Runtime::set_root — the reconcile re-mounts changed fragments (holes are
// re-wired; §12). Never throws.
bool sources_changed(const std::vector<std::string> &sources,
                     std::map<std::string, std::int64_t> &stamps);

// Whether this build has the YAML parser (libcvc built with yaml-cpp). When
// false, load_* return {ok:false, error:"...built without yaml-cpp..."}.
bool have_yaml();

// Whether this build has the JSON-Schema validator (nlohmann-json +
// json-schema-validator). When true, load_* check documents against the .ari schema
// (LoadResult::schema_issues) and validate_* work.
bool have_jsonschema();

// The .ari schema: the JSON Schema (draft 2020-12, in the subset json-schema-validator
// implements) a document's structure must follow — every widget type, scene node and
// top-level block, and the keys and values each takes. The text of
// src/cvc/ariadne/schema/ari.v1.schema.json, compiled in; installed as
// share/libcvc/ariadne/schema/ari.v1.schema.json for editors. Available regardless of
// have_jsonschema().
std::string ari_schema_json();

// Check a .ari document's structure against the schema, without loading it: the YAML parses,
// and the document, and each of its own units: instantiated with its declared args, follow
// the schema. A top-level block `app` registered (register_ari_block) or the document declares
// in customs: is allowed. It resolves nothing (no import:, load: or include: of an imported
// unit): a load does that, and checks what it reaches (LoadResult::schema_issues). Never throws.
ValidationResult validate_string(cvc::app &app, const std::string &yaml);
// The same for a file; each issue's `source` is the file's path as given.
ValidationResult validate_file(cvc::app &app, const std::string &path);

// Whether `type` is a widget type the loader builds itself (text, slider_int, button, …) rather
// than a custom one (has_widget_type). requires: and (has? ...) accept both.
bool is_builtin_widget_type(const std::string &type);

// Every built-in widget type, in the order the loader looks for a widget's type key: a widget
// map's type is the first of these it has (so `progress: P` with `overlay: 50%` is a progress).
std::vector<std::string> builtin_widget_types();

// Every built-in top-level block (meta, windows, scene, ...): the keys the loader reads itself.
// Any other top-level key is a custom block's (register_ari_block).
std::vector<std::string> builtin_ari_blocks();

// The libcvc version this build reports (CVC_VERSION_STRING) — what the
// min_libcvc gate compares against. Exposed for tooling / diagnostics.
std::string libcvc_version();

// Compare dotted numeric versions: true iff `have` >= `need` (major.minor.patch;
// missing components are 0; a pre-release/build suffix after '-' or '+' is
// ignored). Exposed so callers can pre-check a document's min_libcvc.
bool version_at_least(const std::string &have, const std::string &need);

// --- extensibility: custom top-level document blocks -------------------------
//
// A parser for a CUSTOM top-level block — a document key that is NOT a built-in
// (built-ins: meta/menubar/windows/overlays/root/children/scene/customs). When a
// loaded document contains a registered key, `parse` is called with that block's
// content as a neutral Value and the LoadResult being built, so it can stash parsed
// data into `LoadResult::extras` (or append to `root`/`scene`) and push notes into
// `out.warnings`. This keeps the loader open: a new block plugs in without editing
// load_node. Registration is the app's and thread-safe; register before load_*.
using AriBlockParser = std::function<void(const Value &content, LoadResult &out)>;

// Register (or replace) `app`'s parser for a custom top-level block `key`. Registering a
// built-in key is a no-op (built-ins own their dispatch). Documents loaded against another app
// don't see it.
void register_ari_block(cvc::app &app, const std::string &key, AriBlockParser parse);

// Whether `app` has a parser for custom block `key` (test/introspection).
bool has_ari_block(cvc::app &app, const std::string &key);

} // namespace ariadne
} // namespace cvc

#endif // CVC_ARIADNE_LOADER_H
