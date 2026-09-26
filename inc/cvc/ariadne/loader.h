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
// a .ari document and a hand-built tree are interchangeable. The full schema
// (frame/layout/size, expressions, scene graph, meta/validation) follows in later
// slices; unknown keys are ignored, not errors, so a fuller document still loads
// its P0 subset.

#include <functional>
#include <string>
#include <utility>
#include <vector>

#include <cvc/ariadne/scene.h>
#include <cvc/ariadne/value.h>
#include <cvc/ariadne/widget.h>

namespace cvc {
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

struct LoadResult {
  bool ok = false;                   // false if the load failed (see `error`)
  Widget root;                       // a Group of the document's widgets (empty on failure)
  Scene scene;                       // the parsed `scene:` block (§9; empty if none)
  Meta meta;                         // parsed provenance (may be empty)
  std::vector<CustomRequirement> customs; // declared `customs:` (widget/node/block)
  // The `init:` block's state_exec script (verbatim text; empty if none). The loader
  // only CAPTURES it (it has no cvc::app and never runs the DSL); the host runs it once
  // at load via cvc::ariadne::run_init, scoped to the document prefix.
  std::string init_script;
  // Custom top-level blocks (register_ari_block), keyed by block name — whatever the
  // registered parser stored. Empty unless a document uses a registered custom block.
  std::vector<std::pair<std::string, Value>> extras;
  std::vector<std::string> warnings; // non-fatal issues (unknown types, empty binds, …)
  std::string error;                 // human-readable message when !ok
};

// Parse a .ari document from an in-memory string. Never throws. Enforces the
// meta.min_libcvc gate (fatal if this libcvc is too old); collects semantic
// warnings; a missing meta block is a warning, not an error.
LoadResult load_string(const std::string &yaml);

// Parse a .ari document from a file path. Never throws.
LoadResult load_file(const std::string &path);

// Whether this build has the YAML parser (libcvc built with yaml-cpp). When
// false, load_* return {ok:false, error:"...built without yaml-cpp..."}.
bool have_yaml();

// Whether this build has the JSON-Schema validator (nlohmann-json +
// json-schema-validator). When true, load_* run Layer-2 structural validation
// (roadmap §15) against the .ari schema and add any violations to `warnings`.
bool have_jsonschema();

// The JSON Schema (draft 2020-12) for a .ari document — the machine-readable
// structural contract Layer-2 validates against, also usable by external tooling
// (an editor, `ari validate`). Available regardless of have_jsonschema().
std::string ari_schema_json();

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
// load_node. Registration is process-global and thread-safe; register before load_*.
using AriBlockParser = std::function<void(const Value &content, LoadResult &out)>;

// Register (or replace) the parser for a custom top-level block `key`. Registering a
// built-in key is a no-op (built-ins own their dispatch). Mirrors the libcvc registry
// idiom (a thread-safe name→handler map with a process-global instance).
void register_ari_block(const std::string &key, AriBlockParser parse);

// Whether a custom block `key` has a registered parser (test/introspection).
bool has_ari_block(const std::string &key);

} // namespace ariadne
} // namespace cvc

#endif // CVC_ARIADNE_LOADER_H
