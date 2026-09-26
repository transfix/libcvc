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

#include <string>
#include <vector>

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

struct LoadResult {
  bool ok = false;                   // false if the load failed (see `error`)
  Widget root;                       // a Group of the document's widgets (empty on failure)
  Meta meta;                         // parsed provenance (may be empty)
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

// The libcvc version this build reports (CVC_VERSION_STRING) — what the
// min_libcvc gate compares against. Exposed for tooling / diagnostics.
std::string libcvc_version();

// Compare dotted numeric versions: true iff `have` >= `need` (major.minor.patch;
// missing components are 0; a pre-release/build suffix after '-' or '+' is
// ignored). Exposed so callers can pre-check a document's min_libcvc.
bool version_at_least(const std::string &have, const std::string &need);

} // namespace ariadne
} // namespace cvc

#endif // CVC_ARIADNE_LOADER_H
