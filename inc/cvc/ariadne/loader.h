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

#include <cvc/ariadne/widget.h>

namespace cvc {
namespace ariadne {

struct LoadResult {
  bool ok = false;     // false if parsing failed (see `error`) or yaml is absent
  Widget root;         // a Group holding the document's widgets (empty on failure)
  std::string error;   // human-readable message when !ok
};

// Parse a .ari document from an in-memory string. Never throws.
LoadResult load_string(const std::string &yaml);

// Parse a .ari document from a file path. Never throws.
LoadResult load_file(const std::string &path);

// Whether this build has the YAML parser (libcvc built with yaml-cpp). When
// false, load_* return {ok:false, error:"...built without yaml-cpp..."}.
bool have_yaml();

} // namespace ariadne
} // namespace cvc

#endif // CVC_ARIADNE_LOADER_H
