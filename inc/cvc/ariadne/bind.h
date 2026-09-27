#ifndef CVC_ARIADNE_BIND_H
#define CVC_ARIADNE_BIND_H

// Ariadne state-binding primitives (cvc::ariadne). Pure libcvc — NO ImGui/VTK.
//
// These are the ONE definition of how an Ariadne bind path becomes a live
// cvc::state read/write, shared by BOTH the widget Runtime (src/cvc/ariadne/
// ariadne.cpp) and the scene binder (the cvcGL realizer, §9). Sharing them is the
// point: a widget `bind: demo.show_mesh` and a scene node `visible: demo.show_mesh`
// MUST resolve to the identical state key, or the checkbox and the mesh would drift
// onto two different keys. Lift, never re-hand-roll.

#include <boost/lexical_cast.hpp>
#include <cctype>
#include <cvc/core/app.h>
#include <cvc/core/state.h>
#include <exception>
#include <string>
#include <vector>

namespace cvc {
namespace ariadne {

// Resolve a bind path to an absolute cvc::state path:
//   leading '/'  -> app-root-absolute (strip the '/')
//   else, prefix -> "<prefix>.<bind>"   (splice on cvc::state::SEPARATOR)
//   else         -> bind as-is (app-root-relative / empty prefix)
// This is verbatim the rule Runtime::Impl::resolve uses for widgets.
inline std::string resolve_bind(const std::string &prefix, const std::string &bind) {
  if (bind.empty())
    return bind;
  if (bind[0] == '/')
    return bind.substr(1);
  if (prefix.empty())
    return bind;
  return prefix + cvc::state::SEPARATOR + bind;
}

// §12 load: follow a TRANSPARENT link node to its terminal target for a READ. A mount's
// parent-scope "hole" is a transparent link (§7.8.3); a bound widget reading it must see THROUGH
// to the granted target, matching what a WRITE already does (state::value(v) routes through a
// writable transparent link). A non-link / opaque / broken / cyclic link stays put (the same
// fallback resolvedValue documents), so this only changes reads of a resolvable transparent link
// — of which an Ariadne tree has none except deliberately-planted holes. Reads use the resolved
// node; SEEDS/WRITES stay on the ORIGINAL node so the link's writability gate still applies (a
// read-only hole keeps its own local value on write, never scribbling the target).
inline cvc::state &read_effective(cvc::state &s) {
  if (s.isLink() && s.linkMode() == cvc::state::link_mode::transparent) {
    const cvc::state::link_resolution lr = s.resolveLink();
    if (lr.kind == cvc::state::link_resolution_kind::resolved && lr.target)
      return *lr.target;
  }
  return s;
}

// Read a state value, seeding it with `def` when the path has no value yet. Never
// throws into a render frame (unreadable/unconvertible -> `def`).
template <typename T> T read_or_seed(cvc::app &ctx, const std::string &path, const T &def) {
  try {
    cvc::state &s = cvc::state::instance(ctx)(path);
    cvc::state &eff = read_effective(s); // read through a transparent-link hole to its target
    if (eff.value().empty()) {
      s.value(def); // seed via the ORIGINAL node: writable hole -> target, else local/own
      return def;
    }
    return eff.value<T>();
  } catch (const std::exception &) {
    return def;
  }
}

// Read a state value WITHOUT seeding — returns `def` when the path has no value yet
// and never writes. This is for a pure FOLLOWER of a key that some other layer owns
// and seeds (e.g. a scene `visible:` bind reading a key a widget checkbox seeds with
// its own `def:`): a follower must not pre-empt the owner's default. Never throws.
template <typename T> T read_or(cvc::app &ctx, const std::string &path, const T &def) {
  try {
    cvc::state &eff = read_effective(cvc::state::instance(ctx)(path));
    if (eff.value().empty())
      return def;
    return eff.value<T>();
  } catch (const std::exception &) {
    return def;
  }
}

// Interpret a state string as a BOOLEAN 0/1, tolerantly. cvc::state backs a bool as an int
// ("1"/"0"), but state_exec renders a bool as "true"/"false" — so an `init:`/action `(state-set
// "flag" "true")` or a bool computed in a program would otherwise be UNREADABLE by an int-backed
// bool widget (value<int>("true") throws). Accept both spellings (and yes/no/on/off), plus any
// numeric (non-zero -> 1); anything else -> `def`. Used ONLY by the bool widgets + visibility
// reads, NEVER by slider_int (a real integer must not be coerced to 0/1).
inline int to_bool_int(const std::string &s, int def) {
  std::string t;
  for (char c : s)
    if (!std::isspace(static_cast<unsigned char>(c)))
      t += static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  if (t == "1" || t == "true" || t == "yes" || t == "on" || t == "y" || t == "t")
    return 1;
  if (t == "0" || t == "false" || t == "no" || t == "off" || t == "n" || t == "f")
    return 0;
  try {
    return boost::lexical_cast<int>(s) != 0 ? 1 : 0; // any other numeric -> truthiness
  } catch (const std::exception &) {
    return def;
  }
}

// Read a BOOLEAN state key as 0/1, seeding `def` when unset — the bool analogue of
// read_or_seed<int> (checkbox / menu-toggle), tolerant of the "true"/"false" spelling a
// program/init: writes. Never throws into a frame.
inline int read_bool_or_seed(cvc::app &ctx, const std::string &path, int def) {
  try {
    cvc::state &s = cvc::state::instance(ctx)(path);
    cvc::state &eff = read_effective(s);
    const std::string v = eff.value();
    if (v.empty()) {
      s.value(def); // seed via the ORIGINAL node (writable-hole aware, like read_or_seed)
      return def;
    }
    return to_bool_int(v, def);
  } catch (const std::exception &) {
    return def;
  }
}

// Read a BOOLEAN state key as 0/1 WITHOUT seeding — the bool analogue of read_or<int> (a scene
// `visible:` follower), tolerant of "true"/"false". Never throws.
inline int read_bool_or(cvc::app &ctx, const std::string &path, int def) {
  try {
    cvc::state &eff = read_effective(cvc::state::instance(ctx)(path));
    const std::string v = eff.value();
    if (v.empty())
      return def;
    return to_bool_int(v, def);
  } catch (const std::exception &) {
    return def;
  }
}

// Write a state value; swallows read-only / unwritable (never throws into a frame).
// A write equal to the current value is a no-op inside cvc::state (equality guard),
// so re-writing every frame fires observers only on an actual change.
//
// Goes through the typed value<T> setter. Both setters now share one Phase 8 routing
// block, so a write through a writable transparent-link HOLE reaches its resolved target
// (§12) whichever setter is used — but value<T> also records the target's real
// valueTypeName instead of clobbering it to "std::string" the way the string setter does.
// For a std::string T, .value(v) resolves to the (preferred non-template) string overload,
// so text writes are unchanged; int/double writes now land typed. value<T> encodes via the
// same lexical_cast the string path used, so the stored string is byte-identical for a plain
// (non-link) node — this only upgrades the recorded type, never the representation.
template <typename T> void write(cvc::app &ctx, const std::string &path, const T &v) {
  try {
    cvc::state::instance(ctx)(path).value(v);
  } catch (const std::exception &) {
  }
}

// One scene-node visibility binding (§9): mirror `source_path` (the resolved
// `visible: <path>` bind) into `target_path` (the node's own `<node>.visible` key).
// Both are absolute state paths resolved once, at realize time — no VTK node pointer
// is captured, so the poll can never dangle into a torn-down node. The node's own
// state_object machinery (SceneNode::handleStateChanged) turns the `.visible` write
// into a setVisible on the owner thread.
struct SceneVisibilityBinding {
  std::string source_path;     // resolved bind path some widget layer OWNS (+ seeds)
  std::string target_path;     // the node's `<node-state-path>.visible` key
  bool default_visible = true; // fallback ONLY while the source has no value yet
};

// Poll every visibility binding once and mirror source -> node `.visible`. Reads and
// writes cvc::state only (no VTK) — safe to call each frame from the host render
// loop. The source is read WITHOUT seeding: a scene `visible:` bind is a follower, so
// whichever widget owns the key (e.g. a checkbox with its own `def:`) is the sole
// seeder — the node's `default_visible` is only a fallback until then, and is never
// written into the shared source key. Reading as int matches the 0/1 encoding the
// widgets use; the write to the node key no-ops unless the value actually changed.
// Call from the owner/render thread so the node's setVisible runs inline the frame.
inline void sync_scene_visibility(cvc::app &app,
                                  const std::vector<SceneVisibilityBinding> &bindings) {
  for (const SceneVisibilityBinding &b : bindings) {
    const int v = read_bool_or(app, b.source_path, b.default_visible ? 1 : 0); // visible = a bool
    write<int>(app, b.target_path, v);
  }
}

} // namespace ariadne
} // namespace cvc

#endif // CVC_ARIADNE_BIND_H
