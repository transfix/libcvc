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

// Read a state value, seeding it with `def` when the path has no value yet. Never
// throws into a render frame (unreadable/unconvertible -> `def`).
template <typename T> T read_or_seed(cvc::app &ctx, const std::string &path, const T &def) {
  try {
    cvc::state &s = cvc::state::instance(ctx)(path);
    const std::string raw = s.value();
    if (raw.empty()) {
      s.value(def);
      return def;
    }
    return s.value<T>();
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
    cvc::state &s = cvc::state::instance(ctx)(path);
    const std::string raw = s.value();
    if (raw.empty())
      return def;
    return s.value<T>();
  } catch (const std::exception &) {
    return def;
  }
}

// Write a state value; swallows read-only / unwritable (never throws into a frame).
// A write equal to the current value is a no-op inside cvc::state (equality guard),
// so re-writing every frame fires observers only on an actual change.
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
    const int v = read_or<int>(app, b.source_path, b.default_visible ? 1 : 0);
    write<int>(app, b.target_path, v);
  }
}

} // namespace ariadne
} // namespace cvc

#endif // CVC_ARIADNE_BIND_H
