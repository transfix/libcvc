// pycvc_state.h — direct access to the injected app's state tree, plus a
// director base for Python push callbacks on state changes.
//
// Phase-3 rearchitecture. All operations act on the ONE shared root,
// cvc::state::instance(pycvc::ctx()) — the same tree the host app (and the DSL,
// Phase 4) see, so a write here is visible everywhere. state_set()/state_get()
// live in pycvc_context.h (Phase 0); this header adds has/children/remove and
// the observer. There is no facade State object: these are direct functions on
// the real tree.
#pragma once

#include <memory>
#include <string>
#include <vector>

namespace pycvc {

// ── Direct state access on the shared root ──────────────────────────────
// True iff a node exists at `path` (dotted, e.g. "a.b.c").
bool state_has(const std::string &path);
// Immediate child leaf-names of `path` ("" = the root). Reduced from
// cvc::state::children()'s recursive absolute-path list to single segments.
std::vector<std::string> state_children(const std::string &path);
// Remove the node at `path` (and its subtree). Idempotent; the root is never
// removed.
void state_remove(const std::string &path);

// ── Push callbacks (SWIG director) ──────────────────────────────────────
// Subclass in Python and override on_changed(path); call watch() to connect to
// the shared root's tree-wide childChanged signal (fires for EVERY mutation
// anywhere, carrying the full dotted path — real push, no polling). The signal
// fires SYNCHRONOUSLY on the state-writing thread, which may not hold the GIL,
// so the C++ dispatch acquires it before the Python upcall and swallows any
// exception (a bad observer must never unwind into the writer). The connection
// is a scoped_connection owned here: unwatch() or destruction disconnects it
// deterministically, so a dead Python observer is never called.
class state_observer {
public:
  state_observer();
  virtual ~state_observer();

  // Overridden in Python. `path` is the dotted path of the changed node.
  virtual void on_changed(const std::string &path);

  void watch();          // connect to the shared root's childChanged
  void unwatch();        // disconnect (idempotent)
  bool watching() const; // currently connected?

private:
  struct Impl;                 // holds the boost::signals2::scoped_connection
  std::shared_ptr<Impl> impl_; // (kept out of the SWIG-visible surface)
};

} // namespace pycvc
