// pycvc_state.h — direct access to an app's state tree, plus a director base
// for Python push callbacks on state changes.
//
// Every function takes the cvc::app EXPLICITLY (no module-global): all ops act
// on that app's own root, cvc::state::instance(*app) — the same tree the host
// app (and the DSL, Phase 4) see, so a write is visible everywhere the same app
// is used. There is no facade State object; these are direct functions on the
// real tree.
#pragma once

#include <memory>
#include <string>
#include <vector>

namespace cvc {
class app;
}

namespace pycvc {

// ── Direct state access on a given app's root ───────────────────────────
// set navigates/creates dotted-path nodes (state::operator()); get reads
// without creating and throws on a missing path.
void state_set(const std::shared_ptr<cvc::app> &app, const std::string &path,
               const std::string &value);
std::string state_get(const std::shared_ptr<cvc::app> &app, const std::string &path);
bool state_has(const std::shared_ptr<cvc::app> &app, const std::string &path);
// Immediate child leaf-names of `path` ("" = the root).
std::vector<std::string> state_children(const std::shared_ptr<cvc::app> &app,
                                        const std::string &path);
// Remove the node at `path` (and its subtree). Idempotent; the root is never
// removed.
void state_remove(const std::shared_ptr<cvc::app> &app, const std::string &path);

// ── Push callbacks (SWIG director) ──────────────────────────────────────
// Subclass in Python and override on_changed(path); call watch(app) to connect
// to that app's tree-wide childChanged signal (fires for EVERY mutation
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

  void watch(const std::shared_ptr<cvc::app> &app); // connect to app's root childChanged
  void unwatch();                                   // disconnect (idempotent)
  bool watching() const;                            // currently connected?

private:
  struct Impl;                 // holds the boost::signals2::scoped_connection
  std::shared_ptr<Impl> impl_; // (kept out of the SWIG-visible surface)
};

} // namespace pycvc
