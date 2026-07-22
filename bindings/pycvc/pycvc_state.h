// pycvc_state.h — Python-facing facade over the libcvc reactive state tree
// (cvc::state / cvc::state_object).
//
// SWIG-safe, exactly like pycvc_geometry.h / pycvc_volume.h: it forward-
// declares the cvc types and keeps a raw pointer to the app-scoped root plus a
// pimpl (std::shared_ptr<StateImpl>) for the change-recorder machinery. ONLY
// pycvc_state.cpp includes libcvc's headers, so the SWIG parser sees nothing
// but std::string / std::vector / bool signatures.
//
// cvc::state is a thread-safe tree of dotted-path nodes, each holding a string
// value; setting a value fires a boost::signals2 `valueChanged` signal. This
// facade binds to the per-process root (cvc::state::instance(process_ctx()))
// — the SAME root pycvc.Exec runs DSL programs against — so state written from
// Python is visible to the DSL and back.
#pragma once

#include <memory>
#include <string>
#include <vector>

namespace cvc {
class state;
class app;
} // namespace cvc

namespace pycvc {

// Opaque implementation holder (change recorder + signal connections); defined
// only in pycvc_state.cpp so no boost/std-mutex types cross the SWIG parser.
struct StateImpl;

class State {
public:
  // Bind to the process-wide state-tree root (cvc::state::instance of the
  // shared process_ctx()).
  State();
  ~State();

  // ── Read / write nodes by dotted path ──────────────────────────────
  // set() creates intermediate nodes as needed (like the state-set DSL
  // intrinsic: `(*root)(path).value(v)`), then records the path as changed
  // (see poll_changes()).
  void set(const std::string &path, const std::string &value);

  // Value of the node at `path`. Throws if the path does not exist; a node
  // that exists but was never assigned a value returns "".
  std::string get(const std::string &path);

  // True iff a node exists at `path` (does NOT create it).
  bool has(const std::string &path);

  // Immediate child leaf-names of the node at `path` (empty path = root).
  // Returns [] for a missing path. NB: this is the direct children only, not
  // the whole recursive subtree that cvc::state::children() returns.
  std::vector<std::string> children(const std::string &path = std::string());

  // Delete the node at `path` (and its subtree). Idempotent no-op if absent.
  // Implemented via immediate expiry + sweep, mirroring the state-delete
  // intrinsic. The root itself cannot be removed.
  void remove(const std::string &path);

  // ── Change observation (record-and-poll; no C++ callback crosses SWIG) ──
  // The facade owns a boost::signals2 subscription on every node it has
  // written; each value change records the path. poll_changes() returns the
  // set of paths changed since the previous poll (sorted, unique) and clears
  // the record. This is the safe alternative to wiring a Python callable
  // through a C++ signal: no director, no raw callback pointer, and the
  // subscription lifetime is owned entirely on the C++ side.
  std::vector<std::string> poll_changes();

  // ── C++-only bridge for host apps (SWIG-ignored) ───────────────────
  cvc::state &native();
  const cvc::state &native() const;

private:
  void ensure_watch(const std::string &path, cvc::state &node);

  cvc::state *root_;               // app-scoped; outlives this facade (non-owning)
  std::shared_ptr<StateImpl> impl_; // change recorder + signal connections
};

} // namespace pycvc
