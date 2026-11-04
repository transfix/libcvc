// pycvc_state.cpp — implementation of the direct state accessors and the
// state_observer director. Includes Python.h for GIL management around the
// cross-thread director upcall.
// Python.h must precede any standard headers (it sets feature macros the
// standard headers read); keep it first even under clang-format's regrouping.
// clang-format off
#include <Python.h>
// clang-format on

#include "pycvc_state.h"

#include <atomic>
#include <boost/signals2/connection.hpp>
#include <cvc/core/app.h>
#include <cvc/state/state.h>
#include <stdexcept>
#include <vector>

namespace pycvc {

namespace {
constexpr char kSep = '.'; // cvc::state::SEPARATOR

cvc::state &root_of(const std::shared_ptr<cvc::app> &app) {
  if (!app)
    throw std::invalid_argument("pycvc state op: null app handle");
  return cvc::state::instance(*app);
}
} // namespace

// These run on Python threads while an Exec's thread may delete nodes (td 4.12), so every node
// they use is held by a pinned walk (td 4.8): a read holds the node, and a write, or a walk up
// the parents (fullName(), a write's childChanged), holds the chain below the app root.

void state_set(const std::shared_ptr<cvc::app> &app, const std::string &path,
               const std::string &value) {
  // sharedChild(path) creates child nodes as needed, as operator() does; value() fires
  // valueChanged only on an actual change.
  cvc::pin_vector chain; // released leaf first
  root_of(app).sharedChild(path, &chain)->value(value);
}

std::string state_get(const std::shared_ptr<cvc::app> &app, const std::string &path) {
  const cvc::state::state_ptr node = root_of(app).findDescendantShared(path);
  if (!node)
    throw std::out_of_range("pycvc.state_get: no node at path: " + path);
  return node->value();
}

bool state_has(const std::shared_ptr<cvc::app> &app, const std::string &path) {
  return static_cast<bool>(root_of(app).findDescendantShared(path));
}

namespace {
cvc::state::state_ptr node_at(const std::shared_ptr<cvc::app> &app, const std::string &path,
                              const char *op) {
  cvc::state::state_ptr node = root_of(app).findDescendantShared(path);
  if (!node)
    throw std::out_of_range(std::string("pycvc.") + op + ": no node at path: " + path);
  return node;
}
} // namespace

std::string state_comment(const std::shared_ptr<cvc::app> &app, const std::string &path) {
  return node_at(app, path, "state_comment")->comment();
}

std::string state_property(const std::shared_ptr<cvc::app> &app, const std::string &path,
                           const std::string &key) {
  return node_at(app, path, "state_property")->property(key);
}

std::vector<std::string> state_property_keys(const std::shared_ptr<cvc::app> &app,
                                             const std::string &path) {
  return node_at(app, path, "state_property_keys")->property_keys();
}

std::vector<std::string> state_children(const std::shared_ptr<cvc::app> &app,
                                        const std::string &path) {
  // children() names them from fullName(), up the chain the walk holds.
  cvc::pin_vector chain; // released leaf first
  const cvc::state::state_ptr node = root_of(app).findDescendantShared(path, &chain);
  if (!node)
    return {};
  // cvc::state::children() returns the full recursive descendant list as
  // absolute dotted paths (fullName()); reduce to immediate single-segment
  // children of this node.
  std::string prefix = node->fullName();
  if (!prefix.empty())
    prefix.push_back(kSep);
  std::vector<std::string> immediate;
  for (const std::string &full : node->children()) {
    if (full.size() <= prefix.size())
      continue;
    if (full.compare(0, prefix.size(), prefix) != 0)
      continue;
    std::string rest = full.substr(prefix.size());
    if (rest.find(kSep) == std::string::npos)
      immediate.push_back(std::move(rest));
  }
  return immediate;
}

void state_remove(const std::shared_ptr<cvc::app> &app, const std::string &path) {
  // The node is unlinked from its parent, as the state-delete intrinsic does (td 4.8): nothing
  // else is swept, and the root cannot be removed (an empty path does nothing). Idempotent.
  const std::string key = cvc::state::normalize_path(path);
  if (key.empty())
    return;
  cvc::state &root = root_of(app);
  const std::size_t dot = key.rfind(kSep);
  if (dot == std::string::npos) {
    root.removeChild(key);
    return;
  }
  cvc::pin_vector chain; // removeChild emits up the parent's chain; released leaf first
  if (const cvc::state::state_ptr parent = root.findDescendantShared(key.substr(0, dot), &chain))
    parent->removeChild(key.substr(dot + 1));
}

// ── Whole-tree persistence (routes through the root; null app throws) ─────
void state_save(const std::shared_ptr<cvc::app> &app, const std::string &path) {
  root_of(app).save(path);
}

void state_restore(const std::shared_ptr<cvc::app> &app, const std::string &path) {
  root_of(app).restore(path);
}

std::string state_json(const std::shared_ptr<cvc::app> &app) { return root_of(app).json(); }

void state_from_json(const std::shared_ptr<cvc::app> &app, const std::string &json) {
  root_of(app).json(json);
}

// ── state_observer ──────────────────────────────────────────────────────
struct state_observer::Impl {
  // Co-own the app while watching so its state tree (and our signal) outlive us.
  std::shared_ptr<cvc::app> app;
  // scoped_connection auto-disconnects on destruction, so a dead observer's
  // slot is removed before its (director) object is freed.
  boost::signals2::scoped_connection conn;
  // Whether the slot may still call the observer. Disconnecting does not wait for a slot already
  // called: a writer on another thread (an Exec's run, which releases the GIL) can be inside the
  // slot, waiting for the GIL, while the thread holding it drops the observer (R0a-mem-1). So the
  // slot checks this once it has the GIL, and unwatch() and the destructor, which run with the GIL
  // held (from Python), clear it before they disconnect.
  std::shared_ptr<std::atomic<bool>> live;

  void stop_calls() {
    if (live)
      live->store(false, std::memory_order_release);
    live.reset();
  }
};

state_observer::state_observer() : impl_(std::make_shared<Impl>()) {}
state_observer::~state_observer() { impl_->stop_calls(); } // then impl_ disconnects

// Default no-op; Python subclasses override this (director upcall).
void state_observer::on_changed(const std::string & /*path*/) {}

void state_observer::watch(const std::shared_ptr<cvc::app> &app) {
  cvc::state &root = root_of(app);
  impl_->stop_calls(); // a previous watch's slot calls no more
  impl_->app = app;    // keep the app alive while we watch its tree
  const auto live = std::make_shared<std::atomic<bool>>(true);
  impl_->live = live;
  // The root's childChanged fires for every mutation anywhere in the tree,
  // carrying the full dotted path (each node re-fires its parent with
  // name()+SEP+child). This runs on the WRITER thread — acquire the GIL before
  // the Python upcall, and only then see whether the observer is still there.
  impl_->conn = root.childChanged.connect([this, live](const std::string &path) {
    PyGILState_STATE gil = PyGILState_Ensure();
    if (live->load(std::memory_order_acquire)) {
      try {
        this->on_changed(path); // director → Python override
      } catch (...) {
        // A misbehaving observer must never unwind into boost::signals2 / the
        // state writer; drop it (and surface any pending Python error below).
      }
      if (PyErr_Occurred())
        PyErr_Print();
    }
    PyGILState_Release(gil);
  });
}

void state_observer::unwatch() {
  impl_->stop_calls();
  impl_->conn.disconnect();
  impl_->app.reset(); // release the app once we're no longer watching
}

bool state_observer::watching() const { return impl_->conn.connected(); }

} // namespace pycvc
