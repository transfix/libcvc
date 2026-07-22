// pycvc_state.cpp — implementation of the direct state accessors and the
// state_observer director. Includes Python.h for GIL management around the
// cross-thread director upcall.
// Python.h must precede any standard headers (it sets feature macros the
// standard headers read); keep it first even under clang-format's regrouping.
// clang-format off
#include <Python.h>
// clang-format on

#include "pycvc_state.h"

#include "pycvc_context.h"

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/signals2/connection.hpp>
#include <cvc/core/app.h>
#include <cvc/core/state.h>

namespace pycvc {

namespace {
constexpr char kSep = '.'; // cvc::state::SEPARATOR
} // namespace

bool state_has(const std::string &path) {
  return cvc::state::instance(ctx()).findDescendant(path) != nullptr;
}

std::vector<std::string> state_children(const std::string &path) {
  cvc::state &root = cvc::state::instance(ctx());
  cvc::state *node = path.empty() ? &root : root.findDescendant(path);
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

void state_remove(const std::string &path) {
  cvc::state &root = cvc::state::instance(ctx());
  cvc::state *node = root.findDescendant(path);
  if (!node)
    return; // idempotent
  // Immediate expiry + sweep — the same mechanism the state-delete intrinsic
  // uses. (expireAt on the root is a no-op; the root cannot be removed.)
  node->expireAt(boost::posix_time::microsec_clock::universal_time());
  root.sweepExpired();
}

// ── state_observer ──────────────────────────────────────────────────────
struct state_observer::Impl {
  // scoped_connection auto-disconnects on destruction, so a dead observer's
  // slot is removed before its (director) object is freed.
  boost::signals2::scoped_connection conn;
};

state_observer::state_observer() : impl_(std::make_shared<Impl>()) {}
state_observer::~state_observer() = default;

// Default no-op; Python subclasses override this (director upcall).
void state_observer::on_changed(const std::string & /*path*/) {}

void state_observer::watch() {
  cvc::state &root = cvc::state::instance(ctx());
  // The root's childChanged fires for every mutation anywhere in the tree,
  // carrying the full dotted path (each node re-fires its parent with
  // name()+SEP+child). This runs on the WRITER thread — acquire the GIL before
  // the Python upcall.
  impl_->conn = root.childChanged.connect([this](const std::string &path) {
    PyGILState_STATE gil = PyGILState_Ensure();
    try {
      this->on_changed(path); // director → Python override
    } catch (...) {
      // A misbehaving observer must never unwind into boost::signals2 / the
      // state writer; drop it (and surface any pending Python error below).
    }
    if (PyErr_Occurred())
      PyErr_Print();
    PyGILState_Release(gil);
  });
}

void state_observer::unwatch() { impl_->conn.disconnect(); }

bool state_observer::watching() const { return impl_->conn.connected(); }

} // namespace pycvc
