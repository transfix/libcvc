// pycvc_state.cpp — State facade implementation (one of only two TUs, with
// pycvc_exec.cpp, that include libcvc's state headers).
#include "pycvc_state.h"

#include "pycvc_context.h"

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/signals2/connection.hpp>
#include <cvc/core/app.h>
#include <cvc/core/state.h>
#include <cvc/core/state_object.h>
#include <map>
#include <mutex>
#include <set>
#include <stdexcept>

namespace pycvc {

// Shared process-wide app context (declared in pycvc_context.h). Same idiom as
// pycvc_volume.cpp's ctx(), but exposed so pycvc_exec.cpp binds to the SAME
// app root — the invariant that makes the Python<->state<->DSL round trip work.
cvc::app &process_ctx() {
  static cvc::app app;
  return app;
}

namespace {
constexpr char kSep = '.'; // cvc::state::SEPARATOR
} // namespace

// The change record. Kept behind a shared_ptr so the signal slots can hold a
// weak reference to it: even if the State facade is destroyed while a node
// later fires, the slot's weak_ptr::lock() fails and nothing dangles.
struct change_sink {
  std::mutex mu;
  std::set<std::string> pending; // sorted + unique
};

struct StateImpl {
  std::shared_ptr<change_sink> sink = std::make_shared<change_sink>();
  std::mutex conn_mu;
  // path -> subscription on that node's valueChanged. scoped_connection
  // auto-disconnects on destruction, so tearing down the facade detaches all
  // slots before `sink` is freed.
  std::map<std::string, boost::signals2::scoped_connection> conns;
};

State::State()
    : root_(&cvc::state::instance(process_ctx())), impl_(std::make_shared<StateImpl>()) {}
State::~State() = default;

// Subscribe (once per path) to the node's valueChanged so future writes to it
// record the path. The slot captures a weak_ptr to the sink (lifetime-safe)
// and the path by value.
void State::ensure_watch(const std::string &path, cvc::state &node) {
  std::lock_guard<std::mutex> lk(impl_->conn_mu);
  if (impl_->conns.find(path) != impl_->conns.end())
    return;
  std::weak_ptr<change_sink> weak = impl_->sink;
  impl_->conns[path] = node.valueChanged.connect([weak, path]() {
    if (auto s = weak.lock()) {
      std::lock_guard<std::mutex> l(s->mu);
      s->pending.insert(path);
    }
  });
}

void State::set(const std::string &path, const std::string &value) {
  // operator()(path) creates child nodes as needed (same as state-set).
  cvc::state &node = (*root_)(path);
  // Subscribe BEFORE the write so this very change is recorded. The string
  // overload of value() fires valueChanged only when the value actually
  // changes, so poll_changes() reports exactly the paths that changed.
  ensure_watch(path, node);
  node.value(value);
}

std::string State::get(const std::string &path) {
  cvc::state *node = root_->findDescendant(path);
  if (!node)
    throw std::out_of_range("State.get: no node at path: " + path);
  return node->value();
}

bool State::has(const std::string &path) { return root_->findDescendant(path) != nullptr; }

std::vector<std::string> State::children(const std::string &path) {
  cvc::state *node = root_->findDescendant(path);
  if (!node)
    return {};
  // cvc::state::children() returns the FULL recursive descendant list as
  // absolute dotted paths (fullName()). Reduce it to immediate children by
  // stripping this node's prefix and keeping only single-segment remainders.
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

void State::remove(const std::string &path) {
  cvc::state *node = root_->findDescendant(path);
  if (!node)
    return; // idempotent
  // Immediate expiry + sweep — the same mechanism the state-delete intrinsic
  // uses. (expireAt on the root is a no-op; the root cannot be removed.)
  node->expireAt(boost::posix_time::microsec_clock::universal_time());
  root_->sweepExpired();
  // Drop stale subscriptions for the removed subtree so a later re-create at
  // the same path is watched again (the old node's signal is already gone).
  std::lock_guard<std::mutex> lk(impl_->conn_mu);
  const std::string sub = path + kSep;
  for (auto it = impl_->conns.begin(); it != impl_->conns.end();) {
    if (it->first == path || it->first.compare(0, sub.size(), sub) == 0)
      it = impl_->conns.erase(it);
    else
      ++it;
  }
}

std::vector<std::string> State::poll_changes() {
  std::lock_guard<std::mutex> lk(impl_->sink->mu);
  std::vector<std::string> out(impl_->sink->pending.begin(), impl_->sink->pending.end());
  impl_->sink->pending.clear();
  return out;
}

cvc::state &State::native() { return *root_; }
const cvc::state &State::native() const { return *root_; }

} // namespace pycvc
