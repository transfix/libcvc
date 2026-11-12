// server_link.cpp — see the header.

#include "server_link.h"

#include <algorithm>
#include <cvc/state/state.h>
#include <iterator>
#include <memory>
#include <unordered_set>

namespace cvc {
namespace ariadne {
namespace python {

namespace {

// Whether `path` is `at` or under it.
bool at_or_under(const std::string &path, const std::string &at) {
  return path.size() >= at.size() && path.compare(0, at.size(), at) == 0 &&
         (path.size() == at.size() || path[at.size()] == '.');
}

std::size_t tree_bytes(const ServerLink::Tree &t) {
  std::size_t n = t.text.size();
  for (const auto &[name, child] : t.children)
    n += name.size() + 1 + tree_bytes(child);
  return n;
}

} // namespace

std::size_t ServerLink::size_of(const std::string &path, const Tree &tree) {
  return path.size() + tree_bytes(tree);
}

void ServerLink::enqueue(Write w) {
  if (w.tree) {
    // A tree replaces its whole subtree: what is queued at or under it would be overwritten or
    // removed anyway.
    for (auto it = queue_.begin(); it != queue_.end();) {
      if (at_or_under(it->path, w.path)) {
        if (!it->tree)
          sets_.erase(it->path);
        it = queue_.erase(it);
      } else {
        ++it;
      }
    }
  } else if (const auto it = sets_.find(w.path); it != sets_.end()) {
    queue_.erase(it->second); // the path's last write is the one that counts, and its place
    sets_.erase(it);
  }
  const bool is_set = !w.tree;
  const std::string path = w.path;
  queue_.push_back(std::move(w));
  if (is_set)
    sets_[path] = std::prev(queue_.end());
}

void ServerLink::set(std::string path, std::string text) {
  if (path.empty())
    return;
  Write w;
  w.bytes = path.size() + text.size();
  w.path = std::move(path);
  w.text = std::move(text);
  std::lock_guard<std::mutex> lk(mu_);
  enqueue(std::move(w));
}

void ServerLink::set_tree(std::string path, Tree tree) {
  if (path.empty())
    return;
  Write w;
  w.tree = true;
  w.bytes = size_of(path, tree);
  w.path = std::move(path);
  w.value = std::move(tree);
  std::lock_guard<std::mutex> lk(mu_);
  enqueue(std::move(w));
}

std::size_t ServerLink::pending() const {
  std::lock_guard<std::mutex> lk(mu_);
  return queue_.size();
}

// As (state-set-tree) writes a value (state_exec/intrinsics.cpp): a node's text only when it
// changes, then the children the value does not name removed, then the ones it does, each in turn.
// The caller holds `node`'s chain; each call holds its own node while it writes the children.
void ServerLink::write_tree(cvc::state &node, const Tree &tree) {
  if (node.value() != tree.text)
    node.value(tree.text);
  const std::vector<std::string> names = node.childNames();
  if (!names.empty()) {
    std::unordered_set<std::string> keep;
    for (const auto &[name, child] : tree.children)
      keep.insert(name);
    for (const std::string &name : names)
      if (!keep.count(name))
        node.removeChild(name);
  }
  for (const auto &[name, child] : tree.children) {
    const cvc::state::state_ptr c = node.sharedChild(name);
    write_tree(*c, child);
  }
}

void ServerLink::apply_one(cvc::state &root, const Write &w) {
  // The node's ancestors, held while its write walks up them (released leaf first).
  cvc::pin_vector chain;
  const cvc::state::state_ptr node = root.sharedChild(w.path, &chain);
  if (!node)
    return;
  if (w.tree)
    write_tree(*node, w.value);
  else if (node->value() != w.text)
    node->value(w.text);
}

ServerLink::Applied ServerLink::apply(cvc::state &root, std::size_t budget) {
  std::vector<Write> now;
  Applied a;
  {
    std::lock_guard<std::mutex> lk(mu_);
    while (!queue_.empty()) {
      const Write &front = queue_.front();
      if (!now.empty() && budget > 0 && a.bytes + front.bytes > budget)
        break;
      a.bytes += front.bytes;
      if (!front.tree)
        sets_.erase(front.path);
      now.push_back(std::move(queue_.front()));
      queue_.pop_front();
    }
    a.left = queue_.size();
  }
  // Written without the lock, so the script thread queues on while the relay writes.
  for (const Write &w : now)
    apply_one(root, w);
  a.writes = now.size();
  return a;
}

void ServerLink::post(Event event) {
  {
    std::lock_guard<std::mutex> lk(event_mu_);
    if (closed_)
      return;
    events_.push_back(std::move(event));
  }
  event_cv_.notify_all();
}

void ServerLink::close() {
  {
    std::lock_guard<std::mutex> lk(event_mu_);
    closed_ = true;
  }
  event_cv_.notify_all();
  // Nothing will answer now: whatever waits hears that the program has ended.
  std::unordered_map<std::uint64_t, Done> waits;
  {
    std::lock_guard<std::mutex> lk(wait_mu_);
    waits.swap(waits_);
  }
  for (auto &[id, done] : waits)
    done(false, cvc::state_exec::value_t(), "the Python program has ended");
}

bool ServerLink::next(Event &event) {
  std::unique_lock<std::mutex> lk(event_mu_);
  event_cv_.wait(lk, [this] { return closed_ || !events_.empty(); });
  if (events_.empty())
    return false;
  event = std::move(events_.front());
  events_.pop_front();
  return true;
}

void ServerLink::reply(Reply r) {
  std::lock_guard<std::mutex> lk(replies_mu_);
  replies_.push_back(std::move(r));
}

std::vector<ServerLink::Reply> ServerLink::take_replies() {
  std::lock_guard<std::mutex> lk(replies_mu_);
  std::vector<Reply> out;
  out.swap(replies_);
  return out;
}

std::uint64_t ServerLink::await_answer(Done done) {
  std::lock_guard<std::mutex> lk(wait_mu_);
  const std::uint64_t id = next_wait_++;
  waits_.emplace(id, std::move(done));
  return id;
}

bool ServerLink::answer(std::uint64_t id, bool ok, const cvc::state_exec::value_t &value,
                        const std::string &error) {
  Done done;
  {
    std::lock_guard<std::mutex> lk(wait_mu_);
    const auto it = waits_.find(id);
    if (it == waits_.end())
      return false;
    done = std::move(it->second);
    waits_.erase(it);
  }
  done(ok, value, error); // outside the lock: it may settle a call, which may wake its process
  return true;
}

bool ServerLink::request(std::string kind, cvc::state_exec::value_t value,
                         std::chrono::milliseconds timeout, cvc::state_exec::value_t &out,
                         std::string &error) {
  struct waiter {
    std::mutex mu;
    std::condition_variable cv;
    bool done = false, ok = false;
    cvc::state_exec::value_t value;
    std::string error;
  };
  auto w = std::make_shared<waiter>();
  const std::uint64_t id =
      await_answer([w](bool ok, const cvc::state_exec::value_t &v, const std::string &e) {
        std::lock_guard<std::mutex> lk(w->mu);
        w->ok = ok;
        w->value = v;
        w->error = e;
        w->done = true;
        w->cv.notify_all();
      });
  post(Event{std::move(kind), id, std::move(value)});
  std::unique_lock<std::mutex> lk(w->mu);
  if (!w->cv.wait_for(lk, timeout, [&] { return w->done; })) {
    lk.unlock();
    forget(id);
    error = "no answer in " + std::to_string(timeout.count()) + " ms";
    return false;
  }
  out = w->value;
  error = w->error;
  return w->ok;
}

void ServerLink::disconnect(std::string principal) {
  std::lock_guard<std::mutex> lk(disconnects_mu_);
  disconnects_.push_back(std::move(principal));
}

std::vector<std::string> ServerLink::take_disconnects() {
  std::lock_guard<std::mutex> lk(disconnects_mu_);
  std::vector<std::string> out;
  out.swap(disconnects_);
  return out;
}

void ServerLink::set_scope(const std::string &principal, std::vector<std::string> prefixes) {
  std::lock_guard<std::mutex> lk(scopes_mu_);
  auto [it, fresh] = scopes_.try_emplace(principal);
  if (!fresh && it->second == prefixes)
    return;
  it->second = std::move(prefixes);
  if (!fresh && std::find(scope_refreshes_.begin(), scope_refreshes_.end(), principal) ==
                    scope_refreshes_.end())
    scope_refreshes_.push_back(principal);
}

bool ServerLink::scope_of(const std::string &principal, std::vector<std::string> &prefixes) const {
  std::lock_guard<std::mutex> lk(scopes_mu_);
  const auto it = scopes_.find(principal);
  if (it == scopes_.end())
    return false;
  prefixes = it->second;
  return true;
}

std::vector<std::string> ServerLink::take_scope_refreshes() {
  std::lock_guard<std::mutex> lk(scopes_mu_);
  std::vector<std::string> out;
  out.swap(scope_refreshes_);
  return out;
}

void ServerLink::forget(std::uint64_t id) {
  std::lock_guard<std::mutex> lk(wait_mu_);
  waits_.erase(id);
}

void ServerLink::ready(bool ok, std::string why) {
  {
    std::lock_guard<std::mutex> lk(start_mu_);
    if (start_ != Start::waiting)
      return;
    start_ = ok ? Start::serving : Start::failed;
    why_ = std::move(why);
  }
  start_cv_.notify_all();
}

void ServerLink::finish(int status) {
  {
    std::lock_guard<std::mutex> lk(start_mu_);
    finished_ = true;
    status_ = status;
    if (start_ == Start::waiting)
      start_ = Start::finished;
  }
  start_cv_.notify_all();
}

ServerLink::Start ServerLink::wait_start(std::chrono::milliseconds timeout) {
  std::unique_lock<std::mutex> lk(start_mu_);
  start_cv_.wait_for(lk, timeout, [this] { return start_ != Start::waiting; });
  return start_;
}

bool ServerLink::wait_finished(std::chrono::milliseconds timeout) {
  std::unique_lock<std::mutex> lk(start_mu_);
  return start_cv_.wait_for(lk, timeout, [this] { return finished_; });
}

ServerLink::Start ServerLink::start() const {
  std::lock_guard<std::mutex> lk(start_mu_);
  return start_;
}

bool ServerLink::finished() const {
  std::lock_guard<std::mutex> lk(start_mu_);
  return finished_;
}

int ServerLink::status() const {
  std::lock_guard<std::mutex> lk(start_mu_);
  return status_;
}

std::string ServerLink::why() const {
  std::lock_guard<std::mutex> lk(start_mu_);
  return why_;
}

} // namespace python
} // namespace ariadne
} // namespace cvc
