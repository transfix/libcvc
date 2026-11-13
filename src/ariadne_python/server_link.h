// server_link.h — what a server host (arisrvx) and its script thread hand each other
// (docs/roadmap/ARIADNE_SCRIPTING_HOSTS.md §6.5.1). The relay never runs Python, never takes the
// GIL and never waits on the script; the script never writes the relay's tree itself:
//
//   - Python to relay: state writes. pycvc.ariadne's host.set, set_tree and set_series queue them
//     here, on the script thread, and the relay applies them on its next room step (apply()):
//     coalesced per path, at most a budget of bytes a step, the rest left for the next step in
//     order.
//   - Relay to Python: events (a stop, a client's call, the server document's block and its
//     exports' calls). post() from any thread; the script's waker thread takes each one (next(),
//     which blocks without the GIL) and hands it to the asyncio loop. An event's values are DSL
//     values (value_t), which Python receives through pycvc's conversion, so they keep their
//     types; nothing here is JSON.
//   - Python's answers: replies to clients' calls, which the relay sends on its next room step,
//     and answers to work the relay waits on, which settle it (a server document's export).
//   - Startup: the program says when it serves (its ASGI apps have started) or that its startup
//     failed (ready()), and the script thread when the program has ended (finish()). The relay
//     waits for one of them before it listens (wait_start()).
//
// No Python here: the engine (python_engine.cpp) is the side that holds the GIL.
#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <cvc/state/state_exec/types.h>
#include <deque>
#include <functional>
#include <list>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace cvc {
class state;
namespace ariadne {
namespace python {

class ServerLink {
public:
  // A subtree as set_tree writes it, as the DSL's (state-set-tree) does: the node's text, and a
  // list's or dict's children by name (a list's are 0..n-1, its text its length). A node written
  // as a scalar loses every child it had; a list or dict loses the children it does not name.
  struct Tree {
    std::string text;
    bool compound = false; // a list or dict: `children` names what stays
    std::vector<std::pair<std::string, Tree>> children;
  };

  // ── Python to relay ──────────────────────────────────────────────────────────────────────────
  // Queue a write; any thread. A write replaces a queued one it makes moot (the same path's text
  // for set(); everything at or under the path for set_tree()) and goes to the back of the queue,
  // so the queue keeps the order of each path's last write: a section's fields, then its rev.
  void set(std::string path, std::string text);
  void set_tree(std::string path, Tree tree);

  struct Applied {
    std::size_t writes = 0; // applied now
    std::size_t bytes = 0;  // their paths and texts
    std::size_t left = 0;   // still queued
  };
  // On the relay's thread: apply queued writes to `root`, in order, until the next one would take
  // the step past `budget` bytes (0: no budget). The first one always goes, whatever its size, so a
  // write larger than the budget still lands.
  Applied apply(cvc::state &root, std::size_t budget);
  std::size_t pending() const;

  // ── relay to Python ──────────────────────────────────────────────────────────────────────────
  // An event: its kind ("shutdown", "call" a client's call, "load_block" the server document's
  // python: block, "doc_call" one of the document's exports), the id of the answer the relay
  // waits for (0: none), and its value (nil, or a dict of the event's fields).
  struct Event {
    std::string kind;
    std::uint64_t id = 0;
    cvc::state_exec::value_t value;
  };
  void post(Event event);
  void post(std::string kind) { post(Event{std::move(kind), 0, {}}); }
  // next() returns false once the queue is closed and empty: the waker thread ends.
  void close();
  // Block until an event, then true with it; false once closed.
  bool next(Event &event);

  // ── Python's answers ─────────────────────────────────────────────────────────────────────────
  // To a client's call (room_call_relay): queued on the script thread, taken by the relay on its
  // next room step, in order.
  struct Reply {
    enum class Kind { result, progress, error } kind = Kind::result;
    std::string pid, id;            // the caller and its call
    cvc::state_exec::value_t value; // result: the value; error: the detail
    std::string code, message;      // error: its code and message; progress: the message
    double fraction = 0;            // progress
  };
  void reply(Reply r);
  std::vector<Reply> take_replies();
  // To the relay itself, for work it waits on (the server document's block, its exports' calls):
  // `done` runs once, with the value, or with ok false and the error's text, on the thread that
  // answers (the script thread), or with false at close(). Returns the id the event names.
  using Done =
      std::function<void(bool ok, const cvc::state_exec::value_t &value, const std::string &error)>;
  std::uint64_t await_answer(Done done);
  // The answer to `id`; false when nothing waits for it (answered, or given up).
  bool answer(std::uint64_t id, bool ok, const cvc::state_exec::value_t &value,
              const std::string &error = std::string());
  // Post the event `kind` with `value` and wait, on this thread, at most `timeout` for its answer.
  // Ok: true and the value in `out`. Not: false and why in `error` (Python's error, the program
  // having ended, or the wait running out, which forgets the event's answer).
  bool request(std::string kind, cvc::state_exec::value_t value, std::chrono::milliseconds timeout,
               cvc::state_exec::value_t &out, std::string &error);

  // ── who connects (S5) ────────────────────────────────────────────────────────────────────────
  // The program has an authenticate hook (host.authenticate): the relay asks it about each
  // connection (an "authenticate" event) instead of checking the room token. Said before it serves.
  void set_authenticates(bool on) { authenticates_.store(on); }
  bool authenticates() const { return authenticates_.load(); }
  // host.disconnect(principal): queued on the script thread, taken by the relay on its next step,
  // which closes that principal's connections.
  void disconnect(std::string principal);
  std::vector<std::string> take_disconnects();
  // Give up on `id` (its caller went): its answer, when it comes, is dropped.
  void forget(std::uint64_t id);

  // ── what each principal reads (S6) ───────────────────────────────────────────────────────────
  // The program has a read_scope hook (host.read_scope): each principal's connections are sent
  // only the paths under its prefixes. Said before it serves.
  void set_read_scopes(bool on) { read_scopes_.store(on); }
  bool read_scopes() const { return read_scopes_.load(); }
  // A principal's prefixes, as the hook answered: at its authentication (before the verdict, so
  // the relay has them when it admits the connection) and on host.refresh_scope. A change for a
  // principal it had prefixes for already is queued for the relay's next step, which re-scopes
  // that principal's connections.
  void set_scope(const std::string &principal, std::vector<std::string> prefixes);
  // The principal's prefixes; false when the hook has not answered for it.
  bool scope_of(const std::string &principal, std::vector<std::string> &prefixes) const;
  std::vector<std::string> take_scope_refreshes();

  // Whether Python can convert DSL values (pycvc is beside the host): the program says so as it
  // starts. Without it the relay answers clients' calls `unavailable` itself, and a server
  // document's python: block fails its load.
  void set_converts(bool ok) { converts_.store(ok); }
  bool converts() const { return converts_.load(); }

  // ── startup ──────────────────────────────────────────────────────────────────────────────────
  enum class Start { waiting, serving, failed, finished };
  // The program: it serves (ok), or its startup failed with `why`. The first call counts.
  void ready(bool ok, std::string why);
  // The script thread, when the program has ended, with its exit status.
  void finish(int status);
  // Wait at most `timeout` for the program to serve, fail or end; where it is now.
  Start wait_start(std::chrono::milliseconds timeout);
  // Wait at most `timeout` for the program to end; whether it has.
  bool wait_finished(std::chrono::milliseconds timeout);
  Start start() const;
  bool finished() const;
  int status() const; // the program's exit status, once finished
  std::string why() const;

  // The bytes a write counts against the budget: its path's and its texts' (a tree's every node).
  static std::size_t size_of(const std::string &path, const Tree &tree);

private:
  struct Write {
    std::string path;
    bool tree = false;
    std::string text; // set()
    Tree value;       // set_tree()
    std::size_t bytes = 0;
  };
  void enqueue(Write w); // with mu_ held
  static void write_tree(cvc::state &node, const Tree &tree);
  static void apply_one(cvc::state &root, const Write &w);

  mutable std::mutex mu_;
  std::list<Write> queue_;
  std::unordered_map<std::string, std::list<Write>::iterator> sets_; // set()s queued, by path

  mutable std::mutex event_mu_;
  std::condition_variable event_cv_;
  std::deque<Event> events_;
  bool closed_ = false;

  mutable std::mutex replies_mu_;
  std::vector<Reply> replies_;

  mutable std::mutex wait_mu_;
  std::uint64_t next_wait_ = 1;
  std::unordered_map<std::uint64_t, Done> waits_;

  std::atomic<bool> converts_{false};
  std::atomic<bool> authenticates_{false};

  mutable std::mutex disconnects_mu_;
  std::vector<std::string> disconnects_;

  std::atomic<bool> read_scopes_{false};
  mutable std::mutex scopes_mu_;
  std::unordered_map<std::string, std::vector<std::string>> scopes_; // by principal
  std::vector<std::string> scope_refreshes_;

  mutable std::mutex start_mu_;
  std::condition_variable start_cv_;
  Start start_ = Start::waiting;
  bool finished_ = false;
  int status_ = 0;
  std::string why_;
};

} // namespace python
} // namespace ariadne
} // namespace cvc
