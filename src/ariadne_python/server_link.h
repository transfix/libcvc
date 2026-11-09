// server_link.h — what a server host (arisrvx) and its script thread hand each other
// (docs/roadmap/ARIADNE_SCRIPTING_HOSTS.md §6.5.1). The relay never runs Python, never takes the
// GIL and never waits on the script; the script never writes the relay's tree itself:
//
//   - Python to relay: state writes. pycvc.ariadne's host.set, set_tree and set_series queue them
//     here, on the script thread, and the relay applies them on its next room step (apply()):
//     coalesced per path, at most a budget of bytes a step, the rest left for the next step in
//     order.
//   - Relay to Python: events ("shutdown"). post() from any thread; the script's waker thread
//     takes each one (next(), which blocks without the GIL) and hands it to the asyncio loop.
//   - Startup: the program says when it serves (its ASGI apps have started) or that its startup
//     failed (ready()), and the script thread when the program has ended (finish()). The relay
//     waits for one of them before it listens (wait_start()).
//
// No Python here: the engine (python_engine.cpp) is the side that holds the GIL.
#pragma once

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <deque>
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
  void post(std::string event);
  // next() returns false once the queue is closed and empty: the waker thread ends.
  void close();
  // Block until an event, then true with it; false once closed.
  bool next(std::string &event);

  // ── startup ──────────────────────────────────────────────────────────────────────────────────
  enum class Start { waiting, serving, failed, finished };
  // The program: it serves (ok), or its startup failed with `why`. The first call counts.
  void ready(bool ok, std::string why);
  // The script thread, when the program has ended, with its exit status.
  void finish(int status);
  // Wait at most `timeout` for the program to serve, fail or end; where it is now.
  Start wait_start(std::chrono::milliseconds timeout);
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
  std::deque<std::string> events_;
  bool closed_ = false;

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
