#include "closure_natives.h"

#include <algorithm>
#include <charconv>
#include <chrono>
#include <cmath>
#include <cvc/state/state.h>
#include <cvc/state/state_exec/data_job.h>
#include <cvc/state/state_exec/generator.h>
#include <cvc/state/state_exec/heap.h>
#include <cvc/state/state_exec/process.h>
#include <cvc/state/state_exec/scheduler.h>
#include <deque>
#include <queue>
#include <type_traits>
#include <unordered_map>
#include <vector>

namespace cvc::state_exec {

namespace {

// ---------------------------------------------------------------------------
// The object graph
// ---------------------------------------------------------------------------

// The reference-counted objects values point at. Strings, numbers and symbols live inside their
// value_t and are sized with whatever holds it. A native function is opaque, except the defclass
// natives (closure_natives.h), whose captures are edges. A frozen scope (the shared stdlib) is
// left out: it is never garbage and belongs to no process.
enum class node_kind : std::uint8_t { env, closure, list, dict, generator, data };

using dict_vector = std::vector<std::pair<std::string, value_t>>;

template <class F> void env_edge(const environment_ptr &e, F &f) {
  if (e && !e->frozen)
    f(node_kind::env, e);
}

// Call f(kind, shared_ptr) for each object `v` holds directly.
template <class F> void value_edges(const value_t &v, F &f) {
  std::visit(
      [&](const auto &x) {
        using T = std::decay_t<decltype(x)>;
        if constexpr (std::is_same_v<T, list_ptr>) {
          if (x)
            f(node_kind::list, x);
        } else if constexpr (std::is_same_v<T, dict_ptr>) {
          if (x)
            f(node_kind::dict, x);
        } else if constexpr (std::is_same_v<T, closure_ptr>) {
          if (x)
            f(node_kind::closure, x);
        } else if constexpr (std::is_same_v<T, generator_ptr>) {
          if (x)
            f(node_kind::generator, x);
        } else if constexpr (std::is_same_v<T, data_object_ptr>) {
          if (x)
            f(node_kind::data, x);
        } else if constexpr (std::is_same_v<T, native_fn>) {
          if (const auto *c = x.template target<detail::class_constructor>()) {
            if (c->cls_dict)
              f(node_kind::dict, c->cls_dict);
          } else if (const auto *i = x.template target<detail::class_inherited_method>()) {
            if (i->super_cls)
              f(node_kind::dict, i->super_cls);
          }
        }
      },
      v.v);
}

template <class F> void frame_edges(const eval_frame &fr, F &f) {
  value_edges(fr.expr, f);
  env_edge(fr.env, f);
  for (const value_t &v : fr.results)
    value_edges(v, f);
  for (const value_t &v : fr.extra_vals)
    value_edges(v, f);
  for (const std::vector<value_t> &l : fr.extra_lists)
    for (const value_t &v : l)
      value_edges(v, f);
  env_edge(fr.extra_env, f);
}

template <class F> void state_edges(const evaluator_state &st, F &f) {
  for (const eval_frame &fr : st.stack)
    frame_edges(fr, f);
  value_edges(st.result, f);
  value_edges(st.root_expr, f);
  env_edge(st.global_env, f);
  for (const auto &[name, macro] : st.user_macros)
    value_edges(macro.second, f);
}

// Call f(kind, shared_ptr) for each object the node `p` (of kind `k`) holds directly.
template <class F> void node_edges(node_kind k, const void *p, F &f) {
  switch (k) {
  case node_kind::env: {
    const auto &e = *static_cast<const environment *>(p);
    for (const auto &[name, v] : e.bindings)
      value_edges(v, f);
    env_edge(e.outer, f);
    break;
  }
  case node_kind::closure: {
    const auto &c = *static_cast<const closure *>(p);
    env_edge(c.env_snapshot, f);
    for (const value_t &v : c.body)
      value_edges(v, f);
    break;
  }
  case node_kind::list:
    for (const value_t &v : *static_cast<const std::vector<value_t> *>(p))
      value_edges(v, f);
    break;
  case node_kind::dict:
    for (const auto &[key, v] : *static_cast<const dict_vector *>(p))
      value_edges(v, f);
    break;
  case node_kind::generator: {
    const auto &g = *static_cast<const generator *>(p);
    env_edge(g.evaluator.global_env(), f);
    state_edges(g.state, f);
    break;
  }
  case node_kind::data:
    break;
  }
}

// std::queue keeps its container protected; this reads it in place.
template <class T> const std::deque<T> &queue_items(const std::queue<T> &q) {
  struct access : std::queue<T> {
    static const std::deque<T> &items(const std::queue<T> &q) { return q.*&access::c; }
  };
  return access::items(q);
}

// Every reference a process holds directly: its roots.
template <class F> void process_edges(const process &p, F &f) {
  state_edges(p.state, f);
  for (const value_t &v : queue_items(p.inbox))
    value_edges(v, f);
  value_edges(p.exit_code, f);
  for (const auto &[name, v] : p.signal_handlers)
    value_edges(v, f);
  for (const eval_frame &fr : p.saved_stack)
    frame_edges(fr, f);
  value_edges(p.saved_result, f);
  for (const auto &[id, w] : p.watch_handlers)
    value_edges(w.handler, f);
}

// ---------------------------------------------------------------------------
// Sizes: an estimate of the heap bytes each object owns (malloc's own overhead not counted)
// ---------------------------------------------------------------------------

const std::size_t kSso = std::string().capacity();        // a string's inline capacity
constexpr std::size_t kControlBlock = 4 * sizeof(void *); // a shared_ptr's counts and vtable

std::size_t string_heap(const std::string &s) { return s.capacity() > kSso ? s.capacity() + 1 : 0; }

std::size_t inline_heap(const value_t &v) {
  if (const auto *s = std::get_if<std::string>(&v.v))
    return string_heap(*s);
  if (const auto *b = std::get_if<bytes_value>(&v.v))
    return string_heap(b->data);
  if (const auto *y = std::get_if<symbol>(&v.v))
    return string_heap(y->name);
  return 0;
}

std::size_t values_bytes(const std::vector<value_t> &vs) {
  std::size_t n = vs.capacity() * sizeof(value_t);
  for (const value_t &v : vs)
    n += inline_heap(v);
  return n;
}

std::size_t frame_bytes(const eval_frame &fr) {
  std::size_t n = inline_heap(fr.expr) + values_bytes(fr.results) + values_bytes(fr.extra_vals);
  n += fr.extra_strs.capacity() * sizeof(std::string);
  for (const std::string &s : fr.extra_strs)
    n += string_heap(s);
  n += fr.extra_lists.capacity() * sizeof(std::vector<value_t>);
  for (const std::vector<value_t> &l : fr.extra_lists)
    n += values_bytes(l);
  return n;
}

std::size_t state_bytes(const evaluator_state &st) {
  std::size_t n = st.stack.capacity() * sizeof(eval_frame);
  for (const eval_frame &fr : st.stack)
    n += frame_bytes(fr);
  return n + inline_heap(st.result);
}

std::size_t node_bytes(node_kind k, const void *p) {
  switch (k) {
  case node_kind::env: {
    const auto &e = *static_cast<const environment *>(p);
    std::size_t n =
        sizeof(environment) + kControlBlock + e.bindings.bucket_count() * sizeof(void *);
    n += e.bindings.size() *
         (sizeof(std::pair<const std::string, value_t>) + 2 * sizeof(void *)); // the map's nodes
    for (const auto &[name, v] : e.bindings)
      n += string_heap(name) + inline_heap(v);
    return n;
  }
  case node_kind::closure: {
    const auto &c = *static_cast<const closure *>(p);
    std::size_t n = sizeof(closure) + kControlBlock + c.params.capacity() * sizeof(symbol);
    for (const symbol &s : c.params)
      n += string_heap(s.name);
    return n + values_bytes(c.body);
  }
  case node_kind::list:
    return sizeof(std::vector<value_t>) + kControlBlock +
           values_bytes(*static_cast<const std::vector<value_t> *>(p));
  case node_kind::dict: {
    const auto &d = *static_cast<const dict_vector *>(p);
    std::size_t n =
        sizeof(dict_vector) + kControlBlock + d.capacity() * sizeof(dict_vector::value_type);
    for (const auto &[key, v] : d)
      n += string_heap(key) + inline_heap(v);
    return n;
  }
  case node_kind::generator:
    return sizeof(generator) + kControlBlock +
           state_bytes(static_cast<const generator *>(p)->state);
  case node_kind::data:
    return sizeof(data_object) + kControlBlock +
           string_heap(static_cast<const data_object *>(p)->type_name);
  }
  return 0;
}

// What a process holds in its own structures (frames, inbox, handlers), not in shared objects.
std::size_t process_own_bytes(const process &p) {
  std::size_t n = state_bytes(p.state) + inline_heap(p.exit_code) + inline_heap(p.saved_result);
  const std::deque<value_t> &inbox = queue_items(p.inbox);
  n += inbox.size() * sizeof(value_t);
  for (const value_t &v : inbox)
    n += inline_heap(v);
  n += p.saved_stack.capacity() * sizeof(eval_frame);
  for (const eval_frame &fr : p.saved_stack)
    n += frame_bytes(fr);
  return n;
}

// ---------------------------------------------------------------------------
// Measuring a share
// ---------------------------------------------------------------------------

// A process's share of what it reaches: each node's size x the fraction of its owners that are the
// process's. A root reference (a frame, a binding the process holds) gives 1 / owners; a node
// passes its fraction on to the nodes it holds, each edge giving fraction / (child's owners). The
// fractions are added in topological order (Kahn) once the discovery pass has counted each node's
// incoming references from this process; nodes left waiting (a cycle, or a graph that changed
// between slices) are taken as they are. The job holds every node it found, and subtracts that
// one reference when it reads a node's owner count.
class heap_share_job final : public data_job {
public:
  explicit heap_share_job(const process &p) : own_(static_cast<double>(process_own_bytes(p))) {
    // The roots, whole: the references the process holds directly.
    auto root = [this](node_kind k, const auto &sp) { add(k, sp, /*from_root=*/true); };
    process_edges(p, root);
  }

  bool step(std::size_t budget) override {
    std::size_t visited = 0;
    const auto more = [&] { return budget == 0 || visited < budget; };
    while (phase_ == phase::discover && more()) {
      if (cursor_ == graph_.size()) {
        start_propagating();
        break;
      }
      const std::size_t i = cursor_++;
      auto edge = [this](node_kind k, const auto &sp) { add(k, sp, /*from_root=*/false); };
      node_edges(graph_[i].kind, graph_[i].hold.get(), edge);
      ++visited;
      ++nodes_; // data_job: nodes visited
    }
    while (phase_ == phase::propagate && more()) {
      if (ready_.empty()) {
        while (cursor_ < graph_.size() && graph_[cursor_].done)
          ++cursor_;
        if (cursor_ == graph_.size()) {
          phase_ = phase::done;
          break;
        }
        ready_.push_back(cursor_); // waiting on references that never come: take it as it is
      }
      const std::size_t i = ready_.back();
      ready_.pop_back();
      if (graph_[i].done)
        continue;
      graph_[i].done = true;
      const double frac = std::min(1.0, graph_[i].frac);
      share_ += frac * static_cast<double>(node_bytes(graph_[i].kind, graph_[i].hold.get()));
      auto edge = [&](node_kind, const auto &sp) {
        auto it = index_.find(sp.get());
        if (it == index_.end())
          return; // appeared since discovery
        node &c = graph_[it->second];
        if (c.done)
          return;
        c.frac += frac / c.owners;
        if (c.pending > 0 && --c.pending == 0)
          ready_.push_back(it->second);
      };
      node_edges(graph_[i].kind, graph_[i].hold.get(), edge);
      ++visited;
      ++nodes_; // data_job: nodes visited
    }
    return phase_ == phase::done;
  }

  value_t result() override { return value_t{static_cast<int64_t>(std::llround(share_ + own_))}; }

private:
  struct node {
    node_kind kind;
    std::shared_ptr<void> hold;
    double owners;           // the node's use count when found, less this job's reference
    std::size_t in = 0;      // references to it from the process's roots and reachable nodes
    std::size_t roots = 0;   // of which from the roots
    std::size_t pending = 0; // references whose fraction has not arrived yet
    double frac = 0.0;
    bool done = false;
  };
  enum class phase { discover, propagate, done };

  template <class T> void add(node_kind k, const std::shared_ptr<T> &sp, bool from_root) {
    auto [it, inserted] = index_.try_emplace(sp.get(), graph_.size());
    if (inserted) {
      const long owners = sp.use_count(); // read before this job holds it
      graph_.push_back(
          node{k, std::shared_ptr<void>(sp), static_cast<double>(std::max<long>(1, owners))});
    }
    node &n = graph_[it->second];
    ++n.in;
    if (from_root)
      ++n.roots;
  }

  void start_propagating() {
    for (std::size_t i = 0; i < graph_.size(); ++i) {
      node &n = graph_[i];
      n.frac = static_cast<double>(n.roots) / n.owners;
      n.pending = n.in - n.roots;
      if (n.pending == 0)
        ready_.push_back(i);
    }
    std::reverse(ready_.begin(), ready_.end()); // roots first, in discovery order
    cursor_ = 0;
    phase_ = phase::propagate;
  }

  std::vector<node> graph_;
  std::unordered_map<const void *, std::size_t> index_;
  std::vector<std::size_t> ready_;
  phase phase_ = phase::discover;
  std::size_t cursor_ = 0;
  double share_ = 0.0;
  double own_;
};

// ---------------------------------------------------------------------------
// The cycle registry (per thread)
// ---------------------------------------------------------------------------

struct tracked_object {
  node_kind kind;
  std::weak_ptr<void> ptr;
  const void *raw;
};

struct cycle_registry {
  std::vector<tracked_object> objects;
  std::unordered_map<const void *, std::size_t> index; // object -> its entry in `objects`
  std::size_t since_last = 0;                          // newly tracked since the last collection
  std::size_t survivors = 0;     // tracked objects alive after the last collection
  std::size_t last_examined = 0; // objects the last collection walked
  bool exit_pending = false;     // a process ended since the last collection
  std::chrono::steady_clock::time_point last_collection{};

  void reindex() {
    index.clear();
    for (std::size_t i = 0; i < objects.size(); ++i)
      index.emplace(objects[i].raw, i);
  }
};

cycle_registry &registry() {
  thread_local cycle_registry r;
  return r;
}

constexpr std::size_t kMinCollectionGrowth = 256;

template <class T> void track(node_kind k, const std::shared_ptr<T> &p) {
  if (!p)
    return;
  cycle_registry &r = registry();
  if (auto it = r.index.find(p.get()); it != r.index.end()) {
    tracked_object &t = r.objects[it->second];
    if (!t.ptr.owner_before(p) && !p.owner_before(t.ptr))
      return; // already tracked (a closure made in a loop over one scope, a list appended to)
    t = tracked_object{k, std::weak_ptr<void>(p), p.get()}; // a dead object's address, reused
    ++r.since_last;
    return;
  }
  if (r.objects.size() >= 2 * (r.survivors + r.since_last) + 1024) {
    // Most entries died young: drop them before their (weak) control blocks pile up.
    std::erase_if(r.objects, [](const tracked_object &t) { return t.ptr.expired(); });
    r.reindex();
  }
  r.index.emplace(p.get(), r.objects.size());
  r.objects.push_back(tracked_object{k, std::weak_ptr<void>(p), p.get()});
  ++r.since_last;
}

void clear_garbage(node_kind k, void *p) {
  switch (k) {
  case node_kind::env: {
    auto &e = *static_cast<environment *>(p);
    e.bindings.clear();
    e.outer.reset();
    break;
  }
  case node_kind::closure: {
    auto &c = *static_cast<closure *>(p);
    c.env_snapshot.reset();
    c.body.clear();
    break;
  }
  case node_kind::list:
    static_cast<std::vector<value_t> *>(p)->clear();
    break;
  case node_kind::dict:
    static_cast<dict_vector *>(p)->clear();
    break;
  case node_kind::generator: {
    auto &g = *static_cast<generator *>(p);
    g.state = evaluator_state{};
    g.native_step = nullptr;
    g.exhausted = true;
    break;
  }
  case node_kind::data:
    break;
  }
}

// A size_t setting from the tree; nullopt when missing, empty or not a plain decimal number.
std::optional<std::size_t> read_setting(cvc::state &root, const std::string &path) {
  cvc::state *node = root.findDescendant(path);
  if (!node)
    return std::nullopt;
  const std::string v = node->value();
  std::size_t out = 0;
  const auto [end, ec] = std::from_chars(v.data(), v.data() + v.size(), out);
  if (v.empty() || ec != std::errc() || end != v.data() + v.size())
    return std::nullopt;
  return out;
}

} // namespace

// ---------------------------------------------------------------------------
// Public interface
// ---------------------------------------------------------------------------

heap_options heap_options_from_state(cvc::state &root, const std::string &scheduler_id,
                                     const heap_options &base) {
  heap_options opts = base;
  const std::string suffix = ".limits.heap_check_steps";
  if (auto v = read_setting(root, std::string(scheduler::defaults_prefix) + suffix))
    opts.check_steps = *v;
  if (!scheduler_id.empty())
    if (auto v =
            read_setting(root, std::string(scheduler::sched_prefix) + "." + scheduler_id + suffix))
      opts.check_steps = *v;
  return opts;
}

std::unique_ptr<data_job> make_heap_share_job(const process &proc) {
  return std::make_unique<heap_share_job>(proc);
}

std::size_t measure_heap_share(const process &proc) {
  heap_share_job job(proc);
  while (!job.step(0)) {
  }
  return static_cast<std::size_t>(std::get<int64_t>(job.result().v));
}

std::size_t heap_remeasure_bytes(std::size_t last_measured) {
  return std::max<std::size_t>(std::size_t{64} << 10, last_measured / 2);
}

void heap_after_step(process &proc, std::size_t allocated, uint64_t state_bytes_used) {
  evaluator_state &st = proc.state;
  if (st.heap_measured) {
    proc.heap_bytes = *st.heap_measured;
    st.heap_measured.reset();
    proc.heap_measuring = false;
    proc.heap_alloc_bytes = 0;
    proc.heap_steps = 0;
    ++proc.heap_measurements;
    return;
  }
  if (proc.max_memory == 0)
    return;
  proc.heap_alloc_bytes += allocated;
  ++proc.heap_steps;
  if (proc.heap_measuring) {
    if (!st.data_work)
      proc.heap_measuring = false; // dropped with the state it ran in (a signal, a restore)
    return;
  }
  if (st.done || st.reading || st.data_work || st.stack.empty())
    return;
  if (!proc.heap_measure_due(state_bytes_used))
    return;
  // It waits on no frame: evaluation resumes at the top of the stack, as it was, so `base` and
  // `frame_args` are the whole stack and the top frame's values (a snapshot keeps all of them).
  auto work = std::make_shared<pending_data_work>();
  work->job = make_heap_share_job(proc);
  work->what = pending_data_work::kind::heap_measure;
  work->base = st.stack.size();
  work->frame_args = st.stack.back().results.size();
  st.data_work = std::move(work);
  proc.heap_measuring = true;
}

void track_captured_env(const environment_ptr &env) {
  if (env && !env->frozen)
    track(node_kind::env, env);
}

void track_mutated(const list_ptr &list) { track(node_kind::list, list); }
void track_mutated(const dict_ptr &dict) { track(node_kind::dict, dict); }

cycle_stats collect_cycles() {
  cycle_stats stats;
  if (detail::data_step_scope::current())
    return stats; // inside an evaluator step: native code may hold raw pointers into the heap
  cycle_registry &r = registry();
  r.since_last = 0;
  r.exit_pending = false;
  r.last_collection = std::chrono::steady_clock::now();

  struct gc_node {
    node_kind kind;
    std::shared_ptr<void> hold;
    long owners;       // use count when found, less this collection's reference
    long internal = 0; // references to it from the nodes found
    bool live = false;
  };
  std::vector<gc_node> nodes;
  std::unordered_map<const void *, std::size_t> index;

  // The candidates: tracked objects still alive, once each.
  std::vector<tracked_object> alive;
  alive.reserve(r.objects.size());
  for (tracked_object &t : r.objects) {
    std::shared_ptr<void> p = t.ptr.lock();
    if (!p || !index.try_emplace(p.get(), nodes.size()).second)
      continue;
    const long owners = p.use_count() - 1; // less `p`
    nodes.push_back(gc_node{t.kind, std::move(p), owners});
    alive.push_back(std::move(t));
  }
  r.objects = std::move(alive);
  stats.candidates = nodes.size();

  // Everything they reach, and how many references each gets from the others.
  auto found = [&](node_kind k, const auto &sp) {
    auto [it, inserted] = index.try_emplace(sp.get(), nodes.size());
    if (inserted) {
      const long owners = sp.use_count(); // read before this collection holds it
      nodes.push_back(gc_node{k, std::shared_ptr<void>(sp), owners});
    }
    ++nodes[it->second].internal;
  };
  for (std::size_t i = 0; i < nodes.size(); ++i) {
    void *p = nodes[i].hold.get();
    node_edges(nodes[i].kind, p, found);
  }
  stats.examined = nodes.size();

  // Held from outside (more owners than references among the nodes): live, with all it reaches.
  std::vector<std::size_t> stack;
  for (std::size_t i = 0; i < nodes.size(); ++i)
    if (nodes[i].owners > nodes[i].internal) {
      nodes[i].live = true;
      stack.push_back(i);
    }
  auto reach = [&](node_kind, const auto &sp) {
    auto it = index.find(sp.get());
    if (it != index.end() && !nodes[it->second].live) {
      nodes[it->second].live = true;
      stack.push_back(it->second);
    }
  };
  while (!stack.empty()) {
    const std::size_t i = stack.back();
    stack.pop_back();
    node_edges(nodes[i].kind, nodes[i].hold.get(), reach);
  }

  // The rest is garbage: clear it (its parts stay held here, so nothing frees mid-loop), then
  // let go, and it frees.
  for (gc_node &n : nodes)
    if (!n.live) {
      clear_garbage(n.kind, n.hold.get());
      ++stats.freed;
    }
  nodes.clear();

  std::erase_if(r.objects, [](const tracked_object &t) { return t.ptr.expired(); });
  r.reindex();
  r.survivors = r.objects.size();
  r.last_examined = stats.examined;
  return stats;
}

bool cycle_collection_due() {
  const cycle_registry &r = registry();
  // As many new objects as survived, and at least an eighth of what the last collection walked,
  // so a collection's walk costs a bounded amount per object tracked since the one before.
  if (r.since_last >= std::max({kMinCollectionGrowth, r.survivors, r.last_examined / 8}))
    return true;
  return r.exit_pending && r.since_last > 0 &&
         std::chrono::steady_clock::now() - r.last_collection >= std::chrono::seconds(1);
}

void note_process_exit() { registry().exit_pending = true; }

std::size_t tracked_objects() { return registry().objects.size(); }

} // namespace cvc::state_exec
