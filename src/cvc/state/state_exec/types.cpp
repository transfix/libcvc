#include <algorithm>
#include <cvc/state/state_exec/types.h>
#include <new>
#include <sstream>
#include <stdexcept>
#include <time.h>
#include <utility>

namespace cvc::state_exec {

// -- time budgets -------------------------------------------------------------

double thread_cpu_seconds() {
#if defined(CLOCK_THREAD_CPUTIME_ID) && !defined(_WIN32)
  static const bool has_cpu_clock = [] {
    timespec ts;
    return clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ts) == 0;
  }();
  if (has_cpu_clock) {
    timespec ts;
    clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ts);
    return double(ts.tv_sec) + 1e-9 * double(ts.tv_nsec);
  }
#endif
  return std::chrono::duration<double>(time_budget::clock::now().time_since_epoch()).count();
}

namespace {
time_budget::clock::time_point after(time_budget::clock::time_point t, double seconds) {
  return t + std::chrono::duration_cast<time_budget::clock::duration>(
                 std::chrono::duration<double>(seconds));
}
} // namespace

time_budget::time_budget(double seconds) : half_(seconds > 0.0 ? 0.5 * seconds : 0.0) {}

void time_budget::start() {
  running_ = true;
  t0_ = last_ = clock::now();
  gaps_ = 0.0;
  if (on_cpu_)
    c0_ = thread_cpu_seconds();
  next_ = after(t0_, on_cpu_ ? half_ - cpu_ : half_ - wall_);
}

void time_budget::gap(clock::time_point now, std::uint64_t gaps) {
  gaps_ +=
      std::min(std::chrono::duration<double>(now - last_).count(), double(gaps) * half_ / 16.0);
  last_ = now;
}

void time_budget::charge(double cpu_now) {
  cpu_ += std::min(cpu_now - c0_, gaps_);
  c0_ = cpu_now;
  gaps_ = 0.0;
}

void time_budget::stop(std::uint64_t checks) {
  if (!running_)
    return;
  running_ = false;
  const clock::time_point now = clock::now();
  if (on_cpu_) {
    gap(now, checks);
    charge(thread_cpu_seconds());
    return;
  }
  wall_ += std::chrono::duration<double>(now - t0_).count();
  if (wall_ >= half_) { // the first half is spent; what ran past it, unmeasured, is forgiven
    wall_ = half_;
    on_cpu_ = true;
  }
}

bool time_budget::spent(clock::time_point now) {
  if (half_ <= 0.0 || (on_cpu_ && cpu_ >= half_))
    return true;
  if (!running_)
    return false;
  if (on_cpu_)
    gap(now);
  if (now < next_)
    return false;
  const double c = thread_cpu_seconds();
  if (on_cpu_) {
    charge(c);
  } else { // the first half is spent: charge the second on the CPU clock, from now
    wall_ = half_;
    on_cpu_ = true;
    c0_ = c;
    gaps_ = 0.0;
    last_ = now;
  }
  if (cpu_ >= half_)
    return true;
  // What is charged never outruns the steady clock, so the rest of the second half cannot be
  // spent sooner than this.
  next_ = after(now, half_ - cpu_);
  return false;
}

double time_budget::remaining() const {
  if (half_ <= 0.0)
    return 0.0;
  return on_cpu_ ? std::max(0.0, half_ - cpu_) : (half_ - wall_) + half_;
}

// -- cooperative evaluation deadline ----------------------------------------

namespace {
thread_local eval_deadline_guard *t_innermost = nullptr; // the guards armed on this thread
} // namespace

eval_deadline_guard::eval_deadline_guard(std::optional<double> seconds) {
  if (!seconds)
    return;
  budget_.emplace(*seconds);
  budget_->start();
  outer_ = t_innermost;
  t_innermost = this;
}

eval_deadline_guard::~eval_deadline_guard() {
  if (budget_)
    t_innermost = outer_;
}

bool eval_deadline_expired() {
  if (!t_innermost)
    return false;
  const time_budget::clock::time_point now = time_budget::clock::now();
  bool spent = false;
  for (eval_deadline_guard *g = t_innermost; g; g = g->outer_)
    spent = g->budget_->spent(now) || spent;
  return spent;
}

// -- releasing nested values -------------------------------------------------

namespace {
// The owners a release has yet to free: the first few in uninitialized inline slots, the rest
// on the heap, so freeing an ordinary small value constructs and allocates nothing extra.
class release_queue {
public:
  release_queue() = default;
  release_queue(const release_queue &) = delete;
  release_queue &operator=(const release_queue &) = delete;
  ~release_queue() {
    while (count_ > 0)
      slot(--count_).~shared_ptr();
  }

  // Takes `owner`, unless out of memory: then it is left with the caller.
  void push(std::shared_ptr<void> &owner) noexcept {
    if (count_ < kInline) {
      new (&slot(count_++)) std::shared_ptr<void>(std::move(owner));
      return;
    }
    try {
      heap_.push_back(std::move(owner));
    } catch (...) {
      // push_back left `owner` as it was.
    }
  }
  bool pop(std::shared_ptr<void> &out) noexcept {
    if (!heap_.empty()) {
      out = std::move(heap_.back());
      heap_.pop_back();
      return true;
    }
    if (count_ == 0)
      return false;
    std::shared_ptr<void> &s = slot(--count_);
    out = std::move(s);
    s.~shared_ptr();
    return true;
  }

private:
  static constexpr std::size_t kInline = 8;
  std::shared_ptr<void> &slot(std::size_t i) {
    return *std::launder(reinterpret_cast<std::shared_ptr<void> *>(inline_) + i);
  }
  alignas(std::shared_ptr<void>) unsigned char inline_[kInline * sizeof(std::shared_ptr<void>)];
  std::size_t count_ = 0;
  std::vector<std::shared_ptr<void>> heap_;
};

// The queue of the release running on this thread, if one is: release_nested() called while it
// frees something (from that thing's destructor) queues its owner there instead of freeing it
// inline. A raw pointer to the running release's local queue, so no thread-exit destructor is
// involved and nothing outlives the release.
thread_local release_queue *t_release_queue = nullptr;

// The backlog a backlog_scope routes this thread's last owners to, if any.
thread_local detail::release_backlog *t_backlog = nullptr;
} // namespace

void detail::release_nested(std::shared_ptr<void> owner, std::size_t nodes) noexcept {
  if (release_queue *queue = t_release_queue) {
    queue->push(owner); // out of memory: `owner` is still here, and freed here, recursively
    return;
  }
  if (release_backlog *backlog = t_backlog) {
    try {
      backlog->owners_.emplace_back(std::move(owner), nodes);
      return;
    } catch (...) {
      // Out of memory: emplace_back left `owner` as it was; free it now, below.
    }
  }
  release_queue queue;
  t_release_queue = &queue;
  owner.reset();
  for (std::shared_ptr<void> next; queue.pop(next);)
    next.reset(); // a no-op unless this was the last owner
  t_release_queue = nullptr;
}

detail::release_backlog::~release_backlog() { drain(0); }

std::size_t detail::release_backlog::drain(std::size_t budget) {
  release_backlog *outer = t_backlog;
  t_backlog = this; // what freeing an owner drops joins this backlog
  std::size_t freed = 0;
  while (!owners_.empty() && (budget == 0 || freed < budget)) {
    auto [owner, nodes] = std::move(owners_.back());
    owners_.pop_back();
    owner.reset();
    freed += nodes;
  }
  t_backlog = outer;
  return freed;
}

detail::backlog_scope::backlog_scope(release_backlog *backlog)
    : outer_(t_backlog), armed_(backlog != nullptr) {
  if (armed_)
    t_backlog = backlog;
}

detail::backlog_scope::~backlog_scope() {
  if (armed_)
    t_backlog = outer_;
}

// -- per-call allocation limit ----------------------------------------------

namespace {
thread_local eval_alloc_guard *t_alloc_innermost = nullptr;
} // namespace

eval_alloc_guard::eval_alloc_guard(std::optional<std::size_t> max_bytes)
    : limit_(max_bytes), outer_(t_alloc_innermost) {
  if (outer_ && outer_->limit_)
    limit_ = limit_ ? std::min(*limit_, *outer_->limit_) : outer_->limit_;
  t_alloc_innermost = this;
}

eval_alloc_guard::~eval_alloc_guard() { t_alloc_innermost = outer_; }

std::optional<std::size_t> eval_alloc_limit() {
  if (!t_alloc_innermost)
    return kDefaultMaxAllocBytes;
  return t_alloc_innermost->limit_;
}

void eval_alloc_check(std::size_t bytes, const char *who) {
  const std::optional<std::size_t> limit = eval_alloc_limit();
  if (limit && bytes > *limit)
    throw std::runtime_error(std::string(who) + ": the result needs " + std::to_string(bytes) +
                             " bytes, over the allocation limit of " + std::to_string(*limit) +
                             " bytes (limits.max_alloc_bytes, or what is left of max_memory)");
  eval_alloc_note(bytes);
}

void eval_alloc_note(std::size_t bytes) {
  if (eval_alloc_guard *g = t_alloc_innermost)
    g->allocated_ = g->allocated_ > SIZE_MAX - bytes ? SIZE_MAX : g->allocated_ + bytes;
}

// -- value_tag ---------------------------------------------------------------

std::string value_tag::type_name() const {
  return std::visit(
      [](auto &&arg) -> std::string {
        using T = std::decay_t<decltype(arg)>;
        if constexpr (std::is_same_v<T, std::monostate>)
          return "nil";
        else if constexpr (std::is_same_v<T, bool>)
          return "bool";
        else if constexpr (std::is_same_v<T, int64_t>)
          return "int";
        else if constexpr (std::is_same_v<T, double>)
          return "float";
        else if constexpr (std::is_same_v<T, std::string>)
          return "string";
        else if constexpr (std::is_same_v<T, symbol>)
          return "symbol";
        else if constexpr (std::is_same_v<T, list_ptr>)
          return "list";
        else if constexpr (std::is_same_v<T, closure_ptr>)
          return "closure";
        else if constexpr (std::is_same_v<T, dict_ptr>)
          return "dict";
        else if constexpr (std::is_same_v<T, native_fn>)
          return "native_fn";
        else if constexpr (std::is_same_v<T, data_object_ptr>)
          return "data_object";
        else if constexpr (std::is_same_v<T, generator_ptr>)
          return "generator";
        else if constexpr (std::is_same_v<T, bytes_value>)
          return "bytes";
        else
          return "unknown";
      },
      v);
}

// -- environment -------------------------------------------------------------

environment::~environment() { detail::drop_owner(outer); }

value_t *environment::lookup(const std::string &name) {
  for (environment *e = this; e; e = e->outer.get()) {
    auto it = e->bindings.find(name);
    if (it != e->bindings.end())
      return &it->second;
  }
  return nullptr;
}

const value_t *environment::lookup(const std::string &name) const {
  for (const environment *e = this; e; e = e->outer.get()) {
    auto it = e->bindings.find(name);
    if (it != e->bindings.end())
      return &it->second;
  }
  return nullptr;
}

void environment::set(const std::string &name, value_t val) {
  if (frozen)
    throw std::logic_error("set: '" + name + "' in a frozen scope");
  bindings[name] = std::move(val);
}

void environment::set_existing(const std::string &name, value_t val) {
  if (frozen)
    throw std::logic_error("set_existing: '" + name + "' in a frozen scope");
  environment *outermost = this; // the outermost unfrozen scope walked so far
  for (environment *e = this; e && !e->frozen; e = e->outer.get()) {
    auto it = e->bindings.find(name);
    if (it != e->bindings.end()) {
      it->second = std::move(val);
      return;
    }
    outermost = e;
  }
  // Not bound in an unfrozen scope: bind it in the outermost one, which shadows a frozen
  // scope's binding of the same name rather than writing into the shared scope.
  outermost->bindings[name] = std::move(val);
}

environment_ptr environment::extend(environment_ptr parent) {
  auto child = std::make_shared<environment>();
  child->outer = std::move(parent);
  return child;
}

// to_string and values_equal are walks: see data_job.cpp.

} // namespace cvc::state_exec
