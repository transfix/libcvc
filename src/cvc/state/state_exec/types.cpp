#include "scope_mark.h"

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cvc/state/state_exec/exceptions.h> // dsl_error: a refused allocation is MemoryError
#include <cvc/state/state_exec/types.h>
#include <new>
#include <stdexcept>
#include <time.h>
#include <unordered_map>
#include <utility>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#if defined(_M_X64) || defined(_M_IX86)
#include <intrin.h>
#endif
#endif

namespace cvc::state_exec {

// -- time budgets -------------------------------------------------------------

#ifdef _WIN32
namespace {
// The rate QueryThreadCycleTime counts at, in cycles a second, or 0 where it can't be used.
// Windows counts a thread's cycles on the time-stamp counter, so the rate is the TSC's, which
// is constant only where the processor says it is invariant (CPUID 0x80000007, EDX bit 8).
// Measured once, against the performance counter, over a few milliseconds.
double thread_cycle_rate() {
#if defined(_M_X64) || defined(_M_IX86)
  int info[4] = {};
  __cpuid(info, static_cast<int>(0x80000000));
  if (static_cast<unsigned>(info[0]) < 0x80000007u)
    return 0.0;
  __cpuid(info, static_cast<int>(0x80000007));
  if ((info[3] & (1 << 8)) == 0)
    return 0.0;
  LARGE_INTEGER freq, q0, q1;
  if (!QueryPerformanceFrequency(&freq) || freq.QuadPart <= 0)
    return 0.0;
  QueryPerformanceCounter(&q0);
  const unsigned long long c0 = __rdtsc();
  Sleep(5);
  QueryPerformanceCounter(&q1);
  const unsigned long long c1 = __rdtsc();
  const double seconds = double(q1.QuadPart - q0.QuadPart) / double(freq.QuadPart);
  return seconds > 0.0 ? double(c1 - c0) / seconds : 0.0;
#else
  return 0.0; // no time-stamp counter to calibrate against (ARM64): GetThreadTimes
#endif
}
} // namespace
#endif

double thread_cpu_seconds() {
#ifdef _WIN32
  static const double cycle_rate = thread_cycle_rate();
  if (cycle_rate > 0.0) {
    ULONG64 cycles = 0;
    if (QueryThreadCycleTime(GetCurrentThread(), &cycles))
      return double(cycles) / cycle_rate;
  }
  // 100 ns units, advanced at the scheduler tick (about 15.6 ms).
  FILETIME created, exited, kernel, user;
  if (GetThreadTimes(GetCurrentThread(), &created, &exited, &kernel, &user)) {
    const auto ticks = [](const FILETIME &t) {
      return (static_cast<unsigned long long>(t.dwHighDateTime) << 32) | t.dwLowDateTime;
    };
    return 1e-7 * double(ticks(kernel) + ticks(user));
  }
#elif defined(CLOCK_THREAD_CPUTIME_ID)
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

bool eval_deadline_guard::expired() { return budget_ && budget_->spent(); }

bool eval_deadline_expired() {
  if (!t_innermost)
    return false;
  const time_budget::clock::time_point now = time_budget::clock::now();
  bool spent = false;
  for (eval_deadline_guard *g = t_innermost; g; g = g->outer_)
    spent = g->budget_->spent(now) || spent;
  return spent;
}

void eval_deadline_check(const char *what) {
  if (eval_deadline_expired())
    throw evaluation_timeout(std::string(what) + ": evaluation exceeded timeout");
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
    throw dsl_error("MemoryError",
                    std::string(who) + ": the result needs " + std::to_string(bytes) +
                        " bytes, over the allocation limit of " + std::to_string(*limit) +
                        " bytes (limits.max_alloc_bytes, or what is left of max_memory)");
  eval_alloc_note(bytes);
}

void eval_alloc_note(std::size_t bytes) {
  if (eval_alloc_guard *g = t_alloc_innermost)
    g->allocated_ = g->allocated_ > SIZE_MAX - bytes ? SIZE_MAX : g->allocated_ + bytes;
}

// -- async kernels -------------------------------------------------------------

namespace {
thread_local bool t_in_async_kernel = false;
} // namespace

bool in_async_kernel() { return t_in_async_kernel; }

detail::async_kernel_scope::async_kernel_scope(bool running) : outer_(t_in_async_kernel) {
  t_in_async_kernel = running;
}

detail::async_kernel_scope::~async_kernel_scope() { t_in_async_kernel = outer_; }

void detail::check_not_in_async_kernel(const char *what) {
  if (t_in_async_kernel)
    throw std::logic_error(std::string(what) +
                           ": an async function's kernel must not use the evaluator or a "
                           "scheduler (docs/STATE_EXEC_EXCEPTIONS.md 3.7)");
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

namespace {
using binding_map = decltype(environment::bindings);
using binding_node = binding_map::node_type;

// -- scopes recycled on this thread --
//
// A call scope (environment::extend) lives for one call, and a std::unordered_map is costly to
// make: on MSVC its default constructor allocates a list sentinel and a bucket array, so a scope
// cost three allocations before its first binding (the make_shared block, those two), and every
// binding a node more. So a scope from extend() is not destroyed when its last owner drops it:
// it is emptied, as its destructor would empty it, and kept on this thread for the next
// extend(), its map's sentinel and buckets with it. The nodes of its bindings are kept too, for
// the next set() of a new name.
//
// A scope and its shared_ptr control block are one allocation, as make_shared makes them: a
// block with a slot for the control block, then the scope; the control block's allocator hands
// out the slot. So a long-lived scope (a process's) costs one allocation, as before, 16 bytes
// larger. The block is kept (or freed) when the control block is deallocated, which is after the
// scope was emptied.
//
// What is kept is bounded, and per thread, so nothing is shared between threads: a scope or
// node made on one thread and dropped on another is kept by the other. On 64-bit MSVC that is at
// most 64 blocks of 120 bytes, each with its empty map's sentinel and 8 buckets (about 150
// bytes), and 128 nodes of about 130 bytes: under 35 KB per thread that steps a program.
constexpr std::size_t kKeptScopes = 64;
constexpr std::size_t kKeptNodes = 128;
// The slot for a scope's control block: a vtable, two counts and the pointer (24 bytes with
// MSVC's and libstdc++'s shared_ptr, 32 with libc++'s; the deleter and allocator are empty).
constexpr std::size_t kControlBytes = 32;
static_assert(kControlBytes % alignof(environment) == 0);

// A kept scope must be what a new one is: an empty map with a new map's bucket count, so that
// how many buckets a scope has (which heap measurement counts, heap.h) never depends on what the
// thread did before. (MSVC's maps start at 8 buckets and keep them up to 8 bindings; libstdc++'s
// and libc++'s allocate none until the first binding, so there only scopes that never had a
// binding are kept, with the nodes of all of them.)
const std::size_t kFreshBuckets = binding_map().bucket_count();

// A block: the control block's slot, then the scope.
environment *scope_in(void *block) noexcept {
  return std::launder(
      reinterpret_cast<environment *>(static_cast<unsigned char *>(block) + kControlBytes));
}

void *new_block() {
  void *block = ::operator new(kControlBytes + sizeof(environment));
  try {
    ::new (static_cast<unsigned char *>(block) + kControlBytes) environment();
  } catch (...) {
    ::operator delete(block);
    throw;
  }
  return block;
}

void free_block(void *block) noexcept {
  scope_in(block)->~environment();
  ::operator delete(block);
}

struct scope_pool {
  void *scopes[kKeptScopes];
  std::size_t nscopes = 0;
  binding_node nodes[kKeptNodes];
  std::size_t nnodes = 0;

  scope_pool() = default;
  scope_pool(const scope_pool &) = delete;
  scope_pool &operator=(const scope_pool &) = delete;
  ~scope_pool();
};

// Set once the thread has destroyed its pool: scopes dropped later in the thread's exit (by
// another thread_local's destructor) are freed.
thread_local bool t_scope_pool_gone = false;
thread_local scope_pool t_scope_pool;
// The block whose slot the next control block goes in: set by extend() just before it makes the
// shared_ptr, which allocates the control block on this thread, at once.
thread_local void *t_placing = nullptr;

scope_pool *this_thread_pool() noexcept { return t_scope_pool_gone ? nullptr : &t_scope_pool; }

scope_pool::~scope_pool() {
  t_scope_pool_gone = true;
  while (nscopes > 0)
    free_block(scopes[--nscopes]);
  // The nodes go with the array.
}

// Hands out the control block of a scope extend() made: the slot at the start of its block. Its
// deallocation is the block's end: the block is kept for the next extend(), or freed.
template <class T> struct block_allocator {
  using value_type = T;
  block_allocator() = default;
  template <class U> block_allocator(const block_allocator<U> &) noexcept {}

  T *allocate(std::size_t /*n: one control block*/) {
    static_assert(sizeof(T) <= kControlBytes && alignof(T) <= alignof(std::max_align_t),
                  "a scope's control block doesn't fit its slot: raise kControlBytes");
    void *slot = t_placing;
    t_placing = nullptr;
    if (!slot) // only extend() makes these, and it sets the slot first
      throw std::bad_alloc();
    return static_cast<T *>(slot);
  }
  void deallocate(T *p, std::size_t) noexcept {
    void *block = p;
    const environment *e = scope_in(block);
    scope_pool *pool = this_thread_pool();
    if (pool && pool->nscopes < kKeptScopes && e->bindings.empty() &&
        e->bindings.bucket_count() == kFreshBuckets) {
      scope_in(block)->frozen = false;
      pool->scopes[pool->nscopes++] = block;
      return;
    }
    free_block(block); // ~environment of an empty scope with no enclosing one: frees the map
  }
  template <class U> bool operator==(const block_allocator<U> &) const noexcept { return true; }
  template <class U> bool operator!=(const block_allocator<U> &) const noexcept { return false; }
};

// Takes the place of the destructor for a scope extend() made: frees what the destructor would,
// in its order (the enclosing scope, then each binding, through ~value_tag as the map's own
// destruction would, so a release backlog sees the same owners), keeping the binding nodes of a
// scope that will be kept. Its block goes with its control block (block_allocator::deallocate).
struct scope_recycler {
  void operator()(environment *e) const noexcept {
    detail::drop_owner(e->outer);
    e->outer.reset();
    binding_map &b = e->bindings;
    scope_pool *pool = this_thread_pool();
    if (!pool) {
      b.clear(); // as the map's destructor would
      return;
    }
    while (!b.empty()) {
      binding_node node = b.extract(b.begin());
      // The value goes here, and the node is left holding nil, not a moved-from value: that may
      // still hold what it held (libc++ moves a std::function whose callable fits in it by
      // copying the callable), which a kept node would then keep alive.
      {
        const value_t gone = std::exchange(node.mapped(), value_t());
      }
      if (pool->nnodes < kKeptNodes)
        pool->nodes[pool->nnodes++] = std::move(node);
    }
  }
};

// Bind `name` to `val` in `b`, which doesn't have it: in a node this thread kept, if it has one.
void bind_new(binding_map &b, const std::string &name, value_t &&val) {
  scope_pool *pool = this_thread_pool();
  if (!pool || pool->nnodes == 0) {
    b.emplace(name, std::move(val));
    return;
  }
  // Out of the pool first: should the key's copy or the insert throw, the node (with the value,
  // if it got that far) is freed here, as a failed emplace frees its own.
  binding_node node = std::move(pool->nodes[--pool->nnodes]);
  node.key() = name;
  node.mapped() = std::move(val);
  b.insert(std::move(node)); // the name is new, so it goes in
}

// The bucket of a hash in a map of `buckets` buckets: hash mod buckets, which the standard
// libraries compute as a mask when the count is a power of two (MSVC's always is, libc++'s may
// be) and as a remainder otherwise (libstdc++'s prime counts, libc++'s others). Checked against
// the map's own bucket() at startup; if it doesn't hold, lookups use find().
std::size_t bucket_of(std::size_t hash, std::size_t buckets) {
  return (buckets & (buckets - 1)) == 0 ? (hash & (buckets - 1)) : hash % buckets;
}

bool bucket_formula_holds() {
  binding_map m;
  const std::hash<std::string> hasher;
  for (int round = 0; round < 6; ++round) {
    for (int i = 0; i < 64; ++i) {
      const std::string key = "k" + std::to_string(i * 7919 + round);
      const std::size_t buckets = m.bucket_count();
      if (buckets > 0 && bucket_of(hasher(key), buckets) != m.bucket(key))
        return false;
    }
    for (int i = 0; i < (8 << round); ++i)
      m.emplace("n" + std::to_string(round) + "_" + std::to_string(i), value_t());
  }
  return true;
}

// Zero (false: find()) until the check has run, should a lookup run during static initialization.
const bool kBucketFormulaHolds = bucket_formula_holds();

// The binding of `name` (whose hash is `hash`) in `b`, or null.
template <class Map>
auto find_hashed(Map &b, const std::string &name, std::size_t hash) -> decltype(&*b.begin()) {
  if (!kBucketFormulaHolds) {
    auto it = b.find(name);
    return it != b.end() ? &*it : nullptr;
  }
  const std::size_t n = bucket_of(hash, b.bucket_count());
  for (auto it = b.begin(n), end = b.end(n); it != end; ++it)
    if (it->first == name)
      return &*it;
  return nullptr;
}

// Walk the scope chain from `e` for `name`, hashing the name once (not once per scope) and
// skipping empty scopes. The binding found, or null.
template <class Env>
auto binding_in(Env *e, const std::string &name) -> decltype(&*e->bindings.begin()) {
  std::size_t hash = 0;
  bool hashed = false;
  for (; e; e = e->outer.get()) {
    if (e->bindings.empty())
      continue;
    if (!hashed) {
      hash = std::hash<std::string>{}(name);
      hashed = true;
    }
    if (auto *binding = find_hashed(e->bindings, name, hash))
      return binding;
  }
  return nullptr;
}
} // namespace

const std::pair<const std::string, value_t> *
detail::find_marked(const environment *e, const std::string &name, scope_mark &mark) {
  mark = scope_mark{};
  std::size_t hash = 0;
  bool hashed = false;
  std::uint64_t depth = 0, stamp = 0;
  for (; e; e = e->outer.get()) {
    ++depth;
    stamp = stamp_with(stamp, e);
    if (e->bindings.empty())
      continue;
    if (!hashed) {
      hash = std::hash<std::string>{}(name);
      hashed = true;
    }
    if (auto *binding = find_hashed(e->bindings, name, hash)) {
      if (depth <= UINT32_MAX)
        mark = scope_mark{static_cast<std::uint32_t>(depth), stamp};
      return binding;
    }
  }
  return nullptr;
}

namespace {
// The count scope serials are drawn from, a run at a time, and this thread's run: one locked add
// per kSerialRun scopes. (Constant-initialized, so usable at any point of a thread's life.)
constexpr std::uint32_t kSerialRun = 1024;
std::atomic<std::uint32_t> g_next_serial_run{0};
thread_local std::uint32_t t_next_serial = 0;
thread_local std::uint32_t t_serials_left = 0;
} // namespace

std::uint32_t detail::new_scope_serial() noexcept {
  if (t_serials_left == 0) {
    t_next_serial = g_next_serial_run.fetch_add(kSerialRun, std::memory_order_relaxed);
    t_serials_left = kSerialRun;
  }
  --t_serials_left;
  return t_next_serial++;
}

environment::~environment() { detail::drop_owner(outer); }

value_t *environment::lookup(const std::string &name) {
  auto *binding = binding_in(this, name);
  return binding ? &binding->second : nullptr;
}

const value_t *environment::lookup(const std::string &name) const {
  auto *binding = binding_in(this, name);
  return binding ? &binding->second : nullptr;
}

void environment::set(const std::string &name, value_t val) {
  if (frozen)
    throw std::logic_error("set: '" + name + "' in a frozen scope");
  if (!bindings.empty())
    if (auto it = bindings.find(name); it != bindings.end()) {
      it->second = std::move(val);
      return;
    }
  bind_new(bindings, name, std::move(val));
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
  bind_new(outermost->bindings, name, std::move(val));
}

environment_ptr environment::extend(environment_ptr parent) {
  scope_pool *pool = this_thread_pool();
  void *block = nullptr;
  if (pool && pool->nscopes > 0) {
    block = pool->scopes[--pool->nscopes];
    // A new scope, at the address of the one dropped last: a serial of its own tells the two
    // apart (scope_mark.h). A block from new_block() has a new scope's already.
    scope_in(block)->serial.value = detail::new_scope_serial();
  } else {
    block = new_block();
  }
  environment *child = scope_in(block);
  child->outer = std::move(parent);
  t_placing = block; // its control block goes in the block, so making the shared_ptr can't fail
  return environment_ptr(child, scope_recycler{}, block_allocator<environment>());
}

// to_string and values_equal are walks: see data_job.cpp.

} // namespace cvc::state_exec
