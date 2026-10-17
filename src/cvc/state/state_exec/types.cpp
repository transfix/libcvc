#include <algorithm>
#include <cvc/state/state_exec/generator.h>
#include <cvc/state/state_exec/types.h>
#include <sstream>
#include <stdexcept>
#include <time.h>
#include <unordered_set>
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

value_t *environment::lookup(const std::string &name) {
  auto it = bindings.find(name);
  if (it != bindings.end())
    return &it->second;
  if (outer)
    return outer->lookup(name);
  return nullptr;
}

const value_t *environment::lookup(const std::string &name) const {
  auto it = bindings.find(name);
  if (it != bindings.end())
    return &it->second;
  if (outer)
    return outer->lookup(name);
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

// -- to_string ---------------------------------------------------------------

namespace {
// Bounds for to_string: cap total OUTPUT length (a shared/wide structure would otherwise
// expand exponentially) and recursion DEPTH (a deep or cyclic structure would otherwise
// overflow the C++ stack). Past either, emit an ellipsis and stop — enough for debug /
// coercion / error messages, and impossible to weaponise into a hang or OOM.
constexpr std::size_t kToStringMaxLen = std::size_t{1} << 16; // 64 KiB
constexpr int kToStringMaxDepth = 1000;

void to_string_impl(const value_t &val, std::string &out, int depth) {
  if (out.size() >= kToStringMaxLen || depth > kToStringMaxDepth) {
    out += "\xE2\x80\xA6"; // U+2026 HORIZONTAL ELLIPSIS
    return;
  }
  std::visit(
      [&](auto &&arg) {
        using T = std::decay_t<decltype(arg)>;
        if constexpr (std::is_same_v<T, std::monostate>)
          out += "nil";
        else if constexpr (std::is_same_v<T, bool>)
          out += arg ? "#t" : "#f";
        else if constexpr (std::is_same_v<T, int64_t>)
          out += std::to_string(arg);
        else if constexpr (std::is_same_v<T, double>) {
          std::ostringstream oss;
          oss << arg;
          out += oss.str();
        } else if constexpr (std::is_same_v<T, std::string>) {
          out += '"';
          out += arg;
          out += '"';
        } else if constexpr (std::is_same_v<T, bytes_value>) {
          // b"..." — printable ASCII verbatim, everything else (incl. all bytes >= 0x80)
          // hex-escaped as \xNN, so the rendering is always valid ASCII text and never dumps raw
          // binary bytes.
          static const char kHex[] = "0123456789abcdef";
          out += "b\"";
          for (unsigned char b : arg.data) {
            if (b == '"' || b == '\\') {
              out += '\\';
              out += static_cast<char>(b);
            } else if (b >= 0x20 && b < 0x7F) {
              out += static_cast<char>(b);
            } else {
              out += "\\x";
              out += kHex[b >> 4];
              out += kHex[b & 0x0F];
            }
          }
          out += '"';
        } else if constexpr (std::is_same_v<T, symbol>)
          out += arg.name;
        else if constexpr (std::is_same_v<T, list_ptr>) {
          if (!arg || arg->empty()) {
            out += "()";
            return;
          }
          out += '(';
          for (std::size_t i = 0; i < arg->size(); ++i) {
            if (i > 0)
              out += ' ';
            if (out.size() >= kToStringMaxLen) {
              out += "\xE2\x80\xA6";
              break;
            }
            to_string_impl((*arg)[i], out, depth + 1);
          }
          out += ')';
        } else if constexpr (std::is_same_v<T, closure_ptr>)
          out += "<closure>";
        else if constexpr (std::is_same_v<T, dict_ptr>) {
          if (!arg || arg->empty()) {
            out += "{}";
            return;
          }
          out += '{';
          for (std::size_t i = 0; i < arg->size(); ++i) {
            if (i > 0)
              out += ", ";
            if (out.size() >= kToStringMaxLen) {
              out += "\xE2\x80\xA6";
              break;
            }
            out += '"';
            out += (*arg)[i].first;
            out += "\": ";
            to_string_impl((*arg)[i].second, out, depth + 1);
          }
          out += '}';
        } else if constexpr (std::is_same_v<T, native_fn>)
          out += "<native_fn>";
        else if constexpr (std::is_same_v<T, data_object_ptr>)
          out += arg ? "<data_object:" + arg->type_name + ">" : "<data_object:null>";
        else if constexpr (std::is_same_v<T, generator_ptr>)
          out += arg && arg->exhausted ? "<generator:exhausted>" : "<generator>";
        else
          out += "<unknown>";
      },
      val.v);
}
} // namespace

std::string to_string(const value_t &val) {
  std::string out;
  to_string_impl(val, out, 0);
  return out;
}

// -- values_equal ------------------------------------------------------------

namespace {
struct ptr_pair_hash {
  std::size_t operator()(const std::pair<const void *, const void *> &p) const noexcept {
    return std::hash<const void *>()(p.first) * 1000003u ^ std::hash<const void *>()(p.second);
  }
};
using ptr_pair_set = std::unordered_set<std::pair<const void *, const void *>, ptr_pair_hash>;

constexpr int kValuesEqualMaxDepth = 1000; // guard the C++ stack on a deeply-nested compare

// `seen` memoizes the (a,b) pointer-pairs already under comparison. Re-encountering a pair
// (a shared node reached by two paths, or a cycle) returns true co-inductively: this bounds
// work to the number of DISTINCT node-pairs (the physical size), never the exponential
// logical unfolding of a shared DAG, and terminates on cyclic structures instead of
// recursing forever. For finite acyclic values the result is identical to a naive compare.
// A depth cap + deadline poll keep even a deeply-nested or huge NON-shared structure bounded
// (memoization alone does not: distinct nodes are never re-encountered).
bool values_equal_impl(const value_t &a, const value_t &b, ptr_pair_set &seen, int depth) {
  if (depth > kValuesEqualMaxDepth || eval_deadline_expired())
    throw std::runtime_error("values_equal: structure too deep or large to compare");
  if (a.v.index() != b.v.index())
    return false;
  return std::visit(
      [&](auto &&arg_a) -> bool {
        using T = std::decay_t<decltype(arg_a)>;
        auto &arg_b = std::get<T>(b.v);
        if constexpr (std::is_same_v<T, std::monostate>)
          return true;
        else if constexpr (std::is_same_v<T, bool>)
          return arg_a == arg_b;
        else if constexpr (std::is_same_v<T, int64_t>)
          return arg_a == arg_b;
        else if constexpr (std::is_same_v<T, double>)
          return arg_a == arg_b;
        else if constexpr (std::is_same_v<T, std::string>)
          return arg_a == arg_b;
        else if constexpr (std::is_same_v<T, symbol>)
          return arg_a == arg_b;
        else if constexpr (std::is_same_v<T, list_ptr>) {
          if (arg_a == arg_b)
            return true;
          if (!arg_a || !arg_b)
            return false;
          if (arg_a->size() != arg_b->size())
            return false;
          if (!seen.insert({static_cast<const void *>(arg_a.get()),
                            static_cast<const void *>(arg_b.get())})
                   .second)
            return true; // this pair is already being compared (shared/cyclic) -> equal
          for (std::size_t i = 0; i < arg_a->size(); ++i)
            if (!values_equal_impl((*arg_a)[i], (*arg_b)[i], seen, depth + 1))
              return false;
          return true;
        } else if constexpr (std::is_same_v<T, closure_ptr>)
          return arg_a == arg_b; // identity comparison
        else if constexpr (std::is_same_v<T, dict_ptr>) {
          if (arg_a == arg_b)
            return true;
          if (!arg_a || !arg_b)
            return false;
          if (arg_a->size() != arg_b->size())
            return false;
          if (!seen.insert({static_cast<const void *>(arg_a.get()),
                            static_cast<const void *>(arg_b.get())})
                   .second)
            return true;
          for (std::size_t i = 0; i < arg_a->size(); ++i) {
            if ((*arg_a)[i].first != (*arg_b)[i].first)
              return false;
            if (!values_equal_impl((*arg_a)[i].second, (*arg_b)[i].second, seen, depth + 1))
              return false;
          }
          return true;
        } else if constexpr (std::is_same_v<T, native_fn>)
          return false; // functions are never equal
        else if constexpr (std::is_same_v<T, data_object_ptr>)
          return arg_a == arg_b; // identity comparison
        else if constexpr (std::is_same_v<T, generator_ptr>)
          return arg_a == arg_b; // identity comparison
        else if constexpr (std::is_same_v<T, bytes_value>)
          return arg_a == arg_b; // byte-wise value comparison
        else
          return false;
      },
      a.v);
}
} // namespace

bool values_equal(const value_t &a, const value_t &b) {
  ptr_pair_set seen;
  return values_equal_impl(a, b, seen, 0);
}

} // namespace cvc::state_exec
