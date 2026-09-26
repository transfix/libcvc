#include <cvc/core/state_exec/generator.h>
#include <cvc/core/state_exec/types.h>
#include <sstream>
#include <unordered_set>
#include <utility>

namespace cvc::state_exec {

// -- cooperative evaluation deadline ----------------------------------------

namespace {
thread_local std::optional<std::chrono::steady_clock::time_point> t_eval_deadline;
} // namespace

eval_deadline_guard::eval_deadline_guard(std::optional<double> seconds) : prev_(t_eval_deadline) {
  if (seconds) {
    const auto d = std::chrono::steady_clock::now() +
                   std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                       std::chrono::duration<double>(*seconds));
    if (!t_eval_deadline || d < *t_eval_deadline) // tighten only; a nested run never relaxes
      t_eval_deadline = d;
  }
}

eval_deadline_guard::~eval_deadline_guard() { t_eval_deadline = prev_; }

bool eval_deadline_expired() {
  return t_eval_deadline && std::chrono::steady_clock::now() >= *t_eval_deadline;
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

void environment::set(const std::string &name, value_t val) { bindings[name] = std::move(val); }

void environment::set_existing(const std::string &name, value_t val) {
  auto it = bindings.find(name);
  if (it != bindings.end()) {
    it->second = std::move(val);
    return;
  }
  if (outer) {
    outer->set_existing(name, std::move(val));
    return;
  }
  // Not found anywhere — set in this scope
  bindings[name] = std::move(val);
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

// `seen` memoizes the (a,b) pointer-pairs already under comparison. Re-encountering a pair
// (a shared node reached by two paths, or a cycle) returns true co-inductively: this bounds
// work to the number of DISTINCT node-pairs (the physical size), never the exponential
// logical unfolding of a shared DAG, and terminates on cyclic structures instead of
// recursing forever. For finite acyclic values the result is identical to a naive compare.
bool values_equal_impl(const value_t &a, const value_t &b, ptr_pair_set &seen) {
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
            if (!values_equal_impl((*arg_a)[i], (*arg_b)[i], seen))
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
            if (!values_equal_impl((*arg_a)[i].second, (*arg_b)[i].second, seen))
              return false;
          }
          return true;
        } else if constexpr (std::is_same_v<T, native_fn>)
          return false; // functions are never equal
        else if constexpr (std::is_same_v<T, data_object_ptr>)
          return arg_a == arg_b; // identity comparison
        else if constexpr (std::is_same_v<T, generator_ptr>)
          return arg_a == arg_b; // identity comparison
        else
          return false;
      },
      a.v);
}
} // namespace

bool values_equal(const value_t &a, const value_t &b) {
  ptr_pair_set seen;
  return values_equal_impl(a, b, seen);
}

} // namespace cvc::state_exec
