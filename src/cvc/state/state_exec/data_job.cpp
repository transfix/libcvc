/*
  Copyright 2026 The University of Texas at Austin

  data_job — walks over nested values (compare, print, copy, macro expansion) that keep their
             work on the heap and run a slice at a time.
*/

#include <algorithm>
#include <charconv>
#include <cvc/state/state.h>
#include <cvc/state/state_exec/data_job.h>
#include <cvc/state/state_exec/evaluator.h> // evaluation_stats
#include <cvc/state/state_exec/generator.h>
#include <cvc/state/state_exec/scheduler.h> // the settings' key prefixes
#include <optional>
#include <sstream>
#include <stdexcept>
#include <unordered_set>
#include <utility>
#include <vector>

namespace cvc::state_exec {

namespace {

// A list or dict being walked, held by owning pointer and read by index.
struct cursor {
  list_ptr list;
  dict_ptr dict;
  std::size_t next = 0; // the next element to visit

  std::size_t size() const { return list ? list->size() : dict->size(); }
};

// The cursor for `v` if it is a non-empty list or dict.
std::optional<cursor> open_cursor(const value_t &v) {
  if (auto *l = std::get_if<list_ptr>(&v.v); l && *l && !(*l)->empty())
    return cursor{*l, nullptr, 0};
  if (auto *d = std::get_if<dict_ptr>(&v.v); d && *d && !(*d)->empty())
    return cursor{nullptr, *d, 0};
  return std::nullopt;
}

// -- print -------------------------------------------------------------------

// to_string caps total OUTPUT length (a shared/wide structure would otherwise expand
// exponentially, a cyclic one forever); past it, it emits an ellipsis and stops -- enough for
// debug / coercion / error messages, and impossible to weaponise into a hang or OOM. Depth
// needs no cap of its own: the walk keeps its open lists and dicts on a heap stack, not the
// native one, and each level opened costs output, so the length cap bounds that stack too.
constexpr std::size_t kToStringMaxLen = std::size_t{1} << 16; // 64 KiB
constexpr const char *kEllipsis = "\xE2\x80\xA6";             // U+2026 HORIZONTAL ELLIPSIS

class print_walk final : public data_job {
public:
  explicit print_walk(value_t v) : root_(std::move(v)) {}

  bool step(std::size_t budget) override {
    if (!started_) {
      started_ = true;
      print(root_);
    }
    for (std::size_t visited = 0; !open_.empty(); ++visited) {
      if (budget != 0 && visited >= budget)
        return false;
      cursor &c = open_.back();
      const bool is_list = c.list != nullptr;
      const std::size_t i = c.next++;
      if (i >= c.size()) {
        out_ += is_list ? ')' : '}';
        open_.pop_back();
        continue;
      }
      ++nodes_;
      if (i > 0)
        out_ += is_list ? " " : ", ";
      if (out_.size() >= kToStringMaxLen) {
        out_ += kEllipsis;
        out_ += is_list ? ')' : '}';
        open_.pop_back();
        continue;
      }
      if (is_list) {
        print((*c.list)[i]); // may push, invalidating c
      } else {
        const auto &[key, value] = (*c.dict)[i];
        out_ += '"';
        out_ += key;
        out_ += "\": ";
        print(value); // may push, invalidating c
      }
    }
    return true;
  }

  value_t result() override { return value_t{std::move(out_)}; }
  std::string take_string() { return std::move(out_); }

private:
  // Print `val`; a non-empty list or dict is only opened (its bracket printed and a cursor
  // pushed for step() to print its elements and close it).
  void print(const value_t &val) {
    if (out_.size() >= kToStringMaxLen) {
      out_ += kEllipsis;
      return;
    }
    std::visit(
        [&](auto &&arg) {
          using T = std::decay_t<decltype(arg)>;
          if constexpr (std::is_same_v<T, std::monostate>)
            out_ += "nil";
          else if constexpr (std::is_same_v<T, bool>)
            out_ += arg ? "#t" : "#f";
          else if constexpr (std::is_same_v<T, int64_t>)
            out_ += std::to_string(arg);
          else if constexpr (std::is_same_v<T, double>) {
            std::ostringstream oss;
            oss << arg;
            out_ += oss.str();
          } else if constexpr (std::is_same_v<T, std::string>) {
            out_ += '"';
            out_ += arg;
            out_ += '"';
          } else if constexpr (std::is_same_v<T, bytes_value>) {
            // b"..." — printable ASCII verbatim, everything else (incl. all bytes >= 0x80)
            // hex-escaped as \xNN, so the rendering is always valid ASCII text and never dumps
            // raw binary bytes.
            static const char kHex[] = "0123456789abcdef";
            out_ += "b\"";
            for (unsigned char b : arg.data) {
              if (b == '"' || b == '\\') {
                out_ += '\\';
                out_ += static_cast<char>(b);
              } else if (b >= 0x20 && b < 0x7F) {
                out_ += static_cast<char>(b);
              } else {
                out_ += "\\x";
                out_ += kHex[b >> 4];
                out_ += kHex[b & 0x0F];
              }
            }
            out_ += '"';
          } else if constexpr (std::is_same_v<T, symbol>)
            out_ += arg.name;
          else if constexpr (std::is_same_v<T, list_ptr>) {
            if (!arg || arg->empty()) {
              out_ += "()";
              return;
            }
            out_ += '(';
            open_.push_back({arg, nullptr, 0});
          } else if constexpr (std::is_same_v<T, closure_ptr>)
            out_ += "<closure>";
          else if constexpr (std::is_same_v<T, dict_ptr>) {
            if (!arg || arg->empty()) {
              out_ += "{}";
              return;
            }
            out_ += '{';
            open_.push_back({nullptr, arg, 0});
          } else if constexpr (std::is_same_v<T, native_fn>)
            out_ += "<native_fn>";
          else if constexpr (std::is_same_v<T, data_object_ptr>)
            out_ += arg ? "<data_object:" + arg->type_name + ">" : "<data_object:null>";
          else if constexpr (std::is_same_v<T, generator_ptr>)
            out_ += arg && arg->exhausted ? "<generator:exhausted>" : "<generator>";
          else
            out_ += "<unknown>";
        },
        val.v);
  }

  value_t root_;
  bool started_ = false;
  std::string out_;
  std::vector<cursor> open_;
};

class concat_walk final : public data_job {
public:
  concat_walk(std::vector<value_t> parts, std::string sep)
      : parts_(std::move(parts)), sep_(std::move(sep)) {}

  bool step(std::size_t budget) override {
    std::size_t used = 0;
    while (next_ < parts_.size()) {
      if (!part_) {
        if (next_ > 0)
          out_ += sep_;
        if (auto *s = std::get_if<std::string>(&parts_[next_].v)) {
          out_ += *s;
          ++next_;
          continue;
        }
        part_ = std::make_unique<print_walk>(parts_[next_]);
      }
      if (budget != 0 && used >= budget)
        return false;
      const std::size_t before = part_->nodes();
      const bool done = part_->step(budget == 0 ? 0 : budget - used);
      used += std::max<std::size_t>(1, part_->nodes() - before);
      nodes_ += part_->nodes() - before;
      if (!done)
        return false;
      out_ += part_->take_string();
      part_.reset();
      ++next_;
    }
    return true;
  }

  value_t result() override { return value_t{std::move(out_)}; }

private:
  std::vector<value_t> parts_;
  std::string sep_;
  std::size_t next_ = 0;
  std::unique_ptr<print_walk> part_; // the part being printed
  std::string out_;
};

// -- equal -------------------------------------------------------------------

struct ptr_pair_hash {
  std::size_t operator()(const std::pair<const void *, const void *> &p) const noexcept {
    return std::hash<const void *>()(p.first) * 1000003u ^ std::hash<const void *>()(p.second);
  }
};
using ptr_pair_set = std::unordered_set<std::pair<const void *, const void *>, ptr_pair_hash>;

// `seen_` memoizes the container pairs already under comparison: re-encountering one (a shared
// node reached by two paths, or a cycle) counts as equal co-inductively. That bounds the work to
// the DISTINCT node pairs (the physical size), never the exponential logical unfolding of a
// shared DAG, and ends on cyclic structures; for finite acyclic values the result is that of a
// naive compare. A deadline poll per node bounds a huge NON-shared structure (memoization alone
// does not: distinct nodes are never re-encountered).
class equal_walk final : public data_job {
public:
  equal_walk(value_t a, value_t b) : a_(std::move(a)), b_(std::move(b)) {}

  bool step(std::size_t budget) override {
    if (done_)
      return true;
    if (!started_) {
      started_ = true;
      if (!visit(a_, b_))
        return finish(false);
    }
    for (std::size_t visited = 0; !open_.empty(); ++visited) {
      if (budget != 0 && visited >= budget)
        return false;
      auto &[ca, cb] = open_.back();
      const std::size_t n = ca.size();
      if (n != cb.size()) // changed between two slices
        return finish(false);
      const std::size_t i = ca.next++;
      if (i >= n) {
        open_.pop_back();
        continue;
      }
      ++nodes_;
      if (eval_deadline_expired())
        throw std::runtime_error("values_equal: structure too large to compare");
      bool equal;
      // Element references stay valid through visit(), which only pushes onto open_.
      if (ca.list) {
        const value_t &x = (*ca.list)[i];
        const value_t &y = (*cb.list)[i];
        equal = visit(x, y); // may push, invalidating ca and cb
      } else {
        const auto &[kx, x] = (*ca.dict)[i];
        const auto &[ky, y] = (*cb.dict)[i];
        equal = kx == ky && visit(x, y); // may push, invalidating ca and cb
      }
      if (!equal)
        return finish(false);
    }
    return finish(true);
  }

  value_t result() override { return value_t{equal_}; }
  bool equal() const { return equal_; }

private:
  bool finish(bool equal) {
    equal_ = equal;
    done_ = true;
    open_.clear();
    return true;
  }

  // Whether x and y are equal as far as one level shows; a list or dict pair still to compare
  // element by element gets a cursor pair.
  bool visit(const value_t &x, const value_t &y) {
    if (x.v.index() != y.v.index())
      return false;
    return std::visit(
        [&](auto &&arg_a) -> bool {
          using T = std::decay_t<decltype(arg_a)>;
          auto &arg_b = std::get<T>(y.v);
          if constexpr (std::is_same_v<T, std::monostate>)
            return true;
          else if constexpr (std::is_same_v<T, list_ptr> || std::is_same_v<T, dict_ptr>) {
            if (arg_a == arg_b)
              return true;
            if (!arg_a || !arg_b)
              return false;
            if (arg_a->size() != arg_b->size())
              return false;
            if (!seen_)
              seen_.emplace(); // not before the first pair: comparing scalars allocates nothing
            if (!seen_
                     ->insert({static_cast<const void *>(arg_a.get()),
                               static_cast<const void *>(arg_b.get())})
                     .second)
              return true; // this pair is already being compared (shared/cyclic) -> equal
            if constexpr (std::is_same_v<T, list_ptr>)
              open_.push_back({cursor{arg_a, nullptr, 0}, cursor{arg_b, nullptr, 0}});
            else
              open_.push_back({cursor{nullptr, arg_a, 0}, cursor{nullptr, arg_b, 0}});
            return true;
          } else if constexpr (std::is_same_v<T, native_fn>)
            return false; // functions are never equal
          else
            // bool, numbers, strings, symbols and bytes by value; closures, data objects and
            // generators by identity
            return arg_a == arg_b;
        },
        x.v);
  }

  value_t a_, b_;
  bool started_ = false;
  bool done_ = false;
  bool equal_ = false;
  std::optional<ptr_pair_set> seen_;
  std::vector<std::pair<cursor, cursor>> open_;
};

// -- copy --------------------------------------------------------------------

// Memoized on the source node, so a shared-pointer DAG is copied in PHYSICAL time (the copy has
// the same sharing, disjoint from the original) and a cycle is copied as a cycle. A deadline
// poll per node aborts a huge payload at the time budget.
class copy_walk final : public data_job {
public:
  explicit copy_walk(value_t v) : root_(std::move(v)) {}

  bool step(std::size_t budget) override {
    if (!started_) {
      started_ = true;
      result_ = copy(root_);
    }
    for (std::size_t visited = 0; !open_.empty(); ++visited) {
      if (budget != 0 && visited >= budget)
        return false;
      auto &[src, dst] = open_.back();
      const std::size_t i = src.next++;
      if (i >= src.size()) {
        open_.pop_back();
        continue;
      }
      ++nodes_;
      if (eval_deadline_expired())
        throw std::runtime_error("state-data-get: evaluation exceeded time budget");
      if (src.list) {
        list_ptr out = std::get<list_ptr>(dst.v);
        const value_t &e = (*src.list)[i];
        out->push_back(copy(e)); // may push, invalidating src and dst
      } else {
        dict_ptr out = std::get<dict_ptr>(dst.v);
        const auto &[k, e] = (*src.dict)[i];
        out->emplace_back(k, copy(e)); // may push, invalidating src and dst
      }
    }
    return true;
  }

  value_t result() override { return std::move(result_); }

private:
  // The copy of `v`: a list or dict is a new empty one, filled by step() through its cursor.
  value_t copy(const value_t &v) {
    std::optional<cursor> c = open_cursor(v);
    if (!c) {
      // empty or null lists and dicts still get their own (empty) copy
      if (auto *l = std::get_if<list_ptr>(&v.v); l && *l)
        return make_list();
      if (auto *d = std::get_if<dict_ptr>(&v.v); d && *d)
        return make_dict();
      return v; // scalar / symbol (opaque callables are refused at state-data-set)
    }
    const void *key = c->list ? static_cast<const void *>(c->list.get())
                              : static_cast<const void *>(c->dict.get());
    if (auto found = memo_.find(key); found != memo_.end())
      return found->second; // shared node / cycle already copied -> reuse (bounds the walk)
    value_t out;
    if (c->list) {
      auto l = std::make_shared<std::vector<value_t>>();
      l->reserve(c->list->size());
      out = value_t{l};
    } else {
      auto d = std::make_shared<ordered_map>();
      d->reserve(c->dict->size());
      out = value_t{d};
    }
    memo_.emplace(key, out); // registered before its elements are copied
    open_.emplace_back(std::move(*c), out);
    return out;
  }

  value_t root_;
  bool started_ = false;
  value_t result_;
  std::unordered_map<const void *, value_t> memo_;
  std::vector<std::pair<cursor, value_t>> open_;
};

// -- substitute --------------------------------------------------------------

class substitute_walk final : public data_job {
public:
  substitute_walk(value_t tmpl, std::unordered_map<std::string, value_t> subst)
      : root_(std::move(tmpl)), subst_(std::move(subst)) {}

  bool step(std::size_t budget) override {
    if (!started_) {
      started_ = true;
      result_ = expand(root_);
    }
    for (std::size_t visited = 0; !open_.empty(); ++visited) {
      if (budget != 0 && visited >= budget)
        return false;
      auto &[src, dst] = open_.back();
      const std::size_t i = src.next++;
      if (i >= src.list->size()) {
        open_.pop_back();
        continue;
      }
      ++nodes_;
      list_ptr out = dst;
      out->push_back(expand((*src.list)[i])); // may push, invalidating src and dst
    }
    return true;
  }

  value_t result() override { return std::move(result_); }

private:
  value_t expand(const value_t &v) {
    if (auto *sym = std::get_if<symbol>(&v.v)) {
      auto it = subst_.find(sym->name);
      return it != subst_.end() ? it->second : v;
    }
    if (auto *l = std::get_if<list_ptr>(&v.v); l && *l) {
      auto out = std::make_shared<std::vector<value_t>>();
      out->reserve((*l)->size());
      open_.emplace_back(cursor{*l, nullptr, 0}, out);
      return value_t{out};
    }
    return v;
  }

  value_t root_;
  std::unordered_map<std::string, value_t> subst_;
  bool started_ = false;
  value_t result_;
  std::vector<std::pair<cursor, list_ptr>> open_;
};

// The data side of the evaluator step running on this thread, if any.
thread_local detail::data_step_scope *t_scope = nullptr;

// Run `walk` to completion and count its nodes into the current step.
template <class Walk> Walk &finish_walk(Walk &walk) {
  walk.step(0);
  count_data_nodes(walk.nodes());
  return walk;
}

std::optional<std::size_t> read_size_setting(cvc::state &root, const std::string &path) {
  cvc::state *node = root.findDescendant(path);
  if (!node)
    return std::nullopt;
  const std::string v = node->value();
  std::size_t n = 0;
  const auto [end, ec] = std::from_chars(v.data(), v.data() + v.size(), n);
  if (ec != std::errc{} || end != v.data() + v.size() || v.empty())
    return std::nullopt;
  return n;
}

} // namespace

// -- the walks as free functions ----------------------------------------------

std::string to_string(const value_t &val) {
  print_walk walk(val);
  return finish_walk(walk).take_string();
}

bool values_equal(const value_t &a, const value_t &b) {
  equal_walk walk(a, b);
  return finish_walk(walk).equal();
}

// -- jobs ----------------------------------------------------------------------

bool is_compound(const value_t &v) {
  if (auto *l = std::get_if<list_ptr>(&v.v))
    return *l && !(*l)->empty();
  if (auto *d = std::get_if<dict_ptr>(&v.v))
    return *d && !(*d)->empty();
  return false;
}

std::unique_ptr<data_job> make_concat_job(std::vector<value_t> parts, std::string sep) {
  return std::make_unique<concat_walk>(std::move(parts), std::move(sep));
}

std::unique_ptr<data_job> make_equal_job(value_t a, value_t b) {
  return std::make_unique<equal_walk>(std::move(a), std::move(b));
}

std::unique_ptr<data_job> make_print_job(value_t v) {
  return std::make_unique<print_walk>(std::move(v));
}

std::unique_ptr<data_job> make_copy_job(value_t v) {
  return std::make_unique<copy_walk>(std::move(v));
}

std::unique_ptr<data_job> make_substitute_job(value_t tmpl,
                                              std::unordered_map<std::string, value_t> subst) {
  return std::make_unique<substitute_walk>(std::move(tmpl), std::move(subst));
}

value_t run_to_completion(data_job &job) {
  const std::size_t before = job.nodes();
  job.step(0);
  count_data_nodes(job.nodes() - before);
  return job.result();
}

// -- settings ------------------------------------------------------------------

data_options data_options_from_state(cvc::state &root, const std::string &scheduler_id,
                                     const data_options &base) {
  data_options opts = base;
  const auto resolve = [&](const char *key, std::size_t &out) {
    const std::string suffix = std::string(".data.") + key;
    if (auto v = read_size_setting(root, std::string(scheduler::defaults_prefix) + suffix))
      out = *v;
    if (!scheduler_id.empty())
      if (auto v = read_size_setting(root, std::string(scheduler::sched_prefix) + "." +
                                               scheduler_id + suffix))
        out = *v;
  };
  resolve("step_nodes", opts.step_nodes); // 0 = no slicing
  return opts;
}

// -- running a native call's job -------------------------------------------------

value_t run_data_job(std::unique_ptr<data_job> job, std::function<value_t(value_t)> then) {
  detail::data_step_scope *scope = t_scope;
  const bool may_defer = scope && scope->step_nodes_ > 0 && scope->may_defer_ && !scope->deferred_;
  const std::size_t before = job->nodes();
  const bool done = job->step(may_defer ? scope->remaining() : 0);
  count_data_nodes(job->nodes() - before);
  if (!done) {
    scope->deferred_ = std::make_unique<detail::deferred_data_job>(
        detail::deferred_data_job{std::move(job), std::move(then)});
    return nil_value;
  }
  value_t r = job->result();
  return then ? then(std::move(r)) : r;
}

void count_data_nodes(std::size_t nodes) {
  if (nodes == 0)
    return;
  if (detail::data_step_scope *scope = t_scope) {
    scope->nodes_ += nodes;
    scope->stats_.add_data_nodes(nodes);
  }
}

// -- data_step_scope -------------------------------------------------------------

detail::data_step_scope::data_step_scope(std::size_t step_nodes, evaluation_stats &stats,
                                         release_backlog *backlog)
    : step_nodes_(step_nodes), stats_(stats), backlog_(step_nodes > 0 ? backlog : nullptr),
      outer_(t_scope), routing_(backlog_) {
  t_scope = this;
}

detail::data_step_scope::~data_step_scope() { t_scope = outer_; }

std::size_t detail::data_step_scope::remaining() const {
  return step_nodes_ > nodes_ ? step_nodes_ - nodes_ : 1;
}

void detail::data_step_scope::drain_backlog() {
  if (!backlog_ || backlog_->empty() || nodes_ >= step_nodes_)
    return;
  count_data_nodes(backlog_->drain(step_nodes_ - nodes_));
}

detail::data_step_scope *detail::data_step_scope::current() { return t_scope; }

} // namespace cvc::state_exec
