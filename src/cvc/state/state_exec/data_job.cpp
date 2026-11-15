/*
  Copyright 2026 The University of Texas at Austin

  data_job — walks over nested values (compare, print, copy, macro expansion) that keep their
             work on the heap and run a slice at a time.
*/

#include "builtin_class.h"
#include "native_stack.h"

#include <algorithm>
#include <charconv>
#include <cstdint>
#include <cstring>
#include <cvc/state/state.h>
#include <cvc/state/state_exec/async_host.h> // is_async_future
#include <cvc/state/state_exec/data_job.h>
#include <cvc/state/state_exec/detail/lock_home.h> // lock_ops, the lock objects' type names
#include <cvc/state/state_exec/evaluator.h>        // evaluation_stats
#include <cvc/state/state_exec/generator.h>
#include <cvc/state/state_exec/scheduler.h>           // the settings' key prefixes
#include <cvc/state/state_exec/stackless_evaluator.h> // detail::awaitable, kAwaitableType
#include <optional>
#include <stdexcept>
#include <typeinfo>
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

  void release_inputs(detail::release_backlog &backlog) noexcept override {
    backlog.add(std::move(root_));
  }

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
          else if constexpr (std::is_same_v<T, double>)
            out_ += format_number_shortest(arg);
          else if constexpr (std::is_same_v<T, std::string>) {
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
  concat_walk(std::vector<value_t> parts, std::string sep, const char *who)
      : parts_(std::move(parts)), sep_(std::move(sep)), who_(who) {}

  void release_inputs(detail::release_backlog &backlog) noexcept override {
    for (value_t &part : parts_)
      backlog.add(std::move(part));
  }

  bool step(std::size_t budget) override {
    std::size_t used = 0;
    while (next_ < parts_.size()) {
      if (!part_) {
        if (next_ > 0)
          append(sep_);
        if (auto *s = std::get_if<std::string>(&parts_[next_].v)) {
          append(*s);
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
      append(part_->take_string());
      part_.reset();
      ++next_;
    }
    return true;
  }

  value_t result() override { return value_t{std::move(out_)}; }

private:
  void append(const std::string &part) {
    eval_alloc_check(
        ctx_, out_.size() > SIZE_MAX - part.size() ? SIZE_MAX : out_.size() + part.size(), who_);
    out_ += part;
  }

  std::vector<value_t> parts_;
  std::string sep_;
  const char *who_;
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

  void release_inputs(detail::release_backlog &backlog) noexcept override {
    backlog.add(std::move(a_));
    backlog.add(std::move(b_));
  }

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
      eval_deadline_check(ctx_, "values_equal");
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

// The kind of internal object that must not cross a thread, or null: an async call's handle (a
// future's), the with-lock form (it holds its context's raw pointer), a lock wait's handle. Known
// by type name, and by payload type where there is one.
const char *refused_object(const data_object &obj) {
  const std::type_info &payload = obj.payload.type();
  if (obj.type_name == detail::kAwaitableType ||
      payload == typeid(std::shared_ptr<detail::awaitable>))
    return "future";
  if (obj.type_name == detail::kLockOpsType || payload == typeid(detail::lock_ops))
    return "lock form";
  if (obj.type_name == detail::kLockWaitType)
    return "lock wait";
  return nullptr;
}

// Memoized on the source node, so a shared-pointer DAG is copied in PHYSICAL time (the copy has
// the same sharing, disjoint from the original) and a cycle is copied as a cycle. A deadline
// poll per node aborts a huge payload at the time budget.
//
// copy_mode::isolate is state-data-set's copy (td 4.3): it refuses, at any depth, what cannot
// cross a thread, and shares the built-in class dicts, which are read-only (S0.1). It throws
// from step(), so a refusal in any slice leaves no result to write.
class copy_walk final : public data_job {
public:
  copy_walk(value_t v, copy_mode mode, const char *who)
      : root_(std::move(v)), mode_(mode), who_(who) {}

  void release_inputs(detail::release_backlog &backlog) noexcept override {
    backlog.add(std::move(root_));
  }

  bool step(std::size_t budget) override {
    if (!started_) {
      started_ = true;
      result_ = copy(root_, nullptr);
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
      eval_deadline_check(ctx_, who_);
      if (src.list) {
        list_ptr out = std::get<list_ptr>(dst.v);
        const value_t &e = (*src.list)[i];
        out->push_back(copy(e, "list")); // may push, invalidating src and dst
      } else {
        dict_ptr out = std::get<dict_ptr>(dst.v);
        const auto &[k, e] = (*src.dict)[i];
        out->emplace_back(k, copy(e, "dict")); // may push, invalidating src and dst
      }
    }
    return true;
  }

  value_t result() override { return std::move(result_); }

private:
  // The copy of `v`, the root (`in` null) or an element of a list or dict (`in`): a list or dict
  // is a new empty one, filled by step() through its cursor.
  value_t copy(const value_t &v, const char *in) {
    // A built-in class dict is read-only (S0.1), so both modes share it: an exception read back
    // with state-data-get can be stored again.
    if (const auto *d = std::get_if<dict_ptr>(&v.v); d && *d && detail::is_builtin_class(**d))
      return v;
    if (mode_ == copy_mode::isolate)
      isolate_refuse(v, in);
    std::optional<cursor> c = open_cursor(v);
    if (!c) {
      // empty or null lists and dicts still get their own (empty) copy
      if (auto *l = std::get_if<list_ptr>(&v.v); l && *l)
        return make_list();
      if (auto *d = std::get_if<dict_ptr>(&v.v); d && *d)
        return make_dict();
      // Shared: scalars, strings, symbols and bytes are values; a data_object is shared under its
      // read-only contract; share mode also shares callables, generators and handles.
      return v;
    }
    const void *key = c->list ? static_cast<const void *>(c->list.get())
                              : static_cast<const void *>(c->dict.get());
    if (auto found = memo_.find(key); found != memo_.end())
      return found->second.copy; // shared node / cycle already copied -> reuse (bounds the walk)
    if (mode_ == copy_mode::isolate && c->dict)
      isolate_dict(v, in); // once per dict: a dict reached again was checked
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
    // Registered before its elements are copied, with its source held (memo_entry).
    memo_.emplace(key, memo_entry{c->list ? std::shared_ptr<const void>(c->list)
                                          : std::shared_ptr<const void>(c->dict),
                                  out});
    open_.emplace_back(std::move(*c), out);
    return out;
  }

  [[noreturn]] void refuse(const char *kind, const char *in) const {
    std::string what = std::string(who_) + ": cannot store " +
                       (std::strchr("aeiou", kind[0]) ? "an " : "a ") + kind;
    if (in)
      what += std::string(" (nested in a ") + in + ")";
    throw std::runtime_error(what);
  }

  // Isolate mode, before `v` is copied: throws for a callable, a generator or an internal object
  // (refused_object).
  void isolate_refuse(const value_t &v, const char *in) const {
    if (std::holds_alternative<closure_ptr>(v.v))
      refuse("closure", in);
    if (std::holds_alternative<native_fn>(v.v))
      refuse("native function", in);
    if (std::holds_alternative<generator_ptr>(v.v))
      refuse("generator", in);
    if (const auto *obj = std::get_if<data_object_ptr>(&v.v); obj && *obj)
      if (const char *kind = refused_object(**obj))
        refuse(kind, in);
  }

  // Isolate mode, a dict seen for the first time: an async call's future is refused, and so is
  // an instance of a user class, whose methods hold the defining program's live environment
  // (S0-Q4). An instance has the shape defclass and make_exception give it: "__class__" first,
  // holding a class dict whose "__name__" is first (a class of any size). A built-in exception's
  // instance is copied, its class shared, and any other dict is data.
  void isolate_dict(const value_t &v, const char *in) const {
    if (is_async_future(v))
      refuse("future", in);
    const ordered_map &d = *std::get<dict_ptr>(v.v); // not empty: it has a cursor
    if (d.front().first != "__class__")
      return;
    const auto *cls = std::get_if<dict_ptr>(&d.front().second.v);
    if (cls && *cls && !(*cls)->empty() && (*cls)->front().first == "__name__" &&
        !detail::is_builtin_class(**cls))
      refuse("object instance", in);
  }

  value_t root_;
  copy_mode mode_;
  const char *who_; // a string literal: the deadline's and the refusals' label
  bool started_ = false;
  value_t result_;
  // A source list or dict's copy, by the source's address. The entry holds the source as well:
  // one another process drops between two slices stays allocated until the copy is done, so its
  // address can't come back as a new list or dict that the walk would take for it (R0a-mem-2).
  struct memo_entry {
    std::shared_ptr<const void> source;
    value_t copy;
  };
  std::unordered_map<const void *, memo_entry> memo_;
  std::vector<std::pair<cursor, value_t>> open_;
};

// -- substitute --------------------------------------------------------------

class substitute_walk final : public data_job {
public:
  substitute_walk(value_t tmpl, std::unordered_map<std::string, value_t> subst)
      : root_(std::move(tmpl)), subst_(std::move(subst)) {}

  void release_inputs(detail::release_backlog &backlog) noexcept override {
    backlog.add(std::move(root_));
    for (auto &entry : subst_)
      backlog.add(std::move(entry.second));
  }

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

// -- call ----------------------------------------------------------------------

// One application of a function, as a job, so a native function can have a closure called
// through the evaluator (call_function).
class call_walk final : public data_job {
public:
  call_walk(value_t fn, std::vector<value_t> args, const char *who)
      : fn_(std::move(fn)), args_(std::move(args)), who_(who) {}

  void release_inputs(detail::release_backlog &backlog) noexcept override {
    backlog.add(std::move(fn_));
    for (value_t &arg : args_)
      backlog.add(std::move(arg));
  }

  bool step(std::size_t) override {
    if (!asked_) {
      asked_ = true;
      ++nodes_;
      call_ = call_request{&fn_, args_, who_};
    }
    return returned_;
  }

  void call_returned(value_t value) override {
    result_ = std::move(value);
    returned_ = true;
  }

  value_t result() override { return std::move(result_); }

  bool forwards_call() const override { return true; }

  void held_values(const std::function<void(const value_t &)> &visit) const override {
    visit(fn_);
    for (const value_t &a : args_)
      visit(a);
    visit(result_);
  }

  std::size_t held_bytes() const override {
    return sizeof(*this) + args_.capacity() * sizeof(value_t);
  }

private:
  value_t fn_;
  std::vector<value_t> args_;
  const char *who_;
  bool asked_ = false, returned_ = false;
  value_t result_;
};

// A generator advanced a node at a time (make_generator_job).
class generator_walk final : public data_job {
public:
  generator_walk(generator_ptr gen, bool collect) : gen_(std::move(gen)), collect_(collect) {}

  bool step(std::size_t nodes) override;

  value_t result() override { return make_list(std::move(values_)); }

  // The generator itself is the waiting call's argument, which its frame holds.
  void held_values(const std::function<void(const value_t &)> &visit) const override {
    for (const value_t &v : values_)
      visit(v);
  }

  std::size_t held_bytes() const override {
    return sizeof(*this) + values_.capacity() * sizeof(value_t);
  }

private:
  // Keep a value it yielded. True when the walk goes on (collect).
  bool keep(value_t v) {
    if (collect_)
      eval_alloc_check(ctx_, alloc_bytes(values_.size() + 1, sizeof(value_t)), "collect");
    values_.push_back(std::move(v));
    return collect_;
  }

  generator_ptr gen_;
  bool collect_;
  bool advancing_ = false; // a DSL generator is part way to its next yield
  std::vector<value_t> values_;
};

bool generator_walk::step(std::size_t nodes) {
  // Whoever steps a job binds its context (run_to_completion makes one when there is none).
  exec_context &ctx = *ctx_;
  generator &g = *gen_;
  const std::size_t start = nodes_;
  auto spent = [&] { return nodes > 0 && nodes_ - start >= nodes; };
  for (;;) {
    if (g.exhausted)
      return true;
    if (g.native_step) { // range and friends: a value is a node
      if (spent())
        return false;
      eval_deadline_check(&ctx, collect_ ? "collect" : "next"); // (collect (range 0 1e11))
      std::optional<value_t> v = g.native_step();
      ++nodes_;
      if (!v) {
        g.exhausted = true;
        return true;
      }
      if (!keep(std::move(*v)))
        return true;
      continue;
    }
    if (!advancing_) {
      g.state.yielded = false;
      advancing_ = true;
      // Deep-data work in the body slices like the process's own.
      if (detail::data_step_scope *scope = ctx.data)
        g.state.data_step_nodes = scope->step_nodes();
    }
    while (!g.state.done && !g.state.yielded) {
      if (spent())
        return false;
      eval_deadline_check(&ctx, "generator");
      {
        detail::native_nesting level(ctx, "generator"); // its own evaluator, on this stack
        try {
          g.evaluator.step(g.state, ctx); // in this job's context: its deadline, its registry
        } catch (...) {
          if (g.state.uncaught) { // the body raised: the generator is over
            g.exhausted = true;
            advancing_ = false;
          }
          throw;
        }
      }
      ++nodes_;
    }
    advancing_ = false;
    if (g.state.yielded) {
      // A yield that was also the body's last expression counts; the next advance ends it.
      if (!keep(g.state.result))
        return true;
      continue;
    }
    g.exhausted = true;
    return true;
  }
}

// Run `walk` to completion in `ctx` (null: nothing in force) and count its nodes into the step
// running there, if any.
template <class Walk> Walk &finish_walk(Walk &walk, exec_context *ctx) {
  walk.bind(ctx);
  walk.step(0);
  count_data_nodes(ctx, walk.nodes());
  return walk;
}

std::optional<std::size_t> read_size_setting(cvc::state &root, const std::string &path) {
  const std::optional<std::string> value = detail::setting_value(root, path); // held (R0-mid)
  if (!value)
    return std::nullopt;
  const std::string &v = *value;
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
  return finish_walk(walk, nullptr).take_string();
}

bool values_equal(const value_t &a, const value_t &b, exec_context *ctx) {
  equal_walk walk(a, b);
  return finish_walk(walk, ctx).equal();
}

// -- jobs ----------------------------------------------------------------------

bool is_compound(const value_t &v) {
  if (auto *l = std::get_if<list_ptr>(&v.v))
    return *l && !(*l)->empty();
  if (auto *d = std::get_if<dict_ptr>(&v.v))
    return *d && !(*d)->empty();
  return false;
}

std::unique_ptr<data_job> make_concat_job(std::vector<value_t> parts, std::string sep,
                                          const char *who) {
  return std::make_unique<concat_walk>(std::move(parts), std::move(sep), who);
}

std::unique_ptr<data_job> make_equal_job(value_t a, value_t b) {
  return std::make_unique<equal_walk>(std::move(a), std::move(b));
}

std::unique_ptr<data_job> make_print_job(value_t v) {
  return std::make_unique<print_walk>(std::move(v));
}

std::unique_ptr<data_job> make_copy_job(value_t v, copy_mode mode, const char *who) {
  return std::make_unique<copy_walk>(std::move(v), mode, who);
}

std::unique_ptr<data_job> make_substitute_job(value_t tmpl,
                                              std::unordered_map<std::string, value_t> subst) {
  return std::make_unique<substitute_walk>(std::move(tmpl), std::move(subst));
}

std::unique_ptr<data_job> make_generator_job(generator_ptr gen, bool collect) {
  if (!gen)
    throw std::runtime_error(collect ? "collect: expected generator" : "next: expected generator");
  return std::make_unique<generator_walk>(std::move(gen), collect);
}

namespace {
// Make a call a job running here (not on an evaluator) waits for, in `ctx`. A native function's
// own deep-data work must run here too, or its result would be lost: deferral is off for the call.
value_t call_here(exec_context &ctx, const data_job::call_request &call) {
  if (const auto *fn = std::get_if<native_fn>(&call.fn->v)) {
    struct no_defer {
      detail::data_step_scope *scope;
      bool was = scope && scope->defer_allowed();
      explicit no_defer(detail::data_step_scope *s) : scope(s) {
        if (was)
          scope->allow_defer(false);
      }
      ~no_defer() {
        if (was)
          scope->allow_defer(true);
      }
    } guard(ctx.data);
    detail::native_nesting level(ctx, call.who);
    return (*fn)(call_args(call.args, &ctx));
  }
  if (std::holds_alternative<closure_ptr>(call.fn->v))
    throw std::runtime_error(std::string(call.who) +
                             ": calling a DSL function needs a running program (an evaluator)");
  throw std::runtime_error(std::string(call.who) + ": not a function: " + call.fn->type_name());
}
} // namespace

value_t run_to_completion(exec_context *ctx, data_job &job) {
  std::optional<exec_context> own; // a call from outside any state_exec work: a run of its own
  exec_context &c = ctx ? *ctx : own.emplace();
  for (;;) {
    const std::size_t before = job.nodes();
    job.bind(&c);
    const bool done = job.step(0);
    count_data_nodes(&c, job.nodes() - before);
    if (done)
      return job.result();
    std::optional<data_job::call_request> call = job.take_call();
    if (!call)
      throw std::logic_error("data_job: stopped with neither a result nor a call");
    job.call_returned(call_here(c, *call));
  }
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

value_t run_data_job(exec_context *ctx, std::unique_ptr<data_job> job,
                     std::function<value_t(value_t)> then) {
  detail::data_step_scope *scope = ctx ? ctx->data : nullptr;
  if (!scope || !scope->may_defer_ || scope->deferred_) {
    value_t r = run_to_completion(ctx, *job);
    return then ? then(std::move(r)) : r;
  }
  // One slice here (without slicing, all of it), making the native calls the job needs on the
  // way if this is the step's own native call. It waits for the evaluator when the slice runs out,
  // for a DSL function, for a native one called from deeper, and for the work a native function it
  // called left of its own.
  const bool slicing = scope->step_nodes_ > 0;
  std::unique_ptr<detail::deferred_data_job> waits_on;
  for (;;) {
    const std::size_t before = job->nodes();
    job->bind(ctx);
    const bool done = job->step(slicing ? scope->remaining() : 0);
    count_data_nodes(ctx, job->nodes() - before);
    if (done) {
      value_t r = job->result();
      return then ? then(std::move(r)) : r;
    }
    const data_job::call_request *pending = job->pending_call();
    const native_fn *fn = pending ? std::get_if<native_fn>(&pending->fn->v) : nullptr;
    if (!fn || !scope->calls_here())
      break;
    const data_job::call_request call = *job->take_call();
    value_t value;
    {
      detail::native_nesting level(*ctx, call.who);
      value = (*fn)(call_args(call.args, ctx));
    }
    if (scope->deferred_) {
      waits_on = std::move(scope->deferred_);
      break;
    }
    job->call_returned(std::move(value));
  }
  job->own_inputs();
  eval_alloc_note(ctx, job->held_bytes()); // it outlives the call (heap.h)
  scope->deferred_ = std::make_unique<detail::deferred_data_job>(
      detail::deferred_data_job{std::move(job), std::move(then), std::move(waits_on)});
  return nil_value();
}

value_t call_function(exec_context *ctx, const value_t &fn, std::span<const value_t> args,
                      const char *who) {
  if (const auto *native = std::get_if<native_fn>(&fn.v)) {
    // Here, unless an evaluator step can make the call at its own level instead.
    const detail::data_step_scope *scope = ctx ? ctx->data : nullptr;
    if (!scope || !scope->may_defer_ || scope->deferred_ || scope->calls_here()) {
      std::optional<exec_context> own; // a call from outside any state_exec work
      exec_context &c = ctx ? *ctx : own.emplace();
      detail::native_nesting level(c, who);
      return (*native)(call_args(args, &c));
    }
  } else if (const auto *cls = std::get_if<closure_ptr>(&fn.v); !cls || !*cls) {
    throw std::runtime_error(std::string(who) + ": not a function: " + fn.type_name());
  }
  return run_data_job(
      ctx, std::make_unique<call_walk>(fn, std::vector<value_t>(args.begin(), args.end()), who));
}

value_t call_function(exec_context *ctx, const value_t &fn, std::span<const value_t> args,
                      const char *who, std::function<value_t(value_t)> then) {
  const auto *cls = std::get_if<closure_ptr>(&fn.v);
  if (!std::holds_alternative<native_fn>(fn.v) && !(cls && *cls))
    throw std::runtime_error(std::string(who) + ": not a function: " + fn.type_name());
  return run_data_job(
      ctx, std::make_unique<call_walk>(fn, std::vector<value_t>(args.begin(), args.end()), who),
      std::move(then));
}

void count_data_nodes(exec_context *ctx, std::size_t nodes) {
  if (nodes == 0 || !ctx)
    return;
  if (detail::data_step_scope *scope = ctx->data) {
    scope->nodes_ += nodes;
    scope->stats_.add_data_nodes(nodes);
  }
}

// -- native_nesting --------------------------------------------------------------

detail::native_nesting::native_nesting(exec_context &ctx, const char *who) : ctx_(ctx) {
  if (ctx.native_depth >= kMaxNativeNesting || native_stack_left(ctx) < kNativeStackReserve)
    throw std::runtime_error(std::string(who) + ": native calls nested too deeply (" +
                             std::to_string(ctx.native_depth) +
                             " levels; a value or class that refers to itself?)");
  ++ctx.native_depth;
}

detail::native_nesting::~native_nesting() { --ctx_.native_depth; }

// -- data_step_scope -------------------------------------------------------------

detail::data_step_scope::data_step_scope(exec_context &ctx, std::size_t step_nodes,
                                         evaluation_stats &stats, release_backlog *backlog)
    : ctx_(ctx), step_nodes_(step_nodes), stats_(stats),
      backlog_(step_nodes > 0 ? backlog : nullptr), native_depth_(ctx.native_depth),
      outer_(ctx.data) {
  ctx.data = this;
}

detail::data_step_scope::~data_step_scope() { ctx_.data = outer_; }

std::size_t detail::data_step_scope::remaining() const {
  return step_nodes_ > nodes_ ? step_nodes_ - nodes_ : 1;
}

bool detail::data_step_scope::calls_here() const {
  // The evaluator applies a native function without a level of native nesting, so the depth is
  // the step's own only inside that function.
  return ctx_.native_depth == native_depth_ && (step_nodes_ == 0 || nodes_ < step_nodes_);
}

void detail::data_step_scope::drain_backlog() {
  if (!backlog_ || backlog_->empty() || nodes_ >= step_nodes_)
    return;
  count_data_nodes(&ctx_, backlog_->drain(step_nodes_ - nodes_));
}

} // namespace cvc::state_exec
