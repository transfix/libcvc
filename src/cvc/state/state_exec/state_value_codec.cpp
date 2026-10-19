/*
  Copyright 2026 The University of Texas at Austin

  state_value_codec — round-trip DSL values and evaluator state through
                      cvc::state subtrees.
*/

#include <algorithm>
#include <charconv>
#include <cvc/state/state.h>
#include <cvc/state/state_exec/parser.h>
#include <cvc/state/state_exec/state_value_codec.h>
#include <cvc/state/state_list.h>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace cvc::state_exec {

// ---------------------------------------------------------------------------
// helpers
// ---------------------------------------------------------------------------
namespace {

const char *TYPE_KEY = "__type__";

void set_type(cvc::state &n, const char *t) { n(TYPE_KEY).value(std::string(t)); }

// Use findDescendant for read-only lookup — children() returns full
// qualified names (e.g. "parent.child") which makes direct name
// comparison unreliable, and operator() auto-creates missing nodes.
std::string get_type(cvc::state &n) {
  cvc::state *child = n.findDescendant(TYPE_KEY);
  if (child)
    return child->value();
  return "";
}

bool has_child(cvc::state &n, const std::string &name) { return n.findDescendant(name) != nullptr; }

// SE-3: a `bytes` value holds arbitrary octets (NUL, high bytes, non-UTF-8), which a text-oriented
// state serialization could mangle — so the codec stores bytes as base64 (RFC 4648, ASCII-safe).
std::string b64_encode(const std::string &in) {
  static const char *T = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::string out;
  out.reserve(((in.size() + 2) / 3) * 4);
  auto b = [&](std::size_t i) { return static_cast<unsigned>(static_cast<unsigned char>(in[i])); };
  std::size_t i = 0;
  for (; i + 3 <= in.size(); i += 3) {
    const unsigned n = (b(i) << 16) | (b(i + 1) << 8) | b(i + 2);
    out += T[(n >> 18) & 63];
    out += T[(n >> 12) & 63];
    out += T[(n >> 6) & 63];
    out += T[n & 63];
  }
  if (i + 1 == in.size()) {
    const unsigned n = b(i) << 16;
    out += T[(n >> 18) & 63];
    out += T[(n >> 12) & 63];
    out += "==";
  } else if (i + 2 == in.size()) {
    const unsigned n = (b(i) << 16) | (b(i + 1) << 8);
    out += T[(n >> 18) & 63];
    out += T[(n >> 12) & 63];
    out += T[(n >> 6) & 63];
    out += '=';
  }
  return out;
}
std::string b64_decode(const std::string &in) {
  auto val = [](char c) -> int {
    if (c >= 'A' && c <= 'Z')
      return c - 'A';
    if (c >= 'a' && c <= 'z')
      return c - 'a' + 26;
    if (c >= '0' && c <= '9')
      return c - '0' + 52;
    if (c == '+')
      return 62;
    if (c == '/')
      return 63;
    return -1; // padding '=' or any non-base64 byte
  };
  std::string out;
  int buf = 0, bits = 0;
  for (char c : in) {
    const int v = val(c);
    if (v < 0)
      continue;
    buf = (buf << 6) | v;
    bits += 6;
    if (bits >= 8) {
      bits -= 8;
      out += static_cast<char>((buf >> bits) & 0xFF);
    }
  }
  return out;
}

// Nested lists and dicts are written and read with explicit stacks, not native recursion, and
// the state subtree they make is kept shallow too: cvc::state recurses once per TREE level on
// every write (fullName() builds the path parent by parent, childChanged() climbs to the root),
// so a list nested 100000 deep must not become a subtree 200000 deep. A list or dict
// kSpillDepth levels below where its stretch of nesting began is written into the __spill__
// table under the node encode_value was called on, as an entry of its own, and the place it
// would have gone holds a reference: __type__ "spill_ref", value the entry's index. Its own
// nesting starts again from that entry. Values at most kSpillDepth levels deep are written
// exactly as before, with no table.
//
// 8 levels make at most ~30 tree levels per stretch (a dict level is 3: __entries__, the
// index, __val__). A write that deep, under a snapshot root ~5 levels down, fits a 64 KiB
// stack (wasm's); with 16, the dicts' ~55 did not.
//
// Closures and the scopes they capture are part of the same walk. A closure's scopes are not
// written inside it: each scope the walk reaches is written once, as an entry of the __envs__
// table under the node the walk started at (its bindings, and an __outer__ reference to the
// enclosing scope), and a closure's __env__ holds a reference: __type__ "env_ref", value the
// entry's index. A closure whose scope binds the closure itself (any function made by defun)
// refers back to an entry already written instead of writing itself forever, and closures that
// share a scope share its entry, and the environment it decodes to.
//
// Snapshots from before wrote each closure's scope chain inside it (__type__ "env_chain", the
// scopes listed innermost first), and every closure body expression and scope binding value as a
// walk of its own, with its own __spill__ table. Those still decode: a closure whose __env__ is
// missing or an env_chain is one of them. (A closure written now always has an __env__: an
// env_ref, or nil.)
constexpr std::size_t kSpillDepth = 8;
const char *SPILL_KEY = "__spill__";
const char *ENVS_KEY = "__envs__";

// The non-null list or dict a value holds, or null.
const void *container_of(const value_t &v) {
  if (auto *l = std::get_if<list_ptr>(&v.v))
    return l->get();
  if (auto *d = std::get_if<dict_ptr>(&v.v))
    return d->get();
  return nullptr;
}

// Write a value that is not a non-null list, dict or closure.
void encode_leaf(cvc::state &node, const value_t &val);

// Read a value whose __type__ is `type`, which is not "list", "dict", "closure" or "spill_ref".
value_t decode_leaf(cvc::state &node, const std::string &type);

// Start the element list `items` of something with `size` elements. The length is known up
// front, so it is written once, not once per element as state_list::push_back would (every
// write signals each ancestor). An empty one gets the "0" that state_list's constructor writes.
void open_items(cvc::state &items, std::size_t size) {
  if (size > 0)
    items("__len__").value(size);
  else
    (void)state_list(items);
}

// The index an env_ref or spill_ref holds, if it is a whole number below `size`.
std::optional<std::size_t> ref_index(const std::string &ref, std::size_t size) {
  std::size_t i = 0;
  const auto [end, ec] = std::from_chars(ref.data(), ref.data() + ref.size(), i);
  if (ec != std::errc{} || end != ref.data() + ref.size() || i >= size)
    return std::nullopt;
  return i;
}

// One encode walk, from the node encode_value or encode_environment was called on.
class value_writer {
public:
  explicit value_writer(cvc::state &root) : root_(root) {}

  // Write `v` at `node`, and everything it reaches.
  void write(cvc::state &node, const value_t &v) {
    begin(node, v, 0);
    drain();
  }

  // Write a reference to `env` (nil for none) at `node`, and every scope it reaches.
  void write_env(cvc::state &node, const environment_ptr &env) {
    env_ref(node, env);
    drain();
  }

private:
  enum class kind { list, dict, body, scope };

  // Something whose elements are being written under `items`.
  struct frame {
    kind what;
    const value_t *val = nullptr;                                  // list, dict
    const std::vector<value_t> *body = nullptr;                    // body: a closure's expressions
    std::vector<std::pair<std::string, const value_t *>> bindings; // scope
    cvc::state *items = nullptr;
    std::size_t next = 0;
    std::size_t size = 0;
    std::size_t nest = 0; // how many levels below its stretch's start it is
  };

  // Write `v` into `node`: a leaf now, anything with elements by its type and a frame for them.
  void begin(cvc::state &node, const value_t &v, std::size_t nest) {
    if (const auto *cls = std::get_if<closure_ptr>(&v.v); cls && *cls) {
      begin_closure(node, **cls, nest);
      return;
    }
    const void *c = container_of(v);
    if (!c) {
      encode_leaf(node, v);
      return;
    }
    if (!open_.insert(c).second)
      throw std::runtime_error("encode_value: the value contains itself");
    cvc::state *target = &node;
    if (nest >= kSpillDepth) {
      if (!spill_)
        spill_.emplace(root_(SPILL_KEY));
      set_type(node, "spill_ref");
      node.value(std::to_string(spill_->size()));
      target = &spill_->push_back();
      nest = 0;
    }
    const bool is_list = std::holds_alternative<list_ptr>(v.v);
    set_type(*target, is_list ? "list" : "dict");
    cvc::state &items = (*target)(is_list ? "__elems__" : "__entries__");
    const std::size_t size =
        is_list ? std::get<list_ptr>(v.v)->size() : std::get<dict_ptr>(v.v)->size();
    open_items(items, size);
    frame f{is_list ? kind::list : kind::dict};
    f.val = &v;
    f.items = &items;
    f.size = size;
    f.nest = nest;
    stack_.push_back(std::move(f));
  }

  void begin_closure(cvc::state &node, const closure &cls, std::size_t nest) {
    set_type(node, "closure");
    state_list params(node("__params__"));
    for (const symbol &p : cls.params)
      params.push_back().value(p.name);
    node("__variadic__").value(std::string(cls.variadic ? "1" : "0"));
    env_ref(node("__env__"), cls.env_snapshot);
    cvc::state &items = node("__body__");
    open_items(items, cls.body.size());
    if (cls.body.empty())
      return;
    frame f{kind::body};
    f.body = &cls.body;
    f.items = &items;
    f.size = cls.body.size();
    f.nest = nest;
    stack_.push_back(std::move(f));
  }

  // Write a reference to `env`'s entry in the __envs__ table at `node`, giving it an entry
  // (written by drain()) the first time.
  void env_ref(cvc::state &node, const environment_ptr &env) {
    if (!env) {
      set_type(node, "nil");
      return;
    }
    const auto [it, fresh] = env_index_.try_emplace(env.get(), scopes_.size());
    if (fresh) {
      if (!envs_)
        envs_.emplace(root_(ENVS_KEY));
      envs_->push_back();
      scopes_.push_back(env);
    }
    set_type(node, "env_ref");
    node.value(std::to_string(it->second));
  }

  // Start writing the next scope that has an entry but no contents yet.
  void begin_scope() {
    const std::size_t i = scopes_written_++;
    const environment_ptr env = scopes_[i]; // a copy: env_ref() may grow scopes_
    cvc::state &entry = envs_->at(i);
    if (env->outer)
      env_ref(entry("__outer__"), env->outer);
    frame f{kind::scope};
    for (const auto &[name, val] : env->bindings)
      // Native functions can't be serialized; a restore registers the builtins again.
      if (!std::holds_alternative<native_fn>(val.v))
        f.bindings.emplace_back(name, &val);
    std::sort(f.bindings.begin(), f.bindings.end(),
              [](const auto &a, const auto &b) { return a.first < b.first; });
    cvc::state &items = entry("__bindings__");
    open_items(items, f.bindings.size());
    f.items = &items;
    f.size = f.bindings.size();
    if (f.size > 0)
      stack_.push_back(std::move(f));
  }

  void drain() {
    for (;;) {
      while (!stack_.empty()) {
        frame &f = stack_.back();
        if (f.next == f.size) {
          if (f.what == kind::list || f.what == kind::dict)
            open_.erase(container_of(*f.val));
          stack_.pop_back();
          continue;
        }
        const std::size_t i = f.next++;
        const std::size_t nest = f.nest + 1;
        cvc::state &slot = state_list(*f.items).at(i);
        // begin() may push, invalidating f: everything it needs is read from f first.
        switch (f.what) {
        case kind::list:
          begin(slot, (*std::get<list_ptr>(f.val->v))[i], nest);
          break;
        case kind::dict: {
          const auto &[key, val] = (*std::get<dict_ptr>(f.val->v))[i];
          slot("__key__").value(key);
          begin(slot("__val__"), val, nest);
          break;
        }
        case kind::body:
          begin(slot, (*f.body)[i], nest);
          break;
        case kind::scope: {
          const value_t *val = f.bindings[i].second;
          slot("__key__").value(f.bindings[i].first);
          begin(slot("__val__"), *val, 0);
          break;
        }
        }
      }
      if (scopes_written_ == scopes_.size())
        return;
      begin_scope();
    }
  }

  cvc::state &root_;
  std::vector<frame> stack_;
  std::unordered_set<const void *> open_; // the lists and dicts on stack_
  std::optional<state_list> spill_;
  std::optional<state_list> envs_;
  std::unordered_map<const environment *, std::size_t> env_index_;
  std::vector<environment_ptr> scopes_; // by entry index; kept alive while the walk runs
  std::size_t scopes_written_ = 0;
};

// One decode walk, from the node decode_value or decode_environment was called on.
class value_reader {
public:
  explicit value_reader(cvc::state &root) : root_(root) {}

  value_t read(cvc::state &node) {
    value_t out;
    begin(node, out, &root_);
    drain();
    return out;
  }

  environment_ptr read_env(cvc::state &node) {
    environment_ptr env = env_at(node, &root_);
    drain();
    return env;
  }

private:
  enum class kind { list, dict, body, scope };

  // Something whose elements are being read from `items`. `root` is where the walk that wrote
  // them started (its __spill__ and __envs__ tables); `own_roots` marks an older snapshot's body
  // expressions and binding values, each written as a walk of its own.
  struct frame {
    kind what;
    cvc::state *items;
    std::size_t next;
    std::size_t size;
    std::vector<value_t> *list; // list, body
    ordered_map *dict;          // dict
    environment *env;           // scope
    cvc::state *root;
    bool own_roots;
  };

  struct spill_table {
    std::optional<state_list> entries;
    std::vector<bool> used; // each entry is referenced once; a second reference is corrupt
  };

  // An __envs__ entry whose environment exists (something refers to it) but is not filled yet.
  struct pending_scope {
    environment_ptr env;
    cvc::state *entry;
    cvc::state *root;
  };

  void push(kind what, cvc::state &items, std::vector<value_t> *list, ordered_map *dict,
            environment *env, cvc::state *root, bool own_roots) {
    const std::size_t n = state_list(items).size();
    if (list)
      list->reserve(n);
    else if (dict)
      dict->reserve(n);
    stack_.push_back({what, &items, 0, n, list, dict, env, root, own_roots});
  }

  // Read the value at `node` into `out`: a leaf now, anything with elements as an empty value
  // that a frame fills.
  void begin(cvc::state &node, value_t &out, cvc::state *root) {
    cvc::state *src = &node;
    std::string type = get_type(node);
    if (type == "spill_ref") {
      spill_table &t = spills_[root];
      if (!t.entries) {
        if (!has_child(*root, SPILL_KEY))
          throw std::runtime_error("decode_value: spill_ref without a spill table");
        t.entries.emplace((*root)(SPILL_KEY));
        t.used.assign(t.entries->size(), false);
      }
      const std::string ref = node.value();
      const std::optional<std::size_t> i = ref_index(ref, t.used.size());
      if (!i || t.used[*i])
        throw std::runtime_error("decode_value: bad spill_ref: " + ref);
      t.used[*i] = true;
      src = &t.entries->at(*i);
      type = get_type(*src);
      if (type != "list" && type != "dict")
        throw std::runtime_error("decode_value: spill entry " + ref + " is not a list or dict");
    }
    if (type == "list") {
      auto elems = std::make_shared<std::vector<value_t>>();
      out = value_t(elems);
      if (has_child(*src, "__elems__"))
        push(kind::list, (*src)("__elems__"), elems.get(), nullptr, nullptr, root, false);
    } else if (type == "dict") {
      auto entries = std::make_shared<ordered_map>();
      out = value_t(entries);
      if (has_child(*src, "__entries__"))
        push(kind::dict, (*src)("__entries__"), nullptr, entries.get(), nullptr, root, false);
    } else if (type == "closure") {
      begin_closure(*src, out, root);
    } else {
      out = decode_leaf(*src, type);
    }
  }

  void begin_closure(cvc::state &node, value_t &out, cvc::state *root) {
    auto cls = std::make_shared<closure>();
    out = value_t(cls);
    if (has_child(node, "__params__")) {
      state_list params(node("__params__"));
      for (std::size_t i = 0; i < params.size(); ++i)
        cls->params.push_back(symbol{params.at(i).value()});
    }
    if (has_child(node, "__variadic__"))
      cls->variadic = node("__variadic__").value() == "1";
    // Written now: an __env__ that is an env_ref or nil. Anything else is an older snapshot.
    bool current = false;
    if (has_child(node, "__env__")) {
      cvc::state &env = node("__env__");
      const std::string type = get_type(env);
      current = type != "env_chain";
      cls->env_snapshot = env_at(env, root);
    }
    if (has_child(node, "__body__"))
      push(kind::body, node("__body__"), &cls->body, nullptr, nullptr, root, !current);
  }

  // The environment the reference at `node` names: an env_ref into `root`'s __envs__ table
  // (created on first reference, filled by drain()), an older snapshot's inline env_chain, or
  // null.
  environment_ptr env_at(cvc::state &node, cvc::state *root) {
    const std::string type = get_type(node);
    if (type.empty() || type == "nil")
      return nullptr;
    if (type == "env_ref") {
      std::vector<environment_ptr> &table = envs_[root];
      cvc::state *envs = root->findDescendant(ENVS_KEY);
      if (!envs)
        throw std::runtime_error("decode_value: env_ref without an __envs__ table");
      if (table.empty())
        table.resize(state_list(*envs).size());
      const std::string ref = node.value();
      const std::optional<std::size_t> i = ref_index(ref, table.size());
      if (!i)
        throw std::runtime_error("decode_value: bad env_ref: " + ref);
      if (!table[*i]) {
        table[*i] = std::make_shared<environment>();
        pending_.push_back({table[*i], &state_list(*envs).at(*i), root});
      }
      return table[*i];
    }
    // An older snapshot's chain, innermost scope first. Each binding was a walk of its own.
    if (!has_child(node, "__scopes__"))
      return nullptr;
    state_list scopes(node("__scopes__"));
    std::vector<environment_ptr> chain;
    chain.reserve(scopes.size());
    for (std::size_t i = 0; i < scopes.size(); ++i) {
      auto env = std::make_shared<environment>();
      cvc::state &scope = scopes.at(i);
      if (has_child(scope, "__bindings__"))
        push(kind::scope, scope("__bindings__"), nullptr, nullptr, env.get(), nullptr, true);
      if (!chain.empty())
        chain.back()->outer = env;
      chain.push_back(std::move(env));
    }
    return chain.empty() ? nullptr : chain.front();
  }

  void drain() {
    for (;;) {
      while (!stack_.empty()) {
        frame &f = stack_.back();
        if (f.next == f.size) {
          stack_.pop_back();
          continue;
        }
        cvc::state &item = state_list(*f.items).at(f.next++);
        // begin() may push, invalidating f: everything it needs is read from f first.
        switch (f.what) {
        case kind::list:
        case kind::body: {
          cvc::state *root = f.own_roots ? &item : f.root;
          value_t &slot = f.list->emplace_back();
          begin(item, slot, root);
          break;
        }
        case kind::dict: {
          cvc::state *root = f.root;
          std::string key = item("__key__").value();
          value_t &slot = f.dict->emplace_back(std::move(key), value_t{}).second;
          begin(item("__val__"), slot, root);
          break;
        }
        case kind::scope: {
          const bool own_roots = f.own_roots;
          cvc::state *root = f.root;
          value_t &slot = f.env->bindings[item("__key__").value()]; // a stable reference
          cvc::state &val = item("__val__");
          begin(val, slot, own_roots ? &val : root);
          break;
        }
        }
      }
      if (filled_ == pending_.size())
        break;
      const pending_scope p = pending_[filled_++]; // a copy: env_at() may grow pending_
      if (has_child(*p.entry, "__outer__"))
        p.env->outer = env_at((*p.entry)("__outer__"), p.root);
      if (has_child(*p.entry, "__bindings__"))
        push(kind::scope, (*p.entry)("__bindings__"), nullptr, nullptr, p.env.get(), p.root, false);
    }
    check_chains();
  }

  // An __outer__ reference may only lead outward: a scope chain that comes back to itself would
  // make every lookup through it loop forever.
  void check_chains() {
    std::unordered_map<const environment *, bool> done; // false: on the chain being walked
    for (const auto &[root, table] : envs_)
      for (const environment_ptr &start : table) {
        std::vector<const environment *> path;
        for (const environment *e = start.get(); e; e = e->outer.get()) {
          const auto [it, fresh] = done.try_emplace(e, false);
          if (!fresh) {
            if (it->second)
              break;
            for (auto &[r, t] : envs_) // unlink them all, or the loop would never be freed
              for (const environment_ptr &env : t)
                if (env)
                  env->outer.reset();
            throw std::runtime_error("decode_value: an __outer__ chain loops back on itself");
          }
          path.push_back(e);
        }
        for (const environment *e : path)
          done[e] = true;
      }
  }

  cvc::state &root_;
  std::vector<frame> stack_;
  std::unordered_map<cvc::state *, spill_table> spills_;                // by walk root
  std::unordered_map<cvc::state *, std::vector<environment_ptr>> envs_; // by walk root
  std::vector<pending_scope> pending_;
  std::size_t filled_ = 0; // pending_ scopes filled
};

} // namespace

// ---------------------------------------------------------------------------
// encode_value / decode_value
// ---------------------------------------------------------------------------

void encode_value(cvc::state &root, const value_t &val) { value_writer(root).write(root, val); }

value_t decode_value(cvc::state &root) { return value_reader(root).read(root); }

namespace {

void encode_leaf(cvc::state &node, const value_t &val) {
  std::visit(
      [&](auto &&arg) {
        using T = std::decay_t<decltype(arg)>;

        if constexpr (std::is_same_v<T, std::monostate>) {
          set_type(node, "nil");
        } else if constexpr (std::is_same_v<T, bool>) {
          set_type(node, "bool");
          node.value(std::string(arg ? "true" : "false"));
        } else if constexpr (std::is_same_v<T, int64_t>) {
          set_type(node, "int");
          node.value(std::to_string(arg));
        } else if constexpr (std::is_same_v<T, double>) {
          set_type(node, "double");
          // Use enough precision for round-trip
          char buf[32];
          std::snprintf(buf, sizeof(buf), "%.17g", arg);
          node.value(std::string(buf));
        } else if constexpr (std::is_same_v<T, std::string>) {
          set_type(node, "string");
          node.value(arg);
        } else if constexpr (std::is_same_v<T, symbol>) {
          set_type(node, "symbol");
          node.value(arg.name);
        } else if constexpr (std::is_same_v<T, list_ptr>) {
          set_type(node, "list"); // null: a non-null list is the walk's
        } else if constexpr (std::is_same_v<T, dict_ptr>) {
          set_type(node, "dict"); // null: a non-null dict is the walk's
        } else if constexpr (std::is_same_v<T, closure_ptr>) {
          set_type(node, "closure"); // null: a non-null closure is the walk's
        } else if constexpr (std::is_same_v<T, native_fn>) {
          set_type(node, "native_fn");
          // Can't serialize functions — store a marker
          node.value(std::string("__native__"));
        } else if constexpr (std::is_same_v<T, data_object_ptr>) {
          set_type(node, "data_object");
          if (arg) {
            node("__data_type__").value(arg->type_name);
            // The payload is boost::any — store it via state::data()
            node.data(arg->payload);
          }
        } else if constexpr (std::is_same_v<T, bytes_value>) {
          set_type(node, "bytes"); // SE-3: base64 so arbitrary octets survive text serialization
          node.value(b64_encode(arg.data));
        }
      },
      val.v);
}

value_t decode_leaf(cvc::state &node, const std::string &type) {
  if (type.empty() || type == "nil")
    return nil_value;

  if (type == "bool")
    return value_t(node.value() == "true");

  if (type == "int")
    return value_t(static_cast<int64_t>(std::stoll(node.value())));

  if (type == "double")
    return value_t(std::stod(node.value()));

  if (type == "string")
    return value_t(node.value());

  if (type == "bytes") // SE-3
    return make_bytes(b64_decode(node.value()));

  if (type == "symbol")
    return value_t(symbol{node.value()});

  if (type == "native_fn") {
    // Can't restore native functions — return nil
    return nil_value;
  }

  if (type == "data_object") {
    auto dobj = std::make_shared<data_object>();
    if (has_child(node, "__data_type__"))
      dobj->type_name = node("__data_type__").value();
    dobj->payload = node.data();
    return value_t(std::move(dobj));
  }

  throw std::runtime_error("decode_value: unknown type: " + type);
}

} // namespace

// ---------------------------------------------------------------------------
// encode_environment / decode_environment
// ---------------------------------------------------------------------------

void encode_environment(cvc::state &node, const environment_ptr &env) {
  value_writer(node).write_env(node, env);
}

environment_ptr decode_environment(cvc::state &node) { return value_reader(node).read_env(node); }

// ---------------------------------------------------------------------------
// encode_eval_frame
// ---------------------------------------------------------------------------
namespace {

std::string phase_to_string(eval_phase p) {
  switch (p) {
  case eval_phase::init:
    return "init";
  case eval_phase::eval_args:
    return "eval_args";
  case eval_phase::apply:
    return "apply";
  case eval_phase::if_cond:
    return "if_cond";
  case eval_phase::if_branch:
    return "if_branch";
  case eval_phase::begin_next:
    return "begin_next";
  case eval_phase::while_cond:
    return "while_cond";
  case eval_phase::while_body:
    return "while_body";
  case eval_phase::for_body:
    return "for_body";
  case eval_phase::set_value:
    return "set_value";
  case eval_phase::let_bindings:
    return "let_bindings";
  case eval_phase::let_body:
    return "let_body";
  case eval_phase::return_value:
    return "return_value";
  case eval_phase::eval_inner:
    return "eval_inner";
  case eval_phase::super_args:
    return "super_args";
  case eval_phase::defclass_methods:
    return "defclass_methods";
  case eval_phase::yield_value:
    return "yield_value";
  case eval_phase::break_unwind:
    return "break_unwind";
  case eval_phase::try_body:
    return "try_body";
  case eval_phase::try_handler:
    return "try_handler";
  case eval_phase::try_finally:
    return "try_finally";
  case eval_phase::raise_value:
    return "raise_value";
  }
  return "init";
}

eval_phase string_to_phase(const std::string &s) {
  if (s == "eval_args")
    return eval_phase::eval_args;
  if (s == "apply")
    return eval_phase::apply;
  if (s == "if_cond")
    return eval_phase::if_cond;
  if (s == "if_branch")
    return eval_phase::if_branch;
  if (s == "begin_next")
    return eval_phase::begin_next;
  if (s == "while_cond")
    return eval_phase::while_cond;
  if (s == "while_body")
    return eval_phase::while_body;
  if (s == "for_body")
    return eval_phase::for_body;
  if (s == "set_value")
    return eval_phase::set_value;
  if (s == "let_bindings")
    return eval_phase::let_bindings;
  if (s == "let_body")
    return eval_phase::let_body;
  if (s == "return_value")
    return eval_phase::return_value;
  if (s == "eval_inner")
    return eval_phase::eval_inner;
  if (s == "super_args")
    return eval_phase::super_args;
  if (s == "defclass_methods")
    return eval_phase::defclass_methods;
  if (s == "yield_value")
    return eval_phase::yield_value;
  if (s == "break_unwind")
    return eval_phase::break_unwind;
  if (s == "try_body")
    return eval_phase::try_body;
  if (s == "try_handler")
    return eval_phase::try_handler;
  if (s == "try_finally")
    return eval_phase::try_finally;
  if (s == "raise_value")
    return eval_phase::raise_value;
  return eval_phase::init;
}

void encode_value_list(cvc::state &node, const std::vector<value_t> &vals) {
  state_list sl(node);
  for (auto &v : vals)
    encode_value(sl.push_back(), v);
}

std::vector<value_t> decode_value_list(cvc::state &node) {
  state_list sl(node);
  std::vector<value_t> result;
  result.reserve(sl.size());
  for (size_t i = 0; i < sl.size(); ++i)
    result.push_back(decode_value(sl.at(i)));
  return result;
}

void encode_frame(cvc::state &node, const eval_frame &f) {
  encode_value(node("expr"), f.expr);
  encode_environment(node("env"), f.env);
  node("phase").value(phase_to_string(f.phase));
  node("index").value(std::to_string(f.index));

  // results
  if (!f.results.empty())
    encode_value_list(node("results"), f.results);

  // extra_vals
  if (!f.extra_vals.empty())
    encode_value_list(node("extra_vals"), f.extra_vals);

  // extra_strs
  if (!f.extra_strs.empty()) {
    state_list sl(node("extra_strs"));
    for (auto &s : f.extra_strs)
      sl.push_back().value(s);
  }

  // extra_lists
  if (!f.extra_lists.empty()) {
    state_list sl(node("extra_lists"));
    for (auto &inner : f.extra_lists)
      encode_value_list(sl.push_back(), inner);
  }

  // extra_env
  if (f.extra_env)
    encode_environment(node("extra_env"), f.extra_env);

  if (f.function_body)
    node("function_body").value("1");
}

eval_frame decode_frame(cvc::state &node) {
  eval_frame f;
  f.expr = decode_value(node("expr"));
  f.env = decode_environment(node("env"));
  f.phase = string_to_phase(node("phase").value());
  f.index = std::stoll(node("index").value());

  if (has_child(node, "results"))
    f.results = decode_value_list(node("results"));

  if (has_child(node, "extra_vals"))
    f.extra_vals = decode_value_list(node("extra_vals"));

  if (has_child(node, "extra_strs")) {
    state_list sl(node("extra_strs"));
    for (size_t i = 0; i < sl.size(); ++i)
      f.extra_strs.push_back(sl.at(i).value());
  }

  if (has_child(node, "extra_lists")) {
    state_list sl(node("extra_lists"));
    for (size_t i = 0; i < sl.size(); ++i)
      f.extra_lists.push_back(decode_value_list(sl.at(i)));
  }

  if (has_child(node, "extra_env"))
    f.extra_env = decode_environment(node("extra_env"));

  f.function_body = has_child(node, "function_body");

  return f;
}

} // namespace

// ---------------------------------------------------------------------------
// encode_evaluator_state
// ---------------------------------------------------------------------------

void encode_evaluator_state(cvc::state &node, const evaluator_state &es) {
  set_type(node, "evaluator_state");

  // Root expression
  encode_value(node("root_expr"), es.root_expr);

  // Result
  encode_value(node("result"), es.result);

  // Done flag
  node("done").value(std::string(es.done ? "1" : "0"));

  // Frame stack. Deep-data work in flight is not kept (see data_step_nodes below), and the frame
  // waiting on the outermost work runs its call again after a restore: the frames of DSL
  // functions that work called are left out, and the frame keeps only its arguments.
  {
    // A heap measurement waits on no frame, so it never shortens the stack.
    std::size_t keep = es.stack.size();
    std::size_t frame_args = 0;
    bool trims = false;
    for (const pending_data_work *w = es.data_work.get(); w; w = w->outer.get()) {
      if (w->what == pending_data_work::kind::heap_measure)
        continue;
      keep = std::min(w->base, es.stack.size());
      frame_args = w->frame_args;
      trims = true;
    }
    state_list sl(node("stack"));
    for (std::size_t i = 0; i < keep; ++i) {
      if (i + 1 == keep && trims && es.stack[i].results.size() > frame_args) {
        eval_frame waiting = es.stack[i];
        waiting.results.resize(frame_args);
        encode_frame(sl.push_back(), waiting);
      } else {
        encode_frame(sl.push_back(), es.stack[i]);
      }
    }
  }

  // Global environment
  if (es.global_env)
    encode_environment(node("global_env"), es.global_env);

  // User macros — store as state_list entries with explicit names
  if (!es.user_macros.empty()) {
    state_list ml(node("user_macros"));
    for (auto &[name, macro] : es.user_macros) {
      auto &entry = ml.push_back();
      entry("__name__").value(name);
      // Params
      state_list pl(entry("params"));
      for (auto &p : macro.first)
        pl.push_back().value(p.name);
      // Body
      encode_value(entry("body"), macro.second);
    }
  }

  // A script still being read (parser.step_tokens): keep its source and parser settings; the
  // restored state reads it again from the start.
  if (es.reading) {
    auto &r = node("reading");
    r("source").value(es.reading->source());
    r("max_depth").value(std::to_string(es.reading->options().max_depth));
    r("step_tokens").value(std::to_string(es.reading->options().step_tokens));
  }

  // Deep-data slicing (data.step_nodes). Work in flight is not kept: after a restore the waiting
  // native call or macro expansion runs again.
  if (es.data_step_nodes > 0)
    node("data_step_nodes").value(std::to_string(es.data_step_nodes));

  // Stats snapshot
  node("step_count").value(std::to_string(es.stats.get_step_count()));
}

// ---------------------------------------------------------------------------
// decode_evaluator_state
// ---------------------------------------------------------------------------

evaluator_state decode_evaluator_state(cvc::state &node) {
  evaluator_state es;

  // Root expression
  es.root_expr = decode_value(node("root_expr"));

  // Result
  es.result = decode_value(node("result"));

  // Done
  es.done = has_child(node, "done") && node("done").value() == "1";

  // Frame stack
  if (has_child(node, "stack")) {
    state_list sl(node("stack"));
    for (size_t i = 0; i < sl.size(); ++i)
      es.stack.push_back(decode_frame(sl.at(i)));
  }

  // Global environment
  if (has_child(node, "global_env"))
    es.global_env = decode_environment(node("global_env"));

  // User macros
  if (has_child(node, "user_macros")) {
    state_list ml(node("user_macros"));
    for (size_t i = 0; i < ml.size(); ++i) {
      auto &entry = ml.at(i);
      std::string name = entry("__name__").value();
      std::vector<symbol> params;
      if (has_child(entry, "params")) {
        state_list pl(entry("params"));
        for (size_t j = 0; j < pl.size(); ++j)
          params.push_back(symbol{pl.at(j).value()});
      }
      value_t body = decode_value(entry("body"));
      es.user_macros[name] = {std::move(params), std::move(body)};
    }
  }

  if (has_child(node, "data_step_nodes")) {
    const std::string v = node("data_step_nodes").value();
    std::size_t n = 0;
    const auto [end, ec] = std::from_chars(v.data(), v.data() + v.size(), n);
    if (ec == std::errc{} && end == v.data() + v.size())
      es.data_step_nodes = n;
  }

  // A script still being read restarts its reading from the beginning.
  if (has_child(node, "reading")) {
    auto &r = node("reading");
    parse_options opts;
    const auto size_of = [&](const char *key, std::size_t fallback) {
      const std::string v = r(key).value();
      std::size_t n = fallback;
      const auto [end, ec] = std::from_chars(v.data(), v.data() + v.size(), n);
      return ec == std::errc{} && end == v.data() + v.size() ? n : fallback;
    };
    opts.max_depth = size_of("max_depth", opts.max_depth);
    opts.step_tokens = size_of("step_tokens", 1); // a reading state always has a positive slice
    if (opts.step_tokens == 0)
      opts.step_tokens = 1;
    es.reading = std::make_shared<script_reader>(r("source").value(), opts);
  }

  return es;
}

} // namespace cvc::state_exec
