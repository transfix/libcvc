#include <algorithm> // §12 enforce_channel_policy: std::find over the declared/global channel lists
#include <boost/algorithm/string/trim.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/property_tree/json_parser/error.hpp>
#include <charconv>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cvc/state/state.h>
#include <cvc/state/state_exec/async_host.h> // async calls' futures, for await/await-all
#include <cvc/state/state_exec/builtins.h>
#include <cvc/state/state_exec/data_job.h>
#include <cvc/state/state_exec/detail/lock_home.h>
#include <cvc/state/state_exec/intrinsics.h>
#include <cvc/state/state_exec/memory_tracker.h>
#include <cvc/state/state_exec/parser.h> // detail::read_number_text, for state-incr
#include <cvc/state/state_exec/process.h>
#include <cvc/state/state_exec/scheduler.h>
#include <cvc/state/state_json.h>
#include <limits>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_map>
#include <unordered_set>

namespace cvc::state_exec {

namespace {

// ---------------------------------------------------------------------------
// helpers
// ---------------------------------------------------------------------------

void expect_exact(std::span<const value_t> args, size_t n, const char *name) {
  if (args.size() != n)
    throw std::runtime_error(std::string(name) + ": expected " + std::to_string(n) +
                             " argument(s), got " + std::to_string(args.size()));
}

void expect_min(std::span<const value_t> args, size_t n, const char *name) {
  if (args.size() < n)
    throw std::runtime_error(std::string(name) + ": expected at least " + std::to_string(n) +
                             " argument(s), got " + std::to_string(args.size()));
}

const std::string &as_string(const value_t &v, const char *name) {
  if (auto *s = std::get_if<std::string>(&v.v))
    return *s;
  throw std::runtime_error(std::string(name) + ": expected string, got " + v.type_name());
}

// A message payload may be TEXT (string) or BINARY (bytes); both are std::string byte buffers under
// the hood. Return the raw bytes of either and reject anything else. `is_bytes_value` distinguishes
// the two so a caller can tag the content type / route to the bytes wire field.
const std::string &as_string_or_bytes(const value_t &v, const char *name) {
  if (auto *s = std::get_if<std::string>(&v.v))
    return *s;
  if (auto *b = std::get_if<bytes_value>(&v.v))
    return b->data;
  throw std::runtime_error(std::string(name) + ": expected string or bytes, got " + v.type_name());
}
bool is_bytes_value(const value_t &v) { return std::get_if<bytes_value>(&v.v) != nullptr; }

int64_t as_int(const value_t &v, const char *name) {
  if (auto *i = std::get_if<int64_t>(&v.v))
    return *i;
  throw std::runtime_error(std::string(name) + ": expected integer, got " + v.type_name());
}

double as_number(const value_t &v, const char *name) {
  if (auto *i = std::get_if<int64_t>(&v.v))
    return static_cast<double>(*i);
  if (auto *d = std::get_if<double>(&v.v))
    return *d;
  throw std::runtime_error(std::string(name) + ": expected number, got " + v.type_name());
}

// Coerce any DSL value to the string form the (string-typed) state tree stores. A string
// stores raw (NOT to_string's quoted form); scalars store their natural lexical form
// (10, 1.5, true/false — not to_string's #t/#f); a float stores the shortest text that reads
// back as it (1234567.5, not 1.23457e+06); nil empties the node. Compound values
// (list/dict) fall back to the readable to_string() — a structured payload should use
// state-data-set instead. This lets (state-set "n" 10) store "10" instead of throwing on
// a non-string value.
std::string coerce_state_string(const value_t &v) {
  if (auto *s = std::get_if<std::string>(&v.v))
    return *s;
  if (auto *i = std::get_if<int64_t>(&v.v))
    return std::to_string(*i);
  if (auto *d = std::get_if<double>(&v.v))
    return format_number_shortest(*d);
  if (auto *b = std::get_if<bool>(&v.v))
    return *b ? "true" : "false";
  if (v.is_nil())
    return std::string();
  return to_string(v); // list/dict/symbol/closure — readable fallback
}

// A scalar for an atomic native's compare or store (design 4.4): nil is "", so a nil `expected`
// matches an absent or empty node and a nil `new` empties it; a list or dict is a TypeError;
// anything else is coerce_state_string's text.
std::string atomic_scalar(const value_t &v, const char *name) {
  if (v.is_nil())
    return std::string();
  if (is_compound(v))
    throw dsl_error("TypeError", std::string(name) + ": expected a scalar, got " + v.type_name());
  return coerce_state_string(v);
}

void require_root(const intrinsics_context *ctx, const char *name) {
  if (!ctx->root)
    throw std::runtime_error(std::string(name) + ": no state root bound");
}

void require_sched(const intrinsics_context *ctx, const char *name) {
  if (!ctx->sched)
    throw std::runtime_error(std::string(name) + ": no scheduler bound");
}

// ---------------------------------------------------------------------------
// State tree intrinsics
// ---------------------------------------------------------------------------

namespace {
// The node a READ of `path` sees. A transparent link is the sanctioned cross-subtree channel
// (roadmap §7.8.3): a chrooted program cannot NAME anything outside its subtree, but the host may
// plant a transparent link node inside it whose target lies elsewhere (a "named hole"). A read
// sees through it on every hop, so "hole" reads the target and "hole.a" the target's child "a",
// matching what a write does (state::childThroughLinks + value(v) route through a writable link).
// A non-link, an opaque link, or a broken / cyclic / budget-exhausted transparent link stays put
// (the same fallback resolvedValue() documents). nullptr when a segment is absent.
cvc::state *read_node(intrinsics_context *ctx, const std::string &path) {
  return ctx->root->findThroughLinks(path);
}

// The node a WRITE of `path` lands on, made when absent: through a writable transparent link on
// the target, through a read-only one on the link node's own (local) children.
cvc::state &write_node(cvc::state *root, const std::string &path) {
  return root->childThroughLinks(path);
}
} // namespace

value_t intrinsic_state_get(intrinsics_context *ctx, std::span<const value_t> args) {
  expect_exact(args, 1, "state-get");
  require_root(ctx, "state-get");
  auto &path = as_string(args[0], "state-get");
  auto *node = read_node(ctx, path);
  if (!node)
    return nil_value;
  return value_t(node->value());
}

value_t intrinsic_state_set(intrinsics_context *ctx, std::span<const value_t> args) {
  expect_exact(args, 2, "state-set");
  require_root(ctx, "state-set");
  auto &path = as_string(args[0], "state-set");
  // Coerce the value to a string (state stores string-typed scalars) rather than
  // requiring the caller to pre-stringify — (state-set "n" 10) now stores "10". A
  // structured value should use state-data-set (the typed data() channel).
  if (is_compound(args[1])) {
    // A list or dict prints as a walk, a slice per step when the process slices deep data
    // ("Deep data on the executor"); the node is written once the whole string is ready.
    cvc::state *root = ctx->root;
    return run_data_job(make_print_job(args[1]), [root, path = path](value_t printed) {
      write_node(root, path).value(std::get<std::string>(printed.v));
      return nil_value;
    });
  }
  // childThroughLinks creates child nodes as needed
  write_node(ctx->root, path).value(coerce_state_string(args[1]));
  return nil_value;
}

// The node an atomic native writes: `path` under the chroot, made when absent. It must be the node
// a read sees, so a path through a read-only transparent link is refused (read_only_error) rather
// than taking the write on the link's own children.
cvc::state &atomic_node(intrinsics_context *ctx, const std::string &path) {
  return ctx->root->childThroughLinks(path, /*refuse_read_only=*/true);
}

// (state-cas path expected new): #t when the value equalled `expected` and is now `new` (an equal
// `new` succeeds without writing), else #f. An absent node matches only a nil (or "") expected, and
// is made only then.
value_t intrinsic_state_cas(intrinsics_context *ctx, std::span<const value_t> args) {
  expect_exact(args, 3, "state-cas");
  require_root(ctx, "state-cas");
  const auto &path = as_string(args[0], "state-cas");
  const std::string expected = atomic_scalar(args[1], "state-cas");
  const std::string desired = atomic_scalar(args[2], "state-cas");
  if (!read_node(ctx, path) && !expected.empty())
    return value_t(false);
  return value_t(atomic_node(ctx, path).compareAndSetValue(expected, desired));
}

// (state-swap path new): the value it replaced, nil when the node was absent or "".
value_t intrinsic_state_swap(intrinsics_context *ctx, std::span<const value_t> args) {
  expect_exact(args, 2, "state-swap");
  require_root(ctx, "state-swap");
  const auto &path = as_string(args[0], "state-swap");
  const std::string desired = atomic_scalar(args[1], "state-swap");
  std::string old = atomic_node(ctx, path).exchangeValue(desired);
  return old.empty() ? nil_value : value_t(std::move(old));
}

// (state-incr path [delta]): adds `delta` (default 1) to the node's number and returns the new one.
// An absent or empty node counts as 0. The node's text is read as a DSL number token
// (detail::read_number_text); anything else is a ValueError and the node is left as it was. An int
// plus an int stays an int (an overflow is an ArithmeticError); otherwise the sum is a float,
// stored in its shortest round-trip form.
value_t intrinsic_state_incr(intrinsics_context *ctx, std::span<const value_t> args) {
  if (args.empty() || args.size() > 2)
    throw std::runtime_error("state-incr: expected 1 or 2 argument(s), got " +
                             std::to_string(args.size()));
  require_root(ctx, "state-incr");
  const auto &path = as_string(args[0], "state-incr");
  value_t delta(int64_t(1));
  if (args.size() == 2) {
    if (!std::holds_alternative<int64_t>(args[1].v) && !std::holds_alternative<double>(args[1].v))
      throw dsl_error("ValueError",
                      "state-incr: the delta must be a number, got " + args[1].type_name());
    delta = args[1];
  }
  value_t result;
  atomic_node(ctx, path).updateValue([&](const std::string &old) {
    value_t cur(int64_t(0));
    if (!old.empty()) {
      std::optional<value_t> number = detail::read_number_text(old);
      if (!number)
        throw dsl_error("ValueError", "state-incr: not a number: '" + old + "'");
      cur = std::move(*number);
    }
    const auto *a = std::get_if<int64_t>(&cur.v);
    const auto *b = std::get_if<int64_t>(&delta.v);
    if (a && b) {
      if ((*b > 0 && *a > std::numeric_limits<int64_t>::max() - *b) ||
          (*b < 0 && *a < std::numeric_limits<int64_t>::min() - *b))
        throw dsl_error("ArithmeticError",
                        "state-incr: integer overflow: " + old + " + " + std::to_string(*b));
      result = value_t(*a + *b);
      return std::to_string(*a + *b);
    }
    const double sum = as_number(cur, "state-incr") + as_number(delta, "state-incr");
    result = value_t(sum);
    return format_number_shortest(sum);
  });
  return result;
}

value_t intrinsic_state_children(intrinsics_context *ctx, std::span<const value_t> args) {
  expect_exact(args, 1, "state-children");
  require_root(ctx, "state-children");
  auto &path = as_string(args[0], "state-children");
  auto *node = read_node(ctx, path);
  if (!node)
    return make_list();
  auto names = node->children();
  std::vector<value_t> result;
  result.reserve(names.size());
  for (auto &n : names)
    result.emplace_back(n);
  return make_list(std::move(result));
}

value_t intrinsic_state_exists(intrinsics_context *ctx, std::span<const value_t> args) {
  expect_exact(args, 1, "state-exists");
  require_root(ctx, "state-exists");
  auto &path = as_string(args[0], "state-exists");
  return value_t(read_node(ctx, path) != nullptr);
}

value_t intrinsic_state_delete(intrinsics_context *ctx, std::span<const value_t> args) {
  expect_exact(args, 1, "state-delete");
  require_root(ctx, "state-delete");
  auto &path = as_string(args[0], "state-delete");
  auto *node = ctx->root->findDescendant(path);
  if (node) {
    // Use expiry for immediate deletion
    node->expireAt(boost::posix_time::microsec_clock::universal_time());
    ctx->root->sweepExpired();
  }
  return nil_value;
}

value_t intrinsic_state_data_get(intrinsics_context *ctx, std::span<const value_t> args) {
  expect_exact(args, 1, "state-data-get");
  require_root(ctx, "state-data-get");
  auto &path = as_string(args[0], "state-data-get");
  auto *node = read_node(ctx, path);
  if (!node)
    return nil_value;
  auto d = node->data();
  if (d.empty())
    return nil_value;
  // If the payload is a DSL value_t (the shape state-data-set stores), return a DEEP COPY
  // so structured data round-trips transparently — (state-data-set "k" (list 1 2 3)) then
  // (state-data-get "k") yields (1 2 3) — WITHOUT aliasing the node's stored storage (a
  // returned list_ptr/dict_ptr would otherwise let a caller mutate persistent state in
  // place via append/set-nth, defeating the read-only contract). Genuine host data (any
  // other C++ type parked on the node) still comes back as a data_object.
  if (auto *v = boost::any_cast<value_t>(&d))
    return run_data_job(make_copy_job(*v));
  // A RAW std::string blob on the data channel reads back as the DSL `bytes` type (opaque octets),
  // NOT an opaque data_object. That raw representation is what state-data-set stores for a `bytes`
  // value, what the `?data` URI channel writes (uri_state.cpp), and what the §13.9 HTTP cache parks
  // on a node — so a byte blob stays coherent across the resolver and state_exec (types.h: `bytes`
  // is opaque octets backed by std::string, the sanctioned home for HTTP octet-stream bodies).
  if (const std::string *s = boost::any_cast<std::string>(&d))
    return make_bytes(*s);
  auto obj = std::make_shared<data_object>();
  obj->payload = d;
  obj->type_name = d.type().name();
  return value_t(std::move(obj));
}

value_t intrinsic_state_data_set(intrinsics_context *ctx, std::span<const value_t> args) {
  expect_exact(args, 2, "state-data-set");
  require_root(ctx, "state-data-set");
  auto &path = as_string(args[0], "state-data-set");
  // Refuse to store a callable. A native_fn/closure/generator captures an environment or
  // intrinsics_context that may not outlive this (persistent) node — a use-after-free
  // waiting to happen — and a stored callable is an invocation channel that would bypass
  // any environment allowlist a later reader relies on. The data channel is for structured
  // DATA (scalars/lists/dicts), not code.
  if (std::holds_alternative<native_fn>(args[1].v) ||
      std::holds_alternative<closure_ptr>(args[1].v) ||
      std::holds_alternative<generator_ptr>(args[1].v))
    throw std::runtime_error(
        "state-data-set: cannot store a callable (function/closure/generator) as data");
  // A `bytes` value (opaque octets) is stored as a RAW std::string on the data channel — the SAME
  // representation the `?data` URI channel (uri_state.cpp) and the §13.9 HTTP cache use — so a byte
  // blob written from the DSL is readable via state://…?data and a blob parked by the cache reads
  // back as `bytes` here (state-data-get, above). Every other value_t is stored as-is, so
  // structured data (lists/dicts/scalars/text strings) round-trips via the value_t path.
  if (const bytes_value *b = std::get_if<bytes_value>(&args[1].v))
    write_node(ctx->root, path).data(boost::any(b->data));
  else
    write_node(ctx->root, path).data(boost::any(args[1]));
  return nil_value;
}

// ---------------------------------------------------------------------------
// A subtree as JSON, a slice at a time
// ---------------------------------------------------------------------------
//
// state-json and state-from-json are cvc::state::json() and json(j) run as data jobs: the walk
// over the subtree (state_json_writer) and the scan of the document (state_json_reader) do
// data.step_nodes units per scheduler step, counted against max_steps and max_time, and other
// processes run between the slices. Same code as cvc::state's, so the same document. A unit is
// a node visited or a value scanned, and one per 256 bytes (kDataNodeBytes) written or scanned.

namespace {

// A writable transparent link routes a write to its target, as state-set's value() does.
cvc::state *write_through_link(cvc::state *node) {
  if (node && node->isLink() && node->linkMode() == cvc::state::link_mode::transparent &&
      node->linkWritable()) {
    const cvc::state::link_resolution lr = node->resolveLink();
    if (lr.kind == cvc::state::link_resolution_kind::resolved && lr.target)
      return lr.target;
  }
  return node;
}

// The writer holds the nodes it has yet to visit by owning pointer: a node another process
// removes between slices is still written, as it was when its parent was snapshotted. A slice
// ends when its units are spent, a long value counting for its bytes. The text is held to the
// allocation limit as it grows.
class state_json_job final : public data_job {
public:
  explicit state_json_job(cvc::state &node) : writer_(node) {}

  bool step(std::size_t budget) override {
    const std::size_t start = nodes_;
    const std::optional<std::size_t> limit = eval_alloc_limit();
    bool done = false;
    while (!done && (budget == 0 || nodes_ - start < budget)) {
      done = writer_.step(budget == 0 ? 0 : 1);
      const std::size_t size = writer_.text().size();
      nodes_ = writer_.nodes() + size / kDataNodeBytes;
      if (limit && size > *limit)
        eval_alloc_check(size, "state-json"); // throws
    }
    eval_alloc_note(writer_.text().size() - noted_);
    noted_ = writer_.text().size();
    return done;
  }

  value_t result() override { return value_t(writer_.take()); }

private:
  cvc::state_json_writer writer_;
  std::size_t noted_ = 0; // text bytes reported to heap accounting so far
};

// The reader checks the whole document before it sets anything, so a malformed one changes
// nothing even when the check spans many steps. A slice ends when its units are spent, a long
// string counting for its bytes.
class state_from_json_job final : public data_job {
public:
  state_from_json_job(cvc::state &node, std::string_view doc) : reader_(node, doc) {}

  bool step(std::size_t budget) override {
    const std::size_t start = nodes_;
    bool done = false;
    try {
      while (!done && (budget == 0 || nodes_ - start < budget)) {
        done = reader_.step(budget == 0 ? 0 : 1);
        nodes_ = reader_.values() + reader_.bytes() / kDataNodeBytes;
      }
    } catch (const boost::property_tree::json_parser::json_parser_error &e) {
      throw std::runtime_error("state-from-json: line " + std::to_string(e.line()) + ": " +
                               e.message());
    }
    return done;
  }

  value_t result() override { return nil_value; }

  // The document was the call's argument: keep a copy once the job outlives the call.
  void own_inputs() override {
    doc_ = std::string(reader_.doc());
    reader_.rebind(doc_);
  }

private:
  std::string doc_;
  cvc::state_json_reader reader_;
};

// ---------------------------------------------------------------------------
// A value as a subtree, a slice at a time
// ---------------------------------------------------------------------------
//
// (state-set-tree path value) writes a value of dicts, lists and scalars (what json-parse makes)
// as the subtree at `path`, so binds and `repeat:` can show it. A dict's keys and a list's indices
// name children; a scalar is a node's value, as state-set writes it (a float in its shortest
// round-trip form). A list's node holds its length, so `repeat: (state-get "feed.items")` repeats
// a widget per element and `feed.items.{i}.text` reads each; a dict's node holds "". The subtree
// is replaced: a child the value doesn't name is removed, with its subtree, and a node whose text
// is unchanged isn't written. The value is checked whole before anything is written, so one that
// can't be written changes nothing. A unit is an element checked or written, and a node removed.

// A key names one node of its own: not empty, no '.', and nothing a path's trim would drop.
bool names_a_node(const std::string &key) {
  return !key.empty() && key.find(cvc::state::SEPARATOR[0]) == std::string::npos &&
         boost::algorithm::trim_copy(key) == key;
}

// Whether `name` is an index of a list of `size` elements, as state-set-tree writes them.
bool is_index_below(const std::string &name, std::size_t size) {
  if (name.empty() || name.size() > 19 || (name.size() > 1 && name[0] == '0'))
    return false;
  std::size_t i = 0;
  for (const char c : name) {
    if (c < '0' || c > '9')
      return false;
    i = i * 10 + static_cast<std::size_t>(c - '0');
  }
  return i < size;
}

class state_set_tree_job final : public data_job {
public:
  // The node at `path` under `root` is made (or found, through a writable link) once the value
  // has passed its check.
  state_set_tree_job(cvc::state &root, std::string path, value_t v)
      : keep_(root.weak_from_this().lock()), tree_(&root), path_(std::move(path)),
        root_(std::move(v)) {}

  bool step(std::size_t budget) override {
    const std::size_t start = nodes_;
    while (budget == 0 || nodes_ - start < budget) {
      if (!started_) {
        started_ = true;
        visit(root_, writing_ ? write_through_link(&(*tree_)(path_)) : nullptr);
        continue;
      }
      if (open_.empty()) {
        if (writing_)
          return true;
        writing_ = true; // the check passed: the same walk again, writing
        started_ = false;
        continue;
      }
      eval_deadline_check("state-set-tree");
      frame &f = open_.back();
      const std::size_t i = f.next++;
      if (i >= f.size()) {
        on_path_.erase(f.id());
        open_.pop_back();
        continue;
      }
      ++nodes_;
      if (f.list) {
        const value_t &e = (*f.list)[i];
        visit(e, writing_ ? &(*f.node)(std::to_string(i)) : nullptr); // may push, invalidating f
      } else {
        const auto &[key, e] = (*f.dict)[i];
        if (!names_a_node(key))
          throw dsl_error("ValueError",
                          "state-set-tree: the key \"" + key +
                              "\" can't name a node (it is empty, has a '.' or space at an end)");
        visit(e, writing_ ? &(*f.node)(key) : nullptr); // may push, invalidating f
      }
    }
    return false;
  }

  value_t result() override { return nil_value; }

  void held_values(const std::function<void(const value_t &)> &visit) const override {
    visit(root_);
  }

private:
  // A list or dict being walked, held by owning pointer and read by index, and (writing) its node.
  struct frame {
    list_ptr list;
    dict_ptr dict;
    std::size_t next = 0;
    cvc::state::state_ptr node;
    std::size_t size() const { return list ? list->size() : dict->size(); }
    const void *id() const {
      return list ? static_cast<const void *>(list.get()) : static_cast<const void *>(dict.get());
    }
  };

  // Check `v`, and when writing, write it at `n`: a scalar's text, or a list's or dict's own
  // text, its stale children removed and its elements left for step().
  void visit(const value_t &v, cvc::state *n) {
    frame f;
    std::string text;
    if (v.is_nil()) {
    } else if (const auto *b = std::get_if<bool>(&v.v)) {
      text = *b ? "true" : "false";
    } else if (const auto *i = std::get_if<int64_t>(&v.v)) {
      text = std::to_string(*i);
    } else if (const auto *d = std::get_if<double>(&v.v)) {
      text = format_number_shortest(*d);
    } else if (const auto *s = std::get_if<std::string>(&v.v)) {
      text = *s;
    } else if (const auto *l = std::get_if<list_ptr>(&v.v); l && *l) {
      f.list = *l;
      text = std::to_string(f.list->size());
    } else if (const auto *m = std::get_if<dict_ptr>(&v.v); m && *m) {
      f.dict = *m;
    } else {
      throw dsl_error("TypeError",
                      "state-set-tree: a " + v.type_name() + " has no place in the state tree");
    }
    const bool compound = f.list || f.dict;
    if (compound && open_.size() >= kJsonMaxDepth) // a level, empty or not, as json-parse counts
      throw dsl_error("ValueError", "state-set-tree: nested deeper than " +
                                        std::to_string(kJsonMaxDepth) + " levels");
    if (compound && f.size() > 0) {
      if (!on_path_.insert(f.id()).second)
        throw dsl_error("ValueError", std::string("state-set-tree: a ") +
                                          (f.list ? "list" : "dict") + " that contains itself");
    }
    if (n) {
      if (n->value() != text)
        n->value(text);
      remove_stale(*n, f);
      f.node = n->weak_from_this().lock();
    }
    if (compound && f.size() > 0)
      open_.push_back(std::move(f));
  }

  // Remove the children of `n` that the value written there doesn't name: all of them for a
  // scalar. The same expiry state-delete uses, swept from `n`.
  void remove_stale(cvc::state &n, const frame &f) {
    const std::vector<std::string> names = n.childNames();
    if (names.empty())
      return;
    std::unordered_set<std::string_view> keys;
    if (f.dict)
      for (const auto &[k, e] : *f.dict)
        keys.insert(k);
    const auto now = boost::posix_time::microsec_clock::universal_time();
    bool stale = false;
    for (const std::string &name : names) {
      const bool named = f.list   ? is_index_below(name, f.list->size())
                         : f.dict ? keys.count(name) > 0
                                  : false;
      if (named)
        continue;
      if (cvc::state *child = n.findDescendant(name)) {
        child->expireAt(now);
        stale = true;
      }
    }
    if (stale)
      nodes_ += n.sweepExpired();
  }

  cvc::state::state_ptr keep_;
  cvc::state *tree_;
  std::string path_;
  value_t root_;
  bool started_ = false;
  bool writing_ = false; // false: the check pass
  std::vector<frame> open_;
  std::unordered_set<const void *> on_path_; // the lists and dicts open now, for cycles
};

} // namespace

value_t intrinsic_state_json(intrinsics_context *ctx, std::span<const value_t> args) {
  expect_exact(args, 1, "state-json");
  require_root(ctx, "state-json");
  auto &path = as_string(args[0], "state-json");
  cvc::state *node = read_node(ctx, path);
  if (!node)
    return nil_value;
  return run_data_job(std::make_unique<state_json_job>(*node));
}

value_t intrinsic_state_from_json(intrinsics_context *ctx, std::span<const value_t> args) {
  expect_exact(args, 2, "state-from-json");
  require_root(ctx, "state-from-json");
  auto &path = as_string(args[0], "state-from-json");
  auto &doc = as_string(args[1], "state-from-json");
  cvc::state *node = write_through_link(&write_node(ctx->root, path));
  return run_data_job(std::make_unique<state_from_json_job>(*node, doc));
}

value_t intrinsic_state_set_tree(intrinsics_context *ctx, std::span<const value_t> args) {
  expect_exact(args, 2, "state-set-tree");
  require_root(ctx, "state-set-tree");
  auto &path = as_string(args[0], "state-set-tree");
  // The root's subtree is everything the program can reach: replacing it is never what's meant.
  if (path.find_first_not_of(". \t\r\n") == std::string::npos)
    throw dsl_error("ValueError", "state-set-tree: an empty path would replace the whole tree");
  return run_data_job(std::make_unique<state_set_tree_job>(*ctx->root, path, args[1]));
}

value_t intrinsic_state_root_path(intrinsics_context *ctx, std::span<const value_t> args) {
  expect_exact(args, 0, "state-root-path");
  return value_t(ctx->root_path);
}

value_t intrinsic_state_watch(intrinsics_context *ctx, std::span<const value_t> args) {
  expect_exact(args, 2, "state-watch");
  require_root(ctx, "state-watch");
  auto &path = as_string(args[0], "state-watch");
  // args[1] is the handler expression (lambda or quoted form)
  value_t handler = args[1];

  // Navigate to the node (create if needed so we can watch it)
  (void)(*ctx->root)(path);

  // Allocate a watch ID
  int watch_id = ctx->proc->next_watch_id++;

  // Register the handler.  The scheduler polls watched paths each step
  // instead of using boost::signals2 (which segfaults on macOS).
  if (ctx->sched) {
    ctx->sched->set_watch_root(ctx->root);
    ctx->sched->register_watch_handler(ctx->pid, watch_id, handler, path);
  } else {
    // No-scheduler path (unit tests): record handler + initial value.
    std::string initial;
    auto *node = ctx->root->findDescendant(path);
    if (node)
      initial = node->value();
    ctx->proc->watch_handlers[watch_id] = {handler, path, initial};
  }

  // Keep a local registry for unwatch; no signal connection needed.
  ctx->watches[watch_id] = {path, handler, []() {}};

  return value_t(static_cast<int64_t>(watch_id));
}

value_t intrinsic_state_unwatch(intrinsics_context *ctx, std::span<const value_t> args) {
  expect_exact(args, 1, "state-unwatch");
  int64_t watch_id = as_int(args[0], "state-unwatch");
  auto it = ctx->watches.find(static_cast<int>(watch_id));
  if (it == ctx->watches.end())
    return value_t(false);
  ctx->watches.erase(it);
  // Remove handler from the scheduler's process (or ctx->proc if no scheduler)
  if (ctx->sched)
    ctx->sched->unregister_watch_handler(ctx->pid, static_cast<int>(watch_id));
  else
    ctx->proc->watch_handlers.erase(static_cast<int>(watch_id));
  return value_t(true);
}

// ---------------------------------------------------------------------------
// Scheduler intrinsics
// ---------------------------------------------------------------------------

value_t intrinsic_spawn(intrinsics_context *ctx, std::span<const value_t> args) {
  expect_min(args, 1, "spawn");
  require_sched(ctx, "spawn");
  auto &script = as_string(args[0], "spawn");
  execute_options opts;
  if (args.size() > 1)
    opts.name = as_string(args[1], "spawn");
  if (args.size() > 2)
    opts.priority = static_cast<int>(as_int(args[2], "spawn"));
  opts.uid = ctx->uid;
  // A child keeps its parent's owner (design 7.2; td 5 item 6), so a host's kill_owner reaps it.
  if (const process_ptr &running = ctx->sched->current_process())
    opts.owner = running->owner;
  else if (ctx->proc)
    opts.owner = ctx->proc->owner;
  // The host's spawn context (td 4.15) gives the child an environment and a context of its own,
  // which the child's process keeps alive. Without one the child gets the bare builtins.
  std::function<void(int)> spawned;
  if (ctx->spawn_context) {
    spawn_setup s = ctx->spawn_context(*ctx);
    opts.env = std::move(s.env);
    opts.keep_alive = std::move(s.context);
    opts.root_path = ctx->root_path;
    spawned = std::move(s.spawned);
  }
  int pid = ctx->sched->execute(script, opts);
  if (spawned && pid >= 0)
    spawned(pid);
  return value_t(static_cast<int64_t>(pid));
}

value_t intrinsic_fork(intrinsics_context *ctx, std::span<const value_t> args) {
  expect_exact(args, 0, "fork");
  require_sched(ctx, "fork");
  // The running process forks (td B28): its context's pid only when no step runs (a host's
  // context, as pycvc's and Ariadne's, has a placeholder pid).
  int pid = ctx->sched->current_pid();
  if (pid < 0)
    pid = ctx->pid;
  // Inside with-lock the child would run in sections it doesn't hold (design 5.11). The mirror is
  // read directly: asking for the home would make one for a scheduler that never locked.
  if (const process_ptr &p = ctx->sched->current_process(); p && detail::lock_home::holds_locks(*p))
    throw dsl_error("LockError", "fork: not inside with-lock", "dsl",
                    {{"reason", value_t(std::string("fork"))}});
  int child = ctx->sched->fork(pid);
  if (child >= 0 && ctx->on_child)
    ctx->on_child(child);
  return value_t(static_cast<int64_t>(child));
}

value_t intrinsic_self_pid(intrinsics_context *ctx, std::span<const value_t> args) {
  expect_exact(args, 0, "self-pid");
  int pid = (ctx->sched && ctx->sched->current_pid() >= 0) ? ctx->sched->current_pid() : ctx->pid;
  return value_t(static_cast<int64_t>(pid));
}

value_t intrinsic_self_uid(intrinsics_context *ctx, std::span<const value_t> args) {
  expect_exact(args, 0, "self-uid");
  return value_t(ctx->uid);
}

value_t intrinsic_kill(intrinsics_context *ctx, std::span<const value_t> args) {
  expect_exact(args, 1, "kill");
  require_sched(ctx, "kill");
  int pid = static_cast<int>(as_int(args[0], "kill"));
  return value_t(ctx->sched->kill(pid));
}

value_t intrinsic_pause(intrinsics_context *ctx, std::span<const value_t> args) {
  expect_exact(args, 1, "pause");
  require_sched(ctx, "pause");
  int pid = static_cast<int>(as_int(args[0], "pause"));
  return value_t(ctx->sched->pause(pid));
}

value_t intrinsic_resume(intrinsics_context *ctx, std::span<const value_t> args) {
  expect_exact(args, 1, "resume");
  require_sched(ctx, "resume");
  int pid = static_cast<int>(as_int(args[0], "resume"));
  return value_t(ctx->sched->resume(pid));
}

value_t intrinsic_sleep(intrinsics_context *ctx, std::span<const value_t> args) {
  expect_exact(args, 1, "sleep");
  require_sched(ctx, "sleep");
  double seconds = as_number(args[0], "sleep");
  if (seconds < 0)
    throw std::runtime_error("sleep: duration must be non-negative");
  int pid = ctx->sched->current_pid();
  if (pid < 0)
    pid = ctx->pid;
  return value_t(ctx->sched->sleep(pid, seconds));
}

// Park the process `pid` (stepping now) at the native call being applied, waiting on `channel`,
// for `timeout` seconds at most (docs/STATE_EXEC_EXCEPTIONS.md 3.9).
void park_await(intrinsics_context *ctx, int pid, const std::string &channel,
                std::optional<double> timeout, detail::await_request request) {
  const bool parked = timeout ? ctx->sched->receive_message_until(pid, channel, *timeout)
                              : ctx->sched->receive_message(pid, channel);
  if (!parked)
    throw std::runtime_error("cannot suspend process " + std::to_string(pid));
  request_await(std::move(request));
}

// (await f [seconds]): an async call's future returns its value or raises its exception once it
// finishes (at once if it has); a channel future, the value its producer delivers; anything
// else, itself, after a frame. With seconds, a future that hasn't finished by then raises
// TimeoutError; the work goes on, and the future can be awaited again.
value_t intrinsic_await(intrinsics_context *ctx, std::span<const value_t> args) {
  if (args.empty() || args.size() > 2)
    throw dsl_error("TypeError",
                    "await: expected 1 or 2 arguments, got " + std::to_string(args.size()));
  require_sched(ctx, "await");
  std::optional<double> timeout;
  if (args.size() == 2) {
    timeout = as_number(args[1], "await");
    if (*timeout < 0)
      throw dsl_error("ValueError", "await: the timeout must be non-negative");
  }
  int pid = ctx->sched->current_pid();
  // Only a process a scheduler is stepping can park at the call; a unit-test context with no
  // step running falls back to its own pid and the placeholder park.
  const bool stepping = pid >= 0;
  if (pid < 0)
    pid = ctx->pid;
  process *proc = nullptr;
  if (ctx->sched->current_process())
    proc = ctx->sched->current_process().get();
  else if (ctx->proc)
    proc = ctx->proc.get();

  if (is_async_future(args[0])) {
    const std::shared_ptr<detail::awaitable> call = async_call_of_future(args[0]);
    if (!call)
      throw dsl_error("RuntimeError",
                      "await: the call can't resume (its future was restored from a snapshot)",
                      "async");
    if (call->finished())
      return call->outcome();
    const std::string channel = *future_channel_of(args[0]);
    if (stepping && can_await()) {
      park_await(ctx, pid, channel, timeout,
                 {channel, call->function_name(), detail::make_awaitable_handle(call)});
      return nil_value;
    }
    // Nowhere to park (a generator's body, a native call's call): wait here, blocking the step.
    if (!call->wait(timeout))
      throw dsl_error("TimeoutError",
                      call->function_name() + ": no result within the time it was given", "async",
                      {{"function", value_t(call->function_name())}});
    return call->outcome();
  }

  // A channel future {"__future__": chan} → a value-carrying park (the §13.8 proper await):
  // suspend until the producer delivers, then resume with the resolved value threaded into the
  // enclosing expression.
  if (const std::optional<std::string> fchan = future_channel_of(args[0])) {
    if (stepping && can_await()) {
      if (proc && !proc->inbox.empty()) {
        value_t msg = std::move(proc->inbox.front());
        proc->inbox.pop();
        return msg;
      }
      if (std::optional<value_t> pending = ctx->sched->pop_pending_message(*fchan))
        return std::move(*pending);
      park_await(ctx, pid, *fchan, timeout, {*fchan, "await", nil_value});
      return nil_value;
    }
    if (timeout)
      throw dsl_error("RuntimeError", "await: can't wait with a timeout here");
    return park_on_channel(ctx->sched, proc, pid, *fchan);
  }
  // An already-settled value → the cooperative one-frame yield await has always been (a no-op on a
  // scheduler with no frames, where await is identity), then resume with the value unchanged.
  ctx->sched->yield_frame(pid);
  return args[0];
}

// (await-all (list f1 f2 ...)): wait for every future in the list (an async call's, or a
// channel's), then return their values in order, or raise the first failure in list order once
// all have finished (docs/STATE_EXEC_EXCEPTIONS.md 3.3). Anything else in the list is its own
// value. It waits for one future at a time: each channel future's message takes its place in the
// call's list (await_request::slot) and the call is applied again.
value_t intrinsic_await_all(intrinsics_context *ctx, std::span<const value_t> args) {
  expect_exact(args, 1, "await-all");
  require_sched(ctx, "await-all");
  const list_ptr *lp = std::get_if<list_ptr>(&args[0].v);
  if (!lp || !*lp)
    throw dsl_error("TypeError",
                    "await-all: expected a list of futures, got " + args[0].type_name());
  const std::vector<value_t> &items = **lp;
  const int pid = ctx->sched->current_pid();
  const bool parkable = pid >= 0 && can_await();
  for (std::size_t i = 0; i < items.size(); ++i) {
    const value_t &f = items[i];
    if (detail::awaited_of(f))
      continue; // a channel future's message, delivered
    if (!is_async_future(f)) {
      const std::optional<std::string> chan = future_channel_of(f);
      if (!chan)
        continue;
      if (!parkable)
        throw dsl_error("RuntimeError", "await-all: can't wait for a channel future here");
      detail::await_request r{*chan, "await-all", nil_value, /*retry=*/true, i};
      if (std::optional<value_t> pending = ctx->sched->pop_pending_message(*chan)) {
        r.ready = std::move(*pending);
        request_await(std::move(r));
      } else {
        park_await(ctx, pid, *chan, std::nullopt, std::move(r));
      }
      return nil_value;
    }
    const std::shared_ptr<detail::awaitable> call = async_call_of_future(f);
    if (!call)
      throw dsl_error("RuntimeError",
                      "await-all: a call can't resume (its future was restored from a snapshot)",
                      "async");
    if (call->finished())
      continue;
    // The first one still running: wait for it, then look at the list again.
    if (parkable) {
      const std::string channel = *future_channel_of(f);
      park_await(ctx, pid, channel, std::nullopt,
                 {channel, "await-all", detail::make_awaitable_handle(call), /*retry=*/true});
      return nil_value;
    }
    call->wait(std::nullopt);
  }
  std::vector<value_t> values;
  values.reserve(items.size());
  for (const value_t &f : items) { // outcome() raises the first failure in list order
    if (const value_t *msg = detail::awaited_of(f))
      values.push_back(*msg);
    else
      values.push_back(is_async_future(f) ? async_call_of_future(f)->outcome() : f);
  }
  return make_list(std::move(values));
}

value_t intrinsic_ps(intrinsics_context *ctx, std::span<const value_t> args) {
  expect_exact(args, 0, "ps");
  require_sched(ctx, "ps");
  auto procs = ctx->sched->list_processes();
  std::vector<value_t> result;
  result.reserve(procs.size());
  for (auto &pi : procs) {
    // Each process → dict with key fields
    std::vector<std::pair<std::string, value_t>> entries;
    entries.emplace_back("pid", value_t(static_cast<int64_t>(pi.pid)));
    entries.emplace_back("name", value_t(pi.name));
    entries.emplace_back("status", value_t(std::string(to_string(pi.status))));
    entries.emplace_back("priority", value_t(static_cast<int64_t>(pi.priority)));
    entries.emplace_back("uid", value_t(pi.uid));
    entries.emplace_back("steps", value_t(static_cast<int64_t>(pi.step_count)));
    result.push_back(make_dict(std::move(entries)));
  }
  return make_list(std::move(result));
}

// The budget-by-kind fields of `info`, as dict entries.
static void append_budget_usage(std::vector<std::pair<std::string, value_t>> &entries,
                                const process_info &info) {
  entries.emplace_back("read_steps", value_t(static_cast<int64_t>(info.read_steps)));
  entries.emplace_back("eval_steps", value_t(static_cast<int64_t>(info.eval_steps)));
  entries.emplace_back("data_steps", value_t(static_cast<int64_t>(info.data_steps)));
  entries.emplace_back("data_nodes", value_t(static_cast<int64_t>(info.data_nodes)));
  entries.emplace_back("read_time", value_t(info.read_time));
  entries.emplace_back("eval_time", value_t(info.eval_time));
  entries.emplace_back("data_time", value_t(info.data_time));
  // What its async calls were charged (docs/STATE_EXEC_EXCEPTIONS.md 3.8).
  entries.emplace_back("async_calls", value_t(static_cast<int64_t>(info.async_calls)));
  entries.emplace_back("async_steps", value_t(static_cast<int64_t>(info.async_steps)));
  entries.emplace_back("async_time", value_t(info.async_time));
  entries.emplace_back("async_cpu_time", value_t(info.async_cpu_time));
}

value_t intrinsic_inspect(intrinsics_context *ctx, std::span<const value_t> args) {
  expect_exact(args, 1, "inspect");
  require_sched(ctx, "inspect");
  int pid = static_cast<int>(as_int(args[0], "inspect"));
  auto info = ctx->sched->get_process_info(pid);
  if (!info)
    return nil_value;
  std::vector<std::pair<std::string, value_t>> entries;
  entries.emplace_back("pid", value_t(static_cast<int64_t>(info->pid)));
  entries.emplace_back("name", value_t(info->name));
  entries.emplace_back("status", value_t(std::string(to_string(info->status))));
  entries.emplace_back("priority", value_t(static_cast<int64_t>(info->priority)));
  entries.emplace_back("uid", value_t(info->uid));
  entries.emplace_back("gid", value_t(info->gid));
  entries.emplace_back("steps", value_t(static_cast<int64_t>(info->step_count)));
  append_budget_usage(entries, *info);
  entries.emplace_back("elapsed_time", value_t(info->elapsed_time));
  entries.emplace_back("memory", value_t(static_cast<int64_t>(info->current_memory)));
  entries.emplace_back("heap_memory", value_t(static_cast<int64_t>(info->heap_memory)));
  entries.emplace_back("peak_memory", value_t(static_cast<int64_t>(info->peak_memory)));
  entries.emplace_back("max_memory", value_t(static_cast<int64_t>(info->max_memory)));
  entries.emplace_back("max_time", value_t(info->max_time));
  entries.emplace_back("messages", value_t(static_cast<int64_t>(info->message_count)));
  entries.emplace_back("max_messages", value_t(static_cast<int64_t>(info->max_messages)));
  entries.emplace_back("message_bytes", value_t(static_cast<int64_t>(info->message_bytes)));
  entries.emplace_back("max_message_bytes", value_t(static_cast<int64_t>(info->max_message_bytes)));
  entries.emplace_back("parent_pid", value_t(static_cast<int64_t>(info->parent_pid)));
  if (info->exit_error)
    entries.emplace_back("exit_error", value_t(*info->exit_error));
  if (info->exit_exception)
    entries.emplace_back("exception", *info->exit_exception);
  entries.emplace_back("locks_held", value_t(static_cast<int64_t>(info->locks_held)));
  if (info->waiting_on) {
    // A lock key as this program would write it (gap G-17).
    const std::string_view lock_prefix = "lock /";
    std::string on = *info->waiting_on;
    if (on.rfind(lock_prefix, 0) == 0)
      on = "lock " + detail::lock_home::show(on.substr(lock_prefix.size()), ctx->root_path);
    entries.emplace_back("waiting_on", value_t(std::move(on)));
  }
  return make_dict(std::move(entries));
}

// ---------------------------------------------------------------------------
// Resource intrinsics
// ---------------------------------------------------------------------------

value_t intrinsic_memory_usage(intrinsics_context *ctx, std::span<const value_t> args) {
  expect_exact(args, 0, "memory-usage");
  if (!ctx->tracker)
    return value_t(int64_t(0));
  return value_t(static_cast<int64_t>(ctx->tracker->current_bytes(ctx->pid)));
}

value_t intrinsic_memory_limit(intrinsics_context *ctx, std::span<const value_t> args) {
  expect_exact(args, 0, "memory-limit");
  if (!ctx->proc)
    return value_t(int64_t(0));
  return value_t(static_cast<int64_t>(ctx->proc->max_memory));
}

value_t intrinsic_time_elapsed(intrinsics_context *ctx, std::span<const value_t> args) {
  expect_exact(args, 0, "time-elapsed");
  if (!ctx->proc)
    return value_t(0.0);
  return value_t(ctx->proc->elapsed_time());
}

value_t intrinsic_time_limit(intrinsics_context *ctx, std::span<const value_t> args) {
  expect_exact(args, 0, "time-limit");
  if (!ctx->proc)
    return value_t(0.0);
  return value_t(static_cast<double>(ctx->proc->max_time));
}

value_t intrinsic_message_count(intrinsics_context *ctx, std::span<const value_t> args) {
  expect_exact(args, 0, "message-count");
  if (!ctx->proc)
    return value_t(int64_t(0));
  return value_t(static_cast<int64_t>(ctx->proc->message_count));
}

value_t intrinsic_message_limit(intrinsics_context *ctx, std::span<const value_t> args) {
  expect_exact(args, 0, "message-limit");
  if (!ctx->proc)
    return value_t(int64_t(0));
  return value_t(static_cast<int64_t>(ctx->proc->max_messages));
}

value_t intrinsic_message_bytes(intrinsics_context *ctx, std::span<const value_t> args) {
  expect_exact(args, 0, "message-bytes");
  if (!ctx->proc)
    return value_t(int64_t(0));
  return value_t(static_cast<int64_t>(ctx->proc->message_bytes));
}

value_t intrinsic_message_bytes_limit(intrinsics_context *ctx, std::span<const value_t> args) {
  expect_exact(args, 0, "message-bytes-limit");
  if (!ctx->proc)
    return value_t(int64_t(0));
  return value_t(static_cast<int64_t>(ctx->proc->max_message_bytes));
}

// What the current process's budget went on: steps, seconds and deep-data nodes by kind (read,
// eval, data), what its async calls were charged, plus the total steps -- the same fields
// inspect reports for any pid.
value_t intrinsic_budget_usage(intrinsics_context *ctx, std::span<const value_t> args) {
  expect_exact(args, 0, "budget-usage");
  // The running process, as self-pid finds it: the scheduler's current one, else the context's.
  process_info info{};
  if (ctx->sched && ctx->sched->current_pid() >= 0) {
    if (auto running = ctx->sched->get_process_info(ctx->sched->current_pid()))
      info = *running;
  } else if (ctx->proc) {
    info.step_count = ctx->proc->step_count();
    fill_budget_usage(info, ctx->proc->state.stats);
    fill_async_usage(info, *ctx->proc);
  }
  std::vector<std::pair<std::string, value_t>> entries;
  entries.emplace_back("steps", value_t(static_cast<int64_t>(info.step_count)));
  append_budget_usage(entries, info);
  return make_dict(std::move(entries));
}

value_t intrinsic_step_count(intrinsics_context *ctx, std::span<const value_t> args) {
  expect_exact(args, 0, "step-count");
  if (!ctx->proc)
    return value_t(int64_t(0));
  return value_t(static_cast<int64_t>(ctx->proc->step_count()));
}

// ---------------------------------------------------------------------------
// System identity intrinsics
// ---------------------------------------------------------------------------

value_t intrinsic_cluster_id(intrinsics_context *ctx, std::span<const value_t> args) {
  expect_exact(args, 0, "cluster-id");
  return value_t(ctx->cluster_id);
}

value_t intrinsic_node_id(intrinsics_context *ctx, std::span<const value_t> args) {
  expect_exact(args, 0, "node-id");
  return value_t(ctx->node_id);
}

value_t intrinsic_scheduler_stats(intrinsics_context *ctx, std::span<const value_t> args) {
  expect_exact(args, 0, "scheduler-stats");
  require_sched(ctx, "scheduler-stats");
  auto s = ctx->sched->get_stats();
  std::vector<std::pair<std::string, value_t>> entries;
  entries.emplace_back("total", value_t(static_cast<int64_t>(s.total_processes)));
  entries.emplace_back("running", value_t(static_cast<int64_t>(s.running)));
  entries.emplace_back("ready", value_t(static_cast<int64_t>(s.ready)));
  entries.emplace_back("paused", value_t(static_cast<int64_t>(s.paused)));
  entries.emplace_back("terminated", value_t(static_cast<int64_t>(s.terminated)));
  entries.emplace_back("killed", value_t(static_cast<int64_t>(s.killed)));
  entries.emplace_back("failed", value_t(static_cast<int64_t>(s.failed)));
  entries.emplace_back("steps", value_t(static_cast<int64_t>(s.total_steps)));
  return make_dict(std::move(entries));
}

// ---------------------------------------------------------------------------
// Expiry intrinsics
// ---------------------------------------------------------------------------

value_t intrinsic_state_expire(intrinsics_context *ctx, std::span<const value_t> args) {
  expect_exact(args, 2, "state-expire");
  require_root(ctx, "state-expire");
  auto &path = as_string(args[0], "state-expire");
  double secs = as_number(args[1], "state-expire");
  auto *node = ctx->root->findDescendant(path);
  if (!node)
    throw std::runtime_error("state-expire: path not found: " + path);
  node->expireAfter(boost::posix_time::milliseconds(static_cast<int64_t>(secs * 1000.0)));
  return nil_value;
}

value_t intrinsic_state_expire_at(intrinsics_context *ctx, std::span<const value_t> args) {
  expect_exact(args, 2, "state-expire-at");
  require_root(ctx, "state-expire-at");
  auto &path = as_string(args[0], "state-expire-at");
  auto &iso = as_string(args[1], "state-expire-at");
  auto *node = ctx->root->findDescendant(path);
  if (!node)
    throw std::runtime_error("state-expire-at: path not found: " + path);
  auto pt = boost::posix_time::time_from_string(iso);
  node->expireAt(pt);
  return nil_value;
}

value_t intrinsic_state_has_expiry(intrinsics_context *ctx, std::span<const value_t> args) {
  expect_exact(args, 1, "state-has-expiry");
  require_root(ctx, "state-has-expiry");
  auto &path = as_string(args[0], "state-has-expiry");
  auto *node = ctx->root->findDescendant(path);
  if (!node)
    return false_value;
  return value_t(node->hasExpiry());
}

value_t intrinsic_state_is_expired(intrinsics_context *ctx, std::span<const value_t> args) {
  expect_exact(args, 1, "state-is-expired");
  require_root(ctx, "state-is-expired");
  auto &path = as_string(args[0], "state-is-expired");
  auto *node = ctx->root->findDescendant(path);
  if (!node)
    return false_value;
  return value_t(node->isExpired());
}

value_t intrinsic_state_clear_expiry(intrinsics_context *ctx, std::span<const value_t> args) {
  expect_exact(args, 1, "state-clear-expiry");
  require_root(ctx, "state-clear-expiry");
  auto &path = as_string(args[0], "state-clear-expiry");
  auto *node = ctx->root->findDescendant(path);
  if (!node)
    throw std::runtime_error("state-clear-expiry: path not found: " + path);
  node->clearExpiry();
  return nil_value;
}

value_t intrinsic_state_sweep(intrinsics_context *ctx, std::span<const value_t> args) {
  if (args.size() > 1)
    throw std::runtime_error("state-sweep-expired: expected 0-1 argument(s), got " +
                             std::to_string(args.size()));
  require_root(ctx, "state-sweep-expired");
  cvc::state *target = ctx->root;
  if (args.size() == 1) {
    auto &path = as_string(args[0], "state-sweep-expired");
    target = ctx->root->findDescendant(path);
    if (!target)
      return value_t(int64_t(0));
  }
  return value_t(static_cast<int64_t>(target->sweepExpired()));
}

// ---------------------------------------------------------------------------
// Messaging intrinsics
// ---------------------------------------------------------------------------

// §12 channel scoping: resolve a message channel to its scheduler key, mirroring how state paths
// are chrooted. resolve_channel_key (intrinsics.h) does the pure-string rules ('#'-runtime +
// empty-root identity, '/'-global escape, else "<root_path>.channels.<name>"); this adds grant-link
// following — a §12 mount plants a transparent link at <root_path>.channels.<name> (wire_holes), so
// a sender and a granted receiver resolve to ONE absolute key via the SAME resolveLink() state uses
// (no parallel registry, no async_scheduler change). Every msg-* intrinsic resolves once at entry.
std::string resolve_channel(const intrinsics_context *ctx, const std::string &channel) {
  const std::string key = resolve_channel_key(ctx ? ctx->root_path : std::string(), channel);
  // Only a private-by-prefix channel (not a '#'-runtime channel, not a '/'-global, non-empty root)
  // can carry a planted grant link; if one is planted, follow it to the granted target's key.
  if (ctx && ctx->root && !ctx->root_path.empty() && channel.find('#') == std::string::npos &&
      !(!channel.empty() && channel.front() == '/')) {
    const std::string rel = std::string("channels") + cvc::state::SEPARATOR + channel;
    if (cvc::state *n = ctx->root->findDescendant(rel)) {
      // A grant is a transparent link whose TARGET PATH is the shared channel key. Read the target
      // string directly (linkTarget), NOT resolveLink: a channel is a scheduler key, not a state
      // value, so the target node need not exist (resolveLink would report it "broken"). The stored
      // target is app-root-relative, exactly what the sender's own resolve produces — so a granted
      // receiver and the sender agree on one key. Single hop (the §12 mount-grant model).
      if (n->isLink() && n->linkMode() == cvc::state::link_mode::transparent) {
        const std::string tgt = n->linkTarget();
        if (!tgt.empty())
          return tgt; // granted: resolve to the parent/shared channel key
      }
    }
  }
  return key;
}

// §12 channel enforcement (runtime backstop). When the host installed a strict channel_policy on
// this context, a msg-* on an undeclared channel is refused — the completeness layer for DYNAMIC
// channel names the load-time lint can't see. Allowed: a '#'-runtime channel; a declared channel; a
// declared app-root-global (for a '/'-ref); or a channel granted via a planted transparent-link
// (a §12 mount grant). Otherwise throw (the action lane catches it fail-safe and warns once). A
// no-op unless enforce is set (warn/off leave it false — the load lint handled those at load).
void enforce_channel_policy(const intrinsics_context *ctx, const std::string &channel) {
  if (!ctx || !ctx->channels || !ctx->channels->enforce)
    return;
  if (channel.find('#') != std::string::npos)
    return; // runtime-internal channel (tick/key/pointer) — never enforced
  const channel_policy &p = *ctx->channels;
  if (!channel.empty() && channel.front() == '/') {
    const std::string g = channel.substr(1);
    if (std::find(p.global.begin(), p.global.end(), g) != p.global.end())
      return; // a declared app-root-global
    throw std::runtime_error("channel policy: undeclared app-root-global '/" + g +
                             "' (declare it as a global in the channels: block)");
  }
  if (std::find(p.declared.begin(), p.declared.end(), channel) != p.declared.end())
    return;        // a declared channel
  if (ctx->root) { // or a channel granted via a planted transparent-link (a §12 mount grant)
    const std::string rel = std::string("channels") + cvc::state::SEPARATOR + channel;
    if (cvc::state *n = ctx->root->findDescendant(rel))
      if (n->isLink() && n->linkMode() == cvc::state::link_mode::transparent &&
          !n->linkTarget().empty())
        return;
  }
  throw std::runtime_error("channel policy: undeclared channel '" + channel +
                           "' (add it to the channels: block, or relax lint.channels)");
}

value_t intrinsic_msg_send(intrinsics_context *ctx, std::span<const value_t> args) {
  expect_min(args, 2, "msg-send");
  require_root(ctx, "msg-send");
  auto &path = as_string(args[0], "msg-send");
  enforce_channel_policy(ctx,
                         path); // §12 runtime enforcement (strict policy → throw if undeclared)
  auto &payload = as_string_or_bytes(args[1], "msg-send"); // text OR binary
  const bool binary = is_bytes_value(args[1]);
  std::string content_type = binary ? "application/octet-stream" : "text/plain";
  if (args.size() > 2)
    content_type = as_string(args[2], "msg-send");
  auto *node = ctx->root->findDescendant(path);
  if (!node) {
    // Create the node so we can send to it
    node = &(*ctx->root)(path);
  }
  auto result = node->sendMessage(payload, content_type, /*hop_budget=*/64, /*binary=*/binary);
  // Update message counters on the currently executing process.
  process *proc = nullptr;
  if (ctx->sched && ctx->sched->current_process())
    proc = ctx->sched->current_process().get();
  if (proc) {
    proc->message_count++;
    proc->message_bytes += payload.size();
  }
  // Also update ctx->proc if it's a different object (for test fixtures).
  if (ctx->proc && ctx->proc.get() != proc) {
    ctx->proc->message_count++;
    ctx->proc->message_bytes += payload.size();
  }
  const std::string status_str =
      result.status == cvc::state::send_message_result::status_kind::delivered ? "delivered"
                                                                               : "error";

  // Return value to the SENDER: {status, path} — unchanged contract (callers/tests rely on this).
  std::vector<std::pair<std::string, value_t>> ret_entries;
  ret_entries.emplace_back("status", value_t(status_str));
  ret_entries.emplace_back("path", value_t(result.resolved_path));
  auto ret = make_dict(std::move(ret_entries));

  // Deliver an ENVELOPE to receivers on this channel: {status, path, content_type, payload}. The
  // payload keeps the ORIGINAL value type — a string stays a string, bytes stay bytes — so a
  // (msg-recv) on this channel gets the actual binary/text data (not just the {status,path} ack it
  // used to get). The SCHEDULER key is chroot-scoped (resolve_channel); the state-side sendMessage
  // above stays on the raw path — the two buses are independent, as before.
  if (ctx->sched) {
    std::vector<std::pair<std::string, value_t>> env_entries;
    env_entries.emplace_back("status", value_t(status_str));
    env_entries.emplace_back("path", value_t(result.resolved_path));
    env_entries.emplace_back("content_type", value_t(content_type));
    env_entries.emplace_back("payload", args[1]); // original value_t — binary preserved
    ctx->sched->deliver_to_receivers(resolve_channel(ctx, path), make_dict(std::move(env_entries)));
  }

  return ret;
}

value_t intrinsic_msg_recv(intrinsics_context *ctx, std::span<const value_t> args) {
  expect_exact(args, 1, "msg-recv");
  require_sched(ctx, "msg-recv");
  // An async call's future delivers no message: receiving it is awaiting it.
  if (is_async_future(args[0]))
    return intrinsic_await(ctx, args);
  // Accept a future handle {"__future__": chan} (from an async producer like (http-get-async)) OR a
  // raw channel string. A future's channel is an already-resolved '#'-channel (policy-exempt); a
  // raw string goes through §12 policy + chroot-scoping as before.
  std::string ch;
  if (const std::optional<std::string> fchan = future_channel_of(args[0])) {
    ch = *fchan;
  } else {
    const std::string &raw = as_string(args[0], "msg-recv");
    enforce_channel_policy(ctx, raw);
    ch = resolve_channel(ctx, raw);
  }

  int pid = ctx->sched->current_pid();
  if (pid < 0)
    pid = ctx->pid; // fallback for unit-test contexts
  process *proc = nullptr;
  if (ctx->sched->current_process())
    proc = ctx->sched->current_process().get();
  else if (ctx->proc)
    proc = ctx->proc.get();

  return park_on_channel(ctx->sched, proc, pid, ch);
}

value_t intrinsic_msg_pending(intrinsics_context *ctx, std::span<const value_t> args) {
  expect_exact(args, 1, "msg-pending");
  require_sched(ctx, "msg-pending");
  const std::string &raw = as_string(args[0], "msg-pending");
  enforce_channel_policy(ctx, raw);                 // §12 runtime enforcement (strict → throw)
  const std::string ch = resolve_channel(ctx, raw); // §12 chroot-scoped scheduler key
  return value_t(static_cast<int64_t>(ctx->sched->pending_message_count(ch)));
}

} // anonymous namespace

// ---------------------------------------------------------------------------
// Registration
// ---------------------------------------------------------------------------

void register_intrinsics(environment_ptr env, intrinsics_context *ctx) {
  auto reg = [&](const std::string &name, auto fn) {
    builtins::register_fn(env, name,
                          [ctx, fn](std::span<const value_t> args) { return fn(ctx, args); });
  };

  // State tree
  reg("state-get", intrinsic_state_get);
  reg("state-set", intrinsic_state_set);
  reg("state-children", intrinsic_state_children);
  reg("state-exists", intrinsic_state_exists);
  reg("state-delete", intrinsic_state_delete);
  reg("state-data-get", intrinsic_state_data_get);
  reg("state-data-set", intrinsic_state_data_set);
  reg("state-json", intrinsic_state_json);
  reg("state-from-json", intrinsic_state_from_json);
  reg("state-set-tree", intrinsic_state_set_tree);
  reg("state-root-path", intrinsic_state_root_path);
  reg("state-watch", intrinsic_state_watch);
  reg("state-unwatch", intrinsic_state_unwatch);
  reg("state-incr", intrinsic_state_incr);
  reg("state-cas", intrinsic_state_cas);
  reg("state-swap", intrinsic_state_swap);

  // Scheduler
  reg("spawn", intrinsic_spawn);
  reg("fork", intrinsic_fork);
  reg("self-pid", intrinsic_self_pid);
  reg("self-uid", intrinsic_self_uid);
  reg("kill", intrinsic_kill);
  reg("pause", intrinsic_pause);
  reg("resume", intrinsic_resume);
  reg("sleep", intrinsic_sleep);
  reg("await", intrinsic_await);
  reg("await-all", intrinsic_await_all);
  reg("ps", intrinsic_ps);
  reg("inspect", intrinsic_inspect);

  // Resource queries
  reg("memory-usage", intrinsic_memory_usage);
  reg("memory-limit", intrinsic_memory_limit);
  reg("time-elapsed", intrinsic_time_elapsed);
  reg("time-limit", intrinsic_time_limit);
  reg("message-count", intrinsic_message_count);
  reg("message-limit", intrinsic_message_limit);
  reg("message-bytes", intrinsic_message_bytes);
  reg("message-bytes-limit", intrinsic_message_bytes_limit);
  reg("step-count", intrinsic_step_count);
  reg("budget-usage", intrinsic_budget_usage);

  // System identity
  reg("cluster-id", intrinsic_cluster_id);
  reg("node-id", intrinsic_node_id);
  reg("scheduler-stats", intrinsic_scheduler_stats);

  // Expiry
  reg("state-expire", intrinsic_state_expire);
  reg("state-expire-at", intrinsic_state_expire_at);
  reg("state-has-expiry", intrinsic_state_has_expiry);
  reg("state-is-expired", intrinsic_state_is_expired);
  reg("state-clear-expiry", intrinsic_state_clear_expiry);
  reg("state-sweep-expired", intrinsic_state_sweep);

  // Messaging
  reg("msg-send", intrinsic_msg_send);
  reg("msg-recv", intrinsic_msg_recv);
  reg("msg-pending", intrinsic_msg_pending);

  // Locks: with-lock, locks-held, lock-query, %lock-wait (docs/STATE_EXEC_LOCKS.md)
  detail::register_lock_intrinsics(env, ctx);
}

std::function<spawn_setup(const intrinsics_context &)> make_spawn_context(
    environment_ptr base,
    std::function<void(const environment_ptr &, intrinsics_context &)> after_register) {
  if (!base)
    base = builtins::make_default_environment();
  return [base = std::move(base),
          after_register = std::move(after_register)](const intrinsics_context &parent) {
    auto child = std::make_shared<intrinsics_context>();
    child->sched = parent.sched;
    child->root = parent.root;
    child->tracker = parent.tracker;
    child->root_path = parent.root_path;
    child->uid = parent.uid;
    child->cluster_id = parent.cluster_id;
    child->node_id = parent.node_id;
    child->channels = parent.channels;
    child->document = parent.document;
    child->lock_default_timeout = parent.lock_default_timeout;
    child->lock_wait_allowed = parent.lock_wait_allowed;
    child->spawn_context = parent.spawn_context; // grandchildren get one too
    // A placeholder, as hosts give: the intrinsics act for the running process.
    child->proc = make_process();
    child->pid = -1;
    environment_ptr env = environment::extend(base);
    register_intrinsics(env, child.get());
    if (after_register)
      after_register(env, *child);
    return spawn_setup{env, child};
  };
}

namespace detail {

namespace {
// `digits` (a decimal digit string) plus `dir` (+1 or -1) in the last place, or "" when that
// changes how many digits it has.
std::string step_last_digit(std::string digits, int dir) {
  for (std::size_t i = digits.size(); i-- > 0;) {
    if (dir > 0 && digits[i] == '9') {
      digits[i] = '0';
    } else if (dir < 0 && digits[i] == '0') {
      digits[i] = '9';
    } else {
      digits[i] = static_cast<char>(digits[i] + dir);
      return digits[0] == '0' ? std::string() : digits;
    }
  }
  return std::string();
}
} // namespace

std::string format_number_shortest_fallback(double d) {
  if (std::isnan(d))
    return std::signbit(d) ? "-nan" : "nan";
  if (std::isinf(d))
    return d < 0 ? "-inf" : "inf";
  const std::string sign = std::signbit(d) ? "-" : "";
  const double mag = std::fabs(d);
  if (mag == 0)
    return sign + "0";

  // The shortest digits that read back as `mag`, and the power of ten of the first. %.*e gives
  // the correctly rounded p digits. Where `mag` sits at a power of two its rounding interval is
  // lopsided, so those may miss while the p-digit neighbour on the other side reads back: that
  // one is then the shortest, and the one to_chars writes. Candidates are read back as
  // "<digits>e<exp>", with no radix character, so the locale does not matter.
  std::string digits;
  int exp10 = 0;
  for (int p = 1; p <= 17 && digits.empty(); ++p) {
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%.*e", p - 1, mag);
    std::string m;
    const char *c = buf;
    for (; *c && *c != 'e' && *c != 'E'; ++c)
      if (*c >= '0' && *c <= '9')
        m += *c;
    const int e = *c ? std::atoi(c + 1) : 0;
    const auto reads_back = [&](const std::string &cand) {
      const std::string text = cand + "e" + std::to_string(e - (p - 1));
      return std::strtod(text.c_str(), nullptr) == mag;
    };
    for (const std::string &cand : {m, step_last_digit(m, -1), step_last_digit(m, +1)})
      if (!cand.empty() && reads_back(cand)) {
        digits = cand;
        exp10 = e;
        break;
      }
  }
  while (digits.size() > 1 && digits.back() == '0')
    digits.pop_back();

  const int n = static_cast<int>(digits.size());
  const int ae = exp10 < 0 ? -exp10 : exp10;
  std::string sci = digits.substr(0, 1);
  if (n > 1)
    sci += "." + digits.substr(1);
  sci += std::string(exp10 < 0 ? "e-" : "e+") + (ae < 10 ? "0" : "") + std::to_string(ae);
  std::string fixed;
  if (exp10 >= n - 1) {
    // A whole number. Its exact digits are as long as the shortest digits padded with zeros, and
    // of texts of one length to_chars writes the closest, so the exact value (%.0f prints it, and
    // has no radix character).
    char whole[400];
    std::snprintf(whole, sizeof(whole), "%.0f", mag);
    fixed = whole;
  } else if (exp10 >= 0)
    fixed = digits.substr(0, exp10 + 1) + "." + digits.substr(exp10 + 1);
  else
    fixed = "0." + std::string(static_cast<std::size_t>(-exp10 - 1), '0') + digits;
  return sign + (fixed.size() <= sci.size() ? fixed : sci);
}

} // namespace detail

std::string format_number_shortest(double d) {
#if defined(__cpp_lib_to_chars) && __cpp_lib_to_chars >= 201611L
  char buf[32];
  const auto r = std::to_chars(buf, buf + sizeof(buf), d);
  return std::string(buf, r.ptr);
#else
  return detail::format_number_shortest_fallback(d);
#endif
}

void apply_chroot(intrinsics_context &ctx, cvc::state &tree_root, const std::string &root_path) {
  if (root_path.empty()) {
    ctx.root = &tree_root;
    ctx.root_path.clear();
    return;
  }
  // Navigate or create the subtree node
  ctx.root = &tree_root(root_path);
  ctx.root_path = root_path;
}

std::string resolve_channel_key(const std::string &root_path, const std::string &channel) {
  if (channel.find('#') != std::string::npos)
    return channel; // runtime-internal channel (tick/key/pointer) — never scoped
  if (!channel.empty() && channel.front() == '/')
    return channel.substr(1); // explicit app-root-global (the '/' escape; the loader gates its use)
  if (root_path.empty())
    return channel; // root scope: identity — backward-compatible with every raw-key msg-* caller
  return root_path + cvc::state::SEPARATOR + "channels" + cvc::state::SEPARATOR + channel;
}

value_t make_future(const std::string &channel) {
  return make_dict({{"__future__", value_t(channel)}});
}

std::optional<std::string> future_channel_of(const value_t &v) {
  const dict_ptr *dp = std::get_if<dict_ptr>(&v.v);
  if (!dp || !*dp)
    return std::nullopt;
  const std::vector<std::pair<std::string, value_t>> &entries = **dp;
  // An async call's future (async_host.h) has its handle as a second entry.
  if (entries.empty() || entries.size() > 2 || entries[0].first != "__future__" ||
      (entries.size() == 2 && !is_async_future(v)))
    return std::nullopt;
  const std::string *chan = std::get_if<std::string>(&entries[0].second.v);
  if (!chan)
    return std::nullopt;
  return *chan;
}

value_t park_on_channel(scheduler_base *sched, process *proc, int pid, const std::string &ch) {
  // A message that arrived before the park is returned without suspending: first the process inbox,
  // then the scheduler's per-channel pending queue.
  if (proc && !proc->inbox.empty()) {
    value_t msg = std::move(proc->inbox.front());
    proc->inbox.pop();
    return msg;
  }
  if (const std::optional<value_t> pending = sched->pop_pending_message(ch))
    return *pending;

  // A park needs an ENCLOSING frame to receive the value deliver_to_receivers patches in. During
  // the native call the apply frame is on top of the stack; size 1 means it is the ONLY frame — a
  // top-level call whose pop would empty the stack (done=true) and skip the patch, leaving a
  // done-and-waiting zombie. (size 0 == a direct unit-test call with no evaluator frames — allowed;
  // the test drives delivery itself.)
  if (proc && proc->state.stack.size() == 1)
    throw std::runtime_error("cannot suspend a top-level call (no enclosing expression to receive "
                             "the value); wrap it, e.g. bind the result or use (begin …)");

  if (!sched->receive_message(pid, ch))
    throw std::runtime_error("cannot suspend process " + std::to_string(pid));
  // Nil placeholder — deliver_to_receivers overwrites the enclosing frame's results.back() with the
  // delivered value when the message arrives on `ch`.
  return nil_value;
}

} // namespace cvc::state_exec
