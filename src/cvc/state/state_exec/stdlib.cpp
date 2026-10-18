#include <algorithm>
#include <cmath>
#include <cvc/state/state_exec/builtins.h>
#include <cvc/state/state_exec/data_job.h> // the sliced work of the materializers
#include <cvc/state/state_exec/stdlib.h>
#include <cvc/state/state_exec/utf8.h> // SE-1: codepoint-correct length/index/slice
#include <memory>
#include <numeric>
#include <random>
#include <span>
#include <sstream>
#include <stdexcept>
#include <string_view>
#include <unordered_map>

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

// SE-3: extract the raw byte buffer of a bytes value.
const std::string &as_bytes(const value_t &v, const char *name) {
  if (auto *b = std::get_if<bytes_value>(&v.v))
    return b->data;
  throw std::runtime_error(std::string(name) + ": expected bytes, got " + v.type_name());
}

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

// Allocation estimates for eval_alloc_check(), saturating so a huge count never wraps small.
std::size_t sat_add(std::size_t a, std::size_t b) { return a > SIZE_MAX - b ? SIZE_MAX : a + b; }

// One list of n elements: the vector, make_list's shared control block, and the elements.
std::size_t list_bytes(std::size_t n) {
  return sat_add(sizeof(std::vector<value_t>) + 2 * sizeof(void *),
                 alloc_bytes(n, sizeof(value_t)));
}

// Long loops inside one native call honour the evaluation deadline (see collect in builtins.cpp).
void poll_deadline(const char *name) {
  if (eval_deadline_expired())
    throw std::runtime_error(std::string(name) + ": evaluation exceeded time budget");
}

// A list argument by owning pointer (a null one reads as empty).
list_ptr as_list_ptr(const value_t &v, const char *name) {
  if (auto *p = std::get_if<list_ptr>(&v.v))
    return *p ? *p : std::make_shared<std::vector<value_t>>();
  throw std::runtime_error(std::string(name) + ": expected list, got " + v.type_name());
}

dict_ptr as_dict_ptr(const value_t &v, const char *name, const char *what) {
  if (auto *p = std::get_if<dict_ptr>(&v.v))
    return *p ? *p : std::make_shared<ordered_map>();
  throw std::runtime_error(std::string(name) + ": " + what);
}

// A function argument: a native function or a DSL closure.
const value_t &as_function(const value_t &v, const char *name, const char *which) {
  if (std::holds_alternative<native_fn>(v.v) ||
      (std::holds_alternative<closure_ptr>(v.v) && std::get<closure_ptr>(v.v)))
    return v;
  throw std::runtime_error(std::string(name) + ": " + which + " argument must be a function");
}

// The order collections.sort sorts by: numbers by value, strings by bytes; anything else is
// unordered (a merge sort keeps such elements stable, whatever the comparison says).
bool sort_less(const value_t &a, const value_t &b) {
  if (auto *ai = std::get_if<int64_t>(&a.v)) {
    if (auto *bi = std::get_if<int64_t>(&b.v))
      return *ai < *bi;
    if (auto *bd = std::get_if<double>(&b.v))
      return static_cast<double>(*ai) < *bd;
  }
  if (auto *ad = std::get_if<double>(&a.v)) {
    if (auto *bi = std::get_if<int64_t>(&b.v))
      return *ad < static_cast<double>(*bi);
    if (auto *bd = std::get_if<double>(&b.v))
      return *ad < *bd;
  }
  if (auto *as = std::get_if<std::string>(&a.v)) {
    if (auto *bs = std::get_if<std::string>(&b.v))
      return *as < *bs;
  }
  return false;
}

// ---------------------------------------------------------------------------
// Sliced work. The functions that build a result, scan a string or call a function do it as a
// data_job (data_job.h), a slice at a time: on an evaluator that slices (data.step_nodes > 0) a
// call spreads over as many steps as it needs, each counted against max_steps and max_time, and
// the functions a job calls (collections.map/filter/reduce) run as the evaluator's own frames.
// A slice is so many nodes: one per element visited, one per kDataNodeBytes bytes scanned or
// copied. The allocation check comes first, once the result's size is known. A job reads its
// string arguments in place during the call's own slice and copies them only if it outlives the
// call (own_inputs); lists and dicts it holds by owning pointer and reads by index, so one
// changed between two slices changes the answer, never memory safety.
// ---------------------------------------------------------------------------

constexpr std::size_t npos = std::string_view::npos;

// One step() call's share of the work: `budget` nodes (0 = no limit), counted into the job.
class slice {
public:
  slice(std::size_t &nodes, const char *who, std::size_t budget)
      : nodes_(nodes), who_(who), budget_(budget) {}

  bool spent() const { return budget_ != 0 && used_ >= budget_; }
  // What is left for a nested walk (0 = no limit; never 0 while limited).
  std::size_t left() const { return budget_ == 0 ? 0 : budget_ > used_ ? budget_ - used_ : 1; }
  // How many of the next `n` bytes it covers: all without a limit, else at least one node's worth.
  std::size_t bytes(std::size_t n) const {
    const std::size_t nodes = left();
    return nodes == 0 || nodes > n / kDataNodeBytes ? n : nodes * kDataNodeBytes;
  }
  void count(std::size_t nodes) {
    const std::size_t before = nodes_;
    nodes_ += nodes;
    used_ += nodes;
    if ((before >> 6) != (nodes_ >> 6))
      poll_deadline(who_); // about every 64 nodes
  }
  void count_bytes(std::size_t n) { count((n + kDataNodeBytes - 1) / kDataNodeBytes); }

private:
  std::size_t &nodes_;
  const char *who_;
  std::size_t budget_;
  std::size_t used_ = 0;
};

class stdlib_job : public data_job {
protected:
  explicit stdlib_job(const char *who) : who_(who) {}

  slice begin(std::size_t budget) { return slice(nodes_, who_, budget); }

  // Stop for `fn` applied to `args`, both the job's own (see data_job::call_request).
  bool call(const value_t &fn, std::span<const value_t> args) {
    call_ = call_request{&fn, args, who_};
    return false;
  }

  // Copy what `view` still has from `from` into `store` and point it there (own_inputs).
  static void own(std::string_view &view, std::string &store, std::size_t from = 0) {
    store.assign(view.substr(std::min(from, view.size())));
    view = store;
  }

  // Append `part` from `off` to `out`, a slice's worth at a time. True once all of it is in.
  static bool append_part(std::string &out, std::string_view part, std::size_t &off, slice &sl) {
    off = std::min(off, part.size()); // the part changed between slices
    while (off < part.size()) {
      if (sl.spent())
        return false;
      const std::size_t n = sl.bytes(part.size() - off);
      out.append(part.substr(off, n));
      off += n;
      sl.count_bytes(n);
    }
    return true;
  }

  const char *who_;
};

// The first match of `needle` (not empty) in `hay` that starts in [pos, pos + window).
std::size_t find_from(std::string_view hay, std::string_view needle, std::size_t pos,
                      std::size_t window) {
  if (pos >= hay.size())
    return npos;
  const std::size_t end = std::min(hay.size(), sat_add(sat_add(pos, window), needle.size() - 1));
  const std::size_t at = hay.substr(pos, end - pos).find(needle);
  return at == npos ? npos : pos + at;
}

// Counts the non-overlapping matches of `needle` (not empty) in `hay`, left to right.
struct match_counter {
  std::size_t pos = 0;
  std::size_t count = 0;

  bool run(std::string_view hay, std::string_view needle, slice &sl) {
    while (pos < hay.size()) {
      if (sl.spent())
        return false;
      const std::size_t window = sl.bytes(hay.size() - pos);
      const std::size_t at = find_from(hay, needle, pos, window);
      if (at == npos) {
        pos += window;
        sl.count_bytes(window);
        continue;
      }
      sl.count_bytes(at + needle.size() - pos);
      ++count;
      pos = at + needle.size();
    }
    return true;
  }
};

class split_job final : public stdlib_job {
public:
  split_job(std::string_view s, std::string_view delim)
      : stdlib_job("string.split"), s_(s), delim_(delim) {}

  void own_inputs() override {
    own(s_, s_store_);
    own(delim_, delim_store_);
  }

  bool step(std::size_t budget) override {
    slice sl = begin(budget);
    if (!sized_) {
      // Every part costs a list element, so many short parts outgrow the string: count first.
      if (!(delim_.empty() ? count_codepoints(sl) : matches_.run(s_, delim_, sl)))
        return false;
      sized_ = true;
      const std::size_t count = delim_.empty() ? codepoints_ : matches_.count + 1;
      eval_alloc_check(sat_add(list_bytes(count), s_.size()), "string.split");
      parts_.reserve(count);
    }
    return delim_.empty() ? split_codepoints(sl) : split_at_delim(sl);
  }

  value_t result() override { return make_list(std::move(parts_)); }

private:
  bool count_codepoints(slice &sl) {
    while (pos_ < s_.size()) {
      if (sl.spent())
        return false;
      const std::size_t start = pos_, end = pos_ + sl.bytes(s_.size() - pos_);
      while (pos_ < end) {
        char32_t cp;
        std::size_t len;
        utf8::decode(s_, pos_, cp, len);
        pos_ += len;
        ++codepoints_;
      }
      sl.count_bytes(pos_ - start);
    }
    pos_ = 0;
    return true;
  }

  // SE-1 (A3): split into CODEPOINTS, not bytes — each element is one whole UTF-8 codepoint, so
  // a multibyte character is never shredded into invalid fragments.
  bool split_codepoints(slice &sl) {
    while (pos_ < s_.size()) {
      if (sl.spent())
        return false;
      char32_t cp;
      std::size_t len;
      utf8::decode(s_, pos_, cp, len);
      parts_.emplace_back(std::string(s_.substr(pos_, len)));
      pos_ += len;
      sl.count(1);
    }
    return true;
  }

  bool split_at_delim(slice &sl) {
    for (;;) {
      if (sl.spent())
        return false;
      std::size_t at = npos;
      if (scan_ < s_.size()) {
        const std::size_t window = sl.bytes(s_.size() - scan_);
        at = find_from(s_, delim_, scan_, window);
        if (at == npos) {
          scan_ += window;
          sl.count_bytes(window);
          if (scan_ < s_.size())
            continue;
        }
      }
      if (at == npos) {
        parts_.emplace_back(std::string(s_.substr(start_)));
        sl.count(1);
        return true;
      }
      parts_.emplace_back(std::string(s_.substr(start_, at - start_)));
      sl.count_bytes(at + delim_.size() - scan_);
      sl.count(1);
      start_ = scan_ = at + delim_.size();
    }
  }

  std::string_view s_, delim_;
  std::string s_store_, delim_store_;
  bool sized_ = false;
  match_counter matches_;
  std::size_t codepoints_ = 0;
  std::size_t pos_ = 0;   // codepoint splitting
  std::size_t start_ = 0; // the part being found
  std::size_t scan_ = 0;  // where the search for its end goes on
  std::vector<value_t> parts_;
};

class join_job final : public stdlib_job {
public:
  join_job(list_ptr list, std::string_view sep)
      : stdlib_job("string.join"), list_(std::move(list)), sep_(sep) {}

  void own_inputs() override { own(sep_, sep_store_); }

  bool step(std::size_t budget) override {
    slice sl = begin(budget);
    if (!sized_) {
      for (; i_ < list_->size(); ++i_) {
        if (sl.spent())
          return false;
        total_ = sat_add(total_, as_string((*list_)[i_], "string.join").size());
        sl.count(1);
      }
      if (i_ > 0)
        total_ = sat_add(total_, alloc_bytes(i_ - 1, sep_.size()));
      eval_alloc_check(total_, "string.join");
      out_.reserve(total_);
      sized_ = true;
      i_ = 0;
    }
    while (i_ < list_->size()) {
      if (sl.spent())
        return false;
      if (i_ > 0 && !sep_done_) {
        if (!append_part(out_, sep_, off_, sl))
          return false;
        sep_done_ = true;
        off_ = 0;
      }
      const std::string &part = as_string((*list_)[i_], "string.join");
      if (const std::size_t need = sat_add(out_.size(), part.size()); need > total_) {
        eval_alloc_check(need, "string.join"); // the list grew between slices
        total_ = need;
      }
      if (!append_part(out_, part, off_, sl))
        return false;
      sl.count(1);
      ++i_;
      off_ = 0;
      sep_done_ = false;
    }
    return true;
  }

  value_t result() override { return value_t(std::move(out_)); }

private:
  list_ptr list_;
  std::string_view sep_;
  std::string sep_store_;
  bool sized_ = false;
  std::size_t total_ = 0;
  std::size_t i_ = 0;
  std::size_t off_ = 0;
  bool sep_done_ = false;
  std::string out_;
};

class replace_job final : public stdlib_job {
public:
  replace_job(std::string_view s, std::string_view from, std::string_view to)
      : stdlib_job("string.replace"), s_(s), from_(from), to_(to) {}

  void own_inputs() override {
    own(s_, s_store_);
    own(from_, from_store_);
    own(to_, to_store_);
  }

  bool step(std::size_t budget) override {
    slice sl = begin(budget);
    if (!sized_) {
      // Non-overlapping matches, left to right. A longer `to` multiplies the size: count first.
      if (!matches_.run(s_, from_, sl))
        return false;
      sized_ = true;
      const std::size_t count = matches_.count;
      if (count > 0) {
        const std::size_t size =
            sat_add(s_.size() - count * from_.size(), alloc_bytes(count, to_.size()));
        eval_alloc_check(size, "string.replace");
        out_.reserve(size);
      } else {
        out_.reserve(s_.size());
      }
    }
    for (;;) {
      if (sl.spent())
        return false;
      if (in_to_) {
        if (!append_part(out_, to_, off_, sl))
          return false;
        in_to_ = false;
        off_ = 0;
        continue;
      }
      std::size_t at = npos;
      if (scan_ < s_.size()) {
        const std::size_t window = sl.bytes(s_.size() - scan_);
        at = find_from(s_, from_, scan_, window);
        if (at == npos) {
          // No match starts before scan_ + window, so up to there `s` goes out as it is.
          const std::size_t upto = std::min(s_.size(), scan_ + window);
          out_.append(s_.substr(start_, upto - start_));
          sl.count_bytes(upto - scan_);
          start_ = scan_ = upto;
          continue;
        }
      }
      if (at == npos) {
        out_.append(s_.substr(start_));
        return true;
      }
      out_.append(s_.substr(start_, at - start_));
      sl.count_bytes(at + from_.size() - scan_);
      start_ = scan_ = at + from_.size();
      in_to_ = true;
    }
  }

  value_t result() override { return value_t(std::move(out_)); }

private:
  std::string_view s_, from_, to_;
  std::string s_store_, from_store_, to_store_;
  bool sized_ = false;
  match_counter matches_;
  std::size_t start_ = 0, scan_ = 0; // what of `s` is out, and where the search goes on
  bool in_to_ = false;               // appending `to` for the match just found
  std::size_t off_ = 0;
  std::string out_;
};

// Copies a byte range (string.encode, bytes.slice) into a bytes value.
class copy_job final : public stdlib_job {
public:
  copy_job(const char *who, std::string_view from) : stdlib_job(who), from_(from) {
    out_.reserve(from.size());
  }

  void own_inputs() override {
    own(from_, store_, off_);
    off_ = 0;
  }

  bool step(std::size_t budget) override {
    slice sl = begin(budget);
    return append_part(out_, from_, off_, sl);
  }

  value_t result() override { return make_bytes(std::move(out_)); }

private:
  std::string_view from_;
  std::string store_;
  std::size_t off_ = 0;
  std::string out_;
};

class concat_job final : public stdlib_job {
public:
  concat_job(std::vector<std::string_view> parts, std::size_t total)
      : stdlib_job("bytes.concat"), parts_(std::move(parts)) {
    out_.reserve(total);
  }

  void own_inputs() override {
    // What is still to be appended: the rest of the current part and the parts after it.
    for (std::size_t i = next_; i < parts_.size(); ++i)
      store_.emplace_back(parts_[i].substr(i == next_ ? std::min(off_, parts_[i].size()) : 0));
    parts_.assign(store_.begin(), store_.end());
    next_ = 0;
    off_ = 0;
  }

  bool step(std::size_t budget) override {
    slice sl = begin(budget);
    for (; next_ < parts_.size(); ++next_, off_ = 0)
      if (!append_part(out_, parts_[next_], off_, sl))
        return false;
    return true;
  }

  value_t result() override { return make_bytes(std::move(out_)); }

private:
  std::vector<std::string_view> parts_;
  std::vector<std::string> store_;
  std::size_t next_ = 0;
  std::size_t off_ = 0;
  std::string out_;
};

// bytes.decode: checks the bytes are well-formed UTF-8 as it copies them.
class decode_job final : public stdlib_job {
public:
  explicit decode_job(std::string_view bytes) : stdlib_job("bytes.decode"), b_(bytes) {
    out_.reserve(bytes.size());
  }

  void own_inputs() override {
    own(b_, store_, pos_);
    pos_ = 0;
  }

  bool step(std::size_t budget) override {
    slice sl = begin(budget);
    while (pos_ < b_.size()) {
      if (sl.spent())
        return false;
      const std::size_t start = pos_, end = pos_ + sl.bytes(b_.size() - pos_);
      while (pos_ < end) {
        char32_t cp;
        std::size_t len;
        if (!utf8::decode(b_, pos_, cp, len))
          throw std::runtime_error("bytes.decode: invalid UTF-8");
        pos_ += len;
      }
      out_.append(b_.substr(start, pos_ - start));
      sl.count_bytes(pos_ - start);
    }
    return true;
  }

  value_t result() override { return value_t(std::move(out_)); }

private:
  std::string_view b_;
  std::string store_;
  std::size_t pos_ = 0;
  std::string out_;
};

// collections.map / filter: one call of `fn` per element.
class map_job final : public stdlib_job {
public:
  map_job(const char *who, list_ptr list, value_t fn, bool filter)
      : stdlib_job(who), list_(std::move(list)), fn_(std::move(fn)), filter_(filter) {
    if (!filter_)
      out_.reserve(list_->size());
  }

  bool step(std::size_t budget) override {
    slice sl = begin(budget);
    if (i_ >= list_->size())
      return true;
    if (sl.spent())
      return false;
    arg_ = (*list_)[i_++];
    sl.count(1);
    return call(fn_, {&arg_, 1});
  }

  void call_returned(value_t value) override {
    if (!filter_)
      out_.push_back(std::move(value));
    else if (value.is_truthy())
      out_.push_back(std::move(arg_));
  }

  value_t result() override { return make_list(std::move(out_)); }

private:
  list_ptr list_;
  value_t fn_;
  bool filter_;
  std::size_t i_ = 0;
  value_t arg_;
  std::vector<value_t> out_;
};

class reduce_job final : public stdlib_job {
public:
  reduce_job(list_ptr list, value_t fn, value_t init)
      : stdlib_job("collections.reduce"), list_(std::move(list)), fn_(std::move(fn)),
        acc_(std::move(init)) {}

  bool step(std::size_t budget) override {
    slice sl = begin(budget);
    if (i_ >= list_->size())
      return true;
    if (sl.spent())
      return false;
    args_[0] = std::move(acc_);
    args_[1] = (*list_)[i_++];
    sl.count(1);
    return call(fn_, {args_, 2});
  }

  void call_returned(value_t value) override { acc_ = std::move(value); }

  value_t result() override { return std::move(acc_); }

private:
  list_ptr list_;
  value_t fn_;
  value_t acc_;
  std::size_t i_ = 0;
  value_t args_[2];
};

class zip_job final : public stdlib_job {
public:
  zip_job(list_ptr a, list_ptr b, std::size_t n)
      : stdlib_job("collections.zip"), a_(std::move(a)), b_(std::move(b)), n_(n) {
    out_.reserve(n);
  }

  bool step(std::size_t budget) override {
    slice sl = begin(budget);
    for (; i_ < n_ && i_ < a_->size() && i_ < b_->size(); ++i_) {
      if (sl.spent())
        return false;
      out_.push_back(make_list({(*a_)[i_], (*b_)[i_]}));
      sl.count(1);
    }
    return true;
  }

  value_t result() override { return make_list(std::move(out_)); }

private:
  list_ptr a_, b_;
  std::size_t n_;
  std::size_t i_ = 0;
  std::vector<value_t> out_;
};

class flatten_job final : public stdlib_job {
public:
  explicit flatten_job(list_ptr list) : stdlib_job("collections.flatten"), list_(std::move(list)) {}

  bool step(std::size_t budget) override {
    slice sl = begin(budget);
    if (!sized_) {
      // The same sublist may appear many times, so the result can outgrow the argument: count
      // the elements first.
      for (; i_ < list_->size(); ++i_) {
        if (sl.spent())
          return false;
        const auto *sub = std::get_if<list_ptr>(&(*list_)[i_].v);
        total_ = sat_add(total_, sub ? (*sub ? (*sub)->size() : 0) : 1);
        sl.count(1);
      }
      eval_alloc_check(list_bytes(total_), "collections.flatten");
      out_.reserve(total_);
      sized_ = true;
      i_ = 0;
    }
    for (; i_ < list_->size(); ++i_, j_ = 0) {
      const value_t &item = (*list_)[i_];
      if (const auto *sub = std::get_if<list_ptr>(&item.v)) {
        const list_ptr &elems = *sub;
        for (; elems && j_ < elems->size(); ++j_) {
          if (sl.spent())
            return false;
          add((*elems)[j_]);
          sl.count(1);
        }
      } else {
        if (sl.spent())
          return false;
        add(item);
        sl.count(1);
      }
    }
    return true;
  }

  value_t result() override { return make_list(std::move(out_)); }

private:
  void add(const value_t &v) {
    if (out_.size() >= total_) { // the list grew between slices
      eval_alloc_check(list_bytes(out_.size() + 1), "collections.flatten");
      total_ = out_.size() + 1;
    }
    out_.push_back(v);
  }

  list_ptr list_;
  bool sized_ = false;
  std::size_t total_ = 0;
  std::size_t i_ = 0, j_ = 0;
  std::vector<value_t> out_;
};

// A bottom-up merge sort, one node per element moved: n log n nodes, and stable.
class sort_job final : public stdlib_job {
public:
  explicit sort_job(list_ptr list) : stdlib_job("collections.sort"), list_(std::move(list)) {}

  bool step(std::size_t budget) override {
    slice sl = begin(budget);
    if (!copied_) {
      for (; next_ < list_->size(); ++next_) {
        if (sl.spent())
          return false;
        a_.push_back((*list_)[next_]);
        sl.count(1);
      }
      copied_ = true;
      n_ = a_.size();
      if (n_ > 1) {
        b_.resize(n_);
        start_run();
      }
    }
    while (width_ < n_) {
      if (k_ == hi_) { // this pair of runs is merged
        lo_ = hi_;
        if (lo_ >= n_) { // and so is this pass
          std::swap(a_, b_);
          width_ *= 2;
          lo_ = 0;
          if (width_ >= n_)
            break;
        }
        start_run();
        continue;
      }
      if (sl.spent())
        return false;
      if (i_ < mid_ && (j_ >= hi_ || !sort_less(a_[j_], a_[i_])))
        b_[k_++] = std::move(a_[i_++]);
      else
        b_[k_++] = std::move(a_[j_++]);
      sl.count(1);
    }
    return true;
  }

  value_t result() override { return make_list(std::move(a_)); }

private:
  void start_run() {
    mid_ = std::min(lo_ + width_, n_);
    hi_ = std::min(lo_ + 2 * width_, n_);
    i_ = lo_;
    j_ = mid_;
    k_ = lo_;
  }

  list_ptr list_;
  bool copied_ = false;
  std::size_t next_ = 0;
  std::size_t n_ = 0;
  std::vector<value_t> a_, b_; // a_ is sorted in runs of width_; a pass merges them into b_
  std::size_t width_ = 1, lo_ = 0, mid_ = 0, hi_ = 0, i_ = 0, j_ = 0, k_ = 0;
};

class reverse_job final : public stdlib_job {
public:
  explicit reverse_job(list_ptr list)
      : stdlib_job("collections.reverse"), list_(std::move(list)), n_(list_->size()) {
    out_.reserve(n_);
  }

  bool step(std::size_t budget) override {
    slice sl = begin(budget);
    for (; k_ < n_; ++k_) {
      if (sl.spent())
        return false;
      if (const std::size_t i = n_ - 1 - k_; i < list_->size())
        out_.push_back((*list_)[i]);
      sl.count(1);
    }
    return true;
  }

  value_t result() override { return make_list(std::move(out_)); }

private:
  list_ptr list_;
  std::size_t n_;
  std::size_t k_ = 0;
  std::vector<value_t> out_;
};

class range_job final : public stdlib_job {
public:
  // Element k is start + k*step, in unsigned arithmetic (no signed overflow at the int64 ends).
  range_job(uint64_t start, uint64_t step, std::size_t count)
      : stdlib_job("collections.range"), start_(start), step_(step), count_(count) {
    out_.reserve(count);
  }

  bool step(std::size_t budget) override {
    slice sl = begin(budget);
    for (; k_ < count_; ++k_) {
      if (sl.spent())
        return false;
      out_.emplace_back(static_cast<int64_t>(start_ + k_ * step_));
      sl.count(1);
    }
    return true;
  }

  value_t result() override { return make_list(std::move(out_)); }

private:
  uint64_t start_, step_;
  std::size_t count_;
  std::size_t k_ = 0;
  std::vector<value_t> out_;
};

// Quadratic in the list length: one node per comparison, and a comparison of two lists or dicts
// runs as an equality walk sliced with the rest.
class unique_job final : public stdlib_job {
public:
  explicit unique_job(list_ptr list) : stdlib_job("collections.unique"), list_(std::move(list)) {}

  bool step(std::size_t budget) override {
    slice sl = begin(budget);
    for (; i_ < list_->size(); ++i_, j_ = 0) {
      const value_t &item = (*list_)[i_];
      bool found = false;
      while (j_ < out_.size()) {
        if (sl.spent())
          return false;
        bool equal;
        if (cmp_) {
          const std::size_t before = cmp_->nodes();
          const bool done = cmp_->step(sl.left());
          sl.count(std::max<std::size_t>(1, cmp_->nodes() - before));
          if (!done)
            return false;
          equal = std::get<bool>(cmp_->result().v);
          cmp_.reset();
        } else if (is_compound(out_[j_]) && is_compound(item)) {
          cmp_ = make_equal_job(out_[j_], item);
          continue;
        } else {
          equal = values_equal(out_[j_], item);
          sl.count(1);
        }
        if (equal) {
          found = true;
          break;
        }
        ++j_;
      }
      if (!found)
        out_.push_back(item);
    }
    return true;
  }

  value_t result() override { return make_list(std::move(out_)); }

private:
  list_ptr list_;
  std::size_t i_ = 0, j_ = 0;
  std::unique_ptr<data_job> cmp_; // comparing out_[j_] with the element at i_
  std::vector<value_t> out_;
};

// collections.dict-keys / dict-values.
class dict_items_job final : public stdlib_job {
public:
  dict_items_job(const char *who, dict_ptr dict, bool keys)
      : stdlib_job(who), dict_(std::move(dict)), keys_(keys) {
    out_.reserve(dict_->size());
  }

  bool step(std::size_t budget) override {
    slice sl = begin(budget);
    for (; i_ < dict_->size(); ++i_) {
      if (sl.spent())
        return false;
      const auto &[k, v] = (*dict_)[i_];
      if (keys_)
        out_.emplace_back(k);
      else
        out_.push_back(v);
      sl.count(1);
    }
    return true;
  }

  value_t result() override { return make_list(std::move(out_)); }

private:
  dict_ptr dict_;
  bool keys_;
  std::size_t i_ = 0;
  std::vector<value_t> out_;
};

// A copy of d1, then d2's entries: a key already there (its first entry) is updated in place, a
// new one appended. A key index makes it linear rather than quadratic.
class dict_merge_job final : public stdlib_job {
public:
  dict_merge_job(dict_ptr d1, dict_ptr d2)
      : stdlib_job("collections.dict-merge"), d1_(std::move(d1)), d2_(std::move(d2)),
        out_(std::make_shared<ordered_map>()) {
    out_->reserve(d1_->size() + d2_->size());
  }

  bool step(std::size_t budget) override {
    slice sl = begin(budget);
    for (; i_ < d1_->size(); ++i_) {
      if (sl.spent())
        return false;
      index_.try_emplace((*d1_)[i_].first, out_->size());
      out_->push_back((*d1_)[i_]);
      sl.count(1);
    }
    for (; j_ < d2_->size(); ++j_) {
      if (sl.spent())
        return false;
      const auto &[key, value] = (*d2_)[j_];
      if (auto [at, fresh] = index_.try_emplace(key, out_->size()); fresh)
        out_->emplace_back(key, value);
      else
        (*out_)[at->second].second = value;
      sl.count(1);
    }
    return true;
  }

  value_t result() override { return value_t(std::move(out_)); }

private:
  dict_ptr d1_, d2_;
  dict_ptr out_;
  std::unordered_map<std::string, std::size_t> index_; // key -> its entry in out_
  std::size_t i_ = 0, j_ = 0;
};

} // anonymous namespace

// ---------------------------------------------------------------------------
// String module
// ---------------------------------------------------------------------------

void stdlib_registry::register_string_module() {
  register_function("string", "string.split", [](std::span<const value_t> args) -> value_t {
    expect_min(args, 1, "string.split");
    auto &s = as_string(args[0], "string.split");
    std::string_view delim = " ";
    if (args.size() > 1)
      delim = as_string(args[1], "string.split");
    return run_data_job(std::make_unique<split_job>(s, delim));
  });

  register_function("string", "string.join", [](std::span<const value_t> args) -> value_t {
    expect_exact(args, 2, "string.join");
    list_ptr lst = as_list_ptr(args[0], "string.join");
    auto &sep = as_string(args[1], "string.join");
    return run_data_job(std::make_unique<join_job>(std::move(lst), sep));
  });

  register_function("string", "string.replace", [](std::span<const value_t> args) -> value_t {
    expect_exact(args, 3, "string.replace");
    auto &s = as_string(args[0], "string.replace");
    auto &from = as_string(args[1], "string.replace");
    auto &to = as_string(args[2], "string.replace");
    if (from.empty())
      return value_t(s);
    return run_data_job(std::make_unique<replace_job>(s, from, to));
  });

  register_function("string", "string.trim", [](std::span<const value_t> args) -> value_t {
    expect_exact(args, 1, "string.trim");
    auto s = as_string(args[0], "string.trim");
    auto start = s.find_first_not_of(" \t\n\r");
    if (start == std::string::npos)
      return value_t(std::string());
    auto end = s.find_last_not_of(" \t\n\r");
    return value_t(s.substr(start, end - start + 1));
  });

  // SE-1 (A4): ASCII-only case folding, but SAFE — only bytes < 0x80 are folded (cast to unsigned
  // char first, so no signed-char UB), and every byte >= 0x80 (a UTF-8 continuation/lead byte) is
  // left untouched, so multibyte text is never corrupted. Full Unicode case folding (é->É, ß->SS)
  // is a documented follow-up that pulls in utf8proc (roadmap SE-1 note), NOT this.
  register_function("string", "string.upper", [](std::span<const value_t> args) -> value_t {
    expect_exact(args, 1, "string.upper");
    auto s = as_string(args[0], "string.upper");
    for (char &c : s) {
      const unsigned char uc = static_cast<unsigned char>(c);
      if (uc < 0x80)
        c = static_cast<char>(::toupper(uc));
    }
    return value_t(std::move(s));
  });

  register_function("string", "string.lower", [](std::span<const value_t> args) -> value_t {
    expect_exact(args, 1, "string.lower");
    auto s = as_string(args[0], "string.lower");
    for (char &c : s) {
      const unsigned char uc = static_cast<unsigned char>(c);
      if (uc < 0x80)
        c = static_cast<char>(::tolower(uc));
    }
    return value_t(std::move(s));
  });

  register_function("string", "string.starts-with", [](std::span<const value_t> args) -> value_t {
    expect_exact(args, 2, "string.starts-with");
    auto &s = as_string(args[0], "string.starts-with");
    auto &pfx = as_string(args[1], "string.starts-with");
    return value_t(s.size() >= pfx.size() && s.compare(0, pfx.size(), pfx) == 0);
  });

  register_function("string", "string.ends-with", [](std::span<const value_t> args) -> value_t {
    expect_exact(args, 2, "string.ends-with");
    auto &s = as_string(args[0], "string.ends-with");
    auto &sfx = as_string(args[1], "string.ends-with");
    return value_t(s.size() >= sfx.size() &&
                   s.compare(s.size() - sfx.size(), sfx.size(), sfx) == 0);
  });

  register_function("string", "string.contains", [](std::span<const value_t> args) -> value_t {
    expect_exact(args, 2, "string.contains");
    auto &s = as_string(args[0], "string.contains");
    auto &sub = as_string(args[1], "string.contains");
    return value_t(s.find(sub) != std::string::npos);
  });

  // SE-1 (A2): pos/len are CODEPOINT indices, mapped to byte offsets on UTF-8 boundaries, so a
  // slice never splits a multibyte codepoint. byte_offset() clamps to the byte length for indices
  // at/past the codepoint count, so the existing pos-past-end -> "" behavior is preserved.
  register_function("string", "string.substring", [](std::span<const value_t> args) -> value_t {
    expect_min(args, 2, "string.substring");
    auto &s = as_string(args[0], "string.substring");
    int64_t pos = as_int(args[1], "string.substring");
    if (pos < 0)
      pos = 0;
    const std::size_t byte_start = utf8::byte_offset(s, static_cast<std::size_t>(pos));
    if (byte_start >= s.size())
      return value_t(std::string());
    if (args.size() > 2) {
      int64_t len = as_int(args[2], "string.substring");
      if (len < 0)
        len = 0;
      const std::size_t byte_end =
          utf8::byte_offset(s, static_cast<std::size_t>(pos) + static_cast<std::size_t>(len));
      return value_t(s.substr(byte_start, byte_end - byte_start));
    }
    return value_t(s.substr(byte_start));
  });

  // SE-1 (A1): index by CODEPOINT and return the whole codepoint's bytes (1-4), not a single byte.
  register_function("string", "string.char-at", [](std::span<const value_t> args) -> value_t {
    expect_exact(args, 2, "string.char-at");
    auto &s = as_string(args[0], "string.char-at");
    int64_t i = as_int(args[1], "string.char-at");
    if (i < 0)
      throw std::runtime_error("string.char-at: index out of range");
    const std::size_t byte_start = utf8::byte_offset(s, static_cast<std::size_t>(i));
    if (byte_start >= s.size()) // i >= codepoint count
      throw std::runtime_error("string.char-at: index out of range");
    char32_t cp;
    std::size_t len;
    utf8::decode(s, byte_start, cp, len);
    return value_t(s.substr(byte_start, len));
  });

  // SE-1 (A5): count CODEPOINTS, not bytes (length of "café" is 4, not 5). Raw byte size is
  // available as string.byte-length below (and is what the SE-3 bytes type will report for binary).
  register_function("string", "string.length", [](std::span<const value_t> args) -> value_t {
    expect_exact(args, 1, "string.length");
    auto &s = as_string(args[0], "string.length");
    return value_t(static_cast<int64_t>(utf8::count(s)));
  });

  // SE-1: the raw byte count, kept separate now that string.length means codepoints.
  register_function("string", "string.byte-length", [](std::span<const value_t> args) -> value_t {
    expect_exact(args, 1, "string.byte-length");
    auto &s = as_string(args[0], "string.byte-length");
    return value_t(static_cast<int64_t>(s.size()));
  });

  // SE-3: string.encode — the text->binary crossing. state_exec strings are already UTF-8, so
  // encoding to utf-8 just re-tags the same bytes as `bytes`. Only utf-8 is supported.
  register_function("string", "string.encode", [](std::span<const value_t> args) -> value_t {
    expect_min(args, 1, "string.encode");
    auto &s = as_string(args[0], "string.encode");
    if (args.size() > 1) {
      auto &enc = as_string(args[1], "string.encode");
      if (enc != "utf-8" && enc != "utf8" && enc != "UTF-8")
        throw std::runtime_error("string.encode: unsupported encoding '" + enc + "' (only utf-8)");
    }
    return run_data_job(std::make_unique<copy_job>("string.encode", s));
  });
}

// ---------------------------------------------------------------------------
// Bytes module (SE-3) — the binary counterpart of the string module. All ops are BYTE-semantic:
// bytes are opaque octets (no codepoints, no case, no normalization). string.encode / bytes.decode
// (in the string / bytes modules) are the only sanctioned text<->binary crossings.
// ---------------------------------------------------------------------------

void stdlib_registry::register_bytes_module() {
  register_function("bytes", "bytes.length", [](std::span<const value_t> args) -> value_t {
    expect_exact(args, 1, "bytes.length");
    return value_t(static_cast<int64_t>(as_bytes(args[0], "bytes.length").size()));
  });

  register_function("bytes", "bytes.byte-at", [](std::span<const value_t> args) -> value_t {
    expect_exact(args, 2, "bytes.byte-at");
    auto &b = as_bytes(args[0], "bytes.byte-at");
    int64_t i = as_int(args[1], "bytes.byte-at");
    if (i < 0 || static_cast<size_t>(i) >= b.size())
      throw std::runtime_error("bytes.byte-at: index out of range");
    return value_t(static_cast<int64_t>(static_cast<unsigned char>(b[static_cast<size_t>(i)])));
  });

  register_function("bytes", "bytes.slice", [](std::span<const value_t> args) -> value_t {
    expect_min(args, 2, "bytes.slice");
    auto &b = as_bytes(args[0], "bytes.slice");
    int64_t pos = as_int(args[1], "bytes.slice");
    if (pos < 0)
      pos = 0;
    if (static_cast<size_t>(pos) >= b.size())
      return make_bytes(std::string());
    std::size_t len = npos;
    if (args.size() > 2)
      len = static_cast<std::size_t>(std::max<int64_t>(0, as_int(args[2], "bytes.slice")));
    return run_data_job(std::make_unique<copy_job>(
        "bytes.slice", std::string_view(b).substr(static_cast<size_t>(pos), len)));
  });

  register_function("bytes", "bytes.concat", [](std::span<const value_t> args) -> value_t {
    std::size_t total = 0;
    std::vector<std::string_view> parts;
    parts.reserve(args.size());
    for (const value_t &a : args) {
      parts.emplace_back(as_bytes(a, "bytes.concat"));
      total = sat_add(total, parts.back().size());
    }
    eval_alloc_check(total, "bytes.concat");
    return run_data_job(std::make_unique<concat_job>(std::move(parts), total));
  });

  // bytes.decode — binary->text. Only utf-8; the bytes MUST be valid UTF-8 (strict, like Python's
  // bytes.decode default) or it errors rather than yielding an invalid string.
  register_function("bytes", "bytes.decode", [](std::span<const value_t> args) -> value_t {
    expect_min(args, 1, "bytes.decode");
    auto &b = as_bytes(args[0], "bytes.decode");
    if (args.size() > 1) {
      auto &enc = as_string(args[1], "bytes.decode");
      if (enc != "utf-8" && enc != "utf8" && enc != "UTF-8")
        throw std::runtime_error("bytes.decode: unsupported encoding '" + enc + "' (only utf-8)");
    }
    // A string holding the validated UTF-8 bytes.
    return run_data_job(std::make_unique<decode_job>(b));
  });
}

// ---------------------------------------------------------------------------
// Math module
// ---------------------------------------------------------------------------

void stdlib_registry::register_math_module() {
  register_function("math", "math.sqrt", [](std::span<const value_t> args) -> value_t {
    expect_exact(args, 1, "math.sqrt");
    return value_t(std::sqrt(as_number(args[0], "math.sqrt")));
  });

  register_function("math", "math.abs", [](std::span<const value_t> args) -> value_t {
    expect_exact(args, 1, "math.abs");
    if (auto *i = std::get_if<int64_t>(&args[0].v))
      return value_t(std::abs(*i));
    return value_t(std::abs(as_number(args[0], "math.abs")));
  });

  register_function("math", "math.floor", [](std::span<const value_t> args) -> value_t {
    expect_exact(args, 1, "math.floor");
    return value_t(static_cast<int64_t>(std::floor(as_number(args[0], "math.floor"))));
  });

  register_function("math", "math.ceil", [](std::span<const value_t> args) -> value_t {
    expect_exact(args, 1, "math.ceil");
    return value_t(static_cast<int64_t>(std::ceil(as_number(args[0], "math.ceil"))));
  });

  register_function("math", "math.round", [](std::span<const value_t> args) -> value_t {
    expect_exact(args, 1, "math.round");
    return value_t(static_cast<int64_t>(std::round(as_number(args[0], "math.round"))));
  });

  register_function("math", "math.pow", [](std::span<const value_t> args) -> value_t {
    expect_exact(args, 2, "math.pow");
    return value_t(std::pow(as_number(args[0], "math.pow"), as_number(args[1], "math.pow")));
  });

  register_function("math", "math.log", [](std::span<const value_t> args) -> value_t {
    expect_exact(args, 1, "math.log");
    return value_t(std::log(as_number(args[0], "math.log")));
  });

  register_function("math", "math.sin", [](std::span<const value_t> args) -> value_t {
    expect_exact(args, 1, "math.sin");
    return value_t(std::sin(as_number(args[0], "math.sin")));
  });

  register_function("math", "math.cos", [](std::span<const value_t> args) -> value_t {
    expect_exact(args, 1, "math.cos");
    return value_t(std::cos(as_number(args[0], "math.cos")));
  });

  register_function("math", "math.tan", [](std::span<const value_t> args) -> value_t {
    expect_exact(args, 1, "math.tan");
    return value_t(std::tan(as_number(args[0], "math.tan")));
  });

  register_function("math", "math.min", [](std::span<const value_t> args) -> value_t {
    expect_min(args, 2, "math.min");
    double result = as_number(args[0], "math.min");
    for (size_t i = 1; i < args.size(); ++i)
      result = std::min(result, as_number(args[i], "math.min"));
    return value_t(result);
  });

  register_function("math", "math.max", [](std::span<const value_t> args) -> value_t {
    expect_min(args, 2, "math.max");
    double result = as_number(args[0], "math.max");
    for (size_t i = 1; i < args.size(); ++i)
      result = std::max(result, as_number(args[i], "math.max"));
    return value_t(result);
  });

  register_function("math", "math.clamp", [](std::span<const value_t> args) -> value_t {
    expect_exact(args, 3, "math.clamp");
    double val = as_number(args[0], "math.clamp");
    double lo = as_number(args[1], "math.clamp");
    double hi = as_number(args[2], "math.clamp");
    return value_t(std::clamp(val, lo, hi));
  });

  register_function("math", "math.pi", [](std::span<const value_t> args) -> value_t {
    expect_exact(args, 0, "math.pi");
    return value_t(3.14159265358979323846);
  });

  register_function("math", "math.e", [](std::span<const value_t> args) -> value_t {
    expect_exact(args, 0, "math.e");
    return value_t(2.71828182845904523536);
  });

  register_function("math", "math.random", [](std::span<const value_t> args) -> value_t {
    expect_exact(args, 0, "math.random");
    static thread_local std::mt19937 gen{std::random_device{}()};
    std::uniform_real_distribution<double> dist(0.0, 1.0);
    return value_t(dist(gen));
  });
}

// ---------------------------------------------------------------------------
// Collections module
// ---------------------------------------------------------------------------

void stdlib_registry::register_collections_module() {
  // map, filter and reduce take a native function or a DSL function (a lambda, a defun): the
  // evaluator running the program makes each call, a DSL one as its own frames.
  register_function("collections", "collections.map", [](std::span<const value_t> args) -> value_t {
    expect_exact(args, 2, "collections.map");
    list_ptr lst = as_list_ptr(args[0], "collections.map");
    const value_t &fn = as_function(args[1], "collections.map", "second");
    return run_data_job(std::make_unique<map_job>("collections.map", std::move(lst), fn, false));
  });

  register_function(
      "collections", "collections.filter", [](std::span<const value_t> args) -> value_t {
        expect_exact(args, 2, "collections.filter");
        list_ptr lst = as_list_ptr(args[0], "collections.filter");
        const value_t &fn = as_function(args[1], "collections.filter", "second");
        return run_data_job(
            std::make_unique<map_job>("collections.filter", std::move(lst), fn, true));
      });

  register_function(
      "collections", "collections.reduce", [](std::span<const value_t> args) -> value_t {
        expect_exact(args, 3, "collections.reduce");
        list_ptr lst = as_list_ptr(args[0], "collections.reduce");
        const value_t &fn = as_function(args[1], "collections.reduce", "second");
        return run_data_job(std::make_unique<reduce_job>(std::move(lst), fn, args[2]));
      });

  register_function("collections", "collections.zip", [](std::span<const value_t> args) -> value_t {
    expect_exact(args, 2, "collections.zip");
    list_ptr a = as_list_ptr(args[0], "collections.zip");
    list_ptr b = as_list_ptr(args[1], "collections.zip");
    const std::size_t n = std::min(a->size(), b->size());
    eval_alloc_check(sat_add(list_bytes(n), alloc_bytes(n, list_bytes(2))), "collections.zip");
    return run_data_job(std::make_unique<zip_job>(std::move(a), std::move(b), n));
  });

  register_function("collections", "collections.flatten",
                    [](std::span<const value_t> args) -> value_t {
                      expect_exact(args, 1, "collections.flatten");
                      return run_data_job(std::make_unique<flatten_job>(
                          as_list_ptr(args[0], "collections.flatten")));
                    });

  register_function("collections", "collections.sort",
                    [](std::span<const value_t> args) -> value_t {
                      expect_exact(args, 1, "collections.sort");
                      return run_data_job(
                          std::make_unique<sort_job>(as_list_ptr(args[0], "collections.sort")));
                    });

  register_function("collections", "collections.reverse",
                    [](std::span<const value_t> args) -> value_t {
                      expect_exact(args, 1, "collections.reverse");
                      return run_data_job(std::make_unique<reverse_job>(
                          as_list_ptr(args[0], "collections.reverse")));
                    });

  register_function(
      "collections", "collections.range", [](std::span<const value_t> args) -> value_t {
        expect_min(args, 1, "collections.range");
        int64_t start = 0, stop, step = 1;
        if (args.size() == 1) {
          stop = as_int(args[0], "collections.range");
        } else {
          start = as_int(args[0], "collections.range");
          stop = as_int(args[1], "collections.range");
          if (args.size() > 2)
            step = as_int(args[2], "collections.range");
        }
        if (step == 0)
          throw std::runtime_error("collections.range: step cannot be zero");
        // Count the elements in unsigned arithmetic (no signed overflow at the
        // int64 ends), check the size, then build: element k is start + k*step.
        const uint64_t ustart = static_cast<uint64_t>(start);
        const uint64_t ustop = static_cast<uint64_t>(stop);
        const uint64_t ustep = static_cast<uint64_t>(step);
        uint64_t count = 0;
        if (step > 0 && start < stop)
          count = (ustop - ustart - 1) / ustep + 1;
        else if (step < 0 && start > stop)
          count = (ustart - ustop - 1) / (0 - ustep) + 1;
        eval_alloc_check(count > SIZE_MAX ? SIZE_MAX : list_bytes(static_cast<size_t>(count)),
                         "collections.range");
        return run_data_job(
            std::make_unique<range_job>(ustart, ustep, static_cast<std::size_t>(count)));
      });

  register_function("collections", "collections.unique",
                    [](std::span<const value_t> args) -> value_t {
                      expect_exact(args, 1, "collections.unique");
                      return run_data_job(
                          std::make_unique<unique_job>(as_list_ptr(args[0], "collections.unique")));
                    });

  register_function("collections", "collections.dict-keys",
                    [](std::span<const value_t> args) -> value_t {
                      expect_exact(args, 1, "collections.dict-keys");
                      dict_ptr d = as_dict_ptr(args[0], "collections.dict-keys", "expected dict");
                      return run_data_job(std::make_unique<dict_items_job>("collections.dict-keys",
                                                                           std::move(d), true));
                    });

  register_function("collections", "collections.dict-values",
                    [](std::span<const value_t> args) -> value_t {
                      expect_exact(args, 1, "collections.dict-values");
                      dict_ptr d = as_dict_ptr(args[0], "collections.dict-values", "expected dict");
                      return run_data_job(std::make_unique<dict_items_job>(
                          "collections.dict-values", std::move(d), false));
                    });

  register_function(
      "collections", "collections.dict-merge", [](std::span<const value_t> args) -> value_t {
        expect_exact(args, 2, "collections.dict-merge");
        dict_ptr d1 = as_dict_ptr(args[0], "collections.dict-merge", "expected two dicts");
        dict_ptr d2 = as_dict_ptr(args[1], "collections.dict-merge", "expected two dicts");
        eval_alloc_check(
            alloc_bytes(d1->size() + d2->size(), sizeof(std::pair<std::string, value_t>)),
            "collections.dict-merge");
        return run_data_job(std::make_unique<dict_merge_job>(std::move(d1), std::move(d2)));
      });
}

// ---------------------------------------------------------------------------
// stdlib_registry public interface
// ---------------------------------------------------------------------------

stdlib_registry::stdlib_registry() {
  register_string_module();
  register_bytes_module();
  register_math_module();
  register_collections_module();
}

const stdlib_registry &stdlib_registry::standard() {
  static const stdlib_registry registry;
  return registry;
}

environment_ptr stdlib_registry::shared_scope() {
  // Built once (thread-safe static init) and never written again: frozen, so set_existing()
  // shadows a stdlib name in the caller's scope instead of writing here.
  static const environment_ptr scope = [] {
    auto env = std::make_shared<environment>();
    const stdlib_registry &reg = standard();
    for (const std::string &module : reg.list_modules())
      reg.import_module(module, env);
    env->frozen = true;
    return env;
  }();
  return scope;
}

void stdlib_registry::register_function(const std::string &module, const std::string &func_name,
                                        native_fn fn) {
  modules_[module].functions[func_name] = std::move(fn);
}

std::vector<std::string> stdlib_registry::list_functions(const std::string &module) const {
  auto it = modules_.find(module);
  if (it == modules_.end())
    return {};
  std::vector<std::string> names;
  names.reserve(it->second.functions.size());
  for (auto &[k, _] : it->second.functions)
    names.push_back(k);
  std::sort(names.begin(), names.end());
  return names;
}

std::vector<std::string> stdlib_registry::list_modules() const {
  std::vector<std::string> names;
  names.reserve(modules_.size());
  for (auto &[k, _] : modules_)
    names.push_back(k);
  std::sort(names.begin(), names.end());
  return names;
}

void stdlib_registry::import_module(const std::string &module, environment_ptr env,
                                    const std::vector<std::string> &specific_fns) const {
  auto it = modules_.find(module);
  if (it == modules_.end())
    throw std::runtime_error("stdlib: unknown module '" + module + "'");

  if (specific_fns.empty()) {
    // Import all functions
    for (auto &[name, fn] : it->second.functions) {
      builtins::register_fn(env, name, fn);
    }
  } else {
    for (auto &name : specific_fns) {
      auto fit = it->second.functions.find(name);
      if (fit == it->second.functions.end())
        throw std::runtime_error("stdlib: module '" + module + "' has no function '" + name + "'");
      builtins::register_fn(env, name, fit->second);
    }
  }
}

const native_fn *stdlib_registry::lookup_qualified(const std::string &qualified_name) const {
  // Try "module.function" format
  auto dot = qualified_name.find('.');
  if (dot != std::string::npos) {
    auto mod = qualified_name.substr(0, dot);
    auto it = modules_.find(mod);
    if (it != modules_.end()) {
      auto fit = it->second.functions.find(qualified_name);
      if (fit != it->second.functions.end())
        return &fit->second;
    }
  }
  // Scan all modules
  for (auto &[_, mod] : modules_) {
    auto fit = mod.functions.find(qualified_name);
    if (fit != mod.functions.end())
      return &fit->second;
  }
  return nullptr;
}

} // namespace cvc::state_exec
