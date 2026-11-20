/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include "settings_emit.h"

#include <algorithm>
#include <cctype>
#include <climits>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cvc/ariadne/bind.h>
#include <cvc/core/app.h>
#include <cvc/state/state.h>
#include <exception>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace cvc {
namespace ariadne {

namespace {

// The most keys one editor lists, and the deepest branch it opens: a root over a huge or deep
// subtree is cut there, with a warning, rather than stalling the frame that rebuilds it.
constexpr std::size_t kMaxKeys = 2000;
constexpr int kMaxDepth = 32;

// The state path separator, as cvc::state spells it.
const char *sep() { return cvc::state::SEPARATOR; }
std::size_t sep_len() { return std::char_traits<char>::length(cvc::state::SEPARATOR); }

std::string lower(std::string s) {
  for (char &c : s)
    c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  return s;
}

// The last part of a path ("vmax" of "vehicle.vmax").
std::string leaf_of(const std::string &id) {
  const std::size_t dot = id.rfind(sep());
  return dot == std::string::npos ? id : id.substr(dot + sep_len());
}

// A number as the shortest text of up to 17 digits that reads back as the same double.
std::string number_text(double v) {
  char b[40];
  std::snprintf(b, sizeof b, "%.15g", v);
  if (std::strtod(b, nullptr) != v)
    std::snprintf(b, sizeof b, "%.17g", v);
  return b;
}

// The whole of `s` as a number (or an integer); false for anything else, or nothing.
bool parse_number(const std::string &s, double &out) {
  if (s.empty())
    return false;
  char *end = nullptr;
  out = std::strtod(s.c_str(), &end);
  while (end && *end && std::isspace(static_cast<unsigned char>(*end)))
    ++end;
  return end && *end == '\0' && end != s.c_str();
}
bool parse_integer(const std::string &s, long long &out) {
  if (s.empty())
    return false;
  char *end = nullptr;
  out = std::strtoll(s.c_str(), &end, 10);
  while (end && *end && std::isspace(static_cast<unsigned char>(*end)))
    ++end;
  return end && *end == '\0' && end != s.c_str();
}

// Whether `text` matches the glob `pat`: * any run of characters (dots too), ? one, [abc] / [a-z]
// / [!abc] one of a set.
bool glob_match(const char *pat, const char *text) {
  const char *star = nullptr, *resume = nullptr;
  while (*text) {
    if (*pat == '*') {
      star = pat++;
      resume = text;
      continue;
    }
    bool one = false;
    const char *next = pat + 1;
    if (*pat == '?') {
      one = true;
    } else if (*pat == '[') {
      const char *p = pat + 1;
      const bool negate = *p == '!' || *p == '^';
      if (negate)
        ++p;
      bool in = false;
      const char *start = p;
      while (*p && (*p != ']' || p == start)) {
        if (p[1] == '-' && p[2] && p[2] != ']') {
          in = in || (*text >= p[0] && *text <= p[2]);
          p += 3;
        } else {
          in = in || *text == *p;
          ++p;
        }
      }
      if (*p == ']') {
        one = in != negate;
        next = p + 1;
      } else {
        one = *text == '['; // no closing ']': a literal '['
      }
    } else {
      one = *pat != '\0' && *pat == *text;
    }
    if (one) {
      pat = next;
      ++text;
    } else if (star) {
      pat = star + 1;
      text = ++resume;
    } else {
      return false;
    }
  }
  while (*pat == '*')
    ++pat;
  return *pat == '\0';
}

// A `filter:` pattern against a key's path under the root: a glob when it has * ? or [, else a
// substring.
bool pattern_match(const std::string &pat, const std::string &id) {
  if (pat.find_first_of("*?[") != std::string::npos)
    return glob_match(pat.c_str(), id.c_str());
  return id.find(pat) != std::string::npos;
}

// The filter's verdict: a key passes when it matches one of the patterns that include (all of
// them when there are none) and none that exclude ('-' first).
bool filter_pass(const std::vector<std::string> &filter, const std::string &id) {
  bool any_in = false, in = false;
  for (const std::string &f : filter) {
    if (!f.empty() && f[0] == '-') {
      if (pattern_match(f.substr(1), id))
        return false;
      continue;
    }
    any_in = true;
    in = in || pattern_match(f, id);
  }
  return !any_in || in;
}

// The search box's verdict: each of its words is in the key's path or its description (any
// case), and none of its '-' words is.
bool search_pass(const std::string &search, const std::string &id, const std::string &help) {
  const std::string hay = lower(id) + "\n" + lower(help);
  std::size_t at = 0;
  const std::string s = lower(search);
  while (at < s.size()) {
    while (at < s.size() && std::isspace(static_cast<unsigned char>(s[at])))
      ++at;
    std::size_t end = at;
    while (end < s.size() && !std::isspace(static_cast<unsigned char>(s[end])))
      ++end;
    if (end > at) {
      std::string word = s.substr(at, end - at);
      const bool exclude = word[0] == '-' && word.size() > 1;
      if (exclude)
        word = word.substr(1);
      if ((hay.find(word) != std::string::npos) == exclude)
        return false;
    }
    at = end;
  }
  return true;
}

// A unit as a printf format's tail: "" for none or a ratio ("1"), else " <unit>" with any '%'
// doubled.
std::string unit_suffix(const std::string &unit) {
  if (unit.empty() || unit == "1")
    return std::string();
  std::string out = " ";
  for (char c : unit)
    out += c == '%' ? std::string("%%") : std::string(1, c);
  return out;
}

std::string range_text(bool lo, double min, bool hi, double max) {
  if (lo && hi)
    return number_text(min) + " .. " + number_text(max);
  if (lo)
    return "at least " + number_text(min);
  if (hi)
    return "at most " + number_text(max);
  return std::string();
}

// When a change to a key applies, from its access (state_key_meta.h's).
const char *applies_text(const std::string &access, bool read_only) {
  if (read_only || access == "telemetry")
    return "read-only";
  if (access == "build")
    return "applies when it is next built";
  if (access == "knob")
    return "applies live";
  if (access == "command")
    return "an input, acted on when it changes";
  return nullptr;
}

// Whether a value is its default: as numbers when both are, as flags for a toggle, else as text.
bool same_value(SettingKind kind, const std::string &a, const std::string &b) {
  if (kind == SettingKind::Toggle)
    return to_bool_int(a, -1) == to_bool_int(b, -2);
  double x = 0.0, y = 0.0;
  if ((kind == SettingKind::Int || kind == SettingKind::Number) && parse_number(a, x) &&
      parse_number(b, y))
    return x == y;
  return a == b;
}

std::vector<std::string> split_choices(const std::string &text) {
  std::vector<std::string> out;
  std::size_t at = 0;
  while (at <= text.size()) {
    std::size_t end = text.find(',', at);
    if (end == std::string::npos)
      end = text.size();
    std::string c = text.substr(at, end - at);
    const std::size_t a = c.find_first_not_of(" \t"), z = c.find_last_not_of(" \t");
    c = a == std::string::npos ? std::string() : c.substr(a, z - a + 1);
    if (!c.empty())
      out.push_back(std::move(c));
    at = end + 1;
  }
  return out;
}

// A key's editor, from its metadata:
//   read-only (the node is, or access "telemetry"): a readout;
//   choices: a combo;
//   type bool / int / number / enum / string / list (param_table's): a checkbox, an integer, a
//     number, a combo, or a text field;
//   no type: a key with a unit, or a recommended min and max, is a number (state_key_meta's), a
//     unit of "" (stored as none) a count or a flag: a checkbox over 0..1, an integer otherwise,
//     and a count of ticks an integer; a number whose value is not one, and a key with neither, a
//     text field.
SettingKind kind_of(const std::map<std::string, std::string> &props, const std::string &value,
                    bool read_only) {
  const auto prop = [&](const char *k) {
    const auto it = props.find(k);
    return it == props.end() ? std::string() : it->second;
  };
  const std::string access = prop("access"), type = prop("type"), unit = prop("unit");
  if (read_only || access == "telemetry")
    return SettingKind::Readout;
  if (!split_choices(prop("choices")).empty())
    return SettingKind::Choice;
  if (type == "bool")
    return SettingKind::Toggle;
  if (type == "int")
    return SettingKind::Int;
  if (type == "number")
    return SettingKind::Number;
  if (!type.empty())
    return SettingKind::Text;
  const bool ranged = props.count("min") && props.count("max");
  if (!props.count("unit") && !ranged)
    return SettingKind::Text;
  double v = 0.0;
  if (!parse_number(value, v))
    return SettingKind::Text; // described as a number, but holding something else
  if (unit.empty()) {
    double lo = 0.0, hi = 0.0;
    if (parse_number(prop("min"), lo) && parse_number(prop("max"), hi) && lo == 0.0 && hi == 1.0)
      return SettingKind::Toggle;
    return SettingKind::Int;
  }
  if (unit == "tick")
    return SettingKind::Int;
  return SettingKind::Number;
}

// The keys under the roots, as SettingsNodes, for one editor.
class Walker {
public:
  Walker(SettingsContext &ctx, const Widget &w, SettingsCache &cache, const std::string &search)
      : ctx_(ctx), w_(w), cache_(cache), search_(search) {
    for (std::size_t i = 0; i < w.settings_groups.size(); ++i) {
      const SettingsGroupSpec &g = w.settings_groups[i];
      if (!groups_.count(g.key))
        groups_.emplace(g.key, std::make_pair(i, &g));
    }
  }

  // Every root, {as written, absolute}: one root's keys and branches straight under it; with
  // several, each root is a branch of its own, its keys' paths starting with the root as written.
  void walk(std::vector<SettingsNode> &out,
            const std::vector<std::pair<std::string, std::string>> &roots) {
    const bool several = roots.size() > 1;
    for (const auto &[bind, abs] : roots) {
      if (full())
        break;
      cvc::state::state_ptr node;
      try {
        node = cvc::state::instance(ctx_.app).findThroughLinksShared(abs);
      } catch (const std::exception &) {
      }
      if (!node || node->hidden())
        continue;
      const std::string id = several ? (bind[0] == '/' ? bind.substr(1) : bind) : std::string();
      if (node->childNames().empty())
        key(out, *node, abs, several ? id : leaf_of(bind), 0);
      else if (several)
        branch(out, *node, abs, id, 0);
      else
        children(out, *node, abs, std::string(), 0);
    }
  }

  bool truncated() const { return truncated_; }

private:
  bool full() const { return keys_ >= kMaxKeys; }

  // The children of `node` (`id` under the root, "" the root itself), at `depth`: its keys first,
  // in name order (a header has no end, so a key after one would read as under it), then its
  // branches, the ones `groups:` lists first, in its order, then the rest in name order.
  void children(std::vector<SettingsNode> &out, cvc::state &node, const std::string &abs,
                const std::string &id, int depth) {
    struct Child {
      std::string name;
      cvc::state::state_ptr node;
    };
    std::vector<Child> keys, branches;
    for (std::string &c : node.childNames()) {
      cvc::state::state_ptr k;
      try {
        k = node.findThroughLinksShared(c);
      } catch (const std::exception &) {
      }
      if (!k || k->hidden())
        continue;
      (k->childNames().empty() ? keys : branches).push_back({std::move(c), std::move(k)});
    }
    std::stable_sort(branches.begin(), branches.end(), [&](const Child &a, const Child &b) {
      return rank(id, a.name) < rank(id, b.name);
    });
    for (const std::vector<Child> *list : {&keys, &branches})
      for (const Child &c : *list) {
        if (full()) {
          truncated_ = true;
          return;
        }
        const std::string kid = id.empty() ? c.name : id + sep() + c.name;
        const std::string kabs = abs + sep() + c.name;
        if (list == &keys)
          key(out, *c.node, kabs, kid, depth);
        else if (depth + 1 < kMaxDepth)
          branch(out, *c.node, kabs, kid, depth);
        else
          truncated_ = true;
      }
  }

  std::size_t rank(const std::string &parent, const std::string &name) const {
    const auto it = groups_.find(parent.empty() ? name : parent + sep() + name);
    return it == groups_.end() ? groups_.size() : it->second.first;
  }

  // A branch: its header, then what is under it; left out when nothing under it is shown.
  void branch(std::vector<SettingsNode> &out, cvc::state &node, const std::string &abs,
              const std::string &id, int depth) {
    const std::size_t at = out.size();
    SettingsNode g;
    g.group = true;
    g.depth = depth;
    g.id = id;
    g.name = leaf_of(id);
    std::string help;
    try {
      help = node.comment();
    } catch (const std::exception &) {
    }
    g.open = w_.settings_open;
    if (const auto it = groups_.find(id); it != groups_.end()) {
      const SettingsGroupSpec &s = *it->second.second;
      if (!s.title.empty())
        g.name = s.title;
      if (!s.help.empty())
        help = s.help;
      g.open = g.open || s.open;
    }
    g.tooltip = id + (help.empty() ? std::string() : "\n" + help);
    out.push_back(std::move(g));
    const std::size_t keys_before = shown_;
    children(out, node, abs, id, depth + 1);
    if (shown_ == keys_before) {
      out.resize(at); // nothing under it passed: no header either
      return;
    }
    int resettable = 0;
    for (std::size_t i = at + 1; i < out.size(); ++i)
      if (!out[i].group && out[i].changed)
        ++resettable;
    out[at].resettable = resettable;
  }

  void key(std::vector<SettingsNode> &out, cvc::state &node, const std::string &abs,
           const std::string &id, int depth) {
    ++keys_;
    std::string value, comment;
    std::map<std::string, std::string> props;
    bool read_only = false;
    try {
      value = node.value();
      comment = node.comment();
      props = node.properties();
      read_only = node.readOnly();
    } catch (const std::exception &) {
      return;
    }
    cache_.first.emplace(abs, value); // its value when first seen, filtered or not
    const auto prop = [&](const char *k) {
      const auto it = props.find(k);
      return it == props.end() ? std::string() : it->second;
    };
    const std::string help = comment.empty() ? prop("doc") : comment;
    const SettingKind kind = kind_of(props, value, read_only);
    if (kind == SettingKind::Readout && !w_.settings_readouts)
      return;
    if (!filter_pass(w_.settings_filter, id))
      return;
    if (!search_.empty() && !search_pass(search_, id, help))
      return;
    ++shown_;

    SettingsNode n;
    n.depth = depth;
    n.id = id;
    n.path = abs;
    n.name = leaf_of(id);
    n.kind = kind;
    n.value = value;
    n.unit = prop("unit");
    double lo = 0.0, hi = 0.0;
    const bool has_lo = parse_number(prop("min"), lo) && std::isfinite(lo);
    const bool has_hi = parse_number(prop("max"), hi) && std::isfinite(hi);
    n.has_range = has_lo && has_hi && hi > lo;
    if (n.has_range) {
      n.min = lo;
      n.max = hi;
    }
    // Logarithmic: what `scale` says (log or linear), else a positive range spanning 100x.
    const std::string scale = lower(prop("scale"));
    n.log = n.has_range && lo > 0.0 && (scale == "log" || (scale != "linear" && hi >= 100.0 * lo));
    if (kind == SettingKind::Number)
      n.fmt = "%.4g" + unit_suffix(n.unit);
    else if (kind == SettingKind::Int)
      n.fmt = "%d" + unit_suffix(n.unit == "tick" ? std::string("ticks") : n.unit);
    if (kind == SettingKind::Choice)
      n.choices = split_choices(prop("choices"));
    if (kind != SettingKind::Readout) {
      if (props.count("default")) {
        n.def = prop("default");
        n.has_default = true;
      } else if (const auto it = cache_.first.find(abs); it != cache_.first.end()) {
        n.def = it->second;
        n.has_default = true;
      }
      n.changed = n.has_default && !same_value(kind, value, n.def);
    }

    std::string tip = id + "\n" + (help.empty() ? std::string("(no description)") : help);
    if (!n.unit.empty() && n.unit != "1")
      tip += "\nunit: " + n.unit;
    const std::string range = range_text(has_lo, lo, has_hi, hi);
    if (!range.empty())
      tip += "\nrecommended: " + range;
    if (n.has_default)
      tip += (props.count("default") ? "\ndefault: " : "\nwhen first shown: ") +
             (n.def.empty() ? std::string("(empty)") : n.def);
    if (const char *a = applies_text(prop("access"), read_only))
      tip += std::string("\n") + a;
    n.tooltip = std::move(tip);
    out.push_back(std::move(n));
  }

  SettingsContext &ctx_;
  const Widget &w_;
  SettingsCache &cache_;
  const std::string &search_;
  std::unordered_map<std::string, std::pair<std::size_t, const SettingsGroupSpec *>> groups_;
  std::size_t keys_ = 0;  // keys read, shown or not
  std::size_t shown_ = 0; // keys that passed the filter and the search
  bool truncated_ = false;
};

// An edit's text as the key's kind takes it ("" refused): a number that reads as one, an integer,
// a flag as 1 / 0, one of the choices, or any text.
bool accept(const SettingsNode &n, std::string &v) {
  switch (n.kind) {
  case SettingKind::Readout:
    return false;
  case SettingKind::Toggle: {
    const int b = to_bool_int(v, -1);
    if (b < 0)
      return false;
    v = b ? "1" : "0";
    return true;
  }
  case SettingKind::Int: {
    long long i = 0;
    double d = 0.0;
    if (parse_integer(v, i)) {
      v = std::to_string(i);
      return true;
    }
    if (parse_number(v, d) && std::isfinite(d) && std::fabs(d) < 9.0e18) {
      v = std::to_string(std::llround(d));
      return true;
    }
    return false;
  }
  case SettingKind::Number: {
    double d = 0.0;
    if (!parse_number(v, d) || !std::isfinite(d))
      return false;
    v = number_text(d);
    return true;
  }
  case SettingKind::Choice:
    return std::find(n.choices.begin(), n.choices.end(), v) != n.choices.end();
  case SettingKind::Text:
    return true;
  }
  return false;
}

} // namespace

void SettingsWatch::note(const std::string &path) {
  std::lock_guard<std::mutex> lock(mu);
  const char *s = sep();
  const std::size_t sl = sep_len();
  for (auto &[root, n] : roots) {
    const bool at_or_under = path.size() >= root.size() &&
                             path.compare(0, root.size(), root) == 0 &&
                             (path.size() == root.size() || path.compare(root.size(), sl, s) == 0);
    const bool above =
        path.empty() || (root.size() > path.size() && root.compare(0, path.size(), path) == 0 &&
                         root.compare(path.size(), sl, s) == 0);
    if (at_or_under || above)
      ++n;
  }
}

std::uint64_t SettingsWatch::count(const std::string &root) {
  std::lock_guard<std::mutex> lock(mu);
  return roots[root];
}

void emit_settings(SettingsContext &ctx, const Widget &w) {
  Backend &b = ctx.backend;
  const std::string label = w.label;
  std::vector<std::pair<std::string, std::string>> roots; // {as written, absolute}
  for (const std::string &r : w.settings_roots)
    if (!r.empty())
      roots.emplace_back(r, cvc::state::normalize_path(ctx.resolve(r)));
  std::string id = w.id;
  if (id.empty()) {
    id = "settings:" + label;
    if (label.empty())
      for (const std::string &r : w.settings_roots)
        id += r;
  }

  std::string key = id;
  for (const auto &r : roots)
    key += "|" + r.second;
  SettingsCache &cache = ctx.caches[key];
  cache.used = ctx.frame;

  const std::string search_path = w.settings_search ? ctx.resolve("tree." + id + ".search") : "";
  const std::string search =
      w.settings_search ? read_or<std::string>(ctx.app, search_path, std::string()) : "";

  // Rebuilt only when a root saw a write (a value, a comment, a property, a key coming or going)
  // or the search moved.
  std::string built_from = search;
  for (const auto &r : roots)
    built_from += "|" + std::to_string(ctx.watch.count(r.second));
  if (built_from != cache.built_from) {
    SettingsSpec spec;
    spec.search = w.settings_search;
    spec.search_text = search;
    spec.narrowed = !search.empty() || !w.settings_filter.empty();
    Walker walker(ctx, w, cache, search);
    walker.walk(spec.nodes, roots);
    if (walker.truncated())
      ctx.warn_once("ari: settings '" + label + "' lists only the first " +
                    std::to_string(kMaxKeys) + " keys (and " + std::to_string(kMaxDepth) +
                    " levels) under its root — narrow it with filter:");
    if (spec.nodes.empty())
      spec.empty = !search.empty() ? "no setting matches the search" : "no settings";
    cache.spec = std::move(spec);
    cache.built_from = built_from;
  }

  b.push_id(id.c_str());
  const SettingsEdit e = b.settings(label.c_str(), id.c_str(), cache.spec);
  b.pop_id();

  // The writes, through the same path as every bound widget's: `on:` after a committed one.
  const std::vector<SettingsNode> &nodes = cache.spec.nodes;
  bool fire = false;
  if (e.search_changed && w.settings_search)
    write<std::string>(ctx.app, search_path, e.search);
  if (e.node >= 0 && e.node < static_cast<int>(nodes.size())) {
    const SettingsNode &n = nodes[static_cast<std::size_t>(e.node)];
    std::string v = e.value;
    if (!n.group && (e.committed || (e.changed && !w.commit_on_release))) {
      if (accept(n, v)) {
        write<std::string>(ctx.app, n.path, v);
        fire = fire || e.committed;
      } else if (e.committed) {
        ctx.warn_once("ari: settings '" + label + "': '" + e.value + "' is not a value " + n.id +
                      " takes — not written");
      }
    }
  }
  if (e.reset >= 0 && e.reset < static_cast<int>(nodes.size())) {
    const std::size_t at = static_cast<std::size_t>(e.reset);
    const std::size_t end = [&] {
      if (!nodes[at].group)
        return at + 1;
      std::size_t z = at + 1;
      while (z < nodes.size() && nodes[z].depth > nodes[at].depth)
        ++z;
      return z;
    }();
    for (std::size_t i = nodes[at].group ? at + 1 : at; i < end; ++i) {
      const SettingsNode &n = nodes[i];
      if (n.group || !n.changed || !n.has_default || n.kind == SettingKind::Readout)
        continue;
      write<std::string>(ctx.app, n.path, n.def);
      fire = true;
    }
  }
  if (fire && !w.on.empty())
    ctx.enqueue(w.on);
}

void sweep_settings_caches(std::unordered_map<std::string, SettingsCache> &caches,
                           std::uint64_t frame, std::uint64_t max_age) {
  for (auto it = caches.begin(); it != caches.end();)
    it = frame - it->second.used > max_age ? caches.erase(it) : std::next(it);
}

// The default editor: a flat list of this backend's own widgets, each key labelled with its path
// under the root, so a terminal edits the same keys as a window does.
SettingsEdit Backend::settings(const char *label, const char *id, const SettingsSpec &spec) {
  SettingsEdit out;
  if (label && *label)
    text_line(label);
  if (spec.search) {
    const std::string key = std::string(id ? id : "") + ".search";
    const StringEdit e = input_text("search", key.c_str(), spec.search_text, false);
    if (e.committed && e.value != spec.search_text) {
      out.search_changed = true;
      out.search = e.value;
    }
  }
  for (std::size_t i = 0; i < spec.nodes.size(); ++i) {
    const SettingsNode &n = spec.nodes[i];
    const int at = static_cast<int>(i);
    push_id(n.id.c_str());
    if (n.group) {
      // A renamed branch by its title, else by its path.
      const std::string title = n.name == leaf_of(n.id) ? n.id : n.name;
      text_line(("[" + title + "]").c_str());
      if (!n.tooltip.empty())
        set_tooltip(n.tooltip.c_str());
      if (n.resettable > 0 && button("reset all"))
        out.reset = at;
      pop_id();
      continue;
    }
    const char *l = n.id.c_str();
    const auto edit = [&](std::string v, bool changed, bool committed) {
      out.node = at;
      out.value = std::move(v);
      out.changed = changed;
      out.committed = committed;
    };
    switch (n.kind) {
    case SettingKind::Readout: {
      const std::string unit = n.unit.empty() || n.unit == "1" ? "" : " " + n.unit;
      text_value(l, n.value + unit);
      break;
    }
    case SettingKind::Toggle: {
      const BoolEdit e = checkbox(l, to_bool_int(n.value, 0) != 0);
      if (e.committed)
        edit(e.value ? "1" : "0", true, true);
      break;
    }
    case SettingKind::Int:
      if (n.has_range) {
        double v = 0.0;
        parse_number(n.value, v);
        const auto clampi = [](double d) {
          return static_cast<int>(std::clamp(d, static_cast<double>(INT_MIN) + 1.0,
                                             static_cast<double>(INT_MAX) - 1.0));
        };
        const IntEdit e =
            slider_int(l, n.path.c_str(), clampi(std::round(v)), clampi(n.min), clampi(n.max));
        if (e.changed || e.committed)
          edit(std::to_string(e.value), e.changed, e.committed);
      } else {
        const StringEdit e = input_text(l, n.path.c_str(), n.value, false);
        if (e.committed && e.value != n.value)
          edit(e.value, true, true);
      }
      break;
    case SettingKind::Number:
      if (n.has_range) {
        double v = 0.0;
        parse_number(n.value, v);
        const DoubleEdit e =
            n.log ? slider_double_log(l, n.path.c_str(), v, n.min, n.max, n.fmt.c_str())
                  : slider_double(l, n.path.c_str(), v, n.min, n.max, n.fmt.c_str());
        if (e.changed || e.committed)
          edit(number_text(e.value), e.changed, e.committed);
      } else {
        const StringEdit e = input_text(l, n.path.c_str(), n.value, false);
        if (e.committed && e.value != n.value)
          edit(e.value, true, true);
      }
      break;
    case SettingKind::Text: {
      const StringEdit e = input_text(l, n.path.c_str(), n.value, false);
      if (e.committed && e.value != n.value)
        edit(e.value, true, true);
      break;
    }
    case SettingKind::Choice: {
      int cur = -1;
      for (std::size_t c = 0; c < n.choices.size(); ++c)
        if (n.choices[c] == n.value)
          cur = static_cast<int>(c);
      const IndexEdit e = combo(l, cur, n.choices);
      if (e.changed && e.index >= 0 && e.index < static_cast<int>(n.choices.size()) &&
          e.index != cur)
        edit(n.choices[static_cast<std::size_t>(e.index)], true, true);
      break;
    }
    }
    if (!n.tooltip.empty())
      set_tooltip(n.tooltip.c_str());
    if (n.changed) {
      if (button("reset"))
        out.reset = at;
      set_tooltip(("back to " + (n.def.empty() ? std::string("(empty)") : n.def)).c_str());
    }
    pop_id();
  }
  if (spec.nodes.empty() && !spec.empty.empty())
    text_line(spec.empty.c_str());
  return out;
}

} // namespace ariadne
} // namespace cvc
