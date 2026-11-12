/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include "input_trace_json.h"

#include <algorithm>
#include <boost/signals2/connection.hpp>
#include <cmath>
#include <cvc/core/app.h>
#include <cvc/core/config.h> // CVC_VERSION_STRING
#include <cvc/core/input_trace.h>
#include <cvc/core/world_clock.h>
#include <cvc/state/state.h>
#include <fstream>
#include <istream>
#include <mutex>
#include <ostream>
#include <set>
#include <sstream>
#include <stdexcept>

namespace cvc {

namespace tj = trace_json;

namespace {

// A watched or ignored prefix names a subtree: "nav" and "nav." both mean nav and every key
// under it, and neither means nav_plant.
std::string normalize_prefix(std::string p) {
  while (!p.empty() && p.back() == '.')
    p.pop_back();
  return p;
}

bool under(const std::string &path, const std::string &prefix) {
  if (prefix.empty())
    return true;
  if (path.size() < prefix.size() || path.compare(0, prefix.size(), prefix) != 0)
    return false;
  return path.size() == prefix.size() || path[prefix.size()] == '.';
}

bool under_any(const std::string &path, const std::vector<std::string> &prefixes) {
  for (const std::string &p : prefixes)
    if (under(path, p))
      return true;
  return false;
}

// A JSON object's text, checked and made compact. "" is the empty object.
std::string object_text(const std::string &text, const char *what) {
  if (text.empty())
    return "{}";
  tj::value v;
  try {
    v = tj::parse(text);
  } catch (const std::exception &e) {
    throw std::invalid_argument(std::string(what) + " is not JSON: " + e.what());
  }
  if (!v.is_object())
    throw std::invalid_argument(std::string(what) + " is not a JSON object");
  return v.dump();
}

tj::value parse_object(const std::string &text, const char *what) {
  tj::value v = tj::parse(text.empty() ? std::string("{}") : text);
  if (!v.is_object())
    throw std::runtime_error(std::string(what) + " is not a JSON object");
  return v;
}

void append_numbers(std::string &out, const std::vector<double> &v) {
  out += '[';
  for (std::size_t k = 0; k < v.size(); ++k) {
    if (k)
      out += ',';
    out += tj::number(v[k]);
  }
  out += ']';
}

std::vector<double> read_numbers(const tj::value *v, const char *what) {
  std::vector<double> out;
  if (!v || v->t == tj::value::type::null)
    return out;
  if (!v->is_array())
    throw std::runtime_error(std::string(what) + " is not an array");
  for (const tj::value &x : v->a)
    out.push_back(x.t == tj::value::type::null ? std::nan("") : x.number(what));
  return out;
}

const char *kind_name(input_trace_record::kind k) {
  switch (k) {
  case input_trace_record::kind::set:
    return "set";
  case input_trace_record::kind::event:
    return "event";
  default:
    return "check";
  }
}

} // namespace

// ── the header ────────────────────────────────────────────────────────────────

std::string input_trace_header::to_json() const {
  std::string o = "{\"seed\":" + tj::number(seed) + ",\"dt\":" + tj::number(dt) +
                  ",\"fixed_dt\":" + tj::number(fixed_dt) + ",\"phase\":" + tj::number(phase) +
                  ",\"start_tick\":" + tj::number(start_tick) + ",\"world_id\":";
  tj::append_string(o, world_id);
  o += ",\"world\":" + object_text(world, "the header's world");
  o += ",\"agents\":[";
  for (std::size_t k = 0; k < agents.size(); ++k) {
    const input_trace_agent &a = agents[k];
    if (k)
      o += ',';
    o += "{\"id\":" + tj::number(a.id) + ",\"start\":";
    append_numbers(o, a.start);
    o += ",\"goal\":";
    append_numbers(o, a.goal);
    o += ",\"policy\":";
    tj::append_string(o, a.policy);
    o += '}';
  }
  o += "],\"settings\":{";
  bool first = true;
  for (const auto &kv : settings) {
    if (!first)
      o += ',';
    first = false;
    tj::append_string(o, kv.first);
    o += ':';
    tj::append_string(o, kv.second);
  }
  o += "},\"watch\":[";
  for (std::size_t k = 0; k < watch.size(); ++k) {
    if (k)
      o += ',';
    tj::append_string(o, watch[k]);
  }
  o += "],\"build\":" + object_text(build, "the header's build");
  o += ",\"meta\":" + object_text(meta, "the header's meta");
  o += '}';
  return o;
}

namespace {

input_trace_header header_from(const tj::value &v) {
  if (!v.is_object())
    throw std::runtime_error("the header is not a JSON object");
  input_trace_header h;
  if (const tj::value *x = v.get("seed"))
    h.seed = x->u64("header.seed");
  if (const tj::value *x = v.get("dt"))
    h.dt = x->number("header.dt");
  if (const tj::value *x = v.get("fixed_dt"))
    h.fixed_dt = x->number("header.fixed_dt");
  if (const tj::value *x = v.get("phase"))
    h.phase = x->u64("header.phase");
  if (const tj::value *x = v.get("start_tick"))
    h.start_tick = x->u64("header.start_tick");
  if (const tj::value *x = v.get("world_id"))
    h.world_id = x->str("header.world_id");
  if (const tj::value *x = v.get("world")) {
    if (!x->is_object())
      throw std::runtime_error("header.world is not a JSON object");
    h.world = x->dump();
  }
  if (const tj::value *x = v.get("agents")) {
    if (!x->is_array())
      throw std::runtime_error("header.agents is not an array");
    for (const tj::value &a : x->a) {
      if (!a.is_object())
        throw std::runtime_error("an agent is not a JSON object");
      input_trace_agent g;
      if (const tj::value *y = a.get("id"))
        g.id = y->u64("agent.id");
      g.start = read_numbers(a.get("start"), "agent.start");
      g.goal = read_numbers(a.get("goal"), "agent.goal");
      if (const tj::value *y = a.get("policy"))
        g.policy = y->str("agent.policy");
      h.agents.push_back(std::move(g));
    }
  }
  if (const tj::value *x = v.get("settings")) {
    if (!x->is_object())
      throw std::runtime_error("header.settings is not a JSON object");
    for (const auto &kv : x->o)
      h.settings[kv.first] = kv.second.str("a setting");
  }
  if (const tj::value *x = v.get("watch")) {
    if (!x->is_array())
      throw std::runtime_error("header.watch is not an array");
    for (const tj::value &p : x->a)
      h.watch.push_back(p.str("a watched prefix"));
  }
  if (const tj::value *x = v.get("build")) {
    if (!x->is_object())
      throw std::runtime_error("header.build is not a JSON object");
    h.build = x->dump();
  }
  if (const tj::value *x = v.get("meta")) {
    if (!x->is_object())
      throw std::runtime_error("header.meta is not a JSON object");
    h.meta = x->dump();
  }
  return h;
}

} // namespace

input_trace_header input_trace_header::from_json(const std::string &json) {
  return header_from(tj::parse(json));
}

// ── records ───────────────────────────────────────────────────────────────────

input_trace_record input_trace_record::set(std::uint64_t tick, std::string path,
                                           std::string value) {
  input_trace_record r;
  r.type = kind::set;
  r.tick = tick;
  r.path = std::move(path);
  r.value = std::move(value);
  return r;
}

input_trace_record input_trace_record::event(std::uint64_t tick, std::string name,
                                             std::string args) {
  input_trace_record r;
  r.type = kind::event;
  r.tick = tick;
  r.name = std::move(name);
  r.args = object_text(args, "an event's args");
  return r;
}

input_trace_record input_trace_record::check(std::uint64_t tick, std::string hash) {
  input_trace_record r;
  r.type = kind::check;
  r.tick = tick;
  r.hash = std::move(hash);
  return r;
}

std::string input_trace_record::to_json() const {
  std::string o = "{\"tick\":" + tj::number(tick);
  switch (type) {
  case kind::set:
    o += ",\"set\":{\"path\":";
    tj::append_string(o, path);
    o += ",\"value\":";
    tj::append_string(o, value);
    o += "}}";
    break;
  case kind::event:
    o += ",\"event\":{\"name\":";
    tj::append_string(o, name);
    o += ",\"args\":" + object_text(args, "an event's args") + "}}";
    break;
  case kind::check:
    o += ",\"check\":";
    tj::append_string(o, hash);
    o += '}';
    break;
  }
  return o;
}

namespace {

std::string end_line(const input_trace_end &e, const input_trace_header &h) {
  std::string o = "{\"end\":{\"tick\":" + tj::number(e.tick) +
                  ",\"ticks\":" + tj::number(e.ticks(h)) + ",\"hash\":";
  tj::append_string(o, e.hash);
  return o + "}}";
}

std::string header_line(const input_trace_header &h) {
  std::string o = "{\"schema\":";
  tj::append_string(o, kInputTraceSchema);
  return o + ",\"header\":" + h.to_json() + "}";
}

} // namespace

// ── a whole trace ─────────────────────────────────────────────────────────────

input_trace input_trace::parse(std::istream &in) {
  input_trace t;
  bool have_header = false;
  std::string line;
  std::size_t n = 0;
  std::uint64_t last = 0;
  while (std::getline(in, line)) {
    ++n;
    if (!line.empty() && line.back() == '\r')
      line.pop_back();
    if (line.find_first_not_of(" \t") == std::string::npos)
      continue;
    const std::string where = "cvc.input_trace line " + std::to_string(n) + ": ";
    try {
      const tj::value v = tj::parse(line);
      if (!v.is_object())
        throw std::runtime_error("not a JSON object");
      if (!have_header) {
        const tj::value *schema = v.get("schema");
        if (!schema || !schema->is_string())
          throw std::runtime_error("the first line has no schema");
        if (schema->s != kInputTraceSchema)
          throw std::runtime_error("the schema is " + schema->s + ", not " + kInputTraceSchema);
        const tj::value *h = v.get("header");
        if (!h)
          throw std::runtime_error("the first line has no header");
        t.header = header_from(*h);
        last = t.header.start_tick;
        have_header = true;
        continue;
      }
      if (t.end.present)
        throw std::runtime_error("a line after the end");
      if (const tj::value *e = v.get("end")) {
        if (!e->is_object())
          throw std::runtime_error("the end is not a JSON object");
        t.end.present = true;
        if (const tj::value *x = e->get("tick"))
          t.end.tick = x->u64("end.tick");
        if (const tj::value *x = e->get("hash"))
          t.end.hash = x->str("end.hash");
        if (t.end.tick < last)
          throw std::runtime_error("the end's tick is before the last record's");
        continue;
      }
      const tj::value *tick = v.get("tick");
      if (!tick)
        throw std::runtime_error("a record has no tick");
      input_trace_record r;
      r.tick = tick->u64("tick");
      if (const tj::value *s = v.get("set")) {
        r.type = input_trace_record::kind::set;
        const tj::value *p = s->get("path");
        if (!p)
          throw std::runtime_error("a set has no path");
        r.path = p->str("set.path");
        if (const tj::value *x = s->get("value"))
          r.value = x->str("set.value");
      } else if (const tj::value *e = v.get("event")) {
        r.type = input_trace_record::kind::event;
        const tj::value *nm = e->get("name");
        if (!nm)
          throw std::runtime_error("an event has no name");
        r.name = nm->str("event.name");
        if (const tj::value *a = e->get("args")) {
          if (!a->is_object())
            throw std::runtime_error("an event's args are not a JSON object");
          r.args = a->dump();
        }
      } else if (const tj::value *c = v.get("check")) {
        r.type = input_trace_record::kind::check;
        r.hash = c->str("check");
      } else {
        throw std::runtime_error("a record that is none of set, event, check or end");
      }
      if (r.tick < last)
        throw std::runtime_error("tick " + std::to_string(r.tick) + " is before tick " +
                                 std::to_string(last));
      last = r.tick;
      t.records.push_back(std::move(r));
    } catch (const std::exception &e) {
      throw std::runtime_error(where + e.what());
    }
  }
  if (!have_header)
    throw std::runtime_error("cvc.input_trace: no header (an empty file?)");
  return t;
}

input_trace input_trace::parse(const std::string &text) {
  std::istringstream in(text);
  return parse(in);
}

input_trace input_trace::read(const std::string &path) {
  std::ifstream f(path, std::ios::binary);
  if (!f)
    throw std::runtime_error("cvc.input_trace: cannot open " + path);
  return parse(f);
}

void input_trace::write(std::ostream &out) const {
  out << header_line(header) << '\n';
  for (const input_trace_record &r : records)
    out << r.to_json() << '\n';
  if (end.present)
    out << end_line(end, header) << '\n';
}

std::string input_trace::str() const {
  std::ostringstream o;
  write(o);
  return o.str();
}

void input_trace::save(const std::string &path) const {
  std::ofstream f(path, std::ios::binary | std::ios::trunc);
  if (!f)
    throw std::runtime_error("cvc.input_trace: cannot write " + path);
  write(f);
  if (!f.flush())
    throw std::runtime_error("cvc.input_trace: writing " + path + " failed");
}

std::uint64_t input_trace::last_tick() const {
  if (end.present)
    return end.tick;
  std::uint64_t t = header.start_tick;
  if (!records.empty())
    t = std::max(t, records.back().tick);
  return t;
}

std::string input_trace_build() {
  std::string compiler;
#if defined(__clang__)
  compiler = std::string("clang ") + __clang_version__;
#elif defined(__GNUC__)
  compiler = std::string("gcc ") + __VERSION__;
#elif defined(_MSC_VER)
  compiler = "msvc " + std::to_string(_MSC_VER);
#else
  compiler = "unknown";
#endif
  const char *arch =
#if defined(__x86_64__) || defined(_M_X64)
      "x86_64";
#elif defined(__aarch64__) || defined(_M_ARM64)
      "arm64";
#elif defined(__wasm32__)
      "wasm32";
#elif defined(__i386__) || defined(_M_IX86)
      "x86";
#else
      "unknown";
#endif
  const char *os =
#if defined(__EMSCRIPTEN__)
      "emscripten";
#elif defined(_WIN32)
      "windows";
#elif defined(__APPLE__)
      "macos";
#elif defined(__linux__)
      "linux";
#elif defined(__HAIKU__)
      "haiku";
#elif defined(__OpenBSD__)
          "openbsd";
#elif defined(__FreeBSD__)
          "freebsd";
#else
          "unknown";
#endif
  std::string o = "{\"libcvc\":";
  tj::append_string(o, CVC_VERSION_STRING);
  o += ",\"compiler\":";
  tj::append_string(o, compiler);
  o += ",\"arch\":";
  tj::append_string(o, arch);
  o += ",\"os\":";
  tj::append_string(o, os);
  return o + "}";
}

std::string describe(const input_trace &t) {
  const input_trace_header &h = t.header;
  std::ostringstream o;
  o << kInputTraceSchema << "\n";
  std::string kind, complete;
  try {
    const tj::value w = parse_object(h.world, "the header's world");
    if (const tj::value *k = w.get("kind"); k && k->is_string())
      kind = k->s;
    if (const tj::value *c = w.get("complete"); c && c->t == tj::value::type::boolean)
      complete = c->b ? "complete" : "incomplete: the host rebuilds it";
  } catch (const std::exception &) {
  }
  o << "world      " << (h.world_id.empty() ? std::string("(unnamed)") : h.world_id);
  if (!kind.empty())
    o << " (" << kind << (complete.empty() ? "" : ", " + complete) << ")";
  o << "\nseed       " << h.seed << "\n";
  if (h.dt > 0.0 && h.fixed_dt > 0.0) {
    const double k = std::round(h.dt / h.fixed_dt);
    o << "step       " << tj::number(h.dt) << " s, " << tj::number(k) << " quanta of "
      << tj::number(h.fixed_dt) << " s, phase " << h.phase << "\n";
  } else {
    o << "quantum    " << tj::number(h.fixed_dt) << " s\n";
  }
  const std::uint64_t last = t.last_tick();
  o << "ticks      " << h.start_tick << " .. " << last << " (" << (last - h.start_tick)
    << " quanta, " << tj::number(static_cast<double>(last - h.start_tick) * h.fixed_dt) << " s)\n";
  std::map<std::string, std::size_t> policies;
  for (const input_trace_agent &a : h.agents)
    ++policies[a.policy.empty() ? std::string("(unstated)") : a.policy];
  o << "agents     " << h.agents.size();
  for (const auto &p : policies)
    o << (p.first == policies.begin()->first ? ": " : ", ") << p.second << " on " << p.first;
  o << "\n";
  o << "settings   " << h.settings.size() << " keys at the start, watching";
  for (const std::string &w : h.watch)
    o << " " << w;
  o << "\n";
  std::size_t sets = 0, events = 0, checks = 0;
  std::map<std::string, std::size_t> by_key, by_event;
  for (const input_trace_record &r : t.records) {
    if (r.type == input_trace_record::kind::set) {
      ++sets;
      ++by_key[r.path];
    } else if (r.type == input_trace_record::kind::event) {
      ++events;
      ++by_event[r.name];
    } else {
      ++checks;
    }
  }
  o << "records    " << sets << " sets, " << events << " events, " << checks << " checks\n";
  for (const auto &kv : by_key)
    o << "  set      " << kv.first << " x" << kv.second << "\n";
  for (const auto &kv : by_event)
    o << "  event    " << kv.first << " x" << kv.second << "\n";
  if (t.end.present)
    o << "end        tick " << t.end.tick << ", " << t.end.hash << "\n";
  else
    o << "end        none (the recording did not finish)\n";
  try {
    const tj::value b = parse_object(h.build, "the header's build");
    std::vector<std::string> parts;
    for (const char *key : {"libcvc", "compiler", "arch", "os"})
      if (const tj::value *x = b.get(key); x && x->is_string())
        parts.push_back(std::string(key) == "libcvc" ? "libcvc " + x->s : x->s);
    for (std::size_t k = 0; k < parts.size(); ++k)
      o << (k ? ", " : "build      ") << parts[k] << (k + 1 == parts.size() ? "\n" : "");
  } catch (const std::exception &) {
  }
  return o.str();
}

// ── the recorder ──────────────────────────────────────────────────────────────

struct input_trace_recorder::impl {
  impl(cvc::app &a, cvc::world_clock &c, options o) : app(a), clock(c), opts(std::move(o)) {
    for (const std::string &p : opts.watch)
      watch.push_back(normalize_prefix(p));
    for (const std::string &p : opts.ignore)
      ignore.push_back(normalize_prefix(p));
  }

  cvc::app &app;
  cvc::world_clock &clock;
  options opts;
  std::vector<std::string> watch, ignore;

  mutable std::mutex mu;
  bool recording = false, done = false;
  std::string broken;
  input_trace trace;
  std::unique_ptr<std::ofstream> file;
  std::map<std::string, std::string> last; // the value last recorded for each key
  std::set<std::string> pending;           // on_sample: keys changed since the last sample
  std::uint64_t last_tick = 0;
  boost::signals2::scoped_connection conn;

  bool wanted(const std::string &path) const {
    return under_any(path, watch) && !under_any(path, ignore);
  }

  // The key's value, or false when it is gone or an output.
  bool read(const std::string &path, std::string &value) const {
    try {
      const state::state_ptr n = app.root().findDescendantShared(path);
      if (!n)
        return false;
      if (opts.skip_telemetry && n->property("access") == "telemetry")
        return false;
      value = n->value();
      return true;
    } catch (const std::exception &) {
      return false;
    }
  }

  // Stamp a record with the clock's tick; a clock that went back breaks the recording.
  bool stamp_locked(std::uint64_t &tick) {
    tick = clock.tick();
    if (tick < last_tick) {
      broken = "the clock went back from tick " + std::to_string(last_tick) + " to " +
               std::to_string(tick) + " while recording";
      recording = false;
      conn.disconnect();
      return false;
    }
    last_tick = tick;
    return true;
  }

  void write_locked(const std::string &line) {
    if (file) {
      *file << line << '\n';
      file->flush();
    }
  }

  void add_locked(input_trace_record r) {
    write_locked(r.to_json());
    trace.records.push_back(std::move(r));
  }

  void set_locked(const std::string &path, const std::string &value) {
    const auto it = last.find(path);
    const std::string &was = it == last.end() ? std::string() : it->second;
    if (value == was)
      return;
    std::uint64_t tick = 0;
    if (!stamp_locked(tick))
      return;
    last[path] = value;
    add_locked(input_trace_record::set(tick, path, value));
  }

  void changed(const std::string &path) {
    if (!wanted(path))
      return;
    if (opts.stamp == stamping::on_sample) {
      std::lock_guard<std::mutex> lk(mu);
      if (recording)
        pending.insert(path);
      return;
    }
    std::string value;
    if (!read(path, value))
      return;
    std::lock_guard<std::mutex> lk(mu);
    if (recording)
      set_locked(path, value);
  }

  // The watched keys' values now, for the header.
  std::map<std::string, std::string> snapshot() const {
    std::map<std::string, std::string> out;
    for (const std::string &p : watch) {
      state::state_ptr base;
      try {
        base = p.empty() ? state::state_ptr() : app.root().findDescendantShared(p);
      } catch (const std::exception &) {
      }
      std::vector<std::string> paths;
      if (p.empty()) {
        paths = app.root().children();
      } else if (base) {
        paths.push_back(p);
        for (std::string &c : base->children())
          paths.push_back(std::move(c));
      }
      for (const std::string &path : paths) {
        if (!wanted(path))
          continue;
        std::string value;
        if (read(path, value) && !value.empty())
          out[path] = value;
      }
    }
    return out;
  }
};

input_trace_recorder::input_trace_recorder(cvc::app &app, cvc::world_clock &clock)
    : input_trace_recorder(app, clock, options()) {}

input_trace_recorder::input_trace_recorder(cvc::app &app, cvc::world_clock &clock, options o)
    : _impl(std::make_shared<impl>(app, clock, std::move(o))) {}

input_trace_recorder::~input_trace_recorder() {
  std::lock_guard<std::mutex> lk(_impl->mu);
  _impl->recording = false;
  _impl->conn.disconnect();
}

void input_trace_recorder::start(input_trace_header header, const std::string &path) {
  impl &m = *_impl;
  {
    std::lock_guard<std::mutex> lk(m.mu);
    if (m.recording || m.done)
      throw std::logic_error("input_trace_recorder::start: already recorded; one per recorder");
  }
  header.world = object_text(header.world, "the header's world");
  header.meta = object_text(header.meta, "the header's meta");
  header.build = header.build.empty() || header.build == "{}"
                     ? input_trace_build()
                     : object_text(header.build, "the header's build");
  header.fixed_dt = m.clock.fixed_dt();
  header.watch = m.watch;
  std::unique_ptr<std::ofstream> file;
  if (!path.empty()) {
    file = std::make_unique<std::ofstream>(path, std::ios::binary | std::ios::trunc);
    if (!*file)
      throw std::runtime_error("input_trace_recorder::start: cannot write " + path);
  }
  // Watch first, then snapshot: a write landing between the two is in the snapshot or recorded
  // after it (or both, which records nothing more, its value being the snapshot's).
  std::weak_ptr<impl> weak = _impl;
  boost::signals2::connection c =
      m.app.root().childChanged.connect([weak](const std::string &path) {
        if (const std::shared_ptr<impl> p = weak.lock())
          p->changed(path);
      });
  std::lock_guard<std::mutex> lk(m.mu);
  m.conn = c;
  header.start_tick = m.clock.tick();
  header.settings = m.snapshot();
  m.last = header.settings;
  m.last_tick = header.start_tick;
  m.trace = input_trace();
  m.trace.header = std::move(header);
  m.file = std::move(file);
  m.write_locked(header_line(m.trace.header));
  m.recording = true;
}

void input_trace_recorder::sample() {
  impl &m = *_impl;
  std::set<std::string> keys;
  {
    std::lock_guard<std::mutex> lk(m.mu);
    if (!m.recording)
      return;
    keys.swap(m.pending);
  }
  std::vector<std::pair<std::string, std::string>> values;
  for (const std::string &k : keys) {
    std::string v;
    if (m.read(k, v))
      values.emplace_back(k, std::move(v));
  }
  std::lock_guard<std::mutex> lk(m.mu);
  if (!m.recording)
    return;
  for (const auto &kv : values)
    m.set_locked(kv.first, kv.second);
}

void input_trace_recorder::event(const std::string &name, const std::string &args) {
  if (name.empty())
    throw std::invalid_argument("input_trace_recorder::event: an event needs a name");
  input_trace_record r = input_trace_record::event(0, name, args); // checks args
  impl &m = *_impl;
  std::lock_guard<std::mutex> lk(m.mu);
  if (!m.recording)
    return;
  if (!m.stamp_locked(r.tick))
    return;
  m.add_locked(std::move(r));
}

void input_trace_recorder::check(const std::string &hash) {
  impl &m = *_impl;
  std::lock_guard<std::mutex> lk(m.mu);
  if (!m.recording)
    return;
  std::uint64_t tick = 0;
  if (!m.stamp_locked(tick))
    return;
  // A frame that ran no quantum checks the tick it already checked.
  const std::vector<input_trace_record> &rs = m.trace.records;
  if (!rs.empty() && rs.back().type == input_trace_record::kind::check && rs.back().tick == tick &&
      rs.back().hash == hash)
    return;
  m.add_locked(input_trace_record::check(tick, hash));
}

void input_trace_recorder::finish(const std::string &hash) {
  // on_sample: what changed since the last sample is the last input, at this tick.
  if (_impl->opts.stamp == stamping::on_sample)
    sample();
  impl &m = *_impl;
  std::lock_guard<std::mutex> lk(m.mu);
  m.conn.disconnect();
  const bool was = m.recording;
  m.recording = false;
  if (!m.broken.empty()) {
    m.file.reset();
    m.done = true;
    throw std::runtime_error("input_trace_recorder::finish: " + m.broken);
  }
  if (!was)
    throw std::logic_error("input_trace_recorder::finish: not recording");
  std::uint64_t tick = 0;
  m.stamp_locked(tick);
  m.trace.end.present = true;
  m.trace.end.tick = tick;
  m.trace.end.hash = hash;
  m.write_locked(end_line(m.trace.end, m.trace.header));
  if (m.file) {
    m.file->close();
    m.file.reset();
  }
  m.done = true;
}

bool input_trace_recorder::recording() const {
  std::lock_guard<std::mutex> lk(_impl->mu);
  return _impl->recording;
}

bool input_trace_recorder::finished() const {
  std::lock_guard<std::mutex> lk(_impl->mu);
  return _impl->done;
}

input_trace input_trace_recorder::trace() const {
  std::lock_guard<std::mutex> lk(_impl->mu);
  return _impl->trace;
}

std::size_t input_trace_recorder::size() const {
  std::lock_guard<std::mutex> lk(_impl->mu);
  return _impl->trace.records.size();
}

// ── the player ────────────────────────────────────────────────────────────────

input_trace_player::input_trace_player(cvc::app &app, cvc::world_clock &clock, input_trace trace)
    : _app(app), _clock(clock), _trace(std::move(trace)) {
  std::uint64_t last = _trace.header.start_tick;
  for (std::size_t k = 0; k < _trace.records.size(); ++k) {
    const input_trace_record &r = _trace.records[k];
    if (r.tick < last)
      throw std::invalid_argument("input_trace_player: the records are not in tick order");
    last = r.tick;
    if (r.type == input_trace_record::kind::check)
      _checks.push_back(k);
  }
  if (_trace.end.present && _trace.end.tick < last)
    throw std::invalid_argument("input_trace_player: the end is before the last record");
}

void input_trace_player::on_event(std::function<void(const input_trace_record &)> fn) {
  _on_event = std::move(fn);
}

void input_trace_player::set_state_hash(std::function<std::string()> fn) {
  _state_hash = std::move(fn);
}

void input_trace_player::rewind() {
  const input_trace_header &h = _trace.header;
  if (h.fixed_dt > 0.0 && _clock.fixed_dt() != h.fixed_dt) {
    world_clock::config c = _clock.current_config();
    c.fixed_dt = h.fixed_dt;
    _clock.configure(c);
  }
  _clock.set_mode(world_clock::mode::replay);
  _clock.seek_tick(h.start_tick);
  state &root = _app.root();
  for (const auto &kv : h.settings)
    root(kv.first).value(kv.second);
  _next = 0;
  _compared = _matched = 0;
  _diverged = false;
  _end_compared = false;
  _first_divergence = 0;
  _verified_tick = ~std::uint64_t(0);
}

std::vector<input_trace_record> input_trace_player::apply_due() {
  std::vector<input_trace_record> events;
  const std::uint64_t now = _clock.tick();
  state &root = _app.root();
  while (_next < _trace.records.size() && _trace.records[_next].tick <= now) {
    const input_trace_record &r = _trace.records[_next++];
    if (r.type == input_trace_record::kind::set) {
      root(r.path).value(r.value);
    } else if (r.type == input_trace_record::kind::event) {
      if (_on_event)
        _on_event(r);
      events.push_back(r);
    }
  }
  return events;
}

std::uint64_t input_trace_player::next_quantum() {
  const std::uint64_t t = _clock.tick();
  if (t >= last_tick())
    throw std::logic_error("input_trace_player::next_quantum: at the trace's last tick");
  _clock.seek_tick(t + 1);
  return t + 1;
}

std::string input_trace_player::expected_hash() const {
  const std::uint64_t now = _clock.tick();
  if (_trace.end.present && now == _trace.end.tick)
    return _trace.end.hash;
  const auto it =
      std::lower_bound(_checks.begin(), _checks.end(), now, [this](std::size_t k, std::uint64_t t) {
        return _trace.records[k].tick < t;
      });
  if (it != _checks.end() && _trace.records[*it].tick == now)
    return _trace.records[*it].hash;
  return std::string();
}

bool input_trace_player::verify(const std::string &hash) {
  const std::string want = expected_hash();
  if (want.empty())
    return true;
  const std::uint64_t now = _clock.tick();
  const bool same = hash == want;
  if (now == _verified_tick)
    return same; // counted once a tick
  _verified_tick = now;
  ++_compared;
  if (same)
    ++_matched;
  else if (!_diverged) {
    _diverged = true;
    _first_divergence = now;
  }
  if (_trace.end.present && now == _trace.end.tick)
    _end_compared = true;
  return same;
}

std::uint64_t input_trace_player::advance(std::uint64_t n,
                                          const std::function<void(std::uint64_t)> &quantum) {
  if (_state_hash && !expected_hash().empty())
    verify(_state_hash()); // a hash at the tick the run starts from
  std::uint64_t ran = 0;
  while (ran < n && !done()) {
    apply_due();
    const std::uint64_t t = next_quantum();
    if (quantum)
      quantum(t);
    ++ran;
    if (_state_hash && !expected_hash().empty())
      verify(_state_hash());
  }
  return ran;
}

std::uint64_t input_trace_player::run(const std::function<void(std::uint64_t)> &quantum) {
  return advance(~std::uint64_t(0), quantum);
}

bool input_trace_player::done() const { return _clock.tick() >= last_tick(); }

std::uint64_t input_trace_player::last_tick() const { return _trace.last_tick(); }

bool input_trace_player::matched() const {
  if (_diverged)
    return false;
  return _trace.end.present ? _end_compared : _compared > 0;
}

} // namespace cvc
