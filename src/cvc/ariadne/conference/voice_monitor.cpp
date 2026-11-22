/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cvc/ariadne/conference/voice_monitor.h>
#include <cvc/core/app.h>
#include <cvc/state/state.h>
#include <cvc/state/state_key_meta.h>
#include <set>
#include <utility>

namespace cvc {
namespace ariadne {
namespace conference {

namespace {

// The tuning keys under audio.jb, and what a settings: editor offers for each.
const cvc::state_key_meta kTuning[] = {
    {"adaptive.min", "knob", "s", 0.02, 0.4, "The least a voice's buffer holds."},
    {"adaptive.start", "knob", "s", 0.02, 0.4,
     "What a voice's buffer holds when it joins, before it adapts."},
    {"adaptive.max", "knob", "s", 0.1, 1.0, "The most a voice's buffer grows to after late audio."},
    {"adaptive.relax", "knob", "s", 1.0, 60.0,
     "How long a voice plays without running dry before its buffer shrinks a step. Shorter: less "
     "delay after a one-off stall; longer: fewer gaps when stalls come back."},
    {"fixed.delay", "knob", "s", 0.02, 0.6, "What every voice's buffer holds."},
};

std::string ms_list(const std::deque<int> &v) {
  std::string s;
  s.reserve(v.size() * 4);
  for (int x : v) {
    if (!s.empty())
      s += ',';
    s += std::to_string(x);
  }
  return s;
}

bool read_seconds(const std::string &text, double &out) {
  if (text.empty())
    return false;
  char *end = nullptr;
  const double v = std::strtod(text.c_str(), &end);
  if (end == text.c_str() || !std::isfinite(v) || v < 0.0)
    return false;
  out = v;
  return true;
}

bool same_tuning(const cvc::voice_mixer::params &a, const cvc::voice_mixer::params &b) {
  return a.min_delay_s == b.min_delay_s && a.start_delay_s == b.start_delay_s &&
         a.max_delay_s == b.max_delay_s && a.relax_after_s == b.relax_after_s &&
         a.step_s == b.step_s;
}

int ms(double s) { return static_cast<int>(std::lround(s * 1000.0)); }

} // namespace

voice_monitor::voice_monitor(cvc::app &app, std::string prefix, double period_s,
                             std::size_t history)
    : _app(app), _prefix(std::move(prefix)), _period_s(period_s > 0.0 ? period_s : 0.2),
      _history(history > 0 ? history : 1) {}

std::string voice_monitor::key(const std::string &rel) const {
  return _prefix.empty() ? rel : _prefix + "." + rel;
}

void voice_monitor::describe() {
  auto &st = cvc::state::instance(_app);
  const cvc::voice_mixer::params d;
  const std::pair<const char *, double> defaults[] = {{"adaptive.min", d.min_delay_s},
                                                      {"adaptive.start", d.start_delay_s},
                                                      {"adaptive.max", d.max_delay_s},
                                                      {"adaptive.relax", d.relax_after_s},
                                                      {"fixed.delay", 0.12}};
  for (const auto &kv : defaults) {
    char b[32];
    std::snprintf(b, sizeof b, "%g", kv.second);
    auto &node = st(key(std::string("audio.jb.") + kv.first));
    if (node.value().empty())
      node.value(b);
    node.property("default", b);
  }
  cvc::describe_state_keys(_app, key("audio.jb"), kTuning);
  auto &mode = st(key("audio.jb.mode"));
  if (mode.value() != "adaptive" && mode.value() != "fixed")
    mode.value("adaptive");
  mode.comment("adaptive: a voice's buffer grows after late audio and relaxes back (the Adaptive "
               "settings); fixed: every voice's buffer holds the same (Fixed).");
  mode.property("access", "knob");
  mode.property("choices", "adaptive, fixed");
  mode.property("default", "adaptive");
  st(key("audio.reset")).value("0");
  // The readouts, until the first step (the output opens with the first voice).
  st(key("audio.device_ms")).value("0");
  st(key("audio.ahead_ms")).value("0");
  st(key("audio.window_s"))
      .value(std::to_string(static_cast<int>(std::lround(_period_s * _history))));
}

cvc::voice_mixer::params voice_monitor::read_tuning() {
  auto &st = cvc::state::instance(_app);
  cvc::voice_mixer::params p;
  const auto get = [&](const char *k, double &v) {
    read_seconds(st(key(std::string("audio.jb.") + k)).value(), v);
  };
  if (st(key("audio.jb.mode")).value() == "fixed") {
    double fixed = 0.12;
    get("fixed.delay", fixed);
    p.min_delay_s = p.start_delay_s = p.max_delay_s = fixed;
    return p;
  }
  get("adaptive.min", p.min_delay_s);
  get("adaptive.start", p.start_delay_s);
  get("adaptive.max", p.max_delay_s);
  get("adaptive.relax", p.relax_after_s);
  return p;
}

void voice_monitor::step(cvc::voice_mixer &mx, double now, double device_s, double ahead_s) {
  auto &st = cvc::state::instance(_app);
  const cvc::voice_mixer::params want = read_tuning();
  if (!_have_applied || !same_tuning(want, _applied)) {
    mx.set_params(want);
    _applied = want;
    _have_applied = true;
  }
  auto &reset = st(key("audio.reset"));
  if (reset.value() == "1") {
    mx.reset_targets();
    reset.value("0");
  }
  if (_next >= 0.0 && now < _next)
    return;
  _next = (_next < 0.0 || now - _next > _period_s) ? now + _period_s : _next + _period_s;

  std::set<std::string> live;
  const double ms_per_frame = 1000.0 / static_cast<double>(mx.rate());
  for (const std::string &pid : mx.peers()) {
    cvc::voice_mixer::peer_stats s;
    if (!mx.stats(pid, s))
      continue;
    live.insert(pid);
    history &h = _peers[pid];
    h.buffered.push_back(ms(s.buffered_s));
    h.target.push_back(ms(s.target_s));
    while (h.buffered.size() > _history) {
      h.buffered.pop_front();
      h.target.pop_front();
    }
    const std::string base = key("media." + pid + ".jb.");
    st(base + "buffered").value(ms_list(h.buffered));
    st(base + "target").value(ms_list(h.target));
    char line[160];
    std::snprintf(line, sizeof line,
                  "target %d ms, holding %d ms; ran dry %llu (late %llu), trimmed %.0f ms",
                  ms(s.target_s), ms(s.buffered_s), static_cast<unsigned long long>(s.underruns),
                  static_cast<unsigned long long>(s.late_underruns),
                  static_cast<double>(s.trimmed_frames) * ms_per_frame);
    st(base + "now").value(line);
  }
  for (auto it = _peers.begin(); it != _peers.end();)
    it = live.count(it->first) ? std::next(it) : _peers.erase(it);
  st(key("audio.device_ms")).value(std::to_string(ms(device_s)));
  st(key("audio.ahead_ms")).value(std::to_string(ms(ahead_s)));
  st(key("audio.window_s"))
      .value(std::to_string(static_cast<int>(std::lround(_period_s * _history))));
}

void voice_monitor::forget(const std::string &pid) { _peers.erase(pid); }

} // namespace conference
} // namespace ariadne
} // namespace cvc
