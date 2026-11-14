/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// A sim_world's input trace (cvc/nav/sim_world_trace.h). Kept out of sim_world.cpp, as its
// state-tree binding is, so the state tree's headers never meet the drive's.

#include "../core/input_trace_json.h"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <cvc/core/app.h>
#include <cvc/core/clock_follower.h>
#include <cvc/core/input_trace.h>
#include <cvc/core/world_clock.h>
#include <cvc/nav/coef_mlp.h>
#include <cvc/nav/sim_world_trace.h>
#include <cvc/state/state_blob_store.h> // sha256_hex
#include <fstream>
#include <iterator>
#include <sstream>
#include <stdexcept>
#include <type_traits>

namespace cvc {
namespace nav {

namespace tj = cvc::trace_json;

namespace {

// ── bytes for the hash ─────────────────────────────────────────────────────────

void put_u32(std::vector<unsigned char> &b, std::uint32_t v) {
  for (int k = 0; k < 4; ++k)
    b.push_back(static_cast<unsigned char>(v >> (8 * k)));
}

void put_u64(std::vector<unsigned char> &b, std::uint64_t v) {
  for (int k = 0; k < 8; ++k)
    b.push_back(static_cast<unsigned char>(v >> (8 * k)));
}

void put_f32(std::vector<unsigned char> &b, float f) {
  std::uint32_t u = 0;
  std::memcpy(&u, &f, sizeof u);
  put_u32(b, u);
}

// ── a JSON object, written by hand ───────────────────────────────────────────

class obj {
public:
  obj &key(const char *k) {
    s_ += first_ ? "" : ",";
    first_ = false;
    tj::append_string(s_, k);
    s_ += ':';
    return *this;
  }
  obj &num(const char *k, double v) {
    key(k);
    s_ += tj::number(v);
    return *this;
  }
  obj &num(const char *k, float v) { return num(k, static_cast<double>(v)); }
  obj &num(const char *k, int v) {
    key(k);
    s_ += std::to_string(v);
    return *this;
  }
  obj &num(const char *k, std::uint64_t v) {
    key(k);
    s_ += std::to_string(v);
    return *this;
  }
  obj &flag(const char *k, bool v) {
    key(k);
    s_ += v ? "true" : "false";
    return *this;
  }
  obj &str(const char *k, const std::string &v) {
    key(k);
    tj::append_string(s_, v);
    return *this;
  }
  obj &raw(const char *k, const std::string &json) {
    key(k);
    s_ += json;
    return *this;
  }
  std::string done() const { return "{" + s_ + "}"; }

private:
  std::string s_;
  bool first_ = true;
};

template <class T> std::string array_of(const T *v, std::size_t n) {
  std::string s = "[";
  for (std::size_t k = 0; k < n; ++k) {
    if (k)
      s += ',';
    if constexpr (std::is_floating_point_v<T>)
      s += tj::number(static_cast<double>(v[k]));
    else
      s += std::to_string(v[k]);
  }
  return s + "]";
}

// ── reading it back ──────────────────────────────────────────────────────────

void rd(const tj::value &o, const char *k, double &out) {
  if (const tj::value *v = o.get(k))
    out = v->number(k);
}
void rd(const tj::value &o, const char *k, float &out) {
  if (const tj::value *v = o.get(k))
    out = static_cast<float>(v->number(k));
}
void rd(const tj::value &o, const char *k, int &out) {
  if (const tj::value *v = o.get(k))
    out = static_cast<int>(v->i64(k));
}
void rd(const tj::value &o, const char *k, bool &out) {
  if (const tj::value *v = o.get(k)) {
    if (v->t != tj::value::type::boolean)
      throw std::runtime_error(std::string(k) + " is not true or false");
    out = v->b;
  }
}

template <class T> std::vector<T> rd_array(const tj::value *v, const char *what) {
  std::vector<T> out;
  if (!v)
    return out;
  if (!v->is_array())
    throw std::runtime_error(std::string(what) + " is not an array");
  out.reserve(v->a.size());
  for (const tj::value &x : v->a) {
    if constexpr (std::is_floating_point_v<T>)
      out.push_back(static_cast<T>(x.number(what)));
    else
      out.push_back(static_cast<T>(x.i64(what)));
  }
  return out;
}

const tj::value &member(const tj::value &o, const char *k) {
  const tj::value *v = o.get(k);
  if (!v)
    throw std::runtime_error(std::string("the trace's world has no ") + k);
  return *v;
}

// ── maps, run-length coded ───────────────────────────────────────────────────

std::string map_json(const std::uint8_t *d, int rows, int cols) {
  const std::size_t n = static_cast<std::size_t>(rows) * static_cast<std::size_t>(cols);
  std::string runs = "[";
  std::size_t k = 0;
  bool first = true;
  while (k < n) {
    std::size_t j = k + 1;
    while (j < n && d[j] == d[k])
      ++j;
    runs += first ? "" : ",";
    first = false;
    runs += std::to_string(static_cast<int>(d[k])) + "," + std::to_string(j - k);
    k = j;
  }
  runs += "]";
  return obj()
      .num("rows", rows)
      .num("cols", cols)
      .str("sha256", sha256_hex(d, n))
      .raw("rle", runs)
      .done();
}

std::vector<std::uint8_t> map_from(const tj::value &m, int rows, int cols, const char *what) {
  const std::size_t n = static_cast<std::size_t>(rows) * static_cast<std::size_t>(cols);
  if (member(m, "rows").i64("rows") != rows || member(m, "cols").i64("cols") != cols)
    throw std::runtime_error(std::string("the trace's ") + what + " map is not rows x cols");
  const std::vector<std::int64_t> runs = rd_array<std::int64_t>(&member(m, "rle"), "rle");
  if (runs.size() % 2)
    throw std::runtime_error(std::string("the trace's ") + what + " map has an odd run list");
  std::vector<std::uint8_t> out;
  out.reserve(n);
  for (std::size_t k = 0; k < runs.size(); k += 2) {
    if (runs[k] < 0 || runs[k] > 255 || runs[k + 1] < 0 ||
        out.size() + static_cast<std::size_t>(runs[k + 1]) > n)
      throw std::runtime_error(std::string("the trace's ") + what + " map has a bad run");
    out.insert(out.end(), static_cast<std::size_t>(runs[k + 1]),
               static_cast<std::uint8_t>(runs[k]));
  }
  if (out.size() != n)
    throw std::runtime_error(std::string("the trace's ") + what + " map is short");
  if (const tj::value *h = m.get("sha256"))
    if (h->str("sha256") != sha256_hex(out.data(), out.size()))
      throw std::runtime_error(std::string("the trace's ") + what +
                               " map does not match its sha256");
  return out;
}

// ── the config ───────────────────────────────────────────────────────────────

std::string config_json(const sim_world::config &c) {
  const veh_params &v = c.veh;
  const std::string veh = obj()
                              .num("rr", v.rr)
                              .num("d_hat", v.d_hat)
                              .num("dt", v.dt)
                              .num("vmax", v.vmax)
                              .num("L", v.L)
                              .num("delta_max", v.delta_max)
                              .num("a_max", v.a_max)
                              .num("a_lat_max", v.a_lat_max)
                              .num("k_steer", v.k_steer)
                              .num("nsub", v.nsub)
                              .flag("allow_reverse", v.allow_reverse)
                              .num("body_rr", v.body_rr)
                              .num("body_gain", v.body_gain)
                              .num("mass", v.mass)
                              .num("track_width", v.track_width)
                              .num("mu_lookahead", v.mu_lookahead)
                              .num("mu_probes", v.mu_probes)
                              .num("gravity", v.gravity)
                              .flag("risk_governor", v.risk_governor)
                              .num("risk_vmin", v.risk_vmin)
                              .num("risk_onset", v.risk_onset)
                              .num("risk_lookahead", v.risk_lookahead)
                              .num("risk_probes", v.risk_probes)
                              .done();
  return obj()
      .num("rows", c.rows)
      .num("cols", c.cols)
      .num("min_x", c.min_x)
      .num("min_y", c.min_y)
      .num("max_x", c.max_x)
      .num("max_y", c.max_y)
      .num("cx", c.cx)
      .num("cy", c.cy)
      .num("scale", c.scale)
      .num("range_m", c.range_m)
      .num("n_rays", c.n_rays)
      .num("fov_rad", c.fov_rad)
      .num("reach_tol", c.reach_tol)
      .num("sense_every", c.sense_every)
      .flag("freeze_sense", c.freeze_sense)
      .num("l_occ", c.l_occ)
      .num("l_free", c.l_free)
      .num("l_clamp", c.l_clamp)
      .flag("optimistic", c.optimistic)
      .num("p_thresh", c.p_thresh)
      .num("band", c.band)
      .num("ttl_s", c.ttl_s)
      .num("sep_radius", c.sep_radius)
      .num("sep_gain", c.sep_gain)
      .num("min_gap", c.min_gap)
      .num("min_gap_iters", c.min_gap_iters)
      .str("spawn_layout",
           c.spawn_layout == sim_world::config::spawn::opposed ? "opposed" : "scattered")
      .raw("veh", veh)
      .done();
}

sim_world::config config_from(const tj::value &o) {
  if (!o.is_object())
    throw std::runtime_error("the trace's world config is not a JSON object");
  sim_world::config c;
  rd(o, "rows", c.rows);
  rd(o, "cols", c.cols);
  rd(o, "min_x", c.min_x);
  rd(o, "min_y", c.min_y);
  rd(o, "max_x", c.max_x);
  rd(o, "max_y", c.max_y);
  rd(o, "cx", c.cx);
  rd(o, "cy", c.cy);
  rd(o, "scale", c.scale);
  rd(o, "range_m", c.range_m);
  rd(o, "n_rays", c.n_rays);
  rd(o, "fov_rad", c.fov_rad);
  rd(o, "reach_tol", c.reach_tol);
  rd(o, "sense_every", c.sense_every);
  rd(o, "freeze_sense", c.freeze_sense);
  rd(o, "l_occ", c.l_occ);
  rd(o, "l_free", c.l_free);
  rd(o, "l_clamp", c.l_clamp);
  rd(o, "optimistic", c.optimistic);
  rd(o, "p_thresh", c.p_thresh);
  rd(o, "band", c.band);
  rd(o, "ttl_s", c.ttl_s);
  rd(o, "sep_radius", c.sep_radius);
  rd(o, "sep_gain", c.sep_gain);
  rd(o, "min_gap", c.min_gap);
  rd(o, "min_gap_iters", c.min_gap_iters);
  if (const tj::value *s = o.get("spawn_layout"))
    c.spawn_layout = s->str("spawn_layout") == "opposed" ? sim_world::config::spawn::opposed
                                                         : sim_world::config::spawn::scattered;
  if (const tj::value *vp = o.get("veh")) {
    const tj::value &v = *vp;
    veh_params &p = c.veh;
    rd(v, "rr", p.rr);
    rd(v, "d_hat", p.d_hat);
    rd(v, "dt", p.dt);
    rd(v, "vmax", p.vmax);
    rd(v, "L", p.L);
    rd(v, "delta_max", p.delta_max);
    rd(v, "a_max", p.a_max);
    rd(v, "a_lat_max", p.a_lat_max);
    rd(v, "k_steer", p.k_steer);
    rd(v, "nsub", p.nsub);
    rd(v, "allow_reverse", p.allow_reverse);
    rd(v, "body_rr", p.body_rr);
    rd(v, "body_gain", p.body_gain);
    rd(v, "mass", p.mass);
    rd(v, "track_width", p.track_width);
    rd(v, "mu_lookahead", p.mu_lookahead);
    rd(v, "mu_probes", p.mu_probes);
    rd(v, "gravity", p.gravity);
    rd(v, "risk_governor", p.risk_governor);
    rd(v, "risk_vmin", p.risk_vmin);
    rd(v, "risk_onset", p.risk_onset);
    rd(v, "risk_lookahead", p.risk_lookahead);
    rd(v, "risk_probes", p.risk_probes);
  }
  return c;
}

std::string file_sha256(const std::string &path) {
  std::ifstream f(path, std::ios::binary);
  if (!f)
    throw std::runtime_error("cvc::nav::trace_header: cannot read the policy " + path);
  const std::vector<unsigned char> bytes((std::istreambuf_iterator<char>(f)),
                                         std::istreambuf_iterator<char>());
  return sha256_hex(bytes);
}

std::string join(const std::vector<std::string> &v, const char *sep) {
  std::string s;
  for (std::size_t k = 0; k < v.size(); ++k)
    s += (k ? sep : "") + v[k];
  return s;
}

struct policy_source {
  std::string uri, sha256; // as policy_ref
  std::string key;         // the world's key for it ("" for policy 0)
  std::string name;        // the agents' "policy": "default", "sha256:<hex>", or "" (unknown)
};

std::string agent_args(std::uint64_t id, const char *what, const std::vector<float> &v) {
  std::string a = "{\"agent\":" + std::to_string(id) + ",\"";
  a += what;
  a += "\":" + array_of(v.data(), v.size()) + "}";
  return a;
}

int agent_index(const sim_world &w, const tj::value &args, const char *verb) {
  const tj::value *a = args.get("agent");
  if (!a)
    throw std::runtime_error(std::string(verb) + ": no agent");
  const int i = w.index_of(a->u64("agent"));
  if (i < 0)
    throw std::runtime_error(std::string(verb) + ": no agent " + a->s + " in the world");
  return i;
}

std::vector<float> floats(const tj::value &args, const char *k, std::size_t n, const char *verb) {
  const std::vector<float> v = rd_array<float>(args.get(k), k);
  if (v.size() != n)
    throw std::runtime_error(std::string(verb) + ": " + k + " needs " + std::to_string(n) +
                             " numbers");
  return v;
}

} // namespace

std::string state_hash(const sim_world &w) {
  const int n = w.size();
  std::vector<unsigned char> b;
  b.reserve(32 + static_cast<std::size_t>(n) * 41);
  static const char kTag[] = "cvc.nav.state/v1";
  b.insert(b.end(), kTag, kTag + sizeof kTag - 1);
  put_u64(b, static_cast<std::uint64_t>(w.tick()));
  put_u32(b, static_cast<std::uint32_t>(n));
  std::vector<int> mode(static_cast<std::size_t>(n));
  std::vector<std::uint8_t> reached(static_cast<std::size_t>(n));
  if (n > 0)
    w.snapshot(nullptr, nullptr, nullptr, mode.data(), reached.data());
  const float *o = w.positions_normalized(), *g = w.goals_normalized();
  const float *th = w.headings(), *sp = w.speeds_normalized();
  for (int i = 0; i < n; ++i) {
    put_u64(b, w.id_of(i));
    put_f32(b, o[2 * i]);
    put_f32(b, o[2 * i + 1]);
    put_f32(b, g[2 * i]);
    put_f32(b, g[2 * i + 1]);
    put_f32(b, th[i]);
    put_f32(b, sp[i]);
    put_u32(b, static_cast<std::uint32_t>(mode[static_cast<std::size_t>(i)]));
    b.push_back(reached[static_cast<std::size_t>(i)]);
  }
  return "sha256:" + sha256_hex(b);
}

input_trace_header trace_header(const sim_world &w, const trace_options &o) {
  const sim_world::config &c = w.current_config();
  const int n = w.size();
  std::vector<std::string> why;
  if (w.tick() != 0)
    why.push_back("the world has stepped (tick " + std::to_string(w.tick()) +
                  "): a trace starts before the first step");
  if (w.plant())
    why.push_back("a vehicle plant (v1 records the kinematic plant only)");
  if (w.has_terrain())
    why.push_back("terrain");
  if (w.has_material())
    why.push_back("a material stack");
  if (c.veh.n_body > 0)
    why.push_back("a multi-disc body");
  if (c.veh.rr_col || c.veh.body_rr_col || c.veh.body_offsets_col || c.veh.mass_col ||
      c.veh.vmax_col || c.veh.a_max_col || c.veh.L_col)
    why.push_back("per-agent vehicle columns");
  for (int i = 0; i < n; ++i)
    if (w.id_of(i) != static_cast<std::uint64_t>(i) + 1) {
      why.push_back("agents added or removed (the ids are not 1..n)");
      break;
    }
  if (!o.embed_maps)
    why.push_back("the maps are not embedded");

  // Each policy's source.
  std::vector<policy_source> pols(static_cast<std::size_t>(w.policy_count()));
  for (int p = 0; p < w.policy_count(); ++p) {
    policy_source &s = pols[static_cast<std::size_t>(p)];
    s.key = w.policy_key(p);
    const auto it = o.policies.find(p);
    if (it != o.policies.end()) {
      s.uri = it->second.uri;
      s.sha256 = it->second.sha256;
      if (!s.uri.empty() && s.sha256.empty())
        s.sha256 = file_sha256(s.uri);
      if (s.uri.empty() && !s.sha256.empty())
        why.push_back("policy " + std::to_string(p) + " has a sha256 and no file");
      s.name = s.uri.empty() && s.sha256.empty() ? "default" : "sha256:" + s.sha256;
    } else if (p > 0 && s.key == "default") {
      s.name = "default";
    } else if (p > 0 && s.key.rfind("sha256:", 0) == 0) {
      s.sha256 = s.key.substr(7);
      s.name = s.key;
      why.push_back("policy " + std::to_string(p) + " (" + s.key +
                    ") has no file (trace_options::policies)");
    } else {
      why.push_back(p == 0 ? std::string("the world's own policy is not stated "
                                         "(trace_options::policies[0])")
                           : "policy " + std::to_string(p) + "'s file is not known");
    }
  }
  if (!why.empty() && !o.allow_incomplete)
    throw std::invalid_argument("cvc::nav::trace_header: v1 cannot rebuild this world: " +
                                join(why, "; "));

  input_trace_header h;
  h.seed = o.seed;
  h.dt = static_cast<double>(c.veh.dt);
  h.phase = o.phase;
  h.world_id = o.world_id;
  h.meta = o.meta.empty() ? std::string("{}") : o.meta;
  const float *on = w.positions_normalized(), *gn = w.goals_normalized();
  const float *th = w.headings(), *sp = w.speeds_normalized();
  std::vector<int> policy_of(static_cast<std::size_t>(n));
  for (int i = 0; i < n; ++i) {
    input_trace_agent a;
    a.id = w.id_of(i);
    a.start = {on[2 * i], on[2 * i + 1], th[i], sp[i]};
    a.goal = {gn[2 * i], gn[2 * i + 1]};
    policy_of[static_cast<std::size_t>(i)] = w.policy_of(i);
    a.policy = pols[static_cast<std::size_t>(policy_of[static_cast<std::size_t>(i)])].name;
    h.agents.push_back(std::move(a));
  }

  std::string policies = "[";
  for (std::size_t p = 0; p < pols.size(); ++p) {
    policies += p ? "," : "";
    policies += obj()
                    .num("index", static_cast<int>(p))
                    .str("key", pols[p].key)
                    .str("uri", pols[p].uri)
                    .str("sha256", pols[p].sha256)
                    .done();
  }
  policies += "]";
  std::vector<float> colors(static_cast<std::size_t>(n) * 3);
  for (int i = 0; i < n; ++i)
    w.colors(i, colors.data() + 3 * i, nullptr);

  obj world;
  world.str("kind", kTraceWorldKind).flag("complete", why.empty());
  if (!why.empty()) {
    std::string list = "[";
    for (std::size_t k = 0; k < why.size(); ++k) {
      list += k ? "," : "";
      tj::append_string(list, why[k]);
    }
    world.raw("incomplete", list + "]");
  }
  world.str("prefix", o.prefix)
      .str("pull", o.pull_every_step ? "step" : "none")
      .num("threads", o.num_threads)
      .raw("config", config_json(c))
      .num("planes", w.planes());
  if (w.planes() > 1)
    world.raw("map_id", array_of(w.agent_planes(), static_cast<std::size_t>(n)));
  world.raw("colors", array_of(colors.data(), colors.size())).raw("policies", policies);
  if (w.policy_count() > 1)
    world.raw("policy_of", array_of(policy_of.data(), policy_of.size()));
  if (o.embed_maps) {
    world.raw("truth", map_json(w.truth(), w.rows(), w.cols()));
    const std::size_t hw = static_cast<std::size_t>(w.rows()) * static_cast<std::size_t>(w.cols());
    if (std::memcmp(w.truth(), w.prior(), hw) == 0)
      world.raw("prior", "{\"same_as\":\"truth\"}");
    else
      world.raw("prior", map_json(w.prior(), w.rows(), w.cols()));
  }
  h.world = world.done();
  return h;
}

sim_world world_from_trace(const input_trace_header &h, policy_library &lib,
                           const std::map<int, std::string> &policy_files) {
  const tj::value w = tj::parse(h.world);
  const tj::value *kind = w.get("kind");
  if (!kind || !kind->is_string() || kind->s != kTraceWorldKind)
    throw std::runtime_error(
        std::string("cvc::nav::world_from_trace: the trace's world is not a ") + kTraceWorldKind);
  const tj::value *complete = w.get("complete");
  if (!complete || complete->t != tj::value::type::boolean || !complete->b) {
    std::string why;
    if (const tj::value *list = w.get("incomplete"); list && list->is_array())
      for (const tj::value &x : list->a)
        why += (why.empty() ? ": " : "; ") + (x.is_string() ? x.s : x.dump());
    throw std::runtime_error("cvc::nav::world_from_trace: the trace's world is incomplete" + why +
                             "; the host that recorded it rebuilds it");
  }
  const sim_world::config c = config_from(member(w, "config"));
  if (c.rows <= 0 || c.cols <= 0)
    throw std::runtime_error("cvc::nav::world_from_trace: the world has no grid");
  const std::vector<std::uint8_t> truth = map_from(member(w, "truth"), c.rows, c.cols, "truth");
  const tj::value &pm = member(w, "prior");
  const tj::value *same = pm.get("same_as");
  const std::vector<std::uint8_t> prior = same && same->is_string() && same->s == "truth"
                                              ? truth
                                              : map_from(pm, c.rows, c.cols, "prior");

  const int n = static_cast<int>(h.agents.size());
  std::vector<float> on(static_cast<std::size_t>(n) * 2), gn(static_cast<std::size_t>(n) * 2);
  for (int i = 0; i < n; ++i) {
    const input_trace_agent &a = h.agents[static_cast<std::size_t>(i)];
    if (a.id != static_cast<std::uint64_t>(i) + 1)
      throw std::runtime_error("cvc::nav::world_from_trace: agent ids are not 1..n");
    if (a.start.size() < 2 || a.goal.size() < 2)
      throw std::runtime_error("cvc::nav::world_from_trace: an agent has no start or goal");
    on[2 * i] = static_cast<float>(a.start[0]);
    on[2 * i + 1] = static_cast<float>(a.start[1]);
    gn[2 * i] = static_cast<float>(a.goal[0]);
    gn[2 * i + 1] = static_cast<float>(a.goal[1]);
  }
  std::vector<float> color = rd_array<float>(w.get("colors"), "colors");
  if (color.size() != static_cast<std::size_t>(n) * 3)
    color.assign(static_cast<std::size_t>(n) * 3, 0.8f);
  const int planes = w.get("planes") ? static_cast<int>(member(w, "planes").i64("planes")) : 1;
  const std::vector<int> map_id = rd_array<int>(w.get("map_id"), "map_id");
  if (planes > 1 && map_id.size() != static_cast<std::size_t>(n))
    throw std::runtime_error("cvc::nav::world_from_trace: map_id is not one plane per agent");

  // The policies, by index.
  const tj::value &plist = member(w, "policies");
  if (!plist.is_array() || plist.a.empty())
    throw std::runtime_error("cvc::nav::world_from_trace: the world lists no policies");
  std::vector<std::shared_ptr<const coef_mlp>> models;
  std::vector<std::string> keys;
  for (std::size_t p = 0; p < plist.a.size(); ++p) {
    const tj::value &e = plist.a[p];
    const auto f = policy_files.find(static_cast<int>(p));
    if (f != policy_files.end()) {
      models.push_back(std::make_shared<const coef_mlp>(coef_mlp::load(f->second)));
    } else {
      policy_ref ref;
      if (const tj::value *u = e.get("uri"))
        ref.uri = u->str("uri");
      if (const tj::value *s = e.get("sha256"))
        ref.sha256 = s->str("sha256");
      models.push_back(lib.get(ref).model);
    }
    const tj::value *k = e.get("key");
    keys.push_back(k ? k->str("key") : std::string());
  }
  const std::vector<int> policy_of = rd_array<int>(w.get("policy_of"), "policy_of");

  sim_world world(c, truth.data(), prior.data(), coef_mlp(*models[0]), on.data(), gn.data(),
                  color.data(), n, planes > 1 ? map_id.data() : nullptr, planes);
  for (int i = 0; i < n; ++i) {
    const input_trace_agent &a = h.agents[static_cast<std::size_t>(i)];
    const float heading = a.start.size() > 2 ? static_cast<float>(a.start[2]) : world.headings()[i];
    const float speed = a.start.size() > 3 ? static_cast<float>(a.start[3]) : 0.0f;
    world.set_pose(i, on[2 * i], on[2 * i + 1], heading, speed);
    if (static_cast<std::size_t>(i) < policy_of.size()) {
      const int p = policy_of[static_cast<std::size_t>(i)];
      if (p < 0 || p >= static_cast<int>(models.size()))
        throw std::runtime_error("cvc::nav::world_from_trace: an agent's policy is not listed");
      if (p > 0)
        world.set_policy(i, models[static_cast<std::size_t>(p)], keys[static_cast<std::size_t>(p)]);
    }
  }
  return world;
}

bool trace_retarget(sim_world &w, input_trace_recorder *rec, std::uint64_t id, float gx_n,
                    float gy_n) {
  const int i = w.index_of(id);
  if (i < 0)
    return false;
  w.retarget(i, gx_n, gy_n);
  if (rec)
    rec->event("nav.retarget", agent_args(id, "goal", {gx_n, gy_n}));
  return true;
}

bool trace_set_pose(sim_world &w, input_trace_recorder *rec, std::uint64_t id, float x_n, float y_n,
                    float heading, float speed) {
  const int i = w.index_of(id);
  if (i < 0 || !w.set_pose(i, x_n, y_n, heading, speed))
    return false;
  if (rec)
    rec->event("nav.set_pose", agent_args(id, "pose", {x_n, y_n, heading, speed}));
  return true;
}

bool trace_override(sim_world &w, input_trace_recorder *rec, std::uint64_t id, float steer,
                    float accel) {
  const int i = w.index_of(id);
  if (i < 0)
    return false;
  w.override_command(i, steer, accel);
  if (rec)
    rec->event("nav.override", agent_args(id, "command", {steer, accel}));
  return true;
}

void trace_add_obstacle(sim_world &w, input_trace_recorder *rec, int r0, int r1, int c0, int c1) {
  w.add_obstacle(r0, r1, c0, c1);
  if (rec) {
    const int rect[4] = {r0, r1, c0, c1};
    rec->event("nav.add_obstacle", "{\"rect\":" + array_of(rect, 4) + "}");
  }
}

bool apply_trace_event(sim_world &w, const input_trace_record &e) {
  if (e.type != input_trace_record::kind::event)
    return false;
  const std::string &verb = e.name;
  if (verb != "nav.retarget" && verb != "nav.set_pose" && verb != "nav.override" &&
      verb != "nav.add_obstacle")
    return false;
  const tj::value args = tj::parse(e.args);
  try {
    if (verb == "nav.add_obstacle") {
      const std::vector<int> r = rd_array<int>(args.get("rect"), "rect");
      if (r.size() != 4)
        throw std::runtime_error("nav.add_obstacle: rect needs 4 numbers");
      w.add_obstacle(r[0], r[1], r[2], r[3]);
      return true;
    }
    const int i = agent_index(w, args, verb.c_str());
    if (verb == "nav.retarget") {
      const std::vector<float> g = floats(args, "goal", 2, "nav.retarget");
      w.retarget(i, g[0], g[1]);
    } else if (verb == "nav.set_pose") {
      const std::vector<float> p = floats(args, "pose", 4, "nav.set_pose");
      w.set_pose(i, p[0], p[1], p[2], p[3]);
    } else {
      const std::vector<float> u = floats(args, "command", 2, "nav.override");
      w.override_command(i, u[0], u[1]);
    }
  } catch (const std::exception &x) {
    throw std::runtime_error(std::string("cvc::nav::apply_trace_event at tick ") +
                             std::to_string(e.tick) + ": " + x.what());
  }
  return true;
}

replay_result replay_trace(cvc::app &app, const input_trace &t, const replay_options &o) {
  const input_trace_header &h = t.header;
  if (!(h.dt > 0.0) || !(h.fixed_dt > 0.0))
    throw std::runtime_error("cvc::nav::replay_trace: the trace states no dt or fixed_dt");
  const tj::value wj = tj::parse(h.world);
  std::string prefix;
  if (const tj::value *p = wj.get("prefix"))
    prefix = p->str("prefix");
  bool pull = true;
  if (const tj::value *p = wj.get("pull"))
    pull = p->str("pull") == "step";
  int threads = 1;
  if (const tj::value *p = wj.get("threads"))
    threads = static_cast<int>(p->i64("threads"));
  if (o.num_threads > 0)
    threads = o.num_threads;

  replay_result r;
  for (const auto &f : o.policy_files)
    r.notes.push_back("policy " + std::to_string(f.first) + " from " + f.second +
                      ", not the recorded one");
  policy_library lib;
  sim_world world = world_from_trace(h, lib, o.policy_files);

  cvc::world_clock &clock = app.world_clock();
  input_trace_player play(app, clock, t);
  play.on_event([&](const input_trace_record &e) {
    if (!apply_trace_event(world, e))
      throw std::runtime_error("cvc::nav::replay_trace: the event " + e.name + " at tick " +
                               std::to_string(e.tick) +
                               " is not nav's; the host that recorded it replays it");
  });
  play.set_state_hash([&] { return state_hash(world); });
  play.rewind();
  clock_follower nav(ticks_per_step(h.dt, h.fixed_dt), h.phase, clock.tick());
  const int n = world.size();
  r.ticks = play.run([&](std::uint64_t tick) {
    for (std::uint64_t k = nav.take(clock); k > 0; --k) {
      if (pull)
        world.pull(app, prefix);
      world.step(threads);
      ++r.steps;
      if (o.keep_poses) {
        r.pose_ticks.push_back(tick);
        const float *on = world.positions_normalized();
        for (int i = 0; i < n; ++i) {
          r.poses.push_back(on[2 * i]);
          r.poses.push_back(on[2 * i + 1]);
          r.poses.push_back(world.headings()[i]);
          r.poses.push_back(world.speeds_normalized()[i]);
        }
      }
    }
  });
  r.hash = state_hash(world);
  r.has_end = t.end.present;
  r.expected = t.end.present ? t.end.hash : std::string();
  r.compared = play.compared();
  r.agreed = play.matched_count();
  r.diverged = play.diverged();
  r.first_divergence = play.first_divergence();
  r.matched = play.matched();
  return r;
}

std::string describe(const replay_result &r) {
  std::ostringstream o;
  o << "replayed   " << r.ticks << " quanta, " << r.steps << " steps\n";
  o << "hash       " << r.hash << "\n";
  if (r.has_end)
    o << "recorded   " << r.expected << "\n";
  o << "compared   " << r.compared << " hashes, " << r.agreed << " matched\n";
  for (const std::string &note : r.notes)
    o << "note       " << note << "\n";
  if (r.matched)
    o << "result     the replay reproduces the trace\n";
  else if (r.diverged)
    o << "result     DIVERGED: first at tick " << r.first_divergence << "\n";
  else if (!r.has_end)
    o << "result     the trace has no end, so no final hash to compare\n";
  else
    o << "result     not compared\n";
  return o.str();
}

} // namespace nav
} // namespace cvc
