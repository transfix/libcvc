// NOTE: do NOT run `clang-format -i` on this .i file (see pycvc_nav.i): it mangles SWIG's %{ %}.
//
// pycvc_nav_coph.i — Python surface for cvc::nav's CoPH (C6 of docs/roadmap/NAV-COPH-ROADMAP.md):
// the kernels GRL-SNAM's goldens pin, one learner (NavCophAgent), the packets between learners
// (NavCophPacket), the runner's fleet (NavCophFleet) and the driver that runs a fleet on a
// sim_world's vehicles (NavCophDriver, pycvc_gl.cars' --controller coph). NOT a standalone module: %include'd by
// pycvc.i. The logic is the library's (cvc/nav/coph_*.h); these functions only marshal.
//
// Vectors cross as sequences of floats and come back as lists: a state is (x, y, vx, vy), a force
// or a point (x, y). A shape is (x, y, r), a disk, or (x, y, r, hx, hy), a slab of half-thickness r
// about the segment from (x - hx, y - hy) to (x + hx, y + hy). A configuration is a dict over the
// demos' profile (coph_profile(), D7): its keys are coph_config's field names, and any other key is
// an error; nav_coph_profile() returns the whole of it. The communication dict is over
// coph_communication_profile() the same way. A scene, for the potential's kernels, is a dict:
//   {"centers": [(x, y), ...], "weights": [...], "width": 0.8, "obstacles": [shape, ...]}.
//
//   x1 = pycvc.nav_coph_flow((0, 0, 1, 0), (0.5, 0), 0.6, 0.12)
//   s = pycvc.nav_coph_shaping(scene, {"policy_bounds": (0, 10)})   # None under legacy shaping
//   V = pycvc.nav_coph_potential(q, target, policy, scene, s)
//   fleet = pycvc.NavCophFleet({"steps": 2000}, {}, [{"initial": (..), "goal": (..)}, ...],
//                              senders=[0, 1], seed=7)
//   fleet.run_crossing()
//   fleet.success(0), fleet.energy(0)["residual"], fleet.events()

%{
#include <cvc/nav/coph_agent.h>
#include <cvc/nav/coph_allocation.h>
#include <cvc/nav/coph_channel.h>
#include <cvc/nav/coph_config.h>
#include <cvc/nav/coph_driver.h>
#include <cvc/nav/coph_fleet.h>
#include <cvc/nav/coph_model.h>
#include <cvc/nav/coph_packet.h>
#include <functional>
#include <memory>
#include <optional>
#include <random>
#include <string>
#include <utility>
#include <vector>

namespace pycvc {
namespace coph {

using cvc::nav::coph_shape;
using cvc::nav::coph_state;
using cvc::nav::coph_vec2;

// A borrowed item of a dict, or null when absent or None.
PyObject *item(PyObject *dict, const char *key) {
  if (!dict || dict == Py_None)
    return nullptr;
  if (!PyDict_Check(dict))
    throw std::invalid_argument(std::string("expected a dict holding '") + key + "'");
  PyObject *v = PyDict_GetItemString(dict, key);
  return v == Py_None ? nullptr : v;
}

double number(PyObject *o, const std::string &what) {
  if (!o)
    throw std::invalid_argument(what + " is missing");
  const double v = PyFloat_AsDouble(o);
  if (v == -1.0 && PyErr_Occurred()) {
    PyErr_Clear();
    throw std::invalid_argument(what + " is not a number");
  }
  return v;
}

long long integer(PyObject *o, const std::string &what) {
  if (!o || !PyLong_Check(o))
    throw std::invalid_argument(what + " is not an int");
  return PyLong_AsLongLong(o);
}

std::string text(PyObject *o, const std::string &what) {
  if (!o || !PyUnicode_Check(o))
    throw std::invalid_argument(what + " is not a str");
  return PyUnicode_AsUTF8(o);
}

std::vector<double> floats(PyObject *o, const std::string &what) {
  std::vector<double> out;
  if (!o || o == Py_None)
    return out;
  PyObject *seq = PySequence_Fast(o, what.c_str());
  if (!seq) {
    PyErr_Clear();
    throw std::invalid_argument(what + " is not a sequence of numbers");
  }
  const Py_ssize_t n = PySequence_Fast_GET_SIZE(seq);
  for (Py_ssize_t i = 0; i < n; ++i) {
    const double v = PyFloat_AsDouble(PySequence_Fast_GET_ITEM(seq, i));
    if (v == -1.0 && PyErr_Occurred()) {
      PyErr_Clear();
      Py_DECREF(seq);
      throw std::invalid_argument(what + "[" + std::to_string(i) + "] is not a number");
    }
    out.push_back(v);
  }
  Py_DECREF(seq);
  return out;
}

// The items of a sequence (new references released by the caller's loop through `each`).
void each(PyObject *o, const std::string &what, const std::function<void(PyObject *, int)> &f) {
  if (!o || o == Py_None)
    return;
  PyObject *seq = PySequence_Fast(o, what.c_str());
  if (!seq) {
    PyErr_Clear();
    throw std::invalid_argument(what + " is not a sequence");
  }
  const Py_ssize_t n = PySequence_Fast_GET_SIZE(seq);
  try {
    for (Py_ssize_t i = 0; i < n; ++i)
      f(PySequence_Fast_GET_ITEM(seq, i), static_cast<int>(i));
  } catch (...) {
    Py_DECREF(seq);
    throw;
  }
  Py_DECREF(seq);
}

coph_vec2 vec2(PyObject *o, const std::string &what) {
  const std::vector<double> v = floats(o, what);
  if (v.size() != 2)
    throw std::invalid_argument(what + " needs 2 numbers");
  return {v[0], v[1]};
}

coph_state state4(PyObject *o, const std::string &what) {
  const std::vector<double> v = floats(o, what);
  if (v.size() != 4)
    throw std::invalid_argument(what + " needs 4 numbers (x, y, vx, vy)");
  return {v[0], v[1], v[2], v[3]};
}

PyObject *list(const double *v, std::size_t n) {
  PyObject *l = PyList_New(static_cast<Py_ssize_t>(n));
  for (std::size_t i = 0; i < n; ++i)
    PyList_SET_ITEM(l, static_cast<Py_ssize_t>(i), PyFloat_FromDouble(v[i]));
  return l;
}
PyObject *list(const coph_vec2 &v) { return list(v.data(), 2); }
PyObject *list(const coph_state &x) { return list(x.data(), 4); }
PyObject *opt(const std::optional<coph_vec2> &v) {
  if (v)
    return list(*v);
  Py_RETURN_NONE;
}

// Sets d[key] = value and releases value.
void put(PyObject *d, const char *key, PyObject *value) {
  PyDict_SetItemString(d, key, value);
  Py_DECREF(value);
}

coph_shape shape(PyObject *o, const std::string &what) {
  const std::vector<double> v = floats(o, what);
  if (v.size() == 3)
    return coph_shape::disk(v[0], v[1], v[2]);
  if (v.size() == 5)
    return coph_shape::segment(v[0], v[1], v[2], v[3], v[4]);
  throw std::invalid_argument(what + " needs (x, y, r) or (x, y, r, hx, hy)");
}

std::vector<coph_shape> shapes(PyObject *o, const std::string &what) {
  std::vector<coph_shape> out;
  each(o, what, [&](PyObject *s, int i) { out.push_back(shape(s, what + "[" + std::to_string(i) + "]")); });
  return out;
}

PyObject *shape_list(const coph_shape &s) {
  if (s.slab) {
    const double v[5] = {s.x, s.y, s.r, s.hx, s.hy};
    return list(v, 5);
  }
  const double v[3] = {s.x, s.y, s.r};
  return list(v, 3);
}

// ---------------------------------------------------------------------------
// The configurations: dicts over the demos' profile
// ---------------------------------------------------------------------------

// One coph_config field: its name, and how a dict's value sets it and how it reads back. A
// constexpr table of names and plain function pointers, so it is constant data with no
// initialisation at run time (libcvc keeps no process-wide state: tools/singleton_scan).
struct config_field {
  const char *name;
  void (*set)(cvc::nav::coph_config &, PyObject *, const std::string &);
  PyObject *(*get)(const cvc::nav::coph_config &);
};

#define PYCVC_COPH_STR(f) \
  {#f, [](cvc::nav::coph_config &c, PyObject *o, const std::string &w) { c.f = text(o, w); }, \
   [](const cvc::nav::coph_config &c) { return PyUnicode_FromString(c.f.c_str()); }}
#define PYCVC_COPH_DBL(f) \
  {#f, [](cvc::nav::coph_config &c, PyObject *o, const std::string &w) { c.f = number(o, w); }, \
   [](const cvc::nav::coph_config &c) { return PyFloat_FromDouble(c.f); }}
#define PYCVC_COPH_INT(f) \
  {#f, [](cvc::nav::coph_config &c, PyObject *o, const std::string &w) { c.f = static_cast<int>(integer(o, w)); }, \
   [](const cvc::nav::coph_config &c) { return PyLong_FromLong(c.f); }}
#define PYCVC_COPH_PAIR(f) \
  {#f, [](cvc::nav::coph_config &c, PyObject *o, const std::string &w) { \
     const coph_vec2 v = vec2(o, w); \
     c.f = {v[0], v[1]}; \
   }, \
   [](const cvc::nav::coph_config &c) { return Py_BuildValue("(dd)", c.f.first, c.f.second); }}

constexpr config_field kConfigFields[] = {
    PYCVC_COPH_STR(stage_blend_mode), PYCVC_COPH_STR(route_field_mode), PYCVC_COPH_STR(controller_mode),
    PYCVC_COPH_STR(shaping_mode), PYCVC_COPH_DBL(shaping_k), PYCVC_COPH_DBL(shaping_ell),
    PYCVC_COPH_DBL(shaping_eta), PYCVC_COPH_DBL(safety_margin), PYCVC_COPH_INT(obstacle_cap),
    PYCVC_COPH_INT(footprint_cap), PYCVC_COPH_INT(route_expansion_cap), PYCVC_COPH_STR(navigation_target),
    PYCVC_COPH_STR(replay_objective), PYCVC_COPH_STR(replay_domain), PYCVC_COPH_STR(replay_horizon_mode),
    {"replay_update_cap",
     [](cvc::nav::coph_config &c, PyObject *o, const std::string &w) {
       if (o)
         c.replay_update_cap = static_cast<int>(integer(o, w));
       else
         c.replay_update_cap.reset();
     },
     [](const cvc::nav::coph_config &c) -> PyObject * {
       if (c.replay_update_cap)
         return PyLong_FromLong(*c.replay_update_cap);
       Py_RETURN_NONE;
     }},
    PYCVC_COPH_INT(boundary_max_steps), PYCVC_COPH_INT(boundary_follow_steps), PYCVC_COPH_DBL(dt),
    PYCVC_COPH_INT(steps), PYCVC_COPH_DBL(stage_width), PYCVC_COPH_DBL(stage_height),
    PYCVC_COPH_DBL(overlap), PYCVC_COPH_INT(previous), PYCVC_COPH_INT(sample_cap),
    PYCVC_COPH_INT(transition_cap), PYCVC_COPH_INT(anchor_cap), PYCVC_COPH_INT(rbf_side),
    PYCVC_COPH_DBL(rbf_width), PYCVC_COPH_INT(update_every), PYCVC_COPH_INT(replay_steps),
    PYCVC_COPH_INT(solver_steps), PYCVC_COPH_DBL(damping_initial), PYCVC_COPH_PAIR(policy_initial),
    PYCVC_COPH_PAIR(damping_bounds), PYCVC_COPH_PAIR(policy_bounds), PYCVC_COPH_DBL(damping_trust),
    PYCVC_COPH_DBL(policy_trust), PYCVC_COPH_DBL(fit_ridge), PYCVC_COPH_DBL(validation_slack),
    PYCVC_COPH_DBL(sensor_radius), PYCVC_COPH_DBL(force_limit),
};

#undef PYCVC_COPH_STR
#undef PYCVC_COPH_DBL
#undef PYCVC_COPH_INT
#undef PYCVC_COPH_PAIR

const config_field *find_field(const std::string &name) {
  for (const config_field &f : kConfigFields)
    if (name == f.name)
      return &f;
  return nullptr;
}

// The dict's keys over the demos' profile, validated (Config.__post_init__). A key set to None
// clears an optional (replay_update_cap).
cvc::nav::coph_config config(PyObject *d) {
  cvc::nav::coph_config c = cvc::nav::coph_profile();
  if (d && d != Py_None) {
    if (!PyDict_Check(d))
      throw std::invalid_argument("the configuration is not a dict");
    PyObject *key = nullptr, *value = nullptr;
    Py_ssize_t pos = 0;
    while (PyDict_Next(d, &pos, &key, &value)) {
      const std::string k = text(key, "a configuration key");
      const config_field *f = find_field(k);
      if (!f)
        throw std::invalid_argument("coph_config has no field '" + k + "'");
      f->set(c, value == Py_None ? nullptr : value, "configuration['" + k + "']");
    }
  }
  c.validate();
  return c;
}

PyObject *config_dict(const cvc::nav::coph_config &c) {
  PyObject *d = PyDict_New();
  for (const config_field &f : kConfigFields)
    put(d, f.name, f.get(c));
  return d;
}

cvc::nav::coph_communication_config communication(PyObject *d) {
  cvc::nav::coph_communication_config c = cvc::nav::coph_communication_profile();
  if (d && d != Py_None) {
    if (!PyDict_Check(d))
      throw std::invalid_argument("the communication is not a dict");
    PyObject *key = nullptr, *value = nullptr;
    Py_ssize_t pos = 0;
    while (PyDict_Next(d, &pos, &key, &value)) {
      const std::string k = text(key, "a communication key");
      const std::string w = "communication['" + k + "']";
      if (k == "ensemble_size") c.ensemble_size = static_cast<int>(integer(value, w));
      else if (k == "anchors") c.anchors = static_cast<int>(integer(value, w));
      else if (k == "received_cap") c.received_cap = static_cast<int>(integer(value, w));
      else if (k == "ttl") c.ttl = number(value, w);
      else if (k == "temporal_window") c.temporal_window = number(value, w);
      else if (k == "covariance_floor") c.covariance_floor = number(value, w);
      else if (k == "covariance_scale") c.covariance_scale = number(value, w);
      else if (k == "byte_budget") c.byte_budget = static_cast<int>(integer(value, w));
      else if (k == "byte_price") c.byte_price = number(value, w);
      else if (k == "processing_price") c.processing_price = number(value, w);
      else if (k == "rate_regularizer") c.rate_regularizer = number(value, w);
      else if (k == "max_rate") c.max_rate = number(value, w);
      else if (k == "delay_ticks") c.delay_ticks = static_cast<int>(integer(value, w));
      else if (k == "prediction_variant") c.prediction_variant = text(value, w);
      else throw std::invalid_argument("coph_communication_config has no field '" + k + "'");
    }
  }
  c.validate();
  return c;
}

PyObject *communication_dict(const cvc::nav::coph_communication_config &c) {
  PyObject *d = PyDict_New();
  put(d, "ensemble_size", PyLong_FromLong(c.ensemble_size));
  put(d, "anchors", PyLong_FromLong(c.anchors));
  put(d, "received_cap", PyLong_FromLong(c.received_cap));
  put(d, "ttl", PyFloat_FromDouble(c.ttl));
  put(d, "temporal_window", PyFloat_FromDouble(c.temporal_window));
  put(d, "covariance_floor", PyFloat_FromDouble(c.covariance_floor));
  put(d, "covariance_scale", PyFloat_FromDouble(c.covariance_scale));
  put(d, "byte_budget", PyLong_FromLong(c.byte_budget));
  put(d, "byte_price", PyFloat_FromDouble(c.byte_price));
  put(d, "processing_price", PyFloat_FromDouble(c.processing_price));
  put(d, "rate_regularizer", PyFloat_FromDouble(c.rate_regularizer));
  put(d, "max_rate", PyFloat_FromDouble(c.max_rate));
  put(d, "delay_ticks", PyLong_FromLong(c.delay_ticks));
  put(d, "prediction_variant", PyUnicode_FromString(c.prediction_variant.c_str()));
  return d;
}

// "crossing" (or None): the crossing demo's surveyed network; "none": no network, a car's learner.
cvc::nav::coph_network network(PyObject *o) {
  const std::string n = o && o != Py_None ? text(o, "network") : "crossing";
  if (n == "crossing")
    return cvc::nav::coph_crossing_network();
  if (n == "none")
    return cvc::nav::coph_network{};
  throw std::invalid_argument("network is 'crossing' or 'none', not '" + n + "'");
}

// ---------------------------------------------------------------------------
// Scenes, observations, transitions
// ---------------------------------------------------------------------------

// A scene's storage; field() views it (coph_field_view).
struct scene {
  std::vector<coph_vec2> centers;
  std::vector<double> weights;
  std::vector<coph_shape> obstacles;
  double width = 0.8;
  explicit scene(PyObject *d) {
    if (!d || !PyDict_Check(d))
      throw std::invalid_argument("the scene is not a dict");
    each(item(d, "centers"), "scene['centers']", [&](PyObject *c, int i) {
      centers.push_back(vec2(c, "scene['centers'][" + std::to_string(i) + "]"));
    });
    weights = floats(item(d, "weights"), "scene['weights']");
    if (weights.size() != centers.size())
      throw std::invalid_argument("the scene needs one weight per centre");
    obstacles = shapes(item(d, "obstacles"), "scene['obstacles']");
    if (PyObject *w = item(d, "width"))
      width = number(w, "scene['width']");
  }
  cvc::nav::coph_field_view field() const {
    cvc::nav::coph_field_view f;
    f.centers = centers;
    f.weights = weights;
    f.width = width;
    f.obstacles = obstacles;
    return f;
  }
};

// None (legacy shaping), or nav_coph_shaping()'s dict.
std::optional<cvc::nav::coph_shaping> shaping(PyObject *o) {
  if (!o || o == Py_None)
    return std::nullopt;
  cvc::nav::coph_shaping s;
  s.k = number(item(o, "k"), "shaping['k']");
  s.ell = number(item(o, "ell"), "shaping['ell']");
  s.eta = number(item(o, "eta"), "shaping['eta']");
  s.alpha = number(item(o, "alpha"), "shaping['alpha']");
  s.bound = number(item(o, "bound"), "shaping['bound']");
  return s;
}

// {"time", "readings": [{"uid", "time", "q", "value"}, ...], "sensor_center", "sensor_radius"}.
cvc::nav::coph_observation observation(PyObject *d) {
  cvc::nav::coph_observation o;
  if (!d || d == Py_None)
    return o;
  if (PyObject *t = item(d, "time"))
    o.time = number(t, "observation['time']");
  each(item(d, "readings"), "observation['readings']", [&](PyObject *r, int i) {
    const std::string w = "observation['readings'][" + std::to_string(i) + "]";
    cvc::nav::coph_reading rd;
    rd.uid = text(item(r, "uid"), w + "['uid']");
    rd.time = number(item(r, "time"), w + "['time']");
    rd.q = vec2(item(r, "q"), w + "['q']");
    rd.value = number(item(r, "value"), w + "['value']");
    o.readings.push_back(rd);
  });
  if (PyObject *c = item(d, "sensor_center"))
    o.sensor_center = vec2(c, "observation['sensor_center']");
  if (PyObject *r = item(d, "sensor_radius"))
    o.sensor_radius = number(r, "observation['sensor_radius']");
  return o;
}

// {"uid", "time", "x", "u", "after", "version"}.
cvc::nav::coph_transition transition(PyObject *d) {
  cvc::nav::coph_transition t;
  t.uid = integer(item(d, "uid"), "transition['uid']");
  t.time = number(item(d, "time"), "transition['time']");
  t.x = state4(item(d, "x"), "transition['x']");
  t.u = vec2(item(d, "u"), "transition['u']");
  t.after = state4(item(d, "after"), "transition['after']");
  t.version = integer(item(d, "version"), "transition['version']");
  return t;
}

PyObject *control_meta(const cvc::nav::coph_control_meta &m) {
  PyObject *d = PyDict_New();
  put(d, "reason", PyUnicode_FromString(m.reason.c_str()));
  put(d, "route_reason", PyUnicode_FromString(m.route_reason.c_str()));
  put(d, "guard_steps", PyLong_FromLong(m.guard_steps));
  put(d, "projection_candidates", PyLong_FromLong(m.projection_candidates));
  PyObject *nodes = PyList_New(static_cast<Py_ssize_t>(m.route_nodes.size()));
  for (std::size_t i = 0; i < m.route_nodes.size(); ++i)
    PyList_SET_ITEM(nodes, static_cast<Py_ssize_t>(i), PyLong_FromLong(m.route_nodes[i]));
  put(d, "route_nodes", nodes);
  put(d, "waypoint", list(m.waypoint));
  put(d, "applied", list(m.applied));
  put(d, "proposed", list(m.proposed));
  put(d, "infeasible", PyBool_FromLong(m.infeasible));
  put(d, "route_changed", PyBool_FromLong(m.route_changed));
  put(d, "route_expansions", PyLong_FromLong(m.route_expansions));
  PyObject *route = PyList_New(static_cast<Py_ssize_t>(m.route.size()));
  for (std::size_t i = 0; i < m.route.size(); ++i)
    PyList_SET_ITEM(route, static_cast<Py_ssize_t>(i), list(m.route[i]));
  put(d, "route", route);
  put(d, "stagnation", PyBool_FromLong(m.stagnation));
  return d;
}

PyObject *energy(const cvc::nav::coph_energy &e) {
  return Py_BuildValue("{s:d,s:d,s:d,s:d,s:d,s:d,s:L}", "work", e.work, "heat", e.heat, "kinetic_change",
                       e.kinetic_change, "residual", e.residual, "shaping_jumps", e.shaping_jumps,
                       "unallocated_work", e.unallocated_work, "steps", static_cast<long long>(e.steps));
}

PyObject *update_report(const cvc::nav::coph_update_report &r) {
  PyObject *d = PyDict_New();
  put(d, "accepted", PyBool_FromLong(r.accepted));
  put(d, "reason", PyUnicode_FromString(r.reason.c_str()));
  put(d, "finite", PyBool_FromLong(r.finite));
  put(d, "feasible", PyBool_FromLong(r.feasible));
  put(d, "valid", PyBool_FromLong(r.valid));
  put(d, "exhausted", PyBool_FromLong(r.exhausted));
  put(d, "attempts", PyLong_FromSize_t(r.attempts.size()));
  put(d, "version_after", PyLong_FromLongLong(r.version_after));
  put(d, "policy_after", list(r.policy_after));
  put(d, "damping_after", PyFloat_FromDouble(r.damping_after));
  put(d, "damping_proposal", PyFloat_FromDouble(r.damping_proposal));
  put(d, "shaping_jump", PyFloat_FromDouble(r.shaping_jump));
  put(d, "field_validation_before", PyFloat_FromDouble(r.field_validation_before));
  put(d, "field_validation_after", PyFloat_FromDouble(r.field_validation_after));
  put(d, "plant_validation_before", PyFloat_FromDouble(r.plant_validation_before));
  put(d, "plant_validation_after", PyFloat_FromDouble(r.plant_validation_after));
  put(d, "gradient", opt(r.gradient));
  put(d, "bounded", opt(r.bounded));
  put(d, "policy_independent", PyBool_FromLong(r.policy_independent));
  PyObject *msgs = PyList_New(static_cast<Py_ssize_t>(r.messages.size()));
  for (std::size_t i = 0; i < r.messages.size(); ++i)
    PyList_SET_ITEM(msgs, static_cast<Py_ssize_t>(i), PyUnicode_FromString(r.messages[i].c_str()));
  put(d, "messages", msgs);
  return d;
}

// The fleet's controls: [{"tick", "u", "meta": control_meta's dict}, ...]
PyObject *controls(const std::vector<cvc::nav::coph_fleet::control> &v) {
  PyObject *l = PyList_New(static_cast<Py_ssize_t>(v.size()));
  for (std::size_t i = 0; i < v.size(); ++i) {
    PyObject *d = PyDict_New();
    put(d, "tick", PyLong_FromLongLong(v[i].tick));
    put(d, "u", list(v[i].u));
    put(d, "meta", control_meta(v[i].meta));
    PyList_SET_ITEM(l, static_cast<Py_ssize_t>(i), d);
  }
  return l;
}

// The fleet's events: [{"kind", "sender", "receiver", "packet", "reason", "bytes",
// "version_after", "accepted", "time"}, ...]
PyObject *events(const std::vector<cvc::nav::coph_fleet::event> &v) {
  PyObject *l = PyList_New(static_cast<Py_ssize_t>(v.size()));
  for (std::size_t i = 0; i < v.size(); ++i) {
    const auto &e = v[i];
    PyObject *d = PyDict_New();
    put(d, "kind", PyUnicode_FromString(e.kind.c_str()));
    put(d, "sender", PyLong_FromLong(e.sender));
    put(d, "receiver", PyLong_FromLong(e.receiver));
    put(d, "packet", PyUnicode_FromString(e.packet.c_str()));
    put(d, "reason", PyUnicode_FromString(e.reason.c_str()));
    put(d, "bytes", PyLong_FromSize_t(e.bytes));
    if (e.version_after)
      put(d, "version_after", PyLong_FromLongLong(*e.version_after));
    else
      PyDict_SetItemString(d, "version_after", Py_None);
    if (e.accepted)
      put(d, "accepted", PyBool_FromLong(*e.accepted));
    else
      PyDict_SetItemString(d, "accepted", Py_None);
    put(d, "time", PyFloat_FromDouble(e.time));
    PyList_SET_ITEM(l, static_cast<Py_ssize_t>(i), d);
  }
  return l;
}

PyObject *packet_event(const cvc::nav::coph_packet_event &e) {
  return Py_BuildValue("{s:s,s:s,s:s,s:d,s:s,s:s,s:L}", "packet", e.packet.c_str(), "sender",
                       e.sender.c_str(), "recipient", e.recipient.c_str(), "time", e.time, "reason",
                       e.reason.c_str(), "scope", e.scope.c_str(), "version_before",
                       static_cast<long long>(e.version_before));
}

} // namespace coph
} // namespace pycvc
%}

%feature("kwargs") pycvc::nav_coph_potential;
%feature("kwargs") pycvc::nav_coph_grad_potential;
%feature("kwargs") pycvc::nav_coph_command;
%feature("kwargs") pycvc::nav_coph_allocate;
// A packet comes from NavCophAgent.export_to; its C++ side stays C++'s.
%ignore pycvc::NavCophPacket::NavCophPacket;
%ignore pycvc::NavCophPacket::packet;
%feature("kwargs") pycvc::NavCophAgent::NavCophAgent;
%feature("kwargs") pycvc::NavCophFleet::NavCophFleet;
%feature("kwargs") pycvc::NavCophDriver::NavCophDriver;
%newobject pycvc::NavCophAgent::export_to;

%inline %{
namespace pycvc {

// ── The kernels (coph_model.h, coph_allocation.h): GRL-SNAM's G2, G3 and G11 ─────────────────

// The demos' profile and communication, as dicts (the defaults the dicts below fill in from).
PyObject *nav_coph_profile() { return coph::config_dict(cvc::nav::coph_profile()); }
PyObject *nav_coph_communication_profile() {
  return coph::communication_dict(cvc::nav::coph_communication_profile());
}
// A configuration dict over the profile, validated and complete.
PyObject *nav_coph_config(PyObject *overrides) { return coph::config_dict(coph::config(overrides)); }

// model.flow: the state after dt under the force u held, at damping d.
PyObject *nav_coph_flow(PyObject *x, PyObject *u, double damping, double dt) {
  return coph::list(cvc::nav::coph_flow<double>(coph::state4(x, "x"), coph::vec2(u, "u"), damping, dt));
}

// model.balance over the same step: {"work", "heat", "kinetic_change", "residual"}.
PyObject *nav_coph_balance(PyObject *x, PyObject *u, double damping, double dt) {
  const cvc::nav::coph_balance_terms b =
      cvc::nav::coph_balance(coph::state4(x, "x"), coph::vec2(u, "u"), damping, dt);
  return Py_BuildValue("{s:d,s:d,s:d,s:d}", "work", b.work, "heat", b.heat, "kinetic_change",
                       b.kinetic_change, "residual", b.residual);
}

// Progress shaping's parameters for the scene under `config` (its shaping_mode, policy_bounds,
// shaping_k, shaping_ell, shaping_eta; over the profile, whose mode is progress): {"k", "ell",
// "eta", "alpha", "bound"}, or None under legacy shaping.
PyObject *nav_coph_shaping(PyObject *scene, PyObject *config) {
  const coph::scene s(scene);
  const std::optional<cvc::nav::coph_shaping> p = cvc::nav::coph_shaping_for(s.field(), coph::config(config));
  if (!p)
    Py_RETURN_NONE;
  return Py_BuildValue("{s:d,s:d,s:d,s:d,s:d}", "k", p->k, "ell", p->ell, "eta", p->eta, "alpha", p->alpha,
                       "bound", p->bound);
}

// model.risk at q.
double nav_coph_risk(PyObject *q, PyObject *scene) {
  const coph::scene s(scene);
  return cvc::nav::coph_risk<double>(coph::vec2(q, "q"), s.field());
}

// model.potential at q; shaping is nav_coph_shaping()'s (None: legacy).
double nav_coph_potential(PyObject *q, PyObject *target, PyObject *policy, PyObject *scene,
                          PyObject *shaping = nullptr) {
  const coph::scene s(scene);
  return cvc::nav::coph_potential<double>(coph::vec2(q, "q"), coph::vec2(target, "target"),
                                          coph::vec2(policy, "policy"), s.field(), coph::shaping(shaping));
}

// dV/dq at q.
PyObject *nav_coph_grad_potential(PyObject *q, PyObject *target, PyObject *policy, PyObject *scene,
                                  PyObject *shaping = nullptr) {
  const coph::scene s(scene);
  return coph::list(cvc::nav::coph_grad_potential<double>(coph::vec2(q, "q"), coph::vec2(target, "target"),
                                                          coph::vec2(policy, "policy"), s.field(),
                                                          coph::shaping(shaping)));
}

// model.command at x = (q, v): {"applied", "proposed"}.
PyObject *nav_coph_command(PyObject *x, PyObject *target, PyObject *policy, PyObject *scene,
                           PyObject *shaping = nullptr, double limit = 3.0) {
  const coph::scene s(scene);
  const cvc::nav::coph_command_of<double> c = cvc::nav::coph_command<double>(
      coph::state4(x, "x"), coph::vec2(target, "target"), coph::vec2(policy, "policy"), s.field(),
      coph::shaping(shaping), limit);
  PyObject *d = PyDict_New();
  coph::put(d, "applied", coph::list(c.applied));
  coph::put(d, "proposed", coph::list(c.proposed));
  return d;
}

// allocation.allocate: CoPH's force u on an Ackermann car (§3.3.2). {"accel", "steer", "kappa",
// "used", "unallocated"}.
PyObject *nav_coph_allocate(PyObject *u, double heading, double speed, double wheelbase, double steer_max,
                            double accel_max, double speed_floor = cvc::nav::coph_speed_floor) {
  const cvc::nav::coph_allocation a = cvc::nav::coph_allocate(coph::vec2(u, "u"), heading, speed, wheelbase,
                                                              steer_max, accel_max, speed_floor);
  PyObject *d = PyDict_New();
  coph::put(d, "accel", PyFloat_FromDouble(a.accel));
  coph::put(d, "steer", PyFloat_FromDouble(a.steer));
  coph::put(d, "kappa", PyFloat_FromDouble(a.kappa));
  coph::put(d, "used", coph::list(a.used));
  coph::put(d, "unallocated", coph::list(a.unallocated));
  return d;
}

// ── A packet between learners (coph_packet.h) ───────────────────────────────────────────────

class NavCophPacket {
public:
  explicit NavCophPacket(cvc::nav::coph_packet p) : p_(std::move(p)) {}
  // The compact, key-sorted JSON the Python counts, and its length in bytes (D9).
  std::string json() const { return p_.json(); }
  std::size_t bytes() const { return p_.bytes(); }
  std::string identity() const { return p_.identity; }
  std::string sender() const { return p_.sender; }
  std::string recipient() const { return p_.recipient; }
  long long version() const { return p_.version; }
  double timestamp() const { return p_.timestamp; }
  std::size_t anchors() const { return p_.anchors.size(); }
  const cvc::nav::coph_packet &packet() const { return p_; }

private:
  cvc::nav::coph_packet p_;
};

// ── One learner (coph_agent.h): the demos' CrossingLearner, or a car's without a network ──────

class NavCophAgent {
public:
  // A fresh learner at initial (x, y, vx, vy) heading for goal; network "crossing" or "none" (a
  // car's). seed seeds export_to's bootstrap draws (std::mt19937, D8).
  NavCophAgent(PyObject *config, PyObject *communication, PyObject *initial, PyObject *goal,
               const std::string &agent_id = "0", PyObject *network = nullptr, unsigned seed = 0)
      : a_(coph::config(config), coph::communication(communication), coph::state4(initial, "initial"),
           coph::vec2(goal, "goal"), agent_id, coph::network(network)),
        rng_(seed) {}

  std::string id() const { return a_.id(); }
  void set_goal(PyObject *goal) { a_.set_goal(coph::vec2(goal, "goal")); }

  // observation: {"time", "readings": [{"uid", "time", "q", "value"}, ...], "sensor_center",
  // "sensor_radius"}; obstacles: the shapes the sensor sees; transition: {"uid", "time", "x", "u",
  // "after", "version"}, the step that led here, or None.
  void observe(PyObject *observation, PyObject *obstacles = nullptr, PyObject *transition = nullptr) {
    const cvc::nav::coph_observation o = coph::observation(observation);
    const std::vector<cvc::nav::coph_shape> s = coph::shapes(obstacles, "obstacles");
    if (transition && transition != Py_None) {
      const cvc::nav::coph_transition t = coph::transition(transition);
      a_.observe(o, s, &t);
    } else {
      a_.observe(o, s);
    }
  }
  // Whether a new stage began at x.
  bool advance(PyObject *x) { return a_.advance(coph::state4(x, "x")); }
  // The force to hold until the next step; last_control() is its record.
  PyObject *action(PyObject *x) { return coph::list(a_.action(coph::state4(x, "x"))); }
  PyObject *last_control() const { return coph::control_meta(a_.last_control()); }
  // The update at x: whether it committed, why, the new version, policy and damping, and the
  // validation and shaping jump.
  PyObject *update(PyObject *x) { return coph::update_report(a_.update(coph::state4(x, "x"))); }
  // The packet for recipient, or None (the bootstrap indices from this agent's seeded draws).
  NavCophPacket *export_to(PyObject *x, const std::string &recipient) {
    std::optional<cvc::nav::coph_packet> p = a_.export_to(coph::state4(x, "x"), recipient, rng_);
    return p ? new NavCophPacket(std::move(*p)) : nullptr;
  }
  // CrossingLearner.assimilate: {"packet", "sender", "recipient", "time", "reason", "scope",
  // "version_before"}.
  PyObject *assimilate(const NavCophPacket &p) { return coph::packet_event(a_.assimilate(p.packet())); }

  // The committed values: {"version", "damping", "policy", "goal", "now", "stages" (their ids),
  // "geometry" (the store's shapes), "received" (the packets held)}.
  PyObject *state() const {
    const cvc::nav::coph_learner_state &s = a_.state();
    PyObject *d = PyDict_New();
    coph::put(d, "version", PyLong_FromLongLong(s.version));
    coph::put(d, "damping", PyFloat_FromDouble(s.damping));
    coph::put(d, "policy", coph::list(s.policy));
    coph::put(d, "goal", coph::list(s.goal));
    coph::put(d, "now", PyFloat_FromDouble(s.now));
    PyObject *stages = PyList_New(static_cast<Py_ssize_t>(s.stages.size()));
    for (std::size_t i = 0; i < s.stages.size(); ++i)
      PyList_SET_ITEM(stages, static_cast<Py_ssize_t>(i), PyLong_FromLongLong(s.stages[i].id));
    coph::put(d, "stages", stages);
    PyObject *geometry = PyList_New(static_cast<Py_ssize_t>(s.geometry.size()));
    for (std::size_t i = 0; i < s.geometry.size(); ++i)
      PyList_SET_ITEM(geometry, static_cast<Py_ssize_t>(i), coph::shape_list(s.geometry[i]));
    coph::put(d, "geometry", geometry);
    coph::put(d, "received", PyLong_FromSize_t(s.exchange.received.size()));
    return d;
  }

private:
  cvc::nav::coph_agent a_;
  std::mt19937 rng_;
};

// ── The runner's fleet (coph_fleet.h) ───────────────────────────────────────────────────────

class NavCophFleet {
public:
  // agents: [{"initial": (x, y, vx, vy), "goal": (x, y)}, ...], agent k named '<k>'; network
  // "crossing" or "none"; senders: the agents that export (None: none); seed: the fleet's own
  // draws; delay_s: the ideal channel's latency.
  NavCophFleet(PyObject *config, PyObject *communication, PyObject *agents, PyObject *network = nullptr,
               PyObject *senders = nullptr, unsigned seed = 0, double communication_start_s = 0.0,
               double delay_s = 0.0) {
    cvc::nav::coph_fleet_config fc;
    fc.cfg = coph::config(config);
    fc.comm = coph::communication(communication);
    fc.network = coph::network(network);
    for (double s : coph::floats(senders, "senders"))
      fc.senders.push_back(static_cast<int>(s));
    fc.seed = seed;
    fc.communication_start_s = communication_start_s;
    std::vector<cvc::nav::coph_fleet::agent_spec> specs;
    coph::each(agents, "agents", [&](PyObject *a, int i) {
      const std::string w = "agents[" + std::to_string(i) + "]";
      specs.push_back({coph::state4(coph::item(a, "initial"), w + "['initial']"),
                       coph::vec2(coph::item(a, "goal"), w + "['goal']")});
    });
    cvc::nav::coph_ideal_channel_config cc;
    cc.delay_s = delay_s;
    f_ = std::make_unique<cvc::nav::coph_fleet>(fc, specs, std::make_shared<cvc::nav::coph_ideal_channel>(cc));
    n_ = static_cast<int>(specs.size());
  }

  int size() const { return n_; }

  // Run from tick 0 until every agent is done, on the crossing world: world is {"damping": 0.85,
  // "obstacles": [shape, ...] (the doors), "network": "crossing" | "none", "seeds": [...],
  // "base_risk": 0.02, "patches": [(x, y, sx, sy, amplitude), ...], "noise": f(agent, tick, j)},
  // all optional. The GIL is released while it runs; noise takes it back for each call.
  void run_crossing(PyObject *world = nullptr) {
    cvc::nav::coph_crossing_world_config wc;
    wc.cfg = f_->agents().empty() ? cvc::nav::coph_profile() : f_->agents()[0].state().cfg;
    if (PyObject *d = coph::item(world, "damping"))
      wc.damping = coph::number(d, "world['damping']");
    wc.obstacles = coph::shapes(coph::item(world, "obstacles"), "world['obstacles']");
    wc.network = coph::network(coph::item(world, "network"));
    for (double s : coph::floats(coph::item(world, "seeds"), "world['seeds']"))
      wc.seeds.push_back(static_cast<long long>(s));
    if (PyObject *b = coph::item(world, "base_risk"))
      wc.base_risk = coph::number(b, "world['base_risk']");
    coph::each(coph::item(world, "patches"), "world['patches']", [&](PyObject *p, int i) {
      const std::vector<double> v = coph::floats(p, "world['patches'][" + std::to_string(i) + "]");
      if (v.size() != 5)
        throw std::invalid_argument("a patch is (x, y, sx, sy, amplitude)");
      wc.patches.push_back({v[0], v[1], v[2], v[3], v[4]});
    });
    if (PyObject *noise = coph::item(world, "noise")) {
      if (!PyCallable_Check(noise))
        throw std::invalid_argument("world['noise'] is not callable");
      Py_INCREF(noise);
      const std::shared_ptr<PyObject> f(noise, [](PyObject *o) {
        const PyGILState_STATE g = PyGILState_Ensure();
        Py_DECREF(o);
        PyGILState_Release(g);
      });
      wc.noise = [f](int agent, long long tick, int j) -> double {
        const PyGILState_STATE g = PyGILState_Ensure();
        PyObject *r = PyObject_CallFunction(f.get(), "iLi", agent, tick, j);
        const double v = r ? PyFloat_AsDouble(r) : -1.0;
        const bool failed = !r || (v == -1.0 && PyErr_Occurred());
        Py_XDECREF(r);
        if (failed)
          PyErr_Clear();
        PyGILState_Release(g);
        if (failed)
          throw std::runtime_error("world['noise'] raised or did not return a number");
        return v;
      };
    }
    const cvc::nav::coph_crossing_world w(std::move(wc));
    std::string err;
    Py_BEGIN_ALLOW_THREADS
    try {
      f_->run(w);
    } catch (const std::exception &e) {
      err = e.what();
      if (err.empty())
        err = "NavCophFleet.run_crossing failed";
    }
    Py_END_ALLOW_THREADS
    if (!err.empty())
      throw std::runtime_error(err);
  }

  // One boundary at tick, time now: one input per agent, {"x", "hit", "observation", "obstacles",
  // "u_used", "arrived", "goal"} (see NavCophAgent.observe; the last three optional, a vehicle's).
  // Returns each agent's held force, None for an agent that is done.
  PyObject *boundary(long long tick, double now, PyObject *inputs) {
    std::vector<cvc::nav::coph_fleet::input> in;
    coph::each(inputs, "inputs", [&](PyObject *d, int i) {
      const std::string w = "inputs[" + std::to_string(i) + "]";
      cvc::nav::coph_fleet::input x;
      x.x = coph::state4(coph::item(d, "x"), w + "['x']");
      if (PyObject *h = coph::item(d, "hit"))
        x.hit = PyObject_IsTrue(h) == 1;
      x.observation = coph::observation(coph::item(d, "observation"));
      x.obstacles = coph::shapes(coph::item(d, "obstacles"), w + "['obstacles']");
      if (PyObject *u = coph::item(d, "u_used"))
        x.u_used = coph::vec2(u, w + "['u_used']");
      if (PyObject *a = coph::item(d, "arrived"))
        x.arrived = PyObject_IsTrue(a) == 1;
      if (PyObject *g = coph::item(d, "goal"))
        x.goal = coph::vec2(g, w + "['goal']");
      in.push_back(std::move(x));
    });
    if (static_cast<int>(in.size()) != n_)
      throw std::invalid_argument("boundary needs one input per agent");
    const std::vector<std::optional<cvc::nav::coph_vec2>> held = f_->boundary(tick, now, in);
    PyObject *l = PyList_New(static_cast<Py_ssize_t>(held.size()));
    for (std::size_t i = 0; i < held.size(); ++i)
      PyList_SET_ITEM(l, static_cast<Py_ssize_t>(i), coph::opt(held[i]));
    return l;
  }

  bool done(int agent) const { return f_->done(check(agent)); }
  // An agent done at its goal runs again from the next boundary, keeping what it learned and
  // taking that input's "goal" (coph_fleet::resume). False when it is running or ended otherwise.
  bool resume(int agent) { return f_->resume(check(agent)); }
  bool success(int agent) const { return f_->success(check(agent)); }
  bool hit(int agent) const { return f_->hit(check(agent)); }
  PyObject *state(int agent) const { return coph::list(f_->state(check(agent))); }
  // {"work", "heat", "kinetic_change", "residual", "shaping_jumps", "unallocated_work", "steps"}.
  PyObject *energy(int agent) const { return coph::energy(f_->energy(check(agent))); }
  // The agent's learner's committed values (NavCophAgent.state()'s keys).
  PyObject *agent_state(int agent) const {
    const cvc::nav::coph_learner_state &s = f_->agents()[check(agent)].state();
    return Py_BuildValue("{s:L,s:d,s:(dd),s:(dd)}", "version", static_cast<long long>(s.version), "damping",
                         s.damping, "policy", s.policy[0], s.policy[1], "goal", s.goal[0], s.goal[1]);
  }

  // [{"time", "x", "stage", "version", "done"}, ...]
  PyObject *samples(int agent) const {
    const auto &v = f_->samples(check(agent));
    PyObject *l = PyList_New(static_cast<Py_ssize_t>(v.size()));
    for (std::size_t i = 0; i < v.size(); ++i) {
      PyObject *d = PyDict_New();
      coph::put(d, "time", PyFloat_FromDouble(v[i].time));
      coph::put(d, "x", coph::list(v[i].x));
      coph::put(d, "stage", PyLong_FromLongLong(v[i].stage));
      coph::put(d, "version", PyLong_FromLongLong(v[i].version));
      coph::put(d, "done", PyBool_FromLong(v[i].done));
      PyList_SET_ITEM(l, static_cast<Py_ssize_t>(i), d);
    }
    return l;
  }

  // [{"tick", "u", "meta": NavCophAgent.last_control()'s dict}, ...]
  PyObject *controls(int agent) const { return coph::controls(f_->controls(check(agent))); }

  // [{"tick", "packets", "report": NavCophAgent.update()'s dict}, ...]
  PyObject *updates(int agent) const {
    const auto &v = f_->updates(check(agent));
    PyObject *l = PyList_New(static_cast<Py_ssize_t>(v.size()));
    for (std::size_t i = 0; i < v.size(); ++i) {
      PyObject *d = PyDict_New();
      coph::put(d, "tick", PyLong_FromLongLong(v[i].tick));
      PyObject *p = PyList_New(static_cast<Py_ssize_t>(v[i].packets.size()));
      for (std::size_t j = 0; j < v[i].packets.size(); ++j)
        PyList_SET_ITEM(p, static_cast<Py_ssize_t>(j), PyUnicode_FromString(v[i].packets[j].c_str()));
      coph::put(d, "packets", p);
      coph::put(d, "report", coph::update_report(v[i].report));
      PyList_SET_ITEM(l, static_cast<Py_ssize_t>(i), d);
    }
    return l;
  }

  // [{"kind", "sender", "receiver", "packet", "reason", "bytes", "version_after", "accepted",
  // "time"}, ...]: activation, sent, budget_skip or delivered.
  PyObject *events() const { return coph::events(f_->events()); }

private:
  int check(int agent) const {
    if (agent < 0 || agent >= n_)
      throw std::out_of_range("NavCophFleet: no agent " + std::to_string(agent));
    return agent;
  }
  std::unique_ptr<cvc::nav::coph_fleet> f_;
  int n_ = 0;
};

// ── The driver: a fleet on a sim_world's vehicles (coph_driver.h, C5 part 2b) ─────────────────

class NavCophDriver {
public:
  // CoPH drives the vehicles of `sim` (a pycvc.nav_sim_world_create handle, kept alive while the
  // driver lives) that attach() names. config and communication are dicts over the demos'
  // profile; without a "steps" key the run's length is 1,000,000 CoPH steps. froude is s (D4):
  // CoPH's lengths are s world units. origin is CoPH's origin in world units (None: the map's
  // centre), local_goal the sliding goal's distance (D12), base_risk the risk where the sim has no
  // material layer, seed the readings' uid prefix and the fleet's draws, senders the attached
  // vehicles (by attachment order) that export (None: all).
  NavCophDriver(PyObject *sim, PyObject *config = nullptr, PyObject *communication = nullptr,
                double froude = 1.0, PyObject *origin = nullptr, double local_goal = 30.0,
                double speed_floor = cvc::nav::coph_speed_floor, double base_risk = 0.02,
                unsigned seed = 0, PyObject *senders = nullptr) {
    cvc::nav::sim_world *sw = pycvc::sim_world_from(sim);
    cvc::nav::coph_driver_config dc;
    dc.cfg = coph::config(config);
    if (!coph::item(config, "steps")) {
      dc.cfg.steps = 1000000;
      dc.cfg.validate();
    }
    dc.comm = coph::communication(communication);
    for (double v : coph::floats(senders, "senders"))
      dc.senders.push_back(static_cast<int>(v));
    dc.froude = froude;
    if (origin && origin != Py_None)
      dc.origin = coph::vec2(origin, "origin");
    dc.local_goal = local_goal;
    dc.speed_floor = speed_floor;
    dc.base_risk = base_risk;
    dc.seed = seed;
    d_ = std::make_unique<cvc::nav::coph_driver>(*sw, std::move(dc));
    Py_INCREF(sim);
    sim_ = sim;
  }
  ~NavCophDriver() {
    d_.reset(); // it refers to the sim_world; the handle goes after it
    Py_XDECREF(sim_);
  }
  NavCophDriver(const NavCophDriver &) = delete;
  NavCophDriver &operator=(const NavCophDriver &) = delete;

  // Agent i drives by CoPH from the next apply() on: before the first apply(), each once.
  void attach(int i) { d_->attach(i); }
  PyObject *attached() const {
    const std::vector<int> &a = d_->attached();
    PyObject *l = PyList_New(static_cast<Py_ssize_t>(a.size()));
    for (std::size_t i = 0; i < a.size(); ++i)
      PyList_SET_ITEM(l, static_cast<Py_ssize_t>(i), PyLong_FromLong(a[i]));
    return l;
  }
  // Before each nav_sim_world_step: a CoPH step every ticks_per_step() ticks, and each attached
  // vehicle's command every tick (sim_world::override_command).
  void apply() { d_->apply(); }
  int ticks_per_step() const { return d_->ticks_per_step(); }
  long long steps() const { return d_->steps(); }
  // Whether the vehicle (its attachment index) reached its goal; it then brakes and stays.
  bool done(int vehicle) const { return d_->done(check(vehicle)); }
  // Its account in CoPH's frame: {"work", "heat", "kinetic_change", "residual", "shaping_jumps",
  // "unallocated_work", "steps", "risk_cost", "max_step_residual", "allocation_residual"}.
  PyObject *account(int vehicle) const {
    const cvc::nav::coph_driver::ledger &l = d_->account(check(vehicle));
    PyObject *d = coph::energy(l.energy);
    coph::put(d, "risk_cost", PyFloat_FromDouble(l.risk_cost));
    coph::put(d, "max_step_residual", PyFloat_FromDouble(l.max_step_residual));
    coph::put(d, "allocation_residual", PyFloat_FromDouble(l.allocation_residual()));
    return d;
  }
  // The fleet's controls for the vehicle and its events (NavCophFleet's), empty before the first
  // apply().
  PyObject *controls(int vehicle) const {
    check(vehicle);
    if (!d_->fleet())
      return PyList_New(0);
    return coph::controls(d_->fleet()->controls(vehicle));
  }
  PyObject *events() const {
    if (!d_->fleet())
      return PyList_New(0);
    return coph::events(d_->fleet()->events());
  }

private:
  int check(int vehicle) const {
    if (vehicle < 0 || vehicle >= static_cast<int>(d_->attached().size()))
      throw std::out_of_range("NavCophDriver: no attached vehicle " + std::to_string(vehicle));
    return vehicle;
  }
  std::unique_ptr<cvc::nav::coph_driver> d_;
  PyObject *sim_ = nullptr;
};

} // namespace pycvc
%}
