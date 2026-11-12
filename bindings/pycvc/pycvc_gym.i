// pycvc_gym.i — pycvc.WorldGym: a world document driven by reset, step and observe, headless
// (cvc::ariadne::WorldGym, inc/cvc/ariadne/world_gym.h; ARIADNE_WORLDS.md §14, L4).
//
//   gym = pycvc.WorldGym("world_lab.ari", count=4, max_steps=600)
//   obs = gym.reset(seed=7)                      # a dict of numpy arrays, one entry per agent
//   while True:
//       steer, accel = my_policy(obs)            # (n,) each; NaN leaves an agent to nav@1
//       obs, reward, terminated, truncated, info = gym.step(steer, accel)
//       if terminated or truncated:
//           break
//   gym.save_log("episode.csv")                  # or gym.log(): a dict of numpy arrays
//
// The observation: episode, step, time and range_m (numbers); id (uint64); x, y, z, yaw, speed,
// vx, vy, steer, goal_x, goal_y, goal_dist, reach, length, width, wheelbase, max_steer,
// max_accel, max_decel, max_speed (float64); reached, collided, hit_obstacle, hit_vehicle, out
// (bool);
// ranges (n, rays) float32
// and ray_angles (rays,). SI units throughout (m, rad, m/s, m/s^2).
//
// set_reward(fn): fn(before, steer, accel, after) -> (n,) rewards, the dicts and arrays above;
// None puts the progress reward back. It runs inside step(), on the calling thread.
//
// %include'd from pycvc.i after pycvc_ariadne.i. NOTE: a SWIG interface file — do NOT run
// clang-format.
%{
#include <cvc/ariadne/world_gym.h>
#include <cvc/nav/world_grid.h>
#include <cstring>

namespace pycvc {

static void gym_put(PyObject *d, const char *k, PyObject *v) {
  if (v == nullptr)
    throw std::runtime_error(std::string("pycvc.WorldGym: could not make '") + k + "'");
  PyDict_SetItemString(d, k, v);
  Py_DECREF(v);
}

static PyObject *gym_f64(const std::vector<double> &v) {
  npy_intp n = static_cast<npy_intp>(v.size());
  PyObject *a = PyArray_SimpleNew(1, &n, NPY_DOUBLE);
  if (a && n)
    std::memcpy(PyArray_DATA(reinterpret_cast<PyArrayObject *>(a)), v.data(), v.size() * sizeof(double));
  return a;
}

static PyObject *gym_u64(const std::vector<std::uint64_t> &v) {
  npy_intp n = static_cast<npy_intp>(v.size());
  PyObject *a = PyArray_SimpleNew(1, &n, NPY_UINT64);
  if (a && n)
    std::memcpy(PyArray_DATA(reinterpret_cast<PyArrayObject *>(a)), v.data(),
                v.size() * sizeof(std::uint64_t));
  return a;
}

static PyObject *gym_bool(const std::vector<std::uint8_t> &v) {
  npy_intp n = static_cast<npy_intp>(v.size());
  PyObject *a = PyArray_SimpleNew(1, &n, NPY_BOOL);
  if (a)
    for (npy_intp i = 0; i < n; ++i)
      static_cast<npy_bool *>(PyArray_DATA(reinterpret_cast<PyArrayObject *>(a)))[i] = v[i] ? 1 : 0;
  return a;
}

static PyObject *gym_observation_dict(const cvc::ariadne::gym_observation &o) {
  PyObject *d = PyDict_New();
  try {
    gym_put(d, "episode", PyLong_FromUnsignedLongLong(o.episode));
    gym_put(d, "step", PyLong_FromUnsignedLongLong(o.step));
    gym_put(d, "time", PyFloat_FromDouble(o.time));
    gym_put(d, "n", PyLong_FromSize_t(o.n));
    gym_put(d, "id", gym_u64(o.id));
    const std::pair<const char *, const std::vector<double> *> cols[] = {
        {"x", &o.x}, {"y", &o.y}, {"z", &o.z}, {"yaw", &o.yaw}, {"speed", &o.speed},
        {"vx", &o.vx}, {"vy", &o.vy}, {"steer", &o.steer}, {"goal_x", &o.goal_x},
        {"goal_y", &o.goal_y}, {"goal_dist", &o.goal_dist}, {"reach", &o.reach},
        {"length", &o.length}, {"width", &o.width}, {"wheelbase", &o.wheelbase},
        {"max_steer", &o.max_steer}, {"max_accel", &o.max_accel}, {"max_decel", &o.max_decel},
        {"max_speed", &o.max_speed}, {"ray_angles", &o.ray_angles}};
    for (const auto &c : cols)
      gym_put(d, c.first, gym_f64(*c.second));
    gym_put(d, "reached", gym_bool(o.reached));
    gym_put(d, "collided", gym_bool(o.collided));
    gym_put(d, "hit_obstacle", gym_bool(o.hit_obstacle));
    gym_put(d, "hit_vehicle", gym_bool(o.hit_vehicle));
    gym_put(d, "out", gym_bool(o.out));
    gym_put(d, "range_m", PyFloat_FromDouble(o.range_m));
    npy_intp dims[2] = {static_cast<npy_intp>(o.n), static_cast<npy_intp>(o.ray_angles.size())};
    PyObject *r = PyArray_SimpleNew(2, dims, NPY_FLOAT);
    if (r && !o.ranges.empty())
      std::memcpy(PyArray_DATA(reinterpret_cast<PyArrayObject *>(r)), o.ranges.data(),
                  o.ranges.size() * sizeof(float));
    gym_put(d, "ranges", r);
  } catch (...) {
    Py_DECREF(d);
    throw;
  }
  return d;
}

// (n,) doubles from anything numpy takes (a scalar is every agent's).
static std::vector<double> gym_doubles(PyObject *o, std::size_t n, const char *what) {
  PyArrayObject *a = reinterpret_cast<PyArrayObject *>(
      PyArray_FROMANY(o, NPY_DOUBLE, 0, 1, NPY_ARRAY_C_CONTIGUOUS | NPY_ARRAY_FORCECAST));
  if (a == nullptr) {
    PyErr_Clear();
    throw std::invalid_argument(std::string("pycvc.WorldGym.step: ") + what +
                                " must be a number or an (n,) array");
  }
  std::vector<double> out;
  const double *v = static_cast<const double *>(PyArray_DATA(a));
  if (PyArray_NDIM(a) == 0)
    out.assign(n, v[0]);
  else
    out.assign(v, v + PyArray_DIM(a, 0));
  Py_DECREF(a);
  return out;
}

// The Python exception set now, as the C++ error that carries it back out of step().
static python_exception gym_take_py_exception(const char *what) {
  PyObject *type = nullptr, *val = nullptr, *tb = nullptr;
  PyErr_Fetch(&type, &val, &tb);
  PyErr_NormalizeException(&type, &val, &tb);
  if (val != nullptr && tb != nullptr)
    PyException_SetTraceback(val, tb);
  std::string msg = "error", type_name;
  if (val != nullptr)
    if (PyObject *s = PyObject_Str(val)) {
      if (const char *cs = PyUnicode_AsUTF8(s))
        msg = cs;
      Py_DECREF(s);
    }
  if (type != nullptr)
    if (const char *tn = reinterpret_cast<PyTypeObject *>(type)->tp_name)
      type_name = tn;
  std::shared_ptr<PyObject> held = val != nullptr ? python_ref(val) : nullptr;
  PyErr_Clear();
  Py_XDECREF(type);
  Py_XDECREF(val);
  Py_XDECREF(tb);
  return python_exception(std::string(what) + ": " + msg, type_name, std::move(held));
}

static std::string gym_str(PyObject *dict, const char *key, const std::string &def = std::string()) {
  PyObject *v = PyDict_GetItemString(dict, key); // borrowed
  if (v == nullptr || v == Py_None)
    return def;
  PyObject *s = PyObject_Str(v);
  if (s == nullptr) {
    PyErr_Clear();
    return def;
  }
  const char *c = PyUnicode_AsUTF8(s);
  std::string out = c ? c : def;
  Py_DECREF(s);
  return out;
}

static bool gym_num(PyObject *dict, const char *key, double &out) {
  PyObject *v = PyDict_GetItemString(dict, key);
  if (v == nullptr || v == Py_None)
    return false;
  const double d = PyFloat_AsDouble(v);
  if (PyErr_Occurred()) {
    PyErr_Clear();
    throw std::invalid_argument(std::string("pycvc.WorldGym: agent '") + key + "' must be a number");
  }
  out = d;
  return true;
}

static void gym_rgb(PyObject *dict, const char *key, std::array<float, 3> &out) {
  PyObject *v = PyDict_GetItemString(dict, key);
  if (v == nullptr || v == Py_None)
    return;
  PyObject *seq = PySequence_Fast(v, "a colour is three numbers");
  if (seq == nullptr || PySequence_Fast_GET_SIZE(seq) != 3) {
    Py_XDECREF(seq);
    PyErr_Clear();
    throw std::invalid_argument(std::string("pycvc.WorldGym: agent '") + key +
                                "' must be three numbers in [0, 1]");
  }
  for (int i = 0; i < 3; ++i)
    out[i] = static_cast<float>(PyFloat_AsDouble(PySequence_Fast_GET_ITEM(seq, i)));
  Py_DECREF(seq);
  if (PyErr_Occurred()) {
    PyErr_Clear();
    throw std::invalid_argument(std::string("pycvc.WorldGym: agent '") + key + "' must be numbers");
  }
}

// An agent from a dict: archetype, name, team, controller, policy, policy_sha256, mass,
// max_speed, color, color2, metadata, and x, y (a start of its own) with yaw (rad).
static cvc::ariadne::entity_spec gym_agent(PyObject *a) {
  if (!PyDict_Check(a))
    throw std::invalid_argument("pycvc.WorldGym: each agent is a dict");
  cvc::ariadne::entity_spec s;
  s.archetype = gym_str(a, "archetype");
  s.name = gym_str(a, "name");
  s.controller = gym_str(a, "controller");
  s.policy = gym_str(a, "policy");
  s.policy_sha256 = gym_str(a, "policy_sha256");
  double v = 0.0;
  if (gym_num(a, "team", v))
    s.team = static_cast<int>(v);
  if (gym_num(a, "mass", v))
    s.mass = v;
  if (gym_num(a, "max_speed", v))
    s.max_speed = v;
  double x = 0.0, y = 0.0;
  if (gym_num(a, "x", x) && gym_num(a, "y", y)) {
    s.placed = true;
    s.x = x;
    s.y = y;
  }
  if (gym_num(a, "yaw", v))
    s.yaw = v;
  gym_rgb(a, "color", s.color);
  gym_rgb(a, "color2", s.color2);
  if (PyObject *m = PyDict_GetItemString(a, "metadata"); m != nullptr && PyDict_Check(m)) {
    PyObject *k = nullptr, *val = nullptr;
    Py_ssize_t pos = 0;
    while (PyDict_Next(m, &pos, &k, &val)) {
      PyObject *ks = PyObject_Str(k), *vs = PyObject_Str(val);
      if (ks && vs)
        s.metadata[PyUnicode_AsUTF8(ks)] = PyUnicode_AsUTF8(vs);
      Py_XDECREF(ks);
      Py_XDECREF(vs);
    }
    PyErr_Clear();
  }
  return s;
}

class WorldGym {
public:
  WorldGym(const std::string &document, const std::string &prefix, int count,
           const std::string &world, PyObject *agents, PyObject *set, bool wander,
           unsigned long long max_steps, int rays, double range_m, double ready_timeout) {
    cvc::ariadne::gym_options o;
    o.prefix = prefix;
    o.count = count;
    o.world = world;
    o.wander = wander;
    o.max_steps = max_steps;
    o.rays = rays;
    o.range_m = range_m;
    o.ready_timeout = ready_timeout;
    if (agents != nullptr && agents != Py_None) {
      PyObject *seq = PySequence_Fast(agents, "agents is a list of dicts");
      if (seq == nullptr) {
        PyErr_Clear();
        throw std::invalid_argument("pycvc.WorldGym: agents is a list of dicts");
      }
      for (Py_ssize_t i = 0; i < PySequence_Fast_GET_SIZE(seq); ++i) {
        try {
          o.agents.push_back(gym_agent(PySequence_Fast_GET_ITEM(seq, i)));
        } catch (...) {
          Py_DECREF(seq);
          throw;
        }
      }
      Py_DECREF(seq);
    }
    if (set != nullptr && set != Py_None) {
      if (!PyDict_Check(set))
        throw std::invalid_argument("pycvc.WorldGym: set is a dict of state keys and values");
      PyObject *k = nullptr, *v = nullptr;
      Py_ssize_t pos = 0;
      while (PyDict_Next(set, &pos, &k, &v)) {
        PyObject *ks = PyObject_Str(k), *vs = PyObject_Str(v);
        if (ks && vs)
          o.set[PyUnicode_AsUTF8(ks)] = PyUnicode_AsUTF8(vs);
        Py_XDECREF(ks);
        Py_XDECREF(vs);
      }
      PyErr_Clear();
    }
    gym_ = std::make_unique<cvc::ariadne::WorldGym>(document, std::move(o));
  }

  PyObject *reset(unsigned long long seed) {
    const cvc::ariadne::gym_observation *o = nullptr;
    {
      gil_release nogil;
      o = &gym_->reset(seed);
    }
    return gym_observation_dict(*o);
  }

  PyObject *step(PyObject *steer, PyObject *accel) {
    const std::size_t n = gym_->agents();
    const std::vector<double> s = gym_doubles(steer, n, "steer");
    const std::vector<double> a = gym_doubles(accel, n, "accel");
    cvc::ariadne::gym_step r;
    {
      gil_release nogil; // the reward retakes it
      r = gym_->step(s, a);
    }
    PyObject *info = PyDict_New();
    gym_put(info, "return", PyFloat_FromDouble(gym_->episode_return()));
    return Py_BuildValue("(NNOON)", gym_observation_dict(r.observation), gym_f64(r.reward),
                         r.terminated ? Py_True : Py_False, r.truncated ? Py_True : Py_False,
                         info);
  }

  PyObject *observe() const { return gym_observation_dict(gym_->observe()); }

  void set_reward(PyObject *fn) {
    if (fn == nullptr || fn == Py_None) {
      gym_->set_reward(nullptr);
      return;
    }
    if (!PyCallable_Check(fn))
      throw std::invalid_argument("pycvc.WorldGym.set_reward: expected a callable or None");
    std::shared_ptr<PyObject> held = python_ref(fn);
    gym_->set_reward([held](const cvc::ariadne::gym_observation &before,
                            const std::vector<double> &steer, const std::vector<double> &accel,
                            const cvc::ariadne::gym_observation &after) {
      PyGILState_STATE g = PyGILState_Ensure();
      std::vector<double> out;
      PyObject *b = nullptr, *s = nullptr, *a = nullptr, *f = nullptr, *res = nullptr;
      try {
        b = gym_observation_dict(before);
        f = gym_observation_dict(after);
        s = gym_f64(steer);
        a = gym_f64(accel);
        res = PyObject_CallFunctionObjArgs(held.get(), b, s, a, f, nullptr);
        if (res == nullptr)
          throw gym_take_py_exception("pycvc.WorldGym: the reward");
        out = gym_doubles(res, after.n, "the reward");
      } catch (...) {
        Py_XDECREF(b);
        Py_XDECREF(s);
        Py_XDECREF(a);
        Py_XDECREF(f);
        Py_XDECREF(res);
        PyGILState_Release(g);
        throw;
      }
      Py_XDECREF(b);
      Py_XDECREF(s);
      Py_XDECREF(a);
      Py_XDECREF(f);
      Py_XDECREF(res);
      PyGILState_Release(g);
      return out;
    });
  }

  PyObject *log() const {
    const cvc::ariadne::gym_log &l = gym_->log();
    PyObject *d = PyDict_New();
    try {
      gym_put(d, "episode", gym_u64(l.episode));
      gym_put(d, "step", gym_u64(l.step));
      gym_put(d, "id", gym_u64(l.id));
      const std::pair<const char *, const std::vector<double> *> cols[] = {
          {"time", &l.time}, {"x", &l.x}, {"y", &l.y}, {"yaw", &l.yaw}, {"speed", &l.speed},
          {"steer_cmd", &l.steer_cmd}, {"accel_cmd", &l.accel_cmd}, {"goal_dist", &l.goal_dist},
          {"reward", &l.reward}};
      for (const auto &c : cols)
        gym_put(d, c.first, gym_f64(*c.second));
      gym_put(d, "reached", gym_bool(l.reached));
      gym_put(d, "collided", gym_bool(l.collided));
    } catch (...) {
      Py_DECREF(d);
      throw;
    }
    return d;
  }
  void clear_log() { gym_->clear_log(); }
  bool save_log(const std::string &path) const { return gym_->save_log(path); }

  bool set_goal(unsigned long long id, double x, double y) {
    return gym_->set_goal(id, x, y);
  }
  // The navigator's grid of the world on: occupancy (rows, cols) uint8, 1 blocked, row 0 at
  // min_y; height (rows, cols) float32; bounds (min_x, min_y, max_x, max_y), cell centres on the
  // grid points. None before the world has one.
  PyObject *grid() const {
    std::shared_ptr<const cvc::nav::world_grid> g = gym_->vehicles().grid();
    if (!g)
      Py_RETURN_NONE;
    PyObject *d = PyDict_New();
    npy_intp dims[2] = {g->rows, g->cols};
    PyObject *occ = PyArray_SimpleNew(2, dims, NPY_UINT8);
    if (occ && !g->occupancy.empty())
      std::memcpy(PyArray_DATA(reinterpret_cast<PyArrayObject *>(occ)), g->occupancy.data(),
                  g->occupancy.size());
    gym_put(d, "occupancy", occ);
    PyObject *h = PyArray_ZEROS(2, dims, NPY_FLOAT, 0);
    if (h && g->height.size() == g->occupancy.size())
      std::memcpy(PyArray_DATA(reinterpret_cast<PyArrayObject *>(h)), g->height.data(),
                  g->height.size() * sizeof(float));
    gym_put(d, "height", h);
    gym_put(d, "bounds", Py_BuildValue("(dddd)", g->min_x, g->min_y, g->max_x, g->max_y));
    return d;
  }
  double dt() const { return gym_->dt(); }
  std::size_t agents() const { return gym_->agents(); }
  double episode_return() const { return gym_->episode_return(); }
  std::vector<std::string> take_warnings() { return gym_->take_warnings(); }
  // The gym's state tree, for its keys (<prefix>.gym.*, the document's own).
  std::string get(const std::string &path) {
    cvc::state *k = cvc::state::instance(gym_->app()).findDescendant(path);
    return k ? k->value() : std::string();
  }
  void set(const std::string &path, const std::string &value) {
    cvc::state::instance(gym_->app())(path).value(value);
  }

private:
  std::unique_ptr<cvc::ariadne::WorldGym> gym_;
};

} // namespace pycvc
%}

// reset() and step() release the GIL while the world runs (a reward of Python's retakes it), and
// so does the destructor (~AppRuntime ends the document's processes). The constructor reads its
// Python arguments, then loads the document and waits for its world with the GIL held.
PYCVC_ARIADNE_NOGIL(pycvc::WorldGym::~WorldGym)
%exception pycvc::WorldGym::WorldGym {
  try {
    $action
  }
  PYCVC_STATE_EXEC_CATCH
}
%feature("kwargs") pycvc::WorldGym::WorldGym;
%feature("kwargs") pycvc::WorldGym::reset;
%feature("kwargs") pycvc::WorldGym::step;
%feature("docstring") pycvc::WorldGym "A world document run headless as a gym: reset(seed), step(steer, accel), observe(), set_reward(fn), log(), save_log(path). See docs/ARIADNE.md, \"The gym\".";

namespace pycvc {
class WorldGym {
public:
  WorldGym(const std::string &document, const std::string &prefix = "", int count = 1,
           const std::string &world = "", PyObject *agents = nullptr, PyObject *set = nullptr,
           bool wander = false, unsigned long long max_steps = 1200, int rays = 16,
           double range_m = 60.0, double ready_timeout = 120.0);
  ~WorldGym();
  PyObject *reset(unsigned long long seed = 0);
  PyObject *step(PyObject *steer, PyObject *accel);
  PyObject *observe() const;
  void set_reward(PyObject *fn);
  PyObject *log() const;
  void clear_log();
  bool save_log(const std::string &path) const;
  bool set_goal(unsigned long long id, double x, double y);
  PyObject *grid() const;
  double dt() const;
  std::size_t agents() const;
  double episode_return() const;
  std::vector<std::string> take_warnings();
  std::string get(const std::string &path);
  void set(const std::string &path, const std::string &value);
};
} // namespace pycvc
