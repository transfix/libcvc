// pycvc_nav_lab.i — cvc::nav's arms, run log and panels (cvc/nav/nav_arms.h, run_log.h,
// nav_panels.h) and the plant ledger's items (cvc/nav/plant_ledger.h), for pycvc_gl.nav_lab and
// any Python host. docs/NAV_LAB.md has the model; the functions here are thin: the logic is the
// library's, so a Python host, a C++ host and the browser's show the same cells.
//
//   run = pycvc.nav_arms_create([w0, w1], ["default", {"name": "trained", "uri": p}])
//   for _ in range(600):
//       pycvc.nav_arms_step(run)
//   log = pycvc.nav_arms_log(run)                  # live: grows as the run steps
//   pycvc.nav_run_log_publish(app, prefix, log)    # the Stats and Ledger windows, at the last tick
//   pycvc.nav_arms_finish(run)
//   log = pycvc.nav_run_log_read("run.cvcrun")     # a recorded one
//   arrays = pycvc.nav_run_log_arrays(log)         # name -> (T, A, N) / (T, A) float64
//
// After pycvc_nav.i (the sim_world handles are its).

%{
#include <cvc/nav/nav_arms.h>
#include <cvc/nav/nav_panels.h>
#include <cvc/nav/plant_ledger.h>
#include <cvc/nav/run_log.h>

namespace pycvc_nav_lab {

inline PyObject *str(const std::string &s) {
  return PyUnicode_FromStringAndSize(s.data(), static_cast<Py_ssize_t>(s.size()));
}

// Set d[key] = v, stealing v. Throws (after releasing d) when v is null.
inline void put(PyObject *d, const char *key, PyObject *v) {
  if (!v) {
    Py_DECREF(d);
    throw std::runtime_error("pycvc.nav_lab: allocation failed");
  }
  PyDict_SetItemString(d, key, v);
  Py_DECREF(v);
}

inline std::string text_of(PyObject *o, const char *what) {
  if (!o || o == Py_None)
    return std::string();
  if (!PyUnicode_Check(o))
    throw std::invalid_argument(std::string("pycvc.nav_lab: ") + what + " is not a str");
  return PyUnicode_AsUTF8(o);
}

inline PyObject *doubles(const std::vector<double> &v, std::vector<npy_intp> dims) {
  PyObject *a = PyArray_SimpleNew(static_cast<int>(dims.size()), dims.data(), NPY_DOUBLE);
  if (!a)
    throw std::runtime_error("pycvc.nav_lab: array alloc failed");
  std::copy(v.begin(), v.end(),
            static_cast<double *>(PyArray_DATA(reinterpret_cast<PyArrayObject *>(a))));
  return a;
}

// Floats [n] from an array-like, or empty for None.
inline std::vector<float> floats(PyObject *o, int n, const char *what) {
  std::vector<float> out;
  if (!o || o == Py_None)
    return out;
  PyArrayObject *a = reinterpret_cast<PyArrayObject *>(
      PyArray_FROMANY(o, NPY_FLOAT, 0, 2, NPY_ARRAY_C_CONTIGUOUS | NPY_ARRAY_FORCECAST));
  if (!a)
    throw std::invalid_argument(std::string("pycvc.nav_lab: ") + what + " is not numbers");
  if (PyArray_SIZE(a) != n) {
    Py_DECREF(a);
    throw std::invalid_argument(std::string("pycvc.nav_lab: ") + what + " needs " +
                                std::to_string(n) + " numbers");
  }
  const float *p = static_cast<const float *>(PyArray_DATA(a));
  out.assign(p, p + n);
  Py_DECREF(a);
  return out;
}

inline PyObject *ledger_items(std::span<const cvc::nav::plant_ledger_item> items) {
  using K = cvc::nav::plant_ledger_item::kind;
  using S = cvc::nav::plant_ledger_item::source;
  PyObject *list = PyList_New(0);
  for (const cvc::nav::plant_ledger_item &it : items) {
    PyObject *d = PyDict_New();
    put(d, "key", PyUnicode_FromString(it.key));
    put(d, "unit", PyUnicode_FromString(it.unit));
    put(d, "comment", PyUnicode_FromString(it.comment));
    put(d, "kind", PyUnicode_FromString(it.type == K::measure ? "measure"
                                        : it.type == K::count ? "count"
                                                              : "flag"));
    put(d, "source", PyUnicode_FromString(it.from == S::energy ? "energy" : "plant"));
    put(d, "needs", PyLong_FromUnsignedLong(it.needs));
    PyList_Append(list, d);
    Py_DECREF(d);
  }
  return list;
}

// ── handles ──
constexpr const char *kArmsCapsule = "cvc.nav.arms_run";
constexpr const char *kRunLogCapsule = "cvc.nav.run_log";
constexpr const char *kReplayCapsule = "cvc.nav.replay_view";

struct arms_handle {
  std::unique_ptr<cvc::nav::arms_run> run;
  std::vector<PyObject *> worlds; // references to the sim_world capsules it borrows
};

inline void arms_dtor(PyObject *cap) {
  auto *h = static_cast<arms_handle *>(PyCapsule_GetPointer(cap, kArmsCapsule));
  if (!h)
    return;
  h->run.reset(); // before the worlds it borrows
  for (PyObject *w : h->worlds)
    Py_XDECREF(w);
  delete h;
}

inline arms_handle *arms_from(PyObject *cap) {
  if (!cap || !PyCapsule_CheckExact(cap))
    throw std::invalid_argument("pycvc.nav_arms_*: not an arms handle");
  auto *h = static_cast<arms_handle *>(PyCapsule_GetPointer(cap, kArmsCapsule));
  if (!h || !h->run)
    throw std::invalid_argument("pycvc.nav_arms_*: null handle");
  return h;
}

using log_ptr = std::shared_ptr<const cvc::nav::run_log>;

inline void log_dtor(PyObject *cap) {
  delete static_cast<log_ptr *>(PyCapsule_GetPointer(cap, kRunLogCapsule));
}

inline PyObject *log_capsule(log_ptr p) {
  return PyCapsule_New(new log_ptr(std::move(p)), kRunLogCapsule, log_dtor);
}

inline const cvc::nav::run_log &log_from(PyObject *cap) {
  if (!cap || !PyCapsule_CheckExact(cap))
    throw std::invalid_argument("pycvc.nav_run_log_*: not a run log handle");
  auto *p = static_cast<log_ptr *>(PyCapsule_GetPointer(cap, kRunLogCapsule));
  if (!p || !*p)
    throw std::invalid_argument("pycvc.nav_run_log_*: null handle");
  return **p;
}

inline log_ptr log_shared(PyObject *cap) {
  log_from(cap);
  return *static_cast<log_ptr *>(PyCapsule_GetPointer(cap, kRunLogCapsule));
}

inline void replay_dtor(PyObject *cap) {
  delete static_cast<cvc::nav::replay_view *>(PyCapsule_GetPointer(cap, kReplayCapsule));
}

inline cvc::nav::replay_view *replay_from(PyObject *cap) {
  if (!cap || !PyCapsule_CheckExact(cap))
    throw std::invalid_argument("pycvc.nav_replay_*: not a replay handle");
  auto *v = static_cast<cvc::nav::replay_view *>(PyCapsule_GetPointer(cap, kReplayCapsule));
  if (!v)
    throw std::invalid_argument("pycvc.nav_replay_*: null handle");
  return v;
}

inline PyObject *tables_dict(const std::vector<cvc::nav::panel_table> &tables) {
  PyObject *d = PyDict_New();
  for (const cvc::nav::panel_table &t : tables) {
    PyObject *rows = PyList_New(0);
    for (const std::vector<std::string> &r : t.rows) {
      PyObject *row = PyDict_New();
      for (std::size_t c = 0; c < t.columns.size(); ++c)
        put(row, t.columns[c].c_str(), str(c < r.size() ? r[c] : std::string()));
      PyList_Append(rows, row);
      Py_DECREF(row);
    }
    put(d, t.key.c_str(), rows);
  }
  return d;
}

inline PyObject *poses_array(const std::vector<std::vector<cvc::nav::ghost_pose>> &poses,
                             int agents) {
  std::vector<double> v;
  for (const std::vector<cvc::nav::ghost_pose> &arm : poses)
    for (const cvc::nav::ghost_pose &g : arm)
      v.insert(v.end(), {g.x, g.y, g.heading, g.length, g.width, g.wheelbase});
  return doubles(v, {static_cast<npy_intp>(poses.size()), static_cast<npy_intp>(agents), 6});
}

} // namespace pycvc_nav_lab
%}

%feature("kwargs") pycvc::nav_arms_create;
%feature("kwargs") pycvc::nav_run_log_publish;
%feature("kwargs") pycvc::nav_replay_view_update;

%inline %{
namespace pycvc {

// ── the plant ledger's items (cvc/nav/plant_ledger.h) ──────────────────────────────────────────

// Each vehicle's (each episode's) items in their JSON order: {key, unit, comment, kind ("measure",
// "count" or "flag"), source ("energy" or "plant"), needs (plant_sample::measured bits)}.
PyObject *nav_plant_ledger_vehicle() {
  return pycvc_nav_lab::ledger_items(cvc::nav::plant_ledger_vehicle());
}

PyObject *nav_plant_ledger_episode() {
  return pycvc_nav_lab::ledger_items(cvc::nav::plant_ledger_episode());
}

// Agent i's plant ledger now, from the world's nav_stats (armed): a dict per item {key, unit,
// kind, applies, value}, value None where the item does not apply.
PyObject *nav_sim_world_plant_ledger(PyObject *handle, int i) {
  const cvc::nav::episode_nav_stats e = sim_world_from(handle)->nav_stats();
  if (i < 0 || i >= static_cast<int>(e.per_vehicle.size()))
    throw std::out_of_range("pycvc.nav_sim_world_plant_ledger: no agent " + std::to_string(i));
  const cvc::nav::veh_nav_stats &v = e.per_vehicle[static_cast<std::size_t>(i)];
  PyObject *list = pycvc_nav_lab::ledger_items(cvc::nav::plant_ledger_vehicle());
  const std::span<const cvc::nav::plant_ledger_item> items = cvc::nav::plant_ledger_vehicle();
  for (std::size_t j = 0; j < items.size(); ++j) {
    PyObject *d = PyList_GetItem(list, static_cast<Py_ssize_t>(j)); // borrowed
    const bool applies = cvc::nav::plant_ledger_applies(items[j], v);
    PyDict_SetItemString(d, "applies", applies ? Py_True : Py_False);
    if (applies) {
      PyObject *x = PyFloat_FromDouble(items[j].veh(v));
      PyDict_SetItemString(d, "value", x);
      Py_DECREF(x);
    } else {
      PyDict_SetItemString(d, "value", Py_None);
    }
  }
  return list;
}

// ── arms (cvc/nav/nav_arms.h) ──────────────────────────────────────────────────────────────────

// One sim_world handle per arm (each with its policy set) and the arms: a name, or a dict
// {name, uri, sha256, meta (a JSON object's text)}. The run keeps the worlds alive.
PyObject *nav_arms_create(PyObject *worlds, PyObject *arms, unsigned seed = 0,
                          const std::string &scene = "", const std::string &plant = "",
                          const std::string &world = "{}", const std::string &settings = "{}",
                          const std::string &meta = "{}", const std::string &log_path = "",
                          double clear_safety_m = 2.0) {
  if (!PyList_Check(worlds) || !PyList_Check(arms))
    throw std::invalid_argument("pycvc.nav_arms_create: worlds and arms are lists");
  std::vector<cvc::nav::sim_world *> ws;
  for (Py_ssize_t j = 0; j < PyList_Size(worlds); ++j)
    ws.push_back(sim_world_from(PyList_GetItem(worlds, j)));
  std::vector<cvc::nav::arm> as;
  for (Py_ssize_t j = 0; j < PyList_Size(arms); ++j) {
    PyObject *a = PyList_GetItem(arms, j);
    cvc::nav::arm x;
    if (PyUnicode_Check(a)) {
      x.name = PyUnicode_AsUTF8(a);
    } else if (PyDict_Check(a)) {
      x.name = pycvc_nav_lab::text_of(PyDict_GetItemString(a, "name"), "an arm's name");
      x.uri = pycvc_nav_lab::text_of(PyDict_GetItemString(a, "uri"), "an arm's uri");
      x.sha256 = pycvc_nav_lab::text_of(PyDict_GetItemString(a, "sha256"), "an arm's sha256");
      const std::string m = pycvc_nav_lab::text_of(PyDict_GetItemString(a, "meta"), "meta");
      x.meta = m.empty() ? "{}" : m;
    } else {
      throw std::invalid_argument("pycvc.nav_arms_create: an arm is a name or a dict");
    }
    as.push_back(std::move(x));
  }
  cvc::nav::arms_options o;
  o.seed = seed;
  o.scene = scene;
  o.plant = plant;
  o.world = world;
  o.settings = settings;
  o.meta = meta;
  o.log_path = log_path;
  o.stats.clear_safety_m = clear_safety_m;
  auto *h = new pycvc_nav_lab::arms_handle();
  try {
    h->run = std::make_unique<cvc::nav::arms_run>(std::move(ws), std::move(as), std::move(o));
  } catch (...) {
    delete h;
    throw;
  }
  for (Py_ssize_t j = 0; j < PyList_Size(worlds); ++j) {
    PyObject *w = PyList_GetItem(worlds, j);
    Py_INCREF(w);
    h->worlds.push_back(w);
  }
  return PyCapsule_New(h, pycvc_nav_lab::kArmsCapsule, pycvc_nav_lab::arms_dtor);
}

// One tick of every arm's world, timed and recorded (the GIL released while they step).
void nav_arms_step(PyObject *handle, int num_threads = 0) {
  cvc::nav::arms_run &run = *pycvc_nav_lab::arms_from(handle)->run;
  std::string err;
  Py_BEGIN_ALLOW_THREADS
  try {
    run.step(num_threads);
  } catch (const std::exception &e) {
    err = e.what();
  }
  Py_END_ALLOW_THREADS
  if (!err.empty())
    throw std::runtime_error(err);
}

// For a host that steps world a itself (pycvc.nav_sim_world_step): record it, then end the tick.
void nav_arms_record(PyObject *handle, int a, double ms) {
  pycvc_nav_lab::arms_from(handle)->run->record(a, ms);
}

void nav_arms_end_tick(PyObject *handle) { pycvc_nav_lab::arms_from(handle)->run->end_tick(); }

// Every arm's agents to `goals` ((N, 2) world metres) from the next tick: a new leg.
void nav_arms_retarget(PyObject *handle, PyObject *goals) {
  cvc::nav::arms_run &run = *pycvc_nav_lab::arms_from(handle)->run;
  const std::vector<float> g = pycvc_nav_lab::floats(goals, 2 * run.agents(), "goals");
  run.retarget(g.data(), run.agents());
}

// Agent i's goal in every arm (world metres), the others keeping theirs: a new leg.
void nav_arms_retarget_agent(PyObject *handle, int i, double gx, double gy) {
  pycvc_nav_lab::arms_from(handle)->run->retarget(i, gx, gy);
}

// Arm a's agents' sizes ((N,) metres each; None leaves that one to the world) from the next tick.
void nav_arms_set_size(PyObject *handle, int a, PyObject *length, PyObject *width,
                       PyObject *wheelbase) {
  cvc::nav::arms_run &run = *pycvc_nav_lab::arms_from(handle)->run;
  const int n = run.agents();
  const std::vector<float> l = pycvc_nav_lab::floats(length, n, "length"),
                           w = pycvc_nav_lab::floats(width, n, "width"),
                           b = pycvc_nav_lab::floats(wheelbase, n, "wheelbase");
  run.set_size(a, l.empty() ? nullptr : l.data(), w.empty() ? nullptr : w.data(),
               b.empty() ? nullptr : b.data());
}

void nav_arms_finish(PyObject *handle) { pycvc_nav_lab::arms_from(handle)->run->finish(); }

long nav_arms_tick(PyObject *handle) { return pycvc_nav_lab::arms_from(handle)->run->tick(); }

int nav_arms_count(PyObject *handle) { return pycvc_nav_lab::arms_from(handle)->run->arms(); }

int nav_arms_agents(PyObject *handle) {
  return pycvc_nav_lab::arms_from(handle)->run->agents();
}

// The run's log, live: it grows as the run steps.
PyObject *nav_arms_log(PyObject *handle) {
  return pycvc_nav_lab::log_capsule(pycvc_nav_lab::arms_from(handle)->run->shared_log());
}

// ── the run log (cvc/nav/run_log.h) ────────────────────────────────────────────────────────────

PyObject *nav_run_log_read(const std::string &path) {
  return pycvc_nav_lab::log_capsule(
      std::make_shared<const cvc::nav::run_log>(cvc::nav::run_log::read(path)));
}

void nav_run_log_write(PyObject *log, const std::string &path) {
  pycvc_nav_lab::log_from(log).write(path);
}

// The header line (a JSON object's text; json.loads it).
PyObject *nav_run_log_header(PyObject *log) {
  return pycvc_nav_lab::str(pycvc_nav_lab::log_from(log).header().to_json());
}

long nav_run_log_ticks(PyObject *log) { return pycvc_nav_lab::log_from(log).ticks(); }

// Every column as an array: "t" (T,); each agent column (T, A, N); each arm column (T, A);
// "leg_tick" (L,) and "leg_goal" (L, N, 2). NaN where a value is not applicable.
PyObject *nav_run_log_arrays(PyObject *log) {
  const cvc::nav::run_log &l = pycvc_nav_lab::log_from(log);
  const npy_intp T = l.ticks(), A = l.arms(), N = l.agents();
  PyObject *d = PyDict_New();
  pycvc_nav_lab::put(d, "t", pycvc_nav_lab::doubles(l.times(), {T}));
  for (std::size_t c = 0; c < l.header().agent_columns.size(); ++c) {
    std::vector<double> v(static_cast<std::size_t>(T * A * N));
    for (long k = 0; k < T; ++k)
      for (int a = 0; a < A; ++a)
        for (int i = 0; i < N; ++i)
          v[(static_cast<std::size_t>(k) * A + a) * N + i] = l.at(k, a, i, static_cast<int>(c));
    pycvc_nav_lab::put(d, l.header().agent_columns[c].c_str(),
                       pycvc_nav_lab::doubles(v, {T, A, N}));
  }
  for (std::size_t c = 0; c < l.header().arm_columns.size(); ++c) {
    std::vector<double> v(static_cast<std::size_t>(T * A));
    for (long k = 0; k < T; ++k)
      for (int a = 0; a < A; ++a)
        v[static_cast<std::size_t>(k) * A + a] = l.arm_at(k, a, static_cast<int>(c));
    pycvc_nav_lab::put(d, l.header().arm_columns[c].c_str(), pycvc_nav_lab::doubles(v, {T, A}));
  }
  std::vector<double> ticks, goals;
  for (const cvc::nav::run_log_leg &leg : l.legs()) {
    ticks.push_back(static_cast<double>(leg.tick));
    goals.insert(goals.end(), leg.goals.begin(), leg.goals.end());
  }
  const npy_intp L = static_cast<npy_intp>(l.legs().size());
  pycvc_nav_lab::put(d, "leg_tick", pycvc_nav_lab::doubles(ticks, {L}));
  pycvc_nav_lab::put(d, "leg_goal", pycvc_nav_lab::doubles(goals, {L, N, 2}));
  return d;
}

// The legs: {tick, goals ((N, 2) list), nav_stats (a JSON text per arm once the leg ended)}.
PyObject *nav_run_log_legs(PyObject *log) {
  const cvc::nav::run_log &l = pycvc_nav_lab::log_from(log);
  PyObject *list = PyList_New(0);
  for (const cvc::nav::run_log_leg &leg : l.legs()) {
    PyObject *d = PyDict_New();
    pycvc_nav_lab::put(d, "tick", PyLong_FromLong(leg.tick));
    PyObject *g = PyList_New(0);
    for (std::size_t i = 0; i + 1 < leg.goals.size(); i += 2) {
      PyObject *p = Py_BuildValue("(dd)", leg.goals[i], leg.goals[i + 1]);
      PyList_Append(g, p);
      Py_DECREF(p);
    }
    pycvc_nav_lab::put(d, "goals", g);
    PyObject *s = PyList_New(0);
    for (const std::string &j : leg.nav_stats) {
      PyObject *t = pycvc_nav_lab::str(j);
      PyList_Append(s, t);
      Py_DECREF(t);
    }
    pycvc_nav_lab::put(d, "nav_stats", s);
    PyList_Append(list, d);
    Py_DECREF(d);
  }
  return list;
}

// Each arm's nav_stats record at the end (JSON texts), or None before the end line.
PyObject *nav_run_log_end(PyObject *log) {
  const cvc::nav::run_log &l = pycvc_nav_lab::log_from(log);
  if (!l.finished())
    Py_RETURN_NONE;
  PyObject *list = PyList_New(0);
  for (const std::string &j : l.end_nav_stats()) {
    PyObject *t = pycvc_nav_lab::str(j);
    PyList_Append(list, t);
    Py_DECREF(t);
  }
  return list;
}

long nav_run_log_index(PyObject *log, double t) { return pycvc_nav_lab::log_from(log).index(t); }

int nav_run_log_leg_of(PyObject *log, long k) { return pycvc_nav_lab::log_from(log).leg_of(k); }

// The Stats and Ledger tables at tick k (-1: the last) for agent `agent` of each arm, and arm
// `arm`'s plant ledger: key -> list of row dicts, plus "ledger.plant_note" -> its text.
PyObject *nav_run_log_tables(PyObject *log, long k = -1, int agent = 0, int arm = 0) {
  const cvc::nav::run_log &l = pycvc_nav_lab::log_from(log);
  if (k < 0)
    k = l.ticks() - 1;
  std::vector<cvc::nav::panel_table> t = cvc::nav::arms_tables(l, k, agent);
  t.push_back(cvc::nav::plant_table(l, k, arm, agent));
  PyObject *d = pycvc_nav_lab::tables_dict(t);
  pycvc_nav_lab::put(d, "ledger.plant_note",
                     pycvc_nav_lab::str(cvc::nav::plant_note(l, k, arm, agent)));
  return d;
}

// The charts: key -> {"series": [{label, style, x, y}], "markers": [(x, label, kind)]}.
PyObject *nav_run_log_charts(PyObject *log, int arm = 0, int agent = 0,
                             const std::string &book = "dU") {
  const cvc::nav::run_log &l = pycvc_nav_lab::log_from(log);
  PyObject *d = PyDict_New();
  for (const cvc::nav::chart_plot &c : cvc::nav::arms_charts(l, arm, agent, book)) {
    PyObject *series = PyList_New(0);
    for (const cvc::nav::chart_series &s : c.series) {
      PyObject *e = PyDict_New();
      pycvc_nav_lab::put(e, "label", pycvc_nav_lab::str(s.label));
      pycvc_nav_lab::put(e, "style", pycvc_nav_lab::str(s.style));
      pycvc_nav_lab::put(e, "x", pycvc_nav_lab::doubles(s.x, {static_cast<npy_intp>(s.x.size())}));
      pycvc_nav_lab::put(e, "y", pycvc_nav_lab::doubles(s.y, {static_cast<npy_intp>(s.y.size())}));
      PyList_Append(series, e);
      Py_DECREF(e);
    }
    PyObject *marks = PyList_New(0);
    for (const cvc::nav::chart_marker &m : c.markers) {
      PyObject *t = Py_BuildValue("(dss)", m.x, m.label.c_str(), m.kind.c_str());
      PyList_Append(marks, t);
      Py_DECREF(t);
    }
    PyObject *e = PyDict_New();
    pycvc_nav_lab::put(e, "series", series);
    pycvc_nav_lab::put(e, "markers", marks);
    pycvc_nav_lab::put(d, c.key.c_str(), e);
  }
  return d;
}

// Several logs' arms side by side (arm arms[j] of logs[j], renamed names[j]).
PyObject *nav_run_log_stack(PyObject *logs, PyObject *arms, PyObject *names) {
  if (!PyList_Check(logs) || !PyList_Check(arms) || !PyList_Check(names))
    throw std::invalid_argument("pycvc.nav_run_log_stack: logs, arms and names are lists");
  std::vector<const cvc::nav::run_log *> ls;
  std::vector<int> as;
  std::vector<std::string> ns;
  for (Py_ssize_t j = 0; j < PyList_Size(logs); ++j)
    ls.push_back(&pycvc_nav_lab::log_from(PyList_GetItem(logs, j)));
  for (Py_ssize_t j = 0; j < PyList_Size(arms); ++j)
    as.push_back(static_cast<int>(PyLong_AsLong(PyList_GetItem(arms, j))));
  for (Py_ssize_t j = 0; j < PyList_Size(names); ++j)
    ns.push_back(pycvc_nav_lab::text_of(PyList_GetItem(names, j), "a name"));
  return pycvc_nav_lab::log_capsule(std::make_shared<const cvc::nav::run_log>(
      cvc::nav::run_log::stack(ls, as, ns)));
}

// Every arm's agents' poses at time t, (A, N, 6): x, y, heading, length, width, wheelbase.
PyObject *nav_run_log_poses(PyObject *log, double t) {
  const cvc::nav::run_log &l = pycvc_nav_lab::log_from(log);
  return pycvc_nav_lab::poses_array(cvc::nav::poses_at(l, t), l.agents());
}

// The Stats and Ledger windows at tick k (-1: the last) into `app`'s tree under `prefix`, for the
// arm, agent and book the document picks (ledger.agent, ledger.vehicle, ledger.book); with
// charts, the charts too. Returns the choice: {arm, agent, book}.
PyObject *nav_run_log_publish(std::shared_ptr<cvc::app> app, const std::string &prefix,
                              PyObject *log, long k = -1, bool charts = false) {
  if (!app)
    throw std::invalid_argument("nav_run_log_publish: app is None");
  const cvc::nav::run_log &l = pycvc_nav_lab::log_from(log);
  if (k < 0)
    k = l.ticks() - 1;
  const cvc::nav::panel_choice c = cvc::nav::read_choice(*app, prefix, l);
  cvc::nav::publish_tables(*app, prefix, l, k, c);
  if (charts)
    cvc::nav::publish_charts(*app, prefix, l, c);
  return Py_BuildValue("{s:i,s:i,s:s}", "arm", c.arm, "agent", c.agent, "book", c.book.c_str());
}

// ── a replay (cvc::nav::replay_view) ───────────────────────────────────────────────────────────

PyObject *nav_replay_view_create(PyObject *log, double at = 0.0) {
  return PyCapsule_New(new cvc::nav::replay_view(pycvc_nav_lab::log_shared(log), at),
                       pycvc_nav_lab::kReplayCapsule, pycvc_nav_lab::replay_dtor);
}

// A frame: the play head moves (dt s of wall time), the tables show its tick and, once and on a
// new choice, the charts. Returns {k, t, leg, arm, agent, book, poses ((A, N, 6))}.
PyObject *nav_replay_view_update(PyObject *view, std::shared_ptr<cvc::app> app,
                                 const std::string &prefix, double dt, bool charts = true) {
  if (!app)
    throw std::invalid_argument("nav_replay_view_update: app is None");
  cvc::nav::replay_view *v = pycvc_nav_lab::replay_from(view);
  const cvc::nav::replay_view::frame f = v->update(*app, prefix, dt, charts);
  PyObject *d = PyDict_New();
  pycvc_nav_lab::put(d, "k", PyLong_FromLong(f.k));
  pycvc_nav_lab::put(d, "t", PyFloat_FromDouble(f.t));
  pycvc_nav_lab::put(d, "leg", PyLong_FromLong(f.leg));
  pycvc_nav_lab::put(d, "arm", PyLong_FromLong(f.choice.arm));
  pycvc_nav_lab::put(d, "agent", PyLong_FromLong(f.choice.agent));
  pycvc_nav_lab::put(d, "book", pycvc_nav_lab::str(f.choice.book));
  pycvc_nav_lab::put(d, "poses", pycvc_nav_lab::poses_array(f.poses, v->log().agents()));
  return d;
}

// The transport's verbs (sim.step, sim.restart, a chart's seek) and a direct set.
void nav_replay_view_step(PyObject *view) { pycvc_nav_lab::replay_from(view)->head().step(); }

void nav_replay_view_restart(PyObject *view, std::shared_ptr<cvc::app> app,
                             const std::string &prefix) {
  if (!app)
    throw std::invalid_argument("nav_replay_view_restart: app is None");
  pycvc_nav_lab::replay_from(view)->head().restart(*app, prefix);
}

void nav_replay_view_seek(PyObject *view, std::shared_ptr<cvc::app> app,
                          const std::string &prefix) {
  if (!app)
    throw std::invalid_argument("nav_replay_view_seek: app is None");
  pycvc_nav_lab::replay_from(view)->head().seek(*app, prefix);
}

void nav_replay_view_set(PyObject *view, double t) {
  pycvc_nav_lab::replay_from(view)->head().set(t);
}

double nav_replay_view_time(PyObject *view) {
  return pycvc_nav_lab::replay_from(view)->head().time();
}

void nav_replay_view_redraw(PyObject *view) { pycvc_nav_lab::replay_from(view)->redraw(); }

} // namespace pycvc
%}
