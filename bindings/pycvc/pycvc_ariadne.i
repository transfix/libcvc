// pycvc_ariadne.i — the GL-free half of a Python Ariadne host, in pycvc:
//
// - pycvc.host_function(name, fn, is_async=False, with_call=False, done_channel=""): a Python
//   callable as a host function with arguments and a value (pycvc::host_function, pycvc_value.h).
//   pycvc builds it because the value conversion lives here; pycvc_gl.AriRuntime and pycvc.AriApp
//   register it (register_host_function), and the document calls it as (name a b ...).
// - pycvc.AriApp(app, prefix="ari"): cvc::ariadne::AppRuntime with no backend and no scene, for
//   running a document's programs with no window and no GL (tests, CI, a server). Its init:,
//   residents, actions, host functions and state behave as under arigl; nothing is drawn, so a
//   computed bind: or text: is never evaluated, and a document that requires gl.scene does not
//   load. pycvc.ari.Host(mode="headless") drives it. grid(node, cell, half) and world_geometry(node)
//   read the scene's world (SceneWorld) wherever it runs: this runtime's, or, attached to a drawn
//   host's runtime (ari_app_attach), its renderer's.
// - pycvc.ari_app_attach(app, runtime): an AriApp over a scripting host's own AppRuntime (arix,
//   ariglx lend it as a "cvc.ariadne.AppRuntime" capsule, _arihost.runtime()), which it does not
//   own: the host's python: blocks register their functions through it, and
//   pycvc.ari.Host() drives it in arix.
// - pycvc.WorldEntities: the running world's entities (its vehicles, WorldVehicles), from
//   AriApp.entities() or pycvc.world_entities(app, runtime) over a "cvc.ariadne.AppRuntime"
//   capsule (a GL host's: pycvc_gl.AriRuntime.app_runtime_capsule()). It lists, spawns and
//   removes them, sets their intents (set_intent: a steer and acceleration, a force), and places
//   the `pose` entities an outside simulation runs (set_pose, set_poses: no plant integrates them).
//
// The GIL: frame()/step()/load()/run_program's drain and wait_idle() release it, so a Python
// function the document calls (which takes it back) and an async function on a pool thread run.
// %include'd from pycvc.i after pycvc_exec.h. NOTE: a SWIG interface file — do NOT run clang-format.
%{
#include <cvc/ariadne/app_runtime.h>
#include <cvc/ariadne/world_vehicles.h>
#include <cvc/nav/world_grid.h>
#include <cvc/world/world_geometry.h>
#include <cmath>
#include <cstring>
#include <cvc/core/app.h>
#include <cvc/state/state_exec/async_host.h>
#include <stdexcept>

namespace pycvc {

// A Python callable as a C++ callback: it owns a reference (dropped under the GIL), takes the GIL
// for each call, and prints (never propagates) a Python exception, as pycvc_gl's handlers do.
static std::function<void()> ariadne_void_callback(PyObject *fn, const char *who) {
  if (fn == nullptr || !PyCallable_Check(fn))
    throw std::invalid_argument(std::string(who) + ": expected a callable");
  std::shared_ptr<PyObject> held = python_ref(fn);
  return [held]() {
    PyGILState_STATE g = PyGILState_Ensure();
    PyObject *r = PyObject_CallObject(held.get(), nullptr);
    if (r == nullptr)
      PyErr_Print();
    Py_XDECREF(r);
    PyGILState_Release(g);
  };
}

static std::function<void(const std::string &)> ariadne_done_callback(PyObject *fn,
                                                                     const char *who) {
  if (fn == nullptr || fn == Py_None)
    return nullptr;
  if (!PyCallable_Check(fn))
    throw std::invalid_argument(std::string(who) + ": done must be callable or None");
  std::shared_ptr<PyObject> held = python_ref(fn);
  return [held](const std::string &how) {
    PyGILState_STATE g = PyGILState_Ensure();
    PyObject *r = PyObject_CallFunction(held.get(), "s", how.c_str());
    if (r == nullptr)
      PyErr_Print();
    Py_XDECREF(r);
    PyGILState_Release(g);
  };
}

host_function make_host_function(const std::string &name, PyObject *fn, bool is_async,
                                 bool with_call, const std::string &done_channel) {
  if (name.empty())
    throw std::invalid_argument("pycvc.host_function: the name is empty");
  host_function f;
  f.name = name;
  if (is_async) {
    f.marshal = python_async_marshal("pycvc.host_function", name, fn, with_call);
    f.done_channel = done_channel;
  } else {
    if (with_call)
      throw std::invalid_argument("pycvc.host_function: with_call is for an async function");
    if (!done_channel.empty())
      throw std::invalid_argument("pycvc.host_function: done_channel is for an async function");
    f.call = python_native_fn("pycvc.host_function", name, fn);
  }
  return f;
}

// A world's entities for a Python host (WorldVehicles): over an AppRuntime it does not own, finding
// the running world again on each call, so it follows a reload. list(), spawn() and despawn() run
// on the thread that drains the runtime (a frame hook, a host function); set_intent(), intent(),
// clear_intent(), set_pose(), set_poses() and pose() from any thread while the document is loaded.
// Without a world with vehicles, list() is empty and the rest return False / 0 / None.
class WorldEntities {
public:
  WorldEntities(std::shared_ptr<cvc::app> app, cvc::ariadne::AppRuntime *rt)
      : app_(std::move(app)), rt_(rt) {
    if (!app_ || !rt_)
      throw std::invalid_argument("pycvc.WorldEntities: no app or no runtime");
  }
  bool available() const { return vehicles() != nullptr; }

  // One dict per entity: id, name, archetype, team, controller, plant, placed, x, y, z, yaw, speed,
  // power_kw, energy_wh, distance, gear, the command its plant took (steer, accel) and its intent;
  // vx, vy (m/s), steer (rad), goal_x, goal_y, reach (m), reached, collided (a contact with a
  // body other than the ground, or another vehicle, since the last tick); and its body: length, width,
  // wheelbase (m), max_steer (rad), max_accel, max_decel (m/s^2), max_speed (m/s).
  PyObject *list() const {
    PyObject *out = PyList_New(0);
    if (!out)
      return nullptr;
    if (cvc::ariadne::WorldVehicles *v = vehicles())
      for (const cvc::ariadne::entity_info &e : v->entities()) {
        PyObject *d = PyDict_New();
        put(d, "id", PyLong_FromUnsignedLongLong(e.id));
        put(d, "name", PyUnicode_FromString(e.spec.name.c_str()));
        put(d, "archetype", PyUnicode_FromString(e.spec.archetype.c_str()));
        put(d, "team", PyLong_FromLong(e.spec.team));
        put(d, "controller", PyUnicode_FromString(e.spec.controller.c_str()));
        put(d, "plant", PyUnicode_FromString(e.plant.c_str()));
        put(d, "placed", PyBool_FromLong(e.placed ? 1 : 0));
        put(d, "x", PyFloat_FromDouble(e.x));
        put(d, "y", PyFloat_FromDouble(e.y));
        put(d, "z", PyFloat_FromDouble(e.z));
        put(d, "yaw", PyFloat_FromDouble(e.yaw));
        put(d, "speed", PyFloat_FromDouble(e.speed));
        put(d, "power_kw", PyFloat_FromDouble(e.power_kw));
        put(d, "energy_wh", PyFloat_FromDouble(e.energy_wh));
        put(d, "distance", PyFloat_FromDouble(e.distance));
        put(d, "gear", PyLong_FromLong(e.gear));
        put(d, "command_steer", PyFloat_FromDouble(e.command_steer));
        put(d, "command_accel", PyFloat_FromDouble(e.command_accel));
        put(d, "intent", intent_dict(e.intent));
        put(d, "vx", PyFloat_FromDouble(e.vx));
        put(d, "vy", PyFloat_FromDouble(e.vy));
        put(d, "steer", PyFloat_FromDouble(e.steer));
        put(d, "goal_x", PyFloat_FromDouble(e.goal_x));
        put(d, "goal_y", PyFloat_FromDouble(e.goal_y));
        put(d, "reach", PyFloat_FromDouble(e.reach));
        put(d, "reached", PyBool_FromLong(e.reached ? 1 : 0));
        put(d, "collided", PyBool_FromLong(e.obstacle_contacts > 0 || e.vehicle_contact ? 1 : 0));
        cvc::physics::vehicle_desc body = cvc::physics::vehicle_desc::humvee();
        for (const cvc::ariadne::vehicle_archetype &a : v->archetypes())
          if (a.id == e.spec.archetype) {
            try {
              body = cvc::ariadne::vehicle_desc_of(a, e.spec.mass, e.spec.max_speed);
            } catch (const std::exception &) {
            }
            break;
          }
        put(d, "length", PyFloat_FromDouble(2.0 * body.half_extents.x));
        put(d, "width", PyFloat_FromDouble(2.0 * body.half_extents.y));
        put(d, "wheelbase", PyFloat_FromDouble(body.wheelbase()));
        put(d, "max_steer", PyFloat_FromDouble(body.max_steer));
        put(d, "max_accel", PyFloat_FromDouble(body.max_accel));
        put(d, "max_decel", PyFloat_FromDouble(body.max_decel));
        put(d, "max_speed", PyFloat_FromDouble(body.max_speed));
        put(d, "posed", PyBool_FromLong(e.posed ? 1 : 0));
        put(d, "pose", e.posed ? pose_dict(e.pose) : (Py_INCREF(Py_None), Py_None));
        PyList_Append(out, d);
        Py_DECREF(d);
      }
    return out;
  }
  // A spawn (entity_spec): its id, or 0 when it was refused (why() says why). `at` is None (a
  // spawn point, else a free point) or (x, y) / (x, y, yaw_degrees) in world metres.
  unsigned long long spawn(const std::string &archetype, const std::string &name = "",
                           const std::string &controller = "", int team = 0,
                           PyObject *at = nullptr, double mass = 0.0, double max_speed = 0.0) {
    why_.clear();
    cvc::ariadne::WorldVehicles *v = vehicles();
    if (!v) {
      why_ = "no world with vehicles";
      return 0;
    }
    cvc::ariadne::entity_spec s;
    s.archetype = archetype;
    s.name = name;
    s.controller = controller;
    s.team = team;
    s.mass = mass;
    s.max_speed = max_speed;
    if (at && at != Py_None) {
      PyObject *seq = PySequence_Fast(at, "pycvc.WorldEntities.spawn: at must be (x, y[, yaw])");
      if (!seq)
        throw std::invalid_argument("pycvc.WorldEntities.spawn: at must be (x, y[, yaw])");
      const Py_ssize_t n = PySequence_Fast_GET_SIZE(seq);
      double xyz[3] = {0.0, 0.0, 0.0};
      for (Py_ssize_t i = 0; i < n && i < 3; ++i)
        xyz[i] = PyFloat_AsDouble(PySequence_Fast_GET_ITEM(seq, i));
      Py_DECREF(seq);
      if (PyErr_Occurred()) {
        PyErr_Clear();
        throw std::invalid_argument("pycvc.WorldEntities.spawn: at must hold numbers");
      }
      if (n < 2 || n > 3)
        throw std::invalid_argument("pycvc.WorldEntities.spawn: at must be (x, y[, yaw])");
      s.placed = true;
      s.x = xyz[0];
      s.y = xyz[1];
      if (n == 3)
        s.yaw = xyz[2] * 3.14159265358979323846 / 180.0;
    }
    return v->spawn(s, &why_);
  }
  std::string why() const { return why_; }
  bool despawn(unsigned long long id) {
    cvc::ariadne::WorldVehicles *v = vehicles();
    return v && v->despawn(id);
  }
  // An intent (entity_intent): `steer` (rad, + left) and `accel` (m/s^2) as the command, `mode`
  // "replace", "blend" (with `weight`) or "off" ("": replace when a steer or an accel is given,
  // else off), and `force` (fx, fy) in m/s^2, world frame. False for an id that is not an entity.
  bool set_intent(unsigned long long id, PyObject *steer = nullptr, PyObject *accel = nullptr,
                  const std::string &mode = "", double weight = 0.5, PyObject *force = nullptr,
                  bool force_steers = true) {
    cvc::ariadne::WorldVehicles *v = vehicles();
    if (!v)
      return false;
    cvc::ariadne::entity_intent in;
    const bool has_steer = number(steer, "steer", in.steer);
    const bool has_accel = number(accel, "accel", in.accel);
    if (mode.empty())
      in.command = has_steer || has_accel ? cvc::ariadne::entity_intent::mode::replace
                                          : cvc::ariadne::entity_intent::mode::off;
    else if (!cvc::ariadne::parse_intent_mode(mode, in.command))
      throw std::invalid_argument("pycvc.WorldEntities.set_intent: mode must be replace, blend or "
                                  "off, not '" + mode + "'");
    if (!(weight >= 0.0 && weight <= 1.0))
      throw std::invalid_argument("pycvc.WorldEntities.set_intent: weight must be in [0, 1]");
    in.weight = weight;
    if (force && force != Py_None) {
      PyObject *seq = PySequence_Fast(force, "pycvc.WorldEntities.set_intent: force must be (fx, fy)");
      if (!seq)
        throw std::invalid_argument("pycvc.WorldEntities.set_intent: force must be (fx, fy)");
      const bool two = PySequence_Fast_GET_SIZE(seq) == 2;
      if (two) {
        in.force_x = PyFloat_AsDouble(PySequence_Fast_GET_ITEM(seq, 0));
        in.force_y = PyFloat_AsDouble(PySequence_Fast_GET_ITEM(seq, 1));
      }
      Py_DECREF(seq);
      if (!two || PyErr_Occurred()) {
        PyErr_Clear();
        throw std::invalid_argument("pycvc.WorldEntities.set_intent: force must be (fx, fy)");
      }
    }
    in.force_steers = force_steers;
    return v->set_intent(id, in);
  }
  PyObject *intent(unsigned long long id) const {
    cvc::ariadne::WorldVehicles *v = vehicles();
    return intent_dict(v ? v->intent(id) : cvc::ariadne::entity_intent{});
  }
  bool clear_intent(unsigned long long id) {
    cvc::ariadne::WorldVehicles *v = vehicles();
    return v && v->clear_intent(id);
  }
  // A `pose` entity's pose (entity_pose): (x, y) its chassis's centre in world metres, or its rear
  // axle with rear_axle=True; z None (standing on the ground) or metres; heading, pitch and roll in
  // rad; speed (m/s, its wheels roll at it) and steer (rad, + left); t None, or the time (s) of the
  // simulation that put it there (timed poses are drawn between the last two). Taken at the next
  // physics step. False for an id that is not a `pose` entity; RuntimeError for a number that
  // is not finite.
  bool set_pose(unsigned long long id, double x, double y, PyObject *z = nullptr,
                double heading = 0.0, double pitch = 0.0, double roll = 0.0, double speed = 0.0,
                double steer = 0.0, PyObject *t = nullptr, bool rear_axle = false) {
    cvc::ariadne::WorldVehicles *v = vehicles();
    if (!v)
      return false;
    cvc::ariadne::entity_pose p;
    p.x = x;
    p.y = y;
    number(z, "z", p.z, "set_pose");
    p.heading = heading;
    p.pitch = pitch;
    p.roll = roll;
    p.speed = speed;
    p.steer = steer;
    number(t, "t", p.time, "set_pose");
    p.rear_axle = rear_axle;
    for (const double u : {x, y, heading, pitch, roll, speed, steer})
      if (!std::isfinite(u))
        throw std::invalid_argument("pycvc.WorldEntities.set_pose: x, y, the angles, speed and "
                                    "steer must be finite");
    return v->set_pose(id, p);
  }
  // Many poses at once, under one lock: `ids` (N whole numbers) and `poses`, N rows of 2 to 9
  // numbers, x y [z heading pitch roll speed steer t] (NaN for z or t: not given; a short row
  // leaves the rest at set_pose's defaults), a numpy array or nested sequences. How many it took
  // (a row for an id that is not a `pose` entity, or with a number that is not finite, is left).
  unsigned long long set_poses(PyObject *ids, PyObject *poses, bool rear_axle = false) {
    cvc::ariadne::WorldVehicles *v = vehicles();
    if (!v)
      return 0;
    PyArrayObject *ia = reinterpret_cast<PyArrayObject *>(
        PyArray_FROMANY(ids, NPY_UINT64, 1, 1, NPY_ARRAY_IN_ARRAY | NPY_ARRAY_FORCECAST));
    if (!ia) {
      PyErr_Clear();
      throw std::invalid_argument("pycvc.WorldEntities.set_poses: ids must be N whole numbers");
    }
    PyArrayObject *pa = reinterpret_cast<PyArrayObject *>(
        PyArray_FROMANY(poses, NPY_DOUBLE, 2, 2, NPY_ARRAY_IN_ARRAY | NPY_ARRAY_FORCECAST));
    if (!pa) {
      Py_DECREF(ia);
      PyErr_Clear();
      throw std::invalid_argument("pycvc.WorldEntities.set_poses: poses must be N rows of 2 to 9 "
                                  "numbers");
    }
    const npy_intp n = PyArray_DIM(ia, 0), rows = PyArray_DIM(pa, 0), cols = PyArray_DIM(pa, 1);
    if (rows != n || cols < 2 || cols > 9) {
      Py_DECREF(ia);
      Py_DECREF(pa);
      throw std::invalid_argument("pycvc.WorldEntities.set_poses: poses must be one row of 2 to 9 "
                                  "numbers per id");
    }
    const auto *id = static_cast<const unsigned long long *>(PyArray_DATA(ia));
    const auto *m = static_cast<const double *>(PyArray_DATA(pa));
    std::vector<cvc::ariadne::entity_pose_update> batch(static_cast<std::size_t>(n));
    for (npy_intp i = 0; i < n; ++i) {
      cvc::ariadne::entity_pose &p = batch[static_cast<std::size_t>(i)].pose;
      batch[static_cast<std::size_t>(i)].id = id[i];
      double *field[9] = {&p.x, &p.y, &p.z, &p.heading, &p.pitch, &p.roll, &p.speed, &p.steer,
                          &p.time};
      for (npy_intp c = 0; c < cols; ++c)
        *field[c] = m[i * cols + c];
      p.rear_axle = rear_axle;
    }
    Py_DECREF(ia);
    Py_DECREF(pa);
    return v->set_poses(batch);
  }
  // The pose last given to entity `id` (a dict as set_pose's arguments), or None.
  PyObject *pose(unsigned long long id) const {
    cvc::ariadne::WorldVehicles *v = vehicles();
    cvc::ariadne::entity_pose p;
    if (!v || !v->pose(id, p))
      Py_RETURN_NONE;
    return pose_dict(p);
  }
  // How far (m) a pose may be from the last for the entity to move there (farther: at once).
  bool set_pose_jump(double metres) {
    cvc::ariadne::WorldVehicles *v = vehicles();
    if (!v)
      return false;
    v->set_pose_jump(metres);
    return true;
  }
  // Entity `id`'s goal (world metres), which nav@1 drives to; False for an id that is not one.
  bool set_goal(unsigned long long id, double x, double y) {
    cvc::ariadne::WorldVehicles *v = vehicles();
    return v && v->set_goal(id, x, y);
  }
  // Whether nav@1 draws another goal at each one it reaches (True by default); False holds them.
  bool set_wander(bool wander) {
    cvc::ariadne::WorldVehicles *v = vehicles();
    if (!v)
      return false;
    v->set_wander(wander);
    return true;
  }
  // The navigator's grid of the world on, as pycvc.WorldGym.grid gives it: occupancy (rows, cols)
  // uint8 (1 blocked, row 0 at min_y), height (rows, cols) float32, bounds (min_x, min_y, max_x,
  // max_y). None without a world with vehicles, or before it has a grid.
  PyObject *grid() const {
    cvc::ariadne::WorldVehicles *v = vehicles();
    std::shared_ptr<const cvc::nav::world_grid> g = v ? v->grid() : nullptr;
    if (!g)
      Py_RETURN_NONE;
    PyObject *d = PyDict_New();
    npy_intp dims[2] = {g->rows, g->cols};
    PyObject *occ = PyArray_ZEROS(2, dims, NPY_UINT8, 0);
    if (occ && g->occupancy.size() == static_cast<std::size_t>(g->rows) * g->cols)
      std::memcpy(PyArray_DATA(reinterpret_cast<PyArrayObject *>(occ)), g->occupancy.data(),
                  g->occupancy.size());
    put(d, "occupancy", occ);
    PyObject *h = PyArray_ZEROS(2, dims, NPY_FLOAT, 0);
    if (h && g->height.size() == g->occupancy.size() && !g->height.empty())
      std::memcpy(PyArray_DATA(reinterpret_cast<PyArrayObject *>(h)), g->height.data(),
                  g->height.size() * sizeof(float));
    put(d, "height", h);
    put(d, "bounds", Py_BuildValue("(dddd)", g->min_x, g->min_y, g->max_x, g->max_y));
    return d;
  }

private:
  cvc::ariadne::WorldVehicles *vehicles() const {
    cvc::ariadne::SceneWorld *w = rt_->running_world();
    return w ? w->vehicles() : nullptr;
  }
  static void put(PyObject *d, const char *k, PyObject *v) {
    PyDict_SetItemString(d, k, v);
    Py_XDECREF(v);
  }
  static PyObject *intent_dict(const cvc::ariadne::entity_intent &in) {
    PyObject *d = PyDict_New();
    put(d, "mode", PyUnicode_FromString(cvc::ariadne::intent_mode_name(in.command)));
    put(d, "steer", PyFloat_FromDouble(in.steer));
    put(d, "accel", PyFloat_FromDouble(in.accel));
    put(d, "weight", PyFloat_FromDouble(in.weight));
    put(d, "force", Py_BuildValue("(dd)", in.force_x, in.force_y));
    put(d, "force_steers", PyBool_FromLong(in.force_steers ? 1 : 0));
    return d;
  }
  static PyObject *pose_dict(const cvc::ariadne::entity_pose &p) {
    PyObject *d = PyDict_New();
    const auto opt = [](double v) {
      if (std::isnan(v))
        Py_RETURN_NONE;
      return PyFloat_FromDouble(v);
    };
    put(d, "x", PyFloat_FromDouble(p.x));
    put(d, "y", PyFloat_FromDouble(p.y));
    put(d, "z", opt(p.z));
    put(d, "heading", PyFloat_FromDouble(p.heading));
    put(d, "pitch", PyFloat_FromDouble(p.pitch));
    put(d, "roll", PyFloat_FromDouble(p.roll));
    put(d, "speed", PyFloat_FromDouble(p.speed));
    put(d, "steer", PyFloat_FromDouble(p.steer));
    put(d, "t", opt(p.time));
    put(d, "rear_axle", PyBool_FromLong(p.rear_axle ? 1 : 0));
    return d;
  }
  // A Python number into `out`: false for None (not given); throws for anything else.
  static bool number(PyObject *o, const char *name, double &out,
                     const char *who = "set_intent") {
    if (!o || o == Py_None)
      return false;
    const double x = PyFloat_AsDouble(o);
    if (PyErr_Occurred() || !std::isfinite(x)) {
      PyErr_Clear();
      throw std::invalid_argument(std::string("pycvc.WorldEntities.") + who + ": " + name +
                                  " must be a finite number or None");
    }
    out = x;
    return true;
  }
  std::shared_ptr<cvc::app> app_;
  cvc::ariadne::AppRuntime *rt_ = nullptr;
  std::string why_;
};

// A GL-free Ariadne host: an AppRuntime over `app`'s state at `prefix`, with no backend and no
// scene. Its methods mirror pycvc_gl.AriRuntime's, so pycvc.ari.Host drives either.
class AriApp {
public:
  AriApp(std::shared_ptr<cvc::app> app, const std::string &prefix)
      : app_(std::move(app)), prefix_(prefix) {
    if (!app_)
      throw std::invalid_argument("pycvc.AriApp: app is None");
    if (prefix_.empty())
      throw std::invalid_argument("pycvc.AriApp: the prefix is empty");
    // Build the app's lazily made singletons on this thread before a pool thread needs them.
    app_->exec_scheduler();
    app_->exec_async_host();
    owned_ = std::make_unique<cvc::ariadne::AppRuntime>(*app_, prefix_);
    rt_ = owned_.get();
  }
  // Over a runtime someone else owns (ari_app_attach): `rt` outlives this AriApp.
  AriApp(std::shared_ptr<cvc::app> app, cvc::ariadne::AppRuntime *rt)
      : app_(std::move(app)), prefix_(rt ? rt->prefix() : std::string()), rt_(rt) {
    if (!app_ || !rt_)
      throw std::invalid_argument("pycvc.ari_app_attach: no app or no runtime");
  }
  ~AriApp() { owned_.reset(); }

  void add_component_path(const std::string &dir) { rt_->add_component_path(dir); }
  void on(const std::string &event, PyObject *handler) {
    rt_->on(event, ariadne_void_callback(handler, "AriApp.on"));
  }
  void register_host_function(const host_function &f) {
    if (f.is_async())
      rt_->register_async_function(f.name, f.marshal, f.done_channel);
    else
      rt_->register_function(f.name, f.call);
  }
  void register_quit() { rt_->register_quit(); }
  void set_document_args(const std::vector<std::string> &argv, const std::string &invocation,
                         bool validate) {
    rt_->set_document_args(argv, invocation, validate);
  }
  std::vector<std::string> load(const std::string &path) { return rt_->load(path); }
  void frame(double dt_seconds) {
    (void)dt_seconds; // no camera, no scene clock: the drain is the frame
    if (inFrame_)
      throw std::logic_error("AriApp::frame: called from inside a frame (a host handler or "
                             "function)");
    inFrame_ = true;
    try {
      rt_->drain();
    } catch (...) {
      inFrame_ = false;
      throw;
    }
    inFrame_ = false;
  }
  void render() {} // nothing is drawn headless
  void step(double dt_seconds) { frame(dt_seconds); }
  bool should_close() const { return rt_->should_close(); }
  void request_close() { rt_->request_close(); }
  int exit_code() const { return rt_->exit_code(); }
  bool idle() const { return rt_->idle(); }
  void run_program(const std::string &script, PyObject *done) {
    rt_->run_program(script, ariadne_done_callback(done, "AriApp.run_program"));
  }
  bool post_signal(const std::string &signal, const std::string &source) {
    return rt_->post_signal(signal, source);
  }
  std::vector<std::string> take_warnings() { return rt_->take_warnings(); }
  bool reload_if_changed() { return rt_->reload_if_changed(); }
  // sim.step / sim.restart, as the document's Step and Restart buttons raise them
  // (AppRuntime::step_world / restart_world).
  void step_world(unsigned long long n = 1) { rt_->step_world(n); }
  bool restart_world() { return rt_->restart_world(); }
  // The navigator's grid of the world scene node `node` makes (a generated world, a dataset, a
  // terrain, a flat ground): SceneWorld::grid on the scene's world wherever it runs, this runtime
  // or its renderer's adapter (AppRuntime::running_world), so an AriApp attached to a drawn host
  // reads it too. None when the document runs no world, or the node has no world yet.
  world_grid *grid(const std::string &node, double cell, double half = 0.0) {
    cvc::ariadne::SceneWorld *w = rt_->running_world();
    if (!w)
      return nullptr;
    std::shared_ptr<const cvc::nav::world_grid> g = w->grid(node, cell, half);
    return g ? new world_grid(*g) : nullptr;
  }
  // The world scene node `node` makes at run time (a generated world's colliders, a dataset's
  // world): SceneWorld::world_geometry on the scene's world wherever it runs, a copy sharing its
  // arrays (as pycvc.data_world's). What a host's own physics collides with when it is not in that
  // world. None when the document runs no world, or the node has none (yet).
  cvc::world::world_geometry *world_geometry(const std::string &node) {
    cvc::ariadne::SceneWorld *w = rt_->running_world();
    if (!w)
      return nullptr;
    std::shared_ptr<const cvc::world::world_geometry> g = w->world_geometry(node);
    return g ? new cvc::world::world_geometry(*g) : nullptr;
  }
  std::string prefix() const { return prefix_; }
  // The running world's entities (pycvc.WorldEntities), over this runtime.
  WorldEntities *entities() { return new WorldEntities(app_, rt_); }
  bool has_function(const std::string &name) { return rt_->runtime().has_function(name); }
  void wait_idle() {
    if (inFrame_)
      throw std::logic_error("AriApp::wait_idle: called from inside a frame");
    rt_->wait_idle(); // the app's host and the document's own group's (S1.9)
  }
  static bool have_yaml() { return cvc::ariadne::AppRuntime::have_yaml(); }

private:
  std::shared_ptr<cvc::app> app_;
  std::string prefix_;
  std::unique_ptr<cvc::ariadne::AppRuntime> owned_; // null when attached
  cvc::ariadne::AppRuntime *rt_ = nullptr;
  bool inFrame_ = false;
};

// The entities of the world the AppRuntime in a "cvc.ariadne.AppRuntime" capsule runs (a GL
// host's, pycvc_gl.AriRuntime.app_runtime_capsule(); a scripting host's, _arihost.runtime()).
WorldEntities *world_entities(std::shared_ptr<cvc::app> app, PyObject *runtime) {
  if (!runtime || !PyCapsule_CheckExact(runtime))
    throw std::invalid_argument("pycvc.world_entities: runtime is not a PyCapsule");
  void *p = PyCapsule_GetPointer(runtime, "cvc.ariadne.AppRuntime");
  if (!p) {
    PyErr_Clear();
    throw std::invalid_argument(
        "pycvc.world_entities: the capsule is not a \"cvc.ariadne.AppRuntime\"");
  }
  return new WorldEntities(std::move(app), static_cast<cvc::ariadne::AppRuntime *>(p));
}

// An AriApp over the AppRuntime in a "cvc.ariadne.AppRuntime" capsule (a scripting host's,
// _arihost.runtime()), not owning it.
AriApp *ari_app_attach(std::shared_ptr<cvc::app> app, PyObject *runtime) {
  if (!runtime || !PyCapsule_CheckExact(runtime))
    throw std::invalid_argument("pycvc.ari_app_attach: runtime is not a PyCapsule");
  void *p = PyCapsule_GetPointer(runtime, "cvc.ariadne.AppRuntime");
  if (!p) {
    PyErr_Clear();
    throw std::invalid_argument(
        "pycvc.ari_app_attach: the capsule is not a \"cvc.ariadne.AppRuntime\"");
  }
  return new AriApp(std::move(app), static_cast<cvc::ariadne::AppRuntime *>(p));
}

} // namespace pycvc
%}

// The long calls release the GIL (a Python function the document calls retakes it), and so does
// the destructor: ~AppRuntime ends the document's processes. pycvc::gil_release is pycvc_lod.i's.
// File scope, so the names are qualified.
%define PYCVC_ARIADNE_NOGIL(NAME)
%exception NAME {
  try {
    pycvc::gil_release pycvc_nogil;
    $action
  }
  PYCVC_STATE_EXEC_CATCH
}
%enddef
PYCVC_ARIADNE_NOGIL(pycvc::AriApp::frame)
PYCVC_ARIADNE_NOGIL(pycvc::AriApp::step)
PYCVC_ARIADNE_NOGIL(pycvc::AriApp::load)
PYCVC_ARIADNE_NOGIL(pycvc::AriApp::wait_idle)
PYCVC_ARIADNE_NOGIL(pycvc::AriApp::~AriApp)

%feature("kwargs") pycvc::make_host_function;
%rename(host_function) pycvc::make_host_function;
%rename(HostFunction) pycvc::host_function;
%nodefaultctor pycvc::host_function;
%feature("kwargs") pycvc::AriApp::AriApp;
%newobject pycvc::ari_app_attach;
%feature("kwargs") pycvc::AriApp::run_program;
%feature("kwargs") pycvc::AriApp::post_signal;
%feature("kwargs") pycvc::AriApp::set_document_args;
// AriApp keeps its app alive in C++ (a shared_ptr), and its handlers / functions their callables.
// What SWIG sees: the data the C++ struct and class hold stay out of the proxy.
namespace pycvc {
struct host_function {
  %immutable;
  std::string name;
  std::string done_channel;
  %mutable;
  bool is_async() const;
};
host_function make_host_function(const std::string &name, PyObject *fn, bool is_async = false,
                                 bool with_call = false, const std::string &done_channel = "");

%newobject pycvc::AriApp::grid;
%feature("kwargs") pycvc::AriApp::grid;
%newobject pycvc::AriApp::world_geometry;
%newobject pycvc::AriApp::entities;
%newobject pycvc::world_entities;
%feature("kwargs") WorldEntities::spawn;
%feature("kwargs") WorldEntities::set_intent;
%feature("kwargs") WorldEntities::set_pose;
%feature("kwargs") WorldEntities::set_poses;
%nodefaultctor WorldEntities;
class WorldEntities {
public:
  bool available() const;
  PyObject *list() const;
  unsigned long long spawn(const std::string &archetype, const std::string &name = "",
                           const std::string &controller = "", int team = 0,
                           PyObject *at = nullptr, double mass = 0.0, double max_speed = 0.0);
  std::string why() const;
  bool despawn(unsigned long long id);
  bool set_intent(unsigned long long id, PyObject *steer = nullptr, PyObject *accel = nullptr,
                  const std::string &mode = "", double weight = 0.5, PyObject *force = nullptr,
                  bool force_steers = true);
  PyObject *intent(unsigned long long id) const;
  bool clear_intent(unsigned long long id);
  bool set_pose(unsigned long long id, double x, double y, PyObject *z = nullptr,
                double heading = 0.0, double pitch = 0.0, double roll = 0.0, double speed = 0.0,
                double steer = 0.0, PyObject *t = nullptr, bool rear_axle = false);
  unsigned long long set_poses(PyObject *ids, PyObject *poses, bool rear_axle = false);
  PyObject *pose(unsigned long long id) const;
  bool set_pose_jump(double metres);
  bool set_goal(unsigned long long id, double x, double y);
  bool set_wander(bool wander);
  PyObject *grid() const;
};
WorldEntities *world_entities(std::shared_ptr<cvc::app> app, PyObject *runtime);
class AriApp {
public:
  AriApp(std::shared_ptr<cvc::app> app, const std::string &prefix = "ari");
  ~AriApp();
  void add_component_path(const std::string &dir);
  void on(const std::string &event, PyObject *handler);
  void register_host_function(const host_function &f);
  void register_quit();
  void set_document_args(const std::vector<std::string> &argv, const std::string &invocation = "",
                         bool validate = true);
  std::vector<std::string> load(const std::string &path);
  void frame(double dt_seconds);
  void render();
  void step(double dt_seconds);
  bool should_close() const;
  void request_close();
  int exit_code() const;
  bool idle() const;
  void run_program(const std::string &script, PyObject *done = nullptr);
  bool post_signal(const std::string &signal, const std::string &source = "host");
  std::vector<std::string> take_warnings();
  bool reload_if_changed();
  void step_world(unsigned long long n = 1);
  bool restart_world();
  world_grid *grid(const std::string &node, double cell, double half = 0.0);
  cvc::world::world_geometry *world_geometry(const std::string &node);
  std::string prefix() const;
  WorldEntities *entities();
  bool has_function(const std::string &name);
  void wait_idle();
  static bool have_yaml();
};
AriApp *ari_app_attach(std::shared_ptr<cvc::app> app, PyObject *runtime);
} // namespace pycvc
