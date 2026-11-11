// pycvc_physics.i — SWIG surface for cvc::physics and the navigator's plant (P6 of
// docs/roadmap/NAV-VEHICLE-PHYSICS-ROADMAP.md). Python drives the same world, vehicles and plant as
// C++ does, and reads the same values:
//
//   w = pycvc.physics_world(backend="jolt", fixed_dt=0.01)
//   ground = w.add_material("ground", friction=0.9, rolling_resistance=0.015)
//   w.add_box((100, 100, 0.5), position=(0, 0, -0.5), motion="static", material=ground)
//   car = w.add_vehicle(pycvc.vehicle_desc.humvee(), position=(0, 0, 1.2))
//   w.drive(car, steer=0.05, accel=1.5)
//   w.step(100)
//   w.vehicle_state(car)["energy"]["source"]   # J drawn from the fuel
//   w.state_hash(), w.snapshot_json()
//
//   water = pycvc.volume(app); water.set_float_grid([1000.0] * 8, 2, 2, 2, -5, -5, -5, 5, 5, 1)
//   sea = pycvc.wave_train_surface(trains=[{"amplitude": 0.2, "period_s": 4.0}])
//   lake = w.add_field(water, kind="fluid", damping=1e4, surface=sea)   # field.h
//   w.follow(clock); w.field_time()       # the surface is read at the clock's time
//
//   plant = pycvc.nav_plant(w, [pycvc.vehicle_desc.humvee()])
//   pycvc.nav_sim_world_set_plant(sim, plant)   # sim's agents now drive Humvees
//   pycvc.nav_sim_world_set_plant(sim, None)    # back to the kinematic bicycle
//
// NOT a standalone module: %include'd by pycvc.i after pycvc_nav.i (the sim_world handles) and
// pycvc_world.i (height_field, for terrain bodies).
//
// SWIG APPROACH (as pycvc_world.i):
//  * The classes are DECLARED here by hand, as the subset Python gets, and every call goes to the
//    real libcvc headers (#included below). world.h's variants, optionals and move-only types do
//    not marshal.
//  * Handles (bodies, vehicles) cross as ints, the id's value; a stale id raises, as in C++.
//  * Vectors go in as any 3-sequence, quaternions as (w, x, y, z); state comes back as dicts of
//    tuples and floats, the same numbers C++ reads (snapshot_json() is C++'s own serializer).
//  * Lifetimes: a nav_plant (and a dyn_bike_plant given a world) keeps its physics_world alive, a
//    physics_binding its world and app, and a sim_world handle the plant set on it. A field
//    copies its volumes and shares its surface (immutable, so sharing is safe).

%{
#include <cvc/core/world_clock.h>
#include <cvc/geometry/geometry.h>
#include <cvc/geometry/height_field.h>
#include <cvc/nav/plant_types.h>
#include <cvc/nav/sim_world.h>
#include <cvc/nav/terrain.h>
#include <cvc/physics/backend.h>
#include <cvc/physics/dyn_bike_plant.h>
#include <cvc/physics/field.h>
#include <cvc/physics/fluid_surface.h>
#include <cvc/physics/nav_plant.h>
#include <cvc/physics/plant_host.h>
#include <cvc/physics/snapshot_json.h>
#include <cvc/physics/vehicle.h>
#include <cvc/physics/world.h>
#include <cvc/physics/world_binding.h>
#include <cvc/volume/volume.h>
#include <cmath>
#include <cstring>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace pycvc_physics {

namespace ph = cvc::physics;

// A 3-sequence of numbers. None: `fallback`.
ph::vec3 vec3_of(PyObject *o, const char *what, ph::vec3 fallback = {}) {
  if (!o || o == Py_None)
    return fallback;
  PyObject *seq = PySequence_Fast(o, what);
  if (!seq || PySequence_Fast_GET_SIZE(seq) != 3) {
    Py_XDECREF(seq);
    PyErr_Clear();
    throw std::invalid_argument(std::string(what) + ": expected (x, y, z)");
  }
  double v[3];
  for (int i = 0; i < 3; ++i) {
    v[i] = PyFloat_AsDouble(PySequence_Fast_GET_ITEM(seq, i));
    if (PyErr_Occurred()) {
      Py_DECREF(seq);
      PyErr_Clear();
      throw std::invalid_argument(std::string(what) + ": expected numbers");
    }
  }
  Py_DECREF(seq);
  return {v[0], v[1], v[2]};
}

// A quaternion (w, x, y, z). None: the identity.
ph::quat quat_of(PyObject *o, const char *what) {
  if (!o || o == Py_None)
    return {};
  PyObject *seq = PySequence_Fast(o, what);
  if (!seq || PySequence_Fast_GET_SIZE(seq) != 4) {
    Py_XDECREF(seq);
    PyErr_Clear();
    throw std::invalid_argument(std::string(what) + ": expected a quaternion (w, x, y, z)");
  }
  double v[4];
  for (int i = 0; i < 4; ++i) {
    v[i] = PyFloat_AsDouble(PySequence_Fast_GET_ITEM(seq, i));
    if (PyErr_Occurred()) {
      Py_DECREF(seq);
      PyErr_Clear();
      throw std::invalid_argument(std::string(what) + ": expected numbers");
    }
  }
  Py_DECREF(seq);
  return {v[0], v[1], v[2], v[3]};
}

PyObject *tuple3(const ph::vec3 &v) { return Py_BuildValue("(ddd)", v.x, v.y, v.z); }
PyObject *tuple4(const ph::quat &q) { return Py_BuildValue("(dddd)", q.w, q.x, q.y, q.z); }

// Steals `v`; raises if it is null.
void put(PyObject *d, const char *key, PyObject *v) {
  if (!v)
    throw std::runtime_error("pycvc: out of memory building a dict");
  PyDict_SetItemString(d, key, v);
  Py_DECREF(v);
}

ph::backend backend_of(const std::string &s) {
  if (s == "jolt")
    return ph::backend::jolt;
  if (s == "null" || s == "none")
    return ph::backend::null;
  throw std::invalid_argument("physics: backend is 'jolt' or 'null'");
}

ph::motion_type motion_of(const std::string &s) {
  if (s == "dynamic")
    return ph::motion_type::dynamic;
  if (s == "static")
    return ph::motion_type::static_;
  if (s == "kinematic")
    return ph::motion_type::kinematic;
  throw std::invalid_argument("physics: motion is 'dynamic', 'static' or 'kinematic'");
}

ph::material_id material_of(int m) {
  if (m < 0 || m > 65535)
    throw std::invalid_argument("physics: a material id is in 0..65535");
  return static_cast<ph::material_id>(m);
}

std::vector<ph::material_id> materials_of(PyObject *o, const char *what) {
  std::vector<ph::material_id> out;
  if (!o || o == Py_None)
    return out;
  PyObject *seq = PySequence_Fast(o, what);
  if (!seq) {
    PyErr_Clear();
    throw std::invalid_argument(std::string(what) + ": expected a sequence of material ids");
  }
  for (Py_ssize_t i = 0; i < PySequence_Fast_GET_SIZE(seq); ++i) {
    const long v = PyLong_AsLong(PySequence_Fast_GET_ITEM(seq, i));
    if (PyErr_Occurred()) {
      Py_DECREF(seq);
      PyErr_Clear();
      throw std::invalid_argument(std::string(what) + ": expected integers");
    }
    out.push_back(material_of(static_cast<int>(v)));
  }
  Py_DECREF(seq);
  return out;
}

ph::body_id body_of(long id) {
  ph::body_id b;
  b.value = static_cast<std::uint32_t>(id);
  return b;
}

// A sequence of body ids (None: none), for a plant's ground_bodies; `who` names the caller in the
// ValueError.
std::vector<ph::body_id> body_ids(PyObject *ids, const std::string &who) {
  std::vector<ph::body_id> out;
  if (!ids || ids == Py_None)
    return out;
  const std::string bad = who + ": ground_bodies is a sequence of body ids";
  PyObject *seq = PySequence_Fast(ids, bad.c_str());
  if (!seq) {
    PyErr_Clear();
    throw std::invalid_argument(bad);
  }
  const Py_ssize_t n = PySequence_Fast_GET_SIZE(seq);
  for (Py_ssize_t k = 0; k < n; ++k) {
    const long id = PyLong_AsLong(PySequence_Fast_GET_ITEM(seq, k));
    if (PyErr_Occurred()) {
      PyErr_Clear();
      Py_DECREF(seq);
      throw std::invalid_argument(bad);
    }
    out.push_back(body_of(id));
  }
  Py_DECREF(seq);
  return out;
}

ph::vehicle_id vehicle_of(long id) {
  ph::vehicle_id v;
  v.value = static_cast<std::uint32_t>(id);
  return v;
}

PyObject *body_state_dict(const ph::body_state &s) {
  PyObject *d = PyDict_New();
  put(d, "position", tuple3(s.position));
  put(d, "rotation", tuple4(s.rotation));
  put(d, "linear_velocity", tuple3(s.linear_velocity));
  put(d, "angular_velocity", tuple3(s.angular_velocity));
  put(d, "active", PyBool_FromLong(s.active ? 1 : 0));
  return d;
}

PyObject *energy_dict(const ph::vehicle_energy &e) {
  PyObject *d = PyDict_New();
  const std::pair<const char *, double> v[] = {
      {"supplied", e.supplied},
      {"idle", e.idle},
      {"brake", e.brake},
      {"tire", e.tire},
      {"aero", e.aero},
      {"rolling", e.rolling},
      {"medium", e.medium},
      {"drivetrain", e.drivetrain},
      {"suspension", e.suspension},
      {"chassis_damping", e.chassis_damping},
      {"fields", e.fields},
      {"drivetrain_split", e.drivetrain_split},
      {"stored", e.stored},
      {"residual", e.residual},
      {"kinetic", e.kinetic},
      {"potential", e.potential},
      {"spring", e.spring},
      {"engine", e.engine},
      {"regen", e.regen},
      {"source", e.source},
      {"charge", e.charge},
      {"fuel_used", e.fuel_used},
      {"fuel_left", e.fuel_left},
      {"distance", e.distance},
      {"power", e.power},
  };
  for (const auto &kv : v)
    put(d, kv.first, PyFloat_FromDouble(kv.second));
  return d;
}

PyObject *wheel_dict(const ph::wheel_state &w) {
  PyObject *d = PyDict_New();
  const std::pair<const char *, double> v[] = {
      {"steer", w.steer},
      {"angular_velocity", w.angular_velocity},
      {"rotation", w.rotation},
      {"suspension_length", w.suspension_length},
      {"load", w.load},
      {"longitudinal_slip", w.longitudinal_slip},
      {"lateral_slip", w.lateral_slip},
      {"friction", w.friction},
      {"rolling_resistance", w.rolling_resistance},
      {"medium_depth", w.medium_depth},
      {"sinkage", w.sinkage},
      {"rut", w.rut},
      {"medium_resistance", w.medium_resistance},
  };
  for (const auto &kv : v)
    put(d, kv.first, PyFloat_FromDouble(kv.second));
  put(d, "contact", PyBool_FromLong(w.contact ? 1 : 0));
  return d;
}

PyObject *vehicle_state_dict(const ph::vehicle_state &s) {
  PyObject *d = body_state_dict(s.body);
  put(d, "chassis", PyLong_FromUnsignedLong(s.chassis.value));
  put(d, "speed", PyFloat_FromDouble(s.speed));
  put(d, "steer", PyFloat_FromDouble(s.steer));
  put(d, "steer_command", PyFloat_FromDouble(s.steer_command));
  put(d, "throttle", PyFloat_FromDouble(s.throttle));
  put(d, "brake", PyFloat_FromDouble(s.brake));
  put(d, "hand_brake", PyFloat_FromDouble(s.hand_brake));
  put(d, "gear", PyLong_FromLong(s.gear));
  put(d, "engine_rpm", PyFloat_FromDouble(s.engine_rpm));
  put(d, "accel_trim", PyFloat_FromDouble(s.accel_trim));
  put(d, "aero_force", tuple3(s.aero_force));
  put(d, "air_velocity", tuple3(s.air_velocity));
  PyObject *wheels = PyList_New(static_cast<Py_ssize_t>(s.wheels.size()));
  for (std::size_t i = 0; i < s.wheels.size(); ++i)
    PyList_SET_ITEM(wheels, static_cast<Py_ssize_t>(i), wheel_dict(s.wheels[i]));
  put(d, "wheels", wheels);
  put(d, "energy", energy_dict(s.energy));
  return d;
}

PyObject *contact_dict(const ph::contact_event &c) {
  static const char *kinds[] = {"added", "persisted", "removed"};
  PyObject *d = PyDict_New();
  put(d, "type", PyUnicode_FromString(kinds[static_cast<int>(c.type)]));
  put(d, "body_a", PyLong_FromUnsignedLong(c.body_a.value));
  put(d, "body_b", PyLong_FromUnsignedLong(c.body_b.value));
  put(d, "material_a", PyLong_FromLong(c.material_a));
  put(d, "material_b", PyLong_FromLong(c.material_b));
  put(d, "friction", PyFloat_FromDouble(c.friction));
  put(d, "restitution", PyFloat_FromDouble(c.restitution));
  put(d, "point", tuple3(c.point));
  put(d, "normal", tuple3(c.normal));
  put(d, "depth", PyFloat_FromDouble(c.depth));
  return d;
}

PyObject *report_dict(const ph::step_report &r) {
  PyObject *d = PyDict_New();
  put(d, "tick", PyLong_FromUnsignedLongLong(r.tick));
  put(d, "step_ms", PyFloat_FromDouble(r.step_ms));
  put(d, "contacts", PyLong_FromSize_t(r.contacts));
  put(d, "active_bodies", PyLong_FromUnsignedLong(r.active_bodies));
  put(d, "saturated", PyBool_FromLong(r.saturated ? 1 : 0));
  return d;
}

// A body description from the add_* keyword arguments.
ph::body_desc desc_of(ph::shape shape, PyObject *position, PyObject *rotation,
                      const std::string &motion, int material, double mass, double density,
                      PyObject *linear_velocity, PyObject *angular_velocity, double linear_damping,
                      double angular_damping, bool allow_sleeping) {
  ph::body_desc d;
  d.shape = std::move(shape);
  d.position = vec3_of(position, "physics_world.add_*: position");
  d.rotation = quat_of(rotation, "physics_world.add_*: rotation");
  d.motion = motion_of(motion);
  d.material = material_of(material);
  d.mass = mass;
  d.density = density;
  d.linear_velocity = vec3_of(linear_velocity, "physics_world.add_*: linear_velocity");
  d.angular_velocity = vec3_of(angular_velocity, "physics_world.add_*: angular_velocity");
  d.linear_damping = linear_damping;
  d.angular_damping = angular_damping;
  d.allow_sleeping = allow_sleeping;
  return d;
}

ph::field_id field_id_of(long id) {
  ph::field_id f;
  f.value = static_cast<std::uint32_t>(id);
  return f;
}

ph::field_kind field_kind_of(const std::string &s) {
  if (s == "fluid")
    return ph::field_kind::fluid;
  if (s == "medium")
    return ph::field_kind::medium;
  if (s == "flow")
    return ph::field_kind::flow;
  throw std::invalid_argument("physics: a field's kind is 'fluid', 'medium' or 'flow'");
}

// A pycvc.volume, copied; anything else raises with `bad`.
std::shared_ptr<const cvc::volume> volume_of(PyObject *o, const char *bad) {
  void *p = nullptr;
  if (!o || !SWIG_IsOK(SWIG_ConvertPtr(o, &p, SWIGTYPE_p_cvc__volume, 0)) || !p)
    throw std::invalid_argument(bad);
  return std::make_shared<cvc::volume>(*static_cast<const cvc::volume *>(p));
}

// A field from add_field's and set_field's keywords. `velocity`: None, or three pycvc.volume
// (x, y, z). The volumes are copied, the surface shared.
ph::field field_desc_of(const cvc::volume &volume, const std::string &kind, PyObject *velocity,
                        double drag_coefficient, double damping, double traction, bool ruts,
                        double rut_cell, std::shared_ptr<const ph::fluid_surface> surface) {
  ph::field f;
  f.kind = field_kind_of(kind);
  f.volume = std::make_shared<cvc::volume>(volume);
  if (velocity && velocity != Py_None) {
    const char *bad = "physics_world: velocity is three pycvc.volume, one per axis (x, y, z)";
    PyObject *seq = PySequence_Fast(velocity, bad);
    if (!seq || PySequence_Fast_GET_SIZE(seq) != 3) {
      Py_XDECREF(seq);
      PyErr_Clear();
      throw std::invalid_argument(bad);
    }
    try {
      for (int i = 0; i < 3; ++i)
        f.velocity[static_cast<std::size_t>(i)] =
            volume_of(PySequence_Fast_GET_ITEM(seq, i), bad);
    } catch (...) {
      Py_DECREF(seq);
      throw;
    }
    Py_DECREF(seq);
  }
  f.drag_coefficient = drag_coefficient;
  f.damping = damping;
  f.traction = traction;
  f.ruts = ruts;
  f.rut_cell = rut_cell;
  f.surface = std::move(surface);
  return f;
}

PyObject *field_load_dict(const ph::field_load &l) {
  PyObject *d = PyDict_New();
  put(d, "force", tuple3(l.force));
  put(d, "torque", tuple3(l.torque));
  put(d, "immersed_volume", PyFloat_FromDouble(l.immersed_volume));
  return d;
}

// Wave trains from a sequence of dicts; a key a dict lacks keeps wave_train's default.
std::vector<ph::wave_train> wave_trains_of(PyObject *o) {
  std::vector<ph::wave_train> out;
  if (!o || o == Py_None)
    return out;
  const char *bad = "wave_train_surface: trains is a sequence of dicts";
  PyObject *seq = PySequence_Fast(o, bad);
  if (!seq) {
    PyErr_Clear();
    throw std::invalid_argument(bad);
  }
  try {
    for (Py_ssize_t i = 0; i < PySequence_Fast_GET_SIZE(seq); ++i) {
      PyObject *d = PySequence_Fast_GET_ITEM(seq, i);
      if (!PyDict_Check(d))
        throw std::invalid_argument(bad);
      ph::wave_train t;
      PyObject *key, *value;
      Py_ssize_t pos = 0;
      while (PyDict_Next(d, &pos, &key, &value)) {
        const char *k = PyUnicode_AsUTF8(key);
        if (!k) {
          PyErr_Clear();
          throw std::invalid_argument("wave_train_surface: a train's keys are strings");
        }
        const std::string s(k);
        double *slot = s == "amplitude" ? &t.amplitude
                       : s == "period_s" ? &t.period_s
                       : s == "dir_rad"  ? &t.dir_rad
                       : s == "phase"    ? &t.phase
                                         : nullptr;
        if (!slot)
          throw std::invalid_argument("wave_train_surface: a train has no field '" + s + "'");
        *slot = PyFloat_AsDouble(value);
        if (PyErr_Occurred()) {
          PyErr_Clear();
          throw std::invalid_argument("wave_train_surface: a train's " + s + " is a number");
        }
      }
      out.push_back(t);
    }
  } catch (...) {
    Py_DECREF(seq);
    throw;
  }
  Py_DECREF(seq);
  return out;
}

PyObject *wave_train_dict(const ph::wave_train &t) {
  return Py_BuildValue("{s:d,s:d,s:d,s:d}", "amplitude", t.amplitude, "period_s", t.period_s,
                       "dir_rad", t.dir_rad, "phase", t.phase);
}

PyObject *terrain_config_dict(const cvc::nav::terrain_config &c) {
  PyObject *d = PyDict_New();
  put(d, "gravity", PyFloat_FromDouble(c.gravity));
  put(d, "max_grade", PyFloat_FromDouble(c.max_grade));
  put(d, "tip_ratio", PyFloat_FromDouble(c.tip_ratio));
  put(d, "tip_margin", PyFloat_FromDouble(c.tip_margin));
  put(d, "sigma", PyFloat_FromDouble(c.sigma));
  put(d, "lam_grade", PyFloat_FromDouble(c.lam_grade));
  put(d, "lam_grade_hard", PyFloat_FromDouble(c.lam_grade_hard));
  put(d, "k_sharp", PyFloat_FromDouble(c.k_sharp));
  put(d, "d_hat_m", PyFloat_FromDouble(c.d_hat_m));
  put(d, "block_infeasible", PyBool_FromLong(c.block_infeasible ? 1 : 0));
  put(d, "governors", PyBool_FromLong(c.governors ? 1 : 0));
  return d;
}

// A terrain_config from a dict of its fields; keys it lacks keep `base`'s values.
cvc::nav::terrain_config terrain_config_of(PyObject *o, cvc::nav::terrain_config base = {}) {
  if (!o || o == Py_None)
    return base;
  if (!PyDict_Check(o))
    throw std::invalid_argument("terrain config: expected a dict (terrain_config_of's form)");
  PyObject *key, *value;
  Py_ssize_t pos = 0;
  while (PyDict_Next(o, &pos, &key, &value)) {
    const char *k = PyUnicode_AsUTF8(key);
    if (!k) {
      PyErr_Clear();
      throw std::invalid_argument("terrain config: keys are strings");
    }
    const std::string s(k);
    if (s == "block_infeasible" || s == "governors") {
      const int b = PyObject_IsTrue(value);
      if (b < 0) {
        PyErr_Clear();
        throw std::invalid_argument("terrain config: " + s + " is a bool");
      }
      (s == "governors" ? base.governors : base.block_infeasible) = b != 0;
      continue;
    }
    const double v = PyFloat_AsDouble(value);
    if (PyErr_Occurred()) {
      PyErr_Clear();
      throw std::invalid_argument("terrain config: " + s + " is a number");
    }
    if (s == "gravity")
      base.gravity = v;
    else if (s == "max_grade")
      base.max_grade = v;
    else if (s == "tip_ratio")
      base.tip_ratio = v;
    else if (s == "tip_margin")
      base.tip_margin = v;
    else if (s == "sigma")
      base.sigma = v;
    else if (s == "lam_grade")
      base.lam_grade = static_cast<float>(v);
    else if (s == "lam_grade_hard")
      base.lam_grade_hard = static_cast<float>(v);
    else if (s == "k_sharp")
      base.k_sharp = static_cast<float>(v);
    else if (s == "d_hat_m")
      base.d_hat_m = static_cast<float>(v);
    else
      throw std::invalid_argument("terrain config: no field '" + s + "'");
  }
  return base;
}

PyObject *plant_state_dict(const cvc::nav::plant_state &s) {
  PyObject *d = PyDict_New();
  put(d, "x", PyFloat_FromDouble(s.x));
  put(d, "y", PyFloat_FromDouble(s.y));
  put(d, "heading", PyFloat_FromDouble(s.heading));
  put(d, "speed", PyFloat_FromDouble(s.speed));
  put(d, "steer", PyFloat_FromDouble(s.steer));
  put(d, "tire_use", PyFloat_FromDouble(s.tire_use));
  put(d, "load_transfer", PyFloat_FromDouble(s.load_transfer));
  put(d, "pitch", PyFloat_FromDouble(s.pitch));
  put(d, "roll", PyFloat_FromDouble(s.roll));
  return d;
}

// A dyn_bike_plant's books (J), with flow() as "flow".
PyObject *dyn_books_dict(const ph::dyn_bike_books &b) {
  PyObject *d = PyDict_New();
  const std::pair<const char *, double> v[] = {
      {"kinetic", b.kinetic},
      {"drive", b.drive},
      {"brake", b.brake},
      {"dissipated", b.dissipated},
      {"slip", b.slip},
      {"floor", b.floor},
      {"grade", b.grade},
      {"limiter", b.limiter},
      {"contact", b.contact},
      {"contact_energy", b.contact_energy},
      {"contact_stored", b.contact_stored},
      {"contact_dissipated", b.contact_dissipated},
      {"contact_exchange", b.contact_exchange},
      {"residual", b.residual},
      {"flow", b.flow()},
  };
  for (const auto &kv : v)
    put(d, kv.first, PyFloat_FromDouble(kv.second));
  return d;
}

// The compliant contact's settings (cvc/nav/detail/dyn_contact.h) as a dict, and from one: keys
// it lacks keep their defaults.
PyObject *contact_params_dict(const cvc::nav::diff::dyn_contact_params &p) {
  return Py_BuildValue("{s:d,s:d,s:d,s:d,s:d,s:i,s:i,s:d}", "k", p.k, "c", p.c, "friction",
                       p.friction, "slide", p.slide, "skin", p.skin, "per_side", p.per_side,
                       "per_end", p.per_end, "rounding", p.rounding);
}

cvc::nav::diff::dyn_contact_params contact_params_of(PyObject *o) {
  cvc::nav::diff::dyn_contact_params p;
  if (!o || o == Py_None)
    return p;
  if (!PyDict_Check(o))
    throw std::invalid_argument("dyn_bike_plant: contact_params is a dict");
  PyObject *key, *value;
  Py_ssize_t pos = 0;
  while (PyDict_Next(o, &pos, &key, &value)) {
    const char *k = PyUnicode_AsUTF8(key);
    if (!k) {
      PyErr_Clear();
      throw std::invalid_argument("dyn_bike_plant: contact_params' keys are strings");
    }
    const std::string s(k);
    const double v = PyFloat_AsDouble(value);
    if (PyErr_Occurred()) {
      PyErr_Clear();
      throw std::invalid_argument("dyn_bike_plant: contact_params' " + s + " is a number");
    }
    if (s == "per_side" || s == "per_end") {
      if (v != std::floor(v))
        throw std::invalid_argument("dyn_bike_plant: contact_params' " + s +
                                    " is a whole number");
      (s == "per_side" ? p.per_side : p.per_end) = static_cast<int>(v);
    } else if (s == "k")
      p.k = v;
    else if (s == "c")
      p.c = v;
    else if (s == "friction")
      p.friction = v;
    else if (s == "slide")
      p.slide = v;
    else if (s == "skin")
      p.skin = v;
    else if (s == "rounding")
      p.rounding = v;
    else
      throw std::invalid_argument("dyn_bike_plant: contact_params has no field '" + s + "'");
  }
  return p;
}

// A compliant-contact field from a sim_world's signed distance (plane `plane`) as it is now.
// dyn_bike_plant::sdf_field borrows its grid, and the world rebuilds that grid when its map
// changes, so the field keeps a copy of the plane: obstacles the world learns later are not in it.
std::function<void(double, double, double &, double &, double &)>
sdf_field_snapshot(const cvc::nav::field_stack &fs, int plane, double metres_per_world_unit) {
  if (!fs.data || fs.H < 2 || fs.W < 2 || plane < 0 || plane >= fs.M)
    throw std::invalid_argument("dyn_bike_plant: the sim_world has no signed-distance plane " +
                                std::to_string(plane));
  const std::size_t n = static_cast<std::size_t>(3) * fs.H * fs.W;
  auto grid = std::make_shared<std::vector<float>>(fs.data + plane * n, fs.data + (plane + 1) * n);
  cvc::nav::field_stack one = fs;
  one.data = grid->data();
  one.M = 1;
  auto f = ph::dyn_bike_plant::sdf_field(one, 0, metres_per_world_unit);
  return [grid, f](double x, double y, double &phi, double &gx, double &gy) {
    f(x, y, phi, gx, gy);
  };
}

} // namespace pycvc_physics
%}

// ── backends ──────────────────────────────────────────────────────────────────

%inline %{
namespace pycvc {
// The backends this build has ("null", "jolt").
std::vector<std::string> physics_backends() {
  std::vector<std::string> out;
  for (const cvc::physics::backend b : cvc::physics::available_backends())
    out.push_back(cvc::physics::backend_name(b));
  return out;
}
// A backend's capabilities as a dict of bools (dynamics, collision, heightfield, mesh, raycast,
// threads, determinism_same_binary, determinism_cross_platform, exact_save_restore, vehicles)
// and its version.
PyObject *physics_backend_info(const std::string &backend) {
  namespace ph = cvc::physics;
  const ph::backend b = pycvc_physics::backend_of(backend);
  const std::uint32_t c = ph::backend_capabilities(b);
  PyObject *d = PyDict_New();
  const std::pair<const char *, std::uint32_t> caps[] = {
      {"dynamics", ph::cap_dynamics},
      {"collision", ph::cap_collision},
      {"heightfield", ph::cap_heightfield},
      {"mesh", ph::cap_mesh},
      {"raycast", ph::cap_raycast},
      {"threads", ph::cap_threads},
      {"determinism_same_binary", ph::cap_determinism_same_binary},
      {"determinism_cross_platform", ph::cap_determinism_cross_platform},
      {"exact_save_restore", ph::cap_exact_save_restore},
      {"vehicles", ph::cap_vehicles},
  };
  for (const auto &kv : caps)
    pycvc_physics::put(d, kv.first, PyBool_FromLong((c & kv.second) ? 1 : 0));
  pycvc_physics::put(d, "available", PyBool_FromLong(ph::backend_available(b) ? 1 : 0));
  pycvc_physics::put(d, "version", PyUnicode_FromString(ph::backend_version(b).c_str()));
  return d;
}
} // namespace pycvc
%}

// ── cvc::physics::vehicle_desc ────────────────────────────────────────────────

%rename(vehicle_desc) cvc::physics::vehicle_desc;

namespace cvc {
namespace physics {
struct vehicle_desc {
  double mass;
  double max_steer, max_steer_rate;
  double engine_max_torque, engine_min_rpm, engine_max_rpm, engine_inertia;
  double reverse_ratio, differential_ratio;
  double shift_up_rpm, shift_down_rpm;
  double clutch_strength, converter_k, converter_stall, converter_coupling;
  double driveline_efficiency;
  double max_accel, max_decel, max_speed;
  double drag_coefficient, frontal_area, air_density, rolling_resistance;
  double tire_peak_slip, tire_peak_slip_angle, tire_sliding;
  bool energy_ledger;
  static vehicle_desc humvee();
  static vehicle_desc roboracer();
};
} // namespace physics
} // namespace cvc

%extend cvc::physics::vehicle_desc {
  // The chassis box's half extents and the centre of mass (body-local), as tuples.
  PyObject *half_extents() const { return pycvc_physics::tuple3($self->half_extents); }
  PyObject *center_of_mass() const { return pycvc_physics::tuple3($self->center_of_mass); }
  // The forward gears' ratios.
  std::vector<double> gear_ratios() const { return $self->gear_ratios; }
  int axle_count() const { return static_cast<int>($self->axles.size()); }
  // An axle as a dict: x, half_track, z, wheel_radius, wheel_width, steered, driven, brake and
  // suspension.
  PyObject *axle(int i) const {
    if (i < 0 || i >= static_cast<int>($self->axles.size()))
      throw std::invalid_argument("vehicle_desc.axle: no axle " + std::to_string(i));
    const cvc::physics::axle_desc &a = $self->axles[static_cast<std::size_t>(i)];
    PyObject *d = PyDict_New();
    pycvc_physics::put(d, "x", PyFloat_FromDouble(a.x));
    pycvc_physics::put(d, "half_track", PyFloat_FromDouble(a.half_track));
    pycvc_physics::put(d, "z", PyFloat_FromDouble(a.z));
    pycvc_physics::put(d, "wheel_radius", PyFloat_FromDouble(a.wheel_radius));
    pycvc_physics::put(d, "wheel_width", PyFloat_FromDouble(a.wheel_width));
    pycvc_physics::put(d, "steered", PyBool_FromLong(a.steered ? 1 : 0));
    pycvc_physics::put(d, "driven", PyBool_FromLong(a.driven ? 1 : 0));
    pycvc_physics::put(d, "max_brake_torque", PyFloat_FromDouble(a.max_brake_torque));
    pycvc_physics::put(d, "suspension_min", PyFloat_FromDouble(a.suspension_min));
    pycvc_physics::put(d, "suspension_max", PyFloat_FromDouble(a.suspension_max));
    pycvc_physics::put(d, "suspension_frequency", PyFloat_FromDouble(a.suspension_frequency));
    pycvc_physics::put(d, "suspension_damping", PyFloat_FromDouble(a.suspension_damping));
    return d;
  }
}

%template(VehicleDescVector) std::vector<cvc::physics::vehicle_desc>;

// ── cvc::physics::fluid_surface (fluid_surface.h) ─────────────────────────────
// A fluid field's free surface: still_surface or wave_train_surface, held by shared_ptr so a
// field shares the one Python made. The surfaces are C++'s; Python cannot subclass them (the
// step reads a surface from its own thread, without the GIL, and hashes its digest).

%shared_ptr(cvc::physics::fluid_surface)
%shared_ptr(cvc::physics::still_surface)
%shared_ptr(cvc::physics::wave_train_surface)
%nodefaultctor cvc::physics::fluid_surface;
%feature("kwargs") cvc::physics::still_surface::still_surface;
%feature("kwargs") cvc::physics::wave_train_surface::wave_train_surface;

namespace cvc {
namespace physics {
class fluid_surface {
public:
  virtual ~fluid_surface();
};
class still_surface : public fluid_surface {
public:
  ~still_surface();
};
class wave_train_surface : public fluid_surface {
public:
  ~wave_train_surface();
};
} // namespace physics
} // namespace cvc

%extend cvc::physics::fluid_surface {
  // The still water's height at (x, y) (m, world z).
  double level(double x, double y) const { return $self->level(x, y); }
  // The surface's height at (x, y) at time t (s): the level plus the waves.
  double height(double x, double y, double t) const { return $self->height(x, y, t); }
  // The water's velocity (m/s) at p = (x, y, z) at time t, as a tuple.
  PyObject *velocity(PyObject *p, double t) const {
    return pycvc_physics::tuple3(
        $self->velocity(pycvc_physics::vec3_of(p, "fluid_surface.velocity: p"), t));
  }
  // The 64-bit digest of everything the three above depend on (state_hash folds it in).
  unsigned long long digest() const { return $self->digest(); }
}

%extend cvc::physics::still_surface {
  // Still water at `level`, flowing at a uniform `current` (m/s, (x, y, z); None: still).
  still_surface(double level, PyObject *current = nullptr) {
    return new cvc::physics::still_surface(
        level, pycvc_physics::vec3_of(current, "still_surface: current"));
  }
}

%extend cvc::physics::wave_train_surface {
  // Linear Airy trains on water `depth` m deep (<= 0: deep water) about `level`. `trains`: dicts
  // of amplitude (m), period_s, dir_rad (from +x toward +y) and phase (rad); a key a dict lacks
  // keeps wave_train's default (0.5 m, 8 s, 0, 0).
  wave_train_surface(double level = 0.0, double depth = 0.0, PyObject *trains = nullptr,
                     double gravity = 9.80665) {
    return new cvc::physics::wave_train_surface(level, depth,
                                                pycvc_physics::wave_trains_of(trains), gravity);
  }
  int train_count() const { return static_cast<int>($self->trains().size()); }
  // Train i as a dict (amplitude, period_s, dir_rad, phase), and its wavenumber (rad/m) and
  // angular frequency (rad/s).
  PyObject *train(int i) const {
    if (i < 0 || i >= static_cast<int>($self->trains().size()))
      throw std::invalid_argument("wave_train_surface: no train " + std::to_string(i));
    return pycvc_physics::wave_train_dict($self->trains()[static_cast<std::size_t>(i)]);
  }
  double wavenumber(int i) const {
    if (i < 0 || i >= static_cast<int>($self->trains().size()))
      throw std::invalid_argument("wave_train_surface: no train " + std::to_string(i));
    return $self->wavenumber(static_cast<std::size_t>(i));
  }
  double omega(int i) const {
    if (i < 0 || i >= static_cast<int>($self->trains().size()))
      throw std::invalid_argument("wave_train_surface: no train " + std::to_string(i));
    return $self->omega(static_cast<std::size_t>(i));
  }
}

// ── cvc::physics::world ───────────────────────────────────────────────────────

%rename(physics_world) cvc::physics::world;
%feature("kwargs") cvc::physics::world::world;
%feature("kwargs") cvc::physics::world::add_material;
%feature("kwargs") cvc::physics::world::add_box;
%feature("kwargs") cvc::physics::world::add_sphere;
%feature("kwargs") cvc::physics::world::add_hull;
%feature("kwargs") cvc::physics::world::add_mesh;
%feature("kwargs") cvc::physics::world::add_terrain;
%feature("kwargs") cvc::physics::world::set_state;
%feature("kwargs") cvc::physics::world::add_vehicle;
%feature("kwargs") cvc::physics::world::drive;
%feature("kwargs") cvc::physics::world::set_controls;
%feature("kwargs") cvc::physics::world::add_field;
%feature("kwargs") cvc::physics::world::set_field;

namespace cvc {
namespace physics {
class world {
public:
  ~world();
};
} // namespace physics
} // namespace cvc

%extend cvc::physics::world {
  // A world: the backend ("jolt" or "null"), seconds per step, gravity (m/s^2), worker threads
  // and the determinism the run needs ("cross_platform" or "same_binary"; a mode the build
  // cannot deliver is refused).
  world(const std::string &backend = "jolt", double fixed_dt = 0.01, PyObject *gravity = nullptr,
        int threads = 0, const std::string &determinism = "cross_platform",
        int collision_steps = 2, int velocity_steps = 10, int position_steps = 2) {
    cvc::physics::world_config c;
    c.backend = pycvc_physics::backend_of(backend);
    c.fixed_dt = fixed_dt;
    c.gravity = pycvc_physics::vec3_of(gravity, "physics_world: gravity", c.gravity);
    c.threads = threads;
    if (determinism == "cross_platform")
      c.determinism = cvc::physics::determinism::cross_platform;
    else if (determinism == "same_binary")
      c.determinism = cvc::physics::determinism::same_binary;
    else
      throw std::invalid_argument("physics_world: determinism is 'cross_platform' or 'same_binary'");
    c.collision_steps = collision_steps;
    c.velocity_steps = velocity_steps;
    c.position_steps = position_steps;
    return new cvc::physics::world(c);
  }

  std::string backend() const { return cvc::physics::backend_name($self->backend()); }
  double fixed_dt() const { return $self->config().fixed_dt; }
  PyObject *gravity() const { return pycvc_physics::tuple3($self->config().gravity); }

  // ── materials ──
  // Material 0 always exists. Returns the new material's id.
  int add_material(const std::string &name, double friction = 1.0, double restitution = 0.0,
                   double rolling_resistance = 0.0, int nav_material = -1) {
    cvc::physics::material m;
    m.name = name;
    m.friction = static_cast<float>(friction);
    m.restitution = static_cast<float>(restitution);
    m.rolling_resistance = static_cast<float>(rolling_resistance);
    m.nav_material = nav_material;
    return $self->add_material(m);
  }
  // A material as a dict: name, friction, restitution, rolling_resistance, nav_material.
  PyObject *material(int id) const {
    const cvc::physics::material m = $self->get_material(pycvc_physics::material_of(id));
    return Py_BuildValue("{s:s,s:d,s:d,s:d,s:i}", "name", m.name.c_str(), "friction",
                         (double)m.friction, "restitution", (double)m.restitution,
                         "rolling_resistance", (double)m.rolling_resistance, "nav_material",
                         m.nav_material);
  }
  int material_count() const { return static_cast<int>($self->material_count()); }

  // ── bodies ── each returns the new body's id.
  long add_box(PyObject *half_extents, PyObject *position = nullptr, PyObject *rotation = nullptr,
               const std::string &motion = "dynamic", int material = 0, double mass = 0.0,
               double density = 1000.0, PyObject *linear_velocity = nullptr,
               PyObject *angular_velocity = nullptr, double linear_damping = 0.05,
               double angular_damping = 0.05, bool allow_sleeping = true) {
    cvc::physics::box b;
    b.half_extents = pycvc_physics::vec3_of(half_extents, "physics_world.add_box: half_extents");
    return $self
        ->add_body(pycvc_physics::desc_of(b, position, rotation, motion, material, mass, density,
                                          linear_velocity, angular_velocity, linear_damping,
                                          angular_damping, allow_sleeping))
        .value;
  }
  long add_sphere(double radius, PyObject *position = nullptr, PyObject *rotation = nullptr,
                  const std::string &motion = "dynamic", int material = 0, double mass = 0.0,
                  double density = 1000.0, PyObject *linear_velocity = nullptr,
                  PyObject *angular_velocity = nullptr, double linear_damping = 0.05,
                  double angular_damping = 0.05, bool allow_sleeping = true) {
    cvc::physics::sphere s;
    s.radius = radius;
    return $self
        ->add_body(pycvc_physics::desc_of(s, position, rotation, motion, material, mass, density,
                                          linear_velocity, angular_velocity, linear_damping,
                                          angular_damping, allow_sleeping))
        .value;
  }
  // A convex hull of points ((n, 3) numbers).
  long add_hull(PyObject *points, PyObject *position = nullptr, PyObject *rotation = nullptr,
                const std::string &motion = "dynamic", int material = 0, double mass = 0.0,
                double density = 1000.0, PyObject *linear_velocity = nullptr,
                PyObject *angular_velocity = nullptr, double linear_damping = 0.05,
                double angular_damping = 0.05, bool allow_sleeping = true) {
    cvc::physics::convex_hull h;
    PyObject *seq = PySequence_Fast(points, "physics_world.add_hull: points");
    if (!seq) {
      PyErr_Clear();
      throw std::invalid_argument("physics_world.add_hull: points is a sequence of (x, y, z)");
    }
    for (Py_ssize_t i = 0; i < PySequence_Fast_GET_SIZE(seq); ++i)
      h.points.push_back(
          pycvc_physics::vec3_of(PySequence_Fast_GET_ITEM(seq, i), "physics_world.add_hull"));
    Py_DECREF(seq);
    return $self
        ->add_body(pycvc_physics::desc_of(h, position, rotation, motion, material, mass, density,
                                          linear_velocity, angular_velocity, linear_damping,
                                          angular_damping, allow_sleeping))
        .value;
  }
  // A triangle mesh (a pycvc.geometry, shared, not copied); `materials` maps its surface ids to
  // world materials (empty: identity). Static or kinematic.
  long add_mesh(const cvc::geometry &geometry, PyObject *materials = nullptr,
                PyObject *position = nullptr, PyObject *rotation = nullptr,
                const std::string &motion = "static") {
    cvc::physics::mesh m;
    m.geometry = std::make_shared<cvc::geometry>(geometry);
    m.materials = pycvc_physics::materials_of(materials, "physics_world.add_mesh: materials");
    return $self
        ->add_body(pycvc_physics::desc_of(m, position, rotation, motion, 0, 0.0, 1000.0, nullptr,
                                          nullptr, 0.05, 0.05, true))
        .value;
  }
  // A height field (a pycvc.height_field, placed by its own grid); `materials` maps its cell
  // materials to world materials (empty: identity).
  long add_terrain(const cvc::height_field &field, PyObject *materials = nullptr,
                   double max_error = 0.005) {
    cvc::physics::terrain t;
    t.field = std::make_shared<cvc::height_field>(field);
    t.materials = pycvc_physics::materials_of(materials, "physics_world.add_terrain: materials");
    t.max_error = max_error;
    return $self
        ->add_body(pycvc_physics::desc_of(t, nullptr, nullptr, "static", 0, 0.0, 1000.0, nullptr,
                                          nullptr, 0.05, 0.05, true))
        .value;
  }
  void remove_body(long id) { $self->remove_body(pycvc_physics::body_of(id)); }
  bool contains(long id) const { return $self->contains(pycvc_physics::body_of(id)); }
  int body_count() const { return static_cast<int>($self->body_count()); }

  // A body's state as a dict: position, rotation (w, x, y, z), linear_velocity,
  // angular_velocity, active.
  PyObject *state(long id) const {
    return pycvc_physics::body_state_dict($self->state(pycvc_physics::body_of(id)));
  }
  // Several bodies' states under one lock, as (tick, [dict or None, ...]).
  PyObject *states(PyObject *ids) const {
    std::vector<cvc::physics::body_id> b;
    PyObject *seq = PySequence_Fast(ids, "physics_world.states: ids");
    if (!seq) {
      PyErr_Clear();
      throw std::invalid_argument("physics_world.states: ids is a sequence of body ids");
    }
    for (Py_ssize_t i = 0; i < PySequence_Fast_GET_SIZE(seq); ++i) {
      const long id = PyLong_AsLong(PySequence_Fast_GET_ITEM(seq, i));
      if (PyErr_Occurred()) {
        Py_DECREF(seq);
        PyErr_Clear();
        throw std::invalid_argument("physics_world.states: ids are integers");
      }
      b.push_back(pycvc_physics::body_of(id));
    }
    Py_DECREF(seq);
    const cvc::physics::state_batch s = $self->states(b);
    PyObject *list = PyList_New(static_cast<Py_ssize_t>(s.states.size()));
    for (std::size_t i = 0; i < s.states.size(); ++i) {
      PyObject *v = s.states[i] ? pycvc_physics::body_state_dict(*s.states[i])
                                : (Py_INCREF(Py_None), Py_None);
      PyList_SET_ITEM(list, static_cast<Py_ssize_t>(i), v);
    }
    PyObject *out = Py_BuildValue("(KO)", (unsigned long long)s.tick, list);
    Py_DECREF(list);
    return out;
  }
  // Set a body's state; what is left out keeps its value.
  void set_state(long id, PyObject *position = nullptr, PyObject *rotation = nullptr,
                 PyObject *linear_velocity = nullptr, PyObject *angular_velocity = nullptr) {
    const cvc::physics::body_id b = pycvc_physics::body_of(id);
    cvc::physics::body_state s = $self->state(b);
    s.position = pycvc_physics::vec3_of(position, "set_state: position", s.position);
    if (rotation && rotation != Py_None)
      s.rotation = pycvc_physics::quat_of(rotation, "set_state: rotation");
    s.linear_velocity =
        pycvc_physics::vec3_of(linear_velocity, "set_state: linear_velocity", s.linear_velocity);
    s.angular_velocity =
        pycvc_physics::vec3_of(angular_velocity, "set_state: angular_velocity", s.angular_velocity);
    $self->set_state(b, s);
  }
  void add_force(long id, PyObject *force) {
    $self->add_force(pycvc_physics::body_of(id), pycvc_physics::vec3_of(force, "add_force"));
  }
  void add_impulse(long id, PyObject *impulse) {
    $self->add_impulse(pycvc_physics::body_of(id), pycvc_physics::vec3_of(impulse, "add_impulse"));
  }

  // ── fields (field.h) ── add_field returns the field's id.
  // Water, snow or wind over `volume`'s box (a pycvc.volume in world metres: kg/m^3 for a fluid
  // or flow, the bearing modulus N/m^3 for a medium). `kind`: "fluid", "medium" or "flow".
  // `velocity`: None (still) or three pycvc.volume (m/s along x, y, z). `surface`: a fluid's
  // free surface (a still_surface or wave_train_surface), read at field_time(); None keeps the
  // box's flat top. The volumes are copied, so changing one later takes effect at set_field.
  long add_field(const cvc::volume &volume, const std::string &kind = "fluid",
                 PyObject *velocity = nullptr, double drag_coefficient = 1.0,
                 double damping = 0.0, double traction = 0.0, bool ruts = false,
                 double rut_cell = 0.05,
                 std::shared_ptr<const cvc::physics::fluid_surface> surface = nullptr) {
    return $self
        ->add_field(pycvc_physics::field_desc_of(volume, kind, velocity, drag_coefficient,
                                                 damping, traction, ruts, rut_cell,
                                                 std::move(surface)))
        .value;
  }
  // New values for a field, as add_field takes them: what is left out takes add_field's
  // default, not the field's old value (a field is set whole, as in C++).
  void set_field(long id, const cvc::volume &volume, const std::string &kind = "fluid",
                 PyObject *velocity = nullptr, double drag_coefficient = 1.0,
                 double damping = 0.0, double traction = 0.0, bool ruts = false,
                 double rut_cell = 0.05,
                 std::shared_ptr<const cvc::physics::fluid_surface> surface = nullptr) {
    $self->set_field(pycvc_physics::field_id_of(id),
                     pycvc_physics::field_desc_of(volume, kind, velocity, drag_coefficient, damping,
                                                  traction, ruts, rut_cell, std::move(surface)));
  }
  void remove_field(long id) { $self->remove_field(pycvc_physics::field_id_of(id)); }
  int field_count() const { return static_cast<int>($self->field_count()); }
  // What the fields did to a body in the last step: force (N), torque (N m, about its centre of
  // mass) and immersed_volume (m^3).
  PyObject *field_load_on(long id) const {
    return pycvc_physics::field_load_dict($self->field_load_on(pycvc_physics::body_of(id)));
  }

  // ── vehicles ── add_vehicle returns the vehicle's id.
  long add_vehicle(const cvc::physics::vehicle_desc &desc, PyObject *position = nullptr,
                   PyObject *rotation = nullptr) {
    return $self
        ->add_vehicle(desc, pycvc_physics::vec3_of(position, "add_vehicle: position"),
                      pycvc_physics::quat_of(rotation, "add_vehicle: rotation"))
        .value;
  }
  void remove_vehicle(long id) { $self->remove_vehicle(pycvc_physics::vehicle_of(id)); }
  int vehicle_count() const { return static_cast<int>($self->vehicle_count()); }
  // The navigator's command: the bicycle angle (rad, + left) and the acceleration (m/s^2) along
  // the commanded direction, through the driver model.
  void drive(long id, double steer = 0.0, double accel = 0.0) {
    cvc::physics::drive_command c;
    c.steer = steer;
    c.accel = accel;
    $self->drive(pycvc_physics::vehicle_of(id), c);
  }
  // The pedals directly.
  void set_controls(long id, double steer = 0.0, double throttle = 0.0, double brake = 0.0,
                    double hand_brake = 0.0, bool neutral = false) {
    cvc::physics::vehicle_controls c;
    c.steer = steer;
    c.throttle = throttle;
    c.brake = brake;
    c.hand_brake = hand_brake;
    c.neutral = neutral;
    $self->set_controls(pycvc_physics::vehicle_of(id), c);
  }
  // The vehicle's state as a dict: the chassis's body state, speed, steer, pedals, gear,
  // engine_rpm, aero, the wheels (a list of dicts) and the energy ledger (a dict, J).
  PyObject *vehicle_state(long id) const {
    return pycvc_physics::vehicle_state_dict($self->vehicle_state(pycvc_physics::vehicle_of(id)));
  }
  cvc::physics::vehicle_desc vehicle_desc_of(long id) const {
    return $self->get_vehicle(pycvc_physics::vehicle_of(id));
  }

  // ── time ──
  // Steps n times; returns the last step's report as a dict.
  PyObject *step(unsigned long long n = 1) {
    cvc::physics::step_report r;
    Py_BEGIN_ALLOW_THREADS r = $self->step(static_cast<std::uint64_t>(n));
    Py_END_ALLOW_THREADS return pycvc_physics::report_dict(r);
  }
  unsigned long long tick() const { return $self->tick(); }
  double time() const { return $self->time(); }
  // The steps due on a pycvc.world_clock since the last call; returns how many ran. The first
  // call anchors at the clock's tick, runs none, and sets the field time to the clock's time.
  unsigned long long follow(const cvc::world_clock &clock) {
    std::uint64_t n = 0;
    Py_BEGIN_ALLOW_THREADS n = $self->follow(clock);
    Py_END_ALLOW_THREADS return n;
  }
  // The time (s) fluid surfaces are read at: the clock's once follow() has anchored, time()
  // before. set_field_time makes it t at the current tick.
  double field_time() const { return $self->field_time(); }
  void set_field_time(double t) { $self->set_field_time(t); }
  // The last step's contact events, as dicts.
  PyObject *contacts() const {
    const std::vector<cvc::physics::contact_event> c = $self->contacts();
    PyObject *list = PyList_New(static_cast<Py_ssize_t>(c.size()));
    for (std::size_t i = 0; i < c.size(); ++i)
      PyList_SET_ITEM(list, static_cast<Py_ssize_t>(i), pycvc_physics::contact_dict(c[i]));
    return list;
  }
  // The first hit along from -> to, as a dict (body, fraction, point, normal, material), or None.
  PyObject *raycast(PyObject *from, PyObject *to) const {
    const auto h = $self->raycast(pycvc_physics::vec3_of(from, "raycast: from"),
                                  pycvc_physics::vec3_of(to, "raycast: to"));
    if (!h)
      Py_RETURN_NONE;
    PyObject *d = PyDict_New();
    pycvc_physics::put(d, "body", PyLong_FromUnsignedLong(h->body.value));
    pycvc_physics::put(d, "fraction", PyFloat_FromDouble(h->fraction));
    pycvc_physics::put(d, "point", pycvc_physics::tuple3(h->point));
    pycvc_physics::put(d, "normal", pycvc_physics::tuple3(h->normal));
    pycvc_physics::put(d, "material", PyLong_FromLong(h->material));
    return d;
  }

  // ── determinism ──
  unsigned long long state_hash() const { return $self->state_hash(); }
  // The whole simulation state as bytes, and back.
  PyObject *save() const {
    const std::vector<std::uint8_t> s = $self->save();
    return PyBytes_FromStringAndSize(reinterpret_cast<const char *>(s.data()),
                                     static_cast<Py_ssize_t>(s.size()));
  }
  void restore(PyObject *state) {
    char *p = nullptr;
    Py_ssize_t n = 0;
    if (PyBytes_AsStringAndSize(state, &p, &n) != 0) {
      PyErr_Clear();
      throw std::invalid_argument("physics_world.restore: expected bytes from save()");
    }
    $self->restore(std::vector<std::uint8_t>(p, p + n));
  }
  // The whole world at one tick as JSON, by C++'s own serializer (snapshot_json.h): every body,
  // field, vehicle and contact. `bodies`: "all", "moving" or "static".
  std::string snapshot_json(const std::string &bodies = "all") const {
    cvc::physics::json_bodies b = cvc::physics::json_bodies::all;
    if (bodies == "moving")
      b = cvc::physics::json_bodies::moving;
    else if (bodies == "static")
      b = cvc::physics::json_bodies::static_only;
    else if (bodies != "all")
      throw std::invalid_argument("snapshot_json: bodies is 'all', 'moving' or 'static'");
    return cvc::physics::to_json($self->snapshot(), b);
  }
}

// ── cvc::physics::world_binding ───────────────────────────────────────────────

%rename(physics_binding) cvc::physics::world_binding;
%feature("kwargs") cvc::physics::world_binding::world_binding;
// The binding holds the app and the world by reference: keep both alive with it.
%pythonappend cvc::physics::world_binding::world_binding %{
  self._keep = (args[0] if len(args) > 0 else kwargs.get("app"),
                args[1] if len(args) > 1 else kwargs.get("world"))
%}

namespace cvc {
namespace physics {
class world_binding {
public:
  ~world_binding();
  void pull();
  void push();
  void flush();
};
} // namespace physics
} // namespace cvc

%extend cvc::physics::world_binding {
  // The world under `prefix` in the app's state tree (world_binding.h): pull() reads the tree's
  // settings, commands and knobs into the world; push() publishes its telemetry.
  world_binding(std::shared_ptr<cvc::app> app, cvc::physics::world &world,
                const std::string &prefix = "physics") {
    if (!app)
      throw std::invalid_argument("physics_binding: app is None");
    return new cvc::physics::world_binding(*app, world, prefix);
  }
  // Give a body, vehicle or field keys of its own under <prefix>.bodies / vehicles / fields.
  void publish_body(const std::string &name, long id) {
    $self->publish_body(name, pycvc_physics::body_of(id));
  }
  void publish_vehicle(const std::string &name, long id) {
    $self->publish_vehicle(name, pycvc_physics::vehicle_of(id));
  }
  void publish_field(const std::string &name, long id) {
    $self->publish_field(name, pycvc_physics::field_id_of(id));
  }
}

// ── cvc::physics::nav_plant ───────────────────────────────────────────────────

%feature("kwargs") cvc::physics::nav_plant::nav_plant;
// The plant borrows the world: keep it alive with the plant.
%pythonappend cvc::physics::nav_plant::nav_plant %{
  self._world = world
%}
%feature("kwargs") cvc::physics::nav_plant::set_modulation;
%feature("kwargs") cvc::physics::nav_plant::terrain_config_of;

namespace cvc {
namespace physics {
class nav_plant {
public:
  ~nav_plant();
  int size() const;
  void set_latency(double s);
  void set_brake_margin(double share);
  void set_drift_nsub(int n);
  void clear_drift();
  double advance_us() const;
  double advance_us_per_vehicle() const;
  int reference_substeps() const;
};
} // namespace physics
} // namespace cvc

%extend cvc::physics::nav_plant {
  // A physics world's vehicles as a sim_world's plant (nav_plant.h): agent i drives
  // descs[i % len(descs)]. The plant keeps the world alive. `ground_bodies` (ids): the bodies the
  // vehicles drive on, whose contacts with a chassis are not building contacts. `ground` (a
  // pycvc.height_field, the terrain the world collides with) is the ground under each start, for
  // terrain that is not flat (nav_plant_options::ground_height); None: ground_z everywhere.
  nav_plant(cvc::physics::world &world, const std::vector<cvc::physics::vehicle_desc> &descs,
            bool step_world = true, double latency = 0.1, double brake_margin = 0.8,
            double metres_per_world_unit = 1.0, double ground_z = 0.0,
            double spawn_height = -1.0, int drift_nsub = 0, PyObject *drift_bins_g = nullptr,
            PyObject *ground_bodies = nullptr, const cvc::height_field *ground = nullptr) {
    cvc::physics::nav_plant_options o;
    o.step_world = step_world;
    o.latency = latency;
    o.brake_margin = brake_margin;
    o.metres_per_world_unit = metres_per_world_unit;
    o.ground_z = ground_z;
    o.spawn_height = spawn_height;
    o.drift_nsub = drift_nsub;
    if (ground) {
      const cvc::height_field field = *ground; // a copy sharing its arrays
      const double fallback = ground_z;
      o.ground_height = [field, fallback](double x, double y) {
        const double h = field.height(x, y);
        return std::isnan(h) ? fallback : h;
      };
    }
    if (drift_bins_g && drift_bins_g != Py_None) {
      std::vector<long> shape;
      o.drift_bins_g = pycvc_world::doubles(drift_bins_g, "nav_plant: drift_bins_g", shape);
    }
    o.ground_bodies = pycvc_physics::body_ids(ground_bodies, "nav_plant");
    return new cvc::physics::nav_plant(world, descs, o);
  }
  std::string kind() const { return "jolt"; }
  // Agent i as nav reads it (normalized frame): x, y, heading, speed, steer, and the measured
  // tire_use, load_transfer, pitch and roll.
  PyObject *state(int i) const {
    if (i < 0 || i >= $self->size())
      throw std::invalid_argument("nav_plant.state: no agent " + std::to_string(i));
    return pycvc_physics::plant_state_dict($self->state(i));
  }
  // Agent i's vehicle id in the physics world.
  long vehicle(int i) const {
    if (i < 0 || i >= $self->size())
      throw std::invalid_argument("nav_plant.vehicle: no agent " + std::to_string(i));
    return $self->vehicle(i).value;
  }
  // The plant's settings as a dict.
  PyObject *options() const {
    const cvc::physics::nav_plant_options &o = $self->options();
    return Py_BuildValue("{s:O,s:d,s:d,s:d,s:d,s:d,s:i}", "step_world",
                         o.step_world ? Py_True : Py_False, "latency", o.latency, "brake_margin",
                         o.brake_margin, "metres_per_world_unit", o.metres_per_world_unit,
                         "ground_z", o.ground_z, "spawn_height", o.spawn_height, "drift_nsub",
                         o.drift_nsub);
  }
  // How measured tire use and load transfer modulate the drive (H4); 0 gains = off.
  void set_modulation(double tire_onset = 0.7, double tire_gain = 0.0, double tip_onset = 0.4,
                      double tip_gain = 0.0) {
    cvc::nav::plant_modulation m;
    m.tire_onset = static_cast<float>(tire_onset);
    m.tire_gain = static_cast<float>(tire_gain);
    m.tip_onset = static_cast<float>(tip_onset);
    m.tip_gain = static_cast<float>(tip_gain);
    $self->set_modulation(m);
  }
  PyObject *modulation() const {
    const cvc::nav::plant_modulation m = $self->modulation();
    return Py_BuildValue("{s:d,s:d,s:d,s:d}", "tire_onset", (double)m.tire_onset, "tire_gain",
                         (double)m.tire_gain, "tip_onset", (double)m.tip_onset, "tip_gain",
                         (double)m.tip_gain);
  }
  // Agent i's last drift, in SI: position_m, heading_rad, speed_mps, lateral_g, stop_speed_mps.
  PyObject *last_drift(int i) const {
    if (i < 0 || i >= $self->size())
      throw std::invalid_argument("nav_plant.last_drift: no agent " + std::to_string(i));
    const auto &d = $self->last_drift(i);
    return Py_BuildValue("{s:d,s:d,s:d,s:d,s:d}", "position_m", d.position_m, "heading_rad",
                         d.heading_rad, "speed_mps", d.speed_mps, "lateral_g", d.lateral_g,
                         "stop_speed_mps", d.stop_speed_mps);
  }
  // The drift summary: one dict per lateral-g bin (lo_g, hi_g (inf for the last), ticks, and
  // the position, heading and speed rms and max).
  PyObject *drift_summary() const {
    const auto bins = $self->drift_summary();
    PyObject *list = PyList_New(static_cast<Py_ssize_t>(bins.size()));
    for (std::size_t b = 0; b < bins.size(); ++b) {
      const auto &d = bins[b];
      PyList_SET_ITEM(list, static_cast<Py_ssize_t>(b),
                      Py_BuildValue("{s:d,s:d,s:K,s:d,s:d,s:d,s:d,s:d,s:d}", "lo_g", d.lo_g,
                                    "hi_g", d.hi_g, "ticks", (unsigned long long)d.ticks,
                                    "position_rms_m", d.position_rms_m, "position_max_m",
                                    d.position_max_m, "heading_rms_rad", d.heading_rms_rad,
                                    "heading_max_rad", d.heading_max_rad, "speed_rms_mps",
                                    d.speed_rms_mps, "speed_max_mps", d.speed_max_mps));
    }
    return list;
  }
  // The state tree (nav_plant.h): pull() reads <prefix>.nav_plant's knobs; publish() writes the
  // plant's keys and each named agent's vehicles.<name>.nav.* keys.
  void pull(std::shared_ptr<cvc::app> app, const std::string &prefix) {
    if (!app)
      throw std::invalid_argument("nav_plant.pull: app is None");
    $self->pull(*app, prefix);
  }
  void publish(std::shared_ptr<cvc::app> app, const std::string &prefix,
               const std::vector<std::string> &names) const {
    if (!app)
      throw std::invalid_argument("nav_plant.publish: app is None");
    $self->publish(*app, prefix, names);
  }
  // A vehicle's terrain limits (P5) as a dict, for nav_sim_world_set_terrain: its steepest
  // climb on `mu` and its static stability factor.
  static PyObject *terrain_config_of(const cvc::physics::vehicle_desc &desc, double mu = 0.9,
                                     double gravity = 9.80665) {
    return pycvc_physics::terrain_config_dict(
        cvc::physics::nav_plant::terrain_config_of(desc, mu, gravity));
  }
}

// ── cvc::physics::dyn_bike_plant ──────────────────────────────────────────────

%feature("kwargs") cvc::physics::dyn_bike_plant::dyn_bike_plant;
// With a world the plant borrows it: keep it alive with the plant.
%pythonappend cvc::physics::dyn_bike_plant::dyn_bike_plant %{
  self._world = world
%}

namespace cvc {
namespace physics {
class dyn_bike_plant {
public:
  ~dyn_bike_plant();
  int size() const;
  int substeps() const;
  bool collides() const;
};
} // namespace physics
} // namespace cvc

%extend cvc::physics::dyn_bike_plant {
  // The dynamic single-track vehicle as a sim_world's plant (dyn_bike_plant.h): tires, load
  // transfer, actuator lag and the port-Hamiltonian energy books, no Jolt in the loop. Agent i
  // drives descs[i % len(descs)] on a ground of grip `mu`: flat at ground_z, or `ground` (a
  // pycvc.height_field in world metres, copied as it is now; ground_z where it has no height),
  // which the vehicle climbs and descends along its heading. With `world` (a physics_world, kept
  // alive by the plant) each chassis is a box in it that collides with its bodies, except
  // `ground_bodies` (ids), the ground the wheels drive on.
  //
  // The options, all off by default:
  //   wheel_spin  each axle's wheels spin at their own speed, driven and braked by torques; the
  //               books gain the slip line.
  //   side_slope  gravity along and across the heading, from `ground`'s gradient under the centre
  //               of gravity; the grade line is minus the potential's change.
  //   contact     compliant contact, through the plant's external port: each footprint against
  //               `contact_field` (a sim_world handle: a copy of its signed distance on plane
  //               `contact_plane` as it is now) and, with contact_vehicles, the vehicles against
  //               each other. The books gain the contact_* lines. `contact_params` (a dict:
  //               k, c, friction, slide, skin, per_side, per_end, rounding) changes the
  //               defaults. `contact_step_depth` (m; None keeps the plant's default) cuts a
  //               substep in which a point would close more than it into pieces; <= 0 never
  //               does. Not with `world`, which collides the chassis instead.
  dyn_bike_plant(const std::vector<cvc::physics::vehicle_desc> &descs,
                 cvc::physics::world *world = nullptr, PyObject *ground_bodies = nullptr,
                 double mu = 0.9, double substep = 0.01, double drive_lag = -1.0,
                 bool driver = true, double latency = 0.1, double brake_margin = 0.8,
                 double metres_per_world_unit = 1.0, double ground_z = 0.0,
                 double chassis_friction = 0.3, const cvc::height_field *ground = nullptr,
                 bool wheel_spin = false, bool side_slope = false, bool contact = false,
                 PyObject *contact_field = nullptr, int contact_plane = 0,
                 bool contact_vehicles = true, PyObject *contact_params = nullptr,
                 PyObject *contact_step_depth = nullptr) {
    cvc::physics::dyn_bike_plant_options o;
    if (ground) {
      auto hf = std::make_shared<cvc::height_field>(*ground);
      o.plant.ground_height = [hf, ground_z](double x, double y) {
        const double h = hf->height(x, y);
        return std::isfinite(h) ? h : ground_z;
      };
    }
    o.wheel_spin = wheel_spin;
    o.side_slope = side_slope;
    o.contact = contact;
    o.contact_vehicles = contact_vehicles;
    o.contact_params = pycvc_physics::contact_params_of(contact_params);
    if (contact_step_depth && contact_step_depth != Py_None) {
      const double d = PyFloat_AsDouble(contact_step_depth);
      if (PyErr_Occurred()) {
        PyErr_Clear();
        throw std::invalid_argument("dyn_bike_plant: contact_step_depth is a number or None");
      }
      o.contact_step_depth = d;
    }
    if (contact_field && contact_field != Py_None) {
      if (!contact)
        throw std::invalid_argument("dyn_bike_plant: contact_field needs contact=True");
      o.contact_field = pycvc_physics::sdf_field_snapshot(
          pycvc::sim_world_from(contact_field)->field(), contact_plane, metres_per_world_unit);
    }
    if (contact && world)
      throw std::invalid_argument("dyn_bike_plant: contact or a world, not both");
    o.plant.latency = latency;
    o.plant.brake_margin = brake_margin;
    o.plant.metres_per_world_unit = metres_per_world_unit;
    o.plant.ground_z = ground_z;
    o.mu = mu;
    o.substep = substep;
    o.drive_lag = drive_lag;
    o.driver = driver;
    o.physics = world;
    o.chassis_friction = chassis_friction;
    o.ground_bodies = pycvc_physics::body_ids(ground_bodies, "dyn_bike_plant");
    if (!world && !o.ground_bodies.empty())
      throw std::invalid_argument("dyn_bike_plant: ground_bodies needs a world");
    return new cvc::physics::dyn_bike_plant(descs, o);
  }
  std::string kind() const { return "dynamic"; }
  // Agent i as nav reads it (normalized frame): x, y, heading, speed, steer, and tire_use (the
  // most used tire's slip angle over its peak); the single track has no load transfer, pitch or
  // roll of its own.
  PyObject *state(int i) const {
    if (i < 0 || i >= $self->size())
      throw std::invalid_argument("dyn_bike_plant.state: no agent " + std::to_string(i));
    return pycvc_physics::plant_state_dict($self->state(i));
  }
  // Agent i's model state, in SI: the rear axle (x, y, m), the heading psi, the body's forward
  // and lateral speed at the centre of gravity (vx, vy, m/s), the yaw rate r (rad/s), the
  // steering angle delta (rad) and the drive actuator a (m/s^2).
  PyObject *body(int i) const {
    if (i < 0 || i >= $self->size())
      throw std::invalid_argument("dyn_bike_plant.body: no agent " + std::to_string(i));
    const auto &s = $self->body(i);
    return Py_BuildValue("{s:d,s:d,s:d,s:d,s:d,s:d,s:d,s:d}", "x", s.x, "y", s.y, "psi", s.psi,
                         "vx", s.vx, "vy", s.vy, "r", s.r, "delta", s.delta, "a", s.a);
  }
  // Agent i's energy books (J) since attach, and over the last tick: kinetic, drive, brake,
  // dissipated, slip, floor, grade, limiter, contact (the world's), contact_energy (the compliant
  // barrier's level), contact_stored, contact_dissipated, contact_exchange, residual, and flow
  // (everything that moved). kinetic - kinetic_at_attach = drive - brake - dissipated - slip +
  // floor + grade + limiter + contact - contact_stored - contact_dissipated + contact_exchange +
  // residual.
  PyObject *books(int i) const {
    if (i < 0 || i >= $self->size())
      throw std::invalid_argument("dyn_bike_plant.books: no agent " + std::to_string(i));
    return pycvc_physics::dyn_books_dict($self->books(i));
  }
  PyObject *last_tick(int i) const {
    if (i < 0 || i >= $self->size())
      throw std::invalid_argument("dyn_bike_plant.last_tick: no agent " + std::to_string(i));
    return pycvc_physics::dyn_books_dict($self->last_tick(i));
  }
  // Agent i's kinetic energy at attach (J), the books' start.
  double kinetic_at_attach(int i) const {
    if (i < 0 || i >= $self->size())
      throw std::invalid_argument("dyn_bike_plant.kinetic_at_attach: no agent " +
                                  std::to_string(i));
    return $self->kinetic_at_attach(i);
  }
  // With a world: agent i's chassis box (None without one), the substeps of its last tick in
  // which it collided and all since attach, and the agent it touched deepest last tick (-1 none).
  PyObject *chassis(int i) const {
    if (i < 0 || i >= $self->size())
      throw std::invalid_argument("dyn_bike_plant.chassis: no agent " + std::to_string(i));
    if (!$self->collides())
      Py_RETURN_NONE;
    return PyLong_FromUnsignedLong($self->chassis(i).value);
  }
  int collisions_tick(int i) const {
    if (i < 0 || i >= $self->size())
      throw std::invalid_argument("dyn_bike_plant.collisions_tick: no agent " + std::to_string(i));
    return $self->collisions_tick(i);
  }
  long collisions(int i) const {
    if (i < 0 || i >= $self->size())
      throw std::invalid_argument("dyn_bike_plant.collisions: no agent " + std::to_string(i));
    return $self->collisions(i);
  }
  int vehicle_contact_step(int i) const {
    if (i < 0 || i >= $self->size())
      throw std::invalid_argument("dyn_bike_plant.vehicle_contact_step: no agent " +
                                  std::to_string(i));
    return $self->vehicle_contact_step(i);
  }
  // Agent i's wheels' speed (rad/s, + rolling forward) on `axle`, 0 the front and 1 the rear:
  // with wheel_spin their own, else the speed's at the rolling radius.
  double wheel_speed(int i, int axle) const {
    if (i < 0 || i >= $self->size())
      throw std::invalid_argument("dyn_bike_plant.wheel_speed: no agent " + std::to_string(i));
    return $self->wheel_speed(i, axle);
  }
  // With contact: the compliant contact on agent i at the start of its last substep, against the
  // field and the other vehicles: the barrier's energy (J), the damper's and the friction's power
  // (W, >= 0), the points touching, and the body wrench (N, N, N m: along the body, across it, and
  // about the centre of gravity). None without contact.
  PyObject *contact_now(int i) const {
    if (i < 0 || i >= $self->size())
      throw std::invalid_argument("dyn_bike_plant.contact_now: no agent " + std::to_string(i));
    if (!$self->options().contact)
      Py_RETURN_NONE;
    const cvc::nav::diff::dyn_contact &c = $self->contact_now(i);
    double W[3];
    c.wrench(W);
    return Py_BuildValue("{s:d,s:d,s:i,s:(ddd)}", "energy", c.energy, "damping", c.damping,
                         "touching", c.touching, "wrench", W[0], W[1], W[2]);
  }
  // The plant's settings as a dict.
  PyObject *options() const {
    const cvc::physics::dyn_bike_plant_options &o = $self->options();
    return Py_BuildValue(
        "{s:d,s:d,s:d,s:O,s:d,s:d,s:d,s:d,s:d,s:O,s:O,s:O,s:O,s:O,s:O,s:N,s:d}", "mu", o.mu,
        "substep", o.substep, "drive_lag", o.drive_lag, "driver", o.driver ? Py_True : Py_False,
        "latency", o.plant.latency, "brake_margin", o.plant.brake_margin,
        "metres_per_world_unit", o.plant.metres_per_world_unit, "ground_z", o.plant.ground_z,
        "chassis_friction", o.chassis_friction, "ground",
        o.plant.ground_height ? Py_True : Py_False, "wheel_spin",
        o.wheel_spin ? Py_True : Py_False, "side_slope", o.side_slope ? Py_True : Py_False,
        "contact", o.contact ? Py_True : Py_False, "contact_field",
        o.contact_field ? Py_True : Py_False, "contact_vehicles",
        o.contact_vehicles ? Py_True : Py_False, "contact_params",
        pycvc_physics::contact_params_dict(o.contact_params), "contact_step_depth",
        o.contact_step_depth);
  }
}

// ── plant selection and terrain on a sim_world (pycvc_nav.i's handles) ────────

%inline %{
namespace pycvc {

namespace {
// A sim_world handle keeps the plant set on it alive: the capsule's context holds a reference.
void sim_world_hold(PyObject *handle, PyObject *plant) {
  PyObject *old = static_cast<PyObject *>(PyCapsule_GetContext(handle));
  if (PyErr_Occurred())
    PyErr_Clear();
  Py_XINCREF(plant);
  PyCapsule_SetContext(handle, plant);
  Py_XDECREF(old);
}
} // namespace

// Select a sim_world's plant: a pycvc.nav_plant (its agents drive the physics world's
// vehicles), a pycvc.dyn_bike_plant (the dynamic single-track vehicle), or None for nav's own
// kinematic bicycle (sim_world::set_plant). The handle keeps the plant alive while it is set.
PyObject *nav_sim_world_set_plant(PyObject *handle, PyObject *plant) {
  cvc::nav::sim_world *sw = sim_world_from(handle);
  if (!plant || plant == Py_None) {
    sw->set_plant(nullptr);
    sim_world_hold(handle, nullptr);
    Py_RETURN_NONE;
  }
  void *p = nullptr;
  cvc::nav::vehicle_plant *vp = nullptr;
  if (SWIG_IsOK(SWIG_ConvertPtr(plant, &p, SWIGTYPE_p_cvc__physics__nav_plant, 0)) && p)
    vp = static_cast<cvc::physics::nav_plant *>(p);
  else if (SWIG_IsOK(SWIG_ConvertPtr(plant, &p, SWIGTYPE_p_cvc__physics__dyn_bike_plant, 0)) && p)
    vp = static_cast<cvc::physics::dyn_bike_plant *>(p);
  if (!vp)
    throw std::invalid_argument(
        "nav_sim_world_set_plant: expected a pycvc.nav_plant, a pycvc.dyn_bike_plant or None");
  sw->set_plant(vp);
  sim_world_hold(handle, plant);
  Py_RETURN_NONE;
}

// The ticks the sim_world has stepped.
long nav_sim_world_tick(PyObject *handle) { return sim_world_from(handle)->tick(); }

// "kinematic", "jolt" or "dynamic".
std::string nav_sim_world_plant_kind(PyObject *handle) {
  switch (sim_world_from(handle)->plant_type()) {
  case cvc::nav::plant_kind::jolt:
    return "jolt";
  case cvc::nav::plant_kind::dynamic:
    return "dynamic";
  default:
    return "kinematic";
  }
}

// Terrain in the navigator's energy (P5): heights (rows, cols) in metres, row 0 at min y, and a
// config dict (nav_plant.terrain_config_of's form; missing keys keep terrain_config's defaults).
PyObject *nav_sim_world_set_terrain(PyObject *handle, PyObject *heights, PyObject *config) {
  cvc::nav::sim_world *sw = sim_world_from(handle);
  PyArrayObject *a = reinterpret_cast<PyArrayObject *>(
      PyArray_FROMANY(heights, NPY_FLOAT, 2, 2, NPY_ARRAY_C_CONTIGUOUS | NPY_ARRAY_FORCECAST));
  if (!a) {
    PyErr_Clear();
    throw std::invalid_argument("nav_sim_world_set_terrain: heights is a (rows, cols) array");
  }
  if (PyArray_DIM(a, 0) != sw->rows() || PyArray_DIM(a, 1) != sw->cols()) {
    Py_DECREF(a);
    throw std::invalid_argument("nav_sim_world_set_terrain: heights must be the grid's (rows, cols)");
  }
  try {
    sw->set_terrain(static_cast<const float *>(PyArray_DATA(a)),
                    pycvc_physics::terrain_config_of(config));
  } catch (...) {
    Py_DECREF(a);
    throw;
  }
  Py_DECREF(a);
  Py_RETURN_NONE;
}
PyObject *nav_sim_world_set_terrain_config(PyObject *handle, PyObject *config) {
  cvc::nav::sim_world *sw = sim_world_from(handle);
  if (!sw->has_terrain())
    throw std::invalid_argument("nav_sim_world_set_terrain_config: no terrain set");
  sw->set_terrain_config(pycvc_physics::terrain_config_of(config, sw->terrain_settings()));
  Py_RETURN_NONE;
}
PyObject *nav_sim_world_clear_terrain(PyObject *handle) {
  sim_world_from(handle)->clear_terrain();
  Py_RETURN_NONE;
}
// The terrain planes (cvc/nav/terrain.h's terrain_build) for heights (rows, cols) in metres,
// row 0 at min y, on cells cell_w metres square, under a config dict (terrain_config_of's form;
// missing keys keep the defaults): a dict of slope ((2, rows, cols) float32: dz/dx, then dz/dy),
// grade_cost ((rows, cols) float32, 1 at the vehicle's limit), infeasible ((rows, cols) uint8)
// and limit (the steepest feasible slope's tangent). The C++ reference a Python twin of the
// planes (GRL-SNAM's grl_snam.terrain) is checked against.
PyObject *nav_terrain_build(PyObject *heights, double cell_w, PyObject *config = nullptr) {
  PyArrayObject *a = reinterpret_cast<PyArrayObject *>(
      PyArray_FROMANY(heights, NPY_FLOAT, 2, 2, NPY_ARRAY_C_CONTIGUOUS | NPY_ARRAY_FORCECAST));
  if (!a) {
    PyErr_Clear();
    throw std::invalid_argument("nav_terrain_build: heights is a (rows, cols) array");
  }
  const int rows = static_cast<int>(PyArray_DIM(a, 0)), cols = static_cast<int>(PyArray_DIM(a, 1));
  cvc::nav::terrain_planes t;
  try {
    t = cvc::nav::terrain_build(static_cast<const float *>(PyArray_DATA(a)), rows, cols, cell_w,
                                pycvc_physics::terrain_config_of(config));
  } catch (...) {
    Py_DECREF(a);
    throw;
  }
  Py_DECREF(a);
  npy_intp d3[3] = {2, rows, cols}, d2[2] = {rows, cols};
  PyObject *slope = PyArray_SimpleNew(3, d3, NPY_FLOAT);
  PyObject *cost = PyArray_SimpleNew(2, d2, NPY_FLOAT);
  PyObject *inf = PyArray_SimpleNew(2, d2, NPY_UINT8);
  if (!slope || !cost || !inf) {
    Py_XDECREF(slope);
    Py_XDECREF(cost);
    Py_XDECREF(inf);
    throw std::runtime_error("nav_terrain_build: out of memory");
  }
  std::memcpy(PyArray_DATA(reinterpret_cast<PyArrayObject *>(slope)), t.slope.data(),
              t.slope.size() * sizeof(float));
  std::memcpy(PyArray_DATA(reinterpret_cast<PyArrayObject *>(cost)), t.grade_cost.data(),
              t.grade_cost.size() * sizeof(float));
  std::memcpy(PyArray_DATA(reinterpret_cast<PyArrayObject *>(inf)), t.infeasible.data(),
              t.infeasible.size());
  PyObject *d = PyDict_New();
  pycvc_physics::put(d, "slope", slope);
  pycvc_physics::put(d, "grade_cost", cost);
  pycvc_physics::put(d, "infeasible", inf);
  pycvc_physics::put(d, "limit", PyFloat_FromDouble(t.limit));
  return d;
}

// The terrain's config as a dict, or None without one.
PyObject *nav_sim_world_terrain_settings(PyObject *handle) {
  cvc::nav::sim_world *sw = sim_world_from(handle);
  if (!sw->has_terrain())
    Py_RETURN_NONE;
  return pycvc_physics::terrain_config_dict(sw->terrain_settings());
}
// Each agent's ticks on infeasible slope since set_terrain, as a list, or None without one.
PyObject *nav_sim_world_infeasible_entries(PyObject *handle) {
  cvc::nav::sim_world *sw = sim_world_from(handle);
  const int *e = sw->infeasible_entries();
  if (!e)
    Py_RETURN_NONE;
  PyObject *list = PyList_New(sw->size());
  for (int i = 0; i < sw->size(); ++i)
    PyList_SET_ITEM(list, i, PyLong_FromLong(e[i]));
  return list;
}

// The sim_world's settings in the state tree (sim_world::pull / publish, under <prefix>.nav).
PyObject *nav_sim_world_pull(PyObject *handle, std::shared_ptr<cvc::app> app,
                             const std::string &prefix) {
  if (!app)
    throw std::invalid_argument("nav_sim_world_pull: app is None");
  sim_world_from(handle)->pull(*app, prefix);
  Py_RETURN_NONE;
}
PyObject *nav_sim_world_publish(PyObject *handle, std::shared_ptr<cvc::app> app,
                                const std::string &prefix) {
  if (!app)
    throw std::invalid_argument("nav_sim_world_publish: app is None");
  sim_world_from(handle)->publish(*app, prefix);
  Py_RETURN_NONE;
}

} // namespace pycvc
%}

// ── plant_host: the plant chosen by name, in code or live from the state tree ──
// cvc::physics::plant_host (plant_host.h) for Python. It owns the physics world and the plant a
// sim_world drives on, and switches them by name: "kinematic", a preset ("humvee", "roboracer"),
// a comma list cycled over the agents, "dynamic[:presets]" or "dynamic4w[:presets]". pull() does it
// live from <prefix>.nav_plant.vehicle and reads the plant's own knobs. The host holds the
// sim_world it last drove, and puts it back on the kinematic bicycle before it goes, so neither
// outlives what the other points at.
%feature("kwargs") pycvc::plant_host::plant_host;
%feature("kwargs") pycvc::plant_host::use;
%feature("kwargs") pycvc::plant_host::pull;

%inline %{
namespace pycvc {

class plant_host {
public:
  // The world each switch builds: its fixed step (s), and the flat ground (its top's height,
  // grip, rolling resistance and half extent, m) used until set_geometry gives a world; the
  // plant's latency (s), brake margin and metres per nav world unit.
  plant_host(double fixed_dt = 0.01, double ground_mu = 0.9, double rolling_resistance = 0.015,
             double ground_z = 0.0, double ground_half_size = 2000.0, double latency = 0.1,
             double brake_margin = 0.8, double metres_per_world_unit = 1.0) {
    cvc::physics::plant_host_options o;
    o.world.fixed_dt = fixed_dt;
    o.ground_z = ground_z;
    o.ground_mu = ground_mu;
    o.ground_rolling_resistance = rolling_resistance;
    o.ground_half_size = ground_half_size;
    o.plant.latency = latency;
    o.plant.brake_margin = brake_margin;
    o.plant.metres_per_world_unit = metres_per_world_unit;
    opts_ = o;
    host_ = std::make_unique<cvc::physics::plant_host>(opts_);
  }
  ~plant_host() {
    if (sim_) {
      try {
        host_->use(*sim_world_from(sim_), "kinematic");
      } catch (...) {
      }
      Py_DECREF(sim_);
    }
  }

  // Whether this build has Jolt, the presets use() takes, and whether it takes `vehicle` (raises
  // RuntimeError when it does not).
  static bool available() { return cvc::physics::plant_host::available(); }
  static std::vector<std::string> presets() { return cvc::physics::plant_host::presets(); }
  static void check(const std::string &vehicle) { cvc::physics::plant_host::check(vehicle); }

  // The next world's static colliders: a pycvc.world_geometry's collide items (a city bundle's
  // data_world, a generated world's), in place of the flat ground; None goes back to it. The host
  // keeps a copy, and the vehicles start on its terrain (plant_host_options' ground_height). The
  // sim_world it drives goes back to the kinematic bicycle: use() puts it on vehicles in the new
  // world.
  void set_geometry(PyObject *geometry) {
    std::unique_ptr<cvc::world::world_geometry> copy;
    if (geometry && geometry != Py_None) {
      void *p = nullptr;
      if (!SWIG_IsOK(SWIG_ConvertPtr(geometry, &p, SWIGTYPE_p_cvc__world__world_geometry, 0)) || !p)
        throw std::invalid_argument(
            "plant_host.set_geometry: expected a pycvc.world_geometry or None");
      copy = std::make_unique<cvc::world::world_geometry>(
          *static_cast<cvc::world::world_geometry *>(p));
    }
    std::shared_ptr<const cvc::height_field> terrain = copy ? copy->terrain() : nullptr;
    if (terrain) {
      const double fallback = opts_.ground_z;
      opts_.plant.ground_height = [terrain, fallback](double x, double y) {
        const double h = terrain->height(x, y);
        return std::isnan(h) ? fallback : h;
      };
    } else {
      opts_.plant.ground_height = nullptr;
    }
    if (sim_) { // off the old host's plant before that host goes
      try {
        host_->use(*sim_world_from(sim_), "kinematic");
      } catch (...) {
      }
    }
    host_ = std::make_unique<cvc::physics::plant_host>(opts_);
    host_->set_geometry(copy.get());
    geometry_ = std::move(copy);
  }

  // Put sim's agents on `vehicle`, at their poses, at rest, in a new world. Returns what they
  // drive: `vehicle`, or "kinematic" when it asked for vehicles and the build has no Jolt. An
  // unknown name raises and changes nothing.
  std::string use(PyObject *sim, const std::string &vehicle) {
    cvc::nav::sim_world *sw = sim_world_from(sim);
    hold(sim);
    return host_->use(*sw, vehicle);
  }
  // The state tree: <prefix>.nav_plant.vehicle switches the plant when it changes (unset, it is
  // seeded with the current one; an unknown name is ignored), the plant's own knobs are read
  // (nav_plant::pull), and .driving / .available are written. Call it between steps.
  void pull(std::shared_ptr<cvc::app> app, const std::string &prefix, PyObject *sim) {
    if (!app)
      throw std::invalid_argument("plant_host.pull: app is None");
    cvc::nav::sim_world *sw = sim_world_from(sim);
    hold(sim);
    host_->pull(*app, prefix, *sw);
  }

  std::string vehicle() const { return host_->vehicle(); } // what use() was last asked for
  std::string driving() const { return host_->driving(); } // what the agents drive
  bool fallback() const { return host_->fallback(); }
  // "kinematic", "jolt", "dynamic" or "dynamic4w".
  std::string kind() {
    if (host_->dynamic4w())
      return "dynamic4w";
    if (host_->dynamic())
      return "dynamic";
    return host_->plant() ? "jolt" : "kinematic";
  }

  // Agent i as the plant reads it (nav's normalized frame: x, y, heading, speed, steer, tire_use,
  // load_transfer, pitch, roll), or None on the kinematic bicycle.
  PyObject *state(int i) {
    cvc::nav::vehicle_plant *vp = host_->active();
    if (!vp)
      Py_RETURN_NONE;
    check_agent(i, "state");
    return pycvc_physics::plant_state_dict(vp->state(i));
  }
  // Agent i's limits on its plant (nav's normalized frame: wheelbase, max_steer, steer_rate,
  // a_brake, latency, a_max, vmax, a_lat, half_width, front, rear), or None on the bicycle.
  PyObject *limits(int i) {
    cvc::nav::vehicle_plant *vp = host_->active();
    if (!vp)
      Py_RETURN_NONE;
    check_agent(i, "limits");
    const cvc::nav::plant_limits l = vp->limits(i);
    return Py_BuildValue("{s:d,s:d,s:d,s:d,s:d,s:d,s:d,s:d,s:d,s:d,s:d}", "wheelbase",
                         (double)l.wheelbase, "max_steer", (double)l.max_steer, "steer_rate",
                         (double)l.steer_rate, "a_brake", (double)l.a_brake, "latency",
                         (double)l.latency, "a_max", (double)l.a_max, "vmax", (double)l.vmax,
                         "a_lat", (double)l.a_lat, "half_width", (double)l.half_width, "front",
                         (double)l.front, "rear", (double)l.rear);
  }
  // Agent i's Jolt vehicle (physics_world.vehicle_state's dict: the chassis' position, rotation,
  // velocities, speed, steer, pedals, gear, wheels, energy), or None off a Jolt plant.
  PyObject *vehicle_state(int i) {
    cvc::physics::nav_plant *np = host_->plant();
    cvc::physics::world *w = host_->physics();
    if (!np || !w)
      Py_RETURN_NONE;
    check_agent(i, "vehicle_state");
    return pycvc_physics::vehicle_state_dict(w->vehicle_state(np->vehicle(i)));
  }
  // Agent i's dynamic single-track state in SI (x, y, psi, vx, vy, r, delta, a), or None off it.
  PyObject *body(int i) {
    cvc::physics::dyn_bike_plant *d = host_->dynamic();
    if (!d)
      Py_RETURN_NONE;
    check_agent(i, "body");
    const auto &s = d->body(i);
    return Py_BuildValue("{s:d,s:d,s:d,s:d,s:d,s:d,s:d,s:d}", "x", s.x, "y", s.y, "psi", s.psi,
                         "vx", s.vx, "vy", s.vy, "r", s.r, "delta", s.delta, "a", s.a);
  }

private:
  void hold(PyObject *sim) {
    if (sim == sim_)
      return;
    if (sim_) { // the old one back on the bicycle before it is let go
      try {
        host_->use(*sim_world_from(sim_), "kinematic");
      } catch (...) {
      }
    }
    Py_INCREF(sim);
    Py_XDECREF(sim_);
    sim_ = sim;
  }
  void check_agent(int i, const char *what) {
    const int n = sim_ ? sim_world_from(sim_)->size() : 0;
    if (i < 0 || i >= n)
      throw std::invalid_argument(std::string("plant_host.") + what + ": no agent " +
                                  std::to_string(i));
  }
  cvc::physics::plant_host_options opts_;
  std::unique_ptr<cvc::world::world_geometry> geometry_; // borrowed by host_, so it goes after it
  std::unique_ptr<cvc::physics::plant_host> host_;
  PyObject *sim_ = nullptr;
};

} // namespace pycvc
%}
