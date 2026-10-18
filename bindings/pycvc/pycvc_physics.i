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
//  * Lifetimes: a nav_plant keeps its physics_world alive, a physics_binding its world and app,
//    and a sim_world handle the plant set on it.

%{
#include <cvc/geometry/geometry.h>
#include <cvc/geometry/height_field.h>
#include <cvc/nav/plant_types.h>
#include <cvc/nav/sim_world.h>
#include <cvc/nav/terrain.h>
#include <cvc/physics/backend.h>
#include <cvc/physics/nav_plant.h>
#include <cvc/physics/snapshot_json.h>
#include <cvc/physics/vehicle.h>
#include <cvc/physics/world.h>
#include <cvc/physics/world_binding.h>
#include <cstring>
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
  double max_accel, max_decel;
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
  // descs[i % len(descs)]. The plant keeps the world alive.
  nav_plant(cvc::physics::world &world, const std::vector<cvc::physics::vehicle_desc> &descs,
            bool step_world = true, double latency = 0.1, double brake_margin = 0.8,
            double metres_per_world_unit = 1.0, double ground_z = 0.0,
            double spawn_height = -1.0, int drift_nsub = 0, PyObject *drift_bins_g = nullptr) {
    cvc::physics::nav_plant_options o;
    o.step_world = step_world;
    o.latency = latency;
    o.brake_margin = brake_margin;
    o.metres_per_world_unit = metres_per_world_unit;
    o.ground_z = ground_z;
    o.spawn_height = spawn_height;
    o.drift_nsub = drift_nsub;
    if (drift_bins_g && drift_bins_g != Py_None) {
      std::vector<long> shape;
      o.drift_bins_g = pycvc_world::doubles(drift_bins_g, "nav_plant: drift_bins_g", shape);
    }
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
// vehicles), or None for nav's own kinematic bicycle (sim_world::set_plant). The handle keeps
// the plant alive while it is set.
PyObject *nav_sim_world_set_plant(PyObject *handle, PyObject *plant) {
  cvc::nav::sim_world *sw = sim_world_from(handle);
  if (!plant || plant == Py_None) {
    sw->set_plant(nullptr);
    sim_world_hold(handle, nullptr);
    Py_RETURN_NONE;
  }
  void *p = nullptr;
  if (!SWIG_IsOK(SWIG_ConvertPtr(plant, &p, SWIGTYPE_p_cvc__physics__nav_plant, 0)) || !p)
    throw std::invalid_argument("nav_sim_world_set_plant: expected a pycvc.nav_plant or None");
  sw->set_plant(static_cast<cvc::physics::nav_plant *>(p));
  sim_world_hold(handle, plant);
  Py_RETURN_NONE;
}

// The ticks the sim_world has stepped.
long nav_sim_world_tick(PyObject *handle) { return sim_world_from(handle)->tick(); }

// "kinematic" or "jolt".
std::string nav_sim_world_plant_kind(PyObject *handle) {
  return sim_world_from(handle)->plant_type() == cvc::nav::plant_kind::jolt
             ? "jolt"
             : "kinematic";
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
