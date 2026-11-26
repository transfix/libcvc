// pycvc_mujoco.i — SWIG surface for cvc::physics::mujoco_robot (docs/MUJOCO.md): one MuJoCo robot
// (the Unitree Go2 first) on a cvc::height_field, driven from Python, for the C-PHAST lab's gym:
//
//   go2 = pycvc.mujoco_robot(menagerie + "/unitree_go2/scene.xml", terrain=field)
//   base = go2.body("base")
//   go2.apply_wrench(base, (0.0, 60.0, 0.0)); go2.step(50); go2.clear_wrenches()
//   go2.qpos(), go2.qvel(), go2.actuator_forces()   # read-only numpy float64 copies
//   go2.base(), go2.contacts()                      # a dict, a list of dicts
//   go2.energy(), go2.books()                       # MuJoCo's energies, the work books (J)
//   go2.reset()                                     # back to the start: the next episode
//   pycvc.run_mujoco_trial(model, slope_deg=10.0, push=(0.0, 60.0, 0.0))   # a dict
//
// NOT a standalone module: %include'd by pycvc.i after pycvc_physics.i (its vec3 and tuple
// helpers) and pycvc_world.i (height_field, copy_of).
//
// Every pycvc has it. Without CVC_WITH_MUJOCO, mujoco_available() is False and the constructor
// raises RuntimeError.
//
// SWIG APPROACH (as pycvc_physics.i): the class is declared here by hand and every call goes to
// the real header. Bodies, geoms and actuators cross as ints (MuJoCo's ids). Arrays come back as
// read-only numpy copies, vectors as tuples ((w, x, y, z) for rotations), everything else as
// dicts. step() releases the GIL. The terrain is copied in; the robot keeps no Python object.

%{
#include <cvc/physics/mujoco_robot.h>

namespace pycvc_mujoco {

namespace ph = cvc::physics;

// A sequence of numbers.
std::vector<double> doubles_of(PyObject *o, const char *what) {
  PyObject *seq = o ? PySequence_Fast(o, what) : nullptr;
  if (!seq) {
    PyErr_Clear();
    throw std::invalid_argument(std::string(what) + ": expected a sequence of numbers");
  }
  std::vector<double> out(static_cast<std::size_t>(PySequence_Fast_GET_SIZE(seq)));
  for (std::size_t i = 0; i < out.size(); ++i) {
    out[i] = PyFloat_AsDouble(PySequence_Fast_GET_ITEM(seq, static_cast<Py_ssize_t>(i)));
    if (PyErr_Occurred()) {
      Py_DECREF(seq);
      PyErr_Clear();
      throw std::invalid_argument(std::string(what) + ": expected numbers");
    }
  }
  Py_DECREF(seq);
  return out;
}

pycvc::ArrayView array_of(std::vector<double> v) {
  const long n = static_cast<long>(v.size());
  return pycvc_world::copy_of(std::move(v), {n}, pycvc::DType::Float64);
}

ph::actuator_mode mode_of(const std::string &m) {
  if (m == "position")
    return ph::actuator_mode::position;
  if (m == "torque")
    return ph::actuator_mode::torque;
  throw std::invalid_argument("mujoco_robot: a mode is 'position' or 'torque', not '" + m + "'");
}

const char *mode_name(ph::actuator_mode m) {
  return m == ph::actuator_mode::position ? "position" : "torque";
}

PyObject *books_dict(const ph::mujoco_books &b) {
  return Py_BuildValue("{s:d,s:d,s:d,s:d,s:d,s:d,s:d,s:d,s:d,s:d,s:d}", "actuators", b.actuators,
                       "external", b.external, "contacts", b.contacts, "joint_friction",
                       b.joint_friction, "joint_limits", b.joint_limits, "joint_damping",
                       b.joint_damping, "other", b.other, "energy_start", b.energy_start,
                       "energy_now", b.energy_now, "work", b.work(), "residual", b.residual());
}

PyObject *contact_dict(const ph::mujoco_contact &c) {
  PyObject *d = PyDict_New();
  pycvc_physics::put(d, "geom", Py_BuildValue("(ii)", c.geom[0], c.geom[1]));
  pycvc_physics::put(d, "body", Py_BuildValue("(ii)", c.body[0], c.body[1]));
  pycvc_physics::put(d, "position", pycvc_physics::tuple3(c.position));
  pycvc_physics::put(d, "normal", pycvc_physics::tuple3(c.normal));
  pycvc_physics::put(d, "distance", PyFloat_FromDouble(c.distance));
  pycvc_physics::put(d, "force", pycvc_physics::tuple3(c.force));
  pycvc_physics::put(d, "torque", pycvc_physics::tuple3(c.torque));
  return d;
}

PyObject *trial_dict(const ph::mujoco_trial_result &r) {
  PyObject *d = PyDict_New();
  pycvc_physics::put(d, "steps", PyLong_FromLong(r.steps));
  pycvc_physics::put(d, "fell", PyBool_FromLong(r.fell ? 1 : 0));
  pycvc_physics::put(d, "recovered", PyBool_FromLong(r.recovered ? 1 : 0));
  pycvc_physics::put(d, "settled",
                     Py_BuildValue("{s:d,s:d,s:d}", "height", r.settled_height, "up_z",
                                   r.settled_up_z, "speed", r.settled_speed));
  pycvc_physics::put(d, "end",
                     Py_BuildValue("{s:d,s:d,s:d,s:d}", "height", r.end_height, "up_z", r.end_up_z,
                                   "speed", r.end_speed, "displacement", r.end_displacement));
  pycvc_physics::put(d, "min_height", PyFloat_FromDouble(r.min_height));
  pycvc_physics::put(d, "min_up_z", PyFloat_FromDouble(r.min_up_z));
  pycvc_physics::put(d, "max_displacement", PyFloat_FromDouble(r.max_displacement));
  pycvc_physics::put(d, "body_contact_steps", PyLong_FromLong(r.body_contact_steps));
  pycvc_physics::put(d, "books", books_dict(r.books));
  return d;
}

} // namespace pycvc_mujoco
%}

// ── availability ──────────────────────────────────────────────────────────────

%feature("kwargs") pycvc::run_mujoco_trial;

%inline %{
namespace pycvc {
// Whether this pycvc's libcvc has MuJoCo (CVC_WITH_MUJOCO), and which: e.g. "MuJoCo 4.0.0".
bool mujoco_available() { return cvc::physics::mujoco_robot::available(); }
std::string mujoco_version() { return cvc::physics::mujoco_robot::version(); }

// run_mujoco_trial (mujoco_robot.h) as a dict: the robot on a plane rising along +x at slope_deg,
// settled under its servos until push_at, pushed on its base with `push` (N, world axes) for
// push_for, then run to `duration` (s).
PyObject *run_mujoco_trial(const std::string &model, double slope_deg = 0.0, double yaw = 0.0,
                           PyObject *push = nullptr, double push_at = 1.0, double push_for = 0.1,
                           double duration = 4.0, double kp = 60.0, double kd = 2.0,
                           double timestep = 0.0, double half_extent = 4.0, double cell = 0.05) {
  cvc::physics::mujoco_trial_options t;
  t.model = model;
  t.slope_deg = slope_deg;
  t.yaw = yaw;
  t.push = pycvc_physics::vec3_of(push, "run_mujoco_trial: push");
  t.settle = push_at;
  t.push_duration = push_for;
  t.duration = duration;
  t.kp = kp;
  t.kd = kd;
  t.timestep = timestep;
  t.half_extent = half_extent;
  t.cell = cell;
  cvc::physics::mujoco_trial_result r;
  {
    pycvc::gil_release nogil;
    r = cvc::physics::run_mujoco_trial(t);
  }
  return pycvc_mujoco::trial_dict(r);
}
} // namespace pycvc
%}

// ── cvc::physics::mujoco_snapshot ─────────────────────────────────────────────
// Opaque: what save() returns and restore() takes.

%rename(mujoco_snapshot) cvc::physics::mujoco_snapshot;

namespace cvc {
namespace physics {
struct mujoco_snapshot {};
} // namespace physics
} // namespace cvc

// ── cvc::physics::mujoco_robot ────────────────────────────────────────────────

%rename(mujoco_robot) cvc::physics::mujoco_robot;
%feature("kwargs") cvc::physics::mujoco_robot::mujoco_robot;
%feature("kwargs") cvc::physics::mujoco_robot::set_wind;

namespace cvc {
namespace physics {
class mujoco_robot {
public:
  ~mujoco_robot();
};
} // namespace physics
} // namespace cvc

%extend cvc::physics::mujoco_robot {
  // The MJCF file `model`; `terrain` (a pycvc.height_field, copied) replaces its geom `floor`
  // (None keeps the model's floor). The start: `keyframe`'s joint angles with the base at (x, y)
  // heading yaw, tilted with the terrain when align_to_terrain. Every actuator starts in `mode`
  // ("position" or "torque") with the servo gains kp, kd. timestep 0 keeps the model's.
  mujoco_robot(const std::string &model, const cvc::height_field *terrain = nullptr,
               const std::string &floor = "floor", double timestep = 0.0, bool energy = true,
               const std::string &keyframe = "home", double x = 0.0, double y = 0.0,
               double yaw = 0.0, bool align_to_terrain = true,
               const std::string &mode = "position", double kp = 60.0, double kd = 2.0) {
    cvc::physics::mujoco_robot_options o;
    o.model = model;
    if (terrain)
      o.terrain = std::make_shared<cvc::height_field>(*terrain);
    o.floor = floor;
    o.timestep = timestep;
    o.energy = energy;
    o.keyframe = keyframe;
    o.x = x;
    o.y = y;
    o.yaw = yaw;
    o.align_to_terrain = align_to_terrain;
    o.mode = pycvc_mujoco::mode_of(mode);
    o.kp = kp;
    o.kd = kd;
    return new cvc::physics::mujoco_robot(o);
  }

  // ── the model ──
  int nq() const { return $self->nq(); }
  int nv() const { return $self->nv(); }
  int actuators() const { return $self->actuators(); }
  int bodies() const { return $self->bodies(); }
  int geoms() const { return $self->geoms(); }
  // By name; -1 when there is none.
  int actuator(const std::string &name) const { return $self->actuator(name); }
  int body(const std::string &name) const { return $self->body(name); }
  int geom(const std::string &name) const { return $self->geom(name); }
  std::string actuator_name(int i) const { return $self->actuator_name(i); }
  std::string body_name(int b) const { return $self->body_name(b); }
  std::string geom_name(int g) const { return $self->geom_name(g); }
  int geom_body(int g) const { return $self->geom_body(g); }
  // The terrain's geom (-1 when the model's floor was kept).
  int terrain_geom() const { return $self->terrain_geom(); }
  // Actuator i's joint, and where its angle is in qpos and its speed in qvel.
  int actuator_joint(int i) const { return $self->actuator_joint(i); }
  int actuator_qpos(int i) const { return $self->actuator_qpos(i); }
  int actuator_qvel(int i) const { return $self->actuator_qvel(i); }
  // Actuator i's force range (min, max), N m.
  PyObject *force_range(int i) const {
    return Py_BuildValue("(dd)", $self->force_min(i), $self->force_max(i));
  }
  double mass() const { return $self->mass(); }
  double timestep() const { return $self->timestep(); }
  double time() const { return $self->time(); }

  // ── commands ──
  // Every actuator, or actuator i, to "position" or "torque"; its command restarts (a servo holds
  // its joint where it is, a motor applies 0).
  void set_mode(const std::string &mode) { $self->set_mode(pycvc_mujoco::mode_of(mode)); }
  void set_mode(int i, const std::string &mode) { $self->set_mode(i, pycvc_mujoco::mode_of(mode)); }
  std::string mode(int i) const { return pycvc_mujoco::mode_name($self->mode(i)); }
  void set_gains(int i, double kp, double kd) { $self->set_gains(i, kp, kd); }
  double kp(int i) const { return $self->kp(i); }
  double kd(int i) const { return $self->kd(i); }
  // Every actuator's command (one number each: a target in position mode, a force in torque
  // mode), or actuator i's.
  void command(PyObject *u) {
    $self->command(pycvc_mujoco::doubles_of(u, "mujoco_robot.command"));
  }
  void command(int i, double u) { $self->command(i, u); }
  pycvc::ArrayView commands() const { return pycvc_mujoco::array_of($self->commands()); }
  // Every position actuator's target becomes its joint's angle now.
  void hold() { $self->hold(); }

  // ── pushes ── held until replaced or cleared; world axes.
  void apply_wrench(int body, PyObject *force, PyObject *torque = nullptr) {
    $self->apply_wrench(body, pycvc_physics::vec3_of(force, "mujoco_robot.apply_wrench: force"),
                        pycvc_physics::vec3_of(torque, "mujoco_robot.apply_wrench: torque"));
  }
  void apply_force_at(int body, PyObject *force, PyObject *point) {
    $self->apply_force_at(body, pycvc_physics::vec3_of(force, "mujoco_robot.apply_force_at: force"),
                          pycvc_physics::vec3_of(point, "mujoco_robot.apply_force_at: point"));
  }
  void clear_wrenches() { $self->clear_wrenches(); }

  // ── the wind ── the air the robot stands in (a pycvc.wind_flow or scene_wind_flow; None: still
  // air), read at each step's start at time() + time_offset: the time its owner reads it at (a
  // physics world's field time at the robot's start). Each moving geom takes 1/2 rho C_d A |u| u.
  // wind_force() and wind_torque() (about the world's origin) are the last step's, N and N m.
  void set_wind(std::shared_ptr<const cvc::physics::flow_velocity> air, double time_offset = 0.0,
                double drag_coefficient = 1.0, double density = 1.225) {
    $self->set_wind(std::move(air), time_offset, drag_coefficient, density);
  }
  PyObject *wind_force() const { return pycvc_physics::tuple3($self->wind_force()); }
  PyObject *wind_torque() const { return pycvc_physics::tuple3($self->wind_torque()); }

  // ── stepping ── n fixed steps, without the GIL.
  void step(int n = 1) {
    pycvc::gil_release nogil;
    $self->step(n);
  }

  // ── the state ──
  pycvc::ArrayView qpos() const { return pycvc_mujoco::array_of($self->qpos()); }
  pycvc::ArrayView qvel() const { return pycvc_mujoco::array_of($self->qvel()); }
  // The floating base as a dict: position, rotation (w, x, y, z), velocity and spin (world axes),
  // up_z (its up axis's world z).
  PyObject *base() const {
    PyObject *d = PyDict_New();
    pycvc_physics::put(d, "position", pycvc_physics::tuple3($self->base_position()));
    pycvc_physics::put(d, "rotation", pycvc_physics::tuple4($self->base_rotation()));
    pycvc_physics::put(d, "velocity", pycvc_physics::tuple3($self->base_velocity()));
    pycvc_physics::put(d, "spin", pycvc_physics::tuple3($self->base_spin()));
    pycvc_physics::put(d, "up_z", PyFloat_FromDouble($self->base_up_z()));
    return d;
  }
  PyObject *body_position(int b) const { return pycvc_physics::tuple3($self->body_position(b)); }
  PyObject *body_rotation(int b) const { return pycvc_physics::tuple4($self->body_rotation(b)); }

  // ── the last step ──
  pycvc::ArrayView actuator_forces() const {
    return pycvc_mujoco::array_of($self->actuator_forces());
  }
  // A dict per contact: geom and body (pairs; body 0 is the world), position, normal (from geom[0]
  // toward geom[1]), distance (< 0 overlapping), force and torque on body[1] (world axes).
  PyObject *contacts() const {
    const std::vector<cvc::physics::mujoco_contact> c = $self->contacts();
    PyObject *l = PyList_New(static_cast<Py_ssize_t>(c.size()));
    for (std::size_t i = 0; i < c.size(); ++i)
      PyList_SET_ITEM(l, static_cast<Py_ssize_t>(i), pycvc_mujoco::contact_dict(c[i]));
    return l;
  }

  // ── energy ── RuntimeError when built with energy=False.
  // MuJoCo's energies of the current state: potential, kinetic, total (J).
  PyObject *energy() const {
    const cvc::physics::mujoco_energy e = $self->energy();
    return Py_BuildValue("{s:d,s:d,s:d}", "potential", e.potential, "kinetic", e.kinetic,
                         "total", e.total());
  }
  // The work each kind of force did since the start (J): actuators, external, contacts,
  // joint_friction, joint_limits, joint_damping, other; energy_start, energy_now, work, residual.
  PyObject *books() const { return pycvc_mujoco::books_dict($self->books()); }

  // ── snapshots ──
  cvc::physics::mujoco_snapshot save() const { return $self->save(); }
  void restore(const cvc::physics::mujoco_snapshot &s) { $self->restore(s); }
  // Back to how the robot was built (a gym's reset).
  void reset() { $self->reset(); }
}
