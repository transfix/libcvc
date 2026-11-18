/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// MuJoCo behind cvc::physics::mujoco_robot.
//
// This file is compiled as its own target (cvc_physics_mujoco, src/cvc/CMakeLists.txt) with
// mujoco::mujoco linked PRIVATE, so MuJoCo's headers reach only this translation unit. It sees the
// std-only cvc/physics headers and nothing else of libcvc.
//
// Rules:
//  - No process-wide state: no mju_user_* handlers, no plugin registration, no statics. The model
//    and the data are this object's; the model's actuator gains are rewritten in place when a mode
//    or gain changes, which touches no other robot.
//  - Readouts never move the trajectory. The last step's actuator forces and contacts are copied
//    out right after it; reading the current state (energies, body poses) runs mj_forward, which
//    writes only derived quantities, and the solver's warm start is put back around it.
//  - The terrain is a MuJoCo hfield in place of the model's floor geom, with the floor's contact
//    parameters. MuJoCo stores an hfield's elevations normalized to [0, 1] (it rescales whatever
//    it is given), so the geom sits at the lowest height and the field's size[2] is the range.
//    Its rows run along +y from the minimum y and its columns along +x, and each cell is split
//    from (r, c) to (r + 1, c + 1), as in cvc::height_field.

#include "../mujoco_engine.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <mujoco/mujoco.h>
#include <stdexcept>

namespace cvc {
namespace physics {
namespace detail {

namespace {

constexpr const char *kTerrain = "cvc_terrain";
constexpr double kInf = std::numeric_limits<double>::infinity();
// The depth of the terrain's solid below its lowest point (MuJoCo's hfield size[3]), m.
constexpr double kTerrainBase = 1.0;
// How far above the terrain the start is lifted once nothing reaches into it, m.
constexpr double kStartClearance = 1e-4;

struct spec_free {
  void operator()(mjSpec *s) const { mj_deleteSpec(s); }
};
struct model_free {
  void operator()(mjModel *m) const { mj_deleteModel(m); }
};
struct data_free {
  void operator()(mjData *d) const { mj_deleteData(d); }
};

std::string name_of(const mjModel *m, int type, int id) {
  const char *n = mj_id2name(m, type, id);
  return n ? std::string(n) : std::string();
}

quat mul(const quat &a, const quat &b) {
  return {
      a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z, a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
      a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x, a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w};
}

quat normalized(const quat &q) {
  const double n = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
  return {q.w / n, q.x / n, q.y / n, q.z / n};
}

// The shortest rotation taking +z to the unit vector n (n.z > -1).
quat tilt_to(const vec3 &n) { return normalized({1.0 + n.z, -n.y, n.x, 0.0}); }

vec3 rotate(const quat &q, const vec3 &v) {
  const quat p = mul(mul(q, {0.0, v.x, v.y, v.z}), {q.w, -q.x, -q.y, -q.z});
  return {p.x, p.y, p.z};
}

double dot(const mjtNum *a, const std::vector<mjtNum> &b) {
  double s = 0.0;
  for (std::size_t i = 0; i < b.size(); ++i)
    s += a[i] * b[i];
  return s;
}

// Replace the geom `floor` with an hfield geom holding `g`, in the world body, with the floor's
// contact and visual parameters.
void add_terrain(mjSpec *s, const std::string &floor, const mujoco_grid &g) {
  mjsElement *fe = mjs_findElement(s, mjOBJ_GEOM, floor.c_str());
  if (!fe)
    throw std::runtime_error("no geom named '" + floor + "' to replace with the terrain");
  const mjsGeom *f = mjs_asGeom(fe);
  mjsGeom keep = *f; // the numbers; the strings are copied below, before the floor goes
  const std::string material = f->material ? mjs_getString(f->material) : "";
  if (mjs_delete(s, fe) != 0)
    throw std::runtime_error("could not remove the floor geom '" + floor + "'");

  const auto mm = std::minmax_element(g.heights.begin(), g.heights.end());
  const double lo = *mm.first, hi = *mm.second;
  mjsHField *hf = mjs_addHField(s);
  mjs_setName(hf->element, kTerrain);
  hf->nrow = g.rows;
  hf->ncol = g.cols;
  hf->size[0] = 0.5 * g.dx * (g.cols - 1);
  hf->size[1] = 0.5 * g.dy * (g.rows - 1);
  hf->size[2] = hi > lo ? hi - lo : 1.0; // a flat field normalizes to 0 whatever this is
  hf->size[3] = kTerrainBase;
  mjs_setFloat(hf->userdata, g.heights.data(), static_cast<int>(g.heights.size()));

  mjsGeom *t = mjs_addGeom(mjs_findBody(s, "world"), nullptr);
  mjs_setName(t->element, kTerrain);
  t->type = mjGEOM_HFIELD;
  mjs_setString(t->hfieldname, kTerrain);
  t->pos[0] = g.x0 + hf->size[0];
  t->pos[1] = g.y0 + hf->size[1];
  t->pos[2] = lo;
  t->contype = keep.contype;
  t->conaffinity = keep.conaffinity;
  t->condim = keep.condim;
  t->priority = keep.priority;
  std::copy(keep.friction, keep.friction + 3, t->friction);
  t->solmix = keep.solmix;
  std::copy(keep.solref, keep.solref + mjNREF, t->solref);
  std::copy(keep.solimp, keep.solimp + mjNIMP, t->solimp);
  t->margin = keep.margin;
  t->gap = keep.gap;
  t->group = keep.group;
  std::copy(keep.rgba, keep.rgba + 4, t->rgba);
  if (!material.empty())
    mjs_setString(t->material, material.c_str());
}

// One single-joint actuator, as this robot drives it.
struct actuator_info {
  int joint = -1, qpos = -1, qvel = -1;
  double gear = 1.0;
  double fmin = -kInf, fmax = kInf; // N m
  bool joint_limited = false;       // and its range, rad
  double jmin = 0.0, jmax = 0.0;
  actuator_mode mode = actuator_mode::torque;
  double kp = 0.0, kd = 0.0;
};

class engine final : public mujoco_engine {
public:
  engine(const mujoco_robot_options &o, const mujoco_grid *grid, const mujoco_ground &ground) {
    char err[1024] = "";
    std::unique_ptr<mjSpec, spec_free> spec(mj_parseXML(o.model.c_str(), nullptr, err, sizeof err));
    if (!spec)
      throw std::runtime_error(o.model + ": " + err);
    if (grid)
      add_terrain(spec.get(), o.floor, *grid);
    if (o.timestep > 0.0)
      spec->option.timestep = o.timestep;
    if (o.energy)
      spec->option.enableflags |= mjENBL_ENERGY;
    _m.reset(mj_compile(spec.get(), nullptr));
    if (!_m)
      throw std::runtime_error(o.model + ": " + mjs_getError(spec.get()));
    spec.reset();
    _d.reset(mj_makeData(_m.get()));
    if (!_d)
      throw std::runtime_error(o.model + ": MuJoCo could not allocate its data");
    mjModel *m = _m.get();
    mjData *d = _d.get();

    if (m->nout != m->nu || m->nactuator != m->nu)
      throw std::runtime_error(o.model + ": every actuator must have one control and one output");
    _act.resize(static_cast<std::size_t>(m->nu));
    for (int i = 0; i < m->nu; ++i) {
      actuator_info &a = _act[static_cast<std::size_t>(i)];
      if (m->actuator_trntype[i] != mjTRN_JOINT)
        throw std::runtime_error(o.model + ": actuator '" + name_of(m, mjOBJ_ACTUATOR, i) +
                                 "' does not drive a joint");
      a.joint = m->actuator_trnid[2 * i];
      const int type = m->jnt_type[a.joint];
      if (type != mjJNT_HINGE && type != mjJNT_SLIDE)
        throw std::runtime_error(o.model + ": actuator '" + name_of(m, mjOBJ_ACTUATOR, i) +
                                 "' drives a joint with more than one degree of freedom");
      a.qpos = m->jnt_qposadr[a.joint];
      a.qvel = m->jnt_dofadr[a.joint];
      a.gear = m->actuator_gear[6 * i];
      if (m->actuator_forcelimited[i]) {
        a.fmin = m->actuator_forcerange[2 * i];
        a.fmax = m->actuator_forcerange[2 * i + 1];
      } else if (m->actuator_gaintype[i] == mjGAIN_FIXED &&
                 m->actuator_biastype[i] == mjBIAS_NONE && m->actuator_ctrllimited[i]) {
        const double g = m->actuator_gainprm[mjNGAIN * i];
        const double u0 = g * m->actuator_ctrlrange[2 * i],
                     u1 = g * m->actuator_ctrlrange[2 * i + 1];
        a.fmin = std::min(u0, u1);
        a.fmax = std::max(u0, u1);
      }
      a.joint_limited = m->jnt_limited[a.joint] != 0;
      a.jmin = m->jnt_range[2 * a.joint];
      a.jmax = m->jnt_range[2 * a.joint + 1];
      a.kp = o.kp;
      a.kd = o.kd;
    }
    for (int j = 0; j < m->njnt; ++j)
      if (m->jnt_type[j] == mjJNT_FREE) {
        _free = j;
        break;
      }
    if (grid)
      _terrain = mj_name2id(m, mjOBJ_GEOM, kTerrain);

    if (!o.keyframe.empty()) {
      const int key = mj_name2id(m, mjOBJ_KEY, o.keyframe.c_str());
      if (key < 0)
        throw std::runtime_error(o.model + ": no keyframe named '" + o.keyframe + "'");
      mj_resetDataKeyframe(m, d, key);
    } else {
      mj_resetData(m, d);
    }
    place(o, ground);
    for (int i = 0; i < m->nu; ++i)
      set_mode(i, o.mode);
    mj_forward(m, d);
    _fresh = true;
    if (energy_on())
      _books.energy_start = d->energy[0] + d->energy[1];
    _start = save();
  }

  int nq() const override { return static_cast<int>(_m->nq); }
  int nv() const override { return static_cast<int>(_m->nv); }
  int actuators() const override { return static_cast<int>(_m->nu); }
  int bodies() const override { return static_cast<int>(_m->nbody); }
  int geoms() const override { return static_cast<int>(_m->ngeom); }
  int actuator(const std::string &n) const override {
    return mj_name2id(_m.get(), mjOBJ_ACTUATOR, n.c_str());
  }
  int body(const std::string &n) const override {
    return mj_name2id(_m.get(), mjOBJ_BODY, n.c_str());
  }
  int geom(const std::string &n) const override {
    return mj_name2id(_m.get(), mjOBJ_GEOM, n.c_str());
  }
  std::string actuator_name(int i) const override { return name_of(_m.get(), mjOBJ_ACTUATOR, i); }
  std::string body_name(int b) const override { return name_of(_m.get(), mjOBJ_BODY, b); }
  std::string geom_name(int g) const override { return name_of(_m.get(), mjOBJ_GEOM, g); }
  int geom_body(int g) const override { return _m->geom_bodyid[g]; }
  int terrain_geom() const override { return _terrain; }
  int actuator_joint(int i) const override { return at(i).joint; }
  int actuator_qpos(int i) const override { return at(i).qpos; }
  int actuator_qvel(int i) const override { return at(i).qvel; }
  double force_min(int i) const override { return at(i).fmin; }
  double force_max(int i) const override { return at(i).fmax; }
  double mass() const override { return mj_getTotalmass(_m.get()); }
  double timestep() const override { return _m->opt.timestep; }
  double time() const override { return _d->time; }

  void set_mode(int i, actuator_mode mode) override {
    actuator_info &a = at(i);
    a.mode = mode;
    write(i);
    _d->ctrl[i] = mode == actuator_mode::position ? a.gear * _d->qpos[a.qpos] : 0.0;
  }
  actuator_mode mode(int i) const override { return at(i).mode; }
  void set_gains(int i, double kp, double kd) override {
    at(i).kp = kp;
    at(i).kd = kd;
    write(i);
  }
  double kp(int i) const override { return at(i).kp; }
  double kd(int i) const override { return at(i).kd; }
  void command(int i, double u) override { _d->ctrl[i] = u; }
  std::vector<double> commands() const override {
    return std::vector<double>(_d->ctrl, _d->ctrl + _m->nu);
  }
  void hold() override {
    for (int i = 0; i < _m->nu; ++i) {
      const actuator_info &a = at(i);
      if (a.mode == actuator_mode::position)
        _d->ctrl[i] = a.gear * _d->qpos[a.qpos];
    }
  }

  void apply_wrench(int b, const vec3 &f, const vec3 &t) override {
    mjtNum *x = _d->xfrc_applied + 6 * b;
    x[0] = f.x, x[1] = f.y, x[2] = f.z;
    x[3] = t.x, x[4] = t.y, x[5] = t.z;
  }
  void clear_wrenches() override {
    std::fill(_d->xfrc_applied, _d->xfrc_applied + 6 * _m->nbody, 0.0);
  }
  vec3 body_com(int b) override {
    refresh();
    const mjtNum *p = _d->xipos + 3 * b;
    return {p[0], p[1], p[2]};
  }
  void set_air(mujoco_air air) override {
    _air = std::move(air);
    _air_f = _air_t = vec3{};
  }
  vec3 air_force() const override { return _air_f; }
  vec3 air_torque() const override { return _air_t; }

  void step() override {
    mjModel *m = _m.get();
    mjData *d = _d.get();
    const std::size_t nv = static_cast<std::size_t>(m->nv);
    _v0.assign(d->qvel, d->qvel + nv);
    const int bad = bad_count();
    const bool air = static_cast<bool>(_air);
    if (air)
      push_air();
    mj_step(m, d);
    if (bad_count() != bad)
      throw std::runtime_error("MuJoCo found the state unusable (NaN or a runaway value in qpos, "
                               "qvel or qacc) and reset it, at t = " +
                               std::to_string(d->time));
    book();  // the air's wrench with apply_wrench's, as external work
    if (air) // the pushes as they were given
      std::copy(_user.begin(), _user.end(), d->xfrc_applied);
    read_back();
    _fresh = false;
  }

  std::vector<double> qpos() const override {
    return std::vector<double>(_d->qpos, _d->qpos + _m->nq);
  }
  std::vector<double> qvel() const override {
    return std::vector<double>(_d->qvel, _d->qvel + _m->nv);
  }
  vec3 base_position() override {
    if (_free < 0)
      return body_position(1);
    const mjtNum *q = _d->qpos + _m->jnt_qposadr[_free];
    return {q[0], q[1], q[2]};
  }
  quat base_rotation() override {
    if (_free < 0)
      return body_rotation(1);
    const mjtNum *q = _d->qpos + _m->jnt_qposadr[_free] + 3;
    return {q[0], q[1], q[2], q[3]};
  }
  vec3 base_velocity() override {
    if (_free < 0)
      return {};
    const mjtNum *v = _d->qvel + _m->jnt_dofadr[_free];
    return {v[0], v[1], v[2]};
  }
  vec3 base_spin() override {
    if (_free < 0)
      return {};
    const mjtNum *w = _d->qvel + _m->jnt_dofadr[_free] + 3;
    return rotate(base_rotation(), {w[0], w[1], w[2]}); // MuJoCo keeps it in body axes
  }
  vec3 body_position(int b) override {
    refresh();
    const mjtNum *p = _d->xpos + 3 * b;
    return {p[0], p[1], p[2]};
  }
  quat body_rotation(int b) override {
    refresh();
    const mjtNum *q = _d->xquat + 4 * b;
    return {q[0], q[1], q[2], q[3]};
  }

  std::vector<double> actuator_forces() const override { return _forces; }
  std::vector<mujoco_contact> contacts() const override { return _contacts; }

  bool energy_on() const override { return (_m->opt.enableflags & mjENBL_ENERGY) != 0; }
  mujoco_energy energy() override {
    refresh();
    return {_d->energy[0], _d->energy[1]};
  }
  mujoco_books books() override {
    mujoco_books b = _books;
    if (energy_on())
      b.energy_now = energy().total();
    return b;
  }

  mujoco_snapshot save() const override {
    mujoco_snapshot s;
    s.state.resize(static_cast<std::size_t>(mj_stateSize(_m.get(), mjSTATE_INTEGRATION)));
    mj_getState(_m.get(), _d.get(), s.state.data(), mjSTATE_INTEGRATION);
    for (const actuator_info &a : _act) {
      s.modes.push_back(static_cast<std::uint8_t>(a.mode));
      s.kp.push_back(a.kp);
      s.kd.push_back(a.kd);
    }
    s.books = _books;
    return s;
  }
  void restore(const mujoco_snapshot &s) override {
    if (s.state.size() != static_cast<std::size_t>(mj_stateSize(_m.get(), mjSTATE_INTEGRATION)) ||
        s.modes.size() != _act.size() || s.kp.size() != _act.size() || s.kd.size() != _act.size())
      throw std::runtime_error("the snapshot is not this robot's");
    mj_setState(_m.get(), _d.get(), s.state.data(), mjSTATE_INTEGRATION);
    for (std::size_t i = 0; i < _act.size(); ++i) {
      _act[i].mode = static_cast<actuator_mode>(s.modes[i]);
      _act[i].kp = s.kp[i];
      _act[i].kd = s.kd[i];
      write(static_cast<int>(i));
    }
    _books = s.books;
    _forces.clear();
    _contacts.clear();
    _fresh = false;
  }
  void reset() override { restore(_start); }

private:
  actuator_info &at(int i) { return _act.at(static_cast<std::size_t>(i)); }
  const actuator_info &at(int i) const { return _act.at(static_cast<std::size_t>(i)); }

  // Actuator i's gains, bias and ranges in the model, for its mode.
  void write(int i) {
    mjModel *m = _m.get();
    const actuator_info &a = at(i);
    mjtNum *gain = m->actuator_gainprm + mjNGAIN * i;
    mjtNum *bias = m->actuator_biasprm + mjNBIAS * i;
    std::fill(gain, gain + mjNGAIN, 0.0);
    std::fill(bias, bias + mjNBIAS, 0.0);
    m->actuator_gaintype[i] = mjGAIN_FIXED;
    const bool limited = std::isfinite(a.fmin) || std::isfinite(a.fmax);
    m->actuator_forcelimited[i] = limited;
    m->actuator_forcerange[2 * i] = std::isfinite(a.fmin) ? a.fmin : -mjMAXVAL;
    m->actuator_forcerange[2 * i + 1] = std::isfinite(a.fmax) ? a.fmax : mjMAXVAL;
    if (a.mode == actuator_mode::position) {
      gain[0] = a.kp;
      m->actuator_biastype[i] = mjBIAS_AFFINE;
      bias[1] = -a.kp;
      bias[2] = -a.kd;
      m->actuator_ctrllimited[i] = a.joint_limited;
      const double c0 = a.gear * a.jmin, c1 = a.gear * a.jmax;
      m->actuator_ctrlrange[2 * i] = std::min(c0, c1);
      m->actuator_ctrlrange[2 * i + 1] = std::max(c0, c1);
    } else {
      gain[0] = 1.0;
      m->actuator_biastype[i] = mjBIAS_NONE;
      m->actuator_ctrllimited[i] = limited;
      m->actuator_ctrlrange[2 * i] = m->actuator_forcerange[2 * i];
      m->actuator_ctrlrange[2 * i + 1] = m->actuator_forcerange[2 * i + 1];
    }
  }

  // The keyframe's base over `ground`, then raised until no geom reaches into the world's geoms.
  void place(const mujoco_robot_options &o, const mujoco_ground &g) {
    if (_free < 0)
      return;
    mjModel *m = _m.get();
    mjData *d = _d.get();
    mjtNum *q = d->qpos + m->jnt_qposadr[_free];
    const double h0 = q[2];
    const quat key{q[3], q[4], q[5], q[6]};
    const quat yaw{std::cos(0.5 * o.yaw), 0.0, 0.0, std::sin(0.5 * o.yaw)};
    const vec3 up = o.align_to_terrain ? g.normal : vec3{0.0, 0.0, 1.0};
    const quat r = normalized(mul(o.align_to_terrain ? tilt_to(g.normal) : quat{}, mul(yaw, key)));
    q[0] = g.point.x + up.x * h0;
    q[1] = g.point.y + up.y * h0;
    q[2] = g.point.z + up.z * h0;
    q[3] = r.w, q[4] = r.x, q[5] = r.y, q[6] = r.z;
    for (int it = 0; it < 64; ++it) {
      mj_forward(m, d);
      double deepest = 0.0;
      for (int c = 0; c < d->ncon; ++c) {
        const mjContact &k = d->contact[c];
        if (k.geom[0] < 0 || k.geom[1] < 0)
          continue;
        const bool w0 = m->geom_bodyid[k.geom[0]] == 0, w1 = m->geom_bodyid[k.geom[1]] == 0;
        if (w0 != w1)
          deepest = std::min(deepest, k.dist);
      }
      if (deepest >= 0.0)
        break;
      q[2] += kStartClearance - deepest;
    }
  }

  int bad_count() const {
    return _d->warning[mjWARN_BADQPOS].number + _d->warning[mjWARN_BADQVEL].number +
           _d->warning[mjWARN_BADQACC].number;
  }

  // The air's wrench for the step about to be taken, added to xfrc_applied (the pushes kept in
  // _user, put back after the step). The poses and velocities it reads are the step's start's,
  // computed here from qpos and qvel (mj_kinematics, mj_comPos, mj_comVel): mj_step computes the
  // same again from the same state, so this changes nothing else of the step.
  void push_air() {
    const mjModel *m = _m.get();
    mjData *d = _d.get();
    mj_kinematics(m, d);
    mj_comPos(m, d);
    mj_comVel(m, d);
    _air_geoms.clear();
    for (int g = 0; g < m->ngeom; ++g) {
      const int b = m->geom_bodyid[g];
      if (b == 0 || (m->geom_contype[g] == 0 && m->geom_conaffinity[g] == 0))
        continue; // the world's geoms (the terrain), and the visual-only ones
      const mjtNum *box = m->geom_aabb + 6 * g; // centre and half sizes, in the geom's frame
      const mjtNum *R = d->geom_xmat + 9 * g;   // row-major: world = R local
      const mjtNum *p = d->geom_xpos + 3 * g;
      mujoco_air_geom a;
      a.body = b;
      a.centre = {p[0] + R[0] * box[0] + R[1] * box[1] + R[2] * box[2],
                  p[1] + R[3] * box[0] + R[4] * box[1] + R[5] * box[2],
                  p[2] + R[6] * box[0] + R[7] * box[1] + R[8] * box[2]};
      for (int k = 0; k < 3; ++k)
        a.axis[k] = {R[k], R[3 + k], R[6 + k]};
      a.half = {box[3], box[4], box[5]};
      mjtNum v[6]; // at the geom's origin: angular, then linear, world axes
      mj_objectVelocity(m, d, mjOBJ_GEOM, g, v, 0);
      const vec3 w{v[0], v[1], v[2]}, r = a.centre - vec3{p[0], p[1], p[2]};
      a.velocity = {v[3] + w.y * r.z - w.z * r.y, v[4] + w.z * r.x - w.x * r.z,
                    v[5] + w.x * r.y - w.y * r.x};
      _air_geoms.push_back(a);
    }
    const std::size_t nb = static_cast<std::size_t>(m->nbody);
    _air_coms.resize(nb);
    for (std::size_t b = 0; b < nb; ++b)
      _air_coms[b] = {d->xipos[3 * b], d->xipos[3 * b + 1], d->xipos[3 * b + 2]};
    _air_wrench.assign(6 * nb, 0.0);
    _air(d->time, _air_geoms, _air_coms, _air_wrench);
    _user.assign(d->xfrc_applied, d->xfrc_applied + 6 * nb);
    _air_f = _air_t = vec3{};
    for (std::size_t b = 1; b < nb; ++b) {
      const double *w = _air_wrench.data() + 6 * b;
      for (int k = 0; k < 6; ++k)
        d->xfrc_applied[6 * b + k] += w[k];
      const vec3 f{w[0], w[1], w[2]}, c = _air_coms[b];
      _air_f = _air_f + f;
      _air_t = _air_t + vec3{w[3] + c.y * f.z - c.z * f.y, w[4] + c.z * f.x - c.x * f.z,
                             w[5] + c.x * f.y - c.y * f.x};
    }
  }

  // The current state's derived quantities (energies, poses), leaving the trajectory alone.
  void refresh() {
    if (_fresh)
      return;
    const std::size_t nv = static_cast<std::size_t>(_m->nv);
    _warm.assign(_d->qacc_warmstart, _d->qacc_warmstart + nv);
    mj_forward(_m.get(), _d.get());
    std::copy(_warm.begin(), _warm.end(), _d->qacc_warmstart);
    _fresh = true;
  }

  // The step just taken into the books: each force mj_step computed from the step's start, against
  // the mean of the start's and the end's velocities.
  void book() {
    const mjModel *m = _m.get();
    mjData *d = _d.get();
    const std::size_t nv = static_cast<std::size_t>(m->nv);
    const double h = m->opt.timestep;
    _vbar.resize(nv);
    for (std::size_t i = 0; i < nv; ++i)
      _vbar[i] = 0.5 * (_v0[i] + d->qvel[i]);

    _books.actuators += h * dot(d->qfrc_actuator, _vbar);

    _tmp.assign(d->qfrc_applied, d->qfrc_applied + nv);
    for (int b = 1; b < m->nbody; ++b) {
      const mjtNum *x = d->xfrc_applied + 6 * b;
      if (x[0] != 0.0 || x[1] != 0.0 || x[2] != 0.0 || x[3] != 0.0 || x[4] != 0.0 || x[5] != 0.0)
        mj_applyFT(m, d, x, x + 3, d->xipos + 3 * b, b, _tmp.data());
    }
    _books.external += h * dot(_tmp.data(), _vbar);

    if (d->nefc > 0) {
      _jv.resize(static_cast<std::size_t>(d->nefc));
      mj_mulJacVec(m, d, _jv.data(), _vbar.data());
      for (int r = 0; r < d->nefc; ++r) {
        const double w = h * d->efc_force[r] * _jv[static_cast<std::size_t>(r)];
        switch (d->efc_type[r]) {
        case mjCNSTR_CONTACT_FRICTIONLESS:
        case mjCNSTR_CONTACT_PYRAMIDAL:
        case mjCNSTR_CONTACT_ELLIPTIC:
          _books.contacts += w;
          break;
        case mjCNSTR_FRICTION_DOF:
        case mjCNSTR_FRICTION_TENDON:
          _books.joint_friction += w;
          break;
        case mjCNSTR_LIMIT_JOINT:
        case mjCNSTR_LIMIT_TENDON:
          _books.joint_limits += w;
          break;
        default:
          _books.other += w;
          break;
        }
      }
    }

    // Euler (unless its damping is made explicit) and the implicit integrators apply the joints'
    // damping at the end's velocity: -D v1, not the -D v0 that qfrc_damper holds.
    const bool implicit_damping =
        (m->opt.integrator == mjINT_EULER && !(m->opt.disableflags & mjDSBL_EULERDAMP)) ||
        m->opt.integrator == mjINT_IMPLICIT || m->opt.integrator == mjINT_IMPLICITFAST;
    double damping = 0.0, other = 0.0;
    for (std::size_t i = 0; i < nv; ++i) {
      double fd = d->qfrc_damper[i];
      if (implicit_damping)
        fd += m->dof_damping[i] * (_v0[i] - d->qvel[i]);
      damping += fd * _vbar[i];
      other += (d->qfrc_passive[i] - d->qfrc_spring[i] - d->qfrc_damper[i]) * _vbar[i];
    }
    _books.joint_damping += h * damping;
    _books.other += h * other;
  }

  // The last step's actuator forces and contacts, before anything recomputes them.
  void read_back() {
    const mjModel *m = _m.get();
    const mjData *d = _d.get();
    _forces.assign(d->actuator_force, d->actuator_force + m->nu);
    _contacts.clear();
    for (int c = 0; c < d->ncon; ++c) {
      const mjContact &k = d->contact[c];
      if (k.efc_address < 0)
        continue; // detected, but not in the solve (inside its margin's gap, or excluded)
      mujoco_contact x;
      for (int s = 0; s < 2; ++s) {
        x.geom[s] = k.geom[s];
        x.body[s] = k.geom[s] >= 0 ? m->geom_bodyid[k.geom[s]] : -1;
      }
      x.position = {k.pos[0], k.pos[1], k.pos[2]};
      const vec3 n{k.frame[0], k.frame[1], k.frame[2]};
      const vec3 t1{k.frame[3], k.frame[4], k.frame[5]};
      const vec3 t2{k.frame[6], k.frame[7], k.frame[8]};
      x.normal = n;
      x.distance = k.dist;
      mjtNum f[6];
      mj_contactForce(m, d, c, f);
      x.force = n * f[0] + t1 * f[1] + t2 * f[2];
      x.torque = n * f[3] + t1 * f[4] + t2 * f[5];
      _contacts.push_back(x);
    }
  }

  std::unique_ptr<mjModel, model_free> _m;
  std::unique_ptr<mjData, data_free> _d;
  std::vector<actuator_info> _act;
  int _free = -1, _terrain = -1;
  bool _fresh = false;
  mujoco_books _books;
  mujoco_snapshot _start; // as built, for reset()
  std::vector<double> _forces;
  std::vector<mujoco_contact> _contacts;
  std::vector<mjtNum> _v0, _vbar, _tmp, _jv, _warm;
  // The air (set_air): its function, what it reads and gives a step, the pushes kept aside, and
  // its last step's sums.
  mujoco_air _air;
  std::vector<mujoco_air_geom> _air_geoms;
  std::vector<vec3> _air_coms;
  std::vector<double> _air_wrench;
  std::vector<mjtNum> _user;
  vec3 _air_f, _air_t;
};

} // namespace

std::unique_ptr<mujoco_engine> make_mujoco_engine(const mujoco_robot_options &options,
                                                  const mujoco_grid *grid,
                                                  const mujoco_ground &ground) {
  return std::make_unique<engine>(options, grid, ground);
}

std::string mujoco_version() { return std::string("MuJoCo ") + mj_versionString(); }

} // namespace detail
} // namespace physics
} // namespace cvc
