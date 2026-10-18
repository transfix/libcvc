/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <charconv>
#include <cmath>
#include <cstdio>
#include <cvc/physics/backend.h>
#include <cvc/physics/snapshot_json.h>
#include <vector>

namespace cvc {
namespace physics {

namespace {

// A streaming writer: objects and arrays nest, and each value after the first in one is
// preceded by a comma.
class writer {
public:
  std::string out;

  void begin_object() {
    comma();
    out += '{';
    _first.push_back(true);
  }
  void end_object() {
    out += '}';
    _first.pop_back();
  }
  void begin_array() {
    comma();
    out += '[';
    _first.push_back(true);
  }
  void end_array() {
    out += ']';
    _first.pop_back();
  }
  // A key in the current object; the value follows.
  writer &key(const char *k) {
    comma();
    out += '"';
    out += k;
    out += "\":";
    _pending = true;
    return *this;
  }

  void number(double v) {
    comma();
    if (!std::isfinite(v)) {
      out += "null";
      return;
    }
    char buf[32];
#if defined(__cpp_lib_to_chars) && __cpp_lib_to_chars >= 201611L
    // The shortest form that reads back exactly, and no locale.
    const int n = static_cast<int>(std::to_chars(buf, buf + sizeof(buf), v).ptr - buf);
#else
    // 17 significant digits read back exactly; a locale's decimal comma is undone below.
    const int n = std::snprintf(buf, sizeof(buf), "%.17g", v);
#endif
    bool point = false;
    for (int i = 0; i < n; ++i) {
      char c = buf[i];
      if (c == ',')
        c = '.';
      point = point || c == '.' || c == 'e' || c == 'E';
      out += c;
    }
    if (!point)
      out += ".0"; // a double reads back as one
  }
  void integer(std::uint64_t v) {
    comma();
    char buf[24];
    out.append(buf, std::to_chars(buf, buf + sizeof(buf), v).ptr);
  }
  void integer(std::int64_t v) {
    comma();
    char buf[24];
    out.append(buf, std::to_chars(buf, buf + sizeof(buf), v).ptr);
  }
  void integer(int v) { integer(static_cast<std::int64_t>(v)); }
  void integer(std::uint32_t v) { integer(static_cast<std::uint64_t>(v)); }
  void boolean(bool v) {
    comma();
    out += v ? "true" : "false";
  }
  void null() {
    comma();
    out += "null";
  }
  void string(const std::string &v) {
    comma();
    out += '"';
    for (const char ch : v) {
      const unsigned char c = static_cast<unsigned char>(ch);
      switch (c) {
      case '"':
        out += "\\\"";
        break;
      case '\\':
        out += "\\\\";
        break;
      case '\n':
        out += "\\n";
        break;
      case '\r':
        out += "\\r";
        break;
      case '\t':
        out += "\\t";
        break;
      default:
        if (c < 0x20) {
          char buf[8];
          std::snprintf(buf, sizeof(buf), "\\u%04x", c);
          out += buf;
        } else {
          out += ch; // UTF-8 passes through
        }
      }
    }
    out += '"';
  }
  void vec(const vec3 &v) {
    begin_array();
    number(v.x);
    number(v.y);
    number(v.z);
    end_array();
  }
  void rotation(const quat &q) {
    begin_array();
    number(q.w);
    number(q.x);
    number(q.y);
    number(q.z);
    end_array();
  }
  template <class Id> void id(Id i) {
    if (i.valid())
      integer(i.value);
    else
      null();
  }

private:
  void comma() {
    if (_pending) { // the value of the key just written
      _pending = false;
      return;
    }
    if (_first.empty())
      return;
    if (_first.back())
      _first.back() = false;
    else
      out += ',';
  }

  std::vector<bool> _first;
  bool _pending = false;
};

const char *motion_name(motion_type m) {
  switch (m) {
  case motion_type::static_:
    return "static";
  case motion_type::kinematic:
    return "kinematic";
  default:
    return "dynamic";
  }
}

const char *kind_name(field_kind k) {
  switch (k) {
  case field_kind::fluid:
    return "fluid";
  case field_kind::medium:
    return "medium";
  default:
    return "flow";
  }
}

const char *combine_name(combine_rule r) {
  switch (r) {
  case combine_rule::product:
    return "product";
  case combine_rule::min:
    return "min";
  case combine_rule::max:
    return "max";
  case combine_rule::geometric_mean:
    return "geometric_mean";
  default:
    return "average";
  }
}

const char *contacts_name(contact_reporting c) {
  switch (c) {
  case contact_reporting::none:
    return "none";
  case contact_reporting::added_removed:
    return "added_removed";
  default:
    return "all";
  }
}

const char *contact_type_name(contact_event::kind k) {
  switch (k) {
  case contact_event::kind::added:
    return "added";
  case contact_event::kind::persisted:
    return "persisted";
  default:
    return "removed";
  }
}

void header(writer &w, const world_snapshot &s) {
  w.key("format").string("cvc.physics.snapshot/1");
  w.key("tick").integer(s.tick);
  w.key("time").number(s.time);
  w.key("static_version").integer(s.static_version);
}

void body(writer &w, const body_snapshot &b) {
  w.begin_object();
  w.key("id").id(b.id);
  w.key("motion").string(motion_name(b.motion));
  w.key("material").integer(static_cast<std::uint64_t>(b.settings.material));
  w.key("mass").number(b.settings.mass);
  w.key("linear_damping").number(b.settings.linear_damping);
  w.key("angular_damping").number(b.settings.angular_damping);
  w.key("allow_sleeping").boolean(b.settings.allow_sleeping);
  w.key("user_data").integer(b.user_data);
  w.key("position").vec(b.state.position);
  w.key("rotation").rotation(b.state.rotation);
  w.key("velocity").vec(b.state.linear_velocity);
  w.key("angular_velocity").vec(b.state.angular_velocity);
  w.key("active").boolean(b.state.active);
  w.key("field_force").vec(b.fields.force);
  w.key("field_torque").vec(b.fields.torque);
  w.key("immersed_volume").number(b.fields.immersed_volume);
  w.end_object();
}

void bodies(writer &w, const world_snapshot &s, json_bodies which) {
  w.key("bodies").begin_array();
  if (which != json_bodies::static_only)
    for (const body_snapshot &b : s.bodies)
      body(w, b);
  if (which != json_bodies::moving && s.static_bodies)
    for (const body_snapshot &b : *s.static_bodies)
      body(w, b);
  w.end_array();
}

void vehicle(writer &w, const vehicle_snapshot &v) {
  w.begin_object();
  w.key("id").id(v.id);
  const vehicle_desc &d = v.desc;
  w.key("desc").begin_object();
  w.key("half_extents").vec(d.half_extents);
  w.key("center_of_mass").vec(d.center_of_mass);
  w.key("mass").number(d.mass);
  w.key("max_steer").number(d.max_steer);
  w.key("max_steer_rate").number(d.max_steer_rate);
  w.key("engine_max_torque").number(d.engine_max_torque);
  w.key("engine_min_rpm").number(d.engine_min_rpm);
  w.key("engine_max_rpm").number(d.engine_max_rpm);
  w.key("engine_inertia").number(d.engine_inertia);
  w.key("engine_friction_curve").begin_array();
  for (const auto &p : d.engine_friction_curve) {
    w.begin_array();
    w.number(p.first);
    w.number(p.second);
    w.end_array();
  }
  w.end_array();
  w.key("clutch_strength").number(d.clutch_strength);
  w.key("driveline_efficiency").number(d.driveline_efficiency);
  w.key("torque_curve").begin_array();
  for (const auto &p : d.torque_curve) {
    w.begin_array();
    w.number(p.first);
    w.number(p.second);
    w.end_array();
  }
  w.end_array();
  w.key("gear_ratios").begin_array();
  for (const double g : d.gear_ratios)
    w.number(g);
  w.end_array();
  w.key("reverse_ratio").number(d.reverse_ratio);
  w.key("differential_ratio").number(d.differential_ratio);
  w.key("shift_up_rpm").number(d.shift_up_rpm);
  w.key("shift_down_rpm").number(d.shift_down_rpm);
  w.key("shift_up_rpm_light").number(d.shift_up_rpm_light);
  w.key("shift_down_rpm_light").number(d.shift_down_rpm_light);
  w.key("max_accel").number(d.max_accel);
  w.key("max_decel").number(d.max_decel);
  w.key("drag_coefficient").number(d.drag_coefficient);
  w.key("frontal_area").number(d.frontal_area);
  w.key("air_density").number(d.air_density);
  w.key("rolling_resistance").number(d.rolling_resistance);
  w.key("tire_peak_slip").number(d.tire_peak_slip);
  w.key("tire_peak_slip_angle").number(d.tire_peak_slip_angle);
  w.key("tire_sliding").number(d.tire_sliding);
  {
    const energy_source &e = d.energy;
    w.key("energy_source").begin_object();
    w.key("type").string(e.type == energy_source::kind::battery ? "battery"
                         : e.type == energy_source::kind::fuel  ? "fuel"
                                                                : "none");
    w.key("capacity").number(e.capacity);
    w.key("initial_charge").number(e.initial_charge);
    w.key("motor_efficiency").number(e.motor_efficiency);
    w.key("regen_efficiency").number(e.regen_efficiency);
    w.key("max_regen_power").number(e.max_regen_power);
    w.key("tank").number(e.tank);
    w.key("indicated_efficiency").number(e.indicated_efficiency);
    w.key("heating_value").number(e.heating_value);
    w.key("accessory_power").number(e.accessory_power);
    w.end_object();
  }
  w.key("energy_ledger").boolean(d.energy_ledger);
  w.key("axles").begin_array();
  for (const axle_desc &a : d.axles) {
    w.begin_object();
    w.key("x").number(a.x);
    w.key("half_track").number(a.half_track);
    w.key("z").number(a.z);
    w.key("wheel_radius").number(a.wheel_radius);
    w.key("wheel_width").number(a.wheel_width);
    w.key("wheel_inertia").number(a.wheel_inertia);
    w.key("wheel_angular_damping").number(a.wheel_angular_damping);
    w.key("steered").boolean(a.steered);
    w.key("driven").boolean(a.driven);
    w.key("max_brake_torque").number(a.max_brake_torque);
    w.key("max_hand_brake_torque").number(a.max_hand_brake_torque);
    w.key("suspension_min").number(a.suspension_min);
    w.key("suspension_max").number(a.suspension_max);
    w.key("suspension_frequency").number(a.suspension_frequency);
    w.key("suspension_damping").number(a.suspension_damping);
    w.key("anti_roll_stiffness").number(a.anti_roll_stiffness);
    w.end_object();
  }
  w.end_array();
  w.end_object();

  const vehicle_state &s = v.state;
  w.key("state").begin_object();
  w.key("chassis").id(s.chassis);
  w.key("position").vec(s.body.position);
  w.key("rotation").rotation(s.body.rotation);
  w.key("velocity").vec(s.body.linear_velocity);
  w.key("angular_velocity").vec(s.body.angular_velocity);
  w.key("active").boolean(s.body.active);
  w.key("speed").number(s.speed);
  w.key("steer").number(s.steer);
  w.key("steer_command").number(s.steer_command);
  w.key("throttle").number(s.throttle);
  w.key("brake").number(s.brake);
  w.key("hand_brake").number(s.hand_brake);
  w.key("gear").integer(s.gear);
  w.key("engine_rpm").number(s.engine_rpm);
  w.key("aero_force").vec(s.aero_force);
  w.key("air_velocity").vec(s.air_velocity);
  {
    const vehicle_energy &e = s.energy;
    w.key("energy").begin_object();
    w.key("supplied").number(e.supplied);
    w.key("idle").number(e.idle);
    w.key("brake").number(e.brake);
    w.key("tire").number(e.tire);
    w.key("aero").number(e.aero);
    w.key("rolling").number(e.rolling);
    w.key("medium").number(e.medium);
    w.key("drivetrain").number(e.drivetrain);
    w.key("drivetrain_split").number(e.drivetrain_split);
    w.key("suspension").number(e.suspension);
    w.key("chassis_damping").number(e.chassis_damping);
    w.key("fields").number(e.fields);
    w.key("stored").number(e.stored);
    w.key("residual").number(e.residual);
    w.key("kinetic").number(e.kinetic);
    w.key("potential").number(e.potential);
    w.key("spring").number(e.spring);
    w.key("engine").number(e.engine);
    w.key("regen").number(e.regen);
    w.key("source").number(e.source);
    w.key("charge").number(e.charge);
    w.key("fuel_used").number(e.fuel_used);
    w.key("fuel_left").number(e.fuel_left);
    w.key("distance").number(e.distance);
    w.key("power").number(e.power);
    w.key("wh_per_km").number(e.wh_per_km());
    w.end_object();
  }
  w.key("wheels").begin_array();
  for (const wheel_state &h : s.wheels) {
    w.begin_object();
    w.key("steer").number(h.steer);
    w.key("angular_velocity").number(h.angular_velocity);
    w.key("rotation").number(h.rotation);
    w.key("suspension_length").number(h.suspension_length);
    w.key("contact").boolean(h.contact);
    w.key("contact_point").vec(h.contact_point);
    w.key("contact_normal").vec(h.contact_normal);
    w.key("contact_body").id(h.contact_body);
    w.key("contact_material").integer(static_cast<std::uint64_t>(h.contact_material));
    w.key("load").number(h.load);
    w.key("longitudinal_slip").number(h.longitudinal_slip);
    w.key("lateral_slip").number(h.lateral_slip);
    w.key("friction").number(h.friction);
    w.key("rolling_resistance").number(h.rolling_resistance);
    w.key("medium_depth").number(h.medium_depth);
    w.key("sinkage").number(h.sinkage);
    w.key("rut").number(h.rut);
    w.key("medium_resistance").number(h.medium_resistance);
    w.end_object();
  }
  w.end_array();
  w.end_object();
  w.end_object();
}

} // namespace

std::string to_json(const world_snapshot &s, json_bodies which) {
  writer w;
  w.out.reserve(512 + 420 * (s.bodies.size() + (s.static_bodies && which != json_bodies::moving
                                                    ? s.static_bodies->size()
                                                    : 0)));
  w.begin_object();
  header(w, s);
  if (which == json_bodies::static_only) {
    bodies(w, s, which);
    w.end_object();
    return std::move(w.out);
  }

  const world_config &c = s.config;
  w.key("config").begin_object();
  w.key("backend").string(c.backend == backend::jolt ? "jolt" : "null");
  w.key("fixed_dt").number(c.fixed_dt);
  w.key("gravity").vec(c.gravity);
  w.key("origin").vec(c.origin);
  w.key("collision_steps").integer(c.collision_steps);
  w.key("velocity_steps").integer(c.velocity_steps);
  w.key("position_steps").integer(c.position_steps);
  w.key("penetration_slop").number(c.penetration_slop);
  w.key("threads").integer(c.threads);
  w.key("determinism")
      .string(c.determinism == determinism::cross_platform ? "cross_platform" : "same_binary");
  w.key("friction_combine").string(combine_name(c.friction));
  w.key("restitution_combine").string(combine_name(c.restitution));
  w.key("contacts").string(contacts_name(c.contacts));
  w.end_object();

  w.key("report").begin_object();
  w.key("step_ms").number(s.report.step_ms);
  w.key("contacts").integer(static_cast<std::uint64_t>(s.report.contacts));
  w.key("active_bodies").integer(s.report.active_bodies);
  w.key("saturated").boolean(s.report.saturated);
  w.end_object();

  w.key("materials").begin_array();
  for (std::size_t i = 0; i < s.materials.size(); ++i) {
    const material &m = s.materials[i];
    w.begin_object();
    w.key("id").integer(static_cast<std::uint64_t>(i));
    w.key("name").string(m.name);
    w.key("friction").number(m.friction);
    w.key("restitution").number(m.restitution);
    w.key("rolling_resistance").number(m.rolling_resistance);
    w.key("nav_material").integer(m.nav_material);
    w.end_object();
  }
  w.end_array();

  bodies(w, s, which);

  w.key("fields").begin_array();
  for (const field_snapshot &f : s.fields) {
    w.begin_object();
    w.key("id").id(f.id);
    w.key("kind").string(kind_name(f.desc.kind));
    w.key("lo").vec(f.lo);
    w.key("hi").vec(f.hi);
    w.key("moving").boolean(f.desc.velocity[0] != nullptr);
    w.key("drag_coefficient").number(f.desc.drag_coefficient);
    w.key("damping").number(f.desc.damping);
    w.key("traction").number(f.desc.traction);
    w.key("ruts").boolean(f.desc.ruts);
    w.key("rut_cell").number(f.desc.rut_cell);
    w.key("rut_cells").integer(static_cast<std::uint64_t>(f.rut_cells));
    w.key("rut_deepest").number(f.rut_deepest);
    w.end_object();
  }
  w.end_array();

  w.key("vehicles").begin_array();
  for (const vehicle_snapshot &v : s.vehicles)
    vehicle(w, v);
  w.end_array();

  w.key("contacts").begin_array();
  for (const contact_event &e : s.contacts) {
    w.begin_object();
    w.key("type").string(contact_type_name(e.type));
    w.key("body_a").id(e.body_a);
    w.key("body_b").id(e.body_b);
    w.key("sub_shape_a").integer(e.sub_shape_a);
    w.key("sub_shape_b").integer(e.sub_shape_b);
    w.key("material_a").integer(static_cast<std::uint64_t>(e.material_a));
    w.key("material_b").integer(static_cast<std::uint64_t>(e.material_b));
    w.key("friction").number(e.friction);
    w.key("restitution").number(e.restitution);
    w.key("point").vec(e.point);
    w.key("normal").vec(e.normal);
    w.key("depth").number(e.depth);
    w.end_object();
  }
  w.end_array();

  w.end_object();
  return std::move(w.out);
}

} // namespace physics
} // namespace cvc
