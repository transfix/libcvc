// NOTE: do NOT run `clang-format -i` on this .i file (see pycvc_nav.i).
//
// pycvc_trace.i — Python surface for the input trace (cvc/core/input_trace.h) and a sim_world's
// trace (cvc/nav/sim_world_trace.h): record a live session's inputs, play them back at their
// ticks, and replay a trace headlessly. docs/INPUT_TRACE.md has the format.
//
//   app = pycvc.App(); clock = app.world_clock()
//   h = pycvc.nav_trace_header(sim, "grid:demo", seed=7, policies={0: ""})
//   rec = pycvc.input_trace_recorder(app, clock)
//   rec.start(h, "session.trace")
//   ... the frame loop: pycvc.state_set(app, "nav.vehicle.vmax", "5") between frames,
//       pycvc.nav_trace_retarget(sim, rec, 2, gx, gy), pycvc.nav_sim_world_pull(sim, app, "")
//       before each step, rec.check(pycvc.nav_trace_state_hash(sim)) after a frame ...
//   rec.finish(pycvc.nav_trace_state_hash(sim))
//   r = pycvc.nav_trace_replay(pycvc.App(), rec.trace())   # r["matched"]
//
// The player has no callbacks here: a Python host loops quantum by quantum (apply_due returns
// the events for the host to apply; next_quantum moves the clock; verify compares a hash).
// After pycvc_nav.i and pycvc_physics.i: the sim_world handles are theirs.

%{
#include <cvc/core/input_trace.h>
#include <cvc/nav/sim_world_trace.h>
#include <map>

namespace pycvc {

// {index: uri} (a str, "" or None for the built-in default) or {index: (uri, sha256)}.
std::map<int, cvc::nav::policy_ref> trace_policy_refs(PyObject *policies) {
  std::map<int, cvc::nav::policy_ref> out;
  if (!policies || policies == Py_None)
    return out;
  if (!PyDict_Check(policies))
    throw std::invalid_argument("policies is a dict {index: uri or (uri, sha256)}");
  PyObject *k = nullptr, *v = nullptr;
  Py_ssize_t pos = 0;
  while (PyDict_Next(policies, &pos, &k, &v)) {
    const long index = PyLong_AsLong(k);
    if (index == -1 && PyErr_Occurred()) {
      PyErr_Clear();
      throw std::invalid_argument("policies: an index is not an int");
    }
    cvc::nav::policy_ref ref;
    auto text = [](PyObject *o, const char *what) {
      if (o == Py_None)
        return std::string();
      if (!PyUnicode_Check(o))
        throw std::invalid_argument(std::string("policies: ") + what + " is not a str");
      return std::string(PyUnicode_AsUTF8(o));
    };
    if (PyTuple_Check(v) && PyTuple_Size(v) == 2) {
      ref.uri = text(PyTuple_GetItem(v, 0), "a uri");
      ref.sha256 = text(PyTuple_GetItem(v, 1), "a sha256");
    } else {
      ref.uri = text(v, "a uri");
    }
    out[static_cast<int>(index)] = ref;
  }
  return out;
}

// {index: path}: a policy file in place of the recorded one.
std::map<int, std::string> trace_policy_files(PyObject *files) {
  std::map<int, std::string> out;
  for (const auto &kv : trace_policy_refs(files))
    out[kv.first] = kv.second.uri;
  return out;
}

} // namespace pycvc
%}

// A record's kind is its name ("set", "event", "check"): kind_name() / set_kind().
%ignore cvc::input_trace_record::type;
%ignore cvc::input_trace_record::kind;
%rename(make_set) cvc::input_trace_record::set;
%rename(make_event) cvc::input_trace_record::event;
%rename(make_check) cvc::input_trace_record::check;
// The header's agents and settings cross as JSON (to_json / from_json).
%ignore cvc::input_trace_header::agents;
%ignore cvc::input_trace_header::settings;
%ignore cvc::input_trace::parse(std::istream &);
%ignore cvc::input_trace::write;
%ignore cvc::input_trace_end::ticks;
%ignore cvc::kInputTraceSchema;
%rename(input_trace_describe) cvc::describe(const input_trace &);
// The recorder and player take the app handle Python holds (and keep it alive), not app&.
%ignore cvc::input_trace_recorder::options;
%ignore cvc::input_trace_recorder::stamping;
%ignore cvc::input_trace_recorder::input_trace_recorder;
%rename("%s") cvc::input_trace_recorder::input_trace_recorder(std::shared_ptr<cvc::app>,
                                                             cvc::world_clock &);
%rename("%s") cvc::input_trace_recorder::input_trace_recorder(
    std::shared_ptr<cvc::app>, cvc::world_clock &, const std::vector<std::string> &,
    const std::vector<std::string> &, bool, bool);
%pythonappend cvc::input_trace_recorder::input_trace_recorder %{
  self._pycvc_app = _keepalive(args[0])
  self._pycvc_clock = args[1]
%}
%ignore cvc::input_trace_player::input_trace_player;
%rename("%s") cvc::input_trace_player::input_trace_player(std::shared_ptr<cvc::app>,
                                                         cvc::world_clock &,
                                                         const cvc::input_trace &);
// One constructor, so SWIG names its arguments (the recorder's two merge into *args).
%pythonappend cvc::input_trace_player::input_trace_player %{
  self._pycvc_app = _keepalive(app)
  self._pycvc_clock = clock
%}
%ignore cvc::input_trace_player::on_event;
%ignore cvc::input_trace_player::set_state_hash;
%ignore cvc::input_trace_player::advance;
%ignore cvc::input_trace_player::run;

%include "cvc/core/input_trace.h"

// apply_due()'s events, and a trace's records.
namespace std {
  %template(input_trace_records) vector<cvc::input_trace_record>;
}

%extend cvc::input_trace_record {
  std::string kind_name() const {
    switch ($self->type) {
    case cvc::input_trace_record::kind::set: return "set";
    case cvc::input_trace_record::kind::event: return "event";
    default: return "check";
    }
  }
}

%extend cvc::input_trace {
  // The schema this build reads and writes ("cvc.input_trace/v1").
  static std::string schema() { return cvc::kInputTraceSchema; }
  std::size_t record_count() const { return $self->records.size(); }
}

%extend cvc::input_trace_recorder {
  input_trace_recorder(std::shared_ptr<cvc::app> app, cvc::world_clock &clock) {
    if (!app)
      throw std::invalid_argument("input_trace_recorder: app is None");
    return new cvc::input_trace_recorder(*app, clock);
  }
  // watch / ignore: the prefixes; skip_telemetry: skip keys whose access is telemetry;
  // on_sample: record changes at sample() instead of as they land.
  input_trace_recorder(std::shared_ptr<cvc::app> app, cvc::world_clock &clock,
                       const std::vector<std::string> &watch,
                       const std::vector<std::string> &ignore, bool skip_telemetry,
                       bool on_sample) {
    if (!app)
      throw std::invalid_argument("input_trace_recorder: app is None");
    cvc::input_trace_recorder::options o;
    o.watch = watch;
    o.ignore = ignore;
    o.skip_telemetry = skip_telemetry;
    o.stamp = on_sample ? cvc::input_trace_recorder::stamping::on_sample
                        : cvc::input_trace_recorder::stamping::on_write;
    return new cvc::input_trace_recorder(*app, clock, o);
  }
}

%extend cvc::input_trace_player {
  input_trace_player(std::shared_ptr<cvc::app> app, cvc::world_clock &clock,
                     const cvc::input_trace &trace) {
    if (!app)
      throw std::invalid_argument("input_trace_player: app is None");
    return new cvc::input_trace_player(*app, clock, trace);
  }
}

%feature("kwargs") pycvc::nav_trace_header;
%feature("kwargs") pycvc::nav_trace_replay;
%feature("kwargs") pycvc::nav_sim_world_from_trace;

%inline %{
namespace pycvc {

// The trace header for a world before its first step (cvc::nav::trace_header).
cvc::input_trace_header nav_trace_header(PyObject *handle, const std::string &world_id,
                                         unsigned long long seed = 0,
                                         PyObject *policies = nullptr,
                                         const std::string &prefix = "",
                                         bool pull_every_step = true, int num_threads = 1,
                                         unsigned long long phase = 0, bool embed_maps = true,
                                         bool allow_incomplete = false,
                                         const std::string &meta = "{}") {
  cvc::nav::trace_options o;
  o.world_id = world_id;
  o.seed = seed;
  o.policies = trace_policy_refs(policies);
  o.prefix = prefix;
  o.pull_every_step = pull_every_step;
  o.num_threads = num_threads;
  o.phase = phase;
  o.embed_maps = embed_maps;
  o.allow_incomplete = allow_incomplete;
  o.meta = meta;
  return cvc::nav::trace_header(*sim_world_from(handle), o);
}

std::string nav_trace_state_hash(PyObject *handle) {
  return cvc::nav::state_hash(*sim_world_from(handle));
}

// nav's verbs as trace events: applied to the world, and recorded when `rec` is not None.
bool nav_trace_retarget(PyObject *handle, cvc::input_trace_recorder *rec, unsigned long long id,
                        double gx_n, double gy_n) {
  return cvc::nav::trace_retarget(*sim_world_from(handle), rec, id, static_cast<float>(gx_n),
                                  static_cast<float>(gy_n));
}
bool nav_trace_set_pose(PyObject *handle, cvc::input_trace_recorder *rec, unsigned long long id,
                        double x_n, double y_n, double heading, double speed = 0.0) {
  return cvc::nav::trace_set_pose(*sim_world_from(handle), rec, id, static_cast<float>(x_n),
                                  static_cast<float>(y_n), static_cast<float>(heading),
                                  static_cast<float>(speed));
}
bool nav_trace_override(PyObject *handle, cvc::input_trace_recorder *rec, unsigned long long id,
                        double steer, double accel) {
  return cvc::nav::trace_override(*sim_world_from(handle), rec, id, static_cast<float>(steer),
                                  static_cast<float>(accel));
}
void nav_trace_add_obstacle(PyObject *handle, cvc::input_trace_recorder *rec, int r0, int r1,
                            int c0, int c1) {
  cvc::nav::trace_add_obstacle(*sim_world_from(handle), rec, r0, r1, c0, c1);
}
// Apply a played-back event to a world; False for one that is not nav's.
bool nav_trace_apply_event(PyObject *handle, const cvc::input_trace_record &e) {
  return cvc::nav::apply_trace_event(*sim_world_from(handle), e);
}

// The world a header describes, as a sim_world handle (cvc::nav::world_from_trace).
PyObject *nav_sim_world_from_trace(const cvc::input_trace_header &header,
                                   PyObject *policy_files = nullptr) {
  cvc::nav::policy_library lib;
  auto *sw = new cvc::nav::sim_world(
      cvc::nav::world_from_trace(header, lib, trace_policy_files(policy_files)));
  return PyCapsule_New(sw, kSimWorldCapsule, sim_world_capsule_dtor);
}

// Replay a trace headlessly on `app` (a fresh one), as `cvc trace verify` does. Returns a dict:
// matched, has_end, ticks, steps, hash, expected, compared, agreed, diverged, first_divergence,
// notes, report (verify's text) and, with keep_poses, poses ((steps, n, 4) float32: x, y,
// heading, speed, normalized) and pose_ticks.
PyObject *nav_trace_replay(std::shared_ptr<cvc::app> app, const cvc::input_trace &trace,
                           int num_threads = 0, PyObject *policy_files = nullptr,
                           bool keep_poses = false) {
  if (!app)
    throw std::invalid_argument("nav_trace_replay: app is None");
  cvc::nav::replay_options o;
  o.num_threads = num_threads;
  o.policy_files = trace_policy_files(policy_files);
  o.keep_poses = keep_poses;
  cvc::nav::replay_result r;
  std::string err;
  Py_BEGIN_ALLOW_THREADS
  try {
    r = cvc::nav::replay_trace(*app, trace, o);
  } catch (const std::exception &e) {
    err = e.what();
    if (err.empty())
      err = "nav_trace_replay failed";
  }
  Py_END_ALLOW_THREADS
  if (!err.empty())
    throw std::runtime_error(err);
  PyObject *d = PyDict_New();
  auto put = [&](const char *k, PyObject *v) {
    PyDict_SetItemString(d, k, v);
    Py_DECREF(v);
  };
  put("matched", PyBool_FromLong(r.matched));
  put("has_end", PyBool_FromLong(r.has_end));
  put("ticks", PyLong_FromUnsignedLongLong(r.ticks));
  put("steps", PyLong_FromLong(r.steps));
  put("hash", PyUnicode_FromString(r.hash.c_str()));
  put("expected", PyUnicode_FromString(r.expected.c_str()));
  put("compared", PyLong_FromSize_t(r.compared));
  put("agreed", PyLong_FromSize_t(r.agreed));
  put("diverged", PyBool_FromLong(r.diverged));
  put("first_divergence", PyLong_FromUnsignedLongLong(r.first_divergence));
  PyObject *notes = PyList_New(static_cast<Py_ssize_t>(r.notes.size()));
  for (std::size_t k = 0; k < r.notes.size(); ++k)
    PyList_SET_ITEM(notes, static_cast<Py_ssize_t>(k), PyUnicode_FromString(r.notes[k].c_str()));
  put("notes", notes);
  put("report", PyUnicode_FromString(cvc::nav::describe(r).c_str()));
  if (keep_poses) {
    const npy_intp steps = static_cast<npy_intp>(r.pose_ticks.size());
    const npy_intp n = steps ? static_cast<npy_intp>(r.poses.size()) / (steps * 4) : 0;
    npy_intp dims[3] = {steps, n, 4};
    PyObject *poses = PyArray_SimpleNew(3, dims, NPY_FLOAT);
    if (!poses) {
      Py_DECREF(d);
      throw std::runtime_error("nav_trace_replay: alloc failed");
    }
    if (!r.poses.empty())
      std::memcpy(PyArray_DATA(reinterpret_cast<PyArrayObject *>(poses)), r.poses.data(),
                  r.poses.size() * sizeof(float));
    put("poses", poses);
    PyObject *ticks = PyList_New(steps);
    for (npy_intp k = 0; k < steps; ++k)
      PyList_SET_ITEM(ticks, k, PyLong_FromUnsignedLongLong(r.pose_ticks[static_cast<std::size_t>(k)]));
    put("pose_ticks", ticks);
  }
  return d;
}

} // namespace pycvc
%}
