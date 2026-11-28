// pycvc_ph.i — SWIG surface for cvc::ph composition (cvc/ph/compose.h): composed port-Hamiltonian
// systems (C-PHAST, by Shubham Bhardwaj and Chandrajit Bajaj). NOT a standalone module: %include'd
// by pycvc.i.
//
// One handle, ph_composite, built from a "cvc-ph-system/1" file path or its JSON text. States and
// inputs cross as sequences of floats (lists, tuples, numpy arrays); results come back as lists,
// numpy arrays and dicts. The handle owns its composite; laws inside it are shared and immutable.
// The logic is the library's: these methods only marshal.
//
//   g = pycvc.ph_composite("grid.json")
//   r = g.simulate(x0, u, steps=200, dt=0.01, coupling="midpoint")
//   r["x"]          # (steps + 1, state_dim) float64
//   r["residual"]   # per-step energy-balance residual (dH - supplied + dissipated)
//   g.check(100)    # the power balance at random states: {"rate_error", "internal_error", ...}

%{
#include <algorithm>
#include <cvc/ph/compose.h>
#include <cvc/ph/mechanical.h>
#include <cvc/ph/microgrid.h>
#include <cvc/ph/query.h>
#include <cvc/ph/train.h>

namespace pycvc {

std::vector<double> ph_floats(PyObject *o, const char *what) {
  std::vector<double> out;
  if (!o || o == Py_None)
    return out;
  PyObject *seq = PySequence_Fast(o, what);
  if (!seq)
    throw std::invalid_argument(std::string(what) + " is not a sequence of floats");
  const Py_ssize_t n = PySequence_Fast_GET_SIZE(seq);
  out.reserve(static_cast<size_t>(n));
  for (Py_ssize_t i = 0; i < n; ++i) {
    const double v = PyFloat_AsDouble(PySequence_Fast_GET_ITEM(seq, i));
    if (v == -1.0 && PyErr_Occurred()) {
      PyErr_Clear();
      Py_DECREF(seq);
      throw std::invalid_argument(std::string(what) + "[" + std::to_string(i) + "] is not a float");
    }
    out.push_back(v);
  }
  Py_DECREF(seq);
  return out;
}

std::vector<std::vector<double>> ph_rows(PyObject *o, const char *what) {
  std::vector<std::vector<double>> out;
  if (!o || o == Py_None)
    return out;
  PyObject *seq = PySequence_Fast(o, what);
  if (!seq)
    throw std::invalid_argument(std::string(what) + " is not a sequence of rows");
  for (Py_ssize_t i = 0; i < PySequence_Fast_GET_SIZE(seq); ++i)
    out.push_back(ph_floats(PySequence_Fast_GET_ITEM(seq, i), what));
  Py_DECREF(seq);
  return out;
}

std::vector<int> ph_ints(PyObject *o, const char *what) {
  std::vector<int> out;
  for (double v : ph_floats(o, what))
    out.push_back(static_cast<int>(v));
  return out;
}

cvc::ph::summary ph_summary(const std::string &k) {
  if (k == "max_abs")
    return cvc::ph::summary::max_abs;
  if (k == "min")
    return cvc::ph::summary::min;
  if (k == "max")
    return cvc::ph::summary::max;
  if (k == "final")
    return cvc::ph::summary::final_value;
  if (k == "total_loss")
    return cvc::ph::summary::total_loss;
  if (k == "total_work")
    return cvc::ph::summary::total_work;
  if (k == "max_residual")
    return cvc::ph::summary::max_residual;
  throw std::invalid_argument("unknown summary '" + k +
                              "' (max_abs, min, max, final, total_loss, total_work, max_residual)");
}

std::string ph_str(PyObject *o, const char *what) {
  if (!o || !PyUnicode_Check(o))
    throw std::invalid_argument(std::string(what) + " is not a str");
  return PyUnicode_AsUTF8(o);
}

PyObject *ph_list(const double *v, size_t n) {
  PyObject *l = PyList_New(static_cast<Py_ssize_t>(n));
  for (size_t i = 0; i < n; ++i)
    PyList_SET_ITEM(l, static_cast<Py_ssize_t>(i), PyFloat_FromDouble(v[i]));
  return l;
}

} // namespace pycvc
%}

// Keyword arguments for the methods with defaults: g.simulate(x0, u, steps=200, coupling="euler").
%feature("kwargs") pycvc::ph_composite::simulate;
%feature("kwargs") pycvc::ph_composite::check;
%feature("kwargs") pycvc::ph_composite::field;
%feature("kwargs") pycvc::ph_composite::powers;
%feature("kwargs") pycvc::ph_composite::storage;
%feature("kwargs") pycvc::ph_microgrid_json;
%feature("kwargs") pycvc::ph_composite::fit_history;
%feature("kwargs") pycvc::ph_composite::card;
%feature("kwargs") pycvc::ph_train_dgu;
%feature("kwargs") pycvc::ph_arm_json;
%feature("kwargs") pycvc::ph_arm_study;
%feature("kwargs") pycvc::ph_go2_leg_json;

%inline %{
namespace pycvc {

class ph_composite {
public:
  // A "cvc-ph-system/1" file path, or its JSON text (anything starting with '{').
  explicit ph_composite(const std::string &path_or_json) {
    const size_t at = path_or_json.find_first_not_of(" \t\r\n");
    s_ = at != std::string::npos && path_or_json[at] == '{' ? cvc::ph::parse_system(path_or_json)
                                                             : cvc::ph::load_system(path_or_json);
  }

  int blocks() const { return s_.blocks(); }
  int state_dim() const { return s_.state_dim(); }
  int input_dim() const { return s_.input_dim(); }
  int port_dim() const { return s_.port_dim(); }
  std::string describe() const { return cvc::ph::describe(s_); }

  // Block i's name, law kind and (state, port, input) offsets.
  PyObject *block(int i) const {
    if (i < 0 || i >= s_.blocks())
      throw std::out_of_range("ph_composite.block: no block " + std::to_string(i));
    const cvc::ph::block_law &L = s_.law(i);
    return Py_BuildValue("{s:s,s:s,s:i,s:i,s:i,s:i,s:i,s:i}", "name", s_.name(i).c_str(), "kind",
                         L.kind().c_str(), "state_dim", L.state_dim(), "port_dim", L.port_dim(),
                         "input_dim", L.input_dim(), "state_offset", s_.state_offset(i), "port_offset",
                         s_.port_offset(i), "input_offset", s_.input_offset(i));
  }

  // The assembled skew pairing C (port_dim x port_dim, row-major).
  PyObject *pairing() const { return ph_list(s_.pairing().data(), s_.pairing().size()); }

  double energy(PyObject *x) const { return s_.energy(state(x).data()); }

  PyObject *effort(PyObject *x) const {
    const std::vector<double> xs = state(x);
    std::vector<double> e(xs.size());
    s_.effort(xs.data(), e.data());
    return ph_list(e.data(), e.size());
  }

  PyObject *field(PyObject *x, PyObject *u = nullptr) const {
    const std::vector<double> xs = state(x), us = inputs(u);
    std::vector<double> f(xs.size());
    s_.field(xs.data(), us.data(), f.data());
    return ph_list(f.data(), f.size());
  }

  // {"stored", "loss", "external", "internal"} at x, u.
  PyObject *powers(PyObject *x, PyObject *u = nullptr) const {
    const cvc::ph::power_terms p = s_.powers(state(x).data(), inputs(u).data());
    return Py_BuildValue("{s:d,s:d,s:d,s:d}", "stored", p.stored, "loss", p.loss, "external", p.external,
                         "internal", p.internal);
  }

  // A rollout of `steps` steps from x0 under u (one row held, or one row per step). Returns
  // {"x": (steps + 1, state_dim) array, "H", "dH", "supplied", "dissipated", "residual" (per step),
  // "iterations", "converged", "report"}.
  PyObject *simulate(PyObject *x0, PyObject *u = nullptr, int steps = 100, double dt = 0.01,
                     const std::string &scheme = "split", const std::string &coupling = "midpoint",
                     int substeps = 2, int local_substeps = 1, int max_iter = 10, double tol = 1e-6,
                     const std::string &order = "coupling_outside") const {
    cvc::ph::transition_opts o;
    o.dt = dt;
    o.scheme = cvc::ph::step_scheme_of(scheme);
    o.coupling = cvc::ph::coupling_scheme_of(coupling);
    o.order = cvc::ph::split_order_of(order);
    o.substeps = substeps;
    o.local_substeps = local_substeps;
    o.solve.max_iter = max_iter;
    o.solve.tol = tol;
    std::vector<double> us = ph_floats(u, "u");
    if (us.empty())
      us.assign(s_.input_dim(), 0.0);
    const std::vector<double> xs = ph_floats(x0, "x0");
    cvc::ph::rollout_result r;
    std::string err;
    Py_BEGIN_ALLOW_THREADS
    try {
      r = cvc::ph::rollout(s_, o, xs, us, steps);
    } catch (const std::exception &e) {
      err = e.what();
      if (err.empty())
        err = "ph_composite.simulate failed";
    }
    Py_END_ALLOW_THREADS
    if (!err.empty())
      throw std::runtime_error(err);
    const npy_intp rows = static_cast<npy_intp>(r.x.size()), cols = s_.state_dim();
    npy_intp dims[2] = {rows, cols};
    PyObject *X = PyArray_SimpleNew(2, dims, NPY_DOUBLE);
    if (!X)
      throw std::runtime_error("ph_composite.simulate: alloc failed");
    double *dst = static_cast<double *>(PyArray_DATA(reinterpret_cast<PyArrayObject *>(X)));
    for (npy_intp k = 0; k < rows; ++k)
      std::copy(r.x[k].begin(), r.x[k].end(), dst + k * cols);
    std::vector<double> H(r.x.size()), dH, sup, dis, res;
    for (size_t k = 0; k < r.x.size(); ++k)
      H[k] = s_.energy(r.x[k].data());
    PyObject *its = PyList_New(static_cast<Py_ssize_t>(r.stats.size()));
    bool converged = true;
    for (size_t k = 0; k < r.ledger.size(); ++k) {
      dH.push_back(r.ledger[k].dH);
      sup.push_back(r.ledger[k].supplied);
      dis.push_back(r.ledger[k].dissipated);
      res.push_back(r.ledger[k].residual);
      PyList_SET_ITEM(its, static_cast<Py_ssize_t>(k), PyLong_FromLong(r.stats[k].iterations));
      converged = converged && r.stats[k].converged;
    }
    PyObject *d = PyDict_New();
    auto put = [&](const char *k, PyObject *v) {
      PyDict_SetItemString(d, k, v);
      Py_DECREF(v);
    };
    put("x", X);
    put("H", ph_list(H.data(), H.size()));
    put("dH", ph_list(dH.data(), dH.size()));
    put("supplied", ph_list(sup.data(), sup.size()));
    put("dissipated", ph_list(dis.data(), dis.size()));
    put("residual", ph_list(res.data(), res.size()));
    put("iterations", its);
    put("converged", PyBool_FromLong(converged));
    put("report", PyUnicode_FromString(cvc::ph::describe(s_, o, r).c_str()));
    return d;
  }

  // The state whose efforts are e (storage_from_efforts): {"x", "converged", "iterations",
  // "residual"}. For a microgrid, e is each DGU's voltage and current, then each line's current.
  PyObject *storage(PyObject *e, PyObject *x0 = nullptr, int max_iter = 30, double tol = 1e-9) const {
    const cvc::ph::storage_result r =
        cvc::ph::storage_from_efforts(s_, state(e), ph_floats(x0, "x0"), max_iter, tol);
    PyObject *x = ph_list(r.x.data(), r.x.size());
    PyObject *d = Py_BuildValue("{s:O,s:O,s:i,s:d}", "x", x, "converged", r.converged ? Py_True : Py_False,
                                "iterations", r.iterations, "residual", r.residual);
    Py_DECREF(x);
    return d;
  }

  // fit_history (query.h): the state at the query time from measured channels over a window.
  // inputs: one row per step; sample_steps, channels (readout indices) and samples (one row per
  // sample step). Returns {"x_start", "x_now", "xi", "success", "optimality", "rank", "evals",
  // "cost", "admitted"}.
  PyObject *fit_history(PyObject *inputs, PyObject *sample_steps, PyObject *channels, PyObject *samples,
                        PyObject *prior_mean, PyObject *prior_scale, double noise = 0.0, double dt = 0.01,
                        const std::string &coupling = "midpoint", int substeps = 2, int max_evals = 20) const {
    cvc::ph::history h;
    h.inputs = ph_rows(inputs, "inputs");
    h.sample_steps = ph_ints(sample_steps, "sample_steps");
    h.channels = ph_ints(channels, "channels");
    h.samples = ph_rows(samples, "samples");
    cvc::ph::history_fit_options o;
    o.step.dt = dt;
    o.step.coupling = cvc::ph::coupling_scheme_of(coupling);
    o.step.substeps = substeps;
    o.prior_mean = ph_floats(prior_mean, "prior_mean");
    o.prior_scale = ph_floats(prior_scale, "prior_scale");
    o.noise = noise;
    o.max_evals = max_evals;
    cvc::ph::history_fit_result f;
    std::string err;
    Py_BEGIN_ALLOW_THREADS
    try {
      f = cvc::ph::fit_history(s_, h, o);
    } catch (const std::exception &e) {
      err = e.what();
      if (err.empty())
        err = "ph_composite.fit_history failed";
    }
    Py_END_ALLOW_THREADS
    if (!err.empty())
      throw std::runtime_error(err);
    PyObject *xs = ph_list(f.x_start.data(), f.x_start.size()), *xn = ph_list(f.x_now.data(), f.x_now.size()),
             *xi = ph_list(f.xi.data(), f.xi.size());
    PyObject *d = Py_BuildValue("{s:O,s:O,s:O,s:O,s:d,s:i,s:i,s:d,s:O}", "x_start", xs, "x_now", xn, "xi", xi,
                                "success", f.success ? Py_True : Py_False, "optimality", f.optimality, "rank",
                                f.rank, "evals", f.evals, "cost", f.cost, "admitted",
                                f.admitted(s_.state_dim()) ? Py_True : Py_False);
    Py_DECREF(xs);
    Py_DECREF(xn);
    Py_DECREF(xi);
    return d;
  }

  // A candidate's action card (make_card): roll out `steps` from x0 under u and summarize.
  // entries: a list of (name, kind, channels), kind one of max_abs, min, max, final, total_loss,
  // total_work, max_residual. Returns {name: value}.
  PyObject *card(PyObject *x0, PyObject *u, int steps, PyObject *entries, double dt = 0.01,
                 const std::string &coupling = "midpoint", int substeps = 2) const {
    std::vector<cvc::ph::card_entry> spec;
    PyObject *seq = PySequence_Fast(entries, "entries");
    if (!seq)
      throw std::invalid_argument("entries is a list of (name, kind, channels)");
    for (Py_ssize_t i = 0; i < PySequence_Fast_GET_SIZE(seq); ++i) {
      PyObject *e = PySequence_Fast_GET_ITEM(seq, i);
      if (!PyTuple_Check(e) && !PyList_Check(e)) {
        Py_DECREF(seq);
        throw std::invalid_argument("each entry is (name, kind, channels)");
      }
      PyObject *t = PySequence_Fast(e, "entry");
      const Py_ssize_t n = PySequence_Fast_GET_SIZE(t);
      cvc::ph::card_entry ce;
      try {
        ce.name = ph_str(n > 0 ? PySequence_Fast_GET_ITEM(t, 0) : nullptr, "an entry's name");
        ce.kind = ph_summary(ph_str(n > 1 ? PySequence_Fast_GET_ITEM(t, 1) : nullptr, "an entry's kind"));
        if (n > 2)
          ce.channels = ph_ints(PySequence_Fast_GET_ITEM(t, 2), "an entry's channels");
      } catch (...) {
        Py_DECREF(t);
        Py_DECREF(seq);
        throw;
      }
      Py_DECREF(t);
      spec.push_back(ce);
    }
    Py_DECREF(seq);
    cvc::ph::transition_opts o;
    o.dt = dt;
    o.coupling = cvc::ph::coupling_scheme_of(coupling);
    o.substeps = substeps;
    std::vector<double> us = ph_floats(u, "u");
    if (us.empty())
      us.assign(s_.input_dim(), 0.0);
    const cvc::ph::rollout_result r = cvc::ph::rollout(s_, o, state(x0), us, steps);
    const cvc::ph::action_card c = cvc::ph::make_card("", cvc::ph::consequences(s_, r, us), spec);
    PyObject *d = PyDict_New();
    for (size_t i = 0; i < c.names.size(); ++i) {
      PyObject *v = PyFloat_FromDouble(c.values[i]);
      PyDict_SetItemString(d, c.names[i].c_str(), v);
      Py_DECREF(v);
    }
    return d;
  }

  // The system as cvc-ph-system/1 text (system_json).
  std::string system_json() const { return cvc::ph::system_json(s_); }

  // The power balance at random states (check_balance_random): the worst relative errors.
  PyObject *check(int samples = 100, unsigned seed = 1, double scale = 1.0) const {
    const cvc::ph::balance_summary b = cvc::ph::check_balance_random(s_, samples, seed, scale);
    return Py_BuildValue("{s:i,s:d,s:d,s:d}", "samples", b.samples, "rate_error", b.worst.rate_error,
                         "internal_error", b.worst.internal_error, "skew_error", b.worst.skew_error);
  }

private:
  cvc::ph::composite s_;

  std::vector<double> state(PyObject *x) const {
    std::vector<double> v = ph_floats(x, "x");
    if (static_cast<int>(v.size()) != s_.state_dim())
      throw std::invalid_argument("x has " + std::to_string(v.size()) + " entries, the state " +
                                  std::to_string(s_.state_dim()));
    return v;
  }
  std::vector<double> inputs(PyObject *u) const {
    std::vector<double> v = ph_floats(u, "u");
    if (v.empty())
      v.assign(s_.input_dim(), 0.0);
    if (static_cast<int>(v.size()) != s_.input_dim())
      throw std::invalid_argument("u has " + std::to_string(v.size()) + " entries, the inputs " +
                                  std::to_string(s_.input_dim()));
    return v;
  }
};

// A DC microgrid (make_microgrid) as cvc-ph-system/1 text, for ph_composite: `dgus` buses, lines
// as (from, to) pairs (a chain when None), resistances and inductances (one value or one per line;
// the last repeats), and the DGU law: "reference" (q = 2.2 (v + v^3/3), 1.8 H, 1.2 ohm) or
// "linear" (C, L and R from dgu_c, dgu_l, dgu_r).
std::string ph_microgrid_json(int dgus, PyObject *edges = nullptr, PyObject *line_r = nullptr,
                              PyObject *line_l = nullptr, const std::string &dgu = "reference",
                              double dgu_c = 2.2, double dgu_l = 1.8, double dgu_r = 1.2) {
  std::vector<cvc::ph::line_spec> lines;
  if (!edges || edges == Py_None) {
    for (int i = 0; i + 1 < dgus; ++i)
      lines.push_back({i, i + 1, 0.0, 1.0});
  } else {
    PyObject *seq = PySequence_Fast(edges, "edges");
    if (!seq)
      throw std::invalid_argument("edges is a sequence of (from, to) pairs");
    for (Py_ssize_t k = 0; k < PySequence_Fast_GET_SIZE(seq); ++k) {
      const std::vector<double> p = ph_floats(PySequence_Fast_GET_ITEM(seq, k), "an edge");
      if (p.size() != 2) {
        Py_DECREF(seq);
        throw std::invalid_argument("each edge is a (from, to) pair");
      }
      lines.push_back({int(p[0]), int(p[1]), 0.0, 1.0});
    }
    Py_DECREF(seq);
  }
  std::vector<double> rs = ph_floats(line_r, "line_r"), ls = ph_floats(line_l, "line_l");
  if (rs.empty())
    rs = {0.3, 0.4, 0.5};
  if (ls.empty())
    ls = {0.8};
  for (size_t k = 0; k < lines.size(); ++k) {
    lines[k].R = rs[std::min(k, rs.size() - 1)];
    lines[k].L = ls[std::min(k, ls.size() - 1)];
  }
  cvc::ph::law_ptr law;
  if (dgu == "reference")
    law = cvc::ph::reference_dgu();
  else if (dgu == "linear")
    law = cvc::ph::linear_dgu(dgu_c, dgu_l, dgu_r);
  else
    throw std::invalid_argument("dgu is \"reference\" or \"linear\", not " + dgu);
  return cvc::ph::system_json(cvc::ph::make_microgrid(law, dgus, lines).system);
}

// rank_candidates (query.h) over cards given as dicts: cards and bounds are lists of {entry: value}
// (bounds: each entry's error bound, 0 when absent), limits a list of (entry, bound) for upper
// limits or (entry, bound, "lower"), and objective {entry: weight}. Returns {"feasible" (admitted,
// rejected or unresolved per card), "score", "score_bound", "order", "best", "outcome"
// (certified, inconclusive or abstain), "suboptimality"}.
PyObject *ph_rank(PyObject *cards, PyObject *bounds, PyObject *limits, PyObject *objective) {
  if (!PySequence_Check(cards) || !PySequence_Check(bounds) || !PyDict_Check(objective))
    throw std::invalid_argument("ph_rank(cards: list of dicts, bounds: list of dicts, limits, objective: dict)");
  const Py_ssize_t n = PySequence_Size(cards);
  if (PySequence_Size(bounds) != n)
    throw std::invalid_argument("ph_rank: one bounds dict per card");
  std::vector<cvc::ph::action_card> cs;
  std::vector<cvc::ph::card_bounds> bs;
  for (Py_ssize_t a = 0; a < n; ++a) {
    PyObject *c = PySequence_GetItem(cards, a), *b = PySequence_GetItem(bounds, a);
    if (!c || !b || !PyDict_Check(c) || !PyDict_Check(b)) {
      Py_XDECREF(c);
      Py_XDECREF(b);
      throw std::invalid_argument("ph_rank: every card and bound is a dict {entry: value}");
    }
    cvc::ph::action_card card;
    cvc::ph::card_bounds cb;
    PyObject *k = nullptr, *v = nullptr;
    Py_ssize_t pos = 0;
    while (PyDict_Next(c, &pos, &k, &v)) {
      card.names.push_back(ph_str(k, "a card entry"));
      card.values.push_back(PyFloat_AsDouble(v));
      PyObject *bv = PyDict_GetItem(b, k);
      cb.delta.push_back(bv ? PyFloat_AsDouble(bv) : 0.0);
    }
    Py_DECREF(c);
    Py_DECREF(b);
    if (PyErr_Occurred()) {
      PyErr_Clear();
      throw std::invalid_argument("ph_rank: card values and bounds are floats");
    }
    card.candidate = std::to_string(a);
    cs.push_back(card);
    bs.push_back(cb);
  }
  std::vector<cvc::ph::card_limit> ls;
  if (limits && limits != Py_None) {
    PyObject *seq = PySequence_Fast(limits, "limits");
    if (!seq)
      throw std::invalid_argument("limits is a list of (entry, bound[, \"lower\"])");
    for (Py_ssize_t i = 0; i < PySequence_Fast_GET_SIZE(seq); ++i) {
      PyObject *t = PySequence_Fast(PySequence_Fast_GET_ITEM(seq, i), "a limit");
      const Py_ssize_t m = t ? PySequence_Fast_GET_SIZE(t) : 0;
      cvc::ph::card_limit l;
      try {
        if (m < 2)
          throw std::invalid_argument("a limit is (entry, bound[, \"lower\"])");
        l.entry = ph_str(PySequence_Fast_GET_ITEM(t, 0), "a limit's entry");
        l.bound = PyFloat_AsDouble(PySequence_Fast_GET_ITEM(t, 1));
        if (m > 2)
          l.upper = ph_str(PySequence_Fast_GET_ITEM(t, 2), "a limit's side") != "lower";
      } catch (...) {
        Py_XDECREF(t);
        Py_DECREF(seq);
        throw;
      }
      Py_DECREF(t);
      ls.push_back(l);
    }
    Py_DECREF(seq);
  }
  cvc::ph::card_objective obj;
  PyObject *k = nullptr, *v = nullptr;
  Py_ssize_t pos = 0;
  while (PyDict_Next(objective, &pos, &k, &v)) {
    obj.entries.push_back(ph_str(k, "an objective entry"));
    obj.weights.push_back(PyFloat_AsDouble(v));
  }
  const cvc::ph::ranking r = cvc::ph::rank_candidates(cs, bs, ls, obj);
  PyObject *feas = PyList_New(n), *order = PyList_New(n);
  for (Py_ssize_t a = 0; a < n; ++a) {
    PyList_SET_ITEM(feas, a, PyUnicode_FromString(cvc::ph::to_string(r.feasible[a])));
    PyObject *row = PyList_New(n);
    for (Py_ssize_t b = 0; b < n; ++b)
      PyList_SET_ITEM(row, b, PyLong_FromLong(r.order[a][b]));
    PyList_SET_ITEM(order, a, row);
  }
  PyObject *score = ph_list(r.score.data(), r.score.size()), *sb = ph_list(r.score_bound.data(), r.score_bound.size());
  PyObject *best = r.best < 0 ? (Py_INCREF(Py_None), Py_None) : PyLong_FromLong(r.best);
  PyObject *d = Py_BuildValue("{s:O,s:O,s:O,s:O,s:O,s:s,s:d}", "feasible", feas, "score", score, "score_bound", sb,
                              "order", order, "best", best, "outcome", cvc::ph::to_string(r.outcome),
                              "suboptimality", r.suboptimality);
  Py_DECREF(feas);
  Py_DECREF(order);
  Py_DECREF(score);
  Py_DECREF(sb);
  Py_DECREF(best);
  return d;
}

// The isolated-DGU calibration (train.h), as `cvc ph train-dgu` runs it: fit a DGU law to
// trajectories of the reference nonlinear DGU, score it and the fitted-linear control on held-out
// rollouts. Returns {"law" (cvc-ph-law/1 text), "parameters", "linear" and "trained"
// ({"voltage_rmse", "current_rmse", "finite"}), "fit" ({"C", "L", "R"}), "history" (a list of
// {"update", "step", "constraint", "total"})}.
PyObject *ph_train_dgu(int updates = 2000, int constraint_first = 0, int batch = 128, double lr = 1e-4,
                       int threads = 1, int train_runs = 32, int transitions = 200, int test_runs = 16,
                       int test_transitions = 500, unsigned data_seed = 1, unsigned init_seed = 7,
                       PyObject *v_hidden = nullptr, PyObject *m_hidden = nullptr, PyObject *d_hidden = nullptr,
                       int log_every = 0) {
  const cvc::ph::law_ptr ref = cvc::ph::reference_dgu();
  cvc::ph::dgu_dataset_options d;
  d.trajectories = train_runs;
  d.transitions = transitions;
  d.seed = data_seed;
  cvc::ph::dgu_family fam;
  if (v_hidden && v_hidden != Py_None)
    fam.v_hidden = ph_ints(v_hidden, "v_hidden");
  if (m_hidden && m_hidden != Py_None)
    fam.m_hidden = ph_ints(m_hidden, "m_hidden");
  if (d_hidden && d_hidden != Py_None)
    fam.d_hidden = ph_ints(d_hidden, "d_hidden");
  fam.seed = init_seed;
  cvc::ph::dgu_train_options o;
  o.updates = updates;
  o.constraint_first = constraint_first;
  o.batch = batch;
  o.lr = lr;
  o.threads = threads;
  o.log_every = log_every;
  o.seed = init_seed + 1000;
  cvc::ph::linear_fit fit;
  cvc::ph::dgu_train_result r;
  cvc::ph::dgu_eval el, et;
  std::string err;
  Py_BEGIN_ALLOW_THREADS
  try {
    const cvc::ph::dgu_dataset train = cvc::ph::make_dgu_dataset(*ref, d);
    d.trajectories = test_runs;
    d.transitions = test_transitions;
    d.seed = data_seed + 1;
    const cvc::ph::dgu_dataset test = cvc::ph::make_dgu_dataset(*ref, d);
    fit = cvc::ph::fit_linear_dgu(train);
    r = cvc::ph::train_dgu(cvc::ph::warm_start_dgu(fit, fam), train, o);
    el = cvc::ph::evaluate_dgu(cvc::ph::canonical_law(cvc::ph::linear_dgu_spec(fit)), *ref, test);
    et = cvc::ph::evaluate_dgu(cvc::ph::canonical_law(r.law), *ref, test);
  } catch (const std::exception &e) {
    err = e.what();
    if (err.empty())
      err = "ph_train_dgu failed";
  }
  Py_END_ALLOW_THREADS
  if (!err.empty())
    throw std::runtime_error(err);
  auto score = [](const cvc::ph::dgu_eval &e) {
    return Py_BuildValue("{s:d,s:d,s:O}", "voltage_rmse", e.voltage_rmse, "current_rmse", e.current_rmse, "finite",
                         e.finite ? Py_True : Py_False);
  };
  PyObject *hist = PyList_New(static_cast<Py_ssize_t>(r.history.size()));
  for (size_t k = 0; k < r.history.size(); ++k)
    PyList_SET_ITEM(hist, static_cast<Py_ssize_t>(k),
                    Py_BuildValue("{s:i,s:d,s:d,s:d}", "update", r.history[k].update, "step", r.history[k].batch.step,
                                  "constraint", r.history[k].batch.constraint, "total", r.history[k].batch.total));
  PyObject *lin = score(el), *tr = score(et);
  PyObject *fd = Py_BuildValue("{s:d,s:d,s:d}", "C", fit.C, "L", fit.L, "R", fit.R);
  PyObject *out = Py_BuildValue("{s:s,s:i,s:O,s:O,s:O,s:O}", "law",
                                cvc::ph::law_json(cvc::ph::canonical_law(r.law)).c_str(), "parameters", r.parameters,
                                "linear", lin, "trained", tr, "fit", fd, "history", hist);
  Py_DECREF(lin);
  Py_DECREF(tr);
  Py_DECREF(fd);
  Py_DECREF(hist);
  return out;
}

// The planar arm of the count-transfer study (mechanical.h): n known joints, their port 0 paired
// by the skew `coupling` (n x n rows, or None for none), as cvc-ph-system/1 text. Simulate it with
// coupling="cayley", order="inputs_outside", substeps=1, dt=0.02 (Algorithm 2).
std::string ph_arm_json(int n, PyObject *coupling = nullptr) {
  std::vector<double> C;
  if (coupling && coupling != Py_None)
    for (const std::vector<double> &row : ph_rows(coupling, "coupling"))
      C.insert(C.end(), row.begin(), row.end());
  return cvc::ph::system_json(cvc::ph::arm_system(cvc::ph::arm_joint(), n, C));
}

// The count-transfer study, as `cvc ph arm` runs it: the coupling fitted at train_n joints, then
// takeover MSE without and with it at each of test_n. Returns {"U" (smax rows), "smax",
// "mse_start", "mse", "evals", "counts": {n: {"none", "fitted"}}, "system" (the fitted arm at
// train_n, cvc-ph-system/1)}.
PyObject *ph_arm_study(int train_n = 4, PyObject *test_n = nullptr, int train = 200, int test = 20,
                       int steps = 200, int context = 5, int horizon = 50,
                       const std::string &observer = "fit", unsigned seed = 1) {
  std::vector<int> counts = {2, 3, 4, 5, 6, 8};
  if (test_n && test_n != Py_None)
    counts = ph_ints(test_n, "test_n");
  cvc::ph::coupling_fit_options f;
  f.takeover.context = context;
  f.takeover.horizon = horizon;
  if (observer == "fd")
    f.takeover.observer.kind = cvc::ph::observer_kind::finite_difference;
  else if (observer != "fit")
    throw std::invalid_argument("ph_arm_study: observer is 'fit' or 'fd'");
  f.smax = train_n;
  for (int n : counts)
    f.smax = std::max(f.smax, n);
  const cvc::ph::arm_params p;
  cvc::ph::coupling_fit_result fit;
  std::vector<std::pair<double, double>> mse;
  std::string err, system;
  Py_BEGIN_ALLOW_THREADS
  try {
    const cvc::ph::law_ptr joint = cvc::ph::arm_joint(p);
    const cvc::ph::transition_opts o = cvc::ph::arm_transition(p);
    fit = cvc::ph::fit_arm_coupling(joint, train_n, cvc::ph::arm_dataset(train_n, train, steps, seed, p), o, f);
    for (int n : counts) {
      const auto data = cvc::ph::arm_dataset(n, test, steps, seed + 100 + n, p);
      mse.emplace_back(cvc::ph::takeover_mse(cvc::ph::arm_system(joint, n), o, data, f.takeover),
                       cvc::ph::takeover_mse(cvc::ph::arm_system(joint, n, cvc::ph::count_coupling(fit.U, fit.smax, n)),
                                             o, data, f.takeover));
    }
    system = cvc::ph::system_json(cvc::ph::arm_system(joint, train_n, cvc::ph::count_coupling(fit.U, fit.smax, train_n)));
  } catch (const std::exception &e) {
    err = e.what();
    if (err.empty())
      err = "ph_arm_study failed";
  }
  Py_END_ALLOW_THREADS
  if (!err.empty())
    throw std::runtime_error(err);
  PyObject *U = PyList_New(fit.smax);
  for (int a = 0; a < fit.smax; ++a)
    PyList_SET_ITEM(U, a, ph_list(fit.U.data() + size_t(a) * fit.smax, fit.smax));
  PyObject *by = PyDict_New();
  for (size_t k = 0; k < counts.size(); ++k) {
    PyObject *key = PyLong_FromLong(counts[k]);
    PyObject *v = Py_BuildValue("{s:d,s:d}", "none", mse[k].first, "fitted", mse[k].second);
    PyDict_SetItem(by, key, v);
    Py_DECREF(key);
    Py_DECREF(v);
  }
  PyObject *out = Py_BuildValue("{s:O,s:i,s:d,s:d,s:i,s:O,s:s}", "U", U, "smax", fit.smax, "mse_start", fit.mse_start,
                                "mse", fit.mse, "evals", fit.evals, "counts", by, "system", system.c_str());
  Py_DECREF(U);
  Py_DECREF(by);
  return out;
}

// A Go2 leg (mechanical.h: hip fixed, the foot's x, y, z ports, inputs the three joint torques then
// a force at the foot) on compliant ground (contact_law and its Hunt-Crossley damper on the foot's
// z), as cvc-ph-system/1 text. Its blocks are the leg (named by `leg`) and "ground". Simulate with
// coupling="cayley", order="inputs_with_coupling", substeps=1.
std::string ph_go2_leg_json(const std::string &leg = "FL", double stiffness = 2.0e4, double damping = 2.0e5) {
  cvc::ph::composite s;
  const int b = s.add_block(cvc::ph::go2_leg_law(cvc::ph::go2_leg_of(leg)), leg);
  cvc::ph::attach_contact(s, b, 2, stiffness, damping, "ground");
  return cvc::ph::system_json(s);
}

// A Go2 foot in the base frame at the leg's joint angles q: {"r": [x, y, z], "jacobian": 3 rows}.
PyObject *ph_go2_foot(const std::string &leg, PyObject *q) {
  const std::vector<double> qs = ph_floats(q, "q");
  if (qs.size() != 3)
    throw std::invalid_argument("ph_go2_foot: three joint angles");
  double r[3], J[9];
  cvc::ph::go2_foot(cvc::ph::go2_leg_of(leg)).eval(qs.data(), r, J);
  PyObject *rows = PyList_New(3);
  for (int i = 0; i < 3; ++i)
    PyList_SET_ITEM(rows, i, ph_list(J + 3 * i, 3));
  PyObject *rr = ph_list(r, 3);
  PyObject *out = Py_BuildValue("{s:O,s:O}", "r", rr, "jacobian", rows);
  Py_DECREF(rr);
  Py_DECREF(rows);
  return out;
}

} // namespace pycvc
%}
