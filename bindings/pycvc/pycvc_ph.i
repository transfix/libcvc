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
#include <cvc/ph/compose.h>

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
                     int substeps = 2, int local_substeps = 1, int max_iter = 10, double tol = 1e-6) const {
    cvc::ph::transition_opts o;
    o.dt = dt;
    o.scheme = cvc::ph::step_scheme_of(scheme);
    o.coupling = cvc::ph::coupling_scheme_of(coupling);
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

} // namespace pycvc
%}
