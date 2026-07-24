// pycvc_exec.cpp — Exec implementation: run DSL programs in an app's context
// and bridge Python callables into the DSL as native_fn values.
// clang-format off
#include <Python.h>
// clang-format on

#include "pycvc_exec.h"

#include <stdexcept>

#ifdef CVC_STATE_EXEC

#include <cvc/core/app.h>
#include <cvc/core/state.h>
#include <cvc/core/state_exec/builtins.h>
#include <cvc/core/state_exec/intrinsics.h>
#include <cvc/core/state_exec/process.h>
#include <cvc/core/state_exec/scheduler.h>
#include <cvc/core/state_exec/types.h>
#include <span>
#include <string>
#include <variant>
#include <vector>

namespace pycvc {

namespace {
using namespace cvc::state_exec;

// ── value_t <-> Python marshaling ───────────────────────────────────────
// Convert a DSL value to a NEW Python reference. Scalars + list/dict cross;
// opaque DSL types (symbol/closure/native_fn/data/generator) degrade to their
// printed form so a Python fn still receives something sensible.
PyObject *value_to_py(const value_t &v) {
  if (v.is_nil())
    Py_RETURN_NONE;
  if (const auto *b = std::get_if<bool>(&v.v))
    return PyBool_FromLong(*b);
  if (const auto *i = std::get_if<int64_t>(&v.v))
    return PyLong_FromLongLong(static_cast<long long>(*i));
  if (const auto *d = std::get_if<double>(&v.v))
    return PyFloat_FromDouble(*d);
  if (const auto *s = std::get_if<std::string>(&v.v))
    return PyUnicode_FromStringAndSize(s->data(), static_cast<Py_ssize_t>(s->size()));
  if (const auto *l = std::get_if<list_ptr>(&v.v)) {
    const auto &vec = **l;
    PyObject *out = PyList_New(static_cast<Py_ssize_t>(vec.size()));
    for (Py_ssize_t k = 0; k < static_cast<Py_ssize_t>(vec.size()); ++k)
      PyList_SET_ITEM(out, k, value_to_py(vec[k])); // steals the new ref
    return out;
  }
  if (const auto *dd = std::get_if<dict_ptr>(&v.v)) {
    const auto &pairs = **dd;
    PyObject *out = PyDict_New();
    for (const auto &kv : pairs) {
      PyObject *pv = value_to_py(kv.second);
      PyDict_SetItemString(out, kv.first.c_str(), pv);
      Py_DECREF(pv);
    }
    return out;
  }
  // Fallback: the DSL's printed form.
  std::string s = to_string(v);
  return PyUnicode_FromStringAndSize(s.data(), static_cast<Py_ssize_t>(s.size()));
}

// Convert a Python object (borrowed) to a DSL value. bool BEFORE int (bool is a
// subclass of int in Python). dict order is preserved.
value_t py_to_value(PyObject *o) {
  if (o == nullptr || o == Py_None)
    return value_t(std::monostate{});
  if (PyBool_Check(o))
    return value_t(o == Py_True);
  if (PyLong_Check(o))
    return value_t(static_cast<int64_t>(PyLong_AsLongLong(o)));
  if (PyFloat_Check(o))
    return value_t(PyFloat_AsDouble(o));
  if (PyUnicode_Check(o)) {
    Py_ssize_t n = 0;
    const char *s = PyUnicode_AsUTF8AndSize(o, &n);
    return value_t(std::string(s ? s : "", s ? static_cast<size_t>(n) : 0));
  }
  if (PyList_Check(o) || PyTuple_Check(o)) {
    bool tup = PyTuple_Check(o);
    Py_ssize_t n = tup ? PyTuple_Size(o) : PyList_Size(o);
    std::vector<value_t> elems;
    elems.reserve(static_cast<size_t>(n));
    for (Py_ssize_t k = 0; k < n; ++k)
      elems.push_back(py_to_value(tup ? PyTuple_GetItem(o, k) : PyList_GetItem(o, k)));
    return value_t(std::make_shared<std::vector<value_t>>(std::move(elems)));
  }
  if (PyDict_Check(o)) {
    auto pairs = std::make_shared<std::vector<std::pair<std::string, value_t>>>();
    PyObject *k = nullptr, *v = nullptr;
    Py_ssize_t pos = 0;
    while (PyDict_Next(o, &pos, &k, &v)) {
      PyObject *ks = PyObject_Str(k);
      Py_ssize_t n = 0;
      const char *s = ks ? PyUnicode_AsUTF8AndSize(ks, &n) : "";
      pairs->emplace_back(std::string(s ? s : "", s ? static_cast<size_t>(n) : 0), py_to_value(v));
      Py_XDECREF(ks);
    }
    return value_t(dict_ptr(std::move(pairs)));
  }
  // Anything else: its str().
  PyObject *s = PyObject_Str(o);
  Py_ssize_t n = 0;
  const char *cs = s ? PyUnicode_AsUTF8AndSize(s, &n) : "";
  value_t out(std::string(cs ? cs : "", cs ? static_cast<size_t>(n) : 0));
  Py_XDECREF(s);
  return out;
}

std::string fetch_py_error() {
  if (!PyErr_Occurred())
    return "unknown error";
  PyObject *type = nullptr, *val = nullptr, *tb = nullptr;
  PyErr_Fetch(&type, &val, &tb);
  PyErr_NormalizeException(&type, &val, &tb);
  std::string msg = "error";
  if (val) {
    PyObject *s = PyObject_Str(val);
    if (s) {
      const char *cs = PyUnicode_AsUTF8(s);
      if (cs)
        msg = cs;
      Py_DECREF(s);
    }
  }
  Py_XDECREF(type);
  Py_XDECREF(val);
  Py_XDECREF(tb);
  return msg;
}

// Wrap a Python callable as a DSL native_fn. The PyObject is kept alive by a
// shared_ptr whose deleter DECREFs under the GIL, so the fn owns its callable
// for as long as the environment holds it.
native_fn make_python_native_fn(const std::string &name, PyObject *callable) {
  Py_INCREF(callable);
  std::shared_ptr<PyObject> holder(callable, [](PyObject *p) {
    PyGILState_STATE g = PyGILState_Ensure();
    Py_DECREF(p);
    PyGILState_Release(g);
  });
  return [name, holder](std::span<const value_t> args) -> value_t {
    PyGILState_STATE gil = PyGILState_Ensure();
    PyObject *pyargs = PyTuple_New(static_cast<Py_ssize_t>(args.size()));
    for (Py_ssize_t i = 0; i < static_cast<Py_ssize_t>(args.size()); ++i)
      PyTuple_SET_ITEM(pyargs, i, value_to_py(args[i])); // steals the new ref
    PyObject *res = PyObject_CallObject(holder.get(), pyargs);
    Py_DECREF(pyargs);
    if (res == nullptr) {
      // Contain the Python exception: translate to a C++ error carrying the
      // message. (Surfaces out of run() as a Python RuntimeError.)
      std::string msg = fetch_py_error();
      PyGILState_Release(gil);
      throw std::runtime_error("python DSL fn '" + name + "': " + msg);
    }
    value_t out = py_to_value(res);
    Py_DECREF(res);
    PyGILState_Release(gil);
    return out;
  };
}

std::string render_result(const value_t &v) {
  if (const auto *s = std::get_if<std::string>(&v.v))
    return *s;
  if (v.is_nil())
    return "";
  return to_string(v);
}
} // namespace

// Owns everything an evaluation needs. The intrinsics capture &ictx (a raw
// pointer), so ictx must outlive any run — holding it here (Exec-lifetime)
// satisfies that. Mirrors the C++ tests' make_exec_env().
struct Exec::ExecImpl {
  std::shared_ptr<cvc::app> app; // co-own so root/ictx stay valid for our lifetime
  cvc::state *root = nullptr;
  scheduler sched;
  intrinsics_context ictx;
  process_ptr host_proc;
  environment_ptr env;
};

Exec::Exec(const std::shared_ptr<cvc::app> &app) : impl_(std::make_shared<ExecImpl>()) {
  if (!app)
    throw std::invalid_argument("pycvc.Exec: null app handle");
  impl_->app = app; // keep the app alive for as long as this Exec exists
  impl_->root = &cvc::state::instance(*app);
  impl_->sched.set_watch_root(impl_->root);

  impl_->host_proc = make_process();
  impl_->host_proc->pid = 0;
  impl_->host_proc->status = process_status::ready;

  impl_->ictx.sched = &impl_->sched;
  impl_->ictx.root = impl_->root;
  impl_->ictx.proc = impl_->host_proc;
  impl_->ictx.pid = 0;
  impl_->ictx.uid = "pycvc";
  impl_->ictx.cluster_id = "local";
  impl_->ictx.node_id = "local";

  impl_->env = builtins::make_default_environment();
  register_intrinsics(impl_->env, &impl_->ictx);
}

Exec::~Exec() = default;

void Exec::register_fn(const std::string &name, PyObject *callable) {
  if (callable == nullptr || !PyCallable_Check(callable))
    throw std::invalid_argument("pycvc.Exec.register_fn: '" + name + "' is not callable");
  builtins::register_fn(impl_->env, name, make_python_native_fn(name, callable));
}

std::string Exec::run(const std::string &src) {
  execute_options opts;
  opts.name = "pycvc";
  opts.env = impl_->env;
  int pid = impl_->sched.execute(src, opts); // may throw on parse error
  // The scheduler has no try/catch, so a native_fn (e.g. a Python DSL function)
  // that throws aborts run() and leaves its process un-terminated — it would
  // re-run on the NEXT run(). Reap it here so the failure is contained to this
  // call and the Exec stays usable afterward.
  try {
    impl_->sched.run();
  } catch (...) {
    impl_->sched.kill(pid);
    throw;
  }
  std::optional<value_t> result = impl_->sched.get_result(pid);
  if (!result.has_value())
    throw std::runtime_error("pycvc.Exec.run: program produced no result (pid " +
                             std::to_string(pid) + ")");
  return render_result(*result);
}

} // namespace pycvc

#else // !CVC_STATE_EXEC

namespace pycvc {

struct Exec::ExecImpl {};

Exec::Exec(const std::shared_ptr<cvc::app> &) {
  throw std::runtime_error("pycvc.Exec: this libcvc build was compiled without state_exec");
}
Exec::~Exec() = default;
void Exec::register_fn(const std::string &, PyObject *) {
  throw std::runtime_error("pycvc.Exec: this libcvc build was compiled without state_exec");
}
std::string Exec::run(const std::string &) {
  throw std::runtime_error("pycvc.Exec: this libcvc build was compiled without state_exec");
}

} // namespace pycvc

#endif // CVC_STATE_EXEC
