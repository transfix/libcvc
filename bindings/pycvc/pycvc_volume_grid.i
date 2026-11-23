// pycvc_volume_grid.i — the data side of VolumeGridRover (inc/cvc/volume/volume_grid.h), what
// cvcGL's volume_grid node edits and keeps (docs/roadmap/VOLROVER-LIBRARY-PORTS-ROADMAP.md §4, P8):
//
//   ed = pycvc.point_class_editor()
//   ed.set_dims((64, 64, 64))
//   ed.set_classes(classes)                         # a pycvc.point_classes
//   ed.toggle(0, pycvc.slice_axis_z, (12, 40, 7))   # point_class_editor.toggled_added, or _removed
//   ed.select_rect(pycvc.slice_axis_z, 7, 0, 0, 20, 50, pycvc.selection_mode_replace)
//   ed.selection()                                  # [(cls, (i, j, k)), ...]
//   ed.move_selection(pycvc.slice_axis_z, 2, -1)    # (2, -1): the move made
//   ed.delete_selected(); ed.undo()
//   g = pycvc.point_class_gestures()
//   o = g.apply(ed, pycvc.grid_tool_points, 0, 1.0,
//               pycvc.grid_pick(phase="down", axis=pycvc.slice_axis_z, ijk=(4, 5, 6), inside=True))
//   o.points_changed, o.selection_changed, o.band_rect, o.note
//   pycvc.write_point_class_rows(pycvc.State.instance(app).child("doc.grid.classes"), classes)
//   pycvc.write_volume_grid_session(pycvc.volume_grid_session(keys={"grid.view": "xz"}), path)
//
// Voxels (dims, ijk, the pick's voxel) are tuples of ints, and any sequence of ints goes in. A
// point (point_ref) comes back as a (cls, (i, j, k)) tuple; a point argument takes such a tuple or
// a pycvc.point_ref. ed.nearest() is None when nothing is within reach. To learn which point a
// toggle added or removed, pass a pycvc.point_ref to fill: ed.toggle(cls, axis, v, scale, which).
// ed.classes() and session.classes are copies (set them with ed.set_classes(pc) and
// session.classes = pc). A class that does not exist raises IndexError; a voxel outside the
// volume, a bad dimension, a pick phase that does not parse or a session key a session does not
// keep, ValueError; a session file that can't be read or written, RuntimeError.
// volume_grid_params, volume_grid_display_params and volume_grid_source_params have their
// parameter tables (pycvc_params.i). These calls are cheap and keep the GIL.

%{
#include <climits>
#include <cvc/volume/volume_grid.h>
#include <limits>

namespace pycvc {
namespace grid {
// A Python sequence (not text) of n items.
inline bool is_seq(PyObject *o, Py_ssize_t n) {
  if (!o || !PySequence_Check(o) || PyUnicode_Check(o) || PyBytes_Check(o))
    return false;
  const Py_ssize_t size = PySequence_Size(o);
  if (size < 0) {
    PyErr_Clear();
    return false;
  }
  return size == n;
}
// n integers (anything with __index__: ints, numpy integers) into an array; a Python error set
// and false when the sequence is not that.
template <class T, std::size_t N> bool to_array(PyObject *o, std::array<T, N> &out) {
  if (!is_seq(o, static_cast<Py_ssize_t>(N))) {
    PyErr_Format(PyExc_TypeError, "expected a sequence of %d integers", static_cast<int>(N));
    return false;
  }
  for (std::size_t n = 0; n < N; ++n) {
    PyObject *item = PySequence_GetItem(o, static_cast<Py_ssize_t>(n));
    if (!item)
      return false;
    const long long v = PyLong_AsLongLong(item);
    Py_DECREF(item);
    if (v == -1 && PyErr_Occurred())
      return false;
    if (v < static_cast<long long>(std::numeric_limits<T>::min()) ||
        v > static_cast<long long>(std::numeric_limits<T>::max())) {
      PyErr_Format(PyExc_OverflowError, "%lld is out of range", v);
      return false;
    }
    out[n] = static_cast<T>(v);
  }
  return true;
}
template <class T, std::size_t N> PyObject *from_array(const std::array<T, N> &a) {
  PyObject *t = PyTuple_New(static_cast<Py_ssize_t>(N));
  if (!t)
    return nullptr;
  for (std::size_t n = 0; n < N; ++n)
    PyTuple_SET_ITEM(t, static_cast<Py_ssize_t>(n),
                     PyLong_FromLongLong(static_cast<long long>(a[n])));
  return t;
}
// A point as (cls, (i, j, k)).
inline PyObject *from_point(const cvc::point_ref &r) {
  PyObject *ijk = from_array(r.ijk);
  if (!ijk)
    return nullptr;
  return Py_BuildValue("(NN)", PyLong_FromSize_t(r.cls), ijk);
}
inline bool to_point(PyObject *o, cvc::point_ref &out) {
  if (!is_seq(o, 2)) {
    PyErr_SetString(PyExc_TypeError, "expected a point_ref or a (cls, (i, j, k)) pair");
    return false;
  }
  PyObject *c = PySequence_GetItem(o, 0);
  PyObject *ci = c ? PyNumber_Index(c) : nullptr;
  Py_XDECREF(c);
  if (!ci)
    return false;
  const std::size_t cls = PyLong_AsSize_t(ci);
  Py_DECREF(ci);
  if (cls == static_cast<std::size_t>(-1) && PyErr_Occurred())
    return false;
  PyObject *ijk = PySequence_GetItem(o, 1);
  if (!ijk)
    return false;
  const bool ok = to_array(ijk, out.ijk);
  Py_DECREF(ijk);
  if (ok)
    out.cls = cls;
  return ok;
}
inline PyObject *from_points(const std::vector<cvc::point_ref> &v) {
  PyObject *out = PyList_New(static_cast<Py_ssize_t>(v.size()));
  if (!out)
    return nullptr;
  for (std::size_t n = 0; n < v.size(); ++n)
    PyList_SET_ITEM(out, static_cast<Py_ssize_t>(n), from_point(v[n]));
  return out;
}
// A session's keys: a dict of str to str, both ways.
inline bool to_string_map(PyObject *o, std::map<std::string, std::string> &out) {
  PyObject *items = PyMapping_Check(o) ? PyMapping_Items(o) : nullptr;
  if (!items) {
    PyErr_Clear();
    PyErr_SetString(PyExc_TypeError, "expected a dict of str keys and str values");
    return false;
  }
  out.clear();
  const Py_ssize_t n = PyList_Size(items);
  for (Py_ssize_t i = 0; i < n; ++i) {
    PyObject *kv = PyList_GetItem(items, i); // borrowed
    PyObject *k = PyTuple_GetItem(kv, 0), *v = PyTuple_GetItem(kv, 1);
    if (!k || !v || !PyUnicode_Check(k) || !PyUnicode_Check(v)) {
      Py_DECREF(items);
      PyErr_SetString(PyExc_TypeError, "a session's keys and values are str");
      return false;
    }
    Py_ssize_t kn = 0, vn = 0;
    const char *ks = PyUnicode_AsUTF8AndSize(k, &kn);
    const char *vs = ks ? PyUnicode_AsUTF8AndSize(v, &vn) : nullptr;
    if (!vs) {
      Py_DECREF(items);
      return false;
    }
    out[std::string(ks, static_cast<std::size_t>(kn))] =
        std::string(vs, static_cast<std::size_t>(vn));
  }
  Py_DECREF(items);
  return true;
}
inline PyObject *from_string_map(const std::map<std::string, std::string> &m) {
  PyObject *d = PyDict_New();
  if (!d)
    return nullptr;
  for (const auto &kv : m) {
    PyObject *v = PyUnicode_DecodeUTF8(kv.second.data(), static_cast<Py_ssize_t>(kv.second.size()),
                                       "surrogateescape");
    if (!v || PyDict_SetItemString(d, kv.first.c_str(), v) < 0) {
      Py_XDECREF(v);
      Py_DECREF(d);
      return nullptr;
    }
    Py_DECREF(v);
  }
  return d;
}
} // namespace grid
} // namespace pycvc
%}

// The enums' descriptions are for the parameter tables (C++ only); pycvc_volume_slice.i defines
// this too.
#ifndef BOOST_DESCRIBE_ENUM
#define BOOST_DESCRIBE_ENUM(E, ...)
#endif

// ── Typemaps, for this file's declarations only (cleared at its end) ──
// Voxels: std::array<int, 3> (dims, a point's ijk) and the picks' std::array<std::int64_t, N>;
// by const reference for the calls, by value for the %naturalvar members.
%define PYCVC_GRID_INT_ARRAY(T, N)
%typemap(in) const std::array<T, N> & (std::array<T, N> tmp) {
  if (!pycvc::grid::to_array($input, tmp))
    SWIG_fail;
  $1 = &tmp;
}
%typemap(in) std::array<T, N> {
  if (!pycvc::grid::to_array($input, $1))
    SWIG_fail;
}
%typemap(typecheck, precedence=SWIG_TYPECHECK_INT32_ARRAY) const std::array<T, N> &,
                                                           std::array<T, N> {
  $1 = pycvc::grid::is_seq($input, N) ? 1 : 0;
}
%typemap(out) const std::array<T, N> & {
  $result = pycvc::grid::from_array(*$1);
}
%typemap(out) std::array<T, N> {
  $result = pycvc::grid::from_array(static_cast<const std::array<T, N> &>($1));
}
%enddef
PYCVC_GRID_INT_ARRAY(int, 3)
PYCVC_GRID_INT_ARRAY(std::int64_t, 3)
PYCVC_GRID_INT_ARRAY(std::int64_t, 4)

// A point argument: a pycvc.point_ref or a (cls, (i, j, k)) pair.
%typemap(in) const cvc::point_ref & (cvc::point_ref tmp) {
  void *argp = nullptr;
  if (SWIG_IsOK(SWIG_ConvertPtr($input, &argp, $descriptor(cvc::point_ref *), 0)) && argp) {
    $1 = reinterpret_cast<cvc::point_ref *>(argp);
  } else {
    if (!pycvc::grid::to_point($input, tmp))
      SWIG_fail;
    $1 = &tmp;
  }
}
%typemap(typecheck, precedence=SWIG_TYPECHECK_POINTER) const cvc::point_ref & {
  void *vp = nullptr;
  const int flags = SWIG_POINTER_NO_NULL;
  $1 = (SWIG_IsOK(SWIG_ConvertPtr($input, &vp, $descriptor(cvc::point_ref *), flags)) ||
        pycvc::grid::is_seq($input, 2))
           ? 1
           : 0;
}
// Points out: lists of (cls, (i, j, k)); nearest()'s none is None.
%typemap(out) const std::vector<cvc::point_ref> & {
  $result = pycvc::grid::from_points(*$1);
}
%typemap(out) std::vector<cvc::point_ref> {
  $result = pycvc::grid::from_points(static_cast<const std::vector<cvc::point_ref> &>($1));
}
%typemap(out) std::optional<cvc::point_ref> {
  const std::optional<cvc::point_ref> &found = $1;
  if (found) {
    $result = pycvc::grid::from_point(*found);
  } else {
    Py_INCREF(Py_None);
    $result = Py_None;
  }
}
// move_selection()'s move and grid_plane_axes(): (r, u).
%typemap(out) std::pair<int, int> {
  const std::pair<int, int> &ru = $1;
  $result = Py_BuildValue("(ii)", ru.first, ru.second);
}
// ed.classes() and session.classes: a copy Python owns, never a view into the editor or session.
%typemap(out) const cvc::point_classes & {
  $result = SWIG_NewPointerObj(new cvc::point_classes(*$1), $descriptor(cvc::point_classes *),
                               SWIG_POINTER_OWN);
}
// A session's keys: a dict of str to str.
%typemap(in) const std::map<std::string, std::string> & (std::map<std::string, std::string> tmp) {
  if (!pycvc::grid::to_string_map($input, tmp))
    SWIG_fail;
  $1 = &tmp;
}
%typemap(in) std::map<std::string, std::string> {
  if (!pycvc::grid::to_string_map($input, $1))
    SWIG_fail;
}
%typemap(typecheck, precedence=SWIG_TYPECHECK_MAP) const std::map<std::string, std::string> &,
                                                   std::map<std::string, std::string> {
  $1 = PyDict_Check($input) ? 1 : 0;
}
%typemap(out) const std::map<std::string, std::string> & {
  $result = pycvc::grid::from_string_map(*$1);
}
%typemap(out) std::map<std::string, std::string> {
  $result = pycvc::grid::from_string_map(
      static_cast<const std::map<std::string, std::string> &>($1));
}
%naturalvar cvc::point_ref::ijk;
%naturalvar cvc::grid_pick::ijk;
%naturalvar cvc::grid_pick_outcome::slice;
%naturalvar cvc::grid_pick_outcome::band_rect;
%naturalvar cvc::volume_grid_session::keys;
%naturalvar cvc::volume_grid_session::classes;

// ── Exceptions: the C++ types' Python counterparts, for this file's calls ──
%exception {
  try {
    $action
  } catch (const std::out_of_range &e) {
    SWIG_exception(SWIG_IndexError, e.what());
  } catch (const std::invalid_argument &e) {
    SWIG_exception(SWIG_ValueError, e.what());
  } catch (const cvc::exception &e) {
    SWIG_exception(SWIG_RuntimeError, e.what());
  } catch (const std::exception &e) {
    SWIG_exception(SWIG_RuntimeError, e.what());
  } catch (...) {
    SWIG_exception(SWIG_RuntimeError, "pycvc: C++ exception (see libcvc)");
  }
}

%ignore cvc::parse_grid_pick_phase; // grid_pick_phase_from below
%ignore cvc::point_ref::operator!=;
%rename(__eq__) cvc::point_ref::operator==;
%rename(__lt__) cvc::point_ref::operator<;
// read_point_class_rows(rows, variable, timestep, problems=None) is Python, below.
%rename(_read_point_class_rows) cvc::read_point_class_rows;

%include "cvc/volume/volume_grid.h"

PYCVC_PARAM_TABLE(volume_grid_params, cvc::volume_grid_params)
PYCVC_PARAM_TABLE(volume_grid_display_params, cvc::volume_grid_display_params)
PYCVC_PARAM_TABLE(volume_grid_source_params, cvc::volume_grid_source_params)

%extend cvc::point_ref {
  point_ref() { return new cvc::point_ref(); }
  point_ref(std::size_t cls, const std::array<int, 3> &ijk) {
    cvc::point_ref *r = new cvc::point_ref();
    r->cls = cls;
    r->ijk = ijk;
    return r;
  }
  // (cls, (i, j, k)), as the editor hands points out.
  PyObject *as_tuple() const { return pycvc::grid::from_point(*$self); }
  %pythoncode %{
    def __hash__(self):
        return hash(self.as_tuple())

    def __repr__(self):
        return "point_ref(%d, %r)" % (self.cls, self.ijk)
  %}
}

%extend cvc::point_class_editor {
  %pythoncode %{
    def __repr__(self):
        return "<point_class_editor %s, %d classes, %d points, %d selected>" % (
            "x".join(str(d) for d in self.dims()), len(self.classes().classes),
            self.classes().point_count(), len(self.selection()))
  %}
}

%extend cvc::grid_pick_outcome {
  %pythoncode %{
    _fields = ("slice", "band", "band_axis", "band_index", "band_rect", "points_changed",
               "selection_changed", "note")

    def to_dict(self):
        """The outcome's fields as a dict."""
        return {k: getattr(self, k) for k in self._fields}

    def __repr__(self):
        return "grid_pick_outcome(%s)" % ", ".join(
            "%s=%r" % (k, getattr(self, k)) for k in self._fields)
  %}
}

%extend cvc::volume_grid_session {
  bool __eq__(const cvc::volume_grid_session &o) const {
    return $self->keys == o.keys && $self->transfer_function == o.transfer_function &&
           $self->classes == o.classes;
  }
  %pythoncode %{
    def __repr__(self):
        return "<volume_grid_session %d keys, %d classes>" % (len(self.keys),
                                                              len(self.classes.classes))
  %}
}

%inline %{
namespace pycvc {
// A pick's phase by name: down, drag or up; ValueError for anything else.
cvc::grid_pick::phase_t grid_pick_phase_from(const std::string &text) {
  cvc::grid_pick::phase_t p = cvc::grid_pick::phase_t::down;
  if (!cvc::parse_grid_pick_phase(text, p))
    throw std::invalid_argument("'" + text + "' is not a pick phase: down, drag or up");
  return p;
}
} // namespace pycvc
%}

%clear const std::array<int, 3> &;
%clear const std::array<std::int64_t, 3> &;
%clear const std::array<std::int64_t, 4> &;
%clear std::array<int, 3>;
%clear std::array<std::int64_t, 3>;
%clear std::array<std::int64_t, 4>;
%clear const cvc::point_ref &;
%clear const std::vector<cvc::point_ref> &;
%clear std::vector<cvc::point_ref>;
%clear std::optional<cvc::point_ref>;
%clear std::pair<int, int>;
%clear const cvc::point_classes &;
%clear const std::map<std::string, std::string> &;
%clear std::map<std::string, std::string>;
PYCVC_DEFAULT_EXCEPTION

%pythoncode %{
def _pycvc_keywords(cls, names, convert=None):
    """Keyword construction over the defaults for a plain struct (grid_pick, the session)."""
    init = cls.__init__

    def __init__(self, *args, **kw):
        init(self, *args)
        for k, v in kw.items():
            if k not in names:
                raise TypeError("%s: unknown keyword '%s'; valid: %s"
                                % (cls.__name__, k, ", ".join(names)))
            if convert and k in convert:
                convert[k](self, v)
            else:
                setattr(self, k, v)

    cls.__init__ = __init__
    return cls


def _grid_pick_phase(p, v):
    p.phase = grid_pick_phase_from(v) if isinstance(v, str) else v


def _grid_pick_mods(p, v):
    parse_grid_pick_mods(v, p)


# grid_pick(phase="drag", axis=slice_axis_y, ijk=(3, 6, 5), inside=True, mods="shift,ctrl"):
# phase by name or value, mods as the pick.mods text (or shift= / ctrl= / alt=).
_pycvc_keywords(grid_pick, ["phase", "axis", "ijk", "inside", "shift", "ctrl", "alt", "mods"],
                {"phase": _grid_pick_phase, "mods": _grid_pick_mods})
_pycvc_keywords(volume_grid_session, ["keys", "transfer_function", "classes", "contours"])


def read_point_class_rows(rows, variable=0, timestep=0, problems=None):
    """The classes the rows under `rows` (a State) describe, a list of point_class; what was wrong
    with them is appended to `problems` (a list) when it is given."""
    found = StringVector()
    out = list(_read_point_class_rows(rows, variable, timestep, found))
    if problems is not None:
        problems.extend(found)
    return out
%}
