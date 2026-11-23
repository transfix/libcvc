// pycvc_net.i — cvc::net, the connection engine (inc/cvc/net/net_engine.h;
// docs/roadmap/ARIADNE_NET_ENGINE.md §2.7, part N2), in pycvc as it is in C++.
//
// DIRECT WRAP: net_engine.h is %include'd, so Python has the engine's whole API under its C++
// names: net_engine, engine_options, listener_options, framing, inbound_limits, conn_info,
// net_event, net_batch, net_counters, net_stats (listener_stats, thread_stats), close_reason and
// net_event's types (close_reason_slow_reader, net_event.type_frames, framing.mode_delimiter, as
// SWIG spells a C++ enum class). What Python needs beyond the C++ is added here, no more:
//
//  * BYTES. send, send_many and broadcast take any bytes-like object (bytes, bytearray, memoryview)
//    or a str (sent as UTF-8), and copy it once, as the C++ does. framing.delimited also takes a
//    one-byte bytes (b"|").
//  * DURATIONS. A std::chrono::milliseconds (wait's timeout, inbound_limits.window,
//    listener_options.flush_timeout) is an int of milliseconds.
//  * VALUES. A struct member of a class type (listener_options.frames and .limits, net_event.info,
//    net_stats.engine) reads as a copy, so set a whole one (`opts.limits =
//    inbound_limits(window=500)`); a vector member (framing.pause_after, net_stats.listeners and
//    .threads) reads as a list of copies. SWIG's own getters would point into the parent and
//    dangle once it went. The structs' constructors take any of their members as keywords.
//  * FRAMES. A net_batch's `events` is a list of net_event, and each event's `frames` a list of
//    memoryviews into the batch's buffers, with no copy. A memoryview keeps those buffers alive
//    for as long as it lives, so a frame stays valid after the batch is cleared or reused; copy it
//    (bytes(mv)) to keep it without keeping the read it came in.
//  * net_counters.closed is a dict, close reason name -> count.
//  * listen(options=None, **fields) raises OSError (with the C++ error) when it cannot listen, and
//    returns a net_listener: the listener_id, an int, with the asyncio callbacks below.
//  * The GIL. wait() and poll() release it (wait() blocks); so do stop() and the engine's
//    destruction, which join the I/O threads. set_notify(fn) takes a Python callable, which an I/O
//    thread calls with the GIL taken: keep it tiny, or use the waker below.
//  * net_engine.from_capsule(capsule): a host's engine (arisrvx's "cvc.net.net_engine",
//    _arihost.net()), which Python uses and never destroys or stops.
//
// THE ASYNCIO LOOP (§2.3, §2.7). A listener's on_accept, on_frames, on_close, on_writable and
// on_error register callbacks (as decorators) that run on an asyncio loop, never on an I/O thread:
// engine.attach_loop(loop) names it (pycvc.ariadne's Host does, for host.net), else the loop
// running when the first callback is registered. A waker thread, ServerLink's shape, waits in
// wait() without the GIL and hands each batch to the loop, then waits for the loop to take it, so
// what comes in while the loop is busy arrives as one larger batch. A batch is delivered in this
// order: accepts, each listener's frames (one on_frames call a listener, [(conn, [memoryview,
// ...]), ...], each connection once, in arrival order), writables, closes, errors. For any one
// connection that is the order the engine posted them. on_accept(conn, info) may return a reason
// (a str) to refuse the connection: it is closed at once, and its frames in that batch dropped.
// on_close(conn, reason) gets the reason's name ("peer_closed"); one that takes a third parameter
// also gets the event's message. A callback that returns an awaitable runs as a task.
//
// %include'd from pycvc.i. NOTE: a SWIG interface file — do NOT run clang-format.

%{
#include <cvc/net/net_engine.h>

#include <chrono>
#include <cstring>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <unordered_map>

namespace pycvc {
namespace net {

// Release the GIL for a scope (a C++ throw restores it on the way out).
class nogil {
public:
  nogil() : ts_(PyEval_SaveThread()) {}
  ~nogil() { PyEval_RestoreThread(ts_); }
  nogil(const nogil &) = delete;
  nogil &operator=(const nogil &) = delete;

private:
  PyThreadState *ts_;
};

// One frame: a buffer exporter over [data, data + size), holding the capsule that owns a copy of
// its batch's `keep` (the shared buffers), so the frame outlives the batch's clear() or its reuse
// by the next wait(). Its memoryview holds it, so the bytes live as long as the memoryview (and its
// slices) do. The type is made once a module, at its init (pycvc._net_frame), never in a static.
struct frame_buffer {
  PyObject_HEAD
  PyObject *owner;
  const char *data;
  Py_ssize_t size;
};

void frame_buffer_dealloc(PyObject *self) {
  Py_XDECREF(reinterpret_cast<frame_buffer *>(self)->owner);
  PyTypeObject *t = Py_TYPE(self);
  t->tp_free(self);
  Py_DECREF(t);
}

int frame_buffer_get(PyObject *self, Py_buffer *view, int flags) {
  auto *f = reinterpret_cast<frame_buffer *>(self);
  return PyBuffer_FillInfo(view, self, const_cast<char *>(f->data), f->size, /*readonly=*/1,
                           flags);
}

const PyType_Slot frame_buffer_slots[] = {
    {Py_tp_dealloc, reinterpret_cast<void *>(frame_buffer_dealloc)},
    {Py_bf_getbuffer, reinterpret_cast<void *>(frame_buffer_get)},
    {0, nullptr}};
const PyType_Spec frame_buffer_spec = {"pycvc._net_frame", sizeof(frame_buffer), 0,
                                       Py_TPFLAGS_DEFAULT,
                                       const_cast<PyType_Slot *>(frame_buffer_slots)};

// The frame type, for %init (CPython only reads the spec).
PyObject *make_frame_type() { return PyType_FromSpec(const_cast<PyType_Spec *>(&frame_buffer_spec)); }

void delete_keep(PyObject *capsule) {
  delete static_cast<std::vector<std::shared_ptr<const void>> *>(
      PyCapsule_GetPointer(capsule, "pycvc.net_frames"));
}

// A new owner of a copy of the batch's buffers, or nullptr with a Python error set.
PyObject *new_owner(const cvc::net::net_batch &batch) {
  auto *keep = new std::vector<std::shared_ptr<const void>>(batch.keep);
  PyObject *o = PyCapsule_New(keep, "pycvc.net_frames", delete_keep);
  if (o == nullptr)
    delete keep;
  return o;
}

// A memoryview of one frame, or nullptr with a Python error set.
PyObject *frame_view(PyTypeObject *t, PyObject *owner, std::string_view frame) {
  PyObject *b = t->tp_alloc(t, 0);
  if (b == nullptr)
    return nullptr;
  auto *f = reinterpret_cast<frame_buffer *>(b);
  Py_INCREF(owner);
  f->owner = owner;
  f->data = frame.data();
  f->size = static_cast<Py_ssize_t>(frame.size());
  PyObject *mv = PyMemoryView_FromObject(b);
  Py_DECREF(b);
  return mv;
}

// `frame_type` checked: pycvc._net_frame, as %init made it.
PyTypeObject *frame_type_of(PyObject *frame_type) {
  if (frame_type == nullptr || !PyType_Check(frame_type) ||
      reinterpret_cast<PyTypeObject *>(frame_type)->tp_basicsize !=
          static_cast<Py_ssize_t>(sizeof(frame_buffer))) {
    PyErr_SetString(PyExc_TypeError, "expected pycvc._net_frame");
    return nullptr;
  }
  return reinterpret_cast<PyTypeObject *>(frame_type);
}

// A new list of memoryviews of `ev`'s frames, or nullptr with a Python error set.
PyObject *frame_list(PyTypeObject *t, PyObject *owner, const cvc::net::net_event &ev,
                     PyObject *into = nullptr) {
  PyObject *list = into ? into : PyList_New(0);
  if (list == nullptr)
    return nullptr;
  for (std::string_view f : ev.frames) {
    PyObject *mv = frame_view(t, owner, f);
    if (mv == nullptr || PyList_Append(list, mv) < 0) {
      Py_XDECREF(mv);
      if (into == nullptr)
        Py_DECREF(list);
      return nullptr;
    }
    Py_DECREF(mv);
  }
  return list;
}

// net_batch's event list for the asyncio waker, in delivery order:
//   (accepts, frames, writables, closes, errors)
//   accepts:   [(listener, conn, conn_info)], each conn_info made by `info_of`
//   frames:    {listener: [(conn, [memoryview, ...]), ...]}, a connection once per listener, in the
//              order its first frames came
//   writables: [(listener, conn)]
//   closes:    [(listener, conn, reason name, message)]
//   errors:    [(listener, message)]
// A new reference, or nullptr with a Python error set.
PyObject *dispatch_lists(const cvc::net::net_batch &batch, PyTypeObject *frame_type,
                         PyObject *(*info_of)(const cvc::net::conn_info &)) {
  using T = cvc::net::net_event::type;
  PyObject *owner = nullptr, *accepts = nullptr, *frames = nullptr, *writables = nullptr,
           *closes = nullptr, *errors = nullptr, *out = nullptr;
  // Where each connection's frame list is in its listener's list, so a connection's later frames
  // join its first entry.
  std::unordered_map<cvc::net::conn_id, PyObject *> conn_frames; // borrowed, from `frames`
  owner = new_owner(batch);
  accepts = PyList_New(0);
  frames = PyDict_New();
  writables = PyList_New(0);
  closes = PyList_New(0);
  errors = PyList_New(0);
  if (!owner || !accepts || !frames || !writables || !closes || !errors)
    goto fail;
  for (const cvc::net::net_event &ev : batch.events) {
    PyObject *item = nullptr;
    switch (ev.kind) {
    case T::accepted: {
      PyObject *info = info_of(ev.info);
      if (info == nullptr)
        goto fail;
      item = Py_BuildValue("(KKN)", static_cast<unsigned long long>(ev.listener),
                           static_cast<unsigned long long>(ev.conn), info);
      if (item == nullptr || PyList_Append(accepts, item) < 0)
        goto fail_item;
      break;
    }
    case T::frames: {
      auto at = conn_frames.find(ev.conn);
      if (at != conn_frames.end()) {
        if (frame_list(frame_type, owner, ev, at->second) == nullptr)
          goto fail;
        break;
      }
      PyObject *key = PyLong_FromUnsignedLongLong(ev.listener);
      if (key == nullptr)
        goto fail;
      PyObject *per = PyDict_GetItemWithError(frames, key); // borrowed
      if (per == nullptr) {
        if (PyErr_Occurred() || (per = PyList_New(0)) == nullptr ||
            PyDict_SetItem(frames, key, per) < 0) {
          Py_XDECREF(per);
          Py_DECREF(key);
          goto fail;
        }
        Py_DECREF(per); // the dict holds it
      }
      Py_DECREF(key);
      PyObject *list = frame_list(frame_type, owner, ev);
      if (list == nullptr)
        goto fail;
      item = Py_BuildValue("(KO)", static_cast<unsigned long long>(ev.conn), list);
      Py_DECREF(list);
      if (item == nullptr || PyList_Append(per, item) < 0)
        goto fail_item;
      conn_frames[ev.conn] = list; // borrowed: the tuple in `per` holds it
      break;
    }
    case T::writable:
      item = Py_BuildValue("(KK)", static_cast<unsigned long long>(ev.listener),
                           static_cast<unsigned long long>(ev.conn));
      if (item == nullptr || PyList_Append(writables, item) < 0)
        goto fail_item;
      break;
    case T::closed:
      item = Py_BuildValue("(KKss#)", static_cast<unsigned long long>(ev.listener),
                           static_cast<unsigned long long>(ev.conn), cvc::net::to_string(ev.reason),
                           ev.message.data(), static_cast<Py_ssize_t>(ev.message.size()));
      if (item == nullptr || PyList_Append(closes, item) < 0)
        goto fail_item;
      break;
    case T::error:
      item = Py_BuildValue("(Ks#)", static_cast<unsigned long long>(ev.listener),
                           ev.message.data(), static_cast<Py_ssize_t>(ev.message.size()));
      if (item == nullptr || PyList_Append(errors, item) < 0)
        goto fail_item;
      break;
    }
    Py_XDECREF(item);
    continue;
  fail_item:
    Py_XDECREF(item);
    goto fail;
  }
  out = PyTuple_Pack(5, accepts, frames, writables, closes, errors);
fail:
  Py_XDECREF(owner);
  Py_XDECREF(accepts);
  Py_XDECREF(frames);
  Py_XDECREF(writables);
  Py_XDECREF(closes);
  Py_XDECREF(errors);
  return out;
}

// A Python callable as set_notify's function: called on an I/O thread with the GIL taken.
std::function<void()> notify_callback(PyObject *fn) {
  if (fn == nullptr || fn == Py_None)
    return nullptr;
  if (!PyCallable_Check(fn))
    throw std::invalid_argument("net_engine.set_notify: expected a callable or None");
  std::shared_ptr<PyObject> held = python_ref(fn);
  return [held]() {
    PyGILState_STATE g = PyGILState_Ensure();
    PyObject *r = PyObject_CallNoArgs(held.get());
    if (r == nullptr)
      PyErr_WriteUnraisable(held.get());
    Py_XDECREF(r);
    PyGILState_Release(g);
  };
}

} // namespace net
} // namespace pycvc
%}

%init %{
  {
    // pycvc._net_frame: the type of a frame's buffer (its memoryview's exporter), one a module.
    PyObject *frame_type = pycvc::net::make_frame_type();
    if (frame_type == nullptr || PyDict_SetItemString(d, "_net_frame", frame_type) < 0)
      PyErr_Print();
    Py_XDECREF(frame_type);
  }
%}

// ── types the header uses ─────────────────────────────────────────────────────────────────────────
%apply unsigned char { std::uint8_t };
%apply unsigned short { std::uint16_t };

// framing.delimited(byte): an int, or a one-byte bytes / str (b"|").
%typemap(in) std::uint8_t byte {
  if (PyBytes_Check($input) && PyBytes_GET_SIZE($input) == 1) {
    $1 = static_cast<std::uint8_t>(PyBytes_AS_STRING($input)[0]);
  } else if (PyUnicode_Check($input) && PyUnicode_GET_LENGTH($input) == 1 &&
             PyUnicode_READ_CHAR($input, 0) < 256) {
    $1 = static_cast<std::uint8_t>(PyUnicode_READ_CHAR($input, 0));
  } else {
    unsigned long v = PyLong_AsUnsignedLong($input);
    if ((v == static_cast<unsigned long>(-1) && PyErr_Occurred()) || v > 255) {
      PyErr_Clear();
      SWIG_exception_fail(SWIG_TypeError, "framing.delimited: expected a byte (0-255 or b\"|\")");
    }
    $1 = static_cast<std::uint8_t>(v);
  }
}

// std::chrono::milliseconds: an int of milliseconds.
%typemap(in) std::chrono::milliseconds {
  long long ms = PyLong_AsLongLong($input);
  if (ms == -1 && PyErr_Occurred())
    SWIG_exception_fail(SWIG_TypeError, "expected an int of milliseconds");
  $1 = std::chrono::milliseconds(ms);
}
%typemap(in) const std::chrono::milliseconds & (std::chrono::milliseconds tmp) {
  long long ms = PyLong_AsLongLong($input);
  if (ms == -1 && PyErr_Occurred())
    SWIG_exception_fail(SWIG_TypeError, "expected an int of milliseconds");
  tmp = std::chrono::milliseconds(ms);
  $1 = &tmp;
}
%typemap(typecheck, precedence=SWIG_TYPECHECK_INT64) std::chrono::milliseconds,
    const std::chrono::milliseconds & {
  $1 = PyLong_Check($input) ? 1 : 0;
}
%typemap(out) std::chrono::milliseconds {
  $result = PyLong_FromLongLong(static_cast<long long>($1.count()));
}
%typemap(out) const std::chrono::milliseconds & {
  $result = PyLong_FromLongLong(static_cast<long long>($1->count()));
}

// Bytes in: any bytes-like object, or a str as UTF-8, viewed for the call (the engine copies it).
%typemap(in) std::string_view (Py_buffer view, bool held = false) {
  if (PyUnicode_Check($input)) {
    Py_ssize_t n = 0;
    const char *s = PyUnicode_AsUTF8AndSize($input, &n);
    if (s == nullptr)
      SWIG_fail;
    $1 = std::string_view(s, static_cast<std::size_t>(n));
  } else {
    if (PyObject_GetBuffer($input, &view, PyBUF_SIMPLE) < 0) {
      PyErr_Clear();
      SWIG_exception_fail(SWIG_TypeError, "expected bytes, a bytes-like object or a str");
    }
    held = true;
    $1 = std::string_view(static_cast<const char *>(view.buf), static_cast<std::size_t>(view.len));
  }
}
%typemap(freearg) std::string_view {
  if (held$argnum)
    PyBuffer_Release(&view$argnum);
}
%typemap(typecheck, precedence=SWIG_TYPECHECK_STRING) std::string_view {
  $1 = (PyUnicode_Check($input) || PyObject_CheckBuffer($input)) ? 1 : 0;
}

// send_many's connections: any iterable of ints.
%typemap(in) const std::vector<cvc::net::conn_id> & (std::vector<cvc::net::conn_id> tmp) {
  PyObject *it = PyObject_GetIter($input);
  if (it == nullptr)
    SWIG_exception_fail(SWIG_TypeError, "send_many: expected an iterable of connection ids");
  for (PyObject *v; (v = PyIter_Next(it)) != nullptr;) {
    unsigned long long id = PyLong_AsUnsignedLongLong(v);
    Py_DECREF(v);
    if (id == static_cast<unsigned long long>(-1) && PyErr_Occurred()) {
      Py_DECREF(it);
      SWIG_fail;
    }
    tmp.push_back(static_cast<cvc::net::conn_id>(id));
  }
  Py_DECREF(it);
  if (PyErr_Occurred())
    SWIG_fail;
  $1 = &tmp;
}
%typemap(typecheck, precedence=SWIG_TYPECHECK_POINTER) const std::vector<cvc::net::conn_id> & {
  PyObject *it = PyObject_GetIter($input);
  $1 = it != nullptr ? 1 : 0;
  Py_XDECREF(it);
  PyErr_Clear();
}

// ── the header, curated ───────────────────────────────────────────────────────────────────────────
%feature("flatnested") cvc::net::net_stats::listener_stats;
%feature("flatnested") cvc::net::net_stats::thread_stats;
// A member of a class type reads as a copy: SWIG's own getter returns a pointer into the parent,
// which dangles once the parent proxy goes (`e.stats().threads`). Vectors read as lists.
%define PYCVC_NET_VALUE(cls, type, member)
%ignore cls::member;
%extend cls {
  type _get_##member() const { return $self->member; }
  void _set_##member(const type &value) { $self->member = value; }
  %pythoncode %{
    member = property(_get_##member, _set_##member)
  %}
}
%enddef
PYCVC_NET_VALUE(cvc::net::listener_options, cvc::net::framing, frames)
PYCVC_NET_VALUE(cvc::net::listener_options, cvc::net::inbound_limits, limits)
PYCVC_NET_VALUE(cvc::net::net_event, cvc::net::conn_info, info)
PYCVC_NET_VALUE(cvc::net::net_stats, cvc::net::net_counters, engine)
PYCVC_NET_VALUE(cvc::net::net_stats::listener_stats, cvc::net::net_counters, counters)
%ignore cvc::net::framing::pause_after;
%ignore cvc::net::net_stats::listeners;
%ignore cvc::net::net_stats::threads;
%extend cvc::net::framing {
  PyObject *_get_pause_after() const {
    PyObject *list = PyList_New(0);
    for (const std::string &p : $self->pause_after) {
      PyObject *v = list ? PyUnicode_DecodeUTF8(p.data(), static_cast<Py_ssize_t>(p.size()),
                                                "surrogateescape")
                         : nullptr;
      if (v == nullptr || PyList_Append(list, v) < 0)
        Py_CLEAR(list);
      Py_XDECREF(v);
    }
    return list;
  }
  void _set_pause_after(const std::vector<std::string> &value) { $self->pause_after = value; }
  %pythoncode %{
    pause_after = property(_get_pause_after, _set_pause_after)
  %}
}
%extend cvc::net::net_stats {
  PyObject *_get_listeners() const {
    PyObject *list = PyList_New(0);
    for (const auto &l : $self->listeners) {
      PyObject *v = list ? SWIG_NewPointerObj(new cvc::net::net_stats::listener_stats(l),
                                              $descriptor(cvc::net::net_stats::listener_stats *),
                                              SWIG_POINTER_OWN)
                         : nullptr;
      if (v == nullptr || PyList_Append(list, v) < 0)
        Py_CLEAR(list);
      Py_XDECREF(v);
    }
    return list;
  }
  PyObject *_get_threads() const {
    PyObject *list = PyList_New(0);
    for (const auto &t : $self->threads) {
      PyObject *v = list ? SWIG_NewPointerObj(new cvc::net::net_stats::thread_stats(t),
                                              $descriptor(cvc::net::net_stats::thread_stats *),
                                              SWIG_POINTER_OWN)
                         : nullptr;
      if (v == nullptr || PyList_Append(list, v) < 0)
        Py_CLEAR(list);
      Py_XDECREF(v);
    }
    return list;
  }
  %pythoncode %{
    listeners = property(_get_listeners, doc="The listeners' counters: listener_stats copies.")
    threads = property(_get_threads, doc="Each I/O thread's: thread_stats copies.")
  %}
}

%ignore cvc::net::net_engine::impl;
%ignore cvc::net::net_engine::operator=;
%ignore cvc::net::net_engine::listen;
%ignore cvc::net::net_engine::set_notify(std::function<void()>);
%rename(_stop) cvc::net::net_engine::stop;
// The payload overloads: Python bytes are copied once either way, by the string_view ones.
%ignore cvc::net::net_engine::send(conn_id, payload);
%ignore cvc::net::net_engine::broadcast(const std::string &, payload, conn_id);
%ignore cvc::net::net_event::frames; // memoryviews, from net_batch.events
%ignore cvc::net::net_batch::events;
%ignore cvc::net::net_batch::keep;
%ignore cvc::net::net_counters::closed; // a dict, below
%rename(close_reason_name) cvc::net::to_string;
%feature("compactdefaultargs") cvc::net::net_engine::broadcast;
%feature("compactdefaultargs") cvc::net::net_engine::close;

// wait() and poll() release the GIL (wait blocks; neither then waits on the GIL an I/O thread may
// need for set_notify's callable); so do stop() and the destruction, which join the I/O threads.
%define PYCVC_NET_NOGIL(name)
%exception name {
  try {
    pycvc::net::nogil unlocked;
    $action
  } catch (const std::exception &e) {
    SWIG_exception(SWIG_RuntimeError, e.what());
  }
}
%enddef
PYCVC_NET_NOGIL(cvc::net::net_engine::wait)
PYCVC_NET_NOGIL(cvc::net::net_engine::poll)
PYCVC_NET_NOGIL(cvc::net::net_engine::stop)
PYCVC_NET_NOGIL(cvc::net::net_engine::~net_engine)

%include "cvc/net/net_engine.h"

%exception cvc::net::net_engine::_listen {
  try {
    $action
  } catch (const std::system_error &e) {
    PyErr_SetString(PyExc_OSError, e.what());
    SWIG_fail;
  } catch (const std::exception &e) {
    SWIG_exception(SWIG_RuntimeError, e.what());
  }
}
%exception cvc::net::net_engine::set_notify {
  try {
    $action
  } catch (const std::invalid_argument &e) {
    SWIG_exception(SWIG_TypeError, e.what());
  }
}

%extend cvc::net::net_engine {
  // The listener's id; std::system_error (OSError) with the engine's reason when it cannot listen.
  cvc::net::listener_id _listen(const cvc::net::listener_options &options) {
    std::string error;
    const cvc::net::listener_id id = $self->listen(options, &error);
    if (id == 0)
      throw std::system_error(std::make_error_code(std::errc::address_not_available),
                              "net_engine.listen " + options.address + ":" +
                                  std::to_string(options.port) + ": " + error);
    return id;
  }

  void set_notify(PyObject *notify) { $self->set_notify(pycvc::net::notify_callback(notify)); }

  // The engine's address, which keys the asyncio waker (one a C++ engine, however many proxies).
  unsigned long long _address() const { return reinterpret_cast<unsigned long long>($self); }

  %pythoncode %{
    @staticmethod
    def from_capsule(capsule):
        """A host's engine (a "cvc.net.net_engine" capsule: arisrvx's _arihost.net()), which this
        process uses but never destroys or stops."""
        return _net_engine_from_capsule(capsule)

    def listen(self, options=None, **fields):
        """Listen with ``options`` (a listener_options), or one made of ``fields`` (its members as
        keywords: address, port, name, frames, backlog, max_frame, max_queued, limits, no_delay,
        flush_timeout). Returns a net_listener, the listener's id; raises OSError when it cannot."""
        if options is None:
            options = listener_options(**fields)
        elif fields:
            raise TypeError("net_engine.listen: give options or fields, not both")
        return net_listener(self._listen(options), self)

    def stop(self):
        """Close every connection (closed: engine_stopped) and listener, and join the I/O threads.
        The waker delivers what was queued, then ends."""
        self._stop()
        w = _net_wakers.get(self._address())
        if w is not None:
            w.stopped = True
            if w.thread is None or not w.thread.is_alive():  # nothing left to deliver
                del _net_wakers[self._address()]

    def attach_loop(self, loop):
        """Run this engine's listener callbacks on ``loop`` (an asyncio loop). Before the first
        callback is registered; a later call with another loop raises RuntimeError."""
        _net_waker_of(self).attach(loop)

    def detach_loop(self, timeout=5.0):
        """Stop the waker (its thread ends within ``timeout`` seconds); the callbacks stay
        registered for a later attach_loop."""
        _net_waker_of(self).close(timeout)
  %}
}

%extend cvc::net::net_batch {
  std::size_t __len__() const { return $self->events.size(); }
}

%extend cvc::net::net_counters {
  PyObject *_closed() const {
    PyObject *d = PyDict_New();
    if (d == nullptr)
      return nullptr;
    for (std::size_t i = 0; i < cvc::net::kCloseReasons; ++i) {
      PyObject *n = PyLong_FromUnsignedLongLong($self->closed[i]);
      if (n == nullptr ||
          PyDict_SetItemString(d, cvc::net::to_string(static_cast<cvc::net::close_reason>(i)), n) <
              0) {
        Py_XDECREF(n);
        Py_DECREF(d);
        return nullptr;
      }
      Py_DECREF(n);
    }
    return d;
  }
  %pythoncode %{
    closed = property(lambda self: self._closed(),
                      doc="Connections closed, by reason name (a dict).")
  %}
}

%inline %{
// A net_engine proxy over a host's engine, which Python never destroys.
PyObject *_net_engine_from_capsule(PyObject *capsule) {
  void *p = PyCapsule_GetPointer(capsule, "cvc.net.net_engine");
  if (p == nullptr)
    return nullptr;
  return SWIG_NewPointerObj(p, SWIGTYPE_p_cvc__net__net_engine, 0);
}

// net_batch.events: net_event copies, each with its frames as memoryviews (`frames`).
PyObject *_net_batch_events(PyObject *batch_proxy, PyObject *frame_type) {
  PyTypeObject *t = pycvc::net::frame_type_of(frame_type);
  if (t == nullptr)
    return nullptr;
  void *p = nullptr;
  if (!SWIG_IsOK(SWIG_ConvertPtr(batch_proxy, &p, SWIGTYPE_p_cvc__net__net_batch, 0)) || !p) {
    PyErr_SetString(PyExc_TypeError, "expected a net_batch");
    return nullptr;
  }
  const auto &batch = *static_cast<cvc::net::net_batch *>(p);
  PyObject *owner = pycvc::net::new_owner(batch);
  if (owner == nullptr)
    return nullptr;
  PyObject *list = PyList_New(0);
  for (std::size_t i = 0; list != nullptr && i < batch.events.size(); ++i) {
    auto *copy = new cvc::net::net_event(batch.events[i]);
    copy->frames.clear();
    PyObject *ev = SWIG_NewPointerObj(copy, SWIGTYPE_p_cvc__net__net_event, SWIG_POINTER_OWN);
    PyObject *frames = ev ? pycvc::net::frame_list(t, owner, batch.events[i]) : nullptr;
    if (frames == nullptr || PyObject_SetAttrString(ev, "frames", frames) < 0 ||
        PyList_Append(list, ev) < 0) {
      Py_CLEAR(list);
    }
    Py_XDECREF(frames);
    Py_XDECREF(ev);
  }
  Py_DECREF(owner);
  return list;
}

// The asyncio waker's view of a batch (pycvc::net::dispatch_lists).
PyObject *_net_batch_dispatch(PyObject *batch_proxy, PyObject *frame_type) {
  PyTypeObject *t = pycvc::net::frame_type_of(frame_type);
  if (t == nullptr)
    return nullptr;
  void *p = nullptr;
  if (!SWIG_IsOK(SWIG_ConvertPtr(batch_proxy, &p, SWIGTYPE_p_cvc__net__net_batch, 0)) || !p) {
    PyErr_SetString(PyExc_TypeError, "expected a net_batch");
    return nullptr;
  }
  return pycvc::net::dispatch_lists(*static_cast<cvc::net::net_batch *>(p), t,
                                    [](const cvc::net::conn_info &info) {
                                      return SWIG_NewPointerObj(new cvc::net::conn_info(info),
                                                                SWIGTYPE_p_cvc__net__conn_info,
                                                                SWIG_POINTER_OWN);
                                    });
}
%}

%clear std::uint8_t;
%clear std::uint16_t;

%pythoncode %{
net_batch.events = property(lambda self: _net_batch_events(self, _pycvc._net_frame),
                            doc="The batch's events: net_event copies, each with `frames`, a list of "
                                "memoryviews into the batch's buffers.")


def _net_keywords(cls):
    """cls(**fields): a default one with those members set."""
    plain = cls.__init__

    def __init__(self, *args, **fields):
        plain(self, *args)
        for name, value in fields.items():
            if not isinstance(getattr(cls, name, None), property):
                raise TypeError(f"{cls.__name__}: no member {name!r}")
            setattr(self, name, value)

    __init__.__doc__ = f"{cls.__name__}(**members): a default {cls.__name__} with those set."
    cls.__init__ = __init__


for _c in (engine_options, listener_options, inbound_limits, framing):
    _net_keywords(_c)
del _c


class net_listener(int):
    """A listener's id (an int, as net_engine.port() and stop_listening() take), and its asyncio
    callbacks, each a decorator: on_accept(conn, info) (may return a refusal reason),
    on_frames(batch) ([(conn, [memoryview, ...]), ...]), on_close(conn, reason[, message]),
    on_writable(conn) and on_error(message)."""

    def __new__(cls, ident, engine):
        self = super().__new__(cls, ident)
        self.engine = engine
        return self

    @property
    def port(self):
        """The bound port."""
        return self.engine.port(self)

    def stop(self):
        """Stop accepting (net_engine.stop_listening); its open connections stay."""
        self.engine.stop_listening(self)

    def _on(self, what, fn):
        _net_waker_of(self.engine).register(int(self), what, fn)
        return fn

    def on_accept(self, fn):
        return self._on("accept", fn)

    def on_frames(self, fn):
        return self._on("frames", fn)

    def on_close(self, fn):
        return self._on("close", fn)

    def on_writable(self, fn):
        return self._on("writable", fn)

    def on_error(self, fn):
        return self._on("error", fn)


def _net_arity(fn):
    import inspect

    try:
        params = inspect.signature(fn).parameters.values()
    except (TypeError, ValueError):
        return 2
    if any(p.kind == p.VAR_POSITIONAL for p in params):
        return 3
    return sum(1 for p in params if p.kind in (p.POSITIONAL_ONLY, p.POSITIONAL_OR_KEYWORD))


class _NetWaker:
    """One engine's waker: a thread that waits in net_engine.wait() without the GIL and hands each
    batch to the asyncio loop, ServerLink's shape. It takes the next batch once the loop has
    delivered this one, so what arrives while the loop is busy comes as one larger batch."""

    def __init__(self, engine):
        import threading

        self.engine = engine
        self.loop = None
        self.callbacks = {}  # listener id -> {what: fn}
        self.tasks = set()
        self.thread = None
        self.closing = threading.Event()
        self.stopped = False  # the engine's stop() has returned
        self.lock = threading.Lock()

    def attach(self, loop):
        with self.lock:
            if self.loop is not None and self.loop is not loop and self.thread is not None:
                raise RuntimeError("net_engine.attach_loop: the engine already runs on another loop")
            self.loop = loop
            if self.callbacks:
                self._start()

    def register(self, listener, what, fn):
        if not callable(fn):
            raise TypeError(f"net_listener.on_{what}: expected a callable")
        with self.lock:
            entry = (fn, _net_arity(fn)) if what == "close" else (fn, 0)
            self.callbacks.setdefault(listener, {})[what] = entry
            if self.loop is None:
                import asyncio

                try:
                    self.loop = asyncio.get_running_loop()
                except RuntimeError:
                    raise RuntimeError(
                        f"net_listener.on_{what}: no asyncio loop to run it on: call "
                        "engine.attach_loop(loop) first (pycvc.ariadne's host.net is attached)"
                    ) from None
            self._start()

    def _start(self):
        import threading

        if self.thread is not None and self.thread.is_alive():
            return
        self.closing.clear()
        self.thread = threading.Thread(target=self._wake, name="pycvc-net-waker", daemon=True)
        self.thread.start()

    def close(self, timeout=5.0):
        self.closing.set()
        t = self.thread
        if t is not None and t.is_alive():
            import threading

            if t is not threading.current_thread():
                t.join(timeout)
        self.thread = None

    def _wake(self):
        import threading

        batch = net_batch()
        while not self.closing.is_set():
            if not self.engine.wait(batch, 200):
                if self.stopped:  # stop() has posted all it will, and it has been delivered
                    if _net_wakers.get(self.engine._address()) is self:
                        del _net_wakers[self.engine._address()]
                    return
                continue
            taken = threading.Event()
            try:
                self.loop.call_soon_threadsafe(self._deliver, batch, taken)
            except RuntimeError:  # the loop is closed: nothing is left to tell
                return
            batch = net_batch()
            while not taken.wait(0.2):
                if self.closing.is_set() or self.loop.is_closed():
                    return

    def _call(self, fn, *args, then=None):
        try:
            r = fn(*args)
        except Exception:  # noqa: BLE001 - logged, never raised into the loop
            import logging

            logging.getLogger("pycvc.net").exception("%s raised", getattr(fn, "__name__", fn))
            return None
        if r is not None and hasattr(r, "__await__"):
            import asyncio

            task = asyncio.ensure_future(r, loop=self.loop)
            self.tasks.add(task)
            task.add_done_callback(self._task_done)
            if then is not None:
                task.add_done_callback(
                    lambda t: then(t.result()) if not t.cancelled() and t.exception() is None
                    else None)
            return None
        return r

    def _task_done(self, task):
        self.tasks.discard(task)
        if not task.cancelled() and task.exception() is not None:
            import logging

            logging.getLogger("pycvc.net").error("a callback raised", exc_info=task.exception())

    def _deliver(self, batch, taken):
        try:
            accepts, frames, writables, closes, errors = _net_batch_dispatch(batch, _pycvc._net_frame)
            cbs = self.callbacks
            refused = set()
            for listener, conn, info in accepts:
                fn = cbs.get(listener, {}).get("accept")
                if fn is None:
                    continue
                why = self._call(fn[0], conn, info,
                                 then=lambda why, c=conn: self.engine.close(c, False) if why
                                 else None)
                if why:
                    refused.add(conn)
                    self.engine.close(conn, False)
            for listener, entries in frames.items():
                fn = cbs.get(listener, {}).get("frames")
                if fn is None:
                    continue
                if refused:
                    entries = [e for e in entries if e[0] not in refused]
                if entries:
                    self._call(fn[0], entries)
            for listener, conn in writables:
                fn = cbs.get(listener, {}).get("writable")
                if fn is not None:
                    self._call(fn[0], conn)
            for listener, conn, reason, message in closes:
                fn = cbs.get(listener, {}).get("close")
                if fn is None:
                    continue
                if fn[1] >= 3:
                    self._call(fn[0], conn, reason, message)
                else:
                    self._call(fn[0], conn, reason)
            for listener, message in errors:
                fn = cbs.get(listener, {}).get("error")
                if fn is not None:
                    self._call(fn[0], message)
                else:
                    import logging

                    logging.getLogger("pycvc.net").warning("listener %d: %s", listener, message)
        finally:
            batch.clear()
            taken.set()


_net_wakers = {}  # engine address -> _NetWaker


def _net_waker_of(engine):
    key = engine._address()
    w = _net_wakers.get(key)
    if w is None:
        w = _net_wakers[key] = _NetWaker(engine)
    return w

%}
