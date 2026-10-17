// pycvc_value.h — the one conversion between state_exec values (cvc::state_exec::value_t) and
// Python, used by every pycvc API that takes or returns a value (pycvc.parse, encode_value,
// Exec's registered functions, ...). docs/PYCVC_STATE_EXEC_PARITY.md has the full table.
//
// value_t -> Python (and back):
// - nil, bool, int64, double -> None, bool, int, float.
// - string -> str. The text must be valid UTF-8; invalid text raises UnicodeDecodeError.
// - bytes_value -> bytes (bytes or bytearray coming in).
// - list -> list (list or tuple coming in).
// - dict -> dict (keys must be str coming in; others raise TypeError).
// - symbol -> pycvc.symbol.
// - closure, native_fn, data_object, generator -> pycvc.value, an opaque handle that converts
//   back to the same value, so it can be passed into the DSL again.
//
// Strict: an int outside 64 bits raises OverflowError, any other Python type raises TypeError, and
// a value nested deeper than 1000 levels (or a cyclic one) raises ValueError. Defined in the SWIG
// wrapper (pycvc.i), which owns the pycvc.symbol / pycvc.value types. Call with the GIL held.
#pragma once

#include <cvc/state/state_exec/types.h>
#include <span>

struct _object;
typedef _object PyObject;

namespace pycvc {

/// A new reference, or nullptr with a Python exception set.
PyObject *value_to_py(const cvc::state_exec::value_t &v);

/// A new tuple of the converted values (a DSL call's arguments), or nullptr with a Python
/// exception set.
PyObject *values_to_py_tuple(std::span<const cvc::state_exec::value_t> values);

/// Convert `o` into `out`. Returns false, with a Python exception set, when it can't.
bool py_to_value(PyObject *o, cvc::state_exec::value_t &out);

} // namespace pycvc
