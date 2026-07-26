// pycvc_buffer.h — a Python-free description of a contiguous C++ buffer.
//
// The SWIG layer turns an ArrayView into a **zero-copy** numpy array
// (numpy views the C++ memory directly, no data copy) whose `base` is a
// capsule owning a std::shared_ptr to the underlying C++ storage. numpy
// therefore holds the smart pointer: the C++ buffer is freed only when the
// last numpy view (and the owning facade object) are gone. This is the
// "blazing fast and correct" path for large meshes / scalar fields.
#pragma once

#include <memory>
#include <vector>

namespace pycvc {

enum class DType { Float64, Float32, UInt64, UInt8, UInt16 };

struct ArrayView {
  const void *data = nullptr;   // contiguous, row-major buffer
  std::vector<long> shape;      // numpy shape
  DType dtype = DType::Float64; // element type
  bool writable = false;        // may numpy mutate the C++ memory in place?
  std::shared_ptr<void> owner;  // keeps the C++ storage alive for the view
};

} // namespace pycvc
