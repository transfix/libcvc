// pycvc_image.i — SWIG surface for cvc::image (Phase-1 raster image value type).
//
// NOT a standalone module: this file is %include'd by pycvc.i so `image` lands in
// the `pycvc` module (pycvc.image, pycvc.image.RGBA, ...) and pycvc_gl can %import
// it for node.set_texture(image). It relies on the ArrayView typemap + the
// pycvc_owner capsule dtor defined in pycvc.i, and on numpy already imported there.
//
// DIRECT WRAP (mirrors geometry/volume): the real cvc::image is %include'd,
// curated with %ignore for the raw-pointer ctor and the boost-typed
// data()/storage()/registry members, and given a thin %extend surface — a
// zero-copy (H,W,C) numpy() view over the image buffer (dtype from data_type,
// pinned by a keep-alive owner so the buffer outlives the view) + a from_numpy()
// copy constructor, plus flat GRAY/RGB/RGBA + u8/u16/f32 enum aliases so the
// ctor reads pycvc.image(w, h, pycvc.image.RGBA, pycvc.image.u8).

%{
#include <cvc/image/image.h>
%}

// The file-I/O registry (boost::shared_ptr handlers, std::list<string> exts) and
// the free read/write functions don't marshal / are redundant with image.load /
// image.save — keep them off the Python surface.
%ignore cvc::image_file_io;
%ignore cvc::read_image;
%ignore cvc::write_image;
// The interleaved-source ctor takes a const void* (a raw C pointer Python can't
// supply); image.from_numpy() replaces it.
%ignore cvc::image::image(int, int, cvc::image::pixel_format, cvc::image::data_type, const void *);
// Raw pixel access: data() detaches (copy-on-write) and hands back a bare
// pointer; storage() returns a boost::shared_array. Both are opaque in Python —
// numpy() is the supported buffer surface (it pins storage() internally).
%ignore cvc::image::data;
%ignore cvc::image::storage;

%include "cvc/image/image.h"

%extend cvc::image {
  // Zero-copy (H, W, C) numpy view over the image's interleaved buffer. dtype
  // follows data_type (u8->uint8, u16->uint16, f32->float32); writable, so a
  // numpy edit writes the same bytes a cvcGL vtkTexture (from the zero-copy
  // node.set_texture) samples — the live-edit path, no re-copy. The owner pins
  // the exact buffer via storage() (non-detaching), bridged into ArrayView's
  // std::shared_ptr<void> by a keep-alive deleter that captures the boost
  // shared_array — so the buffer outlives the view even across a later
  // copy-on-write detach of the image (a valid, decoupled snapshot).
  pycvc::ArrayView numpy() {
    pycvc::ArrayView v;
    switch ($self->type()) {
    case cvc::image::data_type::u8:
      v.dtype = pycvc::DType::UInt8;
      break;
    case cvc::image::data_type::u16:
      v.dtype = pycvc::DType::UInt16;
      break;
    case cvc::image::data_type::f32:
      v.dtype = pycvc::DType::Float32;
      break;
    }
    v.writable = true;
    v.shape = {static_cast<long>($self->height()), static_cast<long>($self->width()),
               static_cast<long>($self->channels())};
    boost::shared_array<unsigned char> store = $self->storage();
    v.data = store.get();
    v.owner = std::shared_ptr<void>(store.get(), [store](void*) { /* keep-alive */ });
    return v;
  }

  // Build a NEW image by COPYING a numpy array. Accepts a (H,W) or (H,W,C)
  // C-contiguous uint8/uint16/float32 array; channels 1..4 map to
  // GRAY/GRAY_ALPHA/RGB/RGBA. (This copies — the zero-copy path is numpy() on an
  // existing image.)
  static cvc::image from_numpy(PyObject* arr) {
    if (!PyArray_Check(arr))
      throw std::invalid_argument("image.from_numpy: argument is not a numpy array");
    PyArrayObject* a = PyArray_GETCONTIGUOUS(reinterpret_cast<PyArrayObject*>(arr));
    if (!a)
      throw std::invalid_argument("image.from_numpy: could not make the array C-contiguous");
    int nd = PyArray_NDIM(a);
    if (nd != 2 && nd != 3) {
      Py_DECREF(a);
      throw std::invalid_argument("image.from_numpy: expected a (H,W) or (H,W,C) array");
    }
    const npy_intp* dims = PyArray_DIMS(a);
    const int h = static_cast<int>(dims[0]);
    const int w = static_cast<int>(dims[1]);
    const int c = (nd == 3) ? static_cast<int>(dims[2]) : 1;
    cvc::image::data_type dt;
    switch (PyArray_TYPE(a)) {
    case NPY_UINT8:
      dt = cvc::image::data_type::u8;
      break;
    case NPY_UINT16:
      dt = cvc::image::data_type::u16;
      break;
    case NPY_FLOAT32:
      dt = cvc::image::data_type::f32;
      break;
    default:
      Py_DECREF(a);
      throw std::invalid_argument("image.from_numpy: dtype must be uint8, uint16, or float32");
    }
    cvc::image::pixel_format fmt;
    switch (c) {
    case 1:
      fmt = cvc::image::pixel_format::GRAY;
      break;
    case 2:
      fmt = cvc::image::pixel_format::GRAY_ALPHA;
      break;
    case 3:
      fmt = cvc::image::pixel_format::RGB;
      break;
    case 4:
      fmt = cvc::image::pixel_format::RGBA;
      break;
    default:
      Py_DECREF(a);
      throw std::invalid_argument("image.from_numpy: channels (last axis) must be 1..4");
    }
    cvc::image img(w, h, fmt, dt, PyArray_DATA(a)); // copies the interleaved bytes
    Py_DECREF(a);
    return img;
  }

  // Flat enum aliases so pycvc.image.RGBA / pycvc.image.u8 read naturally (SWIG
  // names the nested `enum class` members pixel_format_* / data_type_*; these
  // class-body aliases reference those, defined just above). The ctor accepts
  // them: pycvc.image(w, h, pycvc.image.RGBA, pycvc.image.u8).
%pythoncode %{
    GRAY = pixel_format_GRAY
    GRAY_ALPHA = pixel_format_GRAY_ALPHA
    RGB = pixel_format_RGB
    RGBA = pixel_format_RGBA
    u8 = data_type_u8
    u16 = data_type_u16
    f32 = data_type_f32
%}
}
