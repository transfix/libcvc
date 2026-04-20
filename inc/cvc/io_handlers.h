#ifndef __CVC_IO_HANDLERS_H__
#define __CVC_IO_HANDLERS_H__

#include <cvc/namespace.h>

// Registration functions for I/O handlers.
// Called by app::registerDefaultHandlers() instead of static_init objects
// to avoid static initialization order deadlocks.

namespace CVC_NAMESPACE
{
  // Volume file I/O handlers
  void register_rawiv_io();
  void register_rawv_io();
  void register_mrc_io();
  void register_vtk_io();
  void register_spider_io();
  void register_null_io();

  // Conditionally compiled volume handlers
#ifdef CVC_USING_HDF5
  void register_hdf5_io();
#endif

  // Geometry file I/O handlers
  void register_bunny_io();
  void register_off_io();
  void register_cvcraw_io();
}

#endif
