#ifndef __CVC_IO_HANDLERS_H__
#define __CVC_IO_HANDLERS_H__

#include <cvc/namespace.h>

// Registration functions for I/O handlers.
// Called by app::registerDefaultHandlers() instead of static_init objects
// to avoid static initialization order deadlocks.

namespace CVC_NAMESPACE {
class app;

// Volume file I/O handlers
void register_rawiv_io(app &ctx);
void register_rawv_io(app &ctx);
void register_mrc_io(app &ctx);
void register_vtk_io(app &ctx);
void register_spider_io(app &ctx);
void register_null_io(app &ctx);

// Conditionally compiled volume handlers
#ifdef CVC_USING_HDF5
void register_hdf5_io(app &ctx);
#endif

// Geometry file I/O handlers
void register_bunny_io(app &ctx);
void register_off_io(app &ctx);
void register_cvcraw_io(app &ctx);
} // namespace CVC_NAMESPACE

#endif
