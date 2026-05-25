#ifndef __CVC_IO_HANDLERS_H__
#define __CVC_IO_HANDLERS_H__

#include <cvc/namespace.h>

// Registration functions for I/O handlers.
// Called by app::registerDefaultHandlers() instead of static_init objects
// to avoid static initialization order deadlocks.

namespace cvc {
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

// No-app overloads of the geometry registrations. The geometry handlers
// register themselves into a global handler map and do not depend on
// any cvc::app state.
void register_bunny_io();
void register_off_io();
void register_cvcraw_io();

// Convenience entry point that calls each register_*_io() above. Used
// by geometry_file_io::get_handlers() to populate the handler map on
// first access without requiring a cvc::app instance to exist.
void register_default_geometry_handlers();
} // namespace cvc

#endif
