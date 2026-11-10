#ifndef __CVC_IO_HANDLERS_H__
#define __CVC_IO_HANDLERS_H__

#include <cvc/core/namespace.h>

// Registration functions for the built-in file handlers. Each adds its handler to the given app's
// own table (app::volume_handlers(), app::geometry_handlers()); app::registerDefaultHandlers()
// calls the register_default_* ones, so every app starts with one of each. There is no app-free
// form: a handler table belongs to an app (VOLROVER-LIBRARY-PORTS-ROADMAP.md 1.3).

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

// The Assimp geometry flattening handler (model/assimp_io.cpp), so read_geometry(ctx, "foo.obj")
// works. Registers nothing in a build without Assimp.
void register_assimp_geometry_io(app &ctx);

// Each kind's built-in handlers, in the order the readers try them. (The model and image ones are
// register_default_model_handlers, model_file_io.h, and register_default_image_handlers, image.h.)
void register_default_volume_handlers(app &ctx);
void register_default_geometry_handlers(app &ctx);
} // namespace cvc

#endif
