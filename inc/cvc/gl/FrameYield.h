/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#ifndef __CVC_GL_FRAME_YIELD_H__
#define __CVC_GL_FRAME_YIELD_H__

// Feature test for SceneRenderer / ViewportManager::setFrameYield (or, in C++20,
// `requires { v.setFrameYield(V::FrameYield::App); }`).
#define CVC_GL_HAS_FRAME_YIELD 1
// ... and for lockFrameYield() / frameYieldLocked() (`requires { v.lockFrameYield(); }`).
#define CVC_GL_HAS_FRAME_YIELD_LOCK 1

namespace cvc {
namespace gl {

// Who yields to the browser once per frame in a WebAssembly build -- VTK inside
// every render, or the app's own loop. Namespace scope so SWIG wraps it as-is;
// SceneRenderer and ViewportManager also name it as a nested alias.
//
//   Vtk (default) VTK 9.5's vtkWebAssemblyOpenGLRenderWindow::Frame() calls
//                 emscripten_sleep(0) at the end of every render (when the build
//                 has Asyncify), so each render() is one trip through the
//                 browser's event loop -- and a frame that also yields in its own
//                 loop pays the clamped trip twice.
//   App           the app's loop calls emscripten_sleep(0) once per frame on EVERY
//                 path that renders, so cvcGL turns VTK's in-render sleep off
//                 (SetDoubleBuffer(0) on the render window). Only valid with that
//                 contract kept: a render loop that never yields would never let
//                 the browser paint. A watchdog catches that case (see
//                 ViewportManager::setFrameYield) and restores VTK's yield.
//                 An app linked with -sASYNCIFY_IGNORE_INDIRECT=1 must LOCK App
//                 (ViewportManager::lockFrameYield, or cvcgl_wasm_app's
//                 Module.cvcglFrameYieldLocked): there VTK's yield, reached
//                 through a virtual call, traps, so neither ?frameyield=vtk nor
//                 the watchdog may bring it back.
//
// Native, and wasm without Asyncify, have no in-render yield: App is a no-op
// there. VTK 9.6 removed the in-render yield entirely, so with it every wasm loop
// must yield by itself in either mode.
enum class FrameYield { Vtk, App };

} // namespace gl
} // namespace cvc

#endif // __CVC_GL_FRAME_YIELD_H__
