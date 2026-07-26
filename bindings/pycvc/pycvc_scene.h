// pycvc_scene.h — VTK render helpers + Python-prop bridge for pycvc_gl.
//
// NOT a facade: Python drives the scene through the directly-wrapped cvcGL
// objects (SceneGraph / GraphicsNode / GeometryNode / VolumeNode — see
// pycvc_gl.i). These are just plain utilities over a REAL SceneGraph so a
// standalone script / the console demo can SEE or CAPTURE a scene no host
// window is already rendering, plus a bridge for dropping a Python-built
// vtkProp into the scene. Generic — no domain knowledge.
#pragma once

#include <memory>
#include <string>

class SceneGraph; // cvcGL (global namespace)
class vtkProp;    // VTK (global namespace) — bridged via vtkPythonUtil typemaps

namespace cvc {
class app;
} // namespace cvc

namespace pycvc {

// ── Standalone render helpers over a REAL SceneGraph ─────────────────────────
// The embedded case (volrover3) never uses these — the host owns the render
// loop and Python only mutates the adopted SceneGraph.

// Render one offscreen frame of `sg` to a PNG at `path`. Pumps queued scene
// events + frames the camera first. Needs a GL context (offscreen is fine).
void render_png(SceneGraph &sg, const std::string &path, int width = 1024, int height = 768);

// Render one offscreen frame with an EXPLICIT camera (eye / focal-point / up +
// view angle + clip range) instead of auto-framing — for scripting a chase camera
// across a sequence of frames (e.g. capturing a drive to video). Offscreen is fine.
void render_png_camera(SceneGraph &sg, const std::string &path, int width, int height,
                       double eye_x, double eye_y, double eye_z, double focal_x, double focal_y,
                       double focal_z, double up_x, double up_y, double up_z,
                       double view_angle = 30.0, double clip_near = 1.0, double clip_far = 1e5);

// Open a blocking interactive window on `sg` (needs a display); returns when the
// window closes. Don't call this in the embedded case — the host owns the loop.
void show(SceneGraph &sg, const std::string &title = "pycvc_gl", int width = 1024,
          int height = 768);

// ── Python VTK prop bridge (free function; complements directors) ────────────
// Add a Python-built vtkProp (e.g. a vtkActor from vtkmodules) to `sg` as a
// named node with an explicit bounding box. The vtkProp* in-typemap unwraps the
// live Python VTK object into a C++ vtkProp* the scene renders directly. (With
// directors you can instead subclass GraphicsNode in Python and return the prop
// from getProp(); this is the quick imperative path for an already-built prop.)
// `parent` (default "" = the graphics root) attaches the prop as a CHILD of that
// named node, so it inherits the parent's transform (e.g. a building mesh as a
// child of the terrain node stays aligned to it and moves with it).
void add_prop(SceneGraph &sg, const std::string &name, vtkProp *prop, double minx = -1.0,
              double miny = -1.0, double minz = -1.0, double maxx = 1.0, double maxy = 1.0,
              double maxz = 1.0, const std::string &parent = "");

// Return a node's vtkProp back to Python as a live vtkmodules object (via the
// out-typemap). Null if `name` is absent or was not added via add_prop.
vtkProp *prop(SceneGraph &sg, const std::string &name);

} // namespace pycvc
