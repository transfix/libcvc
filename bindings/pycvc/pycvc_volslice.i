// pycvc_volslice.i — cvc::volslice view-aligned slice renderer value types.
//
// A SUB-interface %include'd into pycvc.i AFTER pycvc_volren.i (NOT its own
// %module): volslice shares volren's math/TF vocabulary — types.h does
// `using volren::{mat4, vec3d, rgba_f, transfer_function, transfer_point,
// baked_transfer_function};` — so those types must already be wrapped when this
// is parsed, and the std::array<double,16> typemap (mat4.m) defined in
// pycvc_volren.i must still be in scope. This file therefore re-%includes NO
// cvc/volren header (re-including would redefine in the same flat module).
//
// The drop-in scene node cvc::gl::VolSliceNode (needs the GeometryNode/VTK
// base) is bound separately in pycvc_gl.i, which %imports these value types.
//
// No new typemaps: every wrapped volslice value struct uses vec3d (a wrapped
// {x,y,z} proxy) and scalars/enums — no std::array member of its own — and
// render_settings has no std::vector member, so no add_*/count/at collection
// %extend is needed (unlike volren's settings).

%{
#include <cvc/volslice/types.h>
#include <cvc/volslice/slicer.h>
#include <cvc/volslice/settings.h>
%}

// ── types.h: the blend_mode / interpolation enum classes ────────────────────
// SWIG flattens namespaces (pycvc has no `nspace`), so enum-class values wrap as
// flat int constants: pycvc.interpolation_linear / _nearest,
// pycvc.blend_mode_alpha / _additive (the same treatment volren's enums get).
//
// The whole `defaults`/`limits` constant namespaces are ignored: Python needs
// none of them, and defaults::lut_size (=256) would otherwise COLLIDE with the
// already-wrapped cvc::volren::defaults::lut_size (=1024) — both flatten to the
// module identifier `lut_size` — emitting a SWIG 302 redefinition. Ignoring the
// namespaces also sidesteps the two enum-class-typed constexprs (defaults::blend
// / defaults::filter), which can choke constant emission.
%ignore cvc::volslice::defaults;
%ignore cvc::volslice::limits;
%include "cvc/volslice/types.h"

// ── slicer.h: box3d, slice_params, slice_geometry + the two free functions ──
// slice_geometry's four std::vector<float>/<uint32_t> members do not marshal
// (no FloatVector/UInt32Vector %template in pycvc.i); expose only its scalar
// accessors (planes()/vertices()/empty()/plane_spacing). view_plane_normal /
// compute_slices take a mat4 PROXY (build mat4(); set .m from a 16-list via the
// volren varin typemap) and return vec3d / slice_geometry.
%ignore cvc::volslice::slice_geometry::positions;
%ignore cvc::volslice::slice_geometry::texcoords;
%ignore cvc::volslice::slice_geometry::fan_offset;
%ignore cvc::volslice::slice_geometry::fan_count;
%include "cvc/volslice/slicer.h"
// Read the computed triangle-fan geometry out to flat lists, so an offline /
// twin / regression path can consume compute_slices()'s vertices (not just count
// them). The raw std::vector<float>/<uint32_t> members stay %ignore'd (no
// FloatVector/UInt32Vector %template, and a same-named %extend would be swallowed
// by the %ignore); these accessors have DISTINCT names. positions/texcoords widen
// float->double (DoubleVector); the fan offsets/counts are uint32 indices ->
// unsigned long (IndexVector), lossless. (A zero-copy numpy view is a deferred
// hot-path follow-up; the copy is fine for the offline analysis path.)
%extend cvc::volslice::slice_geometry {
  std::vector<double> get_positions() const {
    return std::vector<double>($self->positions.begin(), $self->positions.end());
  }
  std::vector<double> get_texcoords() const {
    return std::vector<double>($self->texcoords.begin(), $self->texcoords.end());
  }
  std::vector<unsigned long> fan_offsets() const {
    return std::vector<unsigned long>($self->fan_offset.begin(), $self->fan_offset.end());
  }
  std::vector<unsigned long> fan_counts() const {
    return std::vector<unsigned long>($self->fan_count.begin(), $self->fan_count.end());
  }
}

// ── settings.h: render_settings ─────────────────────────────────────────────
// slices (a by-value slice_params, wrapped above) + filter (interpolation enum
// -> int const) + opacity_correction (bool) + tf (the shared, already-wrapped
// volren::transfer_function) + tf_auto_domain (bool) + window_min/window_max
// (double). No reshape, no collection %extend: it crosses as a plain proxy whose
// members read/write directly (rs.slices.max_planes = 500; rs.filter = ...).
//
// RENAMED to volslice_render_settings: the flat module already has a
// cvc::volren::render_settings, and both would flatten to `render_settings`
// (a SWIG "multiply defined" ERROR). VolSliceNode::config()/setConfig() marshal
// this renamed proxy; a fresh one is pycvc.volslice_render_settings().
%rename(volslice_render_settings) cvc::volslice::render_settings;
%include "cvc/volslice/settings.h"
