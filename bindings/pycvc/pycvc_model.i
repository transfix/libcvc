// pycvc_model.i — SWIG surface for cvc::model (Phase-3 mesh/model value type).
//
// NOT a standalone module: this file is %include'd by pycvc.i (AFTER geometry and
// image are defined, since model::mesh holds a cvc::geometry and material holds a
// cvc::image) so `model` / `material` land in the `pycvc` module and
// `pycvc.load_model(path)` returns a pycvc.model. It relies on the DoubleVector /
// boost::uint64_t typemaps + the %exception block set up in pycvc.i, and on the
// already-wrapped cvc::geometry / cvc::image.
//
// SWIG APPROACH (documented per the phase spec):
//  * DIRECT WRAP of the real header (mirrors pycvc_image.i): %include model.h,
//    curated with %ignore for the members that don't marshal.
//  * The vectors model::meshes / model::materials are re-surfaced as PROPERTIES
//    (model.meshes / model.materials) backed by %template'd std::vector proxies
//    (MeshVector / MaterialVector) — indexable + len(). The raw members are
//    %ignore'd because their std::vector type isn't %template'd yet at the point
//    SWIG wraps the class body, so a %extend accessor placed AFTER the %template
//    is what yields the real sequence proxy.
//  * material's base_color / emissive are boost::array<double,N> and DON'T
//    marshal; base_color_texture is a by-value cvc::image member. All three are
//    %ignore'd and re-exposed via a %extend with a DIFFERENT name (a same-named
//    %extend is swallowed by the %ignore — the documented pycvc.i gotcha), then
//    aliased to the natural name in a %pythoncode block (the pixel_format_* alias
//    trick pycvc_image.i already uses). base_color()/emissive() return plain
//    tuples; base_color_texture() returns a pycvc.image COPY (empty if none).
//  * model::extents() returns the opaque bounding_box (as in pycvc.i, where every
//    bounding_box return is ignored), so it is re-exposed as a 6-tuple
//    (minx..maxz) the same way (extents_bbox %extend aliased to extents).
//  * model::mesh::geom is re-exposed as geometry() (COW copy) — the raw member is
//    %ignore'd and a distinct-named %extend is aliased back, same pattern.
//  * load_model(path) is a %inline free function wrapping cvc::read_model — it
//    takes ONLY a path (read_model needs no app; the returned geometries are bare
//    value types with a null context, which every read-only accessor the bindings
//    expose is fine with), so no app is threaded through.

%{
#include <cvc/model/model.h>
#include <cvc/model/model_file_io.h> // read_model (used by the %inline load_model below)
%}

// model::mesh is a NESTED struct; SWIG (4.x) ignores nested classes by default
// (Warning 325), which would leave MeshVector's elements opaque. flatnested lifts
// it to a normal wrapped proxy so model.meshes[i].geometry()/.material/.name work.
%feature("flatnested", "1") cvc::model::mesh;

// The vectors and the opaque bounding_box extents() are re-surfaced by %extend
// after the %template's below; %ignore the raw members so they don't wrap as
// opaque std::vector pointers.
%ignore cvc::model::meshes;
%ignore cvc::model::materials;
%ignore cvc::model::extents;
%ignore cvc::model::mesh::geom;

// material: boost::array + by-value image members don't marshal cleanly; exposed
// via the renamed-%extend trick below.
%ignore cvc::material::base_color;
%ignore cvc::material::emissive;
%ignore cvc::material::base_color_texture;

%include "cvc/model/model.h"

// Sequence proxies for the mesh/material vectors (indexable + len()). Must come
// AFTER the %include so cvc::model::mesh / cvc::material are fully declared.
%template(MeshVector) std::vector<cvc::model::mesh>;
%template(MaterialVector) std::vector<cvc::material>;

%extend cvc::model::mesh {
  // The mesh's geometry as a pycvc.geometry (a COW copy of the mesh's cvc::geometry).
  cvc::geometry mesh_geometry() const { return $self->geom; }
%pythoncode %{
    geometry = mesh_geometry
%}
}

%extend cvc::material {
  // base_color RGBA multiplier as a 4-tuple (the boost::array<double,4> is ignored).
  PyObject *base_color_rgba() const {
    return Py_BuildValue("(dddd)", $self->base_color[0], $self->base_color[1],
                         $self->base_color[2], $self->base_color[3]);
  }
  // emissive RGB as a 3-tuple (the boost::array<double,3> is ignored).
  PyObject *emissive_rgb() const {
    return Py_BuildValue("(ddd)", $self->emissive[0], $self->emissive[1], $self->emissive[2]);
  }
  // The LOADED base-color texture as a pycvc.image COPY (image.empty() if none /
  // unresolved). Zero-copy sharing isn't offered here — the material owns the
  // decoded pixels and a copy keeps the surface a plain value.
  cvc::image base_color_texture_image() const { return $self->base_color_texture; }
%pythoncode %{
    base_color = base_color_rgba
    emissive = emissive_rgb
    base_color_texture = base_color_texture_image
%}
}

%extend cvc::model {
  // meshes / materials as real sequence proxies (the raw members are ignored;
  // their std::vector type isn't templated at class-wrap time). Exposed as
  // PROPERTIES below so `model.meshes[i]` / `model.materials[i]` read naturally.
  std::vector<cvc::model::mesh> mesh_list() const { return $self->meshes; }
  std::vector<cvc::material> material_list() const { return $self->materials; }
  // extents() as a (minx, miny, minz, maxx, maxy, maxz) 6-tuple — the bounding_box
  // return is opaque here exactly as in pycvc.i.
  std::vector<double> extents_bbox() const {
    cvc::bounding_box b = $self->extents();
    return {b.minx, b.miny, b.minz, b.maxx, b.maxy, b.maxz};
  }
%pythoncode %{
    meshes = property(lambda self: self.mesh_list())
    materials = property(lambda self: self.material_list())
    extents = extents_bbox
%}
}

// pycvc.load_model(path) — the core deliverable. Wraps cvc::read_model (which
// dispatches by extension to the Assimp handler for obj/ply/stl/fbx/gltf/glb/...).
// No app is needed; an unsupported extension / missing handler raises the same
// cvc::exception the %exception block maps to a Python RuntimeError.
%inline %{
namespace pycvc {
cvc::model load_model(const std::string &path) { return cvc::read_model(path); }
} // namespace pycvc
%}
