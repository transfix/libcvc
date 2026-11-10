// pycvc_contour_tree.i — the contour tree of a volume (inc/cvc/volume/contour_tree.h): the tree,
// compute_contour_tree on a volume, and its options with their parameter table
// (inc/cvc/volume/contour_tree_params.h), the first params struct PYCVC_PARAM_TABLE
// (pycvc_params.i) gives to Python:
//
//   opts = pycvc.contour_tree_options(max_dim=64, quantize_u8=True)
//   tree = pycvc.compute_contour_tree(vol, opts)       # the GIL released meanwhile
//   tree.nodes[0].value, tree.arcs[0].hi, tree.components_at(0.5), tree.dims_list()
//
// pycvc.contour_tree_job(vol, opts) is the same as a resumable job (pycvc_jobs.i). The builders,
// the simplifier, the layout and the raw-buffer and mesh overloads are not wrapped.

%{
#include <cvc/volume/contour_tree.h>
#include <cvc/volume/contour_tree_params.h>
%}

// The tree's nested types lift to the module as contour_tree_node, contour_tree_arc and
// contour_tree_node_kind.
%feature("flatnested") cvc::contour_tree::node;
%feature("flatnested") cvc::contour_tree::arc;
%feature("flatnested") cvc::contour_tree::node_kind;
%rename(contour_tree_node) cvc::contour_tree::node;
%rename(contour_tree_arc) cvc::contour_tree::arc;
%rename(contour_tree_node_kind) cvc::contour_tree::node_kind;
%ignore cvc::contour_tree::dims; // a std::array: dims_list() below
%ignore cvc::contour_tree::arcs_at;
%extend cvc::contour_tree {
  // {nx, ny, nz} of the grid it was built on (the resampled one, with max_dim).
  std::vector<unsigned long> dims_list() const {
    return {static_cast<unsigned long>($self->dims[0]), static_cast<unsigned long>($self->dims[1]),
            static_cast<unsigned long>($self->dims[2])};
  }
}

// The options' callback is not a parameter. Only the volume overload of compute_contour_tree is
// wrapped: the raw-buffer and mesh ones, the builders (pycvc_jobs.i's job runs one), the
// simplifier and the layout are not.
%ignore cvc::contour_tree_options::progress;
%ignore cvc::contour_tree_error;
%rename("$ignore", fullname=1) cvc::compute_contour_tree(const float *, uint64, uint64, uint64,
                                                         const contour_tree_options &);
%rename("$ignore", fullname=1) cvc::compute_contour_tree(const float *, uint64, uint64, uint64);
%rename("$ignore", fullname=1) cvc::compute_contour_tree(const contour_mesh &,
                                                         const contour_tree_options &);
%rename("$ignore", fullname=1) cvc::compute_contour_tree(const contour_mesh &);
%ignore cvc::simplify_contour_tree;
%ignore cvc::simplify_contour_tree_to;
%ignore cvc::layout_contour_tree;
%ignore cvc::contour_tree_builder;
%ignore cvc::contour_tree_simplifier;
%ignore cvc::mesh_contour_tree_builder;
%ignore cvc::contour_tree_builder::operator=;
%ignore cvc::contour_tree_simplifier::operator=;
%ignore cvc::mesh_contour_tree_builder::operator=;
// A tree of a big volume takes a while: other Python threads run meanwhile.
%exception cvc::compute_contour_tree {
  try {
    pycvc::gil_release pycvc_nogil;
    $action
  } catch (const std::exception &e) {
    SWIG_exception(SWIG_RuntimeError, e.what());
  } catch (const cvc::exception &e) {
    SWIG_exception(SWIG_RuntimeError, e.what());
  }
}
%include "cvc/volume/contour_tree.h"

%template(contour_tree_node_vector) std::vector<cvc::contour_tree::node>;
%template(contour_tree_arc_vector) std::vector<cvc::contour_tree::arc>;

PYCVC_PARAM_TABLE(contour_tree_options, cvc::contour_tree_options)
