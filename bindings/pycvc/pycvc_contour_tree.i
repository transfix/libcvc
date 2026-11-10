// pycvc_contour_tree.i — cvc::compute_contour_tree's options (inc/cvc/volume/contour_tree.h),
// with their parameter table (inc/cvc/volume/contour_tree_params.h): the first params struct
// given to Python by PYCVC_PARAM_TABLE (pycvc_params.i), so
//
//   opts = pycvc.contour_tree_options(max_dim=64, quantize_u8=True)
//   opts.validate(), opts.to_state(node), pycvc.contour_tree_options.from_state(node)
//
// The rest of the header (the tree, the builders, the one-shot functions) is not wrapped here.

%{
#include <cvc/volume/contour_tree.h>
#include <cvc/volume/contour_tree_params.h>
%}

// The options only: the callback is not a parameter, and the functions and builders are not
// wrapped.
%ignore cvc::contour_tree_options::progress;
%ignore cvc::contour_tree_error;
%ignore cvc::contour_tree;
%ignore cvc::contour_tree::node; // (SWIG still visits an ignored class's nested types)
%ignore cvc::contour_tree::arc;
%ignore cvc::compute_contour_tree;
%ignore cvc::simplify_contour_tree;
%ignore cvc::simplify_contour_tree_to;
%ignore cvc::layout_contour_tree;
%ignore cvc::contour_tree_builder;
%ignore cvc::contour_tree_simplifier;
%ignore cvc::mesh_contour_tree_builder;
%ignore cvc::contour_tree_builder::operator=;
%ignore cvc::contour_tree_simplifier::operator=;
%ignore cvc::mesh_contour_tree_builder::operator=;
%include "cvc/volume/contour_tree.h"

PYCVC_PARAM_TABLE(contour_tree_options, cvc::contour_tree_options)
