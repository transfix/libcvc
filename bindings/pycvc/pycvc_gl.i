// pycvc_gl.i — SWIG module for the cvcGL scene graph (Scene facade).
// %import pulls in pycvc's Geometry/Volume types without re-wrapping them.
%module pycvc_gl

%{
#include <cvc/core/exception.h>  // the %import'd %exception block catches cvc::exception
#include "pycvc_scene.h"
%}

// pycvc.i's %exception (applied via %import) references SWIG_exception, so
// exception.i must be included here too.
%include <exception.i>
%include <std_string.i>
%include <std_vector.i>
%import "pycvc.i"

%include "pycvc_scene.h"
