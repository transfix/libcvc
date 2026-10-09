// cvcGL apps on Linux pull X11 in through VTK's X11 render window, which includes
// <X11/Xlib.h> after VTK's own headers -- so cvc/gl headers can be parsed with X11's
// object-like macros (None, Success, ...) already defined. Mirror that order: VTK headers
// first (X11 before them breaks VTK's own vtksys/Status.hxx on `Success`), then X11, then
// the cvc/gl header. Switch's "no child" constant used to be named `None`.
#include <vtkMatrix4x4.h>
#include <vtkSmartPointer.h>

#if __has_include(<X11/X.h>)
#include <X11/X.h>
#endif

#include <cstdio>
#include <cvc/gl/nodes.h>

int main() {
#ifdef None
  static_assert(None == 0L, "X11's None macro is in effect for this translation unit");
#endif
  static_assert(cvc::gl::Switch::Off == -1, "Switch::Off traverses no child");
  static_assert(cvc::gl::Switch::All == -3, "Switch::All traverses every child");
  std::puts("cvcgl_x11_macros: ok");
  return 0;
}
