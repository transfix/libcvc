// X11's <X11/X.h> defines object-like macros named like ordinary identifiers -- notably
// `None` (0L) -- so a public header that declares anything called `None` stops compiling
// in a translation unit that includes an X11 header first. This TU includes <X11/X.h>
// FIRST (where X11 is available) and then the ariadne widget header, whose BorderShow enum
// used to have a `None` enumerator. Platforms without X11 build it without the macro.
#if __has_include(<X11/X.h>)
#include <X11/X.h>
#endif

#include <cvc/ariadne/widget.h>
#include <gtest/gtest.h>

TEST(X11MacroHeaders, BorderShowDefaultsToOff) {
#ifdef None
  static_assert(None == 0L, "X11's None macro is in effect for this translation unit");
#endif
  cvc::ariadne::Layout layout;
  EXPECT_EQ(layout.borders, cvc::ariadne::BorderShow::Off);
}
