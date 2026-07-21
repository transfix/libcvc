// cvc/gl/context.h — process-wide cvc::app context for cvcGL.
//
// cvcGL is a generic graphics library; it holds its own app context (thread
// pool + state root) rather than depending on any application's accessor.
#pragma once

namespace cvc {
class app;
namespace gl {
// The shared cvc::app that cvcGL's scene graph runs under.
cvc::app &context();
} // namespace gl
} // namespace cvc
