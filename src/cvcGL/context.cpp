#include <cvc/core/app.h>
#include <cvc/gl/context.h>

namespace cvc {
namespace gl {
cvc::app &context() {
  static cvc::app app;
  return app;
}
} // namespace gl
} // namespace cvc
