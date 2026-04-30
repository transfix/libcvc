#include <volrover3/volrover3_app.h>

namespace volrover3
{
  cvc::app& app()
  {
    static cvc::app instance;
    return instance;
  }
}
