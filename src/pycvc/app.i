/*
  app.i — the cvc::app module context.

  Every heavy libcvc operation takes an `app &ctx` first argument
  (inc/cvc/core/app.h).  Python callers never see it: pycvc lazily
  constructs one process-wide cvc::app on first use and threads it through
  internally (volume construction, sdf, ...).  The cvc::app class itself is
  intentionally NOT wrapped — its header is boost-heavy and none of its API
  is needed for the field-pipeline surface.
*/

%{
namespace {
// Lazily-constructed, deliberately leaked process singleton.  Leaking (vs a
// function-local static object) avoids static-destruction-order problems
// with the app's boost::thread members at interpreter shutdown.
cvc::app &pycvc_ctx() {
  static cvc::app *instance = new cvc::app();
  return *instance;
}
} // namespace
%}
