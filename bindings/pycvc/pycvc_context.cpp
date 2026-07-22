// pycvc_context.cpp — implementation of the injected-app substrate.
#include "pycvc_context.h"

#include <cvc/core/app.h>
#include <cvc/core/state.h>
#include <mutex>
#include <stdexcept>
#include <utility>

namespace pycvc {

namespace {

// The one module-wide app handle. Guarded so attach()/detach()/ctx() can be
// called from different threads without racing on the pointer itself. (The app
// it points at is independently thread-safe; this mutex only guards the swap.)
std::mutex g_handleMutex;
std::shared_ptr<cvc::app> &handle() {
  static std::shared_ptr<cvc::app> h;
  return h;
}

// set/get against a specific app's root state tree.
void set_on(cvc::app &c, const std::string &path, const std::string &value) {
  // operator()(path) creates child nodes as needed (same as the state-set
  // intrinsic); value() fires valueChanged only on an actual change.
  cvc::state::instance(c)(path).value(value);
}

std::string get_on(cvc::app &c, const std::string &path) {
  // findDescendant navigates without creating; nullptr means the path is absent.
  cvc::state *node = cvc::state::instance(c).findDescendant(path);
  if (!node)
    throw std::out_of_range("pycvc.state_get: no node at path: " + path);
  return node->value();
}

} // namespace

cvc::app &ctx() {
  std::lock_guard<std::mutex> lk(g_handleMutex);
  std::shared_ptr<cvc::app> &h = handle();
  if (!h)
    h = std::make_shared<cvc::app>(); // lazy standalone app
  return *h;
}

std::shared_ptr<cvc::app> make_app() { return std::make_shared<cvc::app>(); }

void attach(std::shared_ptr<cvc::app> h) {
  std::lock_guard<std::mutex> lk(g_handleMutex);
  handle() = std::move(h);
}

void detach() {
  std::lock_guard<std::mutex> lk(g_handleMutex);
  handle().reset();
}

void state_set(const std::string &path, const std::string &value) { set_on(ctx(), path, value); }

std::string state_get(const std::string &path) { return get_on(ctx(), path); }

void state_set_on(std::shared_ptr<cvc::app> h, const std::string &path, const std::string &value) {
  if (!h)
    throw std::invalid_argument("pycvc.state_set_on: null app handle");
  set_on(*h, path, value);
}

std::string state_get_on(std::shared_ptr<cvc::app> h, const std::string &path) {
  if (!h)
    throw std::invalid_argument("pycvc.state_get_on: null app handle");
  return get_on(*h, path);
}

} // namespace pycvc
