// pycvc_context.h — the injected-app substrate (Phase 0 of the rearchitecture).
//
// pycvc no longer owns a private process singleton for its cvc context. Instead
// every facade routes through ONE module-wide std::shared_ptr<cvc::app>:
//
//   * A host (VolRover's embedded interpreter) calls attach() with the app it
//     already runs under. Python + host then share one app / context / state
//     tree (cvc::state::instance(app) is per-app, so one app == one tree).
//   * With no host attached, ctx() lazily creates a single standalone app so
//     `import pycvc` works on its own.
//   * detach() drops the module's reference; the next ctx() lazily makes a
//     fresh standalone app, isolating subsequent state from the old tree.
//
// cvc::app is non-copyable / non-movable and heavyweight, so it is only ever
// shared via std::shared_ptr<cvc::app> (std, NOT boost::shared_ptr — app uses
// std for handles like this). cvc::app is forward-declared here so this header
// stays lightweight; the .cpp includes <cvc/core/app.h>.
#pragma once

#include <memory>
#include <string>

namespace cvc {
class app;
}

namespace pycvc {

// The module's shared cvc::app. Returns *handle, lazily creating a standalone
// std::make_shared<cvc::app>() the first time it is called with a null handle.
cvc::app &ctx();

// Factory for a fresh app, handed back to Python as a std::shared_ptr<cvc::app>.
// Used by tests (and, later, hosts) to simulate "the host's app".
std::shared_ptr<cvc::app> make_app();

// Install `handle` as THE module app. All facades now bind to its state tree.
void attach(std::shared_ptr<cvc::app> handle);

// Drop the module's app reference. The next ctx() lazily creates a fresh
// standalone app, so state written before detach() is not visible after.
void detach();

// ── State bridge (Phase 0 proof of shared context) ──────────────────
// Free functions over cvc::state::instance(app). The plain forms act on the
// module app (ctx()); the *_on forms act on a GIVEN handle so a test can play
// "the host" and observe the SAME tree the module writes to.
//
// set navigates/creates dotted-path nodes (state::operator()); get reads
// without creating and throws std::out_of_range on a missing path.
void state_set(const std::string &path, const std::string &value);
std::string state_get(const std::string &path);
void state_set_on(std::shared_ptr<cvc::app> handle, const std::string &path,
                  const std::string &value);
std::string state_get_on(std::shared_ptr<cvc::app> handle, const std::string &path);

} // namespace pycvc
