// Ariadne — who a program runs as. See inc/cvc/ariadne/principal.h.

#include <cvc/ariadne/principal.h>
#include <cvc/core/app.h>
#include <cvc/net/credentials.h> // CredentialStore::canonical_principal
#include <cvc/state/state.h>     // cvc::state::SEPARATOR
#include <map>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

namespace cvc {
namespace ariadne {

namespace {

// An app's principals by scope, and the hosts' checks.
struct principal_slot {
  static constexpr const char *app_slot = "cvc.ariadne.principals";
  std::mutex m;
  std::map<std::string, ScopePrincipal> scopes;
  std::map<std::string, PrincipalCheck> checks;
};

// `path` is `scope` or under it.
bool under(const std::string &path, const std::string &scope) {
  if (scope.empty())
    return true;
  return path == scope ||
         (path.size() > scope.size() && path.compare(0, scope.size(), scope) == 0 &&
          path[scope.size()] == cvc::state::SEPARATOR[0]);
}

} // namespace

std::string set_scope_principal(cvc::app &app, const std::string &scope, ScopePrincipal p) {
  const std::string canonical = cvc::net::CredentialStore::canonical_principal(p.principal);
  if (canonical.empty())
    return "'" + p.principal + "' is not a principal (local, file:<dir> or remote:<origin>)";
  p.principal = canonical;
  if (!p.local() && (!p.net_policy.empty() || !p.credential_store.empty() || !p.e2ee_store.empty()))
    return p.principal +
           " is sandboxed: only a local application has a network policy or store of its own";
  principal_slot &s = app.slot<principal_slot>();
  std::vector<PrincipalCheck> checks;
  {
    std::lock_guard<std::mutex> lock(s.m);
    for (const auto &[name, check] : s.checks)
      checks.push_back(check);
  }
  for (const PrincipalCheck &check : checks)
    if (std::string why = check(p); !why.empty())
      return why;
  std::lock_guard<std::mutex> lock(s.m);
  s.scopes[scope] = std::move(p);
  return std::string();
}

void clear_scope_principal(cvc::app &app, const std::string &scope) {
  principal_slot &s = app.slot<principal_slot>();
  std::lock_guard<std::mutex> lock(s.m);
  s.scopes.erase(scope);
}

ScopePrincipal scope_principal(cvc::app &app, const std::string &scope) {
  principal_slot &s = app.slot<principal_slot>();
  std::lock_guard<std::mutex> lock(s.m);
  // Outermost first (a map sorts a scope before the scopes under it): each one that covers
  // `scope` replaces what an outer one said, unless the outer one is sandboxed and it is local.
  ScopePrincipal found;
  for (const auto &[at, p] : s.scopes) {
    if (!under(scope, at))
      continue;
    if (p.local() && !found.local())
      continue; // cannot raise
    found = p;
  }
  return found;
}

std::vector<std::pair<std::string, ScopePrincipal>> scope_principals(cvc::app &app) {
  principal_slot &s = app.slot<principal_slot>();
  std::lock_guard<std::mutex> lock(s.m);
  return {s.scopes.begin(), s.scopes.end()};
}

void set_principal_check(cvc::app &app, const std::string &name, PrincipalCheck check) {
  principal_slot &s = app.slot<principal_slot>();
  std::lock_guard<std::mutex> lock(s.m);
  if (check)
    s.checks[name] = std::move(check);
  else
    s.checks.erase(name);
}

} // namespace ariadne
} // namespace cvc
