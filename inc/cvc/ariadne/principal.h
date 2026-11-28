#ifndef CVC_ARIADNE_PRINCIPAL_H
#define CVC_ARIADNE_PRINCIPAL_H

// Ariadne — who a program runs as (docs/roadmap/ARIADNE_SHELL.md 4.12).
//
// Every mount runs as a principal, fixed when it is mounted from where its bytes came; a document
// cannot name its own:
//   local            a document the launcher or the shell's catalog named, trusted as documents
//                    are today;
//   file:<dir>       a local file the user opened that is not in the catalog, sandboxed with its
//                    own directory in place of an origin;
//   remote:<origin>  a document fetched from a URL, such as remote:https://apps.example.org.
//
// The Runtime records a mount's principal here when it opens the mount, keyed by the mount's scope
// (its absolute state prefix, the root_path its programs are chrooted to), and drops it when the
// mount closes. The hosts that hand a program what only its principal may use (the network, its
// allowlist and credentials, the end-to-end keys and the OIDC sign-ins: net_host.h) look the
// program's scope up here when they build its verbs.
//
// A scope nothing recorded runs as `local`: the outermost document and what it load:s. A scope
// under recorded ones runs as the innermost, except that a `local` one inside a sandboxed one runs
// as the sandboxed one: what a document loads cannot raise it.
//
// One table per app (its slot), locked: the provider that asks may run on any thread.

#include <functional>
#include <string>
#include <utility>
#include <vector>

namespace cvc {
class app;
namespace ariadne {

// What programs under a scope run as.
struct ScopePrincipal {
  // local, file:<dir> or remote:<origin> (cvc::net::CredentialStore::valid_principal).
  std::string principal = "local";
  // Origins granted beyond what the principal reaches by default (its own, for a remote one), as
  // policy origins ("https://api.example.org"). The asking and granting is the shell's (its
  // requires: {name: net.origin, origins: [...]} and the sheet); this is what it granted.
  std::vector<std::string> net_origins;
  // A local application's own network, in place of the launcher's (the shell's
  // shell.apps.<as>.net_policy, .credential_store and .e2ee_store): a --net-policy file, a
  // --credential-store SPEC and an --e2ee-store SPEC. "" each: the launcher's. Only a `local`
  // principal may have them; a sandboxed one's are refused.
  std::string net_policy;
  std::string credential_store;
  std::string e2ee_store;

  bool local() const { return principal == "local"; }
  bool operator==(const ScopePrincipal &) const = default;
};

// Record that programs whose scope is `scope` or under it run as `p`, replacing what was recorded
// for `scope`. "" when it was recorded, else why not, and nothing changed: `p.principal` is not a
// principal, a sandboxed one names a policy or store of its own, or a host's check refused it (the
// network's: its policy file does not load). The Runtime refuses the mount then.
std::string set_scope_principal(cvc::app &app, const std::string &scope, ScopePrincipal p);
// Forget what was recorded for `scope` (not for the scopes under it).
void clear_scope_principal(cvc::app &app, const std::string &scope);
// What a program whose scope is `scope` runs as (above), `local` when nothing covers it. The
// principal is in canonical form (CredentialStore::canonical_principal).
ScopePrincipal scope_principal(cvc::app &app, const std::string &scope);
// Every recorded scope and what it records, sorted by scope.
std::vector<std::pair<std::string, ScopePrincipal>> scope_principals(cvc::app &app);

// A host's check of a principal about to be recorded: "" to accept it, else why not. The network's
// host (enable_net) adds one that loads a local application's own policy and stores, so a bad one
// refuses the mount instead of leaving its programs without a network. Named, so a second enable
// replaces the first; null removes it.
using PrincipalCheck = std::function<std::string(const ScopePrincipal &)>;
void set_principal_check(cvc::app &app, const std::string &name, PrincipalCheck check);

} // namespace ariadne
} // namespace cvc

#endif // CVC_ARIADNE_PRINCIPAL_H
