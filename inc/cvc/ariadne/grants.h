#ifndef CVC_ARIADNE_GRANTS_H
#define CVC_ARIADNE_GRANTS_H
// Asks and grants (docs/roadmap/ARIADNE_SHELL.md 4.5, docs/ARIADNE.md §5.7 and §6.3).
//
// A mounted document reaches nothing beyond its own scope unless it asks and the document that
// loads it grants it. It asks in its `requires:`, with the grant tokens below; the loading
// document grants with `grants:` on its `load:` or its `mounts:` slot. A token holds in the
// mounted document when it asked for it and was granted it, and (has? NAME) says which hold:
//
//   delegate.keys       its on_key: and keys: run for the keys it has the focus for
//   delegate.pointer    its on_pointer: runs for the pointer over its own windows
//   delegate.signals    its on_signal: runs for the signals passed on to it; a list
//                       ({delegate.signals: [SIGTERM, SIGHUP]} in grants:) limits which
//   extend.menubar      it may add items to the parent's menus (SH9)
//   extend.tray         its tray icons join the parent's tray
//   extend.context_menu it may add items to the parent's right-click menus (SH9)
//   extend.settings     its settings are a page of the parent's Settings window (SH9)
//   access.settings     it reads and writes the settings the parent shares (SH9);
//   access.settings.read  ... reads only
//   request.main_view   it may ask to be shown in the main view (SH6)
//   request.open        it may ask its shell to open a document as the shell's own (SH14)
//   net.origin          it reaches the network origins it names in requires:
//                       ({name: net.origin, origins: [...]}) beyond its own (SH11)
//
// - A required token that is not granted refuses the mount, naming it; an optional one leaves
//   (has? ...) false. A token the document did not ask for is not given, whatever the parent
//   grants, except the defaults below, which every mount has had: its tray icons and its own
//   Settings page.
// - Alone, the outermost document holds every delegate.* and extend.* token (what it would
//   delegate or extend is its own) and net.origin for every origin (the launcher's allowlist
//   decides); it holds no access.* or request.* token.
// - With no grants:, a mount is granted the defaults, [extend.tray, extend.settings]; `deny:`
//   takes tokens away, and `grants: []` gives nothing. A mounts: slot's grants: is the most any
//   of its applications may get, and an entry's own `grants` picks from it. A mount grants its own
//   mounts only what it holds itself, so a grant never widens down the tree.
//
// Pure functions over plain data: the loader works out a static load:'s grants, the Runtime a
// mounts: entry's, and both hand the result to the mount (Widget::grants).

#include <cvc/ariadne/keys.h>
#include <cvc/ariadne/widget.h> // HostRequirement
#include <set>
#include <span>
#include <string>
#include <vector>

namespace cvc {
namespace ariadne {

// One grant token and what it means: `meaning` for an author (ari check, the docs), `words` for a
// user deciding whether to grant it (the sheet, SH14).
struct GrantToken {
  const char *name;
  const char *meaning;
  const char *words;
};

// Every grant token, in documentation order (constant data).
std::span<const GrantToken> grant_tokens();

// Is `name` a grant token (answered from a mount's grants, not from the host)?
bool is_grant_token(const std::string &name);

// The tokens granted when the loading document names none: extend.tray, extend.settings.
std::vector<std::string> default_grants();

// Does the outermost document, running alone, hold `name`? Every delegate.* and extend.* token,
// and net.origin; no access.* or request.* token.
bool held_alone(const std::string &name);

// What a loading document grants a mount: tokens, and for two of them what they cover.
struct GrantSet {
  std::set<std::string> tokens;
  // delegate.signals covers every signal, or only `signals` (POSIX names: SIGINT, SIGTERM, ...).
  bool every_signal = true;
  std::vector<std::string> signals;
  // net.origin covers every origin the document asks for, or only `origins` (policy origins,
  // scheme://host[:port]).
  bool every_origin = true;
  std::vector<std::string> origins;

  bool has(const std::string &token) const { return tokens.count(token) != 0; }
  bool covers_signal(const std::string &signal) const;
  bool covers_origin(const std::string &origin) const;
  bool operator==(const GrantSet &) const = default;
};

// A `grants:` list, an item at a time, as the loader reads one from a document and the Runtime
// from a mounts: entry: a token, or a token with what it covers ({delegate.signals: [SIGTERM]},
// {net.origin: [https://api.example.org]}). A bare token covers everything; a list limits it.
class GrantBuilder {
public:
  // Add one item: "" when it was added, else why not (not a grant token; a list on a token that
  // takes none; an origin that is not an http(s) origin, which is left out).
  std::string add(const std::string &token, const std::vector<std::string> *covers = nullptr);
  GrantSet take() const;

private:
  GrantSet g_;
  bool bare_signals_ = false, listed_signals_ = false;
  bool bare_origins_ = false, listed_origins_ = false;
};

// The defaults (default_grants()), as a GrantSet.
GrantSet default_grant_set();

// `granted` minus every token in `deny`.
GrantSet deny_tokens(GrantSet granted, const std::vector<std::string> &deny);

// `granted` narrowed to `pick` (an entry's own grants, picked from its slot's): the tokens in
// both. A token in `pick` the slot does not grant is left out.
GrantSet pick_tokens(GrantSet granted, const std::vector<std::string> &pick);

// One mounted document's asks and grants, worked out where it is mounted, and what it may run
// for them. Carried on its mount (Widget::grants).
struct MountGrants {
  // What the loading document grants it, bounded by what that document holds (bound_grants).
  GrantSet granted;
  // What it holds: the tokens it asked for and was granted, and the defaults it was granted.
  // (has? NAME) answers true for these.
  std::set<std::string> held;
  // delegate.signals, when held: the signals it takes, every one or these.
  bool every_signal = true;
  std::vector<std::string> signals;
  // net.origin, when held: the origins it reaches beyond its principal's default, policy origins
  // in the order it asked for them. SH11's ScopePrincipal::net_origins.
  std::vector<std::string> net_origins;
  // What it requires and was not granted, each named for the refusal ("delegate.keys", or
  // "net.origin https://api.example.org"). Not empty: the mount is refused.
  std::vector<std::string> refused;

  // Its input and signals (docs/ARIADNE.md §6.3), kept when it holds the token for them.
  std::string on_key;
  std::string on_pointer;
  std::string on_signal;
  std::vector<KeyBinding> keys; // its keys: block; its keys live at <its sub-prefix>.keys.<id>

  bool holds(const std::string &token) const { return held.count(token) != 0; }
  bool takes_signal(const std::string &signal) const;
};

// `granted` bounded by `parent`, what the loading document holds (null: it is the outermost
// document, running alone): a token it does not hold is not passed on, delegate.signals covers no
// signal it does not take, and net.origin no origin it does not reach.
GrantSet bound_grants(GrantSet granted, const MountGrants *parent);

// A mount's grants from what it is granted (already bounded) and what it asks for (its
// requires:): held, signals, net_origins and refused. The input fields are left empty.
MountGrants hold_grants(const GrantSet &granted, const std::vector<HostRequirement> &asks);

// An origin as policy origins are written: scheme://host[:port], scheme and host lower case, no
// path, no trailing '/', a default port (80 for http, 443 for https) left out. "" when `origin` is
// not an http or https origin.
std::string policy_origin(const std::string &origin);

// A signal's POSIX name from how a document may spell it: "SIGTERM", "sigterm" or "TERM" all
// give "SIGTERM".
std::string signal_name(const std::string &spelled);

} // namespace ariadne
} // namespace cvc

#endif // CVC_ARIADNE_GRANTS_H
