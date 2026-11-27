// Asks and grants — see the header. The token table, and the arithmetic of granting: a grant
// bounded by what the granter holds, and what a document holds from what it asked and was granted.

#include <algorithm>
#include <cctype>
#include <cvc/ariadne/grants.h>
#include <iterator>

namespace cvc {
namespace ariadne {

namespace {

// Constant data: an array of pointers to string literals, no constructor runs.
constexpr GrantToken kTokens[] = {
    {"delegate.keys", "its on_key: and keys: run for the keys it has the focus for",
     "receive the keys you type while it has the focus"},
    {"delegate.pointer", "its on_pointer: runs for the pointer over its own windows",
     "see the pointer over its windows"},
    {"delegate.signals",
     "its on_signal: runs for the signals passed on to it: an interrupt (Ctrl-C) while it has the "
     "focus, a request to end (SIGTERM, SIGHUP) always; a list in grants: limits which",
     "be told when it is interrupted, or when the shell is asked to end"},
    {"extend.menubar", "it may add items to the parent's menus", "add items to the shell's menus"},
    {"extend.tray", "its tray icons join the parent's tray", "show icons in the shell's tray"},
    {"extend.context_menu", "it may add items to the right-click menus the parent opens to it",
     "add items to the shell's right-click menus"},
    {"extend.settings", "its settings are a page of the parent's Settings window",
     "add a page to the shell's Settings window"},
    {"access.settings", "it reads and writes the settings the parent shares",
     "change the shell's settings"},
    {"access.settings.read", "it reads the settings the parent shares",
     "read the shell's settings"},
    {"request.main_view", "it may ask to be shown in the main view", "ask to fill the main view"},
    {"request.open", "it may ask its shell to open a document as the shell's own application",
     "ask the shell to open other documents"},
    {"net.origin",
     "it reaches the network origins it names in requires: ({name: net.origin, origins: [...]}) "
     "beyond its own",
     "reach other sites on the network"},
};

constexpr const char *kDefaults[] = {"extend.tray", "extend.settings"};

bool starts_with(const std::string &s, const char *prefix) { return s.rfind(prefix, 0) == 0; }

bool contains(const std::vector<std::string> &v, const std::string &x) {
  return std::find(v.begin(), v.end(), x) != v.end();
}

void add_once(std::vector<std::string> &v, const std::string &x) {
  if (!contains(v, x))
    v.push_back(x);
}

std::string lower(std::string s) {
  std::transform(s.begin(), s.end(), s.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return s;
}

// `a` with only what is in `b` too, in `a`'s order.
std::vector<std::string> intersect(const std::vector<std::string> &a,
                                   const std::vector<std::string> &b) {
  std::vector<std::string> out;
  for (const std::string &x : a)
    if (contains(b, x))
      out.push_back(x);
  return out;
}

} // namespace

std::span<const GrantToken> grant_tokens() { return kTokens; }

bool is_grant_token(const std::string &name) {
  return std::any_of(std::begin(kTokens), std::end(kTokens),
                     [&](const GrantToken &t) { return name == t.name; });
}

std::vector<std::string> default_grants() {
  return std::vector<std::string>(std::begin(kDefaults), std::end(kDefaults));
}

bool held_alone(const std::string &name) {
  return is_grant_token(name) &&
         (starts_with(name, "delegate.") || starts_with(name, "extend.") || name == "net.origin");
}

bool GrantSet::covers_signal(const std::string &signal) const {
  return every_signal || contains(signals, signal_name(signal));
}

bool GrantSet::covers_origin(const std::string &origin) const {
  if (every_origin)
    return true;
  const std::string o = policy_origin(origin);
  return !o.empty() && contains(origins, o);
}

std::string GrantBuilder::add(const std::string &token, const std::vector<std::string> *covers) {
  if (!is_grant_token(token))
    return "'" + token + "' is not a grant token";
  if (covers && token != "delegate.signals" && token != "net.origin")
    return "'" + token + "' takes no list (only delegate.signals and net.origin do)";
  g_.tokens.insert(token);
  std::string why;
  if (token == "delegate.signals") {
    if (!covers)
      bare_signals_ = true;
    else {
      listed_signals_ = true;
      for (const std::string &s : *covers)
        add_once(g_.signals, signal_name(s));
    }
  } else if (token == "net.origin") {
    if (!covers)
      bare_origins_ = true;
    else {
      listed_origins_ = true;
      for (const std::string &o : *covers) {
        const std::string p = policy_origin(o);
        if (p.empty())
          why += (why.empty() ? "" : "; ") + ("'" + o + "' is not an http(s) origin");
        else
          add_once(g_.origins, p);
      }
    }
  }
  return why;
}

GrantSet GrantBuilder::take() const {
  GrantSet g = g_;
  g.every_signal = bare_signals_ || !listed_signals_;
  g.every_origin = bare_origins_ || !listed_origins_;
  if (g.every_signal)
    g.signals.clear();
  if (g.every_origin)
    g.origins.clear();
  return g;
}

GrantSet default_grant_set() {
  GrantSet g;
  for (const char *t : kDefaults)
    g.tokens.insert(t);
  return g;
}

GrantSet deny_tokens(GrantSet granted, const std::vector<std::string> &deny) {
  for (const std::string &t : deny)
    granted.tokens.erase(t);
  return granted;
}

GrantSet pick_tokens(GrantSet granted, const std::vector<std::string> &pick) {
  std::set<std::string> kept;
  for (const std::string &t : pick)
    if (granted.has(t))
      kept.insert(t);
  granted.tokens = std::move(kept);
  return granted;
}

bool MountGrants::takes_signal(const std::string &signal) const {
  return holds("delegate.signals") && (every_signal || contains(signals, signal_name(signal)));
}

GrantSet bound_grants(GrantSet granted, const MountGrants *parent) {
  std::set<std::string> kept;
  for (const std::string &t : granted.tokens)
    if (parent ? parent->holds(t) : held_alone(t))
      kept.insert(t);
  granted.tokens = std::move(kept);
  if (!parent)
    return granted; // alone, it takes every signal and reaches every origin the launcher allows
  if (!parent->every_signal) {
    granted.signals =
        granted.every_signal ? parent->signals : intersect(granted.signals, parent->signals);
    granted.every_signal = false;
  }
  // The parent reaches its own origins (its principal's) and the ones it holds: a grant of
  // net.origin passes on only those it holds.
  granted.origins =
      granted.every_origin ? parent->net_origins : intersect(granted.origins, parent->net_origins);
  granted.every_origin = false;
  return granted;
}

MountGrants hold_grants(const GrantSet &granted, const std::vector<HostRequirement> &asks) {
  MountGrants g;
  g.granted = granted;
  // The defaults hold without being asked for: a loaded application has always had its tray
  // icons and its own Settings page.
  for (const char *t : kDefaults)
    if (granted.has(t))
      g.held.insert(t);
  for (const HostRequirement &req : asks) {
    if (!is_grant_token(req.name))
      continue;
    if (req.name == "net.origin") {
      bool any = req.origins.empty() && granted.has("net.origin");
      for (const std::string &o : req.origins) {
        const std::string p = policy_origin(o);
        if (!p.empty() && granted.has("net.origin") && granted.covers_origin(p)) {
          add_once(g.net_origins, p);
          any = true;
        } else if (!req.optional) {
          add_once(g.refused, "net.origin " + (p.empty() ? o : p));
        }
      }
      if (req.origins.empty() && !granted.has("net.origin") && !req.optional)
        add_once(g.refused, "net.origin");
      if (any)
        g.held.insert("net.origin");
      continue;
    }
    // Reading the shared settings is part of reading and writing them.
    const bool given = granted.has(req.name) ||
                       (req.name == "access.settings.read" && granted.has("access.settings"));
    if (given)
      g.held.insert(req.name);
    else if (!req.optional)
      add_once(g.refused, req.name);
  }
  if (g.holds("delegate.signals")) {
    g.every_signal = granted.every_signal;
    g.signals = granted.signals;
  }
  return g;
}

std::string policy_origin(const std::string &origin) {
  const std::size_t colon = origin.find("://");
  if (colon == std::string::npos)
    return {};
  const std::string scheme = lower(origin.substr(0, colon));
  if (scheme != "http" && scheme != "https")
    return {};
  std::string rest = origin.substr(colon + 3);
  rest = rest.substr(0, rest.find_first_of("/?#"));
  if (rest.empty() || rest.find('@') != std::string::npos)
    return {}; // no host, or credentials in it
  std::string host = rest, port;
  if (rest.front() == '[') { // an IPv6 literal: [::1]:8443
    const std::size_t close = rest.find(']');
    if (close == std::string::npos)
      return {};
    host = rest.substr(0, close + 1);
    if (close + 1 < rest.size()) {
      if (rest[close + 1] != ':')
        return {};
      port = rest.substr(close + 2);
    }
  } else if (const std::size_t c = rest.rfind(':'); c != std::string::npos) {
    host = rest.substr(0, c);
    port = rest.substr(c + 1);
  }
  if (host.empty() || host == "[]")
    return {};
  if (!port.empty() && !std::all_of(port.begin(), port.end(),
                                    [](unsigned char ch) { return std::isdigit(ch) != 0; }))
    return {};
  if ((scheme == "http" && port == "80") || (scheme == "https" && port == "443"))
    port.clear();
  return scheme + "://" + lower(host) + (port.empty() ? "" : ":" + port);
}

std::string signal_name(const std::string &spelled) {
  std::string s;
  for (unsigned char c : spelled)
    if (!std::isspace(c))
      s += static_cast<char>(std::toupper(c));
  if (!s.empty() && s.rfind("SIG", 0) != 0)
    s = "SIG" + s;
  return s;
}

} // namespace ariadne
} // namespace cvc
