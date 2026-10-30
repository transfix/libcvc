#ifndef CVC_NET_NET_POLICY_H
#define CVC_NET_NET_POLICY_H

// cvc::net — which origins a host lets its requests reach (docs/ARIADNE.md §10.3).
//
// An origin is a scheme, a host and a port: https://tx.wtf, http://127.0.0.1:8080. A NetPolicy is
// an allowlist of them, plus which origins each named credential (credentials.h) may be sent to.
// cvc::net::send(req, policy) (http_client.h) refuses a request to an origin the policy does not
// allow, and checks every redirect it follows the same way. A default-constructed policy allows
// nothing.
//
// Request URLs are parsed strictly, so the origin checked here is the one the transport connects
// to: only http and https, no user info ("user@host"), no backslash, space or control byte, and a
// host of ASCII letters, digits, '-' and '.', or a bracketed IPv6 literal. An internationalized
// name is written in its punycode form. Anything else is refused, not guessed at.
//
// A policy file holds one directive per line; '#' starts a comment:
//
//   allow https://tx.wtf                         # requests may go to this origin
//   allow http://127.0.0.1:8080
//   credential txwtf https://tx.wtf https://api.tx.wtf   # where credential txwtf may be sent
//
// Every origin a credential line names must also be allowed (by the file or by the host's own
// flags); the host checks that when it puts the policy together.

#include <map>
#include <optional>
#include <string>
#include <vector>

namespace cvc {
namespace net {

struct Origin {
  std::string scheme; // "http" or "https"
  std::string host;   // lowercased; an IPv6 literal keeps its brackets
  int port = 0;       // always explicit: the scheme's default (80, 443) when the URL gave none

  // The origin as written in a policy: "https://tx.wtf" (a default port is left out),
  // "http://127.0.0.1:8080".
  std::string str() const;
  bool operator==(const Origin &) const = default;
};

// The origin of an http(s) URL (a path, query and fragment may follow the authority). nullopt,
// with `error` saying why, for anything the strict parse above refuses.
std::optional<Origin> parse_origin(const std::string &url, std::string *error = nullptr);

// An origin as a policy or a flag writes it: scheme://host[:port], with at most a trailing '/'.
// A path is refused, so nobody reads an allow line as allowing only part of a site.
std::optional<Origin> parse_origin_spec(const std::string &spec, std::string *error = nullptr);

class NetPolicy {
public:
  // Allow requests to `o` (adding one twice is harmless).
  void allow(const Origin &o);
  bool allows(const Origin &o) const;
  // Whether a request to `url` may be made: its origin parses and is allowed. `why` says what is
  // wrong otherwise.
  bool allows_url(const std::string &url, std::string *why = nullptr) const;
  const std::vector<Origin> &allowed() const { return allowed_; }

  // Credential `name` may be sent to (and captured from) `o`. A credential no line names is bound
  // to the origin it was captured from.
  void bind_credential(const std::string &name, const Origin &o);
  const std::map<std::string, std::vector<Origin>> &credential_bindings() const {
    return bindings_;
  }

private:
  std::vector<Origin> allowed_;
  std::map<std::string, std::vector<Origin>> bindings_;
};

// Add a policy file's directives (format above) to `into`. False, with `error` naming the line, on
// the first malformed one; `into` may then hold the lines before it. `source` names the text in
// errors (a file name).
bool parse_net_policy(const std::string &text, NetPolicy &into, std::string *error = nullptr,
                      const std::string &source = "net policy");
// parse_net_policy over the file at `path`.
bool load_net_policy_file(const std::string &path, NetPolicy &into, std::string *error = nullptr);

// The origin of the page this program runs in, in a browser build ("" natively, or where the page
// has no http(s) origin).
std::string page_origin();

} // namespace net
} // namespace cvc

#endif // CVC_NET_NET_POLICY_H
