#ifndef CVC_NET_CREDENTIALS_H
#define CVC_NET_CREDENTIALS_H

// cvc::net — credentials a host keeps for the documents it runs (docs/ARIADNE.md §10.3).
//
// A CredentialStore holds bearer tokens by name. A document never sees one: it names a credential
// on a request (auth: "txwtf") and the host adds "Authorization: Bearer <token>" as the request is
// sent, or it asks the host to take one from a sign-in reply (capture_auth:), and the host keeps
// the token and blanks it out of the reply. So a token never reaches the document's variables or
// the state tree (which a state transport replicates), and never appears in a log or an error
// message.
//
// Each credential is bound to the origins it may go to. A NetPolicy's credential lines bind a name
// to a list of origins; a name no line binds is bound to the origin it was captured from. A token
// is sent only to an origin its credential is bound to.
//
// With a file, the store keeps its credentials across runs: one "<name> <origin> <token>" line
// each, written owner-only (0600 in a 0700 directory on POSIX; a Windows profile directory is the
// user's own) and replaced whole on every change. Without one (and always in a browser), they live
// in memory and end with the process.
//
// Thread-safe: requests on background threads read it while the program's thread forgets.

#include <cstddef>
#include <cvc/net/net_policy.h>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace cvc {
namespace net {

// What a redacted secret is replaced with, in a reply body and its headers.
inline constexpr char kRedacted[] = "[redacted]";

// A credential the host can renew: an OAuth 2.0 access token it holds a refresh token for
// (cvc/ariadne/oidc_host.h). A request that sends the credential (auth:) asks it to renew first
// when the token is about to expire, and once more, then sends again, when the server answers 401.
class CredentialRenewer {
public:
  virtual ~CredentialRenewer() = default;
  // Whether the token is about to expire.
  virtual bool expiring() const = 0;
  // Put a fresh token in the store. `rejected` is the token a server just refused, or "" before a
  // send: when the store holds another token by then, a renewal raced this one, and nothing more
  // is done. "" when the store now holds a token worth sending, else why not (never quoting one).
  virtual std::string renew(const std::string &rejected) = 0;
  // renew() for a caller that must not block: a browser page's frame loop, where requests are
  // started rather than sent (cvc::net::http_starts_async). `done` gets what renew() would return,
  // later, from the browser's event loop, on the thread that called this. By default it calls
  // renew() in place.
  virtual void renew_async(const std::string &rejected, std::function<void(std::string)> done) {
    done(renew(rejected));
  }
};

class CredentialStore {
public:
  // `path` "" keeps credentials in memory only.
  explicit CredentialStore(std::string path = std::string());
  CredentialStore(const CredentialStore &) = delete;
  CredentialStore &operator=(const CredentialStore &) = delete;

  // Read the file (a missing one holds nothing). False, with `error` set and nothing loaded, when
  // it cannot be read, is malformed, or (on POSIX) belongs to another user or is open to others.
  bool load(std::string *error = nullptr);
  const std::string &path() const { return path_; }
  bool persistent() const { return !path_.empty(); }

  // Bind credential `name` to `origins` (replacing any earlier binding of it).
  void bind(const std::string &name, std::vector<Origin> origins);
  // The origins `name` is bound to, or empty when it is bound to wherever it was captured.
  std::vector<Origin> binding(const std::string &name) const;

  // Keep `token`, which `from` returned, as credential `name`. False, and nothing kept, when the
  // name or the token is malformed or `name` is bound to origins without `from`; `error` says why
  // (never quoting the token). A bound name holds one token, which this replaces; an unbound one
  // holds one per origin. With a file, it is written at once; a failed write keeps the token in
  // memory, returns true and sets `error`.
  bool capture(const std::string &name, const Origin &from, const std::string &token,
               std::string *error = nullptr);

  // Hold `token` as credential `name` for this run only, never written to the file: a token the
  // host was handed (--credential-env). `name` must be bound (bind()); the token goes to its bound
  // origins, ahead of one the file holds for it, until a capture replaces it or it is forgotten.
  // False, and nothing held, when the name or the token is malformed or the name is not bound;
  // `error` says why (never quoting the token).
  bool provide(const std::string &name, const std::string &token, std::string *error = nullptr);

  // The token credential `name` sends to `to`, if it has one there.
  std::optional<std::string> token_for(const std::string &name, const Origin &to) const;
  bool has(const std::string &name) const;
  bool has(const std::string &name, const Origin &to) const {
    return token_for(name, to).has_value();
  }
  // Credential names with a token, sorted.
  std::vector<std::string> names() const;

  // Drop every token of `name` (true if there was one), or every token, and any renewer of it. With
  // a file, it is rewritten; `error` reports a failed write.
  bool forget(const std::string &name, std::string *error = nullptr);
  void clear(std::string *error = nullptr);

  // The renewer of credential `name` (null: none), which a request sending it consults.
  void set_renewer(const std::string &name, std::shared_ptr<CredentialRenewer> renewer);
  std::shared_ptr<CredentialRenewer> renewer(const std::string &name) const;

  // A credential name: 1-64 of [A-Za-z0-9._-].
  static bool valid_name(const std::string &name);
  // A bearer token as RFC 6750 writes one: 16-4096 of [A-Za-z0-9-._~+/], then any '='. That
  // keeps a header line from being split, and a redaction from blanking a common word.
  static bool valid_token(const std::string &token);
  // Where the host flag --credential-store SPEC puts the file: a plain name (valid_name) is a
  // file of that name in the user's config directory (%LOCALAPPDATA%\cvc\credentials on Windows,
  // else $XDG_CONFIG_HOME/cvc/credentials or ~/.config/cvc/credentials); anything else is a
  // path. "" when no config directory can be named (or in a browser).
  static std::string resolve_path(const std::string &spec);

private:
  struct Entry {
    std::string name;
    Origin from;
    std::string token;
    bool kept = true; // written to the file (false: provided, this run only)
  };
  bool save_locked(std::string *error) const;
  bool sends_to(const std::string &name, const Entry &e, const Origin &to) const;

  std::string path_;
  mutable std::mutex mutex_;
  std::vector<Entry> entries_;
  std::vector<std::pair<std::string, std::vector<Origin>>> bindings_;
  std::vector<std::pair<std::string, std::shared_ptr<CredentialRenewer>>> renewers_;
};

// The capture half of a sign-in: find the JSON string at `field` (object keys joined by '.', e.g.
// "token" or "data.token") in the reply `body`. On success `token` holds it and `body` has that
// string literal replaced by "[redacted]", every other byte unchanged. False, and `body`
// untouched, when the body is not a JSON object or `field` names no string in it.
bool take_json_string(std::string &body, const std::string &field, std::string &token);

// Replace every occurrence of `secret` in `text` with kRedacted; how many there were.
std::size_t redact(std::string &text, const std::string &secret);

} // namespace net
} // namespace cvc

#endif // CVC_NET_CREDENTIALS_H
