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
// Every credential belongs to a principal (docs/roadmap/ARIADNE_SHELL.md 4.12): who the program
// that captured it runs as. `local` is a document the launcher or the shell's catalog named; a
// document opened from a file is `file:<its directory>`, one fetched from the network
// `remote:<its origin>`. A store object is one principal's view of the credentials: what it
// captures, finds, binds, forgets and renews is that principal's alone, so a document fetched from
// a post cannot send the reader's `txwtf` token by naming it. The constructor makes a new store and
// its `local` view; for_principal() hands out the views of the other principals, which share its
// file, its entries and its lock.
//
// With a file, the store keeps its credentials across runs: one "<principal> <name> <origin>
// <token>" line each (the principal percent-encoded; a line of three words, as a store written
// before principals were kept, is `local`'s), written owner-only (0600 in a 0700 directory on
// POSIX; a Windows profile directory is the user's own) and replaced whole on every change.
// Without one (and always in a browser), they live in memory and end with the process.
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
  // `cancelled` is asked while the refresh is in flight, as HttpRequest::cancelled is (empty:
  // never): a caller whose wait someone else may end (an async call's kernel) passes its own.
  virtual std::string renew(const std::string &rejected,
                            const std::function<bool()> &cancelled) = 0;
  // renew() that nothing cancels.
  std::string renew(const std::string &rejected) { return renew(rejected, {}); }
  // renew() for a caller that must not block: a browser page's frame loop, where requests are
  // started rather than sent (cvc::net::http_starts_async). `done` gets what renew() would return,
  // later, from the browser's event loop, on the thread that called this. By default it calls
  // renew() in place.
  virtual void renew_async(const std::string &rejected, std::function<void(std::string)> done) {
    done(renew(rejected));
  }
};

// The principal of a document the launcher or the shell's catalog named.
inline constexpr char kLocalPrincipal[] = "local";

class CredentialStore {
public:
  // A new store, and its `local` principal's view of it. `path` "" keeps credentials in memory
  // only.
  explicit CredentialStore(std::string path = std::string());
  CredentialStore(const CredentialStore &) = delete;
  CredentialStore &operator=(const CredentialStore &) = delete;

  // The same store as principal `principal` sees it: the same file, entries and lock, and that
  // principal's own credentials, bindings and renewers. Two views of one principal see the same.
  // Null when `principal` is not one (valid_principal).
  std::shared_ptr<CredentialStore> for_principal(const std::string &principal);
  // Whose credentials this view holds.
  const std::string &principal() const { return principal_; }
  // Whether `other` views the same store as the same principal.
  bool same_view(const CredentialStore &other) const {
    return shared_ == other.shared_ && principal_ == other.principal_;
  }

  // Read the file (a missing one holds nothing): every principal's credentials. False, with
  // `error` set and nothing loaded, when it cannot be read, is malformed, or (on POSIX) belongs to
  // another user or is open to others.
  bool load(std::string *error = nullptr);
  const std::string &path() const;
  bool persistent() const { return !path().empty(); }

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
  // Credential names with a token, sorted (this principal's).
  std::vector<std::string> names() const;

  // Drop every token of `name` (true if there was one), or every token, and any renewer of it: this
  // principal's only. With a file, it is rewritten; `error` reports a failed write.
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

  // A principal as written: `local`, `file:<directory>` (a non-empty path without control bytes)
  // or `remote:<origin>` (an origin as parse_origin_spec reads one).
  static bool valid_principal(const std::string &principal);
  // `principal` as the store keys it: a remote one's origin written as Origin::str writes it
  // ("remote:HTTPS://Apps.Example.org:443/" is "remote:https://apps.example.org"), any other as
  // it is. "" when it is not a principal.
  static std::string canonical_principal(const std::string &principal);

private:
  struct Shared; // the file, the entries of every principal, their bindings and renewers, the lock
  CredentialStore(std::shared_ptr<Shared> shared, std::string principal);

  std::shared_ptr<Shared> shared_;
  std::string principal_;
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
