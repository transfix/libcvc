#ifndef CVC_NET_LOOPBACK_REDIRECT_H
#define CVC_NET_LOOPBACK_REDIRECT_H

// cvc::net — the loopback redirect URI of a native OAuth 2.0 client (RFC 8252 7.3,
// docs/ARIADNE.md §10.5).
//
// A native app that signs in through the system browser asks the provider to send the browser
// back to http://127.0.0.1:PORT/PATH, where this listens. It binds 127.0.0.1 only, on an ephemeral
// port unless one is asked for, never blocks (poll() takes what has arrived), and accepts exactly
// one redirect: the first request for PATH. Other paths (a browser's favicon) get a 404. The
// caller checks the redirect's query, then answers the browser tab with a small page that says
// whether it worked and that the tab can be closed, and the listener stops.
//
// Not available in a browser build (a page cannot listen): listen() fails there.

#include <memory>
#include <optional>
#include <string>

namespace cvc {
namespace net {

class LoopbackRedirect {
public:
  LoopbackRedirect();
  ~LoopbackRedirect();
  LoopbackRedirect(const LoopbackRedirect &) = delete;
  LoopbackRedirect &operator=(const LoopbackRedirect &) = delete;

  // Whether this build can listen at all (false in a browser).
  static bool available();

  // Listen on 127.0.0.1:port (0: a port the system picks) for requests to `path` (starting with
  // '/'). False, with `error` saying why, when it cannot.
  bool listen(int port, const std::string &path, std::string *error = nullptr);
  // The redirect URI: http://127.0.0.1:PORT/PATH ("" before listen()).
  std::string redirect_uri() const;

  // Take what has arrived, without blocking. Returns the query string (without '?') of the first
  // request for the path, once; nullopt until then and after.
  std::optional<std::string> poll();

  // Answer the redirect's browser tab: a page saying it worked (`ok`) or not, with `message`
  // (HTML-escaped), and that the tab can be closed. Then stop listening. Safe to call without a
  // redirect (it only stops listening).
  void respond(bool ok, const std::string &message);
  void close();

private:
  struct Impl;
  std::unique_ptr<Impl> m_;
};

} // namespace net
} // namespace cvc

#endif // CVC_NET_LOOPBACK_REDIRECT_H
