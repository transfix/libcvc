// cvc::net — the transport-neutral dispatcher: the process-global current client, the injection
// seam, have_http_backend(), and the shared response-header parser. The actual transports live in
// http_client_curl.cpp (native) and http_client_fetch.cpp (wasm); this TU also supplies the
// NullHttpClient + make_default_backend() when NEITHER backend is compiled. See http_client.h.

#include "http_backends.h"

#include <algorithm>
#include <cctype>
#include <cvc/net/http_client.h>
#include <cvc/net/net_policy.h>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

namespace cvc {
namespace net {

namespace {

// The current client and its guard. The client starts unset and is lazily initialised to the
// compiled default on first use, so no static-init-order dependency on the backend TU. A reader
// copies the shared_ptr out under the lock (current_client) — holding a ref-count — so a concurrent
// set_http_client that swaps the cell cannot destroy the client mid-send().
std::mutex &client_mutex() {
  static std::mutex m;
  return m;
}
std::shared_ptr<HttpClient> &client_cell() {
  static std::shared_ptr<HttpClient> c;
  return c;
}
std::shared_ptr<HttpClient> current_client() {
  std::lock_guard<std::mutex> lock(client_mutex());
  std::shared_ptr<HttpClient> &c = client_cell();
  if (!c)
    c = detail::make_default_backend();
  return c;
}

// Case-insensitive ASCII equality of a method token against a literal.
bool method_ieq(const std::string &m, const char *lit) {
  std::size_t i = 0;
  for (; i < m.size() && lit[i]; ++i) {
    char a = m[i];
    if (a >= 'a' && a <= 'z')
      a = static_cast<char>(a - 'a' + 'A');
    char b = lit[i];
    if (b >= 'a' && b <= 'z')
      b = static_cast<char>(b - 'a' + 'A');
    if (a != b)
      return false;
  }
  return i == m.size() && lit[i] == '\0';
}

// GET and HEAD must not carry a request body: XHR (the wasm backend) silently drops it, and libcurl
// mis-sends it — so we strip it centrally and BOTH targets behave identically.
bool method_forbids_body(const std::string &m) {
  return method_ieq(m, "GET") || method_ieq(m, "HEAD");
}

// A syntactically valid method: a non-empty RFC 7230 "token" (visible ASCII minus separators), and
// short enough for the fetch backend's fixed 32-byte requestMethod buffer (so it is never silently
// truncated on wasm). Rejecting an out-of-spec verb here is uniform across both transports.
bool method_is_valid(const std::string &m) {
  if (m.empty() || m.size() > 31)
    return false;
  for (unsigned char c : m) {
    const bool tchar =
        (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') ||
        std::string("!#$%&'*+-.^_`|~").find(static_cast<char>(c)) != std::string::npos;
    if (!tchar)
      return false;
  }
  return true;
}

// The dispatcher NORMALIZES every request before it reaches a backend, so the curl (native) and
// fetch (wasm) transports see one identical, well-formed request — the "same code on both targets"
// contract this facade promises. Without this the two diverge: an empty method becomes GET on fetch
// but POST-with-body on curl; a GET/HEAD body is sent by curl but dropped by XHR; a >31-char verb
// is truncated only on wasm. We settle all three here, once, rather than in each backend. False
// (with `err` the reply) for a method no backend may send.
bool normalize(HttpRequest &req, HttpResponse &err) {
  if (req.method.empty())
    req.method = "GET"; // the documented default, made explicit so both backends agree
  if (!method_is_valid(req.method)) {
    err.ok = false;
    err.status = 0;
    err.canonical_url = req.url;
    err.error = "cvc::net: invalid HTTP method";
    return false;
  }
  if (method_forbids_body(req.method))
    req.body.clear(); // GET/HEAD carry no body on either transport
  return true;
}

HttpResponse refused(const std::string &url, const std::string &why) {
  HttpResponse r;
  r.ok = false;
  r.status = 0;
  r.canonical_url = url;
  r.error = "cvc::net: refused: " + why;
  return r;
}

bool redirect_status(long status) {
  return status == 301 || status == 302 || status == 303 || status == 307 || status == 308;
}

// Whether the "Name: value" line `line` is a `name` header (case-insensitive).
bool header_named(const std::string &line, const char *name) {
  const std::size_t colon = line.find(':');
  return colon != std::string::npos && method_ieq(line.substr(0, colon), name);
}

// `ref` (a Location value) resolved against the http(s) URL `base` (RFC 3986 section 5.2, without
// dot-segment removal, which only the server reads).
std::string resolve_reference(const std::string &base, const std::string &ref) {
  const std::size_t colon = ref.find(':');
  if (colon != std::string::npos && colon > 0 && ref.find_first_of("/?#") > colon) {
    bool scheme = std::isalpha(static_cast<unsigned char>(ref[0])) != 0;
    for (std::size_t i = 1; i < colon && scheme; ++i) {
      const char c = ref[i];
      scheme = std::isalnum(static_cast<unsigned char>(c)) || c == '+' || c == '-' || c == '.';
    }
    if (scheme)
      return ref; // absolute
  }
  const std::size_t sep = base.find("://");
  if (ref.rfind("//", 0) == 0)
    return base.substr(0, sep + 1) + ref; // network-path reference
  const std::size_t path_at = std::min(base.find_first_of("/?#", sep + 3), base.size());
  const std::string origin = base.substr(0, path_at);
  std::string path = base.substr(path_at);
  path.resize(std::min(path.find('#'), path.size()));
  if (ref.empty())
    return origin + path;
  if (ref[0] == '#')
    return origin + path + ref;
  path.resize(std::min(path.find('?'), path.size()));
  if (ref[0] == '?')
    return origin + (path.empty() ? "/" : path) + ref;
  if (ref[0] == '/')
    return origin + ref;
  if (path.empty())
    path = "/";
  return origin + path.substr(0, path.rfind('/') + 1) + ref;
}

// After `r`, the reply to hop `hops` of a policy-checked request `req` (follow: the caller's
// follow_redirects): true with `req` turned into the next hop's request, or false with `r` the
// answer (an error now, where the redirect may not be followed).
bool next_hop(HttpRequest &req, HttpResponse &r, long hops, bool follow, const NetPolicy &policy) {
  if (!follow || !r.ok || !redirect_status(r.status))
    return false;
  const std::string location = response_header(r.headers, "Location");
  if (location.empty())
    return false;
  if (hops >= req.max_redirects) {
    r.ok = false;
    r.error = "cvc::net: more than " + std::to_string(req.max_redirects) + " redirects";
    return false;
  }
  const std::string next = resolve_reference(req.url, location);
  std::string why;
  if (!policy.allows_url(next, &why)) {
    r = refused(next, "a redirect from " + req.url + ": " + why);
    return false;
  }
  if (parse_origin(next) != parse_origin(req.url))
    std::erase_if(req.headers, [](const std::string &h) {
      return header_named(h, "Authorization") || header_named(h, "Cookie") ||
             header_named(h, "Proxy-Authorization");
    });
  const bool to_get = (r.status == 303 && !method_ieq(req.method, "HEAD")) ||
                      ((r.status == 301 || r.status == 302) && method_ieq(req.method, "POST"));
  if (to_get) {
    req.method = "GET";
    req.body.clear();
    std::erase_if(req.headers, [](const std::string &h) {
      return header_named(h, "Content-Type") || header_named(h, "Content-Length");
    });
  }
  req.url = next;
  return true;
}

// start(req, policy, done) on a transport that does not follow redirects itself: each hop is
// started from the last one's reply, and checked by next_hop as send(req, policy) checks it.
class PolicyTransfer : public HttpTransfer, public std::enable_shared_from_this<PolicyTransfer> {
public:
  PolicyTransfer(std::shared_ptr<HttpClient> client, HttpRequest req,
                 std::shared_ptr<const NetPolicy> policy, HttpDone done)
      : client_(std::move(client)), req_(std::move(req)), policy_(std::move(policy)),
        done_(std::move(done)), follow_(req_.follow_redirects) {
    req_.follow_redirects = false; // every hop comes back here to be checked
  }

  void hop() {
    const unsigned id = ++hop_id_;
    std::shared_ptr<PolicyTransfer> self = shared_from_this();
    std::shared_ptr<HttpTransfer> t =
        client_->start(req_, [self](HttpResponse r) { self->reply(std::move(r)); });
    if (hop_id_ == id && done_) // not answered in place
      current_ = std::move(t);
  }

  void abort() override {
    HttpDone dropped = std::move(done_);
    done_ = nullptr;
    if (std::shared_ptr<HttpTransfer> t = std::move(current_))
      t->abort();
  }

private:
  void reply(HttpResponse r) {
    current_.reset();
    if (!done_)
      return; // aborted
    if (next_hop(req_, r, hops_, follow_, *policy_)) {
      ++hops_;
      hop();
      return;
    }
    HttpDone done = std::move(done_);
    done_ = nullptr;
    done(std::move(r));
  }

  std::shared_ptr<HttpClient> client_;
  HttpRequest req_;
  std::shared_ptr<const NetPolicy> policy_;
  HttpDone done_;
  bool follow_;
  long hops_ = 0;
  unsigned hop_id_ = 0;
  std::shared_ptr<HttpTransfer> current_;
};

} // namespace

std::shared_ptr<HttpTransfer> HttpClient::start(const HttpRequest &req, HttpDone done) {
  HttpResponse r = send(req);
  if (done)
    done(std::move(r));
  return nullptr;
}

HttpResponse send(const HttpRequest &in) {
  HttpRequest req = in; // a working copy we may normalize
  HttpResponse err;
  if (!normalize(req, err))
    return err;
  return current_client()->send(req);
}

HttpResponse send(const HttpRequest &in, const NetPolicy &policy) {
  HttpRequest req = in;
  HttpResponse err;
  if (!normalize(req, err))
    return err;
  std::string why;
  if (!policy.allows_url(req.url, &why))
    return refused(req.url, why);
  const std::shared_ptr<HttpClient> client = current_client();
  if (client->follows_redirects_itself())
    return client->send(req);
  const bool follow = req.follow_redirects;
  req.follow_redirects = false; // every hop comes back here to be checked
  for (long hops = 0;; ++hops) {
    HttpResponse r = client->send(req);
    if (!next_hop(req, r, hops, follow, policy))
      return r;
  }
}

std::shared_ptr<HttpTransfer> start(const HttpRequest &in, HttpDone done) {
  HttpRequest req = in;
  HttpResponse err;
  if (!normalize(req, err)) {
    if (done)
      done(std::move(err));
    return nullptr;
  }
  return current_client()->start(req, std::move(done));
}

std::shared_ptr<HttpTransfer> start(const HttpRequest &in, std::shared_ptr<const NetPolicy> policy,
                                    HttpDone done) {
  if (!policy)
    return start(in, std::move(done));
  HttpRequest req = in;
  HttpResponse err;
  std::string why;
  if (!normalize(req, err) || !policy->allows_url(req.url, &why)) {
    if (err.error.empty())
      err = refused(req.url, why);
    if (done)
      done(std::move(err));
    return nullptr;
  }
  std::shared_ptr<HttpClient> client = current_client();
  if (client->follows_redirects_itself())
    return client->start(req, std::move(done));
  auto t = std::make_shared<PolicyTransfer>(std::move(client), std::move(req), std::move(policy),
                                            std::move(done));
  t->hop();
  return t;
}

bool http_starts_async() { return current_client()->starts_async(); }

std::string response_header(const std::vector<std::string> &lines, const std::string &name) {
  for (const std::string &line : lines) {
    const std::size_t colon = line.find(':');
    if (colon == std::string::npos || !method_ieq(line.substr(0, colon), name.c_str()))
      continue;
    const std::size_t b = line.find_first_not_of(" \t", colon + 1);
    if (b == std::string::npos)
      return std::string();
    const std::size_t e = line.find_last_not_of(" \t\r");
    return line.substr(b, e - b + 1);
  }
  return std::string();
}

void set_http_client(std::unique_ptr<HttpClient> client) {
  std::lock_guard<std::mutex> lock(client_mutex());
  if (client)
    client_cell() = std::shared_ptr<HttpClient>(std::move(client));
  else
    client_cell() = std::shared_ptr<HttpClient>(detail::make_default_backend()); // restore default
}

bool have_http_backend() {
  // Reflects the COMPILED capability, independent of any injected client: ask a fresh default
  // backend (the backend ctors are trivial). Null → false; curl/fetch → true.
  return detail::make_default_backend()->is_real();
}

std::vector<std::string> parse_response_header_blob(const std::string &blob) {
  std::vector<std::string> out;
  std::string line;
  auto flush = [&]() {
    // Drop the "HTTP/… <status>" status line(s) and blank lines; keep "Name: value" header lines.
    if (!line.empty() && line.rfind("HTTP/", 0) != 0)
      out.push_back(line);
    line.clear();
  };
  for (char ch : blob) {
    if (ch == '\n')
      flush();
    else if (ch != '\r')
      line.push_back(ch);
  }
  flush(); // a trailing header with no final newline
  return out;
}

// The no-backend build: the null transport plus the make_default_backend() the linker resolves when
// neither CVC_NET_HAVE_CURL nor CVC_NET_HAVE_FETCH is defined.
#if !defined(CVC_NET_HAVE_CURL) && !defined(CVC_NET_HAVE_FETCH)

namespace {
class NullHttpClient : public HttpClient {
public:
  bool is_real() const override { return false; }
  HttpResponse send(const HttpRequest &req) override {
    HttpResponse r;
    r.ok = false;
    r.status = 0;
    r.canonical_url = req.url;
    r.error = "cvc::net: no HTTP backend compiled in this build";
    return r;
  }
};
} // namespace

namespace detail {
std::unique_ptr<HttpClient> make_default_backend() { return std::make_unique<NullHttpClient>(); }
} // namespace detail

#endif // no backend

} // namespace net
} // namespace cvc
