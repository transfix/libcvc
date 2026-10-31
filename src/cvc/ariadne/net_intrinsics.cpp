// Ariadne — async HTTP host intrinsics for the state_exec program lanes (roadmap §13.8). See
// net_intrinsics.h. The (http-get*) verbs are gated on a compiled cvc::net backend
// (cvc::net::have_http_backend()). A credential's token lives only in C++ here: it is added to the
// request on the worker that sends it and taken out of the reply there, so no value_t (and so no
// program variable, state node or log line) ever holds one.

#include <algorithm>
#include <array>
#include <atomic>
#include <cctype>
#include <cstdint>
#include <cstdio>
#include <cvc/ariadne/ariadne.h> // register_action_intrinsics
#include <cvc/ariadne/net_intrinsics.h>
#include <cvc/ariadne/uri.h> // resolve() — the generic (fetch uri) async resolver
#include <cvc/core/app.h>
#include <cvc/net/credentials.h>
#include <cvc/net/http_client.h>
#include <cvc/net/net_policy.h>
#include <cvc/net/ws_client.h>
#include <cvc/state/state_exec/async_host.h> // register_async_fn — the verbs are async functions
#include <cvc/state/state_exec/builtins.h>   // percent_encode — the query string
#include <cvc/state/state_exec/types.h>      // value_t, make_dict/make_list/make_bytes
#include <map>
#include <mutex>
#include <optional>
#include <random>
#include <span>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace cvc {
namespace ariadne {

namespace se = cvc::state_exec;

namespace {

// Marshal a cvc::net::HttpResponse into the DSL reply dict. EVERY key is populated on every path
// (get-attr throws on a missing key), so an error response is a well-formed dict too. The body is
// `bytes` (opaque octets, the sanctioned home for an HTTP body — the state_exec bytes track).
se::value_t marshal_response(const cvc::net::HttpResponse &r, bool captured = false) {
  std::vector<se::value_t> headers;
  headers.reserve(r.headers.size());
  std::vector<std::pair<std::string, se::value_t>> map; // lowercased name -> value, repeats joined
  for (const std::string &h : r.headers) {
    headers.push_back(se::value_t(h));
    const std::size_t colon = h.find(':');
    if (colon == std::string::npos)
      continue;
    std::string name = h.substr(0, colon);
    std::transform(name.begin(), name.end(), name.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    const std::string value = cvc::net::response_header({h}, name);
    auto it =
        std::find_if(map.begin(), map.end(), [&](const auto &kv) { return kv.first == name; });
    if (it == map.end())
      map.emplace_back(std::move(name), se::value_t(value));
    else
      it->second = se::value_t(std::get<std::string>(it->second.v) + ", " + value);
  }
  return se::make_dict({
      {"ok", se::value_t(r.ok)},
      {"status", se::value_t(static_cast<int64_t>(r.status))},
      {"body", se::make_bytes(r.body)},
      {"url", se::value_t(r.canonical_url)},
      {"headers", se::make_list(std::move(headers))},
      {"header_map", se::make_dict(std::move(map))},
      {"content_type", se::value_t(cvc::net::response_header(r.headers, "Content-Type"))},
      {"captured", se::value_t(captured)},
      {"error", se::value_t(r.error)},
  });
}

se::value_t marshal_error(const std::string &url, const std::string &message) {
  cvc::net::HttpResponse r;
  r.ok = false;
  r.status = 0;
  r.canonical_url = url;
  r.error = message;
  return marshal_response(r);
}

// arg N as a string ("" if absent / not a string).
std::string arg_string(std::span<const se::value_t> args, std::size_t idx) {
  if (args.size() > idx)
    if (const std::string *s = std::get_if<std::string>(&args[idx].v))
      return *s;
  return std::string();
}

// Stringify a scalar option value for a header/query/form field: a string passes through, a bytes
// value as its raw octets, any other scalar via to_string (so (dict "page" 2) yields "2").
std::string field_to_string(const se::value_t &v) {
  if (const std::string *s = std::get_if<std::string>(&v.v))
    return *s;
  if (const se::bytes_value *b = std::get_if<se::bytes_value>(&v.v))
    return b->data;
  return se::to_string(v);
}

// One field capture_auth keeps from a reply.
struct Capture {
  std::string name;            // the credential it becomes
  std::string field = "token"; // object keys joined by '.'
  bool optional = false;       // a reply without it is no error (a refresh need not rotate it)
};

// A credential a form or header value names, (dict "credential" NAME). The option is encoded with
// `placeholder` in its place; the host puts the token there as the request is sent.
struct CredentialValue {
  std::string placeholder;
  std::string name;
  bool form = false; // percent-encoded into a form body (else raw, into a header line)
};

// The credential options of one request (net_intrinsics.h): which credentials to send, as the
// Authorization header (auth) or as form and header values, and which to keep from the reply.
// Names only; the tokens stay in the CredentialStore.
struct RequestAuth {
  std::string use;                     // auth: NAME ("" = none)
  std::vector<Capture> captures;       // capture_auth
  std::vector<CredentialValue> values; // (dict "credential" NAME) values
  bool any() const { return !use.empty() || !captures.empty() || !values.empty(); }
};

std::string credential_name(const se::value_t &v, const char *option) {
  const std::string *s = std::get_if<std::string>(&v.v);
  if (!s || !cvc::net::CredentialStore::valid_name(*s))
    throw std::runtime_error(std::string("http request '") + option +
                             "' option must name a credential (1-64 of A-Z a-z 0-9 . _ -)");
  return *s;
}

// The name a credential value, (dict "credential" NAME), names; null when `v` is not one.
const std::string *credential_value(const se::value_t &v) {
  const se::dict_ptr *d = std::get_if<se::dict_ptr>(&v.v);
  if (!d || (*d)->size() != 1 || (*d)->front().first != "credential")
    return nullptr;
  const std::string *name = std::get_if<std::string>(&(*d)->front().second.v);
  if (!name || !cvc::net::CredentialStore::valid_name(*name))
    throw std::runtime_error(
        "http request: a credential value is (dict \"credential\" NAME), NAME 1-64 of A-Z a-z 0-9 "
        ". _ -");
  return name;
}

// A placeholder for credential `name` in this request: letters and digits only (so percent-encoding
// leaves it alone), and random, so no other byte of the request can be mistaken for it.
std::string placeholder(RequestAuth &auth, const std::string &name, bool form) {
  static thread_local std::mt19937_64 rng{std::random_device{}()};
  char buf[40];
  std::snprintf(buf, sizeof buf, "CVCCREDENTIAL%016llx%016llx",
                static_cast<unsigned long long>(rng()), static_cast<unsigned long long>(rng()));
  auth.values.push_back({buf, name, form});
  return buf;
}

// Percent-encode an ordered dict into "k1=v1&k2=v2" (keys AND values encoded). Shared by the query
// string and the application/x-www-form-urlencoded body; dict order is preserved. A credential
// value becomes a placeholder in a form; a query refuses one (a token does not belong in a URL,
// which servers and proxies log).
std::string encode_form_pairs(const se::dict_ptr &d, RequestAuth &auth, bool form) {
  std::string out;
  for (const auto &kv : *d) {
    if (!out.empty())
      out.push_back('&');
    out += se::percent_encode(kv.first);
    out.push_back('=');
    if (const std::string *name = credential_value(kv.second)) {
      if (!form)
        throw std::runtime_error("http request: a credential may not go in the query (a URL is "
                                 "logged); send it in a form or a header");
      out += placeholder(auth, *name, true);
    } else {
      out += se::percent_encode(field_to_string(kv.second));
    }
  }
  return out;
}

// Append a query string to a URL, inserted before any '#fragment' and joined with '?' or '&'.
std::string url_with_query(std::string url, const std::string &qs) {
  if (qs.empty())
    return url;
  std::string frag;
  if (const std::size_t hash = url.find('#'); hash != std::string::npos) {
    frag = url.substr(hash);
    url.resize(hash);
  }
  url.push_back(url.find('?') == std::string::npos ? '?' : '&');
  url += qs;
  return url + frag;
}

// Collect request headers from a "headers" option: a dict ({Name value} -> "Name: value" lines,
// values stringified, a credential value as a placeholder) OR a list of verbatim "Name: value"
// strings (the escape hatch for a repeated header name or byte-exact control). Anything else
// contributes nothing.
void collect_headers(const se::value_t &v, std::vector<std::string> &out, RequestAuth &auth) {
  if (const se::dict_ptr *d = std::get_if<se::dict_ptr>(&v.v)) {
    for (const auto &kv : **d) {
      const std::string *name = credential_value(kv.second);
      out.push_back(kv.first + ": " +
                    (name ? placeholder(auth, *name, false) : field_to_string(kv.second)));
    }
  } else if (const se::list_ptr *lp = std::get_if<se::list_ptr>(&v.v)) {
    for (const se::value_t &e : **lp)
      if (const std::string *h = std::get_if<std::string>(&e.v))
        out.push_back(*h);
  }
}

// Is a Content-Type header already present (case-insensitive) among the collected lines?
bool has_content_type(const std::vector<std::string> &headers) {
  static const std::string kName = "content-type:";
  for (const std::string &h : headers) {
    if (h.size() < kName.size())
      continue;
    bool match = true;
    for (std::size_t i = 0; i < kName.size(); ++i) {
      char c = h[i];
      if (c >= 'A' && c <= 'Z')
        c = static_cast<char>(c + 32);
      if (c != kName[i]) {
        match = false;
        break;
      }
    }
    if (match)
      return true;
  }
  return false;
}

Capture parse_one_capture(const se::value_t &v) {
  const se::dict_ptr *d = std::get_if<se::dict_ptr>(&v.v);
  if (!d)
    throw std::runtime_error(
        "http request 'capture_auth' option must be a dict of name, field and optional, or a "
        "list of them");
  Capture c;
  for (const auto &kv : **d) {
    if (kv.first == "name") {
      c.name = credential_name(kv.second, "capture_auth");
    } else if (kv.first == "field") {
      const std::string *f = std::get_if<std::string>(&kv.second.v);
      if (!f || f->empty())
        throw std::runtime_error("http request 'capture_auth' field must be a non-empty string");
      c.field = *f;
    } else if (kv.first == "optional") {
      const bool *b = std::get_if<bool>(&kv.second.v);
      if (!b)
        throw std::runtime_error("http request 'capture_auth' optional must be #t or #f");
      c.optional = *b;
    }
  }
  if (c.name.empty())
    throw std::runtime_error("http request 'capture_auth' option needs a name");
  return c;
}

// capture_auth: one {name field optional} dict, or a list of them (one credential each).
void parse_capture(const se::value_t &v, RequestAuth &auth) {
  if (const se::list_ptr *lp = std::get_if<se::list_ptr>(&v.v)) {
    for (const se::value_t &e : **lp)
      auth.captures.push_back(parse_one_capture(e));
    if (auth.captures.empty())
      throw std::runtime_error("http request 'capture_auth' list is empty");
  } else {
    auth.captures.push_back(parse_one_capture(v));
  }
  for (std::size_t i = 0; i < auth.captures.size(); ++i)
    for (std::size_t j = i + 1; j < auth.captures.size(); ++j)
      if (auth.captures[i].name == auth.captures[j].name)
        throw std::runtime_error("http request 'capture_auth' names credential '" +
                                 auth.captures[i].name + "' twice");
}

// Apply an options dict onto `req`: "query" (dict -> percent-encoded query string appended to the
// URL), "headers" (dict or list), "body" (string or bytes -> raw body), "form" (dict ->
// x-www-form-urlencoded body + a default Content-Type unless the caller set one). Unknown keys are
// ignored (forward-compatible: json/timeout/... may come later); "auth", "capture_auth" and
// credential values go to `auth`. A misuse (options not a dict, a wrong value type, or both "body"
// and "form") throws a clear error; the verb runs on the scheduler thread, so this surfaces as an
// ordinary DSL evaluation error, not a hang.
void apply_http_options(cvc::net::HttpRequest &req, const se::value_t &opts, RequestAuth &auth) {
  const se::dict_ptr *dp = std::get_if<se::dict_ptr>(&opts.v);
  if (!dp)
    throw std::runtime_error("http request options must be a dict");
  const se::value_t *query = nullptr, *headers = nullptr, *body = nullptr, *form = nullptr;
  for (const auto &kv : **dp) {
    if (kv.first == "query")
      query = &kv.second;
    else if (kv.first == "headers")
      headers = &kv.second;
    else if (kv.first == "body")
      body = &kv.second;
    else if (kv.first == "form")
      form = &kv.second;
    else if (kv.first == "auth")
      auth.use = credential_name(kv.second, "auth");
    else if (kv.first == "capture_auth")
      parse_capture(kv.second, auth);
  }
  if (headers)
    collect_headers(*headers, req.headers, auth);
  if (query) {
    const se::dict_ptr *qd = std::get_if<se::dict_ptr>(&query->v);
    if (!qd)
      throw std::runtime_error("http request 'query' option must be a dict");
    req.url = url_with_query(std::move(req.url), encode_form_pairs(*qd, auth, false));
  }
  if (body && form)
    throw std::runtime_error("http request options: 'body' and 'form' are mutually exclusive");
  if (body) {
    if (const std::string *s = std::get_if<std::string>(&body->v))
      req.body = *s;
    else if (const se::bytes_value *b = std::get_if<se::bytes_value>(&body->v))
      req.body = b->data;
    else
      throw std::runtime_error("http request 'body' option must be a string or bytes");
  }
  if (form) {
    const se::dict_ptr *fd = std::get_if<se::dict_ptr>(&form->v);
    if (!fd)
      throw std::runtime_error("http request 'form' option must be a dict");
    req.body = encode_form_pairs(*fd, auth, true);
    if (!has_content_type(req.headers))
      req.headers.push_back("Content-Type: application/x-www-form-urlencoded");
  }
}

// Blank `secret` wherever the reply carries it.
void scrub(cvc::net::HttpResponse &r, const std::string &secret) {
  cvc::net::redact(r.body, secret);
  for (std::string &h : r.headers)
    cvc::net::redact(h, secret);
  cvc::net::redact(r.canonical_url, secret);
  cvc::net::redact(r.error, secret);
}

void wipe(std::string &secret) {
  std::fill(secret.begin(), secret.end(), '\0');
  secret.clear();
}

// What one request came back with, before it is a value_t.
struct Outcome {
  cvc::net::HttpResponse reply;
  bool captured = false;
};

Outcome reply_error(const std::string &url, const std::string &message) {
  Outcome o;
  o.reply.ok = false;
  o.reply.status = 0;
  o.reply.canonical_url = url;
  o.reply.error = message;
  return o;
}

// One request with its credentials put in (prepare), or the reply it fails with before it is made.
// `sent` holds the tokens put in, to blank any echo of them in the reply.
struct Prepared {
  cvc::net::HttpRequest req;
  std::optional<cvc::net::Origin> origin;
  std::vector<std::string> sent;
  std::optional<Outcome> failed;
};

// The request as sent: the credentials `auth` names put in (the Authorization header, form and
// header values). Nothing is put in when the request fails here.
Prepared prepare(cvc::net::HttpRequest req, const RequestAuth &auth, const NetAccess &access) {
  Prepared p;
  if (auth.any()) {
    std::string why;
    p.origin = cvc::net::parse_origin(req.url, &why);
    if (!p.origin) {
      p.failed = reply_error(req.url, "http: " + why);
      return p;
    }
    req.follow_redirects = false; // a credential never follows a redirect to another resource
    if (access.policy && !access.policy->allows(*p.origin)) {
      p.failed = reply_error(req.url, "http: refused: " + p.origin->str() +
                                          " is not an origin this host allows");
      return p;
    }
  }
  std::vector<std::string> &sent = p.sent;
  const auto token_for = [&](const std::string &name) -> std::optional<std::string> {
    std::optional<std::string> token = access.credentials->token_for(name, *p.origin);
    if (token)
      sent.push_back(*token);
    return token;
  };
  const auto no_credential = [&](const std::string &name) {
    for (std::string &t : sent)
      wipe(t);
    sent.clear();
    for (std::string &h : req.headers)
      wipe(h);
    wipe(req.body);
    p.failed = reply_error(req.url, "http: no credential '" + name + "' for " + p.origin->str() +
                                        " (sign in first)");
  };
  if (!auth.use.empty()) {
    const std::optional<std::string> token = token_for(auth.use);
    if (!token) {
      no_credential(auth.use);
      return p;
    }
    req.headers.push_back("Authorization: Bearer " + *token);
  }
  for (const CredentialValue &v : auth.values) {
    const std::optional<std::string> token = token_for(v.name);
    if (!token) {
      no_credential(v.name);
      return p;
    }
    if (v.form) {
      const std::string encoded = se::percent_encode(*token);
      if (encoded != *token)
        sent.push_back(encoded); // a reply may echo the form's spelling of it
      if (const std::size_t at = req.body.find(v.placeholder); at != std::string::npos)
        req.body.replace(at, v.placeholder.size(), encoded);
    } else {
      for (std::string &h : req.headers)
        if (const std::size_t at = h.find(v.placeholder); at != std::string::npos)
          h.replace(at, v.placeholder.size(), *token);
    }
  }
  p.req = std::move(req);
  return p;
}

// The reply `r` to the prepared request `p`: the request's copies of its credentials blanked, any
// echo of them in the reply too, and the ones `auth` captures taken out of it, so none reaches a
// value_t.
Outcome complete(Prepared &p, cvc::net::HttpResponse r, const RequestAuth &auth,
                 const NetAccess &access) {
  for (std::string &h : p.req.headers)
    wipe(h);
  wipe(p.req.body);
  for (std::string &t : p.sent) {
    scrub(r, t);
    wipe(t);
  }
  bool captured = false;
  if (!auth.captures.empty() && r.ok && r.status >= 200 && r.status < 300) {
    // Take every field first, so each token is blanked wherever the reply carries it, whichever
    // capture it belongs to, before any is kept.
    std::vector<std::pair<const Capture *, std::string>> taken;
    std::vector<std::string> problems;
    for (const Capture &c : auth.captures) {
      std::string token;
      if (cvc::net::take_json_string(r.body, c.field, token))
        taken.emplace_back(&c, std::move(token));
      else if (!c.optional)
        problems.push_back("the reply has no JSON string at '" + c.field + "'");
    }
    for (const auto &t : taken)
      scrub(r, t.second);
    captured = problems.empty();
    for (auto &[c, token] : taken) {
      std::string why;
      if (!access.credentials->capture(c->name, *p.origin, token, &why))
        captured = false;
      if (!why.empty())
        problems.push_back(c->name + ": " + why);
      wipe(token);
    }
    if (!problems.empty()) {
      std::string joined;
      for (const std::string &pr : problems)
        joined += (joined.empty() ? "" : "; ") + pr;
      r.error = "capture_auth: " + joined;
    }
  }
  return Outcome{std::move(r), captured};
}

// One request, sent in place (blocking): prepare, send, complete.
Outcome perform(cvc::net::HttpRequest req, const RequestAuth &auth, const NetAccess &access) {
  Prepared p = prepare(std::move(req), auth, access);
  if (p.failed)
    return std::move(*p.failed);
  cvc::net::HttpResponse r =
      access.policy ? cvc::net::send(p.req, *access.policy) : cvc::net::send(p.req);
  return complete(p, std::move(r), auth, access);
}

// The kernel of a fully-built request: it runs on the app's background task pool (OFF the
// scheduler thread) and returns the marshalled response dict. A failure to send is a reply with
// ok #f too, as the verbs document: only an HTTP reply's own failure is data, but these verbs have
// always described a failed send that way, and a program checks `ok` either way.
se::async_kernel http_kernel(cvc::net::HttpRequest req, RequestAuth auth, NetAccess access) {
  return [req = std::move(req), auth = std::move(auth),
          access = std::move(access)](se::async_call &) -> se::value_t {
    try {
      const Outcome out = perform(req, auth, access);
      return marshal_response(out.reply, out.captured);
    } catch (const std::exception &e) {
      return marshal_error(req.url, std::string("http: ") + e.what());
    } catch (...) {
      return marshal_error(req.url, "http: unknown error");
    }
  };
}

// Settle a started call with its reply and post it, unless the call was cancelled first: then its
// host may be gone (async_start's contract).
void settle_started(se::async_host &host, const std::shared_ptr<se::async_call> &call,
                    se::value_t value) {
  if (!call->cancel_requested() && call->settle(std::move(value), nullptr))
    host.finish(call);
}

// The same request started rather than sent (L4): where the transport starts requests without
// blocking (the browser's fetch in a build without threads, cvc::net::http_starts_async), the
// calling program parks on it and the frame loop goes on; the browser's event loop delivers the
// reply, which is completed and marshalled there and posted to the scheduler. A killed or timed-out
// call aborts the fetch. Elsewhere the kernel runs as before.
se::async_work http_work(cvc::net::HttpRequest req, RequestAuth auth, NetAccess access,
                         se::async_host &host) {
  se::async_work work;
  const bool starts = cvc::net::http_starts_async();
  work.kernel = http_kernel(starts ? req : std::move(req), auth, access);
  if (!starts)
    return work;
  work.start = [req = std::move(req), auth = std::move(auth), access = std::move(access),
                &host](const std::shared_ptr<se::async_call> &call) {
    auto p = std::make_shared<Prepared>(prepare(req, auth, access));
    if (p->failed) {
      settle_started(host, call, marshal_response(p->failed->reply, p->failed->captured));
      return;
    }
    const std::string url = p->req.url;
    cvc::net::HttpDone done = [p, auth, access, url, call, &host](cvc::net::HttpResponse r) {
      try {
        const Outcome out = complete(*p, std::move(r), auth, access);
        settle_started(host, call, marshal_response(out.reply, out.captured));
      } catch (const std::exception &e) {
        settle_started(host, call, marshal_error(url, std::string("http: ") + e.what()));
      } catch (...) {
        settle_started(host, call, marshal_error(url, "http: unknown error"));
      }
    };
    std::shared_ptr<cvc::net::HttpTransfer> transfer =
        cvc::net::start(p->req, access.policy, std::move(done));
    if (transfer)
      call->on_cancel([weak = std::weak_ptr<cvc::net::HttpTransfer>(transfer)] {
        if (std::shared_ptr<cvc::net::HttpTransfer> t = weak.lock())
          t->abort();
      });
  };
  return work;
}

bool is_live(const NetAccess &access) { return !access.live || access.live->load(); }

// Checks made where the verb is called (the scheduler thread), so a misuse is an ordinary DSL
// error: the network turned off, or a credential option the host cannot honour.
void check_request(const char *verb, const cvc::net::HttpRequest &req, const RequestAuth &auth,
                   const NetAccess &access) {
  if (!is_live(access))
    throw std::runtime_error(std::string(verb) + ": the host has turned the network off");
  if (auth.any() && !access.credentials)
    throw std::runtime_error(std::string(verb) +
                             ": this host keeps no credentials, so it takes no auth, "
                             "capture_auth or credential value");
  if (!auth.use.empty())
    for (const std::string &h : req.headers)
      if (const std::size_t colon = h.find(':'); colon != std::string::npos) {
        std::string name = h.substr(0, colon);
        std::transform(name.begin(), name.end(), name.begin(),
                       [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        if (name == "authorization")
          throw std::runtime_error(std::string(verb) +
                                   ": 'auth' and an Authorization header are exclusive");
      }
}

// (http-get URL [OPTS]) — a GET; OPTS is an options dict (query/headers/auth/capture_auth — see
// apply_http_options).
cvc::net::HttpRequest build_get_request(std::span<const se::value_t> args, RequestAuth &auth) {
  cvc::net::HttpRequest req;
  req.method = "GET";
  req.url = arg_string(args, 0);
  if (args.size() > 1)
    apply_http_options(req, args[1], auth);
  return req;
}

// (http-request METHOD URL [OPTS]) — any method (GET/PUT/POST/PATCH/DELETE/…). OPTS is an options
// dict: query/headers/body/form/auth/capture_auth (see apply_http_options), so a POST carries a
// body + headers, e.g. (http-request "POST" url (dict "auth" "txwtf" "form" (dict "grip" 1))).
cvc::net::HttpRequest build_method_request(std::span<const se::value_t> args, RequestAuth &auth) {
  cvc::net::HttpRequest req;
  req.method = arg_string(args, 0);
  req.url = arg_string(args, 1);
  if (args.size() > 2)
    apply_http_options(req, args[2], auth);
  return req;
}

// Marshal a resolver UriResult into the DSL reply dict (scheme-agnostic — no HTTP status/headers).
// The body is `bytes` (opaque octets), consistent with (http-get)'s body.
se::value_t marshal_uri_result(const UriResult &r) {
  return se::make_dict({
      {"ok", se::value_t(r.ok)},
      {"body", se::make_bytes(r.content)},
      {"url", se::value_t(r.canonical)},
      {"error", se::value_t(r.error)},
  });
}

// The generic async resolve (any registered scheme: file/state/cvc/http): the SYNCHRONOUS
// resolve() as a kernel on the app's background task pool (OFF the scheduler thread), returning
// the marshalled dict, for arbitrary URIs. resolve() is app-free and has its own barrier, so it is
// safe there. A failure is a reply with ok #f, as for the http verbs.
se::async_kernel uri_fetch_kernel(const std::string &uri, const std::string &base) {
  return [uri, base](se::async_call &) -> se::value_t {
    try {
      return marshal_uri_result(resolve(uri, base));
    } catch (const std::exception &e) {
      UriResult er;
      er.ok = false;
      er.canonical = uri;
      er.error = std::string("fetch: ") + e.what();
      return marshal_uri_result(er);
    }
  };
}

// (fetch URI [BASE])'s work: the kernel above, or, for a URI whose scheme resolves without
// blocking (an http(s) URI, resolve_async) where the transport starts requests that way, a start
// that parks the program on the resolve while the frame loop goes on (http_work's L4).
se::async_work uri_fetch_work(std::span<const se::value_t> args, se::async_host &host) {
  std::string uri;
  if (!args.empty())
    if (const std::string *u = std::get_if<std::string>(&args[0].v))
      uri = *u;
  std::string base; // optional second arg: the base for a relative URI
  if (args.size() > 1)
    if (const std::string *b = std::get_if<std::string>(&args[1].v))
      base = *b;
  se::async_work work;
  work.kernel = uri_fetch_kernel(uri, base);
  if (!cvc::net::http_starts_async() || !has_uri_async_handler(parse_uri(uri).scheme))
    return work;
  work.start = [uri, base, &host](const std::shared_ptr<se::async_call> &call) {
    std::function<void()> stop = resolve_async(uri, base, [call, &host](UriResult r) {
      settle_started(host, call, marshal_uri_result(r));
    });
    if (stop)
      call->on_cancel(std::move(stop));
  };
  return work;
}

// The credential verbs and (net-allowed? URL), plain functions on the scheduler thread.
void register_net_verbs(const std::shared_ptr<se::environment> &env, const NetAccess &access) {
  const auto live = [access](const char *verb) {
    if (!is_live(access))
      throw std::runtime_error(std::string(verb) + ": the host has turned the network off");
  };
  // (net-allowed? URL): whether a request to URL may be made here.
  se::builtins::register_fn(env, "net-allowed?",
                            [access, live](std::span<const se::value_t> args) -> se::value_t {
                              live("net-allowed?");
                              const std::string url = arg_string(args, 0);
                              if (access.policy)
                                return se::value_t(access.policy->allows_url(url));
                              return se::value_t(cvc::net::parse_origin(url).has_value());
                            });
  if (!access.credentials)
    return;
  const std::shared_ptr<cvc::net::CredentialStore> store = access.credentials;
  // (net-auth-has? NAME [URL]): whether the host has credential NAME (for URL's origin).
  se::builtins::register_fn(
      env, "net-auth-has?", [store, live](std::span<const se::value_t> args) -> se::value_t {
        live("net-auth-has?");
        const std::string name = arg_string(args, 0);
        if (args.size() < 2)
          return se::value_t(store->has(name));
        const std::optional<cvc::net::Origin> o = cvc::net::parse_origin(arg_string(args, 1));
        return se::value_t(o && store->has(name, *o));
      });
  // (net-auth-forget NAME): drop credential NAME, from the host's file too. #t if there was one.
  se::builtins::register_fn(env, "net-auth-forget",
                            [store, live](std::span<const se::value_t> args) -> se::value_t {
                              live("net-auth-forget");
                              std::string error;
                              const bool dropped = store->forget(arg_string(args, 0), &error);
                              if (!error.empty())
                                throw std::runtime_error("net-auth-forget: " + error);
                              return se::value_t(dropped);
                            });
  // (net-auth-clear): drop every credential.
  se::builtins::register_fn(env, "net-auth-clear",
                            [store, live](std::span<const se::value_t>) -> se::value_t {
                              live("net-auth-clear");
                              std::string error;
                              store->clear(&error);
                              if (!error.empty())
                                throw std::runtime_error("net-auth-clear: " + error);
                              return se::value_t(true);
                            });
}

// --- WebSocket verbs (docs/ARIADNE.md §10.5) ---
//
// A connection is a handle string ("ws:N") in a process-global table, so a program can keep it in
// state and use it from later ticks. (ws-open) connects on the background pool; the rest are plain
// functions on the scheduler thread except (ws-recv), which waits on the pool. In a browser build
// without threads both are started instead, answered by the socket's callbacks (ws_open_work).

constexpr std::size_t kMaxWsConnections = 64;

struct WsTable {
  std::mutex mutex;
  std::map<std::string, std::shared_ptr<cvc::net::WsConnection>> open;
  std::uint64_t next = 1;
};

WsTable &ws_table() {
  static WsTable table;
  return table;
}

std::shared_ptr<cvc::net::WsConnection> ws_lookup(const char *verb,
                                                  std::span<const se::value_t> args) {
  const std::string handle = arg_string(args, 0);
  WsTable &t = ws_table();
  std::lock_guard<std::mutex> lock(t.mutex);
  auto it = t.open.find(handle);
  if (it == t.open.end())
    throw std::runtime_error(std::string(verb) + ": no WebSocket '" + handle +
                             "' (closed, or not one ws-open returned)");
  return it->second;
}

double number_arg(std::span<const se::value_t> args, std::size_t idx, double fallback) {
  if (args.size() > idx) {
    if (const double *d = std::get_if<double>(&args[idx].v))
      return *d;
    if (const int64_t *i = std::get_if<int64_t>(&args[idx].v))
      return static_cast<double>(*i);
  }
  return fallback;
}

// An event as the DSL sees it: {type data code}. Text is a string (for json-parse), binary is
// bytes, a close carries its code and reason, an error its description.
se::value_t marshal_ws_event(const std::optional<cvc::net::WsEvent> &e) {
  if (!e)
    return se::value_t();
  using K = cvc::net::WsEvent::Kind;
  const char *type = e->kind == K::Text     ? "text"
                     : e->kind == K::Binary ? "binary"
                     : e->kind == K::Close  ? "close"
                                            : "error";
  return se::make_dict({
      {"type", se::value_t(std::string(type))},
      {"data", e->kind == K::Binary ? se::make_bytes(e->data) : se::value_t(e->data)},
      {"code", se::value_t(static_cast<int64_t>(e->code))},
  });
}

se::value_t ws_open_reply(bool ok, const std::string &handle, const std::string &url,
                          const std::string &error) {
  return se::make_dict({
      {"ok", se::value_t(ok)},
      {"handle", se::value_t(handle)},
      {"url", se::value_t(url)},
      {"error", se::value_t(error)},
  });
}

// (ws-open URL [OPTS]): OPTS takes "headers", "auth", "query" as the http verbs do, and "timeout"
// (seconds to connect). The origin check uses the http(s) twin of the URL, so --net-allow
// https://h covers wss://h.
struct WsOpen {
  cvc::net::HttpRequest req; // the URL and the handshake's header lines (placeholders unfilled)
  RequestAuth auth;
  double timeout = 15;
  std::string http_url;
};

// The checks made where the verb is called (the scheduler thread), so a misuse is a DSL error.
WsOpen build_ws_open(std::span<const se::value_t> args, const NetAccess &access) {
  WsOpen o;
  o.req.url = arg_string(args, 0);
  if (args.size() > 1) {
    apply_http_options(o.req, args[1], o.auth);
    if (!o.req.body.empty())
      throw std::runtime_error("ws-open: a WebSocket handshake has no body or form");
    if (!o.auth.captures.empty())
      throw std::runtime_error("ws-open: a WebSocket handshake captures no credential");
    if (const se::dict_ptr *dp = std::get_if<se::dict_ptr>(&args[1].v))
      for (const auto &kv : **dp)
        if (kv.first == "timeout") {
          const std::array<se::value_t, 1> one{kv.second};
          o.timeout = number_arg(one, 0, -1);
          if (!(o.timeout > 0))
            throw std::runtime_error("ws-open: 'timeout' must be a positive number of seconds");
        }
  }
  check_request("ws-open", o.req, o.auth, access);
  o.http_url = cvc::net::ws_http_equivalent(o.req.url);
  if (o.http_url == o.req.url)
    throw std::runtime_error("ws-open: a WebSocket URL starts with ws:// or wss://: " + o.req.url);
  return o;
}

// A connect ready to make: the options with the credentials put in, and the tokens put in (to
// blank any echo of them in an error), or the reply it fails with before connecting.
struct WsPrepared {
  cvc::net::WsOptions opts;
  std::vector<std::string> sent;
  std::optional<se::value_t> failed;
};

WsPrepared prepare_ws_open(const WsOpen &o, const NetAccess &access) {
  WsPrepared p;
  std::string why;
  const std::optional<cvc::net::Origin> origin = cvc::net::parse_origin(o.http_url, &why);
  if (!origin) {
    p.failed = ws_open_reply(false, "", o.req.url, "ws-open: " + why);
    return p;
  }
  if (access.policy && !access.policy->allows(*origin)) {
    p.failed =
        ws_open_reply(false, "", o.req.url,
                      "ws-open: refused: " + origin->str() + " is not an origin this host allows");
    return p;
  }
  p.opts.headers = o.req.headers;
  p.opts.connect_timeout_secs = o.timeout;
  const auto fail = [&](const std::string &message) {
    for (std::string &h : p.opts.headers)
      wipe(h);
    for (std::string &t : p.sent)
      wipe(t);
    p.failed = ws_open_reply(false, "", o.req.url, message);
  };
  if (!o.auth.use.empty()) {
    std::optional<std::string> token = access.credentials->token_for(o.auth.use, *origin);
    if (!token) {
      fail("ws-open: no credential '" + o.auth.use + "' for " + origin->str() + " (sign in first)");
      return p;
    }
    p.opts.headers.push_back("Authorization: Bearer " + *token);
    p.sent.push_back(std::move(*token));
  }
  for (const CredentialValue &v : o.auth.values) {
    std::optional<std::string> token = access.credentials->token_for(v.name, *origin);
    if (!token) {
      fail("ws-open: no credential '" + v.name + "' for " + origin->str() + " (sign in first)");
      return p;
    }
    for (std::string &h : p.opts.headers)
      if (const std::size_t at = h.find(v.placeholder); at != std::string::npos)
        h.replace(at, v.placeholder.size(), *token);
    p.sent.push_back(std::move(*token));
  }
  return p;
}

// The reply once the connect is over: the credentials blanked, and the connection, if any, in the
// handle table.
se::value_t finish_ws_open(const WsOpen &o, WsPrepared &p,
                           std::shared_ptr<cvc::net::WsConnection> conn, std::string error) {
  for (std::string &h : p.opts.headers)
    wipe(h);
  for (std::string &t : p.sent) {
    cvc::net::redact(error, t);
    wipe(t);
  }
  if (!conn)
    return ws_open_reply(false, "", o.req.url, error);
  WsTable &t = ws_table();
  std::lock_guard<std::mutex> lock(t.mutex);
  if (t.open.size() >= kMaxWsConnections) {
    conn->close(1013);
    return ws_open_reply(false, "", o.req.url,
                         "ws-open: too many open WebSockets (" + std::to_string(kMaxWsConnections) +
                             "); close some first");
  }
  const std::string handle = "ws:" + std::to_string(t.next++);
  t.open.emplace(handle, std::move(conn));
  return ws_open_reply(true, handle, o.req.url, "");
}

// (ws-open)'s work: connect in a kernel (on the background pool), or, where the backend connects
// without blocking (cvc::net::ws_starts_async: the browser without threads), start the connect
// and let the socket's callbacks post the reply, so the frame loop goes on meanwhile (as
// http_work does for HTTP). A killed or timed-out call abandons the connect.
se::async_work ws_open_work(std::span<const se::value_t> args, const NetAccess &access,
                            se::async_host &host) {
  WsOpen o = build_ws_open(args, access);
  se::async_work work;
  work.kernel = [o, access](se::async_call &) -> se::value_t {
    try {
      WsPrepared p = prepare_ws_open(o, access);
      if (p.failed)
        return std::move(*p.failed);
      std::string error;
      std::shared_ptr<cvc::net::WsConnection> conn =
          cvc::net::ws_connect(o.req.url, p.opts, &error);
      return finish_ws_open(o, p, std::move(conn), std::move(error));
    } catch (const std::exception &e) {
      return ws_open_reply(false, "", o.req.url, std::string("ws-open: ") + e.what());
    }
  };
  if (!cvc::net::ws_starts_async())
    return work;
  work.start = [o = std::move(o), access, &host](const std::shared_ptr<se::async_call> &call) {
    auto p = std::make_shared<WsPrepared>(prepare_ws_open(o, access));
    if (p->failed) {
      settle_started(host, call, std::move(*p->failed));
      return;
    }
    std::function<void()> stop = cvc::net::ws_start(
        o.req.url, p->opts,
        [o, p, call, &host](std::shared_ptr<cvc::net::WsConnection> conn, std::string error) {
          if (call->cancel_requested()) { // nobody waits for it now: don't keep it open
            if (conn)
              conn->close(1000);
            return;
          }
          settle_started(host, call, finish_ws_open(o, *p, std::move(conn), std::move(error)));
        });
    if (stop)
      call->on_cancel(std::move(stop));
  };
  return work;
}

void register_ws_verbs(const std::shared_ptr<se::environment> &env, se::async_host &host,
                       const NetAccess &access, const se::async_options &io) {
  const auto live = [access](const char *verb) {
    if (!is_live(access))
      throw std::runtime_error(std::string(verb) + ": the host has turned the network off");
  };
  // (ws-open URL [OPTS]) / (ws-open-async ...): connect. Reply {ok handle url error}.
  se::register_async_fn(
      env, host, "ws-open",
      [access, &host](std::span<const se::value_t> args) {
        return ws_open_work(args, access, host);
      },
      io);
  // (ws-recv H [SECONDS]) / (ws-recv-async ...): the next event, waiting up to SECONDS (default
  // 30); nil when none came. Events: {type "text"|"binary"|"close"|"error", data, code}.
  se::register_async_fn(
      env, host, "ws-recv",
      [live, &host](std::span<const se::value_t> args) -> se::async_work {
        live("ws-recv");
        std::shared_ptr<cvc::net::WsConnection> conn = ws_lookup("ws-recv", args);
        const double secs = number_arg(args, 1, 30);
        se::async_work work;
        work.kernel = [conn, secs](se::async_call &call) -> se::value_t {
          return marshal_ws_event(conn->recv(secs, [&call] { return call.cancel_requested(); }));
        };
        // Without threads: wait for the socket's next message from its callbacks, not in a loop
        // on the frame's thread (ws_open_work).
        if (cvc::net::ws_starts_async())
          work.start = [conn, secs, &host](const std::shared_ptr<se::async_call> &call) {
            std::function<void()> stop =
                conn->recv_async(secs, [call, &host](std::optional<cvc::net::WsEvent> e) {
                  settle_started(host, call, marshal_ws_event(e));
                });
            if (stop)
              call->on_cancel(std::move(stop));
          };
        return work;
      },
      io);
  // (ws-poll H): the next event already received, or nil; never waits. For on_tick loops.
  se::builtins::register_fn(env, "ws-poll",
                            [live](std::span<const se::value_t> args) -> se::value_t {
                              live("ws-poll");
                              return marshal_ws_event(ws_lookup("ws-poll", args)->recv(0));
                            });
  // (ws-send H TEXT): send a text message. #t, or an error when the connection is closed.
  se::builtins::register_fn(
      env, "ws-send", [live](std::span<const se::value_t> args) -> se::value_t {
        live("ws-send");
        std::shared_ptr<cvc::net::WsConnection> conn = ws_lookup("ws-send", args);
        if (args.size() < 2)
          throw std::runtime_error("ws-send: (ws-send H TEXT)");
        std::string error;
        if (!conn->send_text(field_to_string(args[1]), &error))
          throw std::runtime_error("ws-send: " + error);
        return se::value_t(true);
      });
  // (ws-close H [CODE]): close and forget H. #t if it was still open; a closed or unknown
  // handle is no error, so a program can close in any cleanup path.
  se::builtins::register_fn(env, "ws-close", [](std::span<const se::value_t> args) -> se::value_t {
    std::shared_ptr<cvc::net::WsConnection> conn;
    {
      WsTable &t = ws_table();
      std::lock_guard<std::mutex> lock(t.mutex);
      auto it = t.open.find(arg_string(args, 0));
      if (it == t.open.end())
        return se::value_t(false);
      conn = std::move(it->second);
      t.open.erase(it);
    }
    const bool was = conn->open();
    conn->close(static_cast<int>(number_arg(args, 1, 1000)));
    return se::value_t(was);
  });
  // (ws-open? H): whether H is open (connected, no close seen or sent).
  se::builtins::register_fn(env, "ws-open?", [](std::span<const se::value_t> args) -> se::value_t {
    WsTable &t = ws_table();
    std::lock_guard<std::mutex> lock(t.mutex);
    auto it = t.open.find(arg_string(args, 0));
    return se::value_t(it != t.open.end() && it->second->open());
  });
}

} // namespace

void register_net_intrinsics(cvc::app &app) { register_net_intrinsics(app, NetAccess{}); }

void register_net_intrinsics(cvc::app &app, NetAccess access) {
  // Warm the lazy per-app singletons on THIS thread before any background worker touches them, so a
  // worker never races their first construction (the nav_compute discipline).
  app.computePool();
  app.exec_scheduler();
  app.exec_async_host();
  // (http-get*) need a compiled cvc::net backend; (fetch*) resolve over any registered scheme
  // (file/state/cvc/http) and are useful with or without an HTTP backend.
  const bool have_http = cvc::net::have_http_backend();

  se::async_host &host = app.exec_async_host();
  // I/O calls (async_options::io): a request's network wait and its transport's CPU (one HTTPS
  // request: a 100-300 ms round trip and about 16 ms of CPU for the TLS handshake and the CA
  // store, 130,000 steps) are the host's, not the program's. Charged as other calls are, no
  // action could make one within its default budget of 5000 steps and 0.1 s. The transport's own
  // timeouts and size cap bound a request instead.
  se::async_options io;
  io.io = true;
  const bool have_ws = cvc::net::have_ws_backend();
  register_action_intrinsics([&host, have_http, have_ws, access,
                              io](std::shared_ptr<se::environment> env, se::intrinsics_context &) {
    if (!is_live(access))
      return; // the host turned the network off: later programs do not get the verbs
    // Async functions on the app's exec scheduler (docs/STATE_EXEC_EXCEPTIONS.md 3): each verb
    // parks the calling process while its kernel runs on the app's background task pool and
    // resumes it with the reply dict, at any depth (the whole program included); its "-async"
    // form returns a future for (await f [seconds]) or (await-all (list ...)). A call is charged
    // to its process (its run time against max_time), and a killed or reaped process's call is
    // cancelled.
    if (have_http) {
      // (http-get URL [OPTS]) / (http-get-async URL [OPTS]) — a GET; OPTS is the options dict
      // (query/headers/auth/capture_auth). The future form fans out N fetches.
      se::register_async_fn(
          env, host, "http-get",
          [access, &host](std::span<const se::value_t> args) -> se::async_work {
            RequestAuth auth;
            cvc::net::HttpRequest req = build_get_request(args, auth);
            check_request("http-get", req, auth, access);
            return http_work(std::move(req), std::move(auth), access, host);
          },
          io);

      // (http-request METHOD URL [OPTS]) / (http-request-async ...) — the GENERAL verb: any
      // method with an options dict OPTS carrying query/headers/body/form/auth/capture_auth (see
      // apply_http_options). It resumes with the same reply dict as (http-get).
      se::register_async_fn(
          env, host, "http-request",
          [access, &host](std::span<const se::value_t> args) -> se::async_work {
            RequestAuth auth;
            cvc::net::HttpRequest req = build_method_request(args, auth);
            check_request("http-request", req, auth, access);
            return http_work(std::move(req), std::move(auth), access, host);
          },
          io);
    }

    // (fetch URI [BASE]) / (fetch-async URI [BASE]) — the GENERIC async resolver over ANY
    // registered scheme (file/state/cvc/http), so an ari program async-loads any URI without
    // blocking the scheduler thread. Reply dict: { ok body(bytes) url error }. An http(s) URI goes
    // through the registered handler, which carries the host's policy (enable_net).
    se::register_async_fn(
        env, host, "fetch",
        [&host](std::span<const se::value_t> args) { return uri_fetch_work(args, host); }, io);

    // (ws-*) — WebSocket clients, where this build has a backend (libcurl with ws, a browser).
    if (have_ws)
      register_ws_verbs(env, host, access, io);

    register_net_verbs(env, access);
  });
}

} // namespace ariadne
} // namespace cvc
