// Ariadne — async HTTP host intrinsics for the state_exec program lanes (roadmap §13.8). See
// net_intrinsics.h. The (http-get*) verbs are gated on a compiled cvc::net backend
// (cvc::net::have_http_backend()).

#include <atomic>
#include <cstdint>
#include <cvc/ariadne/ariadne.h> // register_action_intrinsics
#include <cvc/ariadne/net_intrinsics.h>
#include <cvc/ariadne/uri.h> // resolve() — the generic (fetch uri) async resolver
#include <cvc/core/app.h>
#include <cvc/net/http_client.h>
#include <cvc/state/state_exec/async_host.h> // register_async_fn — the verbs are async functions
#include <cvc/state/state_exec/builtins.h>   // percent_encode — the query string
#include <cvc/state/state_exec/types.h>      // value_t, make_dict/make_list/make_bytes
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
se::value_t marshal_response(const cvc::net::HttpResponse &r) {
  std::vector<se::value_t> headers;
  headers.reserve(r.headers.size());
  for (const std::string &h : r.headers)
    headers.push_back(se::value_t(h));
  return se::make_dict({
      {"ok", se::value_t(r.ok)},
      {"status", se::value_t(static_cast<int64_t>(r.status))},
      {"body", se::make_bytes(r.body)},
      {"url", se::value_t(r.canonical_url)},
      {"headers", se::make_list(std::move(headers))},
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

// Percent-encode an ordered dict into "k1=v1&k2=v2" (keys AND values encoded). Shared by the query
// string and the application/x-www-form-urlencoded body; dict order is preserved.
std::string encode_form_pairs(const se::dict_ptr &d) {
  std::string out;
  for (const auto &kv : *d) {
    if (!out.empty())
      out.push_back('&');
    out += se::percent_encode(kv.first);
    out.push_back('=');
    out += se::percent_encode(field_to_string(kv.second));
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
// values stringified) OR a list of verbatim "Name: value" strings (the escape hatch for a repeated
// header name or byte-exact control). Anything else contributes nothing.
void collect_headers(const se::value_t &v, std::vector<std::string> &out) {
  if (const se::dict_ptr *d = std::get_if<se::dict_ptr>(&v.v)) {
    for (const auto &kv : **d)
      out.push_back(kv.first + ": " + field_to_string(kv.second));
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

// Apply an options dict onto `req`: "query" (dict -> percent-encoded query string appended to the
// URL), "headers" (dict or list), "body" (string or bytes -> raw body), "form" (dict ->
// x-www-form-urlencoded body + a default Content-Type unless the caller set one). Unknown keys are
// ignored (forward-compatible: json/timeout/... may come later). A misuse (options not a dict, a
// wrong value type, or both "body" and "form") throws a clear error; the verb runs on the scheduler
// thread, so this surfaces as an ordinary DSL evaluation error, not a hang.
void apply_http_options(cvc::net::HttpRequest &req, const se::value_t &opts) {
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
  }
  if (headers)
    collect_headers(*headers, req.headers);
  if (query) {
    const se::dict_ptr *qd = std::get_if<se::dict_ptr>(&query->v);
    if (!qd)
      throw std::runtime_error("http request 'query' option must be a dict");
    req.url = url_with_query(std::move(req.url), encode_form_pairs(*qd));
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
    req.body = encode_form_pairs(*fd);
    if (!has_content_type(req.headers))
      req.headers.push_back("Content-Type: application/x-www-form-urlencoded");
  }
}

// The kernel of a fully-built request: it runs on the app's background task pool (OFF the
// scheduler thread) and returns the marshalled response dict. A failure to send is a reply with
// ok #f too, as the verbs document: only an HTTP reply's own failure is data, but these verbs have
// always described a failed send that way, and a program checks `ok` either way.
se::async_kernel http_kernel(cvc::net::HttpRequest req) {
  return [req = std::move(req)](se::async_call &) -> se::value_t {
    try {
      return marshal_response(cvc::net::send(req));
    } catch (const std::exception &e) {
      return marshal_error(req.url, std::string("http: ") + e.what());
    } catch (...) {
      return marshal_error(req.url, "http: unknown error");
    }
  };
}

// (http-get URL [OPTS]) — a GET; OPTS is an options dict (query/headers — see apply_http_options).
cvc::net::HttpRequest build_get_request(std::span<const se::value_t> args) {
  cvc::net::HttpRequest req;
  req.method = "GET";
  req.url = arg_string(args, 0);
  if (args.size() > 1)
    apply_http_options(req, args[1]);
  return req;
}

// (http-request METHOD URL [OPTS]) — any method (GET/PUT/POST/PATCH/DELETE/…). OPTS is an options
// dict: query/headers/body/form (see apply_http_options), so a POST/PUT carries a body + headers,
// e.g. (http-request "POST" url (dict "headers" (dict "Authorization" (str-concat "Bearer " tok))
// "form" (dict "grip" 1 "risk" 2))).
cvc::net::HttpRequest build_method_request(std::span<const se::value_t> args) {
  cvc::net::HttpRequest req;
  req.method = arg_string(args, 0);
  req.url = arg_string(args, 1);
  if (args.size() > 2)
    apply_http_options(req, args[2]);
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
se::async_kernel uri_fetch_kernel(std::span<const se::value_t> args) {
  std::string uri;
  if (!args.empty())
    if (const std::string *u = std::get_if<std::string>(&args[0].v))
      uri = *u;
  std::string base; // optional second arg: the base for a relative URI
  if (args.size() > 1)
    if (const std::string *b = std::get_if<std::string>(&args[1].v))
      base = *b;
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

} // namespace

void register_net_intrinsics(cvc::app &app) {
  // Warm the lazy per-app singletons on THIS thread before any background worker touches them, so a
  // worker never races their first construction (the nav_compute discipline).
  app.computePool();
  app.exec_scheduler();
  app.exec_async_host();
  // (http-get*) need a compiled cvc::net backend; (fetch*) resolve over any registered scheme
  // (file/state/cvc/http) and are useful with or without an HTTP backend.
  const bool have_http = cvc::net::have_http_backend();

  se::async_host &host = app.exec_async_host();
  register_action_intrinsics([&host, have_http](std::shared_ptr<se::environment> env,
                                                se::intrinsics_context &) {
    // Async functions on the app's exec scheduler (docs/STATE_EXEC_EXCEPTIONS.md 3): each verb
    // parks the calling process while its kernel runs on the app's background task pool and
    // resumes it with the reply dict, at any depth (the whole program included); its "-async"
    // form returns a future for (await f [seconds]) or (await-all (list ...)). A call is charged
    // to its process (its run time against max_time), and a killed or reaped process's call is
    // cancelled.
    if (have_http) {
      // (http-get URL [OPTS]) / (http-get-async URL [OPTS]) — a GET; OPTS is the options dict
      // (query/headers). The future form is the low-level primitive for fanning out N fetches.
      se::register_async_fn(env, host, "http-get",
                            [](std::span<const se::value_t> args) -> se::async_kernel {
                              return http_kernel(build_get_request(args));
                            });

      // (http-request METHOD URL [OPTS]) / (http-request-async ...) — the GENERAL verb: any
      // method (GET/PUT/POST/PATCH/DELETE/…) with an options dict OPTS carrying
      // query/headers/body/form (see apply_http_options — a bearer token is
      // (dict "headers" (dict "Authorization" …)); a form body is (dict "form" (dict …))). It
      // resumes with the same {ok status body(bytes) url headers error} dict as (http-get).
      se::register_async_fn(env, host, "http-request",
                            [](std::span<const se::value_t> args) -> se::async_kernel {
                              return http_kernel(build_method_request(args));
                            });
    }

    // (fetch URI [BASE]) / (fetch-async URI [BASE]) — the GENERIC async resolver over ANY
    // registered scheme (file/state/cvc/http), so an ari program async-loads any URI without
    // blocking the scheduler thread. Reply dict: { ok body(bytes) url error }.
    se::register_async_fn(env, host, "fetch",
                          [](std::span<const se::value_t> args) { return uri_fetch_kernel(args); });
  });
}

} // namespace ariadne
} // namespace cvc
