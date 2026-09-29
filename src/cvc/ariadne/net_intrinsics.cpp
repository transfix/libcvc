// Ariadne — async HTTP host intrinsics for the state_exec program lanes (roadmap §13.8). See
// net_intrinsics.h. The whole implementation is gated on CVC_STATE_EXEC (no program lanes without
// it) and on a compiled cvc::net backend (cvc::net::have_http_backend()).

#include <cvc/ariadne/ariadne.h> // register_action_intrinsics, have_state_exec
#include <cvc/ariadne/net_intrinsics.h>
#include <cvc/ariadne/uri.h> // resolve() — the generic (fetch uri) async resolver
#include <cvc/core/app.h>
#include <cvc/net/http_client.h>

#ifdef CVC_STATE_EXEC

#include <atomic>
#include <cstdint>
#include <cvc/core/state_exec/async_scheduler.h> // exec_scheduler().post_message
#include <cvc/core/state_exec/builtins.h>   // register_fn — bind the host verb into the lanes
#include <cvc/core/state_exec/intrinsics.h> // resolve_channel_key — scope the reply channel
#include <cvc/core/state_exec/types.h>      // value_t, make_dict/make_list/make_bytes
#include <span>
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

// Append arg N (a list of verbatim "Name: value" strings — e.g. "Authorization: Bearer <token>")
// onto `out`. A non-list / absent arg contributes nothing.
void append_header_list(std::span<const se::value_t> args, std::size_t idx,
                        std::vector<std::string> &out) {
  if (args.size() > idx)
    if (const se::list_ptr *lp = std::get_if<se::list_ptr>(&args[idx].v))
      for (const se::value_t &e : **lp)
        if (const std::string *h = std::get_if<std::string>(&e.v))
          out.push_back(*h);
}

// Kick a fully-built request on a compute-pool worker (OFF the scheduler thread) and post the
// marshalled response dict to a unique '#'-reply channel; return that channel. Shared by all the
// verbs — GET, and the general method verb. The worker ALWAYS posts a dict (an error dict on a
// throw), so a waiter never hangs.
std::string launch_http(cvc::app &app, const std::string &root, cvc::net::HttpRequest req) {
  static std::atomic<std::uint64_t> seq{0};
  const std::string chan =
      "http.reply#" + std::to_string(seq.fetch_add(1, std::memory_order_relaxed));
  const std::string done = se::resolve_channel_key(root, chan); // '#' => == chan
  const std::string url = req.url;
  app.compute_async(
      1, [](int) {},
      [&app, req = std::move(req), done, url] {
        se::value_t payload;
        try {
          payload = marshal_response(cvc::net::send(req));
        } catch (const std::exception &e) {
          payload = marshal_error(url, std::string("http: ") + e.what());
        } catch (...) {
          payload = marshal_error(url, "http: unknown error");
        }
        app.exec_scheduler().post_message(done, payload);
      });
  return done;
}

// (http-get URL [HEADERS]) — a bodyless GET; HEADERS is a list of "Name: value" lines.
cvc::net::HttpRequest build_get_request(std::span<const se::value_t> args) {
  cvc::net::HttpRequest req;
  req.url = arg_string(args, 0);
  append_header_list(args, 1, req.headers);
  return req;
}

// (http-request METHOD URL [BODY [HEADERS]]) — any method (GET/PUT/POST/PATCH/DELETE/…). BODY is a
// string OR bytes value (byte-safe; empty for a bodyless verb); HEADERS a list of "Name: value"
// lines (e.g. an "Authorization: Bearer <token>" for auth, or a "Content-Type: application/json").
cvc::net::HttpRequest build_method_request(std::span<const se::value_t> args) {
  cvc::net::HttpRequest req;
  req.method = arg_string(args, 0);
  req.url = arg_string(args, 1);
  if (args.size() > 2) {
    if (const std::string *s = std::get_if<std::string>(&args[2].v))
      req.body = *s;
    else if (const se::bytes_value *b = std::get_if<se::bytes_value>(&args[2].v))
      req.body = b->data; // binary body (opaque octets)
  }
  append_header_list(args, 3, req.headers);
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

// The generic async resolve (any registered scheme: file/state/cvc/http). Runs the SYNCHRONOUS
// resolve() on a compute-pool worker (OFF the scheduler thread) and posts the marshalled dict to a
// unique '#'-reply channel — the same launch shape as launch_http, for arbitrary URIs.
std::string launch_uri_fetch(cvc::app &app, const std::string &root,
                             std::span<const se::value_t> args) {
  std::string uri;
  if (!args.empty())
    if (const std::string *u = std::get_if<std::string>(&args[0].v))
      uri = *u;
  std::string base; // optional second arg: the base for a relative URI
  if (args.size() > 1)
    if (const std::string *b = std::get_if<std::string>(&args[1].v))
      base = *b;

  static std::atomic<std::uint64_t> seq{0};
  const std::string chan =
      "uri.reply#" + std::to_string(seq.fetch_add(1, std::memory_order_relaxed));
  const std::string done = se::resolve_channel_key(root, chan);
  app.compute_async(
      1, [](int) {},
      [&app, uri, base, done] {
        se::value_t payload;
        try {
          payload =
              marshal_uri_result(resolve(uri, base)); // resolve() is app-free + has its own barrier
        } catch (const std::exception &e) {
          UriResult er;
          er.ok = false;
          er.canonical = uri;
          er.error = std::string("fetch: ") + e.what();
          payload = marshal_uri_result(er);
        } catch (...) {
          UriResult er;
          er.ok = false;
          er.canonical = uri;
          er.error = "fetch: unknown error";
          payload = marshal_uri_result(er);
        }
        app.exec_scheduler().post_message(done, payload);
      });
  return done;
}

} // namespace

void register_net_intrinsics(cvc::app &app) {
  if (!have_state_exec())
    return;
  // Warm the lazy per-app singletons on THIS thread before any background worker touches them, so a
  // worker never races their first construction (the nav_compute discipline).
  app.computePool();
  app.exec_scheduler();
  // (http-get*) need a compiled cvc::net backend; (fetch*) resolve over any registered scheme
  // (file/state/cvc/http) and are useful with or without an HTTP backend.
  const bool have_http = cvc::net::have_http_backend();

  register_action_intrinsics([&app, have_http](std::shared_ptr<se::environment> env,
                                               se::intrinsics_context &ictx) {
    // Capture the lane's chroot so the reply channel is scoped the same way an (msg-recv)/(await)
    // resolves it (the launchers use a UNIQUE '#'-suffixed channel per call — policy-exempt +
    // identity — so two in-flight fetches never collide on the single recv_path a process has).
    const std::string root = ictx.root_path;

    if (have_http) {
      // (http-get-async URL [HEADERS]) — kicks the fetch and returns a FUTURE handle; the program
      // awaits it with (await …) or (msg-recv …). The low-level primitive for fanning out N
      // fetches.
      se::builtins::register_fn(
          env, "http-get-async", [&app, root](std::span<const se::value_t> args) -> se::value_t {
            return se::make_future(launch_http(app, root, build_get_request(args)));
          });

      // (http-get URL [HEADERS]) — TRANSPARENT: kicks the fetch and SELF-PARKS the calling process,
      // resuming with the response dict threaded straight into the enclosing expression (no
      // channel, no msg-recv). Reaches the scheduler via the captured app;
      // current_pid()/current_process() are valid here (set around the evaluator step). Equivalent
      // to (await (http-get-async URL)).
      se::builtins::register_fn(
          env, "http-get", [&app, root](std::span<const se::value_t> args) -> se::value_t {
            const std::string done = launch_http(app, root, build_get_request(args));
            auto &sched = app.exec_scheduler();
            return se::park_on_channel(&sched, sched.current_process().get(), sched.current_pid(),
                                       done);
          });

      // (http-request-async METHOD URL [BODY [HEADERS]]) — the GENERAL verb: any method
      // (GET/PUT/POST/PATCH/DELETE/…) with an optional request BODY (string or bytes) and custom
      // HEADERS (a list of "Name: value" lines — e.g. "Authorization: Bearer <token>"). Returns a
      // FUTURE handle.
      se::builtins::register_fn(env, "http-request-async",
                                [&app, root](std::span<const se::value_t> args) -> se::value_t {
                                  return se::make_future(
                                      launch_http(app, root, build_method_request(args)));
                                });

      // (http-request METHOD URL [BODY [HEADERS]]) — TRANSPARENT self-parking form of the general
      // verb; resumes with the same {ok status body(bytes) url headers error} dict as (http-get).
      // Equivalent to (await (http-request-async METHOD URL BODY HEADERS)).
      se::builtins::register_fn(
          env, "http-request", [&app, root](std::span<const se::value_t> args) -> se::value_t {
            const std::string done = launch_http(app, root, build_method_request(args));
            auto &sched = app.exec_scheduler();
            return se::park_on_channel(&sched, sched.current_process().get(), sched.current_pid(),
                                       done);
          });
    }

    // (fetch-async URI [BASE]) / (fetch URI [BASE]) — the GENERIC async resolver over ANY
    // registered scheme (file/state/cvc/http). fetch-async returns a future; fetch is transparent
    // (self-parks, returns the dict). resolve() runs on the compute pool, so an ari program
    // async-loads any URI without blocking the scheduler thread. Reply dict: { ok body(bytes) url
    // error }.
    se::builtins::register_fn(env, "fetch-async",
                              [&app, root](std::span<const se::value_t> args) -> se::value_t {
                                return se::make_future(launch_uri_fetch(app, root, args));
                              });
    se::builtins::register_fn(env, "fetch",
                              [&app, root](std::span<const se::value_t> args) -> se::value_t {
                                const std::string done = launch_uri_fetch(app, root, args);
                                auto &sched = app.exec_scheduler();
                                return se::park_on_channel(&sched, sched.current_process().get(),
                                                           sched.current_pid(), done);
                              });
  });
}

} // namespace ariadne
} // namespace cvc

#else // !CVC_STATE_EXEC

namespace cvc {
namespace ariadne {
void register_net_intrinsics(cvc::app & /*app*/) {
  // No program lanes without state_exec — nothing to register.
}
} // namespace ariadne
} // namespace cvc

#endif // CVC_STATE_EXEC
