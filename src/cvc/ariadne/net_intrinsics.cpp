// Ariadne — async HTTP host intrinsics for the state_exec program lanes (roadmap §13.8). See
// net_intrinsics.h. The whole implementation is gated on CVC_STATE_EXEC (no program lanes without
// it) and on a compiled cvc::net backend (cvc::net::have_http_backend()).

#include <cvc/ariadne/ariadne.h> // register_action_intrinsics, have_state_exec
#include <cvc/ariadne/net_intrinsics.h>
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

} // namespace

void register_net_intrinsics(cvc::app &app) {
  if (!have_state_exec() || !cvc::net::have_http_backend())
    return;
  // Warm the lazy per-app singletons on THIS thread before any background worker touches them, so a
  // worker never races their first construction (the nav_compute discipline).
  app.computePool();
  app.exec_scheduler();

  register_action_intrinsics(
      [&app](std::shared_ptr<se::environment> env, se::intrinsics_context &ictx) {
        // Capture the lane's chroot so the reply channel is scoped the same way the program's
        // (msg-recv <chan>) will resolve it. The verb generates a UNIQUE '#'-suffixed channel per
        // call (returned verbatim by resolve_channel_key + exempt from channel-policy), so two
        // in-flight fetches never collide on the single recv_path a process has.
        const std::string root = ictx.root_path;
        se::builtins::register_fn(
            env, "http-get-async", [&app, root](std::span<const se::value_t> args) -> se::value_t {
              cvc::net::HttpRequest req;
              if (!args.empty())
                if (const std::string *url = std::get_if<std::string>(&args[0].v))
                  req.url = *url;
              // Optional second arg: a list of verbatim "Name: value" header strings.
              if (args.size() > 1)
                if (const se::list_ptr *lp = std::get_if<se::list_ptr>(&args[1].v))
                  for (const se::value_t &e : **lp)
                    if (const std::string *h = std::get_if<std::string>(&e.v))
                      req.headers.push_back(*h);

              static std::atomic<std::uint64_t> seq{0};
              const std::string chan =
                  "http.reply#" + std::to_string(seq.fetch_add(1, std::memory_order_relaxed));
              const std::string done = se::resolve_channel_key(root, chan); // '#' => == chan

              // Run the BLOCKING fetch on a compute-pool worker; post the marshalled dict when it
              // joins. compute_async returns immediately, so the verb never blocks the scheduler
              // thread. The worker ALWAYS posts (an error dict on a throw), so the parked
              // (msg-recv) always resumes.
              const std::string url = req.url;
              app.compute_async(
                  1, [](int) {},
                  [&app, req, done, url] {
                    se::value_t payload;
                    try {
                      payload = marshal_response(cvc::net::send(req));
                    } catch (const std::exception &e) {
                      payload = marshal_error(url, std::string("http-get: ") + e.what());
                    } catch (...) {
                      payload = marshal_error(url, "http-get: unknown error");
                    }
                    app.exec_scheduler().post_message(done, payload);
                  });
              return se::value_t(chan); // the program awaits with (msg-recv <chan>)
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
