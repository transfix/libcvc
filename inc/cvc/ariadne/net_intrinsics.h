#ifndef CVC_ARIADNE_NET_INTRINSICS_H
#define CVC_ARIADNE_NET_INTRINSICS_H

// Ariadne — async HTTP host intrinsics for the state_exec program lanes (roadmap §13.8). Registers
// an `(http-get-async URL [HEADERS])` verb that fetches over cvc::net OFF the scheduler thread and,
// on completion, posts the response to a UNIQUE reply channel the verb RETURNS. A program awaits it
// with the existing `(msg-recv <chan>)`, which resumes with a dict:
//
//   (get-attr (msg-recv (http-get-async "https://host/x")) "body")
//     ; => dict { ok  status  body(bytes)  url  headers(list)  error }
//
// WHY two-step (verb + msg-recv) and not a transparent `(http-get url)` that returns the body: a
// host verb is a native_fn(std::span<value_t>) with NO intrinsics_context at call time, so it
// cannot read the calling process's pid or self-park; only a core intrinsic (like msg-recv) can. So
// the verb SUBMITS and returns the channel, and msg-recv (a core intrinsic) does the parking +
// value-threading — the exact nav_compute pattern. A transparent single-verb await needs a new
// evaluator hook (a future item).
//
// THREADING: the fetch runs on app.computePool() (a background worker), so the scheduler/host
// thread is never blocked; the worker touches only the one thread-safe seam,
// exec_scheduler().post_message. The verb ALWAYS posts a dict (an error dict on a transport/arg
// failure), so a parked (msg-recv) never hangs forever.
//
// LANES + LIMITS: use it only in a program `on:`/`on:tick`/`on_key`/`on_pointer` lane — NOT in
// `init:` (no per-frame pump, so a park never resumes) and NOT in the reactive read lane
// (visible_when/computed, default-deny). NATIVE ONLY for now: the background-worker model assumes
// native threads (wasm needs the §13.8 async-fetch path). A no-op without a compiled cvc::net
// backend.

namespace cvc {
class app;
namespace ariadne {

// Register the async net intrinsics against `app` (used for the compute pool + the scheduler
// ingress the completion posts to). Appends to the process-global action-intrinsic providers, so
// call it once at setup, before load_*; tear down with cvc::ariadne::clear_action_intrinsics()
// before `app` dies (the provider captures `app` by reference). A no-op if this build lacks
// state_exec or an HTTP backend.
void register_net_intrinsics(cvc::app &app);

} // namespace ariadne
} // namespace cvc

#endif // CVC_ARIADNE_NET_INTRINSICS_H
