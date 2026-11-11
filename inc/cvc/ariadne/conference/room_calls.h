/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#ifndef __CVC_ARIADNE_CONFERENCE_ROOM_CALLS_H__
#define __CVC_ARIADNE_CONFERENCE_ROOM_CALLS_H__

// room-call: a client calls a function its relay offers (docs/roadmap/ARIADNE_SCRIPTING_HOSTS.md
// §6.5.5, S2). arisrvx runs the functions in the operator's Python, on its script thread; a relay
// that runs none answers every call `unavailable`.
//
//   REQUEST  A message on the caller's own path, conf.rooms.<room>.participants.<pid>.call, with
//            ttl_hops 0 so the relay takes it and never fans it out (room_write_gate refuses one
//            that would be). Its payload, a CBOR map: {"id", "fn", "args": [...], "kwargs": {...},
//            "key", "timeout"}. The relay's gate has already bound the connection to <pid>, so the
//            caller is who the path says.
//   REPLY    Messages back on the same path, to the connection holding <pid> only
//            (state_transport::send_message_to_holder), never to another that claims the pid:
//              {"id", "type": "progress", "fraction", "message"}
//              {"id", "type": "result", "value"}
//              {"id", "type": "error", "code", "message", "detail"}
//            Codes: not_found (no such function, or not one clients may call), forbidden (a role
//            gate), deadline_exceeded, too_large (a result past the relay's cap), internal (the
//            function raised; its traceback stays on the relay), unavailable (the relay runs no
//            functions, or its program has stopped), invalid (a malformed call), and whatever a
//            function raises with CallError.
//
// Values are DSL values on both sides (cvc::state_exec::value_t), and the wire between them is
// typed CBOR (cvc/state/state_exec/value_cbor.h, with symbols as themselves) in the message's
// bytes: nil, booleans, integers, floats (2.0 stays a float), strings, bytes, symbols, lists and
// dicts all arrive as they were sent. A value that doesn't cross (a function, a host object) fails
// the call where it is sent, as a TypeError. In arisrvx the relay hands Python the values
// themselves, converted by pycvc as every pycvc value is, so the function sees the types the caller
// sent.
//
// In the DSL (register_room_call_intrinsics, a host that joined a relay; room_call_client):
//   (room-call "fn" arg ...)            parks the program and returns the function's value. One
//                                       dict argument is the call's keyword arguments.
//   (room-call-async "fn" arg ...)      returns a future at once, for (await f).
//   (room-call-opts (dict "key" K "timeout" S) "fn" arg ...)
//                                       the same, with the call's idempotency key and its timeout
//                                       (seconds; the relay's default otherwise).
//   An error raises with its code and detail as fields (forbidden as a PermissionError,
//   deadline_exceeded as a TimeoutError, every other code as a RuntimeError), which a try
//   catches. Each call's progress lands in the document's tree at calls.<fn>.progress and
//   calls.<fn>.message (the latest call of <fn>), and is posted to the call's future.

#include <cstddef>
#include <cstdint>
#include <cvc/state/state_exec/types.h>
#include <functional>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

namespace cvc {
class app;
class state_cluster_shard;
class state_delivery_filter;
class state_transport;
namespace ariadne {
namespace conference {

// The path both a call and its replies ride on.
std::string call_request_path(const std::string &room, const std::string &pid);

// A read scope for the connection that holds `pid` in `room` (state_transport::set_read_scope,
// ARIADNE_SCRIPTING_HOSTS.md §6.5.6): the paths at or below `prefixes`, and always the
// participant's own subtree (room_participant_path, where its calls' replies land) with the room's
// nodes above it, those as nodes alone (a join waits for the room's node), never another
// participant's.
std::shared_ptr<const cvc::state_delivery_filter>
room_read_scope(std::vector<std::string> prefixes, const std::string &room, const std::string &pid);

// The two message kinds on it.
constexpr const char *kCallType = "application/x-cvc-room-call+cbor";
constexpr const char *kCallReplyType = "application/x-cvc-room-call-reply+cbor";

// One call, as the relay took it.
struct room_call_request {
  std::string pid;                 // the caller (the connection's bound pid)
  std::string id;                  // the caller's call id
  std::string fn;                  // the function asked for
  cvc::state_exec::value_t args;   // a list: the positional arguments
  cvc::state_exec::value_t kwargs; // a dict: the keyword arguments
  std::string key;                 // the idempotency key: the caller's, or the call id
  double timeout = 0;              // seconds the caller allows; 0 = the relay's default
  // The call as one dict (pid, id, fn, args, kwargs, key, timeout), for a relay that hands it on.
  cvc::state_exec::value_t to_value() const;
};

// Parse a call message's payload (its CBOR): "" or what is wrong with it.
std::string parse_call(const std::string &pid, std::string_view data, room_call_request &out);

// The relay's side: the calls its clients make, taken on the relay's own thread, and the replies
// it sends back.
class room_call_relay {
public:
  room_call_relay(cvc::state_cluster_shard &shard, std::string room);
  ~room_call_relay();
  room_call_relay(const room_call_relay &) = delete;
  room_call_relay &operator=(const room_call_relay &) = delete;

  // The transports whose connections replies go out on (each reply goes to the one holding the
  // caller's pid, on whichever it is).
  void add_transport(cvc::state_transport *tp);
  // The largest result, in bytes encoded, a call may return (--call-result-max); a bigger one
  // is answered too_large. 0 = no cap. 1 MiB by default.
  void set_result_max(std::size_t bytes);

  // The calls that arrived since the last take, in order. A malformed one is answered `invalid`
  // here and not returned.
  std::vector<room_call_request> take();

  // The replies to a call: `pid`'s, with its id. Each returns the connections written (0: the
  // caller has gone).
  std::size_t result(const std::string &pid, const std::string &id,
                     const cvc::state_exec::value_t &value);
  std::size_t progress(const std::string &pid, const std::string &id, double fraction,
                       const std::string &message);
  std::size_t error(const std::string &pid, const std::string &id, const std::string &code,
                    const std::string &message,
                    const cvc::state_exec::value_t &detail = cvc::state_exec::value_t());
  std::size_t fail(const room_call_request &r, const std::string &code,
                   const std::string &message) {
    return error(r.pid, r.id, code, message);
  }

  struct stats {
    std::uint64_t calls = 0;   // taken
    std::uint64_t refused = 0; // not the sender's own path, or one the relay would fan out
    std::uint64_t replies = 0; // results and errors sent
  };
  stats take_stats();

private:
  std::size_t send(const std::string &pid, const std::string &wire);
  struct impl;
  std::shared_ptr<impl> _impl;
};

// A client's side: the calls this host makes to its relay, and their replies, which arrive on the
// transport's reader thread.
class room_call_client {
public:
  room_call_client(cvc::state_cluster_shard &shard, std::string room, std::string pid);
  ~room_call_client();
  room_call_client(const room_call_client &) = delete;
  room_call_client &operator=(const room_call_client &) = delete;

  const std::string &room() const;
  const std::string &pid() const;

  struct outcome {
    bool ok = false;
    cvc::state_exec::value_t value;  // ok: the function's value
    std::string code;                // not ok: the error's code ...
    std::string message;             // ... its message ...
    cvc::state_exec::value_t detail; // ... and its detail (nil for none)
  };
  using done_fn = std::function<void(const outcome &)>;
  using progress_fn = std::function<void(double fraction, const std::string &message)>;

  // Make a call with `args` (a list) and `kwargs` (a dict); returns its id. `done` runs once, on a
  // reader thread (or here, when the call can't be sent: `unavailable`), and `progress` for each
  // progress reply before it. Throws a TypeError when an argument doesn't cross (a function).
  std::string call(const std::string &fn, const cvc::state_exec::value_t &args,
                   const cvc::state_exec::value_t &kwargs, const std::string &key, double timeout,
                   done_fn done, progress_fn progress = nullptr);
  // Stop waiting for `id` (its caller went): a reply that comes later is dropped.
  void forget(const std::string &id);
  // Every call still waiting ends `unavailable` with `why` (the host lost its relay).
  void fail_all(const std::string &why);

  // The client a host installed for `app`, the one room-call uses; null when it joined no relay.
  static void install(cvc::app &app, std::shared_ptr<room_call_client> client);
  static std::shared_ptr<room_call_client> for_app(cvc::app &app);

private:
  struct impl;
  std::shared_ptr<impl> _impl;
};

// (room-call ...), (room-call-async ...), (room-call-opts ...) for every document of `app`, against
// the client room_call_client::for_app(app) finds at each call. Idempotent;
// register_room_intrinsics calls it.
void register_room_call_intrinsics(cvc::app &app);

} // namespace conference
} // namespace ariadne
} // namespace cvc

#endif
