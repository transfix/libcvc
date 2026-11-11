/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#ifndef __CVC_ARIADNE_CONFERENCE_RELAY_SESSION_H__
#define __CVC_ARIADNE_CONFERENCE_RELAY_SESSION_H__

// A host joins a relay as a client (docs/roadmap/ARIADNE_SCRIPTING_HOSTS.md §6.5.7, S2): ari and
// the x hosts, from a document's relay: block or the host's --relay URL. Joined, the document may
// call the relay's functions with (room-call ...) (room_calls.h), and the namespaces it subscribes
// to are mirrored into its tree, read-only.
//
//   relay:
//     url: grpcs://example.com:7700   # grpc://, grpcs:// (TLS), or ws:// (plain WebSocket)
//     room: ops                       # the relay's --room (default standup)
//     token: env:OPS_ROOM_TOKEN       # the room token: env:NAME or file:PATH, never the token
//     ca: certs/ops-ca.pem            # grpcs://: the CA the relay's certificate chains to
//                                     # (default: gRPC's roots, a public CA)
//     subscribe: [cyprov]             # published namespaces to mirror
//
// Paths are relative to the document's directory. A subscribed namespace lands at its own path
// in the app's tree (cyprov.*) and is linked read-only at the same path under the document's root,
// so a widget binds `cyprov.projects` as it binds any of the document's state. wss:// needs the
// native WebSocket client's TLS, which libcvc does not have: dial grpcs:// instead. `token: oidc`
// is S5's (the authenticate hook) and is refused for now.

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace cvc {
class app;
namespace ariadne {
class AppRuntime;
struct Value;
namespace conference {

struct relay_join {
  std::string url;              // grpc://host:port, grpcs://host:port or ws://host:port[/path]
  std::string room = "standup"; // the relay's --room
  std::string token;            // the room token, "" for none
  std::string ca_pem;           // grpcs://: the CA, "" for gRPC's default roots
  std::vector<std::string> subscribe; // published namespaces to mirror
  std::string name = "ari";           // the participant's node id is this plus a random suffix
};

// A relay: block into `out`, its paths relative to the document at `doc_path`: "" or the problem.
std::string relay_join_from_block(const Value &block, const std::string &doc_path, relay_join &out);

class relay_session {
public:
  explicit relay_session(cvc::app &app);
  ~relay_session(); // leaves the relay
  relay_session(const relay_session &) = delete;
  relay_session &operator=(const relay_session &) = delete;

  // Dial the relay and wait, at most `timeout`, until it has admitted this host (its room's tree
  // has arrived: until then the relay drops what it sends, call replies included). Then
  // (room-call ...) reaches it and the subscriptions are mirrored. "" or why not.
  std::string join(const relay_join &j, std::chrono::milliseconds timeout);
  // Each frame: send this host's own writes (its participant subtree) to the relay.
  void pump();
  // Link the subscribed namespaces under `prefix` (the document's root), read-only.
  void link_subscriptions(const std::string &prefix);

  bool joined() const;
  const std::string &pid() const;
  // "grpcs://host:7700 (room ops) as ari_k3x9qa, mirroring cyprov".
  std::string describe() const;

private:
  struct impl;
  std::unique_ptr<impl> _impl;
};

// The relay: block, kept for the host's load hook (LoadResult::extras). Idempotent.
void register_relay_block();

// A host's join, as ari and the x hosts make it: a load hook on `rt` that joins the relay before
// the document's requires: is checked and its init: runs: the document's relay: block, with what
// `cli` sets over it (--relay's URL, room, CA and token; its empty fields set nothing), or `cli`
// alone for a document with no block. The session stays joined for
// the runtime's life; each frame pumps it. A join that fails fails the load; one that succeeds is
// told to `note`. Returns the session (not joined until a document loads).
std::shared_ptr<relay_session> attach_relay(cvc::app &app, AppRuntime &rt, relay_join cli,
                                            std::function<void(const std::string &)> note = {});

} // namespace conference
} // namespace ariadne
} // namespace cvc

#endif
