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
//     token: env:OPS_ROOM_TOKEN       # env:NAME, file:PATH, or credential:NAME / oidc:NAME
//     ca: certs/ops-ca.pem            # grpcs://: the CA the relay's certificate chains to
//                                     # (default: gRPC's roots, a public CA)
//     subscribe: [cyprov]             # published namespaces to mirror
//
// The token is never in the document itself:
//   env:NAME, file:PATH       read once, at the load; the join happens then, and fails the load
//                             when it can't.
//   credential:NAME           the host's credential NAME (net_host.h, cvc/net/credentials.h), read
//   oidc:NAME                 at every join: oidc:NAME is the credential an (oidc-sign-in ...)
//                             keeps (oidc_host.h), refreshed by its sign-in before it expires.
//                             The credential must be bound to the relay's origin by a policy line
//                             (credential NAME https://host:port for grpcs:// or wss://, http://
//                             for grpc:// or ws://), and that origin allowed. The join waits until
//                             the credential holds a token (a sign-in that has yet to happen);
//                             until it has joined, room-call answers unavailable.
//
// Once joined, a connection the relay closes (it restarted, host.disconnect, a network drop) is
// dialed again, after 1 s and then twice as long each time up to 30 s, with the token read again
// (a credential's renewed first when the relay refused the last one). Meanwhile room-call answers
// unavailable. The participant keeps its node id, so its bound writes are its own again.
//
// The document sees where the session stands under its root: relay.status ("joined", "dialing",
// "waiting for credential NAME (...)", "lost; dialing again in N s (...)") and relay.joins (1 after
// the first join, 2 after a reconnect, ...), written as they change.
//
// Paths are relative to the document's directory. A subscribed namespace lands at its own path
// in the app's tree (cyprov.*) and is linked read-only at the same path under the document's root,
// so a widget binds `cyprov.projects` as it binds any of the document's state. wss:// needs the
// native WebSocket client's TLS, which libcvc does not have: dial grpcs:// instead.

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
  std::string token;            // the token (env:, file:), "" for none
  std::string credential;       // or the host's credential that holds it (credential:, oidc:)
  std::string ca_pem;           // grpcs://: the CA, "" for gRPC's default roots
  std::vector<std::string> subscribe; // published namespaces to mirror
  std::string name = "ari";           // the participant's node id is this plus a random suffix
};

// A relay: block into `out`, its paths relative to the document at `doc_path`: "" or the problem.
std::string relay_join_from_block(const Value &block, const std::string &doc_path, relay_join &out);

// The origin a relay URL's credential is bound to: https://host:port for grpcs:// and wss://,
// http://host:port for grpc:// and ws://. "" for a URL that is none of them.
std::string relay_credential_origin(const std::string &url);

class relay_session {
public:
  explicit relay_session(cvc::app &app);
  ~relay_session(); // leaves the relay
  relay_session(const relay_session &) = delete;
  relay_session &operator=(const relay_session &) = delete;

  // Dial the relay and wait, at most `timeout`, until it has admitted this host (its room's tree
  // has arrived). Then (room-call ...) reaches it and the subscriptions are mirrored. "" or why
  // not. Joined, pump() keeps the session: a lost connection is dialed again.
  std::string join(const relay_join &j, std::chrono::milliseconds timeout);
  // Join from pump() instead: as soon as the token is there (a credential's), on a thread of its
  // own, and again whenever the connection is lost. "" or what is wrong with `j` (its URL).
  std::string begin(const relay_join &j);
  // Each frame: send this host's own writes (its participant subtree) to the relay; notice a lost
  // connection; take a finished dial.
  void pump();
  // Link the subscribed namespaces under `prefix` (the document's root), read-only.
  void link_subscriptions(const std::string &prefix);

  bool joined() const;
  bool started() const; // join() succeeded or begin() was called
  // What the session is doing: "joined", "waiting for credential relaytok", "dialing",
  // "lost; dialing again in 4 s (refused: unauthenticated...)".
  std::string status() const;
  // Joins since it started: 1 after the first, 2 after a reconnect, ...
  std::uint64_t joins() const;
  const std::string &pid() const;
  // "grpcs://host:7700 (room ops) as ari_k3x9qa, mirroring cyprov".
  std::string describe() const;

  // Where a credential's token comes from (the host's credential store, by default): the token
  // credential `name` sends to `origin`, renewed first when `rejected` (the token the relay last
  // refused) is set or it is expiring; "" with `why` when there is none. Called on the dialing
  // thread.
  using token_source = std::function<std::string(const std::string &name, const std::string &origin,
                                                 const std::string &rejected, std::string &why)>;
  void set_token_source(token_source source);
  // Write status() and joins() to `<path>.status` and `<path>.joins` in the app's tree as they
  // change (pump()); "" writes nothing.
  void set_status_path(std::string path);

private:
  void step(); // pump() without the status
  struct link;
  struct impl;
  std::shared_ptr<impl> _impl;
};

// The relay: block on `app`, kept for the host's load hook (LoadResult::extras). Idempotent.
void register_relay_block(cvc::app &app);

// A host's join, as ari and the x hosts make it: a load hook on `rt` that joins the relay before
// the document's requires: is checked and its init: runs: the document's relay: block, with what
// `cli` sets over it (--relay's URL, room, CA, token and credential; its empty fields set nothing),
// or `cli` alone for a document with no block. With a token, a join that fails fails the load;
// with a credential, the join is begun and happens when the credential holds a token. Each frame
// pumps the session, which dials a lost connection again. What happens is told to `note`. Returns
// the session (not joined until a document loads).
std::shared_ptr<relay_session> attach_relay(cvc::app &app, AppRuntime &rt, relay_join cli,
                                            std::function<void(const std::string &)> note = {});

} // namespace conference
} // namespace ariadne
} // namespace cvc

#endif
