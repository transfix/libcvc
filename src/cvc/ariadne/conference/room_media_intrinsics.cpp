/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// Compiled only with CVC_ENABLE_GRPC (CMake gates it): the media verbs wrap stream_bridge, which is
// itself grpc-gated. See room_media_intrinsics.h.

#include <chrono>
#include <cstdlib>                                        // std::atoi / std::strtoull
#include <cvc/ariadne/ariadne.h>                          // register_action_intrinsics
#include <cvc/ariadne/conference/room_intrinsics.h>       // room_participant_path
#include <cvc/ariadne/conference/room_media_intrinsics.h> // this
#include <cvc/ariadne/stream/stream.h>
#include <cvc/ariadne/stream/stream_bridge.h>
#include <cvc/ariadne/stream_intrinsics.h> // document_stream / own_document_stream
#include <cvc/core/app.h>
#include <cvc/state/state.h>
#include <cvc/state/state_cluster_shard.h> // default_for / local_node_id (the publisher's pid)
#include <cvc/state/state_exec/builtins.h>
#include <cvc/state/state_exec/intrinsics.h>
#include <cvc/state/state_exec/types.h>
#include <memory>
#include <span>
#include <string>
#include <utility>
#include <vector>

namespace se = cvc::state_exec;
namespace sx = cvc::ariadne::stream;

namespace cvc {
namespace ariadne {
namespace conference {

namespace {

// <participant>.streams.<kind> — the descriptor subtree a publisher advertises and a subscriber
// reads. One more level down are the leaf fields written/read below.
std::string stream_desc_path(const std::string &room, const std::string &pid,
                             const std::string &kind) {
  const std::string sep = cvc::state::SEPARATOR;
  return room_participant_path(room, pid) + sep + "streams" + sep + kind;
}
std::string field(const std::string &base, const char *leaf) {
  return base + cvc::state::SEPARATOR + leaf;
}

// touch-before-write replicated set (the stream.cpp idiom; see room_intrinsics.cpp).
void set_replicated(cvc::app &app, const std::string &path, const std::string &val) {
  cvc::state &n = app.root()(path);
  n.touch();
  n.value(val);
}
std::string read_str(cvc::app &app, const std::string &path) {
  cvc::state *n = app.root().findDescendant(path);
  return n ? n->value() : std::string();
}
int read_int(cvc::app &app, const std::string &path) {
  const std::string s = read_str(app, path);
  return s.empty() ? 0 : std::atoi(s.c_str());
}

// ── small value_t helpers (same shapes as room_intrinsics / stream_intrinsics) ──
const se::value_t *dict_get(const std::vector<std::pair<std::string, se::value_t>> &d,
                            const std::string &k) {
  for (const auto &kv : d)
    if (kv.first == k)
      return &kv.second;
  return nullptr;
}
std::string get_str(const se::value_t *v, const std::string &def = std::string()) {
  if (v)
    if (const std::string *s = std::get_if<std::string>(&v->v))
      return *s;
  return def;
}
se::value_t err(const std::string &msg) {
  return se::make_dict({{"ok", se::value_t(false)}, {"error", se::value_t(msg)}});
}
// The room id from a (room-join) handle dict (its "room" field) OR a bare room-id string — so a
// .ari can pass (state-get "...") directly, without persisting the handle dict across program-lane
// ticks.
std::string room_id_of(const se::value_t &v) {
  if (const std::string *s = std::get_if<std::string>(&v.v))
    return *s;
  if (const se::dict_ptr *dp = std::get_if<se::dict_ptr>(&v.v))
    if (*dp)
      return get_str(dict_get(**dp, "room"));
  return std::string();
}

} // namespace

bool room_publish(cvc::app &app, sx::stream_bridge &bridge, const std::string &local_endpoint,
                  const std::string &room_id, const std::string &pid, const std::string &kind,
                  sx::stream &local, const std::string &relay_endpoint) {
  if (room_id.empty() || pid.empty() || kind.empty())
    return false;
  const bool sfu = !relay_endpoint.empty();
  // SFU: push the stream to the relay (the server fans it out to subscribers), and advertise the
  // RELAY as the source so subscribers pull the relayed copy through the server (NAT-safe). MESH:
  // publish on our own bridge for peers to pull directly, advertising our own endpoint.
  if (sfu)
    bridge.serve(local, relay_endpoint);
  else
    bridge.serve_port(local);
  const std::string base = stream_desc_path(room_id, pid, kind);
  const sx::format_desc &f = local.format();
  set_replicated(app, field(base, "endpoint"), sfu ? relay_endpoint : local_endpoint);
  set_replicated(app, field(base, "token"), local.token());
  set_replicated(app, field(base, "id"), local.id());
  set_replicated(app, field(base, "scope"), local.scope());
  set_replicated(app, field(base, "codec"), f.codec);
  set_replicated(app, field(base, "fkind"), std::to_string(static_cast<int>(f.kind)));
  set_replicated(app, field(base, "w"), std::to_string(f.w));
  set_replicated(app, field(base, "h"), std::to_string(f.h));
  set_replicated(app, field(base, "stride"), std::to_string(f.stride));
  set_replicated(app, field(base, "rate"), std::to_string(f.sample_rate));
  set_replicated(app, field(base, "ch"), std::to_string(f.channels));
  set_replicated(app, field(base, "bytes"),
                 std::to_string(static_cast<unsigned long long>(f.bytes)));
  return true;
}

std::unique_ptr<sx::stream> room_subscribe(cvc::app &app, sx::stream_bridge &bridge,
                                           const std::string &room_id,
                                           const std::string &target_pid, const std::string &kind) {
  const std::string base = stream_desc_path(room_id, target_pid, kind);
  const std::string endpoint = read_str(app, field(base, "endpoint"));
  const std::string id = read_str(app, field(base, "id"));
  if (endpoint.empty() || id.empty())
    return nullptr; // not published (yet), or not replicated here

  // Rebuild the publisher's format so the proxy's pool is sized identically, and open the proxy
  // under the SAME (scope, id) so its token matches the publisher's — that is what pull routes on.
  sx::stream_params p;
  p.id = id;
  p.root_path = read_str(app, field(base, "scope"));
  p.subscriber_depth = 8; // room for a downstream ring subscription (player / widget)
  p.format.codec = read_str(app, field(base, "codec"));
  p.format.kind = static_cast<sx::frame_kind>(read_int(app, field(base, "fkind")));
  p.format.w = read_int(app, field(base, "w"));
  p.format.h = read_int(app, field(base, "h"));
  p.format.stride = read_int(app, field(base, "stride"));
  p.format.sample_rate = read_int(app, field(base, "rate"));
  p.format.channels = read_int(app, field(base, "ch"));
  {
    const std::string b = read_str(app, field(base, "bytes"));
    p.format.bytes =
        b.empty() ? 0u : static_cast<std::size_t>(std::strtoull(b.c_str(), nullptr, 10));
  }
  std::unique_ptr<sx::stream> proxy = sx::stream::open(app, p);
  if (!proxy)
    return nullptr; // unsized/duplicate
  if (!bridge.pull_from(endpoint, *proxy, std::chrono::milliseconds(3000))) {
    proxy->close();
    return nullptr;
  }
  return proxy;
}

void register_room_media_intrinsics(cvc::app &app, sx::stream_bridge &bridge,
                                    const std::string &local_endpoint,
                                    const std::string &relay_endpoint) {
  register_action_intrinsics(
      [&app, &bridge, local_endpoint, relay_endpoint](std::shared_ptr<se::environment> env,
                                                      se::intrinsics_context &ictx) {
        se::document_scope *doc = ictx.document;

        // (room-publish ROOM {kind token}) -> { ok kind token }.
        se::builtins::register_fn(
            env, "room-publish",
            [&app, &bridge, local_endpoint, relay_endpoint,
             doc](std::span<const se::value_t> args) -> se::value_t {
              if (!doc)
                return err("room-publish: no document scope");
              if (args.size() < 2)
                return err("room-publish: (room-publish ROOM {kind token})");
              const std::string room_id = room_id_of(args[0]);
              if (room_id.empty())
                return err("room-publish: ROOM must be a room-join handle or a room-id");
              // The publisher is always THIS node, so derive the pid from the cluster shard (as
              // room-join does), not from the handle — publish works with a bare room-id too.
              std::string pid = "local";
              if (auto *shard = cvc::state_cluster_shard::default_for(app))
                if (!shard->local_node_id().empty())
                  pid = shard->local_node_id();
              const se::dict_ptr *dp = std::get_if<se::dict_ptr>(&args[1].v);
              if (!dp || !*dp)
                return err("room-publish: second argument must be a { kind token } dict");
              const std::string kind = get_str(dict_get(**dp, "kind"));
              const std::string token = get_str(dict_get(**dp, "token"));
              if (kind.empty() || token.empty())
                return err("room-publish: 'kind' and 'token' are required");
              sx::stream *local = document_stream(doc, token);
              if (!local)
                return err("room-publish: no open stream for that token (stream-open it first)");
              room_publish(app, bridge, local_endpoint, room_id, pid, kind, *local, relay_endpoint);
              return se::make_dict({
                  {"ok", se::value_t(true)},
                  {"kind", se::value_t(kind)},
                  {"token", se::value_t(token)},
              });
            });

        // (room-subscribe ROOM PID KIND) -> { ok token pid kind }.
        se::builtins::register_fn(
            env, "room-subscribe",
            [&app, &bridge, doc](std::span<const se::value_t> args) -> se::value_t {
              if (!doc)
                return err("room-subscribe: no document scope");
              if (args.size() < 3)
                return err("room-subscribe: (room-subscribe ROOM PID KIND)");
              const std::string room_id = room_id_of(args[0]);
              const std::string target_pid = get_str(&args[1]);
              const std::string kind = get_str(&args[2]);
              if (room_id.empty() || target_pid.empty() || kind.empty())
                return err("room-subscribe: ROOM handle, PID and KIND are required");
              std::unique_ptr<sx::stream> proxy =
                  room_subscribe(app, bridge, room_id, target_pid, kind);
              if (!proxy)
                return err("room-subscribe: no such published stream (or the pull failed)");
              // Hand the proxy to the document so stream-play-audio / a stream_view can consume it.
              const std::string token = own_document_stream(doc, std::move(proxy));
              return se::make_dict({
                  {"ok", se::value_t(true)},
                  {"token", se::value_t(token)},
                  {"pid", se::value_t(target_pid)},
                  {"kind", se::value_t(kind)},
              });
            });
      });
}

} // namespace conference
} // namespace ariadne
} // namespace cvc
