/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <cvc/ariadne/ariadne.h> // register_action_intrinsics
#include <cvc/ariadne/stream/stream.h>
#include <cvc/ariadne/stream/stream_channel.h>
#include <cvc/ariadne/stream_intrinsics.h>
#include <cvc/core/app.h>                   // exec_scheduler warm
#include <cvc/core/state_exec/builtins.h>   // register_fn
#include <cvc/core/state_exec/intrinsics.h> // intrinsics_context
#include <cvc/core/state_exec/types.h>      // value_t, make_dict
#include <memory>
#include <mutex>
#include <span>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace se = cvc::state_exec;

namespace cvc {
namespace ariadne {

namespace {

// -------- per-DOCUMENT OWNED stream table (model B: document-lifetime ownership) --------
// Lives in the calling document's se::document_scope (reached via ictx.document), NOT a
// process-static table keyed by app* — so ownership follows the document's lifetime: a stream
// opened here lives until (stream-close) or document teardown, when the Runtime clear()s the scope
// and ~stream_table destroys any still-open stream while the app is still alive. Distinct from
// cvc::ariadne::stream::stream_registry (the non-owning channel side-table); this OWNS the
// unique_ptr.
constexpr const char *kStreamSlot = "cvc.ariadne.stream"; // document_scope slot key
struct stream_table {
  std::mutex mu;
  std::unordered_map<std::string, std::unique_ptr<cvc::ariadne::stream::stream>> by_token;

  void own(std::unique_ptr<cvc::ariadne::stream::stream> s) {
    std::lock_guard<std::mutex> lk(mu);
    by_token[s->token()] = std::move(s); // token is unique in this scope (open() refused a dup)
  }
  cvc::ariadne::stream::stream *find(const std::string &token) {
    std::lock_guard<std::mutex> lk(mu);
    auto it = by_token.find(token);
    return it == by_token.end() ? nullptr : it->second.get();
  }
  bool close(const std::string &token) {
    std::unique_ptr<cvc::ariadne::stream::stream> victim; // destroyed OUTSIDE the lock: ~stream ->
    {                                                     // close() touches state + the scheduler.
      std::lock_guard<std::mutex> lk(mu);
      auto it = by_token.find(token);
      if (it == by_token.end())
        return false;
      victim = std::move(it->second);
      by_token.erase(it);
    }
    return true; // victim's ~stream runs here
  }
  // ~stream_table (document teardown) destroys by_token, closing every stream the author left open.
  // Single-threaded teardown (the owning scope is clear()d by the Runtime), so no lock is needed.
};

// Get-or-create THIS document's stream table. Null only when `doc` is null (the init: lane) or on a
// slot type-collision (a programming error — the key is private to this TU); the verbs surface that
// as a clean error rather than dereferencing null.
stream_table *doc_streams(se::document_scope *doc) {
  return doc ? doc->slot<stream_table>(kStreamSlot) : nullptr;
}

// -------- small value_t helpers --------
int codec_channels(const std::string &c) {
  if (c == "rgba8")
    return 4;
  if (c == "rgb8")
    return 3;
  if (c == "graya8")
    return 2;
  if (c == "gray8")
    return 1;
  return 0;
}
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
long get_int(const se::value_t *v, long def) {
  if (v) {
    if (const int64_t *i = std::get_if<int64_t>(&v->v))
      return static_cast<long>(*i);
    if (const double *d = std::get_if<double>(&v->v))
      return static_cast<long>(*d);
  }
  return def;
}
double get_num(const se::value_t *v, double def) {
  if (v) {
    if (const double *d = std::get_if<double>(&v->v))
      return *d;
    if (const int64_t *i = std::get_if<int64_t>(&v->v))
      return static_cast<double>(*i);
  }
  return def;
}
se::value_t err(const std::string &msg) {
  return se::make_dict({{"ok", se::value_t(false)}, {"error", se::value_t(msg)}});
}
// A handle is the open() dict (read "token") or a bare token string.
std::string handle_token(const se::value_t &v) {
  if (const std::string *s = std::get_if<std::string>(&v.v))
    return *s;
  if (const se::dict_ptr *dp = std::get_if<se::dict_ptr>(&v.v))
    if (*dp)
      return get_str(dict_get(**dp, "token"));
  return std::string();
}

} // namespace

void register_stream_intrinsics(cvc::app &app) {
  app.exec_scheduler(); // warm the lazy scheduler the stream posts lifecycle/seq on

  register_action_intrinsics([&app](std::shared_ptr<se::environment> env,
                                    se::intrinsics_context &ictx) {
    const std::string root = ictx.root_path; // scope streams to the calling document (§4.1)
    se::document_scope *doc = ictx.document; // per-document owner; null on the init: lane

    // (stream-open OPTS) -> handle dict { ok id token seq_channel evt_channel }.
    se::builtins::register_fn(
        env, "stream-open", [&app, root, doc](std::span<const se::value_t> args) -> se::value_t {
          if (!doc)
            return err("stream-open: no document scope (open a stream from an action or resident "
                       "lane, not the load-time init: lane)");
          stream_table *tbl = doc_streams(doc);
          if (!tbl)
            return err("stream-open: internal: stream-table slot unavailable");
          if (args.empty())
            return err("stream-open: missing options dict");
          const se::dict_ptr *dp = std::get_if<se::dict_ptr>(&args[0].v);
          if (!dp || !*dp)
            return err("stream-open: options must be a dict");
          const auto &d = **dp;

          const std::string id = get_str(dict_get(d, "id"));
          if (id.empty())
            return err("stream-open: 'id' is required");
          const std::string codec = get_str(dict_get(d, "codec"), "rgba8");
          const int ch = codec_channels(codec);
          if (ch == 0)
            return err("stream-open: unsupported codec '" + codec + "'");
          // Bound the sizing knobs. w/h feed static_cast<int> (and w*ch -> stride), and
          // subscriber_depth/expected_subscribers feed frame_pool slab-count math
          // (count = subscribers*(depth+in_flight)+working_set). A negative or absurd value would
          // truncate/overflow or wrap size_t and attempt a wild allocation. The DSL is trusted, but
          // a bad dict must fail cleanly (like a missing w/h) rather than OOM-kill the action.
          constexpr long kMaxDim = 65536;  // a raw video frame dimension; w*ch stays within int
          constexpr long kMaxDepth = 1024; // per-subscriber ring/latest depth
          constexpr long kMaxSubs = 4096;  // expected concurrent subscribers (pool pre-size)
          const long w = get_int(dict_get(d, "w"), 0);
          const long h = get_int(dict_get(d, "h"), 0);
          if (w <= 0 || h <= 0 || w > kMaxDim || h > kMaxDim)
            return err("stream-open: 'w' and 'h' are required and must be in [1, 65536]");
          const long sub_depth = get_int(dict_get(d, "subscriber_depth"), 3);
          const long exp_subs = get_int(dict_get(d, "expected_subscribers"), 1);
          if (sub_depth < 1 || sub_depth > kMaxDepth)
            return err("stream-open: 'subscriber_depth' must be in [1, 1024]");
          if (exp_subs < 1 || exp_subs > kMaxSubs)
            return err("stream-open: 'expected_subscribers' must be in [1, 4096]");

          cvc::ariadne::stream::stream_params p;
          p.id = id;
          p.root_path = root;
          p.format.kind = cvc::ariadne::stream::frame_kind::video_raw;
          p.format.codec = codec;
          p.format.w = static_cast<int>(w);
          p.format.h = static_cast<int>(h);
          p.format.stride = static_cast<int>(w * ch); // dense
          p.subscriber_depth = static_cast<std::size_t>(sub_depth);
          p.expected_subscribers = static_cast<std::size_t>(exp_subs);
          p.heartbeat_hz = get_num(dict_get(d, "heartbeat_hz"), 10.0);

          auto s = cvc::ariadne::stream::stream::open(app, p);
          if (!s)
            return err("stream-open: open failed (bad params, or id already open in this scope)");
          const std::string token = s->token();
          const std::string seqc = s->seq_channel();
          const std::string evtc = s->evt_channel();
          tbl->own(std::move(s)); // owned by THIS document
          return se::make_dict({
              {"ok", se::value_t(true)},
              {"id", se::value_t(id)},
              {"token", se::value_t(token)},
              {"seq_channel", se::value_t(seqc)},
              {"evt_channel", se::value_t(evtc)},
          });
        });

    // (stream-info H) -> dict; live=#f when the handle names no open stream.
    se::builtins::register_fn(
        env, "stream-info", [doc](std::span<const se::value_t> args) -> se::value_t {
          if (!doc)
            return err("stream-info: no document scope");
          stream_table *tbl = doc_streams(doc);
          if (!tbl)
            return err("stream-info: internal: stream-table slot unavailable");
          if (args.empty())
            return err("stream-info: missing handle");
          const std::string token = handle_token(args[0]);
          if (token.empty())
            return err("stream-info: handle must be a stream dict or a token string");
          cvc::ariadne::stream::stream *s = tbl->find(token);
          if (!s)
            return se::make_dict({{"ok", se::value_t(true)},
                                  {"live", se::value_t(false)},
                                  {"token", se::value_t(token)}});
          return se::make_dict({
              {"ok", se::value_t(true)},
              {"live", se::value_t(true)},
              {"id", se::value_t(s->id())},
              {"token", se::value_t(s->token())},
              {"seq_channel", se::value_t(s->seq_channel())},
              {"evt_channel", se::value_t(s->evt_channel())},
              {"published", se::value_t(static_cast<int64_t>(s->channel().total_published()))},
              {"subscribers", se::value_t(static_cast<int64_t>(s->channel().subscriber_count()))},
          });
        });

    // (stream-close H) -> { ok #t/#f }.
    se::builtins::register_fn(
        env, "stream-close", [doc](std::span<const se::value_t> args) -> se::value_t {
          if (!doc)
            return err("stream-close: no document scope");
          stream_table *tbl = doc_streams(doc);
          if (!tbl)
            return err("stream-close: internal: stream-table slot unavailable");
          if (args.empty())
            return err("stream-close: missing handle");
          const std::string token = handle_token(args[0]);
          const bool ok = !token.empty() && tbl->close(token);
          return se::make_dict({{"ok", se::value_t(ok)}});
        });
  });
}

} // namespace ariadne
} // namespace cvc
