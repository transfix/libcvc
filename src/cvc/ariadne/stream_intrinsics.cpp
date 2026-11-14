/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <algorithm>
#include <cvc/ariadne/ariadne.h>              // register_action_intrinsics
#include <cvc/ariadne/bind.h>                 // resolve_bind: a raster's mirror path
#include <cvc/ariadne/stream/audio_sources.h> // audio_f32_bytes (size an audio_pcm frame)
#include <cvc/ariadne/stream/raster.h>        // (raster-*): a raster as a stream
#include <cvc/ariadne/stream/stream.h>
#include <cvc/ariadne/stream/stream_channel.h>
#include <cvc/ariadne/stream_intrinsics.h>
#include <cvc/core/app.h>                    // exec_scheduler warm
#include <cvc/state/state_exec/builtins.h>   // register_fn
#include <cvc/state/state_exec/intrinsics.h> // intrinsics_context
#include <cvc/state/state_exec/types.h>      // value_t, make_dict
#include <limits>
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

// This document's rasters (raster-open), owned as its streams are: until (raster-close) or the
// document's teardown.
constexpr const char *kRasterSlot = "cvc.ariadne.raster";
struct raster_table {
  std::mutex mu;
  std::unordered_map<std::string, std::unique_ptr<cvc::ariadne::stream::raster_stream>> by_token;
};
raster_table *doc_rasters(se::document_scope *doc) {
  return doc ? doc->slot<raster_table>(kRasterSlot) : nullptr;
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

  const ActionIntrinsicProvider provider = [&app](std::shared_ptr<se::environment> env,
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

          // Bound the sizing knobs (both kinds). w/h/rate/ch feed static_cast<int> (and w*ch ->
          // stride, fpc*ch*4 -> bytes), and subscriber_depth/expected_subscribers feed frame_pool
          // slab-count math (count = subscribers*(depth+in_flight)+working_set). A negative or
          // absurd value would truncate/overflow or wrap size_t and attempt a wild allocation. The
          // DSL is trusted, but a bad dict must fail cleanly rather than OOM-kill the action.
          constexpr long kMaxDim = 65536;  // a raw video frame dimension; w*ch stays within int
          constexpr long kMaxDepth = 1024; // per-subscriber ring/latest depth
          constexpr long kMaxSubs = 4096;  // expected concurrent subscribers (pool pre-size)
          const long sub_depth = get_int(dict_get(d, "subscriber_depth"), 3);
          const long exp_subs = get_int(dict_get(d, "expected_subscribers"), 1);
          if (sub_depth < 1 || sub_depth > kMaxDepth)
            return err("stream-open: 'subscriber_depth' must be in [1, 1024]");
          if (exp_subs < 1 || exp_subs > kMaxSubs)
            return err("stream-open: 'expected_subscribers' must be in [1, 4096]");

          cvc::ariadne::stream::stream_params p;
          p.id = id;
          p.root_path = root;
          p.subscriber_depth = static_cast<std::size_t>(sub_depth);
          p.expected_subscribers = static_cast<std::size_t>(exp_subs);
          p.heartbeat_hz = get_num(dict_get(d, "heartbeat_hz"), 10.0);

          // The codec selects the kind: "f32" (interleaved float PCM) opens an AUDIO stream —
          // rate/ch and a per-chunk frame size instead of w/h; every other codec opens a raw VIDEO
          // stream (the original rgba8/rgb8/graya8/gray8 path).
          if (codec == "f32") {
            constexpr long kMaxRate = 768000, kMaxCh = 64, kMaxChunk = 65536;
            const long rate = get_int(dict_get(d, "rate"), 48000);
            const long chn = get_int(dict_get(d, "ch"), 1);
            const long fpc = get_int(dict_get(d, "frames_per_chunk"), 1024);
            if (rate < 1 || rate > kMaxRate)
              return err("stream-open: audio 'rate' must be in [1, 768000]");
            if (chn < 1 || chn > kMaxCh)
              return err("stream-open: audio 'ch' must be in [1, 64]");
            if (fpc < 1 || fpc > kMaxChunk)
              return err("stream-open: audio 'frames_per_chunk' must be in [1, 65536]");
            p.format.kind = cvc::ariadne::stream::frame_kind::audio_pcm;
            p.format.codec = "f32";
            p.format.sample_rate = static_cast<int>(rate);
            p.format.channels = static_cast<int>(chn);
            p.format.bytes =
                cvc::ariadne::stream::audio_f32_bytes(static_cast<int>(fpc), static_cast<int>(chn));
          } else if (codec == "mjpeg" || codec == "jpeg") {
            // Compressed video (conference network compression): variable-size JPEG frames, so no
            // fixed channels/stride. w/h are the DECODED resolution (the subscriber's stream_view
            // decodes with the JPEG header). The byte slab is explicit "bytes" or a generous w*h —
            // a JPEG never exceeds the raw size at sane quality; a frame that would is dropped.
            const long w = get_int(dict_get(d, "w"), 0);
            const long h = get_int(dict_get(d, "h"), 0);
            if (w <= 0 || h <= 0 || w > kMaxDim || h > kMaxDim)
              return err("stream-open: 'w' and 'h' are required and must be in [1, 65536]");
            p.format.kind = cvc::ariadne::stream::frame_kind::video_raw;
            p.format.codec = codec;
            p.format.w = static_cast<int>(w);
            p.format.h = static_cast<int>(h);
            p.format.stride = static_cast<int>(w * 4); // informational (decoded geometry)
            const long bytes = get_int(dict_get(d, "bytes"), 0);
            p.format.bytes = bytes > 0 ? static_cast<std::size_t>(bytes)
                                       : static_cast<std::size_t>(w) * static_cast<std::size_t>(h);
          } else {
            const int ch = codec_channels(codec);
            if (ch == 0)
              return err("stream-open: unsupported codec '" + codec + "'");
            const long w = get_int(dict_get(d, "w"), 0);
            const long h = get_int(dict_get(d, "h"), 0);
            if (w <= 0 || h <= 0 || w > kMaxDim || h > kMaxDim)
              return err("stream-open: 'w' and 'h' are required and must be in [1, 65536]");
            p.format.kind = cvc::ariadne::stream::frame_kind::video_raw;
            p.format.codec = codec;
            p.format.w = static_cast<int>(w);
            p.format.h = static_cast<int>(h);
            p.format.stride = static_cast<int>(w * ch); // dense
          }

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
  };
  register_action_intrinsics(app, "cvc.ariadne.stream", provider);

  // The raster verbs (docs/ARIADNE.md §5.4, "Live textures"): a raster in memory, painted and
  // published as a stream of the rectangles that changed, mirrored into state when asked.
  const ActionIntrinsicProvider rasters = [&app](std::shared_ptr<se::environment> env,
                                                 se::intrinsics_context &ictx) {
    namespace sx = cvc::ariadne::stream;
    const std::string root = ictx.root_path;
    se::document_scope *doc = ictx.document;
    // The raster a handle names in this document, or null with `why` set.
    const auto find = [doc](const char *verb, std::span<const se::value_t> args,
                            std::string &why) -> sx::raster_stream * {
      raster_table *tbl = doc_rasters(doc);
      if (!tbl) {
        why = std::string(verb) + ": no document scope";
        return nullptr;
      }
      if (args.empty()) {
        why = std::string(verb) + ": missing raster handle";
        return nullptr;
      }
      const std::string token = handle_token(args[0]);
      std::lock_guard<std::mutex> lk(tbl->mu);
      auto it = tbl->by_token.find(token);
      if (it == tbl->by_token.end()) {
        why = std::string(verb) + ": no open raster '" + token + "'";
        return nullptr;
      }
      return it->second.get();
    };
    // [x y w h] from a list of numbers.
    const auto rect_of = [](const se::value_t &v, sx::raster_rect &r) {
      const se::list_ptr *lp = std::get_if<se::list_ptr>(&v.v);
      if (!lp || !*lp || (*lp)->size() != 4)
        return false;
      const auto &l = **lp;
      r = sx::raster_rect{static_cast<int>(get_int(&l[0], 0)), static_cast<int>(get_int(&l[1], 0)),
                          static_cast<int>(get_int(&l[2], 0)), static_cast<int>(get_int(&l[3], 0))};
      return true;
    };
    const auto handle = [](const sx::raster_stream &r) {
      return se::make_dict({{"ok", se::value_t(true)},
                            {"id", se::value_t(r.stream().id())},
                            {"token", se::value_t(r.token())},
                            {"mirror", se::value_t(r.mirror())},
                            {"w", se::value_t(static_cast<int64_t>(r.width()))},
                            {"h", se::value_t(static_cast<int64_t>(r.height()))}});
    };

    // (raster-open {id w h [key_every] [subscribers] [mirror]}) -> { ok id token mirror w h }. A
    // raster already open under that token in this document is handed back as it is.
    se::builtins::register_fn(
        env, "raster-open",
        [&app, root, doc, handle](std::span<const se::value_t> args) -> se::value_t {
          raster_table *tbl = doc_rasters(doc);
          if (!tbl)
            return err("raster-open: no document scope (open a raster from an action or resident "
                       "lane, not the load-time init: lane)");
          const se::dict_ptr *dp = args.empty() ? nullptr : std::get_if<se::dict_ptr>(&args[0].v);
          if (!dp || !*dp)
            return err("raster-open: options must be a dict");
          const auto &d = **dp;
          sx::raster_params p;
          p.id = get_str(dict_get(d, "id"));
          p.root_path = root;
          p.w = static_cast<int>(get_int(dict_get(d, "w"), 0));
          p.h = static_cast<int>(get_int(dict_get(d, "h"), 0));
          p.key_every = static_cast<int>(get_int(dict_get(d, "key_every"), 30));
          const long subs = get_int(dict_get(d, "subscribers"), 4);
          if (subs < 1 || subs > 64)
            return err("raster-open: 'subscribers' must be in [1, 64]");
          p.expected_subscribers = static_cast<std::size_t>(subs);
          if (const std::string m = get_str(dict_get(d, "mirror")); !m.empty())
            p.mirror = resolve_bind(root, m); // a path like a bind's: through the document's holes
          {
            std::lock_guard<std::mutex> lk(tbl->mu);
            auto it = tbl->by_token.find(sx::stream::registry_key(root, p.id));
            if (it != tbl->by_token.end())
              return handle(*it->second);
          }
          std::string why;
          std::unique_ptr<sx::raster_stream> r = sx::raster_stream::open(app, p, &why);
          if (!r)
            return err("raster-open: " + why);
          se::value_t out = handle(*r);
          std::lock_guard<std::mutex> lk(tbl->mu);
          tbl->by_token[r->token()] = std::move(r);
          return out;
        });

    // (raster-fill H [x y w h] [r g b a]) -> #t: paint a rectangle one colour (0..255 each).
    se::builtins::register_fn(
        env, "raster-fill", [find, rect_of](std::span<const se::value_t> args) -> se::value_t {
          std::string why;
          sx::raster_stream *r = find("raster-fill", args, why);
          if (!r)
            return err(why);
          sx::raster_rect rect;
          const se::list_ptr *cp =
              args.size() > 2 ? std::get_if<se::list_ptr>(&args[2].v) : nullptr;
          if (args.size() < 3 || !rect_of(args[1], rect) || !cp || !*cp || (*cp)->size() != 4)
            return err("raster-fill: (raster-fill H [x y w h] [r g b a])");
          std::uint8_t rgba[4];
          for (int k = 0; k < 4; ++k)
            rgba[k] = static_cast<std::uint8_t>(std::clamp<long>(get_int(&(**cp)[k], 0), 0, 255));
          r->fill(rect, rgba);
          return se::value_t(true);
        });

    // (raster-grid H [x y w h] VALUES [{colormap lo hi alpha}]) -> #t: a w x h grid of numbers
    // (row by row; nil or a non-number is transparent), one pixel each, coloured.
    se::builtins::register_fn(
        env, "raster-grid", [find, rect_of](std::span<const se::value_t> args) -> se::value_t {
          std::string why;
          sx::raster_stream *r = find("raster-grid", args, why);
          if (!r)
            return err(why);
          sx::raster_rect rect;
          const se::list_ptr *vp =
              args.size() > 2 ? std::get_if<se::list_ptr>(&args[2].v) : nullptr;
          if (args.size() < 3 || !rect_of(args[1], rect) || !vp || !*vp)
            return err("raster-grid: (raster-grid H [x y w h] VALUES [OPTS])");
          std::vector<double> values;
          values.reserve((*vp)->size());
          for (const se::value_t &v : **vp)
            values.push_back(get_num(&v, std::numeric_limits<double>::quiet_NaN()));
          std::string cmap = "viridis";
          double lo = 0.0, hi = 1.0;
          long alpha = 255;
          if (args.size() > 3)
            if (const se::dict_ptr *op = std::get_if<se::dict_ptr>(&args[3].v); op && *op) {
              cmap = get_str(dict_get(**op, "colormap"), cmap);
              lo = get_num(dict_get(**op, "lo"), lo);
              hi = get_num(dict_get(**op, "hi"), hi);
              alpha = get_int(dict_get(**op, "alpha"), alpha);
            }
          if (!r->paint_grid(values, rect.w, rect.h, rect.x, rect.y, cmap, lo, hi,
                             static_cast<std::uint8_t>(std::clamp<long>(alpha, 0, 255))))
            return err("raster-grid: VALUES must hold w * h numbers, and '" + cmap +
                       "' must be a colour map (viridis, magma, plasma, inferno, turbo, jet, "
                       "parula, gray)");
          return se::value_t(true);
        });

    // (raster-publish H) -> the frame's number (0: nothing was painted since the last one).
    se::builtins::register_fn(env, "raster-publish",
                              [find](std::span<const se::value_t> args) -> se::value_t {
                                std::string why;
                                sx::raster_stream *r = find("raster-publish", args, why);
                                if (!r)
                                  return err(why);
                                return se::value_t(static_cast<int64_t>(r->publish()));
                              });

    // (raster-key H) -> the number of a key frame sent now.
    se::builtins::register_fn(env, "raster-key",
                              [find](std::span<const se::value_t> args) -> se::value_t {
                                std::string why;
                                sx::raster_stream *r = find("raster-key", args, why);
                                if (!r)
                                  return err(why);
                                return se::value_t(static_cast<int64_t>(r->publish_key()));
                              });

    // (raster-close H) -> { ok #t/#f }.
    se::builtins::register_fn(env, "raster-close",
                              [doc](std::span<const se::value_t> args) -> se::value_t {
                                raster_table *tbl = doc_rasters(doc);
                                if (!tbl || args.empty())
                                  return err("raster-close: no document scope, or no handle");
                                std::unique_ptr<cvc::ariadne::stream::raster_stream> victim;
                                {
                                  std::lock_guard<std::mutex> lk(tbl->mu);
                                  auto it = tbl->by_token.find(handle_token(args[0]));
                                  if (it != tbl->by_token.end()) {
                                    victim = std::move(it->second);
                                    tbl->by_token.erase(it);
                                  }
                                }
                                return se::make_dict({{"ok", se::value_t(victim != nullptr)}});
                              });
  };
  register_action_intrinsics(app, "cvc.ariadne.raster", rasters);
}

// Cross-module accessor onto the document-owned stream table (declared in the header). Reuses the
// same private doc_streams()/stream_table::find() the (stream-*) verbs use, so a companion module
// (e.g. the cvcGL media verbs) can resolve a document's open stream by token to attach a producer,
// without the table type leaking out of this TU.
stream::stream *document_stream(se::document_scope *doc, const std::string &token) {
  stream_table *tbl = doc_streams(doc);
  return tbl ? tbl->find(token) : nullptr;
}

// Hand an already-open stream to the document's owning table (see the header). Mirrors what
// (stream-open) does after stream::open, so the stream is torn down with the document and reachable
// by the other (stream-*) / media verbs by its token.
std::string own_document_stream(se::document_scope *doc,
                                std::unique_ptr<cvc::ariadne::stream::stream> s) {
  if (!s)
    return std::string();
  stream_table *tbl = doc_streams(doc);
  if (!tbl)
    return std::string(); // s is destroyed here
  const std::string token = s->token();
  tbl->own(std::move(s));
  return token;
}

} // namespace ariadne
} // namespace cvc
