/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.

  libcvc is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.
*/

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cvc/ariadne/conference/room_chat.h>
#include <cvc/ariadne/conference/room_intrinsics.h> // room_roster: who is present, and their names
#include <cvc/core/app.h>
#include <cvc/state/state.h>
#include <cvc/state/state_cluster_shard.h>
#include <cvc/state/state_exec/builtins.h> // builtin_json_parse
#include <cvc/state/state_exec/types.h>
#include <cvc/state/state_message.h>
#include <cvc/state/state_message_bus.h>
#include <cvc/state/state_transport.h>
#include <deque>
#include <map>
#include <mutex>
#include <set>
#include <span>
#include <utility>

namespace se = cvc::state_exec;

namespace cvc {
namespace ariadne {
namespace conference {

namespace {

using snapshot_entry = cvc::state_transport::snapshot_entry;

std::string join2(const std::string &a, const std::string &b) {
  return a + cvc::state::SEPARATOR + b;
}

// -------- JSON --------

void put_json_string(std::string &out, const std::string &s) {
  out += '"';
  for (const char ch : s) {
    const auto c = static_cast<unsigned char>(ch);
    switch (c) {
    case '"':
      out += "\\\"";
      break;
    case '\\':
      out += "\\\\";
      break;
    case '\n':
      out += "\\n";
      break;
    case '\t':
      out += "\\t";
      break;
    default:
      if (c < 0x20) {
        char buf[8];
        std::snprintf(buf, sizeof(buf), "\\u%04x", c);
        out += buf;
      } else {
        out += ch; // UTF-8 passes through
      }
    }
  }
  out += '"';
}

// The length of the UTF-8 sequence at S[I] (1-4), or 0 when it is not one: a stray continuation
// byte, an overlong or surrogate encoding, or one cut short.
std::size_t utf8_length(const std::string &s, std::size_t i) {
  const auto b = [&](std::size_t k) { return static_cast<unsigned char>(s[k]); };
  const unsigned char c = b(i);
  if (c < 0x80)
    return 1;
  std::size_t n = 0;
  unsigned cp = 0;
  if ((c & 0xE0) == 0xC0)
    n = 2, cp = c & 0x1F;
  else if ((c & 0xF0) == 0xE0)
    n = 3, cp = c & 0x0F;
  else if ((c & 0xF8) == 0xF0)
    n = 4, cp = c & 0x07;
  else
    return 0;
  if (i + n > s.size())
    return 0;
  for (std::size_t k = 1; k < n; ++k) {
    if ((b(i + k) & 0xC0) != 0x80)
      return 0;
    cp = (cp << 6) | (b(i + k) & 0x3F);
  }
  static constexpr unsigned kMin[] = {0, 0, 0x80, 0x800, 0x10000};
  if (cp < kMin[n] || cp > 0x10FFFF || (cp >= 0xD800 && cp <= 0xDFFF))
    return 0;
  return n;
}

// S with each byte that does not start a UTF-8 sequence replaced by '?' (a name a client wrote).
std::string sanitize_utf8(const std::string &s) {
  std::string out;
  out.reserve(s.size());
  for (std::size_t i = 0; i < s.size();) {
    const std::size_t n = utf8_length(s, i);
    if (n == 0) {
      out += '?';
      ++i;
    } else {
      out.append(s, i, n);
      i += n;
    }
  }
  return out;
}

// The object JSON parses to, or false.
bool parse_object(const std::string &json, se::value_t &out) {
  try {
    const se::value_t arg(json);
    out = se::builtin_json_parse(std::span<const se::value_t>(&arg, 1));
  } catch (...) {
    return false;
  }
  const se::dict_ptr *d = std::get_if<se::dict_ptr>(&out.v);
  return d && *d;
}
const se::value_t *field(const se::value_t &obj, const char *key) {
  if (const se::dict_ptr *d = std::get_if<se::dict_ptr>(&obj.v))
    if (*d)
      for (const auto &kv : **d)
        if (kv.first == key)
          return &kv.second;
  return nullptr;
}
std::string str_field(const se::value_t &obj, const char *key) {
  if (const se::value_t *v = field(obj, key))
    if (const std::string *s = std::get_if<std::string>(&v->v))
      return *s;
  return std::string();
}
double num_field(const se::value_t &obj, const char *key, double def = 0.0) {
  if (const se::value_t *v = field(obj, key)) {
    if (const double *d = std::get_if<double>(&v->v))
      return *d;
    if (const std::int64_t *i = std::get_if<std::int64_t>(&v->v))
      return static_cast<double>(*i);
  }
  return def;
}
// A whole non-negative number field, or false.
bool seq_field(const se::value_t &obj, const char *key, std::uint64_t &out) {
  const double d = num_field(obj, key, -1.0);
  if (!(d >= 0.0) || d > 9.0e15 || d != static_cast<double>(static_cast<std::uint64_t>(d)))
    return false;
  out = static_cast<std::uint64_t>(d);
  return true;
}

std::uint64_t parse_u64(const std::string &s) {
  if (s.empty())
    return 0;
  char *end = nullptr;
  const unsigned long long v = std::strtoull(s.c_str(), &end, 10);
  return end && *end == '\0' ? static_cast<std::uint64_t>(v) : 0;
}

std::string leaf(cvc::app &app, const std::string &path) {
  cvc::state *n = app.root().findDescendant(path);
  return n ? n->value() : std::string();
}

// A write the relay's drain fans out. touch() first, so the observer that replicates the node is
// attached before the value lands (room_intrinsics' set_replicated).
void set_replicated(cvc::app &app, const std::string &path, const std::string &value) {
  cvc::state &n = app.root()(path);
  if (n.value() == value)
    return;
  n.touch();
  n.value(value);
}

double wall_seconds() {
  return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch())
      .count();
}

// -------- the relay's books --------

struct channel_log {
  std::deque<chat_message> kept; // seqs first .. head-1
  std::uint64_t head = 0;        // the next seq
  std::uint64_t first() const { return head - kept.size(); }
};

struct bucket {
  double tokens = 0.0;
  double at = -1.0; // steady seconds of the last refill; < 0: never used
};

struct request {
  std::string pid;
  std::string json;
};

std::string pair_key(const std::string &a, const std::string &b) {
  return a < b ? a + '\n' + b : b + '\n' + a;
}

} // namespace

// -------- paths, encoding, requests --------

std::string chat_root_path(const std::string &room) {
  return std::string("conf") + cvc::state::SEPARATOR + "rooms" + cvc::state::SEPARATOR + room +
         cvc::state::SEPARATOR + "chat";
}

std::string chat_request_path(const std::string &room, const std::string &pid) {
  return join2(room_participant_path(room, pid), "chat");
}

std::string chat_channel_path(const std::string &room, const std::string &channel) {
  return channel.empty() ? join2(chat_root_path(room), "room")
                         : join2(join2(chat_root_path(room), "dm"), channel);
}

std::string encode_chat_message(const chat_message &m) {
  std::string out = "{\"seq\":" + std::to_string(m.seq) + ",\"from\":";
  put_json_string(out, m.from);
  out += ",\"name\":";
  put_json_string(out, m.name);
  char t[48];
  std::snprintf(t, sizeof(t), ",\"t\":%.3f,\"text\":", m.t);
  out += t;
  put_json_string(out, m.text);
  out += ",\"e\":";
  put_json_string(out, m.epoch);
  out += '}';
  return out;
}

bool decode_chat_message(const std::string &json, chat_message &out) {
  se::value_t obj;
  if (!parse_object(json, obj) || !seq_field(obj, "seq", out.seq))
    return false;
  out.from = str_field(obj, "from");
  out.name = str_field(obj, "name");
  out.t = num_field(obj, "t");
  out.text = str_field(obj, "text");
  out.epoch = str_field(obj, "e");
  return !out.from.empty();
}

cvc::state_message make_chat_post(const std::string &room, const std::string &pid,
                                  std::uint64_t cseq, const std::string &to,
                                  const std::string &text) {
  std::string json = "{\"op\":\"post\",\"cseq\":" + std::to_string(cseq) + ",\"to\":";
  put_json_string(json, to);
  json += ",\"text\":";
  put_json_string(json, text);
  json += '}';
  cvc::state_message m = cvc::state_message::make_text(chat_request_path(room, pid),
                                                       std::move(json),
                                                       cvc::state_message::MIME_JSON);
  m.ttl_hops = 0; // to the relay only: never forwarded (room_write_gate refuses anything else)
  return m;
}

cvc::state_message make_chat_page(const std::string &room, const std::string &pid,
                                  const std::string &channel, std::uint64_t before) {
  std::string json = "{\"op\":\"page\",\"channel\":";
  put_json_string(json, channel);
  json += ",\"before\":" + std::to_string(before) + '}';
  cvc::state_message m = cvc::state_message::make_text(chat_request_path(room, pid),
                                                       std::move(json),
                                                       cvc::state_message::MIME_JSON);
  m.ttl_hops = 0;
  return m;
}

bool clean_chat_text(std::string &text, std::size_t text_max, std::string &why) {
  std::string out;
  out.reserve(text.size());
  for (std::size_t i = 0; i < text.size();) {
    const std::size_t n = utf8_length(text, i);
    if (n == 0) {
      why = "the message is not UTF-8 text";
      return false;
    }
    const auto c = static_cast<unsigned char>(text[i]);
    if (n > 1 || c == '\n' || c == '\t' || (c >= 0x20 && c != 0x7F))
      out.append(text, i, n);
    i += n;
  }
  while (!out.empty() && (out.back() == ' ' || out.back() == '\n' || out.back() == '\t'))
    out.pop_back();
  std::size_t lead = 0;
  while (lead < out.size() && out[lead] == '\n')
    ++lead; // blank lines before the text
  out.erase(0, lead);
  if (out.find_first_not_of(" \t\n") == std::string::npos) {
    why = "the message is empty";
    return false;
  }
  if (out.size() > text_max) {
    why = "the message is longer than " + std::to_string(text_max) + " bytes";
    return false;
  }
  text = std::move(out);
  return true;
}

// -------- room_chat_relay --------

struct room_chat_relay::impl {
  cvc::app &app;
  cvc::state_cluster_shard &shard;
  std::string room;
  chat_limits limits;
  std::string epoch;
  std::string request_prefix; // conf.rooms.<room>.participants.
  std::vector<cvc::state_transport *> transports;
  cvc::state_message_bus::subscription_id sub = 0;

  // Filled on transport reader threads, taken by step().
  std::mutex mu;
  std::vector<request> queue;
  std::uint64_t dropped_unread = 0;

  channel_log room_log;
  std::map<std::string, channel_log> dms;               // by pair_key
  std::map<std::string, std::vector<std::string>> peers; // pid -> its conversations' peers
  std::map<std::string, bucket> buckets;
  std::uint64_t rev = 0; // every push carries the next one, so a client's chat.rev always moves
  double last_dm_heads = -1.0;
  stats st;

  impl(cvc::app &a, cvc::state_cluster_shard &s, std::string r, chat_limits l)
      : app(a), shard(s), room(std::move(r)), limits(l) {
    epoch = std::to_string(static_cast<long long>(wall_seconds() * 1000.0));
    request_prefix = join2(join2(join2(join2("conf", "rooms"), room), "participants"), "");
  }

  // Bus callback (reader threads): queue a request made on the sender's own path.
  void on_message(const cvc::state_message &m) {
    if (m.path.size() <= request_prefix.size() || m.path.compare(0, request_prefix.size(),
                                                                 request_prefix) != 0)
      return;
    const std::string rest = m.path.substr(request_prefix.size()); // <pid>.chat
    const std::string suffix = std::string(cvc::state::SEPARATOR) + "chat";
    if (rest.size() <= suffix.size() ||
        rest.compare(rest.size() - suffix.size(), suffix.size(), suffix) != 0)
      return; // some other message on a participant path: not ours
    const std::string pid = rest.substr(0, rest.size() - suffix.size());
    std::lock_guard<std::mutex> lk(mu);
    // Only the sender's own path, and nothing a relay would fan out (a bound-writes relay never
    // admits either; this holds the line without one too).
    if (!cvc::state::isValidStateName(pid) || pid != m.origin_node_id || m.ttl_hops != 0 ||
        queue.size() >= 4096) {
      ++dropped_unread;
      return;
    }
    queue.push_back({pid, m.string_value});
  }

  // Push ENTRIES, then chat.rev, to the connection holding PID on every transport.
  void push(const std::string &pid, std::vector<snapshot_entry> entries) {
    snapshot_entry r;
    r.path = join2(chat_root_path(room), "rev");
    r.string_value = std::to_string(++rev);
    r.origin_node_id = shard.local_node_id();
    entries.push_back(std::move(r));
    for (cvc::state_transport *tp : transports)
      st.pushes += tp->push_snapshot_to_holder(pid, entries);
  }

  snapshot_entry entry(const std::string &path, std::string value) const {
    snapshot_entry e;
    e.path = path;
    e.string_value = std::move(value);
    e.origin_node_id = shard.local_node_id();
    return e;
  }

  // What RECIPIENT holds of a conversation with PEER: its head and first, and the messages.
  void dm_entries(std::vector<snapshot_entry> &out, const std::string &peer,
                  const channel_log &log) const {
    const std::string base = chat_channel_path(room, peer);
    out.push_back(entry(join2(base, "head"), std::to_string(log.head)));
    out.push_back(entry(join2(base, "first"), std::to_string(log.first())));
  }
  void message_entry(std::vector<snapshot_entry> &out, const std::string &channel,
                     const chat_message &m) const {
    out.push_back(entry(join2(join2(chat_channel_path(room, channel), "m"), std::to_string(m.seq)),
                        encode_chat_message(m)));
  }
  snapshot_entry dms_entry(const std::string &pid) const {
    std::string list;
    if (auto it = peers.find(pid); it != peers.end())
      for (const std::string &p : it->second)
        list += (list.empty() ? "" : ",") + p;
    return entry(join2(chat_root_path(room), "dms"), list);
  }

  void refuse(const std::string &pid, std::uint64_t cseq, const std::string &to,
              const std::string &why) {
    ++st.refused;
    std::string json = "{\"cseq\":" + std::to_string(cseq) + ",\"to\":";
    put_json_string(json, to);
    json += ",\"error\":";
    put_json_string(json, why);
    json += '}';
    push(pid, {entry(join2(chat_root_path(room), "error"), std::move(json))});
  }

  bool take_token(const std::string &pid, double now_s) {
    bucket &b = buckets[pid];
    if (b.at < 0.0)
      b.tokens = limits.rate_burst;
    else
      b.tokens = std::min(limits.rate_burst, b.tokens + (now_s - b.at) * limits.rate_per_s);
    b.at = now_s;
    if (b.tokens < 1.0)
      return false;
    b.tokens -= 1.0;
    return true;
  }

  void append(channel_log &log, chat_message &m, std::size_t keep) {
    m.seq = log.head++;
    log.kept.push_back(m);
    while (log.kept.size() > keep)
      log.kept.pop_front();
  }

  void publish_room_counters() {
    set_replicated(app, join2(join2(chat_root_path(room), "room"), "head"),
                   std::to_string(room_log.head));
    set_replicated(app, join2(join2(chat_root_path(room), "room"), "first"),
                   std::to_string(room_log.first()));
  }

  void post(const std::string &pid, const se::value_t &req, double now_s,
            const std::map<std::string, std::string> &present) {
    std::uint64_t cseq = 0;
    (void)seq_field(req, "cseq", cseq);
    const std::string to = str_field(req, "to");
    auto self = present.find(pid);
    if (self == present.end())
      return refuse(pid, cseq, to, "you are not in the room");
    std::string text = str_field(req, "text"), why;
    if (!clean_chat_text(text, limits.text_max, why))
      return refuse(pid, cseq, to, why);
    if (!to.empty()) {
      if (!limits.dms)
        return refuse(pid, cseq, to, "direct messages are off on this relay");
      if (to == pid)
        return refuse(pid, cseq, to, "you cannot send a direct message to yourself");
      if (!present.count(to))
        return refuse(pid, cseq, to, "they are not in the room");
    }
    if (!take_token(pid, now_s))
      return refuse(pid, cseq, to, "you are sending too fast; wait a moment");

    chat_message m;
    m.from = pid;
    m.name = sanitize_utf8(self->second.empty() ? pid : self->second);
    m.t = wall_seconds();
    m.text = std::move(text);
    m.epoch = epoch;
    if (to.empty()) {
      append(room_log, m, limits.room_keep);
      ++st.posts;
      for (const auto &[p, name] : present) {
        std::vector<snapshot_entry> e;
        message_entry(e, "", m);
        push(p, std::move(e));
      }
      publish_room_counters();
      return;
    }
    const std::string key = pair_key(pid, to);
    auto it = dms.find(key);
    if (it == dms.end()) {
      const auto count = [&](const std::string &p) {
        auto pi = peers.find(p);
        return pi == peers.end() ? std::size_t(0) : pi->second.size();
      };
      if (count(pid) >= limits.dm_channels_max)
        return refuse(pid, cseq, to, "you have too many conversations");
      if (count(to) >= limits.dm_channels_max)
        return refuse(pid, cseq, to, "they have too many conversations");
      it = dms.emplace(key, channel_log{}).first;
      peers[pid].push_back(to);
      peers[to].push_back(pid);
    }
    append(it->second, m, limits.dm_keep);
    ++st.posts;
    ++st.dms;
    // Each of the two holds the conversation under the other's pid; the sender's copy is its
    // confirmation.
    for (const auto &[holder, peer] : {std::make_pair(pid, to), std::make_pair(to, pid)}) {
      std::vector<snapshot_entry> e;
      message_entry(e, peer, m);
      dm_entries(e, peer, it->second);
      e.push_back(dms_entry(holder));
      push(holder, std::move(e));
    }
  }

  void page(const std::string &pid, const se::value_t &req,
            const std::map<std::string, std::string> &present) {
    if (!present.count(pid))
      return;
    std::uint64_t before = 0;
    if (!seq_field(req, "before", before))
      return;
    const std::string channel = str_field(req, "channel");
    const channel_log *log = &room_log;
    if (!channel.empty()) {
      auto it = dms.find(pair_key(pid, channel));
      log = it == dms.end() || channel == pid ? nullptr : &it->second;
    }
    ++st.pages;
    std::vector<snapshot_entry> e;
    if (log) {
      const std::uint64_t hi = std::min(before, log->head);
      const std::uint64_t lo = std::max(log->first(), hi > limits.page ? hi - limits.page : 0);
      for (std::uint64_t s = lo; s < hi; ++s)
        message_entry(e, channel, log->kept[static_cast<std::size_t>(s - log->first())]);
      if (!channel.empty())
        dm_entries(e, channel, *log);
    }
    push(pid, std::move(e)); // an empty page still moves chat.rev: the request was answered
  }

  // Each present participant's conversation heads (and its list), so it notices a lost push.
  void push_dm_heads(const std::map<std::string, std::string> &present) {
    for (const auto &[pid, list] : peers) {
      if (!present.count(pid) || list.empty())
        continue;
      std::vector<snapshot_entry> e;
      for (const std::string &peer : list)
        if (auto it = dms.find(pair_key(pid, peer)); it != dms.end())
          dm_entries(e, peer, it->second);
      e.push_back(dms_entry(pid));
      push(pid, std::move(e));
    }
  }

  std::map<std::string, std::string> roster() const {
    std::map<std::string, std::string> out;
    for (const room_participant &p : room_roster(app, room))
      out[p.pid] = p.name;
    return out;
  }
};

room_chat_relay::room_chat_relay(cvc::app &app, cvc::state_cluster_shard &shard, std::string room,
                                 chat_limits limits)
    : _impl(std::make_unique<impl>(app, shard, std::move(room), limits)) {
  impl *self = _impl.get();
  _impl->sub = shard.message_bus().subscribe(
      self->request_prefix.substr(0, self->request_prefix.size() - 1),
      [self](const cvc::state_message &m) { self->on_message(m); });
  set_replicated(app, join2(chat_root_path(_impl->room), "epoch"), _impl->epoch);
  _impl->publish_room_counters();
}

room_chat_relay::~room_chat_relay() { _impl->shard.message_bus().unsubscribe(_impl->sub); }

void room_chat_relay::add_transport(cvc::state_transport *tp) {
  if (tp && std::find(_impl->transports.begin(), _impl->transports.end(), tp) ==
                _impl->transports.end())
    _impl->transports.push_back(tp);
}

void room_chat_relay::step(double now_s) {
  impl &d = *_impl;
  std::vector<request> todo;
  {
    std::lock_guard<std::mutex> lk(d.mu);
    todo.swap(d.queue);
    d.st.dropped += d.dropped_unread;
    d.dropped_unread = 0;
  }
  const bool heads_due = d.last_dm_heads < 0.0 || now_s - d.last_dm_heads >= 5.0;
  if (todo.empty() && !(heads_due && !d.peers.empty()))
    return;
  const std::map<std::string, std::string> present = d.roster();
  for (const request &r : todo) {
    se::value_t req;
    if (!parse_object(r.json, req)) {
      ++d.st.dropped;
      continue;
    }
    const std::string op = str_field(req, "op");
    if (op == "post")
      d.post(r.pid, req, now_s, present);
    else if (op == "page")
      d.page(r.pid, req, present);
    else
      ++d.st.dropped;
  }
  if (heads_due) {
    d.last_dm_heads = now_s;
    d.push_dm_heads(present);
  }
}

void room_chat_relay::participant_left(const std::string &pid) {
  impl &d = *_impl;
  auto it = d.peers.find(pid);
  if (it == d.peers.end())
    return;
  std::set<std::string> present;
  for (const room_participant &p : room_roster(d.app, d.room))
    present.insert(p.pid);
  const std::vector<std::string> list = it->second;
  for (const std::string &peer : list) {
    if (present.count(peer))
      continue; // it can still read the conversation
    d.dms.erase(pair_key(pid, peer));
    for (const auto &[a, b] : {std::make_pair(pid, peer), std::make_pair(peer, pid)}) {
      auto pi = d.peers.find(a);
      if (pi == d.peers.end())
        continue;
      pi->second.erase(std::remove(pi->second.begin(), pi->second.end(), b), pi->second.end());
      if (pi->second.empty())
        d.peers.erase(pi);
    }
  }
  d.buckets.erase(pid);
}

room_chat_relay::stats room_chat_relay::take_stats() {
  stats s = _impl->st;
  _impl->st = stats{};
  return s;
}

const chat_limits &room_chat_relay::limits() const noexcept { return _impl->limits; }
const std::string &room_chat_relay::epoch() const noexcept { return _impl->epoch; }

// -------- room_chat_client --------

namespace {

constexpr double kRetrySeconds = 3.0; // an unanswered page request is sent again
constexpr double kGapSeconds = 2.0;   // the head ahead of what is held this long: a lost push

struct client_channel {
  std::map<std::uint64_t, chat_message> held;
  std::uint64_t head = 0, first = 0;
  std::uint64_t oldest = 0, newest = 0; // the contiguous run, when count > 0
  std::size_t count = 0;
  std::uint64_t view_rev = 0;
  std::uint64_t seen_rev = ~std::uint64_t(0); // chat.rev at the last scan
  std::uint64_t seen_head = ~std::uint64_t(0);
  bool pending = false;
  std::uint64_t pending_before = 0;
  double pending_at = 0.0;
  double gap_since = -1.0;
  std::string error;
  std::uint64_t error_cseq = 0;
};

} // namespace

struct room_chat_client::impl {
  cvc::app &app;
  std::string room, pid;
  sender_fn send;
  chat_limits limits;
  std::string root; // conf.rooms.<room>.chat
  std::string epoch;
  std::map<std::string, client_channel> channels;
  std::uint64_t cseq = 0;
  std::uint64_t error_seen_cseq = 0; // the newest chat.error already handed to its channel
  std::vector<conversation> convs;
  std::uint64_t convs_rev = ~std::uint64_t(0);

  impl(cvc::app &a, std::string r, std::string p, sender_fn s)
      : app(a), room(std::move(r)), pid(std::move(p)), send(std::move(s)),
        root(chat_root_path(room)) {
    if (!send)
      send = [this](const cvc::state_message &m) {
        cvc::state_cluster_shard *shard = cvc::state_cluster_shard::default_for(app);
        if (!shard || !shard->transport())
          return false;
        shard->send_message(m);
        return true;
      };
  }

  // A new relay process: what an earlier one pushed is not this history.
  void check_epoch() {
    const std::string e = leaf(app, join2(root, "epoch"));
    if (e != epoch) {
      epoch = e;
      channels.clear();
      convs.clear();
      convs_rev = ~std::uint64_t(0);
    }
  }

  // The message leaf at SEQ, into C.held. True when it is held (now or already).
  bool read_message(const std::string &base, client_channel &c, std::uint64_t seq) {
    if (c.held.count(seq))
      return true;
    const std::string v = leaf(app, join2(join2(base, "m"), std::to_string(seq)));
    chat_message m;
    if (v.empty() || !decode_chat_message(v, m) || m.seq != seq || m.epoch != epoch)
      return false;
    c.held.emplace(seq, std::move(m));
    return true;
  }

  void request_page(const std::string &channel, client_channel &c, std::uint64_t before,
                    double now_s) {
    if (send(make_chat_page(room, pid, channel, before))) {
      c.pending = true;
      c.pending_before = before;
      c.pending_at = now_s;
    }
  }

  void scan(const std::string &channel, client_channel &c, std::uint64_t rev) {
    const std::string base = chat_channel_path(room, channel);
    c.head = parse_u64(leaf(app, join2(base, "head")));
    c.first = parse_u64(leaf(app, join2(base, "first")));
    if (c.held.empty()) {
      // A fresh view of a tree that may already hold messages (a reloaded document): walk down
      // from the newest while they are there.
      for (std::uint64_t s = c.head; s > c.first && read_message(base, c, s - 1); --s)
        ;
    } else {
      for (std::uint64_t s = std::max(c.held.rbegin()->first + 1, c.first); s < c.head; ++s)
        (void)read_message(base, c, s);
    }
    if (c.pending) {
      const std::uint64_t hi = std::min(c.pending_before, c.head);
      const std::uint64_t lo = std::max(c.first, hi > limits.page ? hi - limits.page : 0);
      bool all = true;
      for (std::uint64_t s = lo; s < hi; ++s)
        all = read_message(base, c, s) && all;
      // Answered: the whole page is here, or the relay pushed (chat.rev moved) and no longer
      // keeps what is missing.
      if (all || (rev != c.seen_rev && hi <= c.first))
        c.pending = false;
    }
    // The error for a post to this channel, once each.
    const std::string err = leaf(app, join2(root, "error"));
    se::value_t obj;
    std::uint64_t ecseq = 0;
    if (!err.empty() && parse_object(err, obj) && seq_field(obj, "cseq", ecseq) &&
        ecseq > error_seen_cseq && str_field(obj, "to") == channel) {
      error_seen_cseq = ecseq;
      c.error = str_field(obj, "error");
    }
    c.seen_rev = rev;
    c.seen_head = c.head;

    // The contiguous run ending at the newest held message.
    std::uint64_t oldest = 0, newest = 0;
    std::size_t count = 0;
    if (!c.held.empty()) {
      newest = oldest = c.held.rbegin()->first;
      count = 1;
      while (oldest > 0 && c.held.count(oldest - 1))
        --oldest, ++count;
    }
    if (count != c.count || oldest != c.oldest || newest != c.newest) {
      c.count = count;
      c.oldest = oldest;
      c.newest = newest;
      ++c.view_rev;
    }
  }

  info refresh(const std::string &channel, double now_s) {
    check_epoch();
    client_channel &c = channels[channel];
    const std::uint64_t rev = parse_u64(leaf(app, join2(root, "rev")));
    // The room's head is replicated; a conversation's comes with its pushes (and chat.rev).
    const std::uint64_t head =
        channel.empty() ? parse_u64(leaf(app, join2(chat_channel_path(room, channel), "head")))
                        : c.head;
    const bool retry = c.pending && now_s - c.pending_at >= kRetrySeconds;
    if (rev != c.seen_rev || head != c.seen_head || retry)
      scan(channel, c, rev);
    if (retry && c.pending)
      request_page(channel, c, c.pending_before, now_s);
    if (!c.pending && c.head > c.first) {
      if (c.count == 0) {
        request_page(channel, c, c.head, now_s); // the first view: the newest page
      } else if (c.newest + 1 < c.head) {
        if (c.gap_since < 0.0)
          c.gap_since = now_s;
        else if (now_s - c.gap_since >= kGapSeconds) {
          request_page(channel, c, c.head, now_s);
          c.gap_since = now_s;
        }
      } else {
        c.gap_since = -1.0;
      }
    }
    info out;
    out.available = !epoch.empty();
    out.rev = c.view_rev;
    out.count = c.count;
    out.head = c.head;
    out.first = c.first;
    out.more = c.count > 0 ? c.oldest > c.first : c.head > c.first;
    out.loading = c.pending;
    out.error = c.error;
    return out;
  }
};

room_chat_client::room_chat_client(cvc::app &app, std::string room, std::string pid,
                                   sender_fn send)
    : _impl(std::make_unique<impl>(app, std::move(room), std::move(pid), std::move(send))) {}

room_chat_client::~room_chat_client() = default;

bool room_chat_client::post(const std::string &text, const std::string &to, std::string &why) {
  impl &d = *_impl;
  std::string t = text;
  if (!clean_chat_text(t, d.limits.text_max, why))
    return false;
  if (!d.send(make_chat_post(d.room, d.pid, ++d.cseq, to, t))) {
    why = "not connected to a relay";
    return false;
  }
  d.channels[to].error.clear();
  return true;
}

room_chat_client::info room_chat_client::refresh(const std::string &channel, double now_s) {
  return _impl->refresh(channel, now_s);
}

std::vector<chat_message> room_chat_client::messages(const std::string &channel) const {
  std::vector<chat_message> out;
  auto it = _impl->channels.find(channel);
  if (it == _impl->channels.end() || it->second.count == 0)
    return out;
  const client_channel &c = it->second;
  out.reserve(c.count);
  for (auto m = c.held.find(c.oldest); m != c.held.end() && m->first <= c.newest; ++m)
    out.push_back(m->second);
  return out;
}

bool room_chat_client::more(const std::string &channel, double now_s) {
  impl &d = *_impl;
  (void)d.refresh(channel, now_s);
  client_channel &c = d.channels[channel];
  if (c.pending)
    return true;
  if (c.count == 0 ? c.head > c.first : c.oldest > c.first)
    d.request_page(channel, c, c.count == 0 ? c.head : c.oldest, now_s);
  return c.pending;
}

std::vector<room_chat_client::conversation> room_chat_client::conversations() {
  impl &d = *_impl;
  d.check_epoch();
  const std::uint64_t rev = parse_u64(leaf(d.app, join2(d.root, "rev")));
  if (rev == d.convs_rev)
    return d.convs;
  d.convs_rev = rev;
  d.convs.clear();
  const std::string list = leaf(d.app, join2(d.root, "dms"));
  std::size_t start = 0;
  while (start < list.size()) {
    const std::size_t comma = list.find(',', start);
    const std::string peer =
        list.substr(start, comma == std::string::npos ? std::string::npos : comma - start);
    if (!peer.empty())
      d.convs.push_back(
          {peer, parse_u64(leaf(d.app, join2(chat_channel_path(d.room, peer), "head")))});
    if (comma == std::string::npos)
      break;
    start = comma + 1;
  }
  return d.convs;
}

const std::string &room_chat_client::pid() const noexcept { return _impl->pid; }
const std::string &room_chat_client::room() const noexcept { return _impl->room; }

} // namespace conference
} // namespace ariadne
} // namespace cvc
