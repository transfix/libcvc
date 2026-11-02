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

#ifndef __CVC_ARIADNE_CONFERENCE_ROOM_CHAT_H__
#define __CVC_ARIADNE_CONFERENCE_ROOM_CHAT_H__

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace cvc {
class app;
class state_cluster_shard;
class state_transport;
struct state_message;

namespace ariadne {
namespace conference {

// ----------------
// Room chat and direct messages (docs/roadmap/ARIADNE_CHAT.md)
// ----------------
// The relay keeps the history of every channel -- the room's, and one conversation per pair of
// participants who message each other -- and is its only writer. None of it is in the relay's
// shared tree, which every client is sent whole (the admit snapshot, the WebSocket re-broadcast):
// each client is pushed what it may see, and only the connection holding its pid gets it
// (state_transport::push_snapshot_to_holder). In a client's tree, under conf.rooms.<room>.chat:
//
//   epoch, room.head, room.first        replicated from the relay's tree: the relay process's id,
//                                       the room's next seq and the oldest seq it still keeps
//   room.m.<seq>                        pushed: a room message, to every present participant as it
//                                       is posted, and a page of them to a client that asks
//   dms                                 pushed: the peers this participant has a conversation
//                                       with, comma-separated
//   dm.<peer>.{head,first,m.<seq>}      pushed to the two participants of a conversation only, each
//                                       holding it under the OTHER's pid
//   error                               pushed to a poster: {"cseq","to","error"} for a refused post
//   rev                                 pushed last in every push: changes each time, so a client
//                                       reads this leaf and the room head per tick and looks
//                                       further only when one of them moved
//
// A message leaf is one JSON object, {"seq","from","name","t","text","e"}: its seq in its
// channel, the sender's pid and its name when it posted, the relay's clock in seconds since the
// epoch, the text, and the relay's epoch (what an earlier relay process pushed is discarded).
//
// A client asks with a state_message on its OWN path, conf.rooms.<room>.participants.<pid>.chat,
// with ttl_hops 0: the relay takes it and forwards it to no one (room_write_gate refuses a chat
// message with any other ttl_hops). Its string_value is JSON:
//   {"op":"post","cseq":N,"to":"" | <pid>,"text":...}   post to the room, or a DM to <pid>
//   {"op":"page","channel":"" | <pid>,"before":SEQ}     push the page of messages before SEQ

// The relay's rules (and the page size a client asks in).
struct chat_limits {
  std::size_t page = 32;            // messages per page
  std::size_t room_keep = 5000;     // the room's history the relay keeps
  std::size_t dm_keep = 1000;       // each conversation's
  std::size_t text_max = 2000;      // bytes of text, once cleaned (clean_chat_text)
  std::size_t dm_channels_max = 64; // conversations one participant may have
  double rate_burst = 10.0;         // posts in a burst (a token bucket per participant)
  double rate_per_s = 2.0;          // refilled at this many a second
  bool dms = true;                  // DMs at all (arisrv turns them off without bound writes)
};

struct chat_message {
  std::uint64_t seq = 0;
  std::string from;  // the sender's pid
  std::string name;  // its name when it posted (its pid when it had none)
  double t = 0.0;    // the relay's clock: seconds since the Unix epoch
  std::string text;  // may hold newlines
  std::string epoch; // the relay process's epoch
};

// conf.rooms.<room>.chat, and a participant's request path, conf.rooms.<room>.participants.<pid>.chat.
std::string chat_root_path(const std::string &room);
std::string chat_request_path(const std::string &room, const std::string &pid);
// Where a client holds CHANNEL ("" the room, else the peer's pid): <chat root>.room or .dm.<peer>.
std::string chat_channel_path(const std::string &room, const std::string &channel);

std::string encode_chat_message(const chat_message &m);
bool decode_chat_message(const std::string &json, chat_message &out);

// The requests, as messages a client's shard sends (state_cluster_shard::send_message).
cvc::state_message make_chat_post(const std::string &room, const std::string &pid,
                                  std::uint64_t cseq, const std::string &to,
                                  const std::string &text);
cvc::state_message make_chat_page(const std::string &room, const std::string &pid,
                                  const std::string &channel, std::uint64_t before);

// Clean a post's text as the relay does: carriage returns and every control character except
// newline and tab dropped, trailing whitespace trimmed. False, with the reason in WHY, when what
// is left is empty, longer than TEXT_MAX bytes, or not UTF-8.
bool clean_chat_text(std::string &text, std::size_t text_max, std::string &why);

// ----------------
// room_chat_relay: the relay's side
// ----------------
// Subscribes to the shard's message bus for the room's chat requests (on transport reader
// threads, which only queue them); step() applies them on the relay's main loop: a post is
// checked (the sender present, the limits), stamped and appended, and pushed to who may see it; a
// page request is answered with a push to the requester. It writes epoch, room.head and room.first
// into the shard's tree, so the relay's drain fans them out with its other writes. Every ~5 s it
// re-pushes each present participant's conversation heads, so a client notices a DM push it lost.
//
// LIFETIME: the app, the shard and the added transports must outlive it.
// THREADING: everything but the bus callback on the relay's main loop.
class room_chat_relay {
public:
  struct stats {
    std::uint64_t posts = 0;   // messages appended, room and DMs
    std::uint64_t dms = 0;     // of them, DMs
    std::uint64_t pages = 0;   // page requests answered
    std::uint64_t refused = 0; // posts refused (the poster is pushed chat.error)
    std::uint64_t dropped = 0; // requests never applied: malformed, not the sender's, queue full
    std::uint64_t pushes = 0;  // push frames written (one per receiving connection)
  };

  room_chat_relay(cvc::app &app, cvc::state_cluster_shard &shard, std::string room,
                  chat_limits limits = {});
  ~room_chat_relay();

  room_chat_relay(const room_chat_relay &) = delete;
  room_chat_relay &operator=(const room_chat_relay &) = delete;

  // A transport that carries the pushes. Null or a repeat: ignored.
  void add_transport(cvc::state_transport *tp);

  // Apply the requests received since the last step. NOW_S: seconds on a steady clock.
  void step(double now_s);

  // A participant left: a conversation ends (is forgotten) once neither of its two is present.
  void participant_left(const std::string &pid);

  stats take_stats();
  const chat_limits &limits() const noexcept;
  const std::string &epoch() const noexcept;

private:
  // Defined in room_chat.cpp over file-local types, so its containers are not exported from
  // cvc.dll.
  struct impl;
  std::unique_ptr<impl> _impl;
};

// ----------------
// room_chat_client: a participant's side
// ----------------
// What this participant holds of each channel, read from its own tree as the relay pushed it,
// with the requests that keep it complete: the newest page the first time a channel is read, the
// page before the oldest message on more(), the newest page again when the room's head stays
// ahead of what it holds (a lost push), and a request unanswered for 3 s sent again. A channel
// shows the contiguous run of messages ending at the newest one held. One request per channel is
// in flight at a time.
//
// CHANNEL is "" for the room, or a peer's pid. NOW_S: seconds on a steady clock. One thread.
class room_chat_client {
public:
  // Sends a request; true when it went out. The default sends through the app's default
  // state_cluster_shard (send_message), which needs that shard's transport set.
  using sender_fn = std::function<bool(const cvc::state_message &)>;

  struct info {
    bool available = false;  // the relay keeps chat (it published chat.epoch)
    std::uint64_t rev = 0;   // changes whenever the messages held for the channel change
    std::size_t count = 0;   // messages in the contiguous run
    std::uint64_t head = 0;  // the channel's next seq, as last heard
    std::uint64_t first = 0; // the oldest seq the relay keeps
    bool more = false;       // older messages the relay keeps than the run holds
    bool loading = false;    // a page request is in flight
    std::string error;       // why the last post to this channel was refused ("" since a send)
  };
  struct conversation {
    std::string peer;
    std::uint64_t head = 0;
  };

  room_chat_client(cvc::app &app, std::string room, std::string pid, sender_fn send = nullptr);
  ~room_chat_client();

  room_chat_client(const room_chat_client &) = delete;
  room_chat_client &operator=(const room_chat_client &) = delete;

  // Post TEXT to the room (TO "") or to participant TO. False, with the reason in WHY, when the
  // text is empty or too long once cleaned, or it could not be sent.
  bool post(const std::string &text, const std::string &to, std::string &why);
  info refresh(const std::string &channel, double now_s);
  // The contiguous run, oldest first, as of the last refresh of the channel.
  std::vector<chat_message> messages(const std::string &channel) const;
  // Ask for the page before the oldest message held (the newest page when none is). True when a
  // request is in flight after the call.
  bool more(const std::string &channel, double now_s);
  // The conversations this participant has, in the order the relay lists them.
  std::vector<conversation> conversations();

  const std::string &pid() const noexcept;
  const std::string &room() const noexcept;

private:
  struct impl;
  std::unique_ptr<impl> _impl;
};

} // namespace conference
} // namespace ariadne
} // namespace cvc

#endif // __CVC_ARIADNE_CONFERENCE_ROOM_CHAT_H__
