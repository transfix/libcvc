/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#ifndef __CVC_STATE_MESSAGE_H__
#define __CVC_STATE_MESSAGE_H__

#include <cstdint>
#include <cvc/namespace.h>
#include <string>
#include <vector>

namespace CVC_NAMESPACE {

// ----------------
// cvc::state_message
// ----------------
// Out-of-band message that travels alongside state mutations but
// does NOT touch the change journal, the replica's vector clock, or
// the seen-set used for mutation loop suppression. Messages carry
// transient signals (UI events, cross-process commands, ephemeral
// notifications) that should reach subscribers without becoming
// part of replicated state.
//
// Identity:
//   (origin_node_id, message_id) is the dedup key. A receiving bus
//   admits each unique pair at most once.
//
// Routing:
//   `cluster_id` scopes delivery to peers in the same cluster, the
//   same way state_mutation.cluster_id does.
//   `path` is the tree path the message is associated with;
//   subscribers register a path prefix.
//   `ttl_hops` is a future cross-peer hop count (decremented on
//   each forward); 0 means "do not forward across peers".
//
// Payload:
//   `content_type` is a media type chosen by the sender. Use
//   media-type-like tags ("text/plain", "application/octet-stream",
//   "application/json", "application/x-cvc-geometry+protobuf",
//   etc.). When `content_type` is empty, effective_content_type()
//   returns MIME_TEXT if `bytes` is empty and `string_value` is
//   non-empty, MIME_OCTET if `bytes` is non-empty, and MIME_TEXT
//   for the fully-empty case.
//
//   Senders that want strict typing should set `content_type`
//   explicitly. The make_* helpers do this and are the recommended
//   way to construct messages.
//
//   `string_value` is a convenience for short text payloads.
//   `bytes` carries arbitrary binary payload. Both may be set, and
//   the receiver decides how to interpret them based on
//   `content_type`. For typed objects, prefer `bytes` plus an
//   application-defined media type.
//
struct state_message {
  static constexpr const char *MIME_TEXT = "text/plain";
  static constexpr const char *MIME_OCTET = "application/octet-stream";
  static constexpr const char *MIME_JSON = "application/json";

  std::string cluster_id;
  std::string origin_node_id;
  std::string message_id;
  std::string path;
  std::uint32_t ttl_hops = 8;
  std::string content_type;
  std::string string_value;
  std::vector<unsigned char> bytes;

  // Returns the explicit `content_type` if non-empty, otherwise
  // a default chosen from the payload shape. Never returns empty.
  std::string effective_content_type() const noexcept;

  // Plain-text payload. The resulting message has
  // content_type="text/plain" (or `mime` if specified), the text
  // in `string_value`, and an empty `bytes`.
  static state_message make_text(std::string path, std::string text, std::string mime = MIME_TEXT);

  // Untyped binary payload. The resulting message has
  // content_type="application/octet-stream", bytes set, and an
  // empty string_value.
  static state_message make_bytes(std::string path, std::vector<unsigned char> bytes);

  // Typed binary payload. Caller supplies an explicit media type;
  // both bytes and a sidecar string_value are settable so the
  // helper can build messages whose application-defined codec
  // wants both fields (e.g. JSON envelope + binary attachment).
  static state_message make_typed(std::string path, std::string content_type,
                                  std::vector<unsigned char> bytes,
                                  std::string string_value = std::string());
};

} // namespace CVC_NAMESPACE

#endif // __CVC_STATE_MESSAGE_H__
