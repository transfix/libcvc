/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#ifndef __CVC_STATE_MESSAGE_H__
#define __CVC_STATE_MESSAGE_H__

#include <cvc/namespace.h>

#include <cstdint>
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
//   `content_type` is an opaque media-type-like tag chosen by the
//   sender. `bytes` is arbitrary opaque payload. `string_value` is
//   provided for cheap text messages so simple senders/receivers do
//   not need to encode/decode bytes.
//
struct state_message {
  std::string cluster_id;
  std::string origin_node_id;
  std::string message_id;
  std::string path;
  std::uint32_t ttl_hops = 8;
  std::string content_type;
  std::string string_value;
  std::vector<unsigned char> bytes;
};

} // namespace CVC_NAMESPACE

#endif // __CVC_STATE_MESSAGE_H__
