/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#ifndef __CVC_STATE_IDENTITY_H__
#define __CVC_STATE_IDENTITY_H__

#include <cvc/core/namespace.h>
#include <string>

namespace cvc {

// ----------------
// Peer identity helpers (ARIADNE_CONFERENCE_ROADMAP §5f)
// ----------------
// Two credentials a peer can carry before there are usernames:
//
//   room token  A shared secret the relay checks before it takes a connection
//               (state_transport_grpc / state_transport_ws auth_config): who may join at all.
//   device id   A stable identifier for the device a peer runs on, sent in its Hello. It is NOT
//               a secret -- it is computed from values others can know -- so it recognises a
//               returning device but proves nothing.

// Whether `a` and `b` are equal, in time that depends only on their lengths (a token check that
// does not leak how much of a guess was right).
bool constant_time_equal(const std::string &a, const std::string &b) noexcept;

// Whether `token` can be a room token: 16 to 256 characters from [A-Za-z0-9._~-]. The WebSocket
// transport carries it in a Sec-WebSocket-Protocol entry, which allows no other characters.
bool valid_room_token(const std::string &token) noexcept;

// Whether `id` can be a device id on the wire: 1 to 64 characters from [A-Za-z0-9._-]. A receiver
// ignores any other (an oversized one would cost it work on every link report).
bool valid_device_id(const std::string &id) noexcept;

// This device's default identity: CVC_DEVICE_ID when set (a test, or a browser page that keeps
// a random id in localStorage, since browsers expose no hardware identity), otherwise the first
// 32 hex digits of a salted SHA-256 over the OS machine id (/etc/machine-id, /var/lib/dbus/
// machine-id or /etc/hostid; the Windows MachineGuid; the macOS host UUID), the host name and the
// OS user. The raw values never leave the process, and the salt keeps the hash from matching
// another program's. Stable across launches on one machine and user; the same for two clients
// started there; "" when none of the inputs could be read, and always "" under Emscripten
// without CVC_DEVICE_ID (a browser has no machine inputs, and its constant environment would give
// every browser the same id).
std::string default_device_id();

// `device_id` as one relay sees it: the first 32 hex digits of a salted SHA-256 over SCOPE (the
// relay's address) and the id, so two relays (two operators) cannot match one device's ids. ""
// for an empty id. A client sends this, not the raw default_device_id().
std::string scoped_device_id(const std::string &device_id, const std::string &scope);

} // namespace cvc

#endif // __CVC_STATE_IDENTITY_H__
