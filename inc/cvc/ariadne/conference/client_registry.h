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

#ifndef __CVC_ARIADNE_CONFERENCE_CLIENT_REGISTRY_H__
#define __CVC_ARIADNE_CONFERENCE_CLIENT_REGISTRY_H__

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cvc/state/state_exec/types.h> // value_t
#include <deque>
#include <map>
#include <set>
#include <string>
#include <vector>

namespace cvc {
class app;
struct state_link_report;
} // namespace cvc

namespace cvc {
namespace ariadne {
namespace conference {

// ----------------
// The relay's record of its clients (docs/roadmap/ARISRV_IDENTITY.md, ID1)
// ----------------
// Every client that connects gets a client id: "c" and 31 hex digits of a salted hash of its
// device id (state_identity.h's default_device_id, scoped to the relay, as its Hello carries it),
// so it is also a state name. A client that sends no device id is hashed on its addresses
// instead. Each id gets a record: its username, its device tag, and
// a log of its connections with their addresses and whether each is still open.
//
// The id is an identifier, not a credential: anyone who knows a device's inputs can compute its
// device id. `verified` says whether the device proved its key (state_device_key.h).
//
// The relay writes two things from it, each changed-only:
//   conf.rooms.<room>.participants.<pid>.profile.name   the username, relay-owned
//                                                       (room_write_gate refuses a client's write)
//   arisrv.clients.<client id>.*                         the record (kRoot), its connections at
//                                                       connections.n<seq>; outside conf and every
//                                                       publish: namespace, so no client is sent it
//
// One registry per relay, owned by its main(), used on the relay's main loop only (no locks).
class client_registry {
public:
  // The private root the records are mirrored under.
  static constexpr const char *kRoot = "arisrv.clients";

  struct options {
    std::string room;                 // the room whose participants' names it writes
    std::string salt;                 // the id hash's salt; "" makes a random one
    std::size_t max_clients = 4096;   // records kept; the inactive one seen longest ago goes first
    std::size_t max_connections = 32; // connections kept in each record's log
  };

  struct connection {
    std::uint64_t seq =
        0;           // its number in its record's log (arisrv.clients.<id>.connections.n<seq>)
    std::string pid; // the node id its Hello named
    std::string transport;      // "grpc" | "ws"
    std::string public_address; // the address the relay saw, without its port
    std::string local_address; // the client's own end, as its Hello reported it ("" from a browser)
    std::string opened, closed; // UTC, ISO 8601; closed "" while it is open
    bool active = false;
  };

  struct record {
    std::string id;       // the client id
    std::string username; // the server's name for it
    std::string device; // its room-scoped device tag (room_device_tag), "" until it proves its key
    bool verified = false;
    std::string first_seen, last_seen; // UTC, ISO 8601
    bool active = false;               // any connection open
    std::deque<connection> connections;
    std::uint64_t next_seq = 1;
    std::deque<double> renames; // when it renamed (update()'s clock), for the rate limit
    double seen_s = 0.0;        // when it was last seen active (update()'s clock), for eviction
    bool dirty = true;          // its fields changed since they were last mirrored
    bool log_dirty = true;      // its connections changed since they were last mirrored
  };

  // Why a request was refused: a room-call error code (room_calls.h) and its message.
  struct refusal {
    std::string code, message;
  };

  explicit client_registry(options o);

  // The client id for a device id, or, when DEVICE_ID is "", for its two addresses.
  std::string client_id(const std::string &device_id, const std::string &local_address,
                        const std::string &public_address) const;

  // One pass over the relay's links (state_transport::link_reports, both transports'), at NOW_S
  // seconds on a steady clock and WALL, the same moment in UTC (utc_iso). An accepted link that has
  // sent its Hello and is new opens a connection in its client's record (made on first sight, with
  // a guest name). One gone since the last pass is closed.
  void update(const std::vector<cvc::state_link_report> &links, double now_s,
              const std::string &wall);

  // The client id behind the connection holding PID; "" when none is known. #1290's ledger keys on
  // it.
  std::string identity_key(const std::string &pid) const;
  const record *find(const std::string &client_id) const;
  const record *record_of(const std::string &pid) const;
  // The records, by id.
  const std::map<std::string, record> &records() const noexcept { return _records; }

  // Rename PID's client to REQUESTED (cleaned by clean_username). False, with WHY, when PID has no
  // record (`unavailable`), the name is not one (`invalid`), another active client has it
  // (`invalid`, unless MAKE_UNIQUE, which adds "-2", "-3", ... instead: room_join's request), or it
  // renamed kMaxRenames times in the last minute (`rate_limited`). The same name is a success that
  // changes nothing.
  bool set_username(const std::string &pid, const std::string &requested, double now_s,
                    refusal &why, bool make_unique = false);
  static constexpr std::size_t kMaxRenames = 4;

  // PID's record as server.whoami answers it: {client_id, username, verified, device, admin}.
  // Nil when PID has none.
  cvc::state_exec::value_t whoami(const std::string &pid) const;

  // Write each PRESENT pid's profile.name from its record, and mirror every record under kRoot.
  void write(cvc::app &app, const std::set<std::string> &present);

  // A requested username cleaned: trimmed, 1-32 bytes of valid UTF-8 with no control characters.
  // False when nothing usable is left.
  static bool clean_username(const std::string &in, std::string &out);
  static constexpr std::size_t kUsernameMax = 32;
  // An address as a transport reports it ("ipv4:1.2.3.4:5678", "ipv6:[::1]:5678", "1.2.3.4:5678")
  // without its scheme, brackets or port.
  static std::string address_host(const std::string &address);
  // TP in UTC, ISO 8601 to the second ("2026-11-16T01:02:03Z").
  static std::string utc_iso(std::chrono::system_clock::time_point tp);

private:
  record *find_mut(const std::string &id);
  record *record_of_mut(const std::string &pid);
  // NAME made unique among the active records other than SELF, by a "-2", "-3", ... suffix.
  std::string unique_name(const std::string &name, const std::string &self) const;
  bool name_taken(const std::string &name, const std::string &self) const;
  record &admit(const std::string &id, double now_s, const std::string &wall);
  void evict_one();
  void mirror(cvc::app &app);

  struct link_state {
    std::string client_id;
    std::uint64_t seq = 0;
    std::string pid;
    bool holds = false;
  };

  options _opt;
  std::map<std::string, record> _records;
  std::map<std::string, link_state> _links;    // by "<transport>#<connection>"
  std::map<std::string, std::string> _holders; // pid -> its holding link's key
  std::map<std::string, std::string> _written; // pid -> the profile.name last written
  std::set<std::string> _mirrored;             // record ids under kRoot
  std::map<std::string, std::set<std::uint64_t>> _mirrored_seqs; // id -> connection seqs there
  bool _dirty = true;
};

} // namespace conference
} // namespace ariadne
} // namespace cvc

#endif // __CVC_ARIADNE_CONFERENCE_CLIENT_REGISTRY_H__
