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
#include <optional>
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
// An rcon login (ID3) elevates one connection, the one holding a pid, until it closes or another
// link takes the pid over; rcon_guard below limits wrong passwords.
//
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

  // What a client says it is (ID5a, ARISRV_IDENTITY.md 2.7): a claim, used to route and offer
  // services, never to grant anything. clean_client() makes a valid one of anything.
  struct client_info {
    std::string kind = "unknown"; // an app's name ("nicotina"), else its host's ("arigl"), or
                                  // "nmdc"; "unknown" when it said nothing
    std::string version;          // free text for the log (a libcvc version, an NMDC tag)
    std::vector<std::string> capabilities; // the services it can use: "chat", "files.share", ...
  };

  // Where a connection's address is (@host.geolocate, ID4c): any field may be "" (GeoLite2 City
  // has no ASN).
  struct geo_info {
    std::string country, region, city, asn;
    bool empty() const noexcept {
      return country.empty() && region.empty() && city.empty() && asn.empty();
    }
  };

  struct connection {
    std::uint64_t seq =
        0;           // its number in its record's log (arisrv.clients.<id>.connections.n<seq>)
    std::string pid; // the node id its Hello named; "" for a program's connection
    std::string transport;      // "grpc" | "ws", or a program's listener's ("nmdc", "nmdcs", ...)
    std::string public_address; // the address the relay saw, without its port
    std::string local_address; // the client's own end, as its Hello reported it ("" from a browser)
    std::string opened, closed; // UTC, ISO 8601; closed "" while it is open
    bool active = false;
    bool engine = false; // a program's connection (ID5b): no Hello, no pid, no room presence
    std::string key;     // the program's key for it (ID5b); "" for a transport's
    client_info client;
    geo_info geo;
  };

  // A client's own caps, where an admin set them (server.limits); an unset field keeps the
  // server document's (ServerDocument::set_client_limits, ID4a, which also holds them to the
  // document's caps or not, by its switch). Kept by client id, so a reconnect keeps them while the
  // relay runs. Python guests (ID4c) add a `python` flag.
  struct client_limits {
    std::optional<bool> programs, guests; // false refuses the client that kind; true enables
                                          // nothing the document doesn't
    std::uint64_t steps = 0;              // 0: the document's
    double time = 0.0;                    // seconds; 0: the document's
    std::uint64_t memory_mb = 0;          // a guest's linear memory too; 0: the document's
    bool empty() const noexcept {
      return !programs && !guests && steps == 0 && time == 0.0 && memory_mb == 0;
    }
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
    bool banned = false;        // refused at each connection (server.ban), until unbanned
    std::uint64_t rcon_failures = 0; // wrong rcon passwords its connections sent
    client_limits limits;            // where an admin set them (server.limits)
    client_info client; // what its newest open connection says it is (the last one's, once none is)
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
  // Write PID's profile.name from its record now, present or not: a rename is in the room before
  // anything the client sends after asking for it (a chat post right after room-join, say). The
  // roster shows no participant until its presence.joined, so an early name shows nothing.
  void write_name(cvc::app &app, const std::string &pid);

  // A requested username cleaned: trimmed, 1-32 bytes of valid UTF-8 with no control characters.
  // False when nothing usable is left.
  static bool clean_username(const std::string &in, std::string &out,
                             std::size_t max = kUsernameMax);
  static constexpr std::size_t kUsernameMax = 32;
  // An address as a transport reports it ("ipv4:1.2.3.4:5678", "ipv6:[::1]:5678", "1.2.3.4:5678")
  // without its scheme, brackets or port.
  static std::string address_host(const std::string &address);
  // TP in UTC, ISO 8601 to the second ("2026-11-16T01:02:03Z").
  static std::string utc_iso(std::chrono::system_clock::time_point tp);

  // -------- rcon and the admin's calls (ID3) --------

  // Whether the connection holding PID is elevated by rcon.
  bool is_admin(const std::string &pid) const;
  // Elevate the connection holding PID, or drop its elevation; false when no known link holds it.
  bool set_admin(const std::string &pid, bool on);
  // A wrong rcon password from PID's connection, for its record.
  void note_rcon_failure(const std::string &pid);

  // The connection holding PID, as rcon checks it: the address the relay saw and whether it is
  // TLS. False when no known link holds it.
  struct holder {
    std::string public_address;
    bool secure = false;
  };
  bool holder_of(const std::string &pid, holder &out) const;

  // One open connection: as a transport numbers it (state_link_report::connection), or, for a
  // program's connection (ID5b), by the program's key, which the relay hands the program's closer
  // with the reason and kick or ban, so the program can tell the client why before it closes.
  struct link_ref {
    std::string transport;        // "grpc" | "ws", or the program's listener's ("nmdc", ...)
    std::uint64_t connection = 0; // a transport's; 0 for a program's
    std::string key;              // a program's; "" for a transport's
  };
  // The open connections of client ID_OR_PID (a client id, or the pid its connection holds):
  // what server.kick closes.
  std::vector<link_ref> links_of(const std::string &id_or_pid) const;
  // Ban (or unban) client ID: each connection it makes is closed as soon as a pass sees it. False
  // for an unknown id.
  bool ban(const std::string &id, bool on);
  // The open connections of banned clients that update() found, for the relay to close.
  std::vector<link_ref> take_banned_links();
  // Rename client ID as an admin: no rate limit, and still unique among the active clients.
  bool rename(const std::string &id, const std::string &name, refusal &why);
  // Set client ID's limit KEY, or with nil take it back to the document's: programs and guests
  // are true or false, steps and memory_mb whole numbers not below 0, time a number of seconds not
  // below 0 (0 is the document's too). False, with WHY (`invalid`), for an unknown id or key, or a
  // value of the wrong type.
  bool set_limit(const std::string &id, const std::string &key,
                 const cvc::state_exec::value_t &value, refusal &why);
  // PID's client's limits; nullopt when it has no record or none are set. On the relay's room
  // loop, as every call here is (ServerDocument asks from run_client_*, which runs there).
  std::optional<client_limits> limits(const std::string &pid) const;

  // -------- a program's connections (ID5b, ARISRV_IDENTITY.md 2.8) --------
  //
  // A connection a relay's program accepts itself (an NMDC client, on the net engine or on its
  // own sockets) has no Hello and no pid. The program reports it, from its accept hook
  // (host.clients.open_engine), under a key of its own that it doesn't reuse while the connection
  // is open (1-128 bytes of [A-Za-z0-9._:-], such as "nmdc:41"). It is kept until close_engine:
  // update() leaves it alone. Each call is on the room loop, as the rest are.

  // Open the program's connection KEY, accepted on its listener TRANSPORT ("nmdc", "nmdcs", ...),
  // in the record of its addresses' client id (client_id's fallback: it sends no device id), with
  // what CLIENT says it is and, when the program looked it up, where it is (GEO). Returns the
  // client id; "" with WHY: `forbidden` for a banned client (the program closes it), `invalid`
  // for a KEY already open or that is no key, or a TRANSPORT that is no short lowercase name.
  std::string open_engine(const std::string &key, const std::string &transport,
                          const std::string &public_address, const std::string &local_address,
                          const client_info &client, const geo_info &geo, double now_s,
                          const std::string &wall, refusal &why);
  // The username KEY's protocol gives its client (an NMDC nick): cleaned as set_username cleans
  // one, but up to kProgramUsernameMax bytes (NMDC's nicks), and unique among the active clients,
  // never with a suffix (a renamed NMDC nick breaks its client): false with WHY, `invalid` for no
  // name or one another active client has (the program refuses the login), `unavailable` for no
  // such connection.
  bool name_engine(const std::string &key, const std::string &nick, refusal &why);
  static constexpr std::size_t kProgramUsernameMax = 64;
  // What KEY says it is now: its protocol told, or changed it (an NMDC $MyINFO, a share that went
  // to 0). False for no such connection.
  bool update_engine(const std::string &key, const client_info &client);
  // KEY closed at WALL. False for no such connection.
  bool close_engine(const std::string &key, const std::string &wall);

  // CLIENT made valid: a kind of 1-32 bytes of [a-z0-9._-] ("unknown" otherwise), a version of at
  // most 64 printable ASCII bytes, and capabilities of 1-64 bytes of [a-z0-9._-] each, at most 32,
  // in order with no repeats; anything else is dropped.
  static client_info clean_client(const client_info &client);

  // Every record, active ones first, then the latest seen (server.clients): a list of
  // {client_id, username, active, admin, banned, verified, device, first_seen, last_seen, kind,
  // capabilities}.
  cvc::state_exec::value_t list_value() const;
  // One record (server.client): the list's fields, its limits, its rcon failures and its
  // connection log. Nil for an unknown id.
  cvc::state_exec::value_t record_value(const std::string &id) const;

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
    std::string transport;
    std::uint64_t connection = 0;
    std::string public_address;
    bool secure = false;
    bool engine = false; // a program's connection: close_engine ends it, not update()
    std::string key;     // the program's key for it
  };
  bool record_admin(const std::string &id) const;
  // REC's client: its newest active connection's, else its newest connection's.
  static void take_client(record &rec);
  static std::string engine_key(const std::string &key);
  static bool valid_program_key(const std::string &key);
  cvc::state_exec::value_t summary(const record &rec) const;

  options _opt;
  std::map<std::string, record> _records;
  std::map<std::string, link_state> _links;    // by "<transport>#<connection>"
  std::map<std::string, std::string> _holders; // pid -> its holding link's key
  std::map<std::string, std::string> _written; // pid -> the profile.name last written
  std::set<std::string> _mirrored;             // record ids under kRoot
  std::map<std::string, std::set<std::uint64_t>> _mirrored_seqs; // id -> connection seqs there
  bool _dirty = true;
  std::set<std::string> _admins;  // the keys of the links rcon elevated
  std::vector<link_ref> _to_kick; // banned clients' links, for take_banned_links
};

// How many wrong rcon passwords a relay takes (ID3): after kFreeTries from one address, or by one
// client, it checks no more from that address or client for kFirstLockS, and twice as long after
// each further one (at most kMaxLockS). A right password clears both.
class rcon_guard {
public:
  static constexpr int kFreeTries = 3;
  static constexpr double kFirstLockS = 30.0;
  static constexpr double kMaxLockS = 3600.0;
  // Seconds until a try from ADDRESS by client CLIENT may be checked; 0 when it may be now.
  double wait(const std::string &address, const std::string &client, double now_s) const;
  void failed(const std::string &address, const std::string &client, double now_s);
  void succeeded(const std::string &address, const std::string &client);

private:
  struct entry {
    int failures = 0;
    double lock_s = 0.0, locked_until = 0.0, last_s = 0.0;
  };
  void fail_one(const std::string &key, double now_s);
  std::map<std::string, entry> _by; // "a:<address>", "c:<client id>"
  static constexpr std::size_t kMaxEntries = 8192;
};

} // namespace conference
} // namespace ariadne
} // namespace cvc

#endif // __CVC_ARIADNE_CONFERENCE_CLIENT_REGISTRY_H__
