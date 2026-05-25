/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <cvc/state_cluster_shard.h>
#include <cvc/state_message_bus.h>
#include <cvc/state_transport_ipc.h>
#include <fcntl.h>
#include <poll.h>
#include <stdexcept>
#include <string>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/un.h>
#include <unistd.h>

namespace cvc {

namespace {

constexpr std::uint32_t kMagic = 0x43564354u; // 'CVCT'
constexpr std::uint16_t kVersion = 1;
constexpr std::uint16_t kMaxVersion = 1; // highest version we understand
constexpr std::uint16_t kMsgHello = 1;
constexpr std::uint16_t kMsgMutation = 2;
constexpr std::uint16_t kMsgOob = 3;
constexpr std::uint16_t kMsgChunkReq = 4;
constexpr std::uint16_t kMsgChunkRsp = 5;
constexpr std::uint16_t kMsgSnapReq = 6;
constexpr std::uint16_t kMsgSnapRsp = 7;
constexpr std::size_t kMaxFrameBytes = 64u * 1024u * 1024u;

void put_u8(std::vector<unsigned char> &out, std::uint8_t v) { out.push_back(v); }

void put_u16(std::vector<unsigned char> &out, std::uint16_t v) {
  out.push_back(static_cast<unsigned char>(v & 0xff));
  out.push_back(static_cast<unsigned char>((v >> 8) & 0xff));
}

void put_u32(std::vector<unsigned char> &out, std::uint32_t v) {
  for (int i = 0; i < 4; ++i)
    out.push_back(static_cast<unsigned char>((v >> (8 * i)) & 0xff));
}

void put_u64(std::vector<unsigned char> &out, std::uint64_t v) {
  for (int i = 0; i < 8; ++i)
    out.push_back(static_cast<unsigned char>((v >> (8 * i)) & 0xff));
}

void put_string(std::vector<unsigned char> &out, const std::string &s) {
  put_u32(out, static_cast<std::uint32_t>(s.size()));
  out.insert(out.end(), s.begin(), s.end());
}

void put_bytes(std::vector<unsigned char> &out, const std::vector<unsigned char> &b) {
  put_u32(out, static_cast<std::uint32_t>(b.size()));
  out.insert(out.end(), b.begin(), b.end());
}

class reader {
public:
  reader(const unsigned char *p, std::size_t n) : _p(p), _end(p + n) {}

  bool ok() const { return _p <= _end; }
  std::size_t remaining() const { return static_cast<std::size_t>(_end - _p); }

  std::uint8_t u8() {
    if (remaining() < 1) {
      _p = _end + 1;
      return 0;
    }
    return *_p++;
  }
  std::uint16_t u16() {
    if (remaining() < 2) {
      _p = _end + 1;
      return 0;
    }
    std::uint16_t v = static_cast<std::uint16_t>(_p[0]) | (static_cast<std::uint16_t>(_p[1]) << 8);
    _p += 2;
    return v;
  }
  std::uint32_t u32() {
    if (remaining() < 4) {
      _p = _end + 1;
      return 0;
    }
    std::uint32_t v = 0;
    for (int i = 0; i < 4; ++i)
      v |= static_cast<std::uint32_t>(_p[i]) << (8 * i);
    _p += 4;
    return v;
  }
  std::uint64_t u64() {
    if (remaining() < 8) {
      _p = _end + 1;
      return 0;
    }
    std::uint64_t v = 0;
    for (int i = 0; i < 8; ++i)
      v |= static_cast<std::uint64_t>(_p[i]) << (8 * i);
    _p += 8;
    return v;
  }
  std::string str() {
    std::uint32_t n = u32();
    if (!ok() || remaining() < n) {
      _p = _end + 1;
      return {};
    }
    std::string s(reinterpret_cast<const char *>(_p), n);
    _p += n;
    return s;
  }
  std::vector<unsigned char> bytes() {
    std::uint32_t n = u32();
    if (!ok() || remaining() < n) {
      _p = _end + 1;
      return {};
    }
    std::vector<unsigned char> b(_p, _p + n);
    _p += n;
    return b;
  }

private:
  const unsigned char *_p;
  const unsigned char *_end;
};

std::vector<unsigned char> encode_mutation(const state_mutation &m) {
  std::vector<unsigned char> out;
  out.reserve(256);
  put_string(out, m.cluster_id);
  put_string(out, m.tree_id);
  put_string(out, m.origin_node_id);
  put_u64(out, m.sequence);
  put_string(out, m.mutation_id);
  put_string(out, m.path);
  put_u8(out, static_cast<std::uint8_t>(m.op));
  put_string(out, m.type_name);
  put_string(out, m.string_value);
  put_u8(out, static_cast<std::uint8_t>(m.payload.kind));
  switch (m.payload.kind) {
  case state_payload_kind::none:
    break;
  case state_payload_kind::inline_bytes:
    put_bytes(out, m.payload.inline_bytes);
    break;
  case state_payload_kind::blob:
    put_string(out, m.payload.blob.digest);
    put_u64(out, m.payload.blob.size_bytes);
    put_string(out, m.payload.blob.codec);
    break;
  }
  put_u8(out, m.latest_value_only ? 1 : 0);
  put_u64(out, m.hlc_time);
  return out;
}

bool decode_mutation(const std::vector<unsigned char> &bytes, state_mutation &out) {
  reader r(bytes.data(), bytes.size());
  out.cluster_id = r.str();
  out.tree_id = r.str();
  out.origin_node_id = r.str();
  out.sequence = r.u64();
  out.mutation_id = r.str();
  out.path = r.str();
  std::uint8_t op = r.u8();
  out.op = static_cast<state_mutation_op>(op);
  out.type_name = r.str();
  out.string_value = r.str();
  std::uint8_t kind = r.u8();
  out.payload.kind = static_cast<state_payload_kind>(kind);
  switch (out.payload.kind) {
  case state_payload_kind::none:
    break;
  case state_payload_kind::inline_bytes:
    out.payload.inline_bytes = r.bytes();
    break;
  case state_payload_kind::blob:
    out.payload.blob.digest = r.str();
    out.payload.blob.size_bytes = r.u64();
    out.payload.blob.codec = r.str();
    break;
  default:
    return false;
  }
  out.latest_value_only = r.u8() != 0;
  // hlc_time was added later; tolerate its absence for backward compat.
  if (r.ok() && r.remaining() >= 8)
    out.hlc_time = r.u64();
  return r.ok();
}

std::vector<unsigned char> encode_message(const state_message &m) {
  std::vector<unsigned char> out;
  out.reserve(128 + m.bytes.size());
  put_string(out, m.cluster_id);
  put_string(out, m.origin_node_id);
  put_string(out, m.message_id);
  put_string(out, m.path);
  put_u32(out, m.ttl_hops);
  put_string(out, m.content_type);
  put_string(out, m.string_value);
  put_bytes(out, m.bytes);
  return out;
}

bool decode_message(const std::vector<unsigned char> &bytes, state_message &out) {
  reader r(bytes.data(), bytes.size());
  out.cluster_id = r.str();
  out.origin_node_id = r.str();
  out.message_id = r.str();
  out.path = r.str();
  out.ttl_hops = r.u32();
  out.content_type = r.str();
  out.string_value = r.str();
  out.bytes = r.bytes();
  return r.ok();
}

bool write_all(int fd, const unsigned char *data, std::size_t n) {
  while (n > 0) {
    ssize_t w = ::send(fd, data, n, MSG_NOSIGNAL);
    if (w < 0) {
      if (errno == EINTR)
        continue;
      return false;
    }
    if (w == 0)
      return false;
    data += w;
    n -= static_cast<std::size_t>(w);
  }
  return true;
}

bool read_all(int fd, unsigned char *data, std::size_t n, const std::atomic<bool> &running) {
  while (n > 0) {
    if (!running.load(std::memory_order_acquire))
      return false;
    pollfd pfd{};
    pfd.fd = fd;
    pfd.events = POLLIN;
    int pr = ::poll(&pfd, 1, 200);
    if (pr < 0) {
      if (errno == EINTR)
        continue;
      return false;
    }
    if (pr == 0)
      continue;
    if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) {
      ssize_t r = ::recv(fd, data, n, 0);
      if (r <= 0)
        return false;
      data += r;
      n -= static_cast<std::size_t>(r);
      continue;
    }
    ssize_t r = ::recv(fd, data, n, 0);
    if (r < 0) {
      if (errno == EINTR)
        continue;
      return false;
    }
    if (r == 0)
      return false;
    data += r;
    n -= static_cast<std::size_t>(r);
  }
  return true;
}

} // namespace

struct state_transport_ipc::connection {
  int fd = -1;
  std::mutex write_mu;
  std::thread reader_thread;
  std::atomic<bool> alive{true};
  std::string remote_node_id;
  std::string remote_cluster_id;
};

state_transport_ipc::state_transport_ipc() = default;

state_transport_ipc::~state_transport_ipc() { stop(); }

void state_transport_ipc::start(const std::string &path, const std::string &node_id,
                                const std::string &cluster_id) {
  if (_running.load())
    throw std::runtime_error("state_transport_ipc::start: already running");

  _listen_path = path;
  _node_id = node_id;
  _cluster_id = cluster_id;

  if (path.size() >= sizeof(sockaddr_un{}.sun_path))
    throw std::runtime_error("state_transport_ipc::start: socket path too long");

  ::unlink(path.c_str());

  int fd = ::socket(AF_UNIX, SOCK_STREAM, 0);
  if (fd < 0)
    throw std::runtime_error(std::string("state_transport_ipc::start: socket: ") +
                             std::strerror(errno));

  sockaddr_un addr{};
  addr.sun_family = AF_UNIX;
  std::strncpy(addr.sun_path, path.c_str(), sizeof(addr.sun_path) - 1);
  if (::bind(fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
    int err = errno;
    ::close(fd);
    throw std::runtime_error(std::string("state_transport_ipc::start: bind: ") +
                             std::strerror(err));
  }
  if (::listen(fd, 8) < 0) {
    int err = errno;
    ::close(fd);
    throw std::runtime_error(std::string("state_transport_ipc::start: listen: ") +
                             std::strerror(err));
  }

  _listen_fd = fd;
  _running.store(true);
  _accept_thread = std::thread([this]() { accept_loop(); });
}

void state_transport_ipc::stop() {
  if (!_running.exchange(false)) {
    // Even if not running, make sure listener fd is closed.
    if (_listen_fd >= 0) {
      ::close(_listen_fd);
      _listen_fd = -1;
    }
    return;
  }

  if (_listen_fd >= 0) {
    ::shutdown(_listen_fd, SHUT_RDWR);
    ::close(_listen_fd);
    _listen_fd = -1;
  }
  if (_accept_thread.joinable())
    _accept_thread.join();

  std::vector<std::shared_ptr<connection>> conns;
  {
    std::lock_guard<std::mutex> lk(_conns_mu);
    conns.swap(_conns);
  }
  for (auto &c : conns) {
    if (c->fd >= 0) {
      ::shutdown(c->fd, SHUT_RDWR);
      ::close(c->fd);
      c->fd = -1;
    }
    c->alive.store(false);
    if (c->reader_thread.joinable())
      c->reader_thread.join();
  }

  if (!_listen_path.empty())
    ::unlink(_listen_path.c_str());
  _listen_path.clear();
}

void state_transport_ipc::accept_loop() {
  while (_running.load()) {
    pollfd pfd{};
    pfd.fd = _listen_fd;
    pfd.events = POLLIN;
    int pr = ::poll(&pfd, 1, 200);
    if (pr < 0) {
      if (errno == EINTR)
        continue;
      break;
    }
    if (pr == 0)
      continue;
    if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL))
      break;

    int cfd = ::accept(_listen_fd, nullptr, nullptr);
    if (cfd < 0) {
      if (errno == EINTR || errno == EAGAIN)
        continue;
      break;
    }

    auto conn = std::make_shared<connection>();
    conn->fd = cfd;
    {
      std::lock_guard<std::mutex> lk(_conns_mu);
      _conns.push_back(conn);
    }
    send_hello(*conn);
    conn->reader_thread = std::thread([this, conn]() { reader_loop(conn); });
  }
}

bool state_transport_ipc::connect_to_peer(const std::string &path,
                                          std::chrono::milliseconds timeout) {
  if (path.size() >= sizeof(sockaddr_un{}.sun_path))
    return false;

  int fd = ::socket(AF_UNIX, SOCK_STREAM, 0);
  if (fd < 0)
    return false;

  sockaddr_un addr{};
  addr.sun_family = AF_UNIX;
  std::strncpy(addr.sun_path, path.c_str(), sizeof(addr.sun_path) - 1);

  auto deadline = std::chrono::steady_clock::now() + timeout;
  while (true) {
    if (::connect(fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) == 0)
      break;
    if (errno == EINTR)
      continue;
    if (std::chrono::steady_clock::now() >= deadline) {
      ::close(fd);
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  auto conn = std::make_shared<connection>();
  conn->fd = fd;
  {
    std::lock_guard<std::mutex> lk(_conns_mu);
    _conns.push_back(conn);
  }
  send_hello(*conn);
  conn->reader_thread = std::thread([this, conn]() { reader_loop(conn); });
  return true;
}

void state_transport_ipc::send_hello(connection &c) {
  std::vector<unsigned char> body;
  put_string(body, _node_id);
  put_string(body, _cluster_id);
  std::lock_guard<std::mutex> lk(c.write_mu);
  write_frame_locked(c, kMsgHello, body);
}

bool state_transport_ipc::write_frame_locked(connection &c, std::uint16_t msg_type,
                                             const std::vector<unsigned char> &payload) {
  if (c.fd < 0 || !c.alive.load())
    return false;
  std::vector<unsigned char> hdr;
  hdr.reserve(12 + payload.size());
  put_u32(hdr, kMagic);
  put_u16(hdr, kVersion);
  put_u16(hdr, msg_type);
  put_u32(hdr, static_cast<std::uint32_t>(payload.size()));
  hdr.insert(hdr.end(), payload.begin(), payload.end());
  if (!write_all(c.fd, hdr.data(), hdr.size())) {
    c.alive.store(false);
    return false;
  }
  _sent_frames.fetch_add(1, std::memory_order_relaxed);
  return true;
}

void state_transport_ipc::reader_loop(std::shared_ptr<connection> conn) {
  while (_running.load() && conn->alive.load()) {
    unsigned char hdr[12];
    if (!read_all(conn->fd, hdr, sizeof(hdr), _running))
      break;

    reader hr(hdr, sizeof(hdr));
    std::uint32_t magic = hr.u32();
    std::uint16_t version = hr.u16();
    std::uint16_t mtype = hr.u16();
    std::uint32_t len = hr.u32();
    if (magic != kMagic || version > kMaxVersion || len > kMaxFrameBytes)
      break;

    std::vector<unsigned char> body(len);
    if (len > 0 && !read_all(conn->fd, body.data(), len, _running))
      break;

    _recv_frames.fetch_add(1, std::memory_order_relaxed);

    // Skip unknown message types instead of disconnecting, so that
    // older nodes can coexist with newer nodes that send new frame
    // types.
    if (mtype > kMsgSnapRsp && mtype != kMsgHello)
      continue;

    if (mtype == kMsgHello) {
      reader br(body.data(), body.size());
      conn->remote_node_id = br.str();
      conn->remote_cluster_id = br.str();
      continue;
    }
    if (mtype == kMsgMutation) {
      state_mutation m;
      if (decode_mutation(body, m)) {
        _recv_mutations.fetch_add(1, std::memory_order_relaxed);
        dispatch_inbound(m);
      }
      continue;
    }
    if (mtype == kMsgOob) {
      state_message m;
      if (decode_message(body, m)) {
        _recv_messages.fetch_add(1, std::memory_order_relaxed);
        dispatch_inbound_message(m);
      }
      continue;
    }
    if (mtype == kMsgChunkReq) {
      // Serve chunk from local blob store.
      reader br(body.data(), body.size());
      std::string digest = br.str();
      std::uint64_t req_id = br.u64();
      std::vector<unsigned char> rsp;
      put_string(rsp, digest);
      put_u64(rsp, req_id);
      if (_blob_store) {
        std::vector<unsigned char> chunk;
        if (_blob_store->get(digest, chunk)) {
          put_u8(rsp, 1); // found
          put_bytes(rsp, chunk);
        } else {
          put_u8(rsp, 0);
          put_u32(rsp, 0); // empty bytes
        }
      } else {
        put_u8(rsp, 0);
        put_u32(rsp, 0);
      }
      std::lock_guard<std::mutex> wlk(conn->write_mu);
      write_frame_locked(*conn, kMsgChunkRsp, rsp);
      continue;
    }
    if (mtype == kMsgChunkRsp) {
      reader br(body.data(), body.size());
      std::string digest = br.str();
      std::uint64_t req_id = br.u64();
      bool found = br.u8() != 0;
      std::vector<unsigned char> data = br.bytes();
      {
        std::lock_guard<std::mutex> lk(_chunk_waiters_mu);
        auto it = _chunk_waiters.find(req_id);
        if (it != _chunk_waiters.end()) {
          it->second->found = found;
          it->second->data = std::move(data);
          it->second->done = true;
        }
      }
      _chunk_waiters_cv.notify_all();
      continue;
    }
    if (mtype == kMsgSnapReq) {
      // Serve a snapshot request from a peer.
      reader br(body.data(), body.size());
      std::string cid = br.str();
      std::string prefix = br.str();
      std::uint64_t req_id = br.u64();

      // Walk registered shards, gather state entries.
      std::vector<unsigned char> rsp;
      put_u64(rsp, req_id);
      std::uint32_t entry_count = 0;
      std::size_t count_offset = rsp.size();
      put_u32(rsp, 0); // placeholder

      {
        std::lock_guard<std::mutex> slk(_shards_mu);
        for (auto *shard : _shards) {
          if (shard->cluster_id() != cid)
            continue;
          auto snap = shard->snapshot(prefix);
          for (const auto &e : snap) {
            put_string(rsp, e.path);
            put_string(rsp, e.string_value);
            put_string(rsp, e.comment);
            put_u8(rsp, e.hidden ? 1 : 0);
            put_u8(rsp, e.read_only ? 1 : 0);
            put_string(rsp, e.type_name);
            put_string(rsp, e.origin_node_id);
            put_u64(rsp, e.sequence);
            ++entry_count;
          }
        }
      }
      // Patch entry count.
      for (int i = 0; i < 4; ++i)
        rsp[count_offset + i] = static_cast<unsigned char>((entry_count >> (8 * i)) & 0xFF);

      std::lock_guard<std::mutex> wlk(conn->write_mu);
      write_frame_locked(*conn, kMsgSnapRsp, rsp);
      continue;
    }
    if (mtype == kMsgSnapRsp) {
      reader br(body.data(), body.size());
      std::uint64_t req_id = br.u64();
      std::uint32_t count = br.u32();
      std::vector<state_transport::snapshot_entry> entries;
      entries.reserve(count);
      for (std::uint32_t i = 0; i < count && br.ok(); ++i) {
        snapshot_entry e;
        e.path = br.str();
        e.string_value = br.str();
        e.comment = br.str();
        e.hidden = br.u8() != 0;
        e.read_only = br.u8() != 0;
        e.type_name = br.str();
        e.origin_node_id = br.str();
        e.sequence = br.u64();
        entries.push_back(std::move(e));
      }
      {
        std::lock_guard<std::mutex> lk(_snap_waiters_mu);
        auto it = _snap_waiters.find(req_id);
        if (it != _snap_waiters.end()) {
          it->second->entries = std::move(entries);
          it->second->done = true;
        }
      }
      _snap_waiters_cv.notify_all();
      continue;
    }
    // Unknown frame: skip silently.
  }

  conn->alive.store(false);
  if (conn->fd >= 0) {
    ::shutdown(conn->fd, SHUT_RDWR);
    // fd closed in stop() to avoid races with concurrent writers.
  }
}

void state_transport_ipc::register_shard(state_cluster_shard *shard) {
  if (shard == nullptr)
    return;
  std::lock_guard<std::mutex> lk(_shards_mu);
  if (std::find(_shards.begin(), _shards.end(), shard) == _shards.end())
    _shards.push_back(shard);
}

void state_transport_ipc::unregister_shard(state_cluster_shard *shard) {
  if (shard == nullptr)
    return;
  std::lock_guard<std::mutex> lk(_shards_mu);
  _shards.erase(std::remove(_shards.begin(), _shards.end(), shard), _shards.end());
}

state_transport::publish_stats state_transport_ipc::publish(const state_mutation &m) {
  publish_stats stats{};

  // Local in-process delivery first (matches inproc semantics for
  // shards registered on this endpoint).
  std::vector<state_cluster_shard *> local_peers;
  {
    std::lock_guard<std::mutex> lk(_shards_mu);
    local_peers.reserve(_shards.size());
    for (auto *s : _shards) {
      if (s == nullptr)
        continue;
      if (s->cluster_id() != m.cluster_id)
        continue;
      if (s->local_node_id() == m.origin_node_id)
        continue;
      local_peers.push_back(s);
    }
  }
  for (auto *peer : local_peers) {
    if (!_peers.should_deliver(peer->local_node_id(), m.path)) {
      _peers.note_delivery_filtered(peer->local_node_id());
      continue;
    }
    auto r = peer->ingest_remote(m);
    if (r.applied) {
      ++stats.delivered;
      _delivered.fetch_add(1, std::memory_order_relaxed);
      _peers.note_mutation_delivered(peer->local_node_id());
    } else if (r.duplicate) {
      ++stats.delivered;
      ++stats.duplicates;
    } else if (r.rejected) {
      ++stats.rejected;
    }
  }

  // Remote delivery: serialize once, send over every live connection.
  std::vector<unsigned char> body = encode_mutation(m);
  std::vector<std::shared_ptr<connection>> conns;
  {
    std::lock_guard<std::mutex> lk(_conns_mu);
    conns = _conns;
  }
  for (auto &c : conns) {
    if (!c || !c->alive.load())
      continue;
    if (!c->remote_node_id.empty() && !_peers.should_deliver(c->remote_node_id, m.path)) {
      _peers.note_delivery_filtered(c->remote_node_id);
      continue;
    }
    std::lock_guard<std::mutex> wlk(c->write_mu);
    if (write_frame_locked(*c, kMsgMutation, body)) {
      ++stats.delivered;
      if (!c->remote_node_id.empty())
        _peers.note_mutation_delivered(c->remote_node_id);
    }
  }

  _published.fetch_add(1, std::memory_order_relaxed);
  return stats;
}

std::size_t state_transport_ipc::pump_shard(state_cluster_shard &shard) {
  auto pending = shard.drain_local();
  for (const auto &m : pending)
    publish(m);
  return pending.size();
}

std::size_t state_transport_ipc::pump_all() {
  std::vector<state_cluster_shard *> snap;
  {
    std::lock_guard<std::mutex> lk(_shards_mu);
    snap = _shards;
  }
  std::size_t total = 0;
  for (auto *s : snap)
    if (s != nullptr)
      total += pump_shard(*s);
  return total;
}

void state_transport_ipc::flush() {
  // Writes are synchronous under each connection's write mutex,
  // so by the time publish() returns, frames are on the wire.
  // Briefly take/release each connection's write mutex to ensure
  // any concurrent in-flight publish() has finished.
  std::vector<std::shared_ptr<connection>> conns;
  {
    std::lock_guard<std::mutex> lk(_conns_mu);
    conns = _conns;
  }
  for (auto &c : conns) {
    if (!c)
      continue;
    std::lock_guard<std::mutex> wlk(c->write_mu);
    (void)wlk;
  }
}

state_transport::publish_message_stats
state_transport_ipc::publish_message(const state_message &m) {
  publish_message_stats stats{};

  // Local in-process delivery.
  std::vector<state_cluster_shard *> local_peers;
  {
    std::lock_guard<std::mutex> lk(_shards_mu);
    local_peers.reserve(_shards.size());
    for (auto *s : _shards) {
      if (s == nullptr)
        continue;
      if (!m.cluster_id.empty() && s->cluster_id() != m.cluster_id)
        continue;
      if (s->local_node_id() == m.origin_node_id)
        continue;
      local_peers.push_back(s);
    }
  }
  for (auto *peer : local_peers) {
    if (!_peers.should_deliver(peer->local_node_id(), m.path)) {
      _peers.note_delivery_filtered(peer->local_node_id());
      continue;
    }
    if (peer->ingest_remote_message(m)) {
      ++stats.delivered;
      _peers.note_message_delivered(peer->local_node_id());
    } else
      ++stats.duplicates;
  }

  // Remote delivery.
  std::vector<unsigned char> body = encode_message(m);
  std::vector<std::shared_ptr<connection>> conns;
  {
    std::lock_guard<std::mutex> lk(_conns_mu);
    conns = _conns;
  }
  for (auto &c : conns) {
    if (!c || !c->alive.load())
      continue;
    if (!c->remote_node_id.empty() && !_peers.should_deliver(c->remote_node_id, m.path)) {
      _peers.note_delivery_filtered(c->remote_node_id);
      continue;
    }
    std::lock_guard<std::mutex> wlk(c->write_mu);
    if (write_frame_locked(*c, kMsgOob, body)) {
      ++stats.peers;
      if (!c->remote_node_id.empty())
        _peers.note_message_delivered(c->remote_node_id);
    }
  }

  return stats;
}

void state_transport_ipc::dispatch_inbound(const state_mutation &m) {
  std::vector<state_cluster_shard *> peers;
  {
    std::lock_guard<std::mutex> lk(_shards_mu);
    peers.reserve(_shards.size());
    for (auto *s : _shards) {
      if (s == nullptr)
        continue;
      if (s->cluster_id() != m.cluster_id)
        continue;
      if (s->local_node_id() == m.origin_node_id)
        continue;
      peers.push_back(s);
    }
  }
  for (auto *peer : peers) {
    auto r = peer->ingest_remote(m);
    if (r.applied)
      _delivered.fetch_add(1, std::memory_order_relaxed);
  }
}

std::size_t state_transport_ipc::shard_count() const {
  std::lock_guard<std::mutex> lk(_shards_mu);
  return _shards.size();
}

std::size_t state_transport_ipc::connection_count() const {
  std::lock_guard<std::mutex> lk(_conns_mu);
  std::size_t n = 0;
  for (auto &c : _conns)
    if (c && c->alive.load())
      ++n;
  return n;
}

std::uint64_t state_transport_ipc::wait_for_received(std::uint64_t target,
                                                     std::chrono::milliseconds timeout) {
  auto deadline = std::chrono::steady_clock::now() + timeout;
  while (_recv_mutations.load() < target) {
    if (std::chrono::steady_clock::now() >= deadline)
      break;
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  return _recv_mutations.load();
}

std::uint64_t state_transport_ipc::wait_for_received_messages(std::uint64_t target,
                                                              std::chrono::milliseconds timeout) {
  auto deadline = std::chrono::steady_clock::now() + timeout;
  while (_recv_messages.load() < target) {
    if (std::chrono::steady_clock::now() >= deadline)
      break;
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  return _recv_messages.load();
}

void state_transport_ipc::dispatch_inbound_message(const state_message &m) {
  std::vector<state_cluster_shard *> peers;
  {
    std::lock_guard<std::mutex> lk(_shards_mu);
    peers.reserve(_shards.size());
    for (auto *s : _shards) {
      if (s == nullptr)
        continue;
      if (!m.cluster_id.empty() && s->cluster_id() != m.cluster_id)
        continue;
      if (s->local_node_id() == m.origin_node_id)
        continue;
      peers.push_back(s);
    }
  }
  for (auto *peer : peers)
    (void)peer->ingest_remote_message(m);
}

bool state_transport_ipc::fetch_chunk(const std::string &digest, chunk_callback on_chunk) {
  if (digest.empty())
    return false;

  // Try local blob store first.
  if (_blob_store) {
    std::vector<unsigned char> bytes;
    if (_blob_store->get(digest, bytes)) {
      if (on_chunk)
        on_chunk(digest, bytes);
      return true;
    }
  }

  // Ask connected peers.
  std::uint64_t req_id = _next_chunk_req_id.fetch_add(1, std::memory_order_relaxed);
  auto waiter = std::make_shared<chunk_waiter>();
  {
    std::lock_guard<std::mutex> lk(_chunk_waiters_mu);
    _chunk_waiters[req_id] = waiter;
  }

  // Build and send request to all live connections.
  std::vector<unsigned char> req_body;
  put_string(req_body, digest);
  put_u64(req_body, req_id);

  std::vector<std::shared_ptr<connection>> conns;
  {
    std::lock_guard<std::mutex> lk(_conns_mu);
    conns = _conns;
  }
  std::size_t sent = 0;
  for (auto &c : conns) {
    if (!c || !c->alive.load())
      continue;
    std::lock_guard<std::mutex> wlk(c->write_mu);
    if (write_frame_locked(*c, kMsgChunkReq, req_body))
      ++sent;
  }

  bool result = false;
  if (sent > 0) {
    // Wait for the first response (up to 10 seconds).
    std::unique_lock<std::mutex> lk(_chunk_waiters_mu);
    _chunk_waiters_cv.wait_for(lk, std::chrono::seconds(10), [&]() { return waiter->done; });
    if (waiter->done && waiter->found) {
      if (on_chunk)
        on_chunk(digest, waiter->data);
      result = true;
    }
  }

  // Clean up waiter.
  {
    std::lock_guard<std::mutex> lk(_chunk_waiters_mu);
    _chunk_waiters.erase(req_id);
  }

  return result;
}

bool state_transport_ipc::request_snapshot(const std::string &cluster_id,
                                           const std::string &path_prefix,
                                           snapshot_callback on_entries) {
  std::uint64_t req_id = _next_snap_req_id.fetch_add(1, std::memory_order_relaxed);
  auto waiter = std::make_shared<snap_waiter>();
  {
    std::lock_guard<std::mutex> lk(_snap_waiters_mu);
    _snap_waiters[req_id] = waiter;
  }

  // Build request frame.
  std::vector<unsigned char> req_body;
  put_string(req_body, cluster_id);
  put_string(req_body, path_prefix);
  put_u64(req_body, req_id);

  std::vector<std::shared_ptr<connection>> conns;
  {
    std::lock_guard<std::mutex> lk(_conns_mu);
    conns = _conns;
  }
  std::size_t sent = 0;
  for (auto &c : conns) {
    if (!c || !c->alive.load())
      continue;
    std::lock_guard<std::mutex> wlk(c->write_mu);
    if (write_frame_locked(*c, kMsgSnapReq, req_body))
      ++sent;
  }

  bool result = false;
  if (sent > 0) {
    std::unique_lock<std::mutex> lk(_snap_waiters_mu);
    _snap_waiters_cv.wait_for(lk, std::chrono::seconds(10), [&]() { return waiter->done; });
    if (waiter->done) {
      if (on_entries)
        on_entries(waiter->entries, /*final=*/true);
      result = true;
    }
  }

  {
    std::lock_guard<std::mutex> lk(_snap_waiters_mu);
    _snap_waiters.erase(req_id);
  }
  return result;
}

} // namespace cvc
