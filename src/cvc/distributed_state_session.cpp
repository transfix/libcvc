/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <chrono>
#include <cvc/distributed_state_session.h>
#include <cvc/state_transport_inproc.h>
#ifndef _WIN32
#include <cvc/state_transport_ipc.h>
#endif
#ifdef CVC_ENABLE_GRPC
#include <cvc/state_transport_grpc.h>
#endif

namespace CVC_NAMESPACE {

distributed_state_session::distributed_state_session(const distributed_state_config &config)
    : _config(config) {}

distributed_state_session::~distributed_state_session() { stop(); }

std::shared_ptr<distributed_state_session>
distributed_state_session::join(app &ctx, const distributed_state_config &config) {
  // Use shared_ptr with private ctor via raw-new + shared_ptr(raw).
  auto session = std::shared_ptr<distributed_state_session>(new distributed_state_session(config));

  // 1. Create blob store.
  session->_blob_store = std::make_unique<memory_state_blob_store>();

  // 2. Create transport.
  switch (config.transport) {
  case transport_kind::inproc: {
    session->_transport = std::make_unique<state_transport_inproc>();
    break;
  }
  case transport_kind::ipc: {
#ifndef _WIN32
    auto ipc = std::make_unique<state_transport_ipc>();
    if (!config.listen_address.empty())
      ipc->start(config.listen_address, config.node_id, config.cluster_id);
    session->_transport = std::move(ipc);
#else
    throw std::runtime_error(
        "distributed_state_session: IPC transport is not available on Windows");
#endif
    break;
  }
  case transport_kind::grpc: {
#ifdef CVC_ENABLE_GRPC
    auto grpc = std::make_unique<state_transport_grpc>();
    if (!config.listen_address.empty())
      grpc->start(config.listen_address, config.node_id, config.cluster_id);
    session->_transport = std::move(grpc);
#else
    throw std::runtime_error(
        "distributed_state_session: gRPC transport requested but CVC_ENABLE_GRPC is not defined");
#endif
    break;
  }
  }

  // 3. Connect to seed peers.
  for (const auto &seed : config.seeds) {
#ifndef _WIN32
    if (auto *ipc = dynamic_cast<state_transport_ipc *>(session->_transport.get()))
      ipc->connect_to_peer(seed);
#endif
#ifdef CVC_ENABLE_GRPC
    if (auto *grpc = dynamic_cast<state_transport_grpc *>(session->_transport.get()))
      grpc->connect_to_peer(seed);
#endif
  }

  // 4. Create shard.
  session->_shard = std::make_unique<state_cluster_shard>(ctx, config.cluster_id, config.node_id,
                                                          config.root_path);

  // 5. Configure shard policies.
  session->_shard->set_enforce_authority(config.enforce_authority);
  session->_shard->set_enforce_write_policy(config.enforce_write_policy);
  session->_shard->set_enforce_delegation(config.enforce_delegation);
  session->_shard->set_resolve_conflicts(config.resolve_conflicts);
  session->_shard->set_enforce_interest(config.enforce_interest);

  // 6. Apply mounts.
  for (const auto &mount : config.mounts) {
    session->_shard->add_interest(mount.path);

    switch (mount.mode) {
    case sync_mode::authoritative:
      session->_shard->write_policy().allow(mount.path, {config.node_id});
      break;
    case sync_mode::read_only:
      // Interest registered but no write policy entry → reads only.
      break;
    case sync_mode::read_write:
    case sync_mode::delegated:
      break;
    }
  }

  // 7. Wire transport ↔ shard.
  session->_transport->register_shard(session->_shard.get());
  session->_shard->set_transport(session->_transport.get());

  // 8. Create admin facade.
  session->_admin = std::make_unique<state_distributed_admin>();
  session->_admin->attach_shard(session->_shard.get());
  session->_admin->attach_blob_store(session->_blob_store.get());

  // 9. Attach shard (start observing state changes).
  session->_shard->attach();
  session->_shard->install_as_default();

  // 10. Start pump thread.
  session->_running.store(true, std::memory_order_release);
  if (config.pump_interval_ms > 0) {
    session->_pump_thread = std::thread([session]() { session->pump_loop(); });
  }

  return session;
}

void distributed_state_session::stop() {
  if (!_running.exchange(false, std::memory_order_acq_rel))
    return;

  // 1. Join pump thread.
  if (_pump_thread.joinable())
    _pump_thread.join();

  // 2. Detach shard.
  if (_shard) {
    _shard->uninstall_as_default();
    _shard->detach();
  }

  // 3. Unregister shard from transport.
  if (_transport && _shard)
    _transport->unregister_shard(_shard.get());

    // 4. Stop transport.
#ifndef _WIN32
  if (auto *ipc = dynamic_cast<state_transport_ipc *>(_transport.get()))
    ipc->stop();
#endif
#ifdef CVC_ENABLE_GRPC
  if (auto *grpc = dynamic_cast<state_transport_grpc *>(_transport.get()))
    grpc->stop();
#endif
}

void distributed_state_session::sync_path(const std::string &path, sync_mode mode) {
  _shard->add_interest(path);

  switch (mode) {
  case sync_mode::authoritative:
    _shard->write_policy().allow(path, {_config.node_id});
    break;
  case sync_mode::read_only:
  case sync_mode::read_write:
  case sync_mode::delegated:
    break;
  }
}

void distributed_state_session::delegate(const std::string &path, const delegation_target &target) {
  _shard->publish_delegation(path, target.cluster_id, target.endpoint, target.lease_duration_ns);
}

void distributed_state_session::undelegate(const std::string &path) {
  _shard->publish_revocation(path);
}

void distributed_state_session::pump_loop() {
  const auto interval = std::chrono::milliseconds(_config.pump_interval_ms);
  while (_running.load(std::memory_order_acquire)) {
    _transport->pump_all();
    _pump_cycles.fetch_add(1, std::memory_order_relaxed);
    std::this_thread::sleep_for(interval);
  }
  // Final drain.
  _transport->pump_all();
}

} // namespace CVC_NAMESPACE
