# Distributed State Synchronization Roadmap

Date: May 19, 2026

## Purpose

Add an optional distributed synchronization layer for `cvc::state` so multiple libcvc processes, potentially on different machines, can share one logical state tree in near real time. The feature should preserve the current local API while adding explicit cluster membership, conflict handling, large-object transport, subtree delegation, and low-latency fanout across many nodes.

The current state API is a per-`cvc::app` tree of dot-separated paths. Nodes store string values, arbitrary `boost::any` data, metadata (`comment`, `hidden`, `readOnly`, `lastMod`, `initialized`), and emit Boost.Signals2 callbacks when local values, data, or children change. This roadmap adds a synchronization layer around that API rather than changing normal state access patterns.

## Goals

- Keep existing `cvc::state` call sites working unchanged for local state.
- Allow a process to join a cluster and mirror selected paths from a shared logical state tree.
- Make low latency a primary design constraint for scalar and metadata mutations. The common path should avoid disk I/O, whole-tree scans, unnecessary serialization, and per-mutation connection setup.
- Scale to many nodes in a cluster with subscriptions, sharding, fanout routing, bounded queues, and backpressure rather than broadcasting every mutation to every process.
- Support a large, complex network of trees and delegated subtrees, where many clusters can own different path prefixes and clients can resolve, cache, and subscribe across that network.
- Propagate local changes to peers with causal metadata, loop prevention, and deterministic conflict rules.
- Support typed scalar values, strings, node metadata, child creation/removal/reset/touch, and arbitrary data objects.
- Move very large objects, including volumes and geometry, without forcing all bytes through protobuf messages.
- Allow subtrees to be delegated to another cluster or authority, similar to DNS subdelegation.
- Make the feature optional in CMake and keep libcvc usable with no networking dependencies.
- Provide security primitives suitable for multi-machine clusters: TLS, authentication, authorization, and path-level write policy.

## Non-Goals For The First Version

- Fully general distributed transactions across many state paths.
- Automatic semantic merge of arbitrary C++ objects.
- Replacing `boost::any` inside `cvc::state` immediately.
- Replicating every transient callback side effect. The replicated object is the state mutation log and resulting tree, not arbitrary local thread execution history.
- Guaranteeing deterministic hard real-time deadlines. Low latency is central to the design, but the first version should promise observable latency budgets and backpressure behavior rather than hard real-time scheduling guarantees.

## Recommended Direction

Use gRPC for control-plane and mutation-stream communication, protobuf for compact typed mutation envelopes, and a separate chunked/blob transport for large object payloads. Keep the distributed layer beside `cvc::state` as an attachable service rather than embedding networking directly in every setter. The hot path should be an already-open bidirectional stream carrying compact mutation envelopes; setup, discovery, snapshots, and blob transfers should not block small state updates.

Application code should continue to write through `app.root()(path).value(...)` and `data(...)`. A `distributed_state_session` subscribes to local state signals, converts local mutations into replicated operations, applies remote operations back to the local tree under a guarded apply scope, and emits normal local signals so existing `state_object` handlers continue to work.

## Architecture

1. `state_change_journal`

   Local append-only queue of mutations observed from `cvc::state`. It records path, operation type, node metadata, value/data type names, origin node, sequence number, logical timestamp, payload reference, and durability status.

2. `state_sync_adapter`

   Bridge between the in-memory state tree and the distributed layer. It connects to local signals and provides a remote-apply guard so replicated writes do not get echoed back as new local writes.

3. `state_replica`

   Maintains replica metadata: cluster membership, known peers, vector/version clocks, last applied sequence per peer, path ownership, active leases, snapshots, and in-flight blob transfers.

4. `state_subscription_router`

   Maintains path-prefix subscriptions and routes each mutation only to interested local consumers, peer streams, or delegated authorities. It should use longest-prefix indexes or tries so routing cost scales with path depth, not total tree size.

5. `state_cluster_shard`

   Partitions authority and fanout responsibility across a cluster. A cluster may have one authority for a small deployment, but the architecture should allow multiple shard leaders keyed by path prefix or hash ranges so many-node clusters do not bottleneck on a single process.

6. `state_transport_grpc`

   Implements bidirectional streaming RPCs for peer sync, snapshots, membership, leases, and delegation referrals.

7. `state_blob_store`

   Stores large payloads content-addressably by hash. It can be backed by local filesystem, memory for tests, shared filesystem, object storage, or future RDMA/zero-copy paths. The mutation stream carries descriptors; peers fetch or stream chunks separately.

8. `state_codec_registry`

   Maps libcvc type names to serializers/deserializers. Scalars can be encoded directly in protobuf. Large libcvc objects such as `cvc::volume` and `cvc::geometry` should initially use existing file I/O handlers into a canonical transfer format, preferably HDF5/CVC for volumes and a simple binary or existing supported geometry format for meshes.

9. `state_authority_map`

   Tracks which cluster has write authority for each subtree, how referrals are resolved, and how leases are acquired/renewed.

## Latency And Scale Requirements

Low latency and many-node scale should be designed in from the first prototype.

- Scalar and metadata updates should travel over persistent bidirectional streams.
- Local setters should enqueue mutations quickly and return after local state is updated. Network send, persistence, and remote acknowledgments should run asynchronously unless the caller explicitly asks for a durability or quorum wait.
- Use small protobuf envelopes for values and metadata. Avoid JSON in the hot path.
- Coalesce bursts on the same path when subscribers only need the latest state, while preserving ordered mutation logs for peers that request replay.
- Prioritize control and scalar mutations ahead of large blob chunk traffic on shared links.
- Keep callback dispatch local and immediate. Remote replication should not hold state-node mutexes while doing network or disk work.
- Use membership/gossip or a lightweight control-plane authority so nodes can discover peers without all-to-all manual configuration.
- Route by path-prefix subscriptions. A renderer subscribed to `scene.camera` should not receive every `simulation.volume.brick.*` update.
- Support cluster sharding by path prefix or hash so no single process must fan out all mutations in a large deployment.
- Use bounded per-peer queues, backpressure signals, and drop/coalesce policies for paths that are marked latest-value-only.
- Treat every shared tree as a named namespace (`tree_id`) with its own authority map, subscriptions, and retention policy.
- Use longest-prefix delegation and referral caching so clients can traverse a network of delegated subtrees without centralizing every lookup.
- Provide diagnostics for route resolution, authority ownership, subscription fanout, and cross-cluster latency under `__system.distributed.*`.

## Large Object Strategy

Large payloads must not be inlined in mutation messages. Use content-addressed blob manifests containing digest, size, codec, and chunk descriptors.

First implementation:

- Small values and small data under a configurable threshold, e.g. 64 KiB, are inlined.
- Large data is serialized to the local `state_blob_store` and referenced by digest.
- Peers receiving the mutation check whether the digest exists locally.
- Missing chunks are fetched by streaming RPC with backpressure.
- A node applies metadata after the manifest is known, but may expose the data as lazy/resolving until bytes are available. Strict consumers can use wait APIs.
- Volumes should eventually support subvolume/range fetches so a renderer can request visible bricks instead of waiting for a full multi-gigabyte object.

## Subtree Delegation

Delegation should behave like a mount/referral system. A cluster can own the root tree but delegate `simulation.volume` to another cluster. The parent cluster stores a delegation record at or above that path; clients route writes and subscriptions to the authoritative cluster for that subtree.

Routing rules:

- Longest-prefix match chooses the responsible cluster for any path.
- Local cluster may cache delegated subtree state but cannot write without a valid lease/authority token.
- Delegation changes are mutations in the parent authority's tree and must be versioned.
- The target cluster exports a subtree root that maps to the delegated path; internally it may store it as its own root or under a mount path.
- Referrals should be followed automatically by the session and exposed through diagnostics.

## Implementation Phases

### Phase 0: Design Spike

- Review desired consistency semantics.
- Decide whether the first version is peer-to-peer, leader-based per cluster, or supports both.
- Define initial latency budgets for scalar updates, metadata updates, blob manifests, and blob hydration on localhost, LAN, and WAN-like links.
- Define cluster scale targets for the first production design, including expected nodes per cluster, trees per process, path count, mutation rate, and subscriber count.
- Choose canonical codecs for `cvc::volume` and `cvc::geometry`.
- Decide whether snapshots should include `boost::any` data immediately or begin with value/metadata plus selected codecs.

### Phase 1: Local Mutation Journal

- Add internal mutation structs and a journal with unit tests.
- Add `state_sync_adapter` that observes local state changes and records mutations.
- Add path-prefix subscription indexes and latest-value coalescing locally, before networking.
- Add remote-apply guard to prevent echo loops.
- Add tests for value, data, metadata, reset, touch, child creation, and callback firing.
- No networking yet.

### Phase 2: Codec Registry And Blob Store

- Add `state_codec_registry` with built-in scalar/string codecs.
- Add file-backed content-addressed blob store.
- Add codecs for `cvc::geometry` and `cvc::volume` using existing I/O handlers or HDF5 where available.
- Add chunk manifests, hash verification, resume support, and tests using generated large payloads without committing large files.

### Phase 3: Single-Machine gRPC Sync

- Add protobuf schema and generated C++ integration.
- Implement bidirectional mutation stream between two processes on localhost.
- Support initial snapshot, incremental mutations, acknowledgments, reconnect, and replay from journal.
- Measure latency for scalar mutations and callback delivery; fail tests or benchmarks when regressions exceed chosen budgets.
- Add integration tests that launch two libcvc test processes and verify values/data stay synchronized.

### Phase 4: Multi-Node Cluster Semantics

- Add membership, node identity, TLS, auth tokens/certs, and per-path write policy.
- Add subscription routing and path-prefix fanout so many-node clusters do not rely on all-to-all broadcast.
- Add shard ownership for path prefixes or hash ranges, with resharding hooks for later operations tooling.
- Add conflict detection, deterministic conflict resolution, backpressure, and observability under `__system.distributed.*`.

### Phase 5: Subtree Delegation

- Add authority map and longest-prefix routing.
- Implement `ResolvePath`, delegation records, lease acquisition, lease renewal, and referral following.
- Add tests for root cluster delegating a subtree to a second cluster and clients receiving the right updates.
- Add failure-mode tests for expired delegation, unreachable delegated cluster, and authority transfer.

### Phase 6: Performance And Production Hardening

- Benchmark scalar update fanout, callback latency, snapshot size, blob throughput, and reconnect recovery.
- Benchmark large-tree routing with millions of paths and many delegated prefixes.
- Benchmark many-node fanout with selective subscriptions and slow-peer isolation.
- Add delta/chunk updates for volumes and geometry arrays.
- Add compression options per codec and per network link.
- Add admin tooling for inspection, manual resync, and blob garbage collection.
- Document operational patterns for small interactive clusters and larger distributed processing clusters.

## Testing Requirements

Every production unit added for this feature must have unit tests in the same change or an explicitly documented reason why it cannot be tested yet. The network implementation must not be considered complete until it has integration, stress, and performance coverage.

Required layers:

- Unit tests for mutation construction, journal replay, subscription routing, echo suppression, codecs, blob chunking, lease logic, and route resolution.
- Multi-process integration tests once transport exists. These should launch separate libcvc test helpers, communicate over localhost, verify bidirectional sync, verify callback delivery, and exercise reconnect/replay.
- Large-object integration tests using generated volumes/geometry and a temporary blob store.
- Many-node simulation tests with selective subscriptions, shard routing, slow-peer backpressure, and resync from snapshots.
- Large tree-network tests with many delegated prefixes and referral-cache invalidation.
- Stress tests gated by an explicit option or environment variable so normal CI remains fast, but nightly/opt-in runs can push path counts, mutation rates, and node counts.
- Performance tests that record scalar enqueue latency, remote delivery latency, callback latency, routing cost, blob throughput, snapshot time, and reconnect recovery time. These tests should start as reporting benchmarks, then gain thresholds once baseline data is stable.
- Failure tests for duplicate mutations, out-of-order delivery, partial blob transfers, stale leases, split authority, slow peers, and transport interruption.

## First Milestone

Build a prototype that synchronizes string values and metadata between two local processes over gRPC, with the adapter/journal architecture already in place and the hot path shaped for low latency. Do not start with large objects or delegation in code, but do include subscription routing and latency measurement so the prototype can grow toward many nodes instead of being rewritten.

The first merged milestone should include:

- new optional CMake flag and generated protobuf build;
- `distributed_state_session::join()` for localhost peer sync;
- scalar value and metadata replication;
- persistent bidirectional streams with no per-mutation connection setup;
- path-prefix subscription routing, even if the first test has only two processes;
- initial snapshot on join;
- journal replay after reconnect;
- basic latency benchmark output for local enqueue and remote callback delivery;
- tests that prove existing local callbacks still fire on remote updates.
