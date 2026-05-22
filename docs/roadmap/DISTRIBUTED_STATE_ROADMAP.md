# Distributed State Synchronization Roadmap

Date: May 22, 2026

## Status Snapshot

- Phases 1, 2, 3a, 3b, 3c, 3d, 3e, 4, 5, 6 are landed on `master` (mutation journal + adapter, codec registry + chunked blob store, replica/authority map, cluster shard, transport interface + inproc/ipc/gRPC, OOB messaging, multi-node cluster semantics with TLS + write policy, subtree delegation with leases, admin facade, per-codec compression, per-peer bounded outbox).
- Phase 8 (link nodes) slices 4a–4e landed via PR #78; writable transparent links and cycle-collapse tests landed via PR #79. Slice 6 (resolveRemote + cross-cluster link tests) in flight on `feature/phase8-slice6-resolve-remote`:
  - 4a: inbound interest filter on subscription routing.
  - 4b: `link_mode` (transparent/opaque) on link nodes plus `resolvedValue` with hop budget.
  - 4c: `state_distributed_admin::transparent_link_index` and `transparent_link_aliases`.
  - 4d: `state_sync_adapter::subscriptions_for_path` expands subscriptions through transparent aliases with a cumulative forwarded-through-link counter.
  - 4e: alias resolver follows transparent-link chains BFS-style to a fixed point with a `hop_budget` (default 64), root-target guard, and cycle termination.
  - Writable transparent links: `state::linkWritable()` / `setLinkWritable(bool)` opt a transparent link into write-through; covered by `StateWritableLinkTest` (11 cases).
  - Subscription-collapse over N-link cycles: covered by `StateSyncAdapterLinkForwardingTest`.
  - `state_distributed_admin::link_cycles()` static cycle enumerator.
  - Slice 6: `state::resolveRemote(hop_budget)` — pull-on-demand remote link resolution composing with delegation + lease expiry. Cross-cluster link tests covering authority transfer, lease expiry invalidation, and sendMessage routing over transparent links without the caller naming a cluster_id.
- Phase 7 (perf + production hardening) and Phase 9 (network analytics) are not started.

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
- Provide an out-of-band node-to-node messaging channel that does not mutate state, with TTL-bounded propagation across the state tree and across the cluster.
- Support symbolic *link* nodes that reference another path (in the same tree, another tree, or another cluster), including link-to-root and other cycles, with deterministic cycle detection.
- Provide live network analytics — bandwidth, latency, cluster/tree size, per-node footprint — distributed via the OOB messaging channel so any node can render a real-time picture of the cluster.

## Non-Goals For The First Version

- Fully general distributed transactions across many state paths.
- Automatic semantic merge of arbitrary C++ objects.
- Replacing `boost::any` inside `cvc::state` immediately.
- Replicating every transient callback side effect. The replicated object is the state mutation log and resulting tree, not arbitrary local thread execution history.
- Guaranteeing deterministic hard real-time deadlines. Low latency is central to the design, but the first version should promise observable latency budgets and backpressure behavior rather than hard real-time scheduling guarantees.

## Recommended Direction

Use gRPC (provided by libcvc-deps v1.1.0) for cross-host control-plane and mutation-stream communication, protobuf for compact typed mutation envelopes, and a separate chunked/blob transport for large object payloads. For same-host work, prefer a faster transport (in-process for tests and embedded multi-shard, fast IPC for cross-process). Keep the distributed layer beside `cvc::state` as an attachable service rather than embedding networking directly in every setter. The hot path should be an already-open bidirectional stream carrying compact mutation envelopes; setup, discovery, snapshots, and blob transfers should not block small state updates.

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

6. `state_transport` (interface) plus concrete transports

   Pluggable peer-to-peer carrier for mutation envelopes, snapshots, control messages, and out-of-band messages. See "Transport Tier Design" below.

7. `state_blob_store`

   Stores large payloads content-addressably by hash. It can be backed by local filesystem, memory for tests, shared filesystem, object storage, or future RDMA/zero-copy paths. The mutation stream carries descriptors; peers fetch or stream chunks separately.

8. `state_codec_registry`

   Maps libcvc type names to serializers/deserializers. Scalars can be encoded directly in protobuf. Large libcvc objects such as `cvc::volume` and `cvc::geometry` should initially use existing file I/O handlers into a canonical transfer format, preferably HDF5/CVC for volumes and a simple binary or existing supported geometry format for meshes.

9. `state_authority_map`

   Tracks which cluster has write authority for each subtree, how referrals are resolved, and how leases are acquired/renewed.

10. `state_message_bus`

    Out-of-band notification channel. Dispatches non-state messages to per-node signals and propagates them along the tree (and optionally to peers via the transport) under a TTL.

## Transport Tier Design

Transports are pluggable behind a small `state_transport` interface so the cluster shard, replica, and message bus do not depend on any specific wire format. Three concrete transports are planned:

- `state_transport_inproc` — pure in-memory delivery between shards in the same process. Used for unit tests, integration tests across many simulated peers, and any embedded scenario where a single process hosts multiple state trees that should appear as a cluster.
- `state_transport_ipc` — fast same-host cross-process transport. First implementation uses a UNIX domain socket with length-prefixed framing; a later variant can use a shared-memory ring buffer if benchmarks justify it.
- `state_transport_grpc` — bidirectional streaming gRPC over TCP/TLS, used cross-host or cross-host-equivalent (containers on a virtual network). Built on the gRPC and protobuf shipped in libcvc-deps v1.1.0; gated behind a CMake flag so libcvc still builds without networking.

The interface carries a small set of operations: `register_shard`, `unregister_shard`, `send_mutation(cluster_id, mutation)`, `send_message(cluster_id, message)`, `flush()`. Implementations are free to batch or coalesce on the wire as long as observable per-shard ordering and loop-detection metadata are preserved.

### gRPC Overhead And When To Bypass It

Order-of-magnitude latency on a modern Linux box:

- gRPC over loopback TCP: ~30–100 μs per RPC; bidirectional streaming amortizes well so steady-state per-message cost is the lower end of that range.
- UNIX domain sockets with simple length-prefixed framing: ~5–20 μs.
- Shared-memory ring buffer with a futex/eventfd wakeup: typically <1 μs.

For cross-host traffic, gRPC dominates anything we'd realistically build ourselves and is the right default. For same-host clusters with high-frequency scalar updates (renderers, simulation visualizers, brick-streamers) the difference is large enough to matter, so a same-host transport is worth implementing.

The plan: ship gRPC first for any non-trivial deployment, ship `state_transport_inproc` alongside it for tests, and add `state_transport_ipc` once we have benchmark numbers showing same-host RPC cost is a real bottleneck. The `state_transport` interface is designed so swapping transports is configuration, not source change.

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

## Out-Of-Band Messaging

Some application traffic is not a state mutation. Examples include "render this frame", "user clicked", "begin recording", or progress pings. These should not be encoded as state writes (every write would persist in the journal, fan out to subscribers, and contribute to vector clocks). The system needs a separate notification path that rides on the same transports.

Design:

- Each `cvc::state` node exposes a `messageReceived(const state_message&)` Boost.Signals2 signal.
- A `state_message` carries: a target path (the node it is delivered to), an opaque payload (`std::string` of bytes; codec is the application's choice), a `max_depth` (TTL, hop count from the original target), an origin node id, a message id, and a timestamp.
- `app.root()(path).sendMessage(payload, max_depth)` (or an equivalent free function) dispatches a message to the target node.
- On receipt at any node, the node fires `messageReceived` with depth equal to the hops already taken. If `max_depth > 0` after the local fire, the node forwards a copy with `max_depth - 1` to all immediate children. Nodes track recently-seen `(origin, message_id)` pairs to suppress duplicates.
- Cluster-wide propagation is opt-in: the message bus offers the message to the local `state_transport`, which delivers it to peers with the same TTL semantics. Peers re-enter the propagation algorithm at the target path. The same `(origin, message_id)` deduplication prevents loops across peers.
- Messages do not write to the journal and do not advance vector clocks. Delivery is best-effort under backpressure: bounded per-peer queues, drop on overflow, with a counter under `__system.distributed.messages.*`.

This is intentionally similar in spirit to IP-packet TTL plus a multicast tree: scope is bounded by `max_depth`, loops are bounded by the dedup set, and the path namespace gives natural addressing without an extra topic system.

## Implementation Phases

### Phase 0: Design Spike

- Review desired consistency semantics.
- Decide whether the first version is peer-to-peer, leader-based per cluster, or supports both.
- Define initial latency budgets for scalar updates, metadata updates, blob manifests, and blob hydration on localhost, LAN, and WAN-like links.
- Define cluster scale targets for the first production design, including expected nodes per cluster, trees per process, path count, mutation rate, and subscriber count.
- Choose canonical codecs for `cvc::volume` and `cvc::geometry`.
- Decide whether snapshots should include `boost::any` data immediately or begin with value/metadata plus selected codecs.

### Phase 1: Local Mutation Journal (done)

- Add internal mutation structs and a journal with unit tests.
- Add `state_sync_adapter` that observes local state changes and records mutations.
- Add path-prefix subscription indexes and latest-value coalescing locally, before networking.
- Add remote-apply guard to prevent echo loops.
- Add tests for value, data, metadata, reset, touch, child creation, and callback firing.

### Phase 2: Codec Registry And Blob Store (done)

- Add `state_codec_registry` with built-in scalar/string codecs.
- Add file/memory-backed content-addressed blob store.
- Add codecs for `cvc::geometry` and `cvc::volume` using existing I/O handlers or HDF5 where available.
- Add chunk manifests, hash verification, resume support, and tests using generated large payloads without committing large files.

### Phase 3a: Replica And Authority Map (done)

- Add per-peer vector-clock metadata, last-applied tracking, and a "seen set" for echo suppression.
- Add longest-prefix authority/delegation lookup with tests.

### Phase 3b: Cluster Shard Binding (done)

- Per-tree shard struct binding journal + router + adapter + replica + authority map + blob store + codec registry, with unit tests and stress/perf gates.

### Phase 3c: Transport Layer (done)

- Define the `state_transport` interface and an inproc concrete `state_transport_inproc` for tests and embedded multi-shard scenarios.
- Wire `state_cluster_shard` so it can publish local mutations through a transport and ingest remote ones via the existing remote-apply path on the adapter.
- Tests: 2-shard convergence (set on A → B observes), loop suppression on round trip, fan-out across N shards, vector-clock equality after quiescence, opt-in stress and perf gates.

### Phase 3d: Same-Host Fast IPC Transport (done)

- `state_transport_ipc` over UNIX domain sockets with length-prefixed framing.
- Benchmark against gRPC loopback.
- Optional shared-memory ring buffer variant if numbers justify the complexity. (deferred to Phase 7.)

### Phase 3e: gRPC Transport (done)

- Add protobuf schema and generated C++ integration (using gRPC and protobuf from libcvc-deps v1.1.0).
- `state_transport_grpc` with bidirectional streaming for mutations, snapshots, control, and out-of-band messages.
- Reconnect, replay from journal, initial snapshot on join, latency benchmarks gated behind CMake flag.

### Phase 4: Out-Of-Band Messaging (done)

- Add `state_message`, `state_message_bus`, and the `messageReceived` signal on `cvc::state`.
- Add `sendMessage` API and TTL-bounded local tree propagation with `(origin, id)` dedup.
- Extend each transport (`inproc`, `ipc`, `grpc`) to carry messages alongside mutations with the same dedup metadata so messages can cross peer boundaries.
- Tests: local propagation depth, dedup across multi-path delivery, cross-peer propagation, drop-under-backpressure counters, message does not appear in journal and does not advance clocks.

### Phase 5: Multi-Node Cluster Semantics (done)

- Add membership, node identity, TLS, auth tokens/certs, and per-path write policy.
- Add subscription routing and path-prefix fanout so many-node clusters do not rely on all-to-all broadcast.
- Add shard ownership for path prefixes or hash ranges, with resharding hooks for later operations tooling.
- Add conflict detection, deterministic conflict resolution, backpressure, and observability under `__system.distributed.*`.

### Phase 6: Subtree Delegation (done)

- Add authority map and longest-prefix routing wired into the live transport (already-implemented map gets driven by real referrals).
- Implement `ResolvePath`, delegation records, lease acquisition, lease renewal, and referral following.
- Add tests for root cluster delegating a subtree to a second cluster and clients receiving the right updates.
- Add failure-mode tests for expired delegation, unreachable delegated cluster, and authority transfer.
- Phase-6 follow-ons that also landed alongside: admin facade (`state_distributed_admin`), blob GC, typed message payloads + bounded-queue backpressure, per-peer outbox, chunked blob writer/reader with manifest + resume, per-codec compression options.

### Phase 7: Performance And Production Hardening

- Benchmark scalar update fanout, callback latency, snapshot size, blob throughput, reconnect recovery, and message propagation depth/latency.
- Benchmark large-tree routing with millions of paths and many delegated prefixes.
- Benchmark many-node fanout with selective subscriptions and slow-peer isolation.
- Add delta/chunk updates for volumes and geometry arrays.
- Add compression options per codec and per network link.
- Add admin tooling for inspection, manual resync, and blob garbage collection.
- Document operational patterns for small interactive clusters and larger distributed processing clusters.

### Phase 8: Link Nodes (Symbolic References) (in progress)

State trees need *link* nodes that hold no data of their own and instead point to another path in the same tree, in another cluster, or at the root of an entire tree. Links are the distributed-tree analog of a symlink and let us share subtrees, mount remote clusters, and build graph-shaped views over a tree-shaped store.

Delivered so far on `feature/distributed-state-sync` (PR #78), PR #79, and `feature/phase8-slice6-resolve-remote`:

- 4a — inbound interest filter: subscription router exposes `subscriptions_for(path)` with longest-prefix semantics so the adapter can drive interest-based dispatch.
- 4b — `link_mode` (`transparent`/`opaque`) on link nodes; `state::resolvedValue(path, hop_budget = 64)` follows transparent links to a value with cycle detection.
- 4c — `state_distributed_admin::transparent_link_index(root)` enumerates `{link_path, target_path}` entries (root target canonicalized to empty); `transparent_link_aliases(root, path, hop_budget)` returns aliases for a given path, dot-segment boundary-aware so prefix spoofing is rejected.
- 4d — `state_sync_adapter::subscriptions_for_path(path)` returns direct-router subscriptions plus subscriptions installed at every transparent alias, deduped by subscription id; `forwarded_through_link_count()` reports cumulative subscriptions added via alias expansion. The local-dispatch path now uses this expanded lookup.
- 4e — alias resolver follows chains BFS-style to a fixed point with a `hop_budget` (default 64). A root-target / at-or-under-link guard terminates 2-cycles and prevents nested re-aliasing.
- Writable transparent links: `state::linkWritable()` / `setLinkWritable(bool)` opt a transparent link into write-through. When enabled, `state::value(v)` routes the write to the resolved terminal target with the same cycle/budget semantics as `resolvedValue()`; broken/cyclic resolves raise `read_only_error`. Default is `false`, preserving existing semantics where writes land on the link node. Covered by `StateWritableLinkTest` (11 cases).
- Subscription-collapse over N-link cycles: `StateSyncAdapterLinkForwardingTest` covers two-link cycle, self-loop, N-link chain-with-cycle, and resolver termination under `hop_budget`, asserting a single logical subscription per cycle.
- `state_distributed_admin::link_cycles()` static cycle enumerator exposed for tooling.
- Slice 6 — `state::resolveRemote(hop_budget)`: extends `resolveLink()` with authority-map awareness. When a link target is absent locally, consults the default shard's `state_delegation_manager` to classify the target as `resolved_remote` (active delegation), `lease_expired`, or `broken`. Cross-cluster link tests in `state_cross_cluster_link_test` cover: authority transfer mid-test, lease expiry invalidation, `sendMessage` over transparent links without naming a `cluster_id`, two-shard delegation propagation, and compile-time API checks.

Remaining for Phase 8:

- Bench: 1M-path tree with 10k links and a few cycles; assert resolver/subscription latency stays bounded.

- Add a `state_link` node kind alongside scalar/value/group nodes. A link records:
  - target `path` (absolute, may be the empty path meaning the tree root) — this is the **only** required field for the developer-facing API,
  - optional `tree_id` override (defaults to "the tree the resolver determines owns `path`"),
  - optional pinned `lease_id` for cross-cluster links so authority changes invalidate the link cleanly,
  - optional `mode` flags: `read_only`, `transparent` (clients see through it as the target's contents) vs `opaque` (clients see a link node and resolve it explicitly).
  - The owning `cluster_id` is **never** part of the developer API. It is derived at resolve time from the authority map (`state_authority_map::owner_for(path)`) and cached on the link record as an opportunistic hint; an authority change invalidates the cache and the next resolve recomputes it.
- Developer-facing API (cluster-agnostic):
  - `app.root()("scene.geometry").linkTo("data.world.geometry")` is sufficient. The developer never names `cluster_id`; the resolver locates the owning cluster via longest-prefix lookup on the authority map (the same machinery Phase 6 already added).
  - `app.root()("scene.geometry").sendMessage(payload, max_depth)` on a link node forwards through the link to `data.world.geometry`. The OOB bus consults `state_authority_map::owner_for(resolved_path)` to pick the transport peer; if the resolved path is local the message is delivered in-process, otherwise it rides the inproc/ipc/grpc transport to the owning cluster with the same TTL/dedup semantics. Callers never pass `cluster_id`.
  - Read, write, and subscribe APIs behave the same: every entry point takes a path (or a node handle) and resolves the owner internally. Cross-cluster routing is an implementation detail of `state_subscription_router` + `state_authority_map`, not part of the surface area.
- Resolution:
  - Reads through a transparent link follow the link in the resolver and return the target's value, with a depth budget enforced.
  - Subscriptions placed under a transparent link translate into subscriptions at the target path, and target-side mutations fan out to link-side subscribers via the existing subscription router.
  - Writes through a read-only link fail; writes through a writable link route through the authority map exactly like a write at the target path would.
- Loop support and detection:
  - Loops are *legal* by design — a link may eventually reach itself (including a link back to the tree root). The resolver must terminate.
  - Each resolution carries a hop budget (default 64) and a visited-set keyed by the *resolved* `(tree_id, path)` pair (cluster identity is recovered from the authority map and folded into the key only to disambiguate identically-named paths in independent trees). On revisit or budget exhaustion the resolver returns a structured `cycle_detected` result naming the cycle path, instead of recursing.
  - A static analyzer enumerates link nodes and reports any closed cycle through the link graph, exposed via `state_distributed_admin::link_cycles()` for tooling.
  - Subscription installation through a chain of links collapses repeats so an N-link cycle does not register N copies of the same callback.
- Cross-cluster links reuse the existing referral / lease machinery transparently: when the resolver determines the target path is owned by a remote cluster, it follows referrals and renews leases without the caller naming a `cluster_id`. Link freshness rides on lease renewal.
- Tests:
  - Local link resolution, transparent vs opaque mode, read-through and write-through, hop budget exhaustion.
  - Self-referential link, two-link cycle, link-to-root cycle: each must produce a deterministic `cycle_detected` outcome rather than infinite recursion or stack overflow.
  - Subscription fan-in across a transparent link including dedup with the cycle resolver.
  - Cross-cluster link with a real second cluster shard, with lease expiry invalidating the link.
  - Cluster-agnostic API: `linkTo("data.world.geometry")` and `sendMessage(...)` on the link node must succeed when authority for `data.world.geometry` is local, when it has been delegated to a second cluster mid-test, and after authority moves back — the test never names a `cluster_id`. A regression that requires the caller to know the owning cluster fails this test.
  - Bench: 1M-path tree with 10k links, including a few cycles, and assert resolver/subscription latency stays bounded.

### Phase 9: Network Analytics And Live Telemetry

Each node needs a live picture of the cluster's health and shape so operators (and the system itself, for routing decisions) can answer questions like "how big is this tree, how fast can I move a blob to peer X, how many nodes are in cluster Y" without per-query polling. Analytics piggyback on the OOB messaging bus and the existing `state_distributed_metrics` module so they cost no extra connections.

- Per-node sampler (`state_node_telemetry`):
  - Counters and rolling EWMAs for: bytes-sent / bytes-received per peer and per transport; mutation publish rate; message admit/dedup/drop rates; bounded-queue depths and overflow events; blob bytes uploaded/downloaded with throughput EWMA.
  - Latency histograms for: local enqueue, replica apply, remote delivery, callback dispatch, blob round-trip, message propagation. Use t-digest or HDR-style buckets so percentiles (p50/p90/p99) are stable.
  - Tree-shape stats: total path count, owned/local path count, total bytes-on-disk, bytes-in-memory, blob-store residency, link-node count, cycle count.
  - Cluster-shape stats: cluster member count, server-role count vs client-role count, delegated subtree count, peer reachability bitmap, last-ack age per peer.
- Live distribution:
  - A dedicated message family `__telemetry.<topic>` rides the OOB bus with `content_type=application/x-cvc-telemetry+cbor` (or json for debug). Subscribers can listen on a prefix and aggregate.
  - Each node periodically (configurable, default 1 s) publishes a *delta* report; on join or on demand it publishes a *full* report. Deltas are coalesced under the latest-value-only path policy so slow consumers always see the freshest snapshot.
  - Cluster-level rollups are computed by any subscribing node: a `state_telemetry_aggregator` consumes telemetry messages and exposes a derived view (cluster bandwidth, total nodes, total tree bytes, etc.). Multiple aggregators may run; results are eventually consistent.
  - Sensitive numbers (per-peer connection counts, queue depths) are tagged with the originating node id so aggregators can deduplicate when a node multi-homes.
- Routing feedback:
  - Slow-peer isolation and per-codec compression decisions read the same telemetry: a peer whose EWMA latency or queue depth crosses a threshold is automatically isolated, and a transfer to a high-RTT peer can flip on stronger compression.
- API surface:
  - `state_node_telemetry::snapshot()` returns a typed report (struct of structs of counters/EWMAs/histograms) for in-process consumers.
  - `state_distributed_admin::telemetry()` mirrors the local snapshot plus the latest aggregator view of every known peer.
  - `state_distributed_admin::to_text()` gains a `[telemetry]` section for human-readable dumps.
- Tests:
  - Counters increment monotonically under load; EWMAs converge.
  - Histogram percentiles stable under synthetic latency mix.
  - Telemetry messages round-trip through the bus with dedup and TTL respected; aggregator produces correct cluster totals.
  - Failure mode: an aggregator stops receiving from a peer for > 3× publish interval and marks the peer `stale`.
  - Bench: 100-node simulated cluster, 1 Hz telemetry, assert per-node CPU overhead is < 1% and aggregator memory is bounded.

## Testing Requirements

Every production unit added for this feature must have unit tests in the same change or an explicitly documented reason why it cannot be tested yet. The network implementation must not be considered complete until it has integration, stress, and performance coverage.

Required layers:

- Unit tests for mutation construction, journal replay, subscription routing, echo suppression, codecs, blob chunking, lease logic, route resolution, transport delivery, and message TTL.
- Multi-process integration tests once a cross-process transport exists. These should launch separate libcvc test helpers, communicate over IPC or gRPC localhost, verify bidirectional sync, verify callback delivery, and exercise reconnect/replay.
- Large-object integration tests using generated volumes/geometry and a temporary blob store.
- Many-node simulation tests with selective subscriptions, shard routing, slow-peer backpressure, and resync from snapshots — runnable purely in-process via `state_transport_inproc`.
- Large tree-network tests with many delegated prefixes and referral-cache invalidation.
- Stress tests gated by an explicit option or environment variable so normal CI remains fast, but nightly/opt-in runs can push path counts, mutation rates, and node counts.
- Performance tests that record scalar enqueue latency, remote delivery latency, callback latency, routing cost, blob throughput, snapshot time, reconnect recovery time, and message propagation latency. These tests should start as reporting benchmarks, then gain thresholds once baseline data is stable.
- Failure tests for duplicate mutations, out-of-order delivery, partial blob transfers, stale leases, split authority, slow peers, and transport interruption.

## First Networking Milestone

Build a prototype that synchronizes string values and metadata between two local processes over gRPC, with the adapter/journal architecture already in place and the hot path shaped for low latency. Do not start with large objects or delegation in code, but do include subscription routing, latency measurement, and a basic `state_transport_inproc` so the same shard logic can run in unit tests and over the network unchanged.

The first merged networking milestone should include:

- new optional CMake flag and generated protobuf build using libcvc-deps v1.1.0 gRPC/protobuf;
- `state_transport` interface and `state_transport_inproc` concrete implementation;
- `distributed_state_session::join()` for localhost peer sync over gRPC;
- scalar value and metadata replication;
- persistent bidirectional streams with no per-mutation connection setup;
- path-prefix subscription routing, even if the first test has only two processes;
- initial snapshot on join;
- journal replay after reconnect;
- basic latency benchmark output for local enqueue and remote callback delivery;
- tests that prove existing local callbacks still fire on remote updates.
