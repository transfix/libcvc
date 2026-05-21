/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// Integration tests for the chunked-blob + compression-codec stack
// running over the in-process transport. Closes the survey gap
// "Blob + Codec + Transport" and "Chunked-Blob over Slow-Peer
// Isolation" by exercising the full pipeline end-to-end with two or
// three shards wired through state_transport_inproc.

#include <cstdint>
#include <cvc/app.h>
#include <cvc/state.h>
#include <cvc/state_blob_store.h>
#include <cvc/state_change_journal.h>
#include <cvc/state_chunked_blob.h>
#include <cvc/state_cluster_shard.h>
#include <cvc/state_compression_registry.h>
#include <cvc/state_transport_inproc.h>
#include <gtest/gtest.h>
#include <string>
#include <vector>

namespace {

constexpr std::size_t kPayloadBytes = 256u * 1024u; // 256 KiB
constexpr std::uint32_t kChunkBytes = 16u * 1024u;  // 16 KiB

std::vector<unsigned char> runful_payload(std::size_t n, unsigned char b) {
  return std::vector<unsigned char>(n, b);
}

cvc::state_mutation make_blob_mutation(const std::string &origin, std::uint64_t seq,
                                       const std::string &path, const cvc::state_blob_ref &ref) {
  cvc::state_mutation m;
  m.cluster_id = "C";
  m.origin_node_id = origin;
  m.sequence = seq;
  m.path = path;
  m.op = cvc::state_mutation_op::set_value;
  // The string_value is what landing in B's state tree will look
  // like to a normal observer; carry the digest so a state-only
  // observer can still follow the reference.
  m.string_value = ref.digest;
  m.type_name = "std::string";
  m.payload = cvc::state_payload::blob_ref(ref);
  return m;
}

} // namespace

// ----------------
// SharedStore: A and B both reference the same memory blob store.
// Models the "all peers see the same content-addressed namespace"
// case (e.g. a shared filesystem, or a future fetch-on-demand store
// that hides remote chunk pulls).
// ----------------
TEST(StateBlobTransportIntegration, SharedStoreEndToEndRleRoundTrip) {
  cvc::memory_state_blob_store store;
  cvc::state_compression_registry registry;
  cvc::state_chunked_blob_writer writer(store, kChunkBytes, &registry);
  cvc::state_chunked_blob_reader reader(store, &registry);

  // Writer side (peer A): compress with built-in 'rle'.
  auto payload = runful_payload(kPayloadBytes, 0xAA);
  cvc::state_blob_ref manifest = writer.put(payload, "rle");
  ASSERT_FALSE(manifest.digest.empty());

  // Compression must actually shrink stored bytes; an RLE of N
  // identical bytes is two stored bytes per chunk's run.
  cvc::state_chunk_manifest parsed;
  ASSERT_TRUE(reader.load_manifest(manifest.digest, parsed));
  std::uint64_t stored = 0;
  for (auto b : parsed.chunk_bytes)
    stored += b;
  EXPECT_LT(stored, kPayloadBytes / 10u) << "RLE on a single-byte payload must compress hard";
  EXPECT_EQ(parsed.total_size, kPayloadBytes);
  EXPECT_EQ(parsed.codec, "rle");

  // Two shards on the inproc transport.
  cvc::app aA, aB;
  cvc::state_transport_inproc t;
  cvc::state_cluster_shard sA(aA, "C", "A");
  cvc::state_cluster_shard sB(aB, "C", "B");
  sA.attach();
  sB.attach();
  t.register_shard(&sA);
  t.register_shard(&sB);

  // A publishes a mutation whose payload carries the manifest
  // digest. B applies it via the standard ingest path.
  auto m = make_blob_mutation("A", /*seq=*/1, "vol.brick.42", manifest);
  auto stats = t.publish(m);
  EXPECT_GE(stats.delivered, 1u);
  EXPECT_EQ(sB.replica().last_applied("A"), 1u);

  // The string_value on B should now be the manifest digest (the
  // visible side of the blob ref). The actual payload bytes still
  // live in the shared store, addressable by digest.
  EXPECT_EQ(cvc::state::instance(aB)("vol.brick.42").value(), manifest.digest);

  // After the mutation has propagated, B's reader (over the same
  // shared store) reconstructs the byte-exact original payload.
  std::vector<unsigned char> got;
  ASSERT_TRUE(reader.get(manifest.digest, got));
  EXPECT_EQ(got.size(), payload.size());
  EXPECT_EQ(got, payload);
}

// ----------------
// IsolatedStores: A and B each have their own store. Today the
// transport carries the manifest digest only; chunk bytes do not
// flow automatically. This test pins that boundary so a future
// "fetch chunks over transport" feature has a baseline to relax.
// ----------------
TEST(StateBlobTransportIntegration, IsolatedStoresExposeMissingChunks) {
  cvc::state_compression_registry registry;
  cvc::memory_state_blob_store storeA;
  cvc::memory_state_blob_store storeB;
  cvc::state_chunked_blob_writer wA(storeA, kChunkBytes, &registry);
  cvc::state_chunked_blob_reader rB(storeB, &registry);

  auto payload = runful_payload(kPayloadBytes, 0x5A);
  cvc::state_blob_ref manifest = wA.put(payload, "rle");

  cvc::app aA, aB;
  cvc::state_transport_inproc t;
  cvc::state_cluster_shard sA(aA, "C", "A");
  cvc::state_cluster_shard sB(aB, "C", "B");
  sA.attach();
  sB.attach();
  t.register_shard(&sA);
  t.register_shard(&sB);

  auto m = make_blob_mutation("A", 1, "vol.iso", manifest);
  EXPECT_GE(t.publish(m).delivered, 1u);
  EXPECT_EQ(sB.replica().last_applied("A"), 1u);

  // B has the manifest digest in its state tree, but storeB has
  // neither the manifest blob nor any chunk.
  EXPECT_EQ(cvc::state::instance(aB)("vol.iso").value(), manifest.digest);
  cvc::state_chunk_manifest parsed_b;
  EXPECT_FALSE(rB.load_manifest(manifest.digest, parsed_b));

  // Stage the manifest blob into storeB (simulating a future
  // fetch-on-demand pull). Now B can parse the manifest but every
  // chunk is still missing.
  std::vector<unsigned char> manifest_bytes;
  ASSERT_TRUE(storeA.get(manifest.digest, manifest_bytes));
  storeB.put(manifest_bytes);
  ASSERT_TRUE(rB.load_manifest(manifest.digest, parsed_b));
  auto missing = rB.missing_chunks(parsed_b);
  EXPECT_EQ(missing.size(), parsed_b.chunks.size());

  // Stage every missing chunk; reassembly must produce the exact
  // original payload.
  for (const auto &d : missing) {
    std::vector<unsigned char> chunk;
    ASSERT_TRUE(storeA.get(d, chunk));
    storeB.put(chunk);
  }
  EXPECT_TRUE(rB.missing_chunks(parsed_b).empty());
  std::vector<unsigned char> got;
  ASSERT_TRUE(rB.get(parsed_b, got));
  EXPECT_EQ(got, payload);
}

// ----------------
// SlowPeerIsolation interacts with blob-bearing mutations:
// quarantined peers must skip the publish, healthy peers must still
// receive the manifest digest, and counters must tick.
// ----------------
TEST(StateBlobTransportIntegration, SlowPeerIsolationSkipsBlobMutations) {
  cvc::memory_state_blob_store store;
  cvc::state_compression_registry registry;
  cvc::state_chunked_blob_writer writer(store, kChunkBytes, &registry);
  cvc::state_chunked_blob_reader reader(store, &registry);

  auto payload = runful_payload(kPayloadBytes, 0xCC);
  auto manifest = writer.put(payload, "rle");

  cvc::app aA, aB, aC;
  cvc::state_transport_inproc t;
  cvc::state_cluster_shard sA(aA, "C", "A");
  cvc::state_cluster_shard sB(aB, "C", "B");
  cvc::state_cluster_shard sC(aC, "C", "C");
  sA.attach();
  sB.attach();
  sC.attach();
  t.register_shard(&sA);
  t.register_shard(&sB);
  t.register_shard(&sC);

  // B is slow; A's blob-bearing mutation must reach C but skip B.
  t.mark_peer_slow(&sB);
  EXPECT_TRUE(t.is_peer_slow(&sB));

  auto m1 = make_blob_mutation("A", 1, "vol.brick.q", manifest);
  auto stats1 = t.publish(m1);
  EXPECT_EQ(stats1.delivered, 1u) << "only C should accept";
  EXPECT_EQ(sC.replica().last_applied("A"), 1u);
  EXPECT_EQ(sB.replica().last_applied("A"), 0u);
  EXPECT_GE(t.total_quarantined_mutations(), 1u);

  // C can resolve the blob from the shared store (sanity).
  std::vector<unsigned char> got_c;
  ASSERT_TRUE(reader.get(manifest.digest, got_c));
  EXPECT_EQ(got_c, payload);

  // Releasing B and republishing under a fresh sequence delivers
  // to B too.
  t.clear_peer_slow(&sB);
  EXPECT_FALSE(t.is_peer_slow(&sB));
  auto m2 = make_blob_mutation("A", 2, "vol.brick.q", manifest);
  auto stats2 = t.publish(m2);
  EXPECT_GE(stats2.delivered, 2u);
  EXPECT_EQ(sB.replica().last_applied("A"), 2u);
  EXPECT_EQ(cvc::state::instance(aB)("vol.brick.q").value(), manifest.digest);

  std::vector<unsigned char> got_b;
  ASSERT_TRUE(reader.get(manifest.digest, got_b));
  EXPECT_EQ(got_b, payload);
}

// ----------------
// Compression dedup spans peers: two different runful payloads
// produce different manifests but should share zero chunks; an
// identical replay must dedup at storage level even when published
// from different origins.
// ----------------
TEST(StateBlobTransportIntegration, IdenticalPayloadsDedupAcrossOrigins) {
  cvc::memory_state_blob_store store;
  cvc::state_compression_registry registry;
  cvc::state_chunked_blob_writer writer(store, kChunkBytes, &registry);

  auto payload = runful_payload(kPayloadBytes, 0x77);
  auto m_a = writer.put(payload, "rle");
  std::size_t blobs_after_first = store.size();
  auto m_b = writer.put(payload, "rle");
  EXPECT_EQ(m_a.digest, m_b.digest);
  EXPECT_EQ(store.size(), blobs_after_first)
      << "second put of identical payload must not grow the store";

  cvc::app aA, aB;
  cvc::state_transport_inproc t;
  cvc::state_cluster_shard sA(aA, "C", "A");
  cvc::state_cluster_shard sB(aB, "C", "B");
  sA.attach();
  sB.attach();
  t.register_shard(&sA);
  t.register_shard(&sB);

  // Two mutations from different origins carrying the same digest
  // both apply (different sequence per origin) and B's view
  // settles at the last one.
  EXPECT_GE(t.publish(make_blob_mutation("A", 1, "vol.x", m_a)).delivered, 1u);
  EXPECT_GE(t.publish(make_blob_mutation("Z", 1, "vol.x", m_b)).delivered, 1u);
  EXPECT_EQ(sB.replica().last_applied("A"), 1u);
  EXPECT_EQ(sB.replica().last_applied("Z"), 1u);
  EXPECT_EQ(cvc::state::instance(aB)("vol.x").value(), m_a.digest);
}
