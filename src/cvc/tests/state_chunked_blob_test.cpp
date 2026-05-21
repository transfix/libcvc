/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <cvc/state_chunked_blob.h>

#include <cstdint>
#include <random>
#include <string>
#include <vector>

#include <gtest/gtest.h>

namespace {

std::vector<unsigned char> make_payload(std::size_t n, std::uint32_t seed = 1) {
  std::mt19937 rng(seed);
  std::uniform_int_distribution<int> d(0, 255);
  std::vector<unsigned char> v(n);
  for (auto &b : v)
    b = static_cast<unsigned char>(d(rng));
  return v;
}

} // namespace

TEST(StateChunkedBlobTest, ManifestSerializeRoundTrip) {
  cvc::state_chunk_manifest m;
  m.version = 1;
  m.chunk_size = 4;
  m.total_size = 9;
  m.codec = "raw";
  m.content_digest = "abc";
  m.chunks = {"d1", "d2", "d3"};
  m.chunk_bytes = {4, 4, 1};

  auto bytes = m.serialize();
  cvc::state_chunk_manifest parsed;
  ASSERT_TRUE(cvc::state_chunk_manifest::parse(bytes, parsed));
  EXPECT_EQ(parsed.chunk_size, 4u);
  EXPECT_EQ(parsed.total_size, 9u);
  EXPECT_EQ(parsed.codec, "raw");
  EXPECT_EQ(parsed.content_digest, "abc");
  ASSERT_EQ(parsed.chunks.size(), 3u);
  EXPECT_EQ(parsed.chunks[2], "d3");
  ASSERT_EQ(parsed.chunk_bytes.size(), 3u);
  EXPECT_EQ(parsed.chunk_bytes[2], 1u);
}

TEST(StateChunkedBlobTest, ManifestRejectsBadMagic) {
  std::vector<unsigned char> bytes = {'X', 'Y', 'Z', 'Q', 1, 0, 0, 0};
  cvc::state_chunk_manifest out;
  EXPECT_FALSE(cvc::state_chunk_manifest::parse(bytes, out));
}

TEST(StateChunkedBlobTest, WriteThenReadFullPayload) {
  cvc::memory_state_blob_store store;
  cvc::state_chunked_blob_writer w(store, 256);
  cvc::state_chunked_blob_reader r(store);

  auto payload = make_payload(1000); // 4 chunks: 256+256+256+232
  auto ref = w.put(payload, "raw");
  EXPECT_FALSE(ref.digest.empty());
  EXPECT_EQ(w.total_chunks_written(), 4u);
  EXPECT_EQ(w.total_bytes_written(), 1000u);
  EXPECT_EQ(w.total_manifests_written(), 1u);

  cvc::state_chunk_manifest m;
  ASSERT_TRUE(r.load_manifest(ref.digest, m));
  EXPECT_EQ(m.chunks.size(), 4u);
  EXPECT_EQ(m.total_size, 1000u);
  EXPECT_EQ(m.chunk_bytes.back(), 232u);

  std::vector<unsigned char> out;
  ASSERT_TRUE(r.get(ref.digest, out));
  EXPECT_EQ(out, payload);
  EXPECT_TRUE(r.missing_chunks(m).empty());
}

TEST(StateChunkedBlobTest, ChunkDedupAcrossPayloads) {
  cvc::memory_state_blob_store store;
  cvc::state_chunked_blob_writer w(store, 64);

  auto p1 = make_payload(256, 7);
  auto p2 = p1; // identical: every chunk should dedup
  w.put(p1);
  const auto chunks_first = w.total_chunks_written();
  const auto dedup_first = w.total_chunks_dedup();
  w.put(p2);
  EXPECT_EQ(w.total_chunks_written(), chunks_first); // no new bytes
  EXPECT_EQ(w.total_chunks_dedup(), dedup_first + 4u);
}

TEST(StateChunkedBlobTest, ResumeReportsMissingChunks) {
  cvc::memory_state_blob_store source;
  cvc::memory_state_blob_store dest;
  cvc::state_chunked_blob_writer ws(source, 64);
  auto payload = make_payload(200, 3); // 4 chunks
  auto ref = ws.put(payload, "raw");

  // Simulate a partial replication: copy the manifest plus chunks 0
  // and 2 to the destination.
  std::vector<unsigned char> manifest_bytes;
  ASSERT_TRUE(source.get(ref.digest, manifest_bytes));
  dest.put(manifest_bytes, "raw");
  cvc::state_chunk_manifest m;
  ASSERT_TRUE(cvc::state_chunk_manifest::parse(manifest_bytes, m));
  ASSERT_EQ(m.chunks.size(), 4u);
  for (std::size_t i : {std::size_t{0}, std::size_t{2}}) {
    std::vector<unsigned char> c;
    ASSERT_TRUE(source.get(m.chunks[i], c));
    dest.put(c);
  }

  cvc::state_chunked_blob_reader rd(dest);
  auto missing = rd.missing_chunks(ref.digest);
  ASSERT_EQ(missing.size(), 2u);
  EXPECT_EQ(missing[0], m.chunks[1]);
  EXPECT_EQ(missing[1], m.chunks[3]);

  // Reassembly fails until the gaps are filled.
  std::vector<unsigned char> out;
  EXPECT_FALSE(rd.get(ref.digest, out));

  // Fill the missing chunks and retry.
  for (const auto &d : missing) {
    std::vector<unsigned char> c;
    ASSERT_TRUE(source.get(d, c));
    dest.put(c);
  }
  EXPECT_TRUE(rd.missing_chunks(ref.digest).empty());
  ASSERT_TRUE(rd.get(ref.digest, out));
  EXPECT_EQ(out, payload);
}

TEST(StateChunkedBlobTest, EmptyPayloadProducesValidManifest) {
  cvc::memory_state_blob_store store;
  cvc::state_chunked_blob_writer w(store, 1024);
  cvc::state_chunked_blob_reader r(store);

  std::vector<unsigned char> empty;
  auto ref = w.put(empty);
  cvc::state_chunk_manifest m;
  ASSERT_TRUE(r.load_manifest(ref.digest, m));
  EXPECT_EQ(m.chunks.size(), 0u);
  EXPECT_EQ(m.total_size, 0u);

  std::vector<unsigned char> out;
  ASSERT_TRUE(r.get(ref.digest, out));
  EXPECT_TRUE(out.empty());
}

TEST(StateChunkedBlobTest, ContentDigestDetectsCorruption) {
  cvc::memory_state_blob_store store;
  cvc::state_chunked_blob_writer w(store, 64);
  auto payload = make_payload(200, 5);
  auto ref = w.put(payload);

  cvc::state_chunk_manifest m;
  ASSERT_TRUE(cvc::state_chunked_blob_reader(store).load_manifest(ref.digest,
                                                                  m));
  // Corrupt the manifest's recorded content digest, re-parse via a
  // hand-built manifest and verify get() rejects.
  m.content_digest = std::string(64, 'f');
  std::vector<unsigned char> out;
  cvc::state_chunked_blob_reader r(store);
  EXPECT_FALSE(r.get(m, out));
}

TEST(StateChunkedBlobTest, ChunkSizeLargerThanPayloadEmitsOneChunk) {
  cvc::memory_state_blob_store store;
  cvc::state_chunked_blob_writer w(store, 1024);
  auto payload = make_payload(50, 9);
  auto ref = w.put(payload);

  cvc::state_chunk_manifest m;
  ASSERT_TRUE(cvc::state_chunked_blob_reader(store).load_manifest(ref.digest,
                                                                  m));
  ASSERT_EQ(m.chunks.size(), 1u);
  EXPECT_EQ(m.chunk_bytes[0], 50u);

  std::vector<unsigned char> out;
  ASSERT_TRUE(cvc::state_chunked_blob_reader(store).get(ref.digest, out));
  EXPECT_EQ(out, payload);
}
