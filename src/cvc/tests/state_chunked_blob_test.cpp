/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <cstdint>
#include <cvc/core/state_chunked_blob.h>
#include <gtest/gtest.h>
#include <mutex>
#include <random>
#include <string>
#include <thread>
#include <vector>

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
  ASSERT_TRUE(cvc::state_chunked_blob_reader(store).load_manifest(ref.digest, m));
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
  ASSERT_TRUE(cvc::state_chunked_blob_reader(store).load_manifest(ref.digest, m));
  ASSERT_EQ(m.chunks.size(), 1u);
  EXPECT_EQ(m.chunk_bytes[0], 50u);

  std::vector<unsigned char> out;
  ASSERT_TRUE(cvc::state_chunked_blob_reader(store).get(ref.digest, out));
  EXPECT_EQ(out, payload);
}

TEST(StateChunkedBlobTest, ManifestPreservesCodecField) {
  cvc::memory_state_blob_store store;
  cvc::state_chunked_blob_writer w(store, 64);
  auto payload = make_payload(100, 11);
  auto ref = w.put(payload, "zstd-3");
  EXPECT_EQ(ref.codec, "zstd-3");

  cvc::state_chunk_manifest m;
  ASSERT_TRUE(cvc::state_chunked_blob_reader(store).load_manifest(ref.digest, m));
  EXPECT_EQ(m.codec, "zstd-3");
}

TEST(StateChunkedBlobTest, ManifestRejectsTruncatedBytes) {
  cvc::state_chunk_manifest m;
  m.chunk_size = 4;
  m.total_size = 8;
  m.chunks = {"d1", "d2"};
  m.chunk_bytes = {4, 4};
  auto bytes = m.serialize();
  for (std::size_t cut = 4; cut < bytes.size(); ++cut) {
    std::vector<unsigned char> partial(bytes.begin(), bytes.begin() + cut);
    cvc::state_chunk_manifest out;
    EXPECT_FALSE(cvc::state_chunk_manifest::parse(partial, out)) << "cut=" << cut;
  }
}

TEST(StateChunkedBlobTest, ManifestRejectsWrongVersion) {
  // Build a manifest with version 2 manually.
  std::vector<unsigned char> bytes;
  bytes.insert(bytes.end(), {'C', 'V', 'C', 'M'});
  // version=2 LE
  bytes.insert(bytes.end(), {2, 0, 0, 0});
  // chunk_size=0
  bytes.insert(bytes.end(), {0, 0, 0, 0});
  // total_size=0 (8 bytes)
  bytes.insert(bytes.end(), 8, 0);
  // codec_len=0, content_digest_len=0, count=0
  bytes.insert(bytes.end(), 12, 0);

  cvc::state_chunk_manifest out;
  EXPECT_FALSE(cvc::state_chunk_manifest::parse(bytes, out));
}

TEST(StateChunkedBlobTest, ZeroChunkSizeNormalizedToOne) {
  cvc::memory_state_blob_store store;
  cvc::state_chunked_blob_writer w(store, 0);
  EXPECT_EQ(w.chunk_size(), 1u);
  auto payload = make_payload(3, 13);
  auto ref = w.put(payload);
  cvc::state_chunk_manifest m;
  ASSERT_TRUE(cvc::state_chunked_blob_reader(store).load_manifest(ref.digest, m));
  EXPECT_EQ(m.chunks.size(), 3u);
  EXPECT_EQ(m.chunk_size, 1u);
}

TEST(StateChunkedBlobTest, ReaderHandlesMissingManifest) {
  cvc::memory_state_blob_store store;
  cvc::state_chunked_blob_reader r(store);
  cvc::state_chunk_manifest m;
  EXPECT_FALSE(r.load_manifest("deadbeef", m));
  EXPECT_TRUE(r.missing_chunks("deadbeef").empty());
  std::vector<unsigned char> out;
  EXPECT_FALSE(r.get("deadbeef", out));
}

TEST(StateChunkedBlobTest, ReaderHandlesMalformedManifestBlob) {
  // Inject random bytes under a digest as if it were a manifest.
  cvc::memory_state_blob_store store;
  std::vector<unsigned char> garbage = {'n', 'o', 'p', 'e', 1, 2, 3, 4};
  auto ref = store.put(garbage);
  cvc::state_chunked_blob_reader r(store);
  cvc::state_chunk_manifest m;
  EXPECT_FALSE(r.load_manifest(ref.digest, m));
  std::vector<unsigned char> out;
  EXPECT_FALSE(r.get(ref.digest, out));
}

TEST(StateChunkedBlobTest, RewriteSamePayloadDedupsAllChunks) {
  cvc::memory_state_blob_store store;
  cvc::state_chunked_blob_writer w(store, 64);
  auto payload = make_payload(192, 17); // 3 chunks
  w.put(payload);
  EXPECT_EQ(w.total_chunks_written(), 3u);
  EXPECT_EQ(w.total_chunks_dedup(), 0u);

  // Same payload again: every chunk dedups, manifest dedups too.
  const auto bytes_before = w.total_bytes_written();
  const auto store_size_before = store.size();
  w.put(payload);
  EXPECT_EQ(w.total_chunks_written(), 3u); // unchanged
  EXPECT_EQ(w.total_chunks_dedup(), 3u);
  EXPECT_EQ(w.total_bytes_written(), bytes_before);
  EXPECT_EQ(store.size(), store_size_before); // manifest already there
}

TEST(StateChunkedBlobTest, OverlappingPayloadsShareChunks) {
  // Two payloads that share a 64-byte prefix should share at least
  // one chunk in the store.
  cvc::memory_state_blob_store store;
  cvc::state_chunked_blob_writer w(store, 64);

  std::vector<unsigned char> base = make_payload(64, 19);
  std::vector<unsigned char> p1 = base;
  std::vector<unsigned char> p2 = base;
  // Append divergent suffixes.
  for (int i = 0; i < 64; ++i)
    p1.push_back(static_cast<unsigned char>(i));
  for (int i = 0; i < 64; ++i)
    p2.push_back(static_cast<unsigned char>(0xff - i));

  w.put(p1);
  const auto written_after_first = w.total_chunks_written();
  w.put(p2);
  // Second put: shared first chunk dedups, suffix is new.
  EXPECT_EQ(w.total_chunks_dedup(), 1u);
  EXPECT_EQ(w.total_chunks_written(), written_after_first + 1u);
}

TEST(StateChunkedBlobTest, ManifestRoundTripWithEmptyCodec) {
  cvc::state_chunk_manifest m;
  m.codec = "";
  m.content_digest = "";
  auto bytes = m.serialize();
  cvc::state_chunk_manifest out;
  ASSERT_TRUE(cvc::state_chunk_manifest::parse(bytes, out));
  EXPECT_TRUE(out.codec.empty());
  EXPECT_TRUE(out.content_digest.empty());
}

TEST(StateChunkedBlobTest, ManifestRoundTripWithNoChunks) {
  cvc::state_chunk_manifest m;
  m.chunk_size = 1024;
  m.total_size = 0;
  auto bytes = m.serialize();
  cvc::state_chunk_manifest out;
  ASSERT_TRUE(cvc::state_chunk_manifest::parse(bytes, out));
  EXPECT_EQ(out.chunks.size(), 0u);
  EXPECT_EQ(out.chunk_size, 1024u);
}

TEST(StateChunkedBlobTest, GetReturnsFalseOnMissingChunkAfterEvict) {
  cvc::memory_state_blob_store store;
  cvc::state_chunked_blob_writer w(store, 64);
  auto payload = make_payload(192, 23);
  auto ref = w.put(payload);
  cvc::state_chunk_manifest m;
  ASSERT_TRUE(cvc::state_chunked_blob_reader(store).load_manifest(ref.digest, m));
  ASSERT_EQ(m.chunks.size(), 3u);

  // Erase the middle chunk to simulate a partial loss.
  EXPECT_TRUE(store.erase(m.chunks[1]));

  cvc::state_chunked_blob_reader r(store);
  std::vector<unsigned char> out;
  EXPECT_FALSE(r.get(ref.digest, out));
  auto missing = r.missing_chunks(ref.digest);
  ASSERT_EQ(missing.size(), 1u);
  EXPECT_EQ(missing[0], m.chunks[1]);
}

TEST(StateChunkedBlobTest, ConcurrentWritersAreSafe) {
  cvc::memory_state_blob_store store;
  cvc::state_chunked_blob_writer w(store, 128);

  constexpr int kThreads = 4;
  constexpr int kPerThread = 16;
  std::vector<std::thread> ts;
  std::vector<std::string> all_digests(kThreads * kPerThread);
  std::mutex out_mutex;
  for (int t = 0; t < kThreads; ++t) {
    ts.emplace_back([&, t]() {
      for (int i = 0; i < kPerThread; ++i) {
        auto p = make_payload(300, static_cast<std::uint32_t>(t * 100 + i));
        auto ref = w.put(p);
        std::lock_guard<std::mutex> lk(out_mutex);
        all_digests[t * kPerThread + i] = ref.digest;
      }
    });
  }
  for (auto &th : ts)
    th.join();

  cvc::state_chunked_blob_reader r(store);
  for (const auto &d : all_digests) {
    ASSERT_FALSE(d.empty());
    cvc::state_chunk_manifest m;
    EXPECT_TRUE(r.load_manifest(d, m));
    std::vector<unsigned char> out;
    EXPECT_TRUE(r.get(d, out));
    EXPECT_EQ(out.size(), 300u);
  }
}

// ----------------
// Compression integration tests (Phase 6 bullet 3)
// ----------------

#include <cvc/core/state_compression_registry.h>

TEST(StateChunkedBlobTest, CompressionShrinksRunfulPayload) {
  cvc::memory_state_blob_store store;
  cvc::state_compression_registry reg;
  cvc::state_chunked_blob_writer w(store, 512, &reg);

  std::vector<unsigned char> payload(4096, 0xAA);
  auto ref = w.put(payload, "rle");

  cvc::state_chunked_blob_reader r(store, &reg);
  cvc::state_chunk_manifest m;
  ASSERT_TRUE(r.load_manifest(ref.digest, m));
  EXPECT_EQ(m.codec, "rle");
  EXPECT_EQ(m.total_size, 4096u);

  // 8 chunks of 512 bytes; each compresses 512 0xAA bytes into
  // ceil(512/255)=3 pairs * 2 = 6 stored bytes per chunk.
  ASSERT_EQ(m.chunks.size(), 8u);
  ASSERT_EQ(m.chunk_bytes.size(), 8u);
  std::uint64_t total_stored = 0;
  for (auto cb : m.chunk_bytes)
    total_stored += cb;
  EXPECT_LT(total_stored, payload.size());
  EXPECT_EQ(total_stored, 8u * 6u);

  std::vector<unsigned char> out;
  ASSERT_TRUE(r.get(ref.digest, out));
  EXPECT_EQ(out, payload);
}

TEST(StateChunkedBlobTest, CompressionRoundTripsRandomPayload) {
  cvc::memory_state_blob_store store;
  cvc::state_compression_registry reg;
  cvc::state_chunked_blob_writer w(store, 256, &reg);

  auto payload = make_payload(2000, 42);
  auto ref = w.put(payload, "rle");

  cvc::state_chunked_blob_reader r(store, &reg);
  std::vector<unsigned char> out;
  ASSERT_TRUE(r.get(ref.digest, out));
  EXPECT_EQ(out, payload);
}

TEST(StateChunkedBlobTest, CompressionEmptyCodecBackCompat) {
  cvc::memory_state_blob_store store;
  cvc::state_chunked_blob_writer w(store, 128); // no registry
  auto payload = make_payload(500, 7);
  auto ref = w.put(payload, ""); // empty codec = identity

  cvc::state_chunked_blob_reader r(store);
  std::vector<unsigned char> out;
  ASSERT_TRUE(r.get(ref.digest, out));
  EXPECT_EQ(out, payload);
}

TEST(StateChunkedBlobTest, CompressionRawCodecIsIdentity) {
  cvc::memory_state_blob_store store;
  cvc::state_compression_registry reg;
  cvc::state_chunked_blob_writer w(store, 128, &reg);
  auto payload = make_payload(500, 11);
  auto ref = w.put(payload, "raw");

  cvc::state_chunked_blob_reader r(store, &reg);
  cvc::state_chunk_manifest m;
  ASSERT_TRUE(r.load_manifest(ref.digest, m));
  EXPECT_EQ(m.codec, "raw");
  std::uint64_t total_stored = 0;
  for (auto cb : m.chunk_bytes)
    total_stored += cb;
  EXPECT_EQ(total_stored, payload.size());
  std::vector<unsigned char> out;
  ASSERT_TRUE(r.get(ref.digest, out));
  EXPECT_EQ(out, payload);
}

TEST(StateChunkedBlobTest, CompressionUnknownCodecAtWriteThrows) {
  cvc::memory_state_blob_store store;
  cvc::state_compression_registry reg;
  cvc::state_chunked_blob_writer w(store, 128, &reg);
  auto payload = make_payload(100, 1);
  EXPECT_THROW(w.put(payload, "no_such_codec"), std::runtime_error);
}

TEST(StateChunkedBlobTest, CompressionReaderWithoutRegistryFails) {
  cvc::memory_state_blob_store store;
  cvc::state_compression_registry reg;
  cvc::state_chunked_blob_writer w(store, 128, &reg);
  std::vector<unsigned char> payload(256, 0xFF);
  auto ref = w.put(payload, "rle");

  // Reader has no registry and codec is non-trivial -> get must fail.
  cvc::state_chunked_blob_reader r(store);
  std::vector<unsigned char> out;
  EXPECT_FALSE(r.get(ref.digest, out));
  EXPECT_TRUE(out.empty());
}

TEST(StateChunkedBlobTest, CompressionReaderUnknownCodecFails) {
  cvc::memory_state_blob_store store;
  cvc::state_compression_registry reg_w;
  cvc::state_chunked_blob_writer w(store, 128, &reg_w);
  std::vector<unsigned char> payload(256, 0xFF);
  auto ref = w.put(payload, "rle");

  // Reader has a registry but rle has been stripped from it.
  cvc::state_compression_registry reg_r;
  // shadow the rle codec by re-registering only raw
  // (we cannot remove, so use a separate empty-ish registry)
  struct empty_reg : public cvc::state_compression_registry {};
  // Simpler: build an alternative registry minus rle by leveraging
  // the public API: construct a fresh one and overwrite rle with a
  // codec that always fails decode.
  struct broken : public cvc::state_compression_codec {
    std::string id() const override { return "rle"; }
    std::vector<unsigned char> encode(const std::vector<unsigned char> &in) const override {
      return in;
    }
    bool decode(const std::vector<unsigned char> &,
                std::vector<unsigned char> &out) const override {
      out.clear();
      return false;
    }
  };
  reg_r.register_codec(std::make_shared<broken>());

  cvc::state_chunked_blob_reader r(store, &reg_r);
  std::vector<unsigned char> out;
  EXPECT_FALSE(r.get(ref.digest, out));
  EXPECT_TRUE(out.empty());
}

TEST(StateChunkedBlobTest, CompressionDedupAcrossPayloadsAtCompressedLevel) {
  cvc::memory_state_blob_store store;
  cvc::state_compression_registry reg;
  cvc::state_chunked_blob_writer w(store, 256, &reg);

  // Two different payloads that contain the same all-0xCC chunk.
  std::vector<unsigned char> p1(256, 0xCC);
  p1.insert(p1.end(), 256, 0x01);
  std::vector<unsigned char> p2(256, 0xCC);
  p2.insert(p2.end(), 256, 0x02);

  auto r1 = w.put(p1, "rle");
  auto r2 = w.put(p2, "rle");
  EXPECT_NE(r1.digest, r2.digest);

  // Both manifests should reference the same digest for the
  // first chunk (since the compressed bytes for 256x0xCC are
  // identical).
  cvc::state_chunked_blob_reader rd(store, &reg);
  cvc::state_chunk_manifest m1, m2;
  ASSERT_TRUE(rd.load_manifest(r1.digest, m1));
  ASSERT_TRUE(rd.load_manifest(r2.digest, m2));
  ASSERT_GE(m1.chunks.size(), 1u);
  ASSERT_GE(m2.chunks.size(), 1u);
  EXPECT_EQ(m1.chunks[0], m2.chunks[0]);
}
