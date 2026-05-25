/*
  Copyright 2026 The University of Texas at Austin
  Tests for the lazy data hydrator.
*/

#include <chrono>
#include <cvc/core/app.h>
#include <cvc/core/state.h>
#include <cvc/core/state_blob_store.h>
#include <cvc/core/state_chunked_blob.h>
#include <cvc/core/state_codec_registry.h>
#include <cvc/core/state_compression_registry.h>
#include <cvc/core/state_data_hydrator.h>
#include <cvc/core/state_transport_inproc.h>
#include <gtest/gtest.h>
#include <string>

using namespace cvc;

class DataHydratorTest : public ::testing::Test {
protected:
  void SetUp() override {
    // Register a trivial "test_string" codec.
    state_codec_registry::encode_func enc = [](const boost::any &v) -> std::vector<unsigned char> {
      auto s = boost::any_cast<std::string>(v);
      return std::vector<unsigned char>(s.begin(), s.end());
    };
    state_codec_registry::decode_func dec = [](const std::vector<unsigned char> &b) -> boost::any {
      return std::string(b.begin(), b.end());
    };
    codecs.register_codec("test_string", enc, dec);
  }

  memory_state_blob_store store;
  state_codec_registry codecs;
  state_compression_registry compression;
};

static state_blob_ref store_chunked(memory_state_blob_store &store, const std::string &payload) {
  state_chunked_blob_writer writer(store, 64); // small chunks for testing
  std::vector<unsigned char> bytes(payload.begin(), payload.end());
  return writer.put(bytes);
}

// ---- basic lifecycle ----

TEST_F(DataHydratorTest, UnknownPathBeforeRequest) {
  state_data_hydrator h(store, codecs);
  EXPECT_EQ(h.status("/foo"), state_data_hydrator::hydration_status::unknown);
  EXPECT_FALSE(h.is_hydrated("/foo"));
}

TEST_F(DataHydratorTest, HydrateLocalChunks) {
  auto ref = store_chunked(store, "hello world");

  state_data_hydrator h(store, codecs);
  auto s = h.request("/test", ref.digest, "test_string");
  EXPECT_EQ(s, state_data_hydrator::hydration_status::ready);
  EXPECT_TRUE(h.is_hydrated("/test"));
  EXPECT_EQ(h.total_hydrated(), 1u);
}

TEST_F(DataHydratorTest, HydrateAndInstallOnState) {
  auto ref = store_chunked(store, "payload123");

  cvc::app a;
  auto &root = cvc::state::instance(a)("test_hydrate");
  state_data_hydrator h(store, codecs);
  auto s = h.request("/val", ref.digest, "test_string", &root);
  EXPECT_EQ(s, state_data_hydrator::hydration_status::ready);
  EXPECT_EQ(root.data<std::string>(), "payload123");
}

TEST_F(DataHydratorTest, WaitReturnsImmediatelyWhenReady) {
  auto ref = store_chunked(store, "data");

  state_data_hydrator h(store, codecs);
  h.request("/p", ref.digest, "test_string");

  auto s = h.wait("/p", std::chrono::milliseconds(100));
  EXPECT_EQ(s, state_data_hydrator::hydration_status::ready);
}

TEST_F(DataHydratorTest, WaitUnknownPathReturnsUnknown) {
  state_data_hydrator h(store, codecs);
  auto s = h.wait("/nope", std::chrono::milliseconds(10));
  EXPECT_EQ(s, state_data_hydrator::hydration_status::unknown);
}

// ---- callback ----

TEST_F(DataHydratorTest, OnHydratedCallback) {
  auto ref = store_chunked(store, "cb_data");

  state_data_hydrator h(store, codecs);

  std::string notified_path;
  state_data_hydrator::hydration_status notified_status =
      state_data_hydrator::hydration_status::unknown;

  h.on_hydrated([&](const std::string &p, state_data_hydrator::hydration_status s) {
    notified_path = p;
    notified_status = s;
  });

  h.request("/cb", ref.digest, "test_string");
  EXPECT_EQ(notified_path, "/cb");
  EXPECT_EQ(notified_status, state_data_hydrator::hydration_status::ready);
}

// ---- cancel ----

TEST_F(DataHydratorTest, CancelRemovesEntry) {
  auto ref = store_chunked(store, "data");
  state_data_hydrator h(store, codecs);
  h.request("/x", ref.digest, "test_string");
  EXPECT_TRUE(h.cancel("/x"));
  EXPECT_EQ(h.status("/x"), state_data_hydrator::hydration_status::unknown);
  // Cancel nonexistent returns false.
  EXPECT_FALSE(h.cancel("/nonexistent"));
}

// ---- transport fetch path ----

TEST_F(DataHydratorTest, FetchViaTranportInproc) {
  // Store chunks in a separate "remote" store.
  memory_state_blob_store remote_store;
  auto ref = store_chunked(remote_store, "remote_payload");

  // Local store has the manifest only — copy it.
  {
    std::vector<unsigned char> manifest_bytes;
    ASSERT_TRUE(remote_store.get(ref.digest, manifest_bytes));
    store.put(manifest_bytes);
  }

  // Copy manifest chunk digests to know which chunks are on remote.
  state_chunk_manifest manifest;
  {
    std::vector<unsigned char> mb;
    ASSERT_TRUE(store.get(ref.digest, mb));
    ASSERT_TRUE(state_chunk_manifest::parse(mb, manifest));
  }

  // Copy chunk data to remote store (they're already there)
  // and set up transport to look at remote store for chunks.
  state_transport_inproc transport;
  transport.set_blob_store(&remote_store);

  state_data_hydrator h(store, codecs);
  h.set_transport(&transport);

  auto s = h.request("/remote", ref.digest, "test_string");
  EXPECT_EQ(s, state_data_hydrator::hydration_status::ready);
  EXPECT_TRUE(h.is_hydrated("/remote"));
  EXPECT_GT(h.total_chunks_fetched(), 0u);
}

// ---- diagnostics ----

TEST_F(DataHydratorTest, Diagnostics) {
  state_data_hydrator h(store, codecs);
  EXPECT_EQ(h.total_hydrated(), 0u);
  EXPECT_EQ(h.total_failed(), 0u);
  EXPECT_EQ(h.total_chunks_fetched(), 0u);
  EXPECT_EQ(h.pending_count(), 0u);

  auto ref = store_chunked(store, "diag");
  h.request("/d", ref.digest, "test_string");
  EXPECT_EQ(h.total_hydrated(), 1u);
  EXPECT_EQ(h.pending_count(), 0u);
}

// ---- bad codec ----

TEST_F(DataHydratorTest, UnknownCodecStillReady) {
  // When the type_name has no registered codec, the hydrator
  // reassembles bytes but skips decoding → still marked ready.
  auto ref = store_chunked(store, "data");
  state_data_hydrator h(store, codecs);
  auto s = h.request("/unknown", ref.digest, "nonexistent_codec");
  EXPECT_EQ(s, state_data_hydrator::hydration_status::ready);
}

TEST_F(DataHydratorTest, ThrowingCodecFails) {
  // Register a codec that always throws during decode.
  state_codec_registry::encode_func enc = [](const boost::any &) -> std::vector<unsigned char> {
    return {};
  };
  state_codec_registry::decode_func dec = [](const std::vector<unsigned char> &) -> boost::any {
    throw std::runtime_error("intentional decode failure");
  };
  codecs.register_codec("bad_codec", enc, dec);

  auto ref = store_chunked(store, "data");
  state_data_hydrator h(store, codecs);
  auto s = h.request("/bad", ref.digest, "bad_codec");
  EXPECT_EQ(s, state_data_hydrator::hydration_status::failed);
  EXPECT_EQ(h.total_failed(), 1u);
}

// ---- retry ----

TEST_F(DataHydratorTest, RetryPendingNoOp) {
  state_data_hydrator h(store, codecs);
  EXPECT_EQ(h.retry_pending(), 0u);
}

// ---- multiple hydrations ----

TEST_F(DataHydratorTest, MultiplePathsIndependent) {
  auto r1 = store_chunked(store, "alpha");
  auto r2 = store_chunked(store, "bravo");

  state_data_hydrator h(store, codecs);
  h.request("/a", r1.digest, "test_string");
  h.request("/b", r2.digest, "test_string");

  EXPECT_TRUE(h.is_hydrated("/a"));
  EXPECT_TRUE(h.is_hydrated("/b"));
  EXPECT_EQ(h.total_hydrated(), 2u);
}
