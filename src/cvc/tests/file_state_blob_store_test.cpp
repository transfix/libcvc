/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <algorithm>
#include <cvc/core/state_blob_store.h>
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <string>
#include <thread>
#include <vector>

using namespace cvc;

namespace {

// Create a unique temp directory for each test.
class FileBlobStoreTest : public ::testing::Test {
protected:
  void SetUp() override {
    _dir = std::filesystem::temp_directory_path() /
           ("cvc_blob_test_" + std::to_string(getpid()) + "_" + std::to_string(_seq++));
    std::filesystem::create_directories(_dir);
  }

  void TearDown() override {
    std::error_code ec;
    std::filesystem::remove_all(_dir, ec);
  }

  std::string dir() const { return _dir.string(); }

private:
  std::filesystem::path _dir;
  static int _seq;
};

int FileBlobStoreTest::_seq = 0;

std::vector<unsigned char> make_bytes(const std::string &s) { return {s.begin(), s.end()}; }

} // namespace

TEST_F(FileBlobStoreTest, ConstructCreatesDirectory) {
  auto sub = dir() + "/store";
  file_state_blob_store store(sub);
  EXPECT_TRUE(std::filesystem::exists(sub));
  EXPECT_EQ(store.size(), 0u);
  EXPECT_EQ(store.bytes_stored(), 0u);
}

TEST_F(FileBlobStoreTest, PutAndGet) {
  file_state_blob_store store(dir());
  auto data = make_bytes("hello world");
  auto ref = store.put(data);

  EXPECT_FALSE(ref.digest.empty());
  EXPECT_EQ(ref.size_bytes, data.size());
  EXPECT_EQ(store.size(), 1u);
  EXPECT_EQ(store.bytes_stored(), data.size());

  std::vector<unsigned char> out;
  ASSERT_TRUE(store.get(ref.digest, out));
  EXPECT_EQ(out, data);
}

TEST_F(FileBlobStoreTest, PutWithCodec) {
  file_state_blob_store store(dir());
  auto data = make_bytes("test payload");
  auto ref = store.put(data, "application/x-cvc-test");

  EXPECT_EQ(ref.codec, "application/x-cvc-test");
  EXPECT_EQ(ref.size_bytes, data.size());
}

TEST_F(FileBlobStoreTest, PutDedups) {
  file_state_blob_store store(dir());
  auto data = make_bytes("dedup test");
  auto ref1 = store.put(data);
  auto ref2 = store.put(data);

  EXPECT_EQ(ref1.digest, ref2.digest);
  EXPECT_EQ(store.size(), 1u);
  EXPECT_EQ(store.bytes_stored(), data.size());
}

TEST_F(FileBlobStoreTest, HasAndErase) {
  file_state_blob_store store(dir());
  auto data = make_bytes("eraseme");
  auto ref = store.put(data);

  EXPECT_TRUE(store.has(ref.digest));
  EXPECT_TRUE(store.erase(ref.digest));
  EXPECT_FALSE(store.has(ref.digest));
  EXPECT_EQ(store.size(), 0u);
  EXPECT_EQ(store.bytes_stored(), 0u);

  // Double erase returns false.
  EXPECT_FALSE(store.erase(ref.digest));
}

TEST_F(FileBlobStoreTest, GetMissingReturnsFalse) {
  file_state_blob_store store(dir());
  std::vector<unsigned char> out;
  EXPECT_FALSE(store.get("nonexistent_digest", out));
}

TEST_F(FileBlobStoreTest, DigestsEnumeration) {
  file_state_blob_store store(dir());
  auto ref1 = store.put(make_bytes("one"));
  auto ref2 = store.put(make_bytes("two"));
  auto ref3 = store.put(make_bytes("three"));

  auto all = store.digests();
  EXPECT_EQ(all.size(), 3u);

  std::sort(all.begin(), all.end());
  std::vector<std::string> expected = {ref1.digest, ref2.digest, ref3.digest};
  std::sort(expected.begin(), expected.end());
  EXPECT_EQ(all, expected);
}

TEST_F(FileBlobStoreTest, PersistenceAcrossInstances) {
  auto data1 = make_bytes("persist one");
  auto data2 = make_bytes("persist two");
  std::string d1, d2;

  {
    file_state_blob_store store(dir());
    d1 = store.put(data1).digest;
    d2 = store.put(data2).digest;
    EXPECT_EQ(store.size(), 2u);
  }

  // Reopen from the same directory.
  file_state_blob_store store2(dir());
  EXPECT_EQ(store2.size(), 2u);
  EXPECT_EQ(store2.bytes_stored(), data1.size() + data2.size());
  EXPECT_TRUE(store2.has(d1));
  EXPECT_TRUE(store2.has(d2));

  std::vector<unsigned char> out;
  ASSERT_TRUE(store2.get(d1, out));
  EXPECT_EQ(out, data1);
  ASSERT_TRUE(store2.get(d2, out));
  EXPECT_EQ(out, data2);
}

TEST_F(FileBlobStoreTest, GitStyleDirectoryLayout) {
  file_state_blob_store store(dir());
  auto data = make_bytes("layout test");
  auto ref = store.put(data);

  // The file should be at <root>/<prefix2>/<rest>
  auto prefix = ref.digest.substr(0, 2);
  auto rest = ref.digest.substr(2);
  auto expected_path = std::filesystem::path(dir()) / prefix / rest;
  EXPECT_TRUE(std::filesystem::exists(expected_path));
}

TEST_F(FileBlobStoreTest, LargeBlob) {
  file_state_blob_store store(dir());

  // 1 MiB blob.
  std::vector<unsigned char> big(1024 * 1024);
  for (std::size_t i = 0; i < big.size(); ++i)
    big[i] = static_cast<unsigned char>(i & 0xFF);

  auto ref = store.put(big);
  EXPECT_EQ(ref.size_bytes, big.size());

  std::vector<unsigned char> out;
  ASSERT_TRUE(store.get(ref.digest, out));
  EXPECT_EQ(out, big);
}

TEST_F(FileBlobStoreTest, ConcurrentPuts) {
  file_state_blob_store store(dir());
  constexpr int N = 50;

  std::vector<std::thread> threads;
  threads.reserve(N);
  for (int i = 0; i < N; ++i) {
    threads.emplace_back([&store, i] {
      auto data = make_bytes("concurrent_" + std::to_string(i));
      store.put(data);
    });
  }
  for (auto &t : threads)
    t.join();

  EXPECT_EQ(store.size(), static_cast<std::size_t>(N));
}

TEST_F(FileBlobStoreTest, EmptyBlob) {
  file_state_blob_store store(dir());
  std::vector<unsigned char> empty;
  auto ref = store.put(empty);
  EXPECT_EQ(ref.size_bytes, 0u);
  EXPECT_EQ(store.size(), 1u);
  EXPECT_EQ(store.bytes_stored(), 0u);

  std::vector<unsigned char> out;
  ASSERT_TRUE(store.get(ref.digest, out));
  EXPECT_TRUE(out.empty());
}

TEST_F(FileBlobStoreTest, EraseRemovesFile) {
  file_state_blob_store store(dir());
  auto data = make_bytes("will be erased");
  auto ref = store.put(data);

  auto prefix = ref.digest.substr(0, 2);
  auto rest = ref.digest.substr(2);
  auto file_path = std::filesystem::path(dir()) / prefix / rest;
  EXPECT_TRUE(std::filesystem::exists(file_path));

  store.erase(ref.digest);
  EXPECT_FALSE(std::filesystem::exists(file_path));
}
