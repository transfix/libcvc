/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <cvc/state_chunked_blob.h>

#include <cstring>

namespace CVC_NAMESPACE {

namespace {

constexpr unsigned char MAGIC[4] = {'C', 'V', 'C', 'M'};

void append_u32(std::vector<unsigned char> &out, std::uint32_t v) {
  out.push_back(static_cast<unsigned char>(v & 0xff));
  out.push_back(static_cast<unsigned char>((v >> 8) & 0xff));
  out.push_back(static_cast<unsigned char>((v >> 16) & 0xff));
  out.push_back(static_cast<unsigned char>((v >> 24) & 0xff));
}

void append_u64(std::vector<unsigned char> &out, std::uint64_t v) {
  for (int i = 0; i < 8; ++i)
    out.push_back(static_cast<unsigned char>((v >> (8 * i)) & 0xff));
}

void append_str(std::vector<unsigned char> &out, const std::string &s) {
  append_u32(out, static_cast<std::uint32_t>(s.size()));
  out.insert(out.end(), s.begin(), s.end());
}

bool read_u32(const std::vector<unsigned char> &b, std::size_t &pos,
              std::uint32_t &out) {
  if (pos + 4 > b.size())
    return false;
  out = static_cast<std::uint32_t>(b[pos]) |
        (static_cast<std::uint32_t>(b[pos + 1]) << 8) |
        (static_cast<std::uint32_t>(b[pos + 2]) << 16) |
        (static_cast<std::uint32_t>(b[pos + 3]) << 24);
  pos += 4;
  return true;
}

bool read_u64(const std::vector<unsigned char> &b, std::size_t &pos,
              std::uint64_t &out) {
  if (pos + 8 > b.size())
    return false;
  out = 0;
  for (int i = 0; i < 8; ++i)
    out |= static_cast<std::uint64_t>(b[pos + i]) << (8 * i);
  pos += 8;
  return true;
}

bool read_str(const std::vector<unsigned char> &b, std::size_t &pos,
              std::string &out) {
  std::uint32_t n = 0;
  if (!read_u32(b, pos, n))
    return false;
  if (pos + n > b.size())
    return false;
  out.assign(reinterpret_cast<const char *>(b.data() + pos), n);
  pos += n;
  return true;
}

} // namespace

// ---------- state_chunk_manifest ----------

std::vector<unsigned char> state_chunk_manifest::serialize() const {
  std::vector<unsigned char> out;
  out.reserve(64 + chunks.size() * 80);
  out.insert(out.end(), MAGIC, MAGIC + 4);
  append_u32(out, version);
  append_u32(out, chunk_size);
  append_u64(out, total_size);
  append_str(out, codec);
  append_str(out, content_digest);
  append_u32(out, static_cast<std::uint32_t>(chunks.size()));
  for (std::size_t i = 0; i < chunks.size(); ++i) {
    append_str(out, chunks[i]);
    append_u32(out, i < chunk_bytes.size() ? chunk_bytes[i] : 0u);
  }
  return out;
}

bool state_chunk_manifest::parse(const std::vector<unsigned char> &bytes,
                                 state_chunk_manifest &out) {
  if (bytes.size() < 4 || std::memcmp(bytes.data(), MAGIC, 4) != 0)
    return false;
  std::size_t pos = 4;
  if (!read_u32(bytes, pos, out.version))
    return false;
  if (out.version != 1)
    return false;
  if (!read_u32(bytes, pos, out.chunk_size))
    return false;
  if (!read_u64(bytes, pos, out.total_size))
    return false;
  if (!read_str(bytes, pos, out.codec))
    return false;
  if (!read_str(bytes, pos, out.content_digest))
    return false;
  std::uint32_t count = 0;
  if (!read_u32(bytes, pos, count))
    return false;
  out.chunks.clear();
  out.chunks.reserve(count);
  out.chunk_bytes.clear();
  out.chunk_bytes.reserve(count);
  for (std::uint32_t i = 0; i < count; ++i) {
    std::string d;
    std::uint32_t n = 0;
    if (!read_str(bytes, pos, d))
      return false;
    if (!read_u32(bytes, pos, n))
      return false;
    out.chunks.push_back(std::move(d));
    out.chunk_bytes.push_back(n);
  }
  return pos == bytes.size();
}

// ---------- state_chunked_blob_writer ----------

state_chunked_blob_writer::state_chunked_blob_writer(state_blob_store &store,
                                                     std::uint32_t chunk_size)
    : _store(store), _chunk_size(chunk_size == 0 ? 1u : chunk_size) {}

state_blob_ref
state_chunked_blob_writer::put(const std::vector<unsigned char> &bytes,
                               const std::string &codec) {
  state_chunk_manifest m;
  m.version = 1;
  m.chunk_size = _chunk_size;
  m.total_size = bytes.size();
  m.codec = codec;
  m.content_digest = sha256_hex(bytes);

  const std::size_t total = bytes.size();
  std::size_t off = 0;
  std::vector<unsigned char> chunk;
  while (off < total) {
    const std::size_t n = std::min<std::size_t>(_chunk_size, total - off);
    chunk.assign(bytes.begin() + off, bytes.begin() + off + n);
    const bool already = _store.has(sha256_hex(chunk));
    state_blob_ref ref = _store.put(chunk);
    if (already) {
      _chunks_dedup.fetch_add(1, std::memory_order_relaxed);
    } else {
      _chunks_written.fetch_add(1, std::memory_order_relaxed);
      _bytes_written.fetch_add(n, std::memory_order_relaxed);
    }
    m.chunks.push_back(ref.digest);
    m.chunk_bytes.push_back(static_cast<std::uint32_t>(n));
    off += n;
  }
  // Empty payloads still produce a valid (zero-chunk) manifest.

  std::vector<unsigned char> manifest_bytes = m.serialize();
  state_blob_ref manifest_ref = _store.put(manifest_bytes, codec);
  _manifests_written.fetch_add(1, std::memory_order_relaxed);
  return manifest_ref;
}

// ---------- state_chunked_blob_reader ----------

state_chunked_blob_reader::state_chunked_blob_reader(state_blob_store &store)
    : _store(store) {}

bool state_chunked_blob_reader::load_manifest(
    const std::string &manifest_digest, state_chunk_manifest &out) const {
  std::vector<unsigned char> bytes;
  if (!_store.get(manifest_digest, bytes))
    return false;
  return state_chunk_manifest::parse(bytes, out);
}

std::vector<std::string> state_chunked_blob_reader::missing_chunks(
    const state_chunk_manifest &manifest) const {
  std::vector<std::string> missing;
  for (const auto &d : manifest.chunks) {
    if (!_store.has(d))
      missing.push_back(d);
  }
  return missing;
}

std::vector<std::string> state_chunked_blob_reader::missing_chunks(
    const std::string &manifest_digest) const {
  state_chunk_manifest m;
  if (!load_manifest(manifest_digest, m))
    return {};
  return missing_chunks(m);
}

bool state_chunked_blob_reader::get(const state_chunk_manifest &manifest,
                                    std::vector<unsigned char> &out) const {
  out.clear();
  out.reserve(static_cast<std::size_t>(manifest.total_size));
  std::vector<unsigned char> chunk;
  for (const auto &d : manifest.chunks) {
    if (!_store.get(d, chunk))
      return false;
    out.insert(out.end(), chunk.begin(), chunk.end());
  }
  if (!manifest.content_digest.empty() &&
      sha256_hex(out) != manifest.content_digest) {
    out.clear();
    return false;
  }
  return true;
}

bool state_chunked_blob_reader::get(const std::string &manifest_digest,
                                    std::vector<unsigned char> &out) const {
  state_chunk_manifest m;
  if (!load_manifest(manifest_digest, m))
    return false;
  return get(m, out);
}

} // namespace CVC_NAMESPACE
