/*
  Copyright 2026 The University of Texas at Austin
  This file is part of libcvc.
  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/
// cvc::archive — reading archives from memory: tar (ustar, pax, GNU long names), plain or
// compressed with gzip or zstd, and zip (stored and deflate). What Ariadne's data: block unpacks
// (docs/roadmap/ARIADNE_DATA_ROADMAP.md): a cvcpkg package, a downloaded tarball, a zip.
//
//   const auto files = cvc::archive::unpack(bytes);            // every regular file
//   for (const auto &f : files) use(f.path, *f.bytes);
//
// A format is recognized by its first bytes, never by a file name: cvcpkg names every archive
// .tar.zst, but its data packages are gzip tars and its Windows ones zip.
//
// Safe by construction: only regular files come back; a member whose path is absolute, climbs
// out (`..`) or names a drive is refused; links, devices and directories are skipped; and the
// entry count and the unpacked bytes are capped (limits), so a hostile archive cannot fill the
// memory or write outside the tree. Nothing touches the disk.
//
// gzip and zip's deflate are decoded here (RFC 1951/1952), zstd by libzstd: no zlib dependency.
#ifndef CVC_UTILITY_ARCHIVE_H
#define CVC_UTILITY_ARCHIVE_H

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace cvc {
namespace archive {

// What a byte string is, by its first bytes. gzip and zstd are streams: what they hold is
// found by classify().
enum class format { none, gzip, zstd, tar, zip };
format sniff(std::string_view bytes);
const char *format_name(format f);

// What the bytes hold: a tar (plain, or inside a gzip or zstd stream), a zip, one compressed
// file (a gzip or zstd stream that is not a tar), or none of these (plain bytes).
enum class kind { plain, tar, zip, compressed };
kind classify(std::string_view bytes);

struct limits {
  std::size_t max_entries = 100000;     // regular files kept, at most
  std::uint64_t max_bytes = 2ull << 30; // unpacked (or decompressed) bytes, at most
};

// One regular file of an archive: its normalized relative path ("share/a/b.json") and its
// bytes, shared rather than copied.
struct file {
  std::string path;
  std::shared_ptr<const std::string> bytes;
};

// The regular files of a tar (plain, gzip or zstd) or a zip, in archive order. `keep(path)`
// chooses which to keep (all when empty); a file not kept is skipped without being stored.
// Throws std::runtime_error naming the problem: not an archive, truncated or corrupt data, an
// unsafe path, a cap passed, a zip method other than stored or deflate.
std::vector<file> unpack(std::string_view bytes, const limits &lim = {},
                         const std::function<bool(const std::string &)> &keep = {});

// One gzip or zstd stream decompressed (concatenated members too). Throws std::runtime_error
// past `max_bytes` or on corrupt data.
std::string decompress(std::string_view bytes, std::uint64_t max_bytes = limits{}.max_bytes);

// A member path made relative and canonical: separators to '/', "." and empty components
// dropped. "" when it is unsafe: absolute, with a drive letter, or with a ".." component.
std::string safe_path(std::string_view path);

// Writers, for tests and tools. pack_tar: a ustar archive of (path, bytes) pairs, with GNU long
// names past 100 bytes. gzip: a gzip stream of stored (uncompressed) deflate blocks. zstd: a
// zstd frame.
std::string pack_tar(const std::vector<std::pair<std::string, std::string>> &files);
std::string gzip(std::string_view bytes);
std::string zstd(std::string_view bytes, int level = 3);

// The CRC-32 (ISO-HDLC, as gzip and zip use) of `bytes`, continuing from `crc`.
std::uint32_t crc32(std::string_view bytes, std::uint32_t crc = 0);

} // namespace archive
} // namespace cvc

#endif // CVC_UTILITY_ARCHIVE_H
