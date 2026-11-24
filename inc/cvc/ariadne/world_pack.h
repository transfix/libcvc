/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.

  libcvc is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.
*/

// cvc::ariadne::world_pack: a world document and its assets with an index (ARIADNE_WORLDS.md
// section 12, Q16; OW5k), here a directory on disk: a mounted share, a checkout, an unpacked
// archive, an application's install folder.
//
// The index is `world-pack.index` at the pack's root, one line per file:
//
//   <sha256, 64 lowercase hex> <size in bytes> <path relative to the root, '/' separated>
//
// Blank lines and lines starting with '#' are skipped; a path may hold spaces (it is the rest of
// the line). write_world_pack_index makes one for every file under a directory.
//
// A client finds a document by its content (find: the SHA-256 the room names, room_world_source)
// rather than by its name, so a pack can come from anywhere. What it reads is checked against the
// index first (verify: every listed file's size and SHA-256), so a pack from an untrusted place can
// be missing things or wrong, which the client then reports, but it can't substitute content.
//
// world_replica looks in the packs a client is pointed at (CVC_WORLD_PACKS, add_pack) when the
// room's document URI does not resolve here to the relay's bytes.

#ifndef __CVC_ARIADNE_WORLD_PACK_H__
#define __CVC_ARIADNE_WORLD_PACK_H__

#include <cstdint>
#include <string>
#include <vector>

namespace cvc {
namespace ariadne {

class world_pack {
public:
  // The index's name at a pack's root.
  static constexpr const char *kIndex = "world-pack.index";

  struct entry {
    std::string sha256; // lowercase hex
    std::uint64_t size = 0;
    std::string path; // relative to the root, '/' separated
  };

  // Opens DIR's index. Throws std::runtime_error for a directory without one, a malformed line, a
  // path that leaves the pack (absolute, or through ..), or one listed twice.
  explicit world_pack(const std::string &dir);

  const std::string &root() const { return _root; } // absolute
  const std::vector<entry> &entries() const { return _entries; }

  // The file listed with this SHA-256 (either case), its absolute path; "" when none is.
  std::string find(const std::string &sha256) const;

  // Whether every listed file is there with its size and SHA-256. WHY: the first that is not.
  // Each file is read once per call.
  bool verify(std::string *why = nullptr) const;

private:
  std::string _root;
  std::vector<entry> _entries;
};

// Writes DIR's index: every regular file under it but the index, its SHA-256, size and path,
// sorted by path. Returns how many it listed. Throws std::runtime_error when DIR is not a
// directory or a file can't be read.
std::size_t write_world_pack_index(const std::string &dir);

// The packs a client is pointed at by the CVC_WORLD_PACKS environment variable: directories
// separated by ';' on Windows and ':' elsewhere, as PATH is; empty entries skipped.
std::vector<std::string> world_pack_dirs_from_env();

} // namespace ariadne
} // namespace cvc

#endif // __CVC_ARIADNE_WORLD_PACK_H__
