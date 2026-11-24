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

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <cvc/ariadne/world_pack.h>
#include <cvc/state/state_blob_store.h> // sha256_hex
#include <filesystem>
#include <fstream>
#include <iterator>
#include <set>
#include <sstream>
#include <stdexcept>

namespace cvc {
namespace ariadne {

namespace fs = std::filesystem;

namespace {

std::string lower(std::string s) {
  for (char &c : s)
    c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  return s;
}

bool is_hex64(const std::string &s) {
  if (s.size() != 64)
    return false;
  for (const char c : s)
    if (!std::isxdigit(static_cast<unsigned char>(c)))
      return false;
  return true;
}

// The SHA-256 of the file at P and its size; false when it can't be read.
bool hash_file(const fs::path &p, std::string &sha, std::uint64_t &size) {
  std::ifstream in(p, std::ios::binary);
  if (!in)
    return false;
  const std::vector<unsigned char> bytes((std::istreambuf_iterator<char>(in)),
                                         std::istreambuf_iterator<char>());
  sha = cvc::sha256_hex(bytes);
  size = bytes.size();
  return true;
}

// Whether a relative '/' path stays inside the pack: not absolute, no "..", no empty part.
bool inside(const std::string &path) {
  if (path.empty() || path.front() == '/' || path.find('\\') != std::string::npos ||
      path.find(':') != std::string::npos)
    return false;
  std::size_t at = 0;
  while (at <= path.size()) {
    const std::size_t end = std::min(path.find('/', at), path.size());
    const std::string part = path.substr(at, end - at);
    if (part.empty() || part == "." || part == "..")
      return false;
    at = end + 1;
  }
  return true;
}

} // namespace

world_pack::world_pack(const std::string &dir) {
  std::error_code ec;
  const fs::path root = fs::absolute(fs::path(dir), ec);
  if (ec || !fs::is_directory(root, ec))
    throw std::runtime_error("world pack " + dir + ": not a directory");
  _root = root.lexically_normal().generic_string();
  std::ifstream in(root / kIndex, std::ios::binary);
  if (!in)
    throw std::runtime_error("world pack " + dir + ": no " + kIndex);
  std::set<std::string> seen;
  std::string line;
  int n = 0;
  while (std::getline(in, line)) {
    ++n;
    if (!line.empty() && line.back() == '\r')
      line.pop_back();
    if (line.empty() || line.front() == '#')
      continue;
    const auto bad = [&](const std::string &what) {
      return std::runtime_error("world pack " + dir + ": " + kIndex + " line " + std::to_string(n) +
                                ": " + what);
    };
    const std::size_t a = line.find(' ');
    const std::size_t b = a == std::string::npos ? a : line.find(' ', a + 1);
    if (b == std::string::npos)
      throw bad("not <sha256> <size> <path>");
    entry e;
    e.sha256 = lower(line.substr(0, a));
    const std::string size = line.substr(a + 1, b - a - 1);
    e.path = line.substr(b + 1);
    if (!is_hex64(e.sha256))
      throw bad("not a SHA-256");
    if (size.empty() || size.find_first_not_of("0123456789") != std::string::npos)
      throw bad("not a size");
    e.size = std::stoull(size);
    if (!inside(e.path))
      throw bad("the path " + e.path + " leaves the pack");
    if (!seen.insert(e.path).second)
      throw bad(e.path + " listed twice");
    _entries.push_back(std::move(e));
  }
}

std::string world_pack::find(const std::string &sha256) const {
  const std::string want = lower(sha256);
  for (const entry &e : _entries)
    if (e.sha256 == want)
      return (fs::path(_root) / fs::path(e.path)).lexically_normal().generic_string();
  return {};
}

bool world_pack::verify(std::string *why) const {
  for (const entry &e : _entries) {
    std::string sha;
    std::uint64_t size = 0;
    const auto fail = [&](const std::string &what) {
      if (why)
        *why = e.path + ": " + what;
      return false;
    };
    if (!hash_file(fs::path(_root) / fs::path(e.path), sha, size))
      return fail("missing");
    if (size != e.size)
      return fail("its size is not the index's");
    if (sha != e.sha256)
      return fail("its SHA-256 is not the index's");
  }
  return true;
}

std::size_t write_world_pack_index(const std::string &dir) {
  std::error_code ec;
  const fs::path root = fs::absolute(fs::path(dir), ec);
  if (ec || !fs::is_directory(root, ec))
    throw std::runtime_error("world pack " + dir + ": not a directory");
  std::vector<world_pack::entry> entries;
  for (const fs::directory_entry &d : fs::recursive_directory_iterator(root)) {
    if (!d.is_regular_file())
      continue;
    const std::string rel = d.path().lexically_relative(root).generic_string();
    if (rel == world_pack::kIndex)
      continue;
    world_pack::entry e;
    e.path = rel;
    if (!hash_file(d.path(), e.sha256, e.size))
      throw std::runtime_error("world pack " + dir + ": can't read " + rel);
    entries.push_back(std::move(e));
  }
  std::sort(entries.begin(), entries.end(),
            [](const world_pack::entry &x, const world_pack::entry &y) { return x.path < y.path; });
  std::ofstream out(root / world_pack::kIndex, std::ios::binary | std::ios::trunc);
  if (!out)
    throw std::runtime_error("world pack " + dir + ": can't write " + world_pack::kIndex);
  out << "# A world pack's index (ARIADNE_WORLDS.md section 12): <sha256> <size> <path>\n";
  for (const world_pack::entry &e : entries)
    out << e.sha256 << ' ' << e.size << ' ' << e.path << '\n';
  if (!out)
    throw std::runtime_error("world pack " + dir + ": can't write " + world_pack::kIndex);
  return entries.size();
}

std::vector<std::string> world_pack_dirs_from_env() {
  std::vector<std::string> out;
  const char *v = std::getenv("CVC_WORLD_PACKS");
  if (!v)
    return out;
#ifdef _WIN32
  const char sep = ';';
#else
  const char sep = ':';
#endif
  std::stringstream ss(v);
  std::string dir;
  while (std::getline(ss, dir, sep))
    if (!dir.empty())
      out.push_back(dir);
  return out;
}

} // namespace ariadne
} // namespace cvc
