// wasm_binary.cpp — see the header. The binary format's sections, as the WebAssembly core
// specification (§5.5) lays them out: an id byte, the content's size (unsigned LEB128), the
// content. Only the import section (2) is read, and only the memory section (5) is rewritten.

#include "wasm_binary.h"

#include <algorithm>

namespace cvc {
namespace ariadne {
namespace wasm {

namespace {

constexpr std::uint64_t kPage = 65536;
constexpr std::uint64_t kMaxPages = 65536; // wasm32's 4 GiB

// The bytes of one section, read front to back; `bad` once a read ran past the end.
struct Reader {
  std::string_view s;
  std::size_t at = 0;
  bool bad = false;

  bool done() const { return at >= s.size(); }
  std::uint8_t byte() {
    if (at >= s.size()) {
      bad = true;
      return 0;
    }
    return static_cast<std::uint8_t>(s[at++]);
  }
  // An unsigned LEB128 (a signed one's bytes are skipped the same way).
  std::uint64_t leb() {
    std::uint64_t v = 0;
    for (int shift = 0; shift < 64; shift += 7) {
      const std::uint8_t b = byte();
      if (bad)
        return 0;
      v |= static_cast<std::uint64_t>(b & 0x7f) << shift;
      if (!(b & 0x80))
        return v;
    }
    bad = true;
    return 0;
  }
  void skip(std::uint64_t n) {
    if (n > s.size() - at) {
      bad = true;
      at = s.size();
    } else {
      at += static_cast<std::size_t>(n);
    }
  }
};

void put_leb(std::string &out, std::uint64_t v) {
  do {
    auto b = static_cast<std::uint8_t>(v & 0x7f);
    v >>= 7;
    if (v)
      b |= 0x80;
    out.push_back(static_cast<char>(b));
  } while (v);
}

// A value type or a reference type's bytes: one, or a (ref null? heaptype) prefix and its index.
void skip_type(Reader &r) {
  const std::uint8_t t = r.byte();
  if (t == 0x63 || t == 0x64)
    r.leb();
}

// Limits: the flags, the minimum and the maximum the flags say is there.
void skip_limits(Reader &r) {
  const std::uint8_t flags = r.byte();
  r.leb();
  if (flags & 1)
    r.leb();
}

// Whether the import section imports a memory; false, with `error`, when it can't be read.
bool imports_memory(std::string_view content, bool &imported, std::string &error) {
  Reader r{content};
  const std::uint64_t n = r.leb();
  for (std::uint64_t i = 0; i < n && !r.bad; ++i) {
    r.skip(r.leb()); // the module's name
    r.skip(r.leb()); // the import's name
    switch (r.byte()) {
    case 0x00: // a function: its type's index
      r.leb();
      break;
    case 0x01: // a table: its element type and limits
      skip_type(r);
      skip_limits(r);
      break;
    case 0x02:
      imported = true;
      return true;
    case 0x03: // a global: its type and mutability
      skip_type(r);
      r.byte();
      break;
    case 0x04: // a tag: its attribute and type's index
      r.byte();
      r.leb();
      break;
    default:
      error = "it imports something of a kind the host doesn't know";
      return false;
    }
  }
  if (r.bad) {
    error = "its import section runs past its end";
    return false;
  }
  return true;
}

// The memory section's content with each memory capped at `cap` pages (0: as it is); the first
// memory's maximum after, in `max_pages`.
bool cap_memories(std::string_view content, std::uint64_t cap, std::string &out,
                  std::uint64_t &max_pages, std::string &error) {
  Reader r{content};
  const std::uint64_t n = r.leb();
  put_leb(out, n);
  for (std::uint64_t i = 0; i < n && !r.bad; ++i) {
    const std::uint8_t flags = r.byte();
    if (flags & 0x02) {
      error = "its memory is shared (the threads proposal): the host caps a plain memory only";
      return false;
    }
    if (flags & 0x04) {
      error = "its memory is a memory64: the host runs wasm32 guests only";
      return false;
    }
    if (flags & ~0x01) {
      error = "its memory's limits are of a kind the host doesn't know (flags " +
              std::to_string(flags) + ")";
      return false;
    }
    const std::uint64_t min = r.leb();
    const bool has_max = (flags & 1) != 0;
    std::uint64_t max = has_max ? r.leb() : kMaxPages;
    if (r.bad)
      break;
    if (cap > 0 && min > cap) {
      error = "its memory starts at " + std::to_string(min) +
              " pages of 64 KiB, past its limit of " + std::to_string(cap) +
              " (ariadne.limits.wasm_max_memory, or the block's limits: memory_mb)";
      return false;
    }
    if (cap > 0)
      max = std::min(max, cap);
    const bool capped = cap > 0 || has_max;
    out.push_back(static_cast<char>(capped ? 0x01 : 0x00));
    put_leb(out, min);
    if (capped)
      put_leb(out, max);
    if (i == 0)
      max_pages = max;
  }
  if (r.bad || !r.done()) {
    error = "its memory section isn't what its size says";
    return false;
  }
  return true;
}

} // namespace

std::string cap_memory(std::string_view module, std::uint64_t max_bytes, std::string &error,
                       std::uint64_t *max_pages) {
  error.clear();
  if (max_pages)
    *max_pages = 0;
  if (module.size() < 8 || module.substr(0, 4) != std::string_view("\0asm", 4) ||
      module.substr(4, 4) != std::string_view("\x01\0\0\0", 4)) {
    error = "it isn't a WebAssembly module (no \\0asm header of version 1)";
    return {};
  }
  const std::uint64_t cap = max_bytes > 0 ? std::min(max_bytes / kPage, kMaxPages) : 0;
  if (max_bytes > 0 && cap == 0) {
    error = "its limit of " + std::to_string(max_bytes) + " bytes is less than a page of 64 KiB";
    return {};
  }
  std::string out(module.substr(0, 8));
  out.reserve(module.size() + 8);
  Reader r{module, 8};
  std::uint64_t pages = 0;
  while (!r.done()) {
    const std::uint8_t id = r.byte();
    const std::uint64_t size = r.leb();
    if (r.bad || size > module.size() - r.at) {
      error = "a section runs past the module's end";
      return {};
    }
    const std::string_view content = module.substr(r.at, static_cast<std::size_t>(size));
    r.skip(size);
    if (id == 2) {
      bool imported = false;
      if (!imports_memory(content, imported, error))
        return {};
      if (imported) {
        error = "it imports its memory: the host caps only a memory the module defines";
        return {};
      }
    }
    if (id != 5) {
      out.push_back(static_cast<char>(id));
      put_leb(out, size);
      out.append(content);
      continue;
    }
    std::string memories;
    if (!cap_memories(content, cap, memories, pages, error))
      return {};
    out.push_back(static_cast<char>(id));
    put_leb(out, memories.size());
    out.append(memories);
  }
  if (max_pages)
    *max_pages = pages;
  return out;
}

} // namespace wasm
} // namespace ariadne
} // namespace cvc
