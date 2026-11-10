/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// value_cbor.h — DSL values as CBOR (RFC 8949), typed: how values cross a WebAssembly guest's
// boundary (docs/roadmap/ARIADNE_SCRIPTING_HOSTS.md §8.4, src/ariadne_wasm) and a room-call
// between a client and its relay (§6.5.5, cvc/ariadne/conference/room_calls.h).
//
// The mapping mirrors state-tree values:
//   nil        null               bool    false / true
//   integer    unsigned / negative integer (64-bit)
//   float      float64 (float16 and float32 are read too), so 2.0 stays a float
//   string     text string        bytes   byte string
//   symbol     with cbor_options::symbols, tag kCborSymbolTag (39, "identifier" in IANA's
//              registry) over its name, which reads back as a symbol; without, a text string,
//              which a guest reads as one (and tag 39 then reads as its text)
//   list       array              dict    map with text keys, in order
//   host object (a data_object): a handle, kCborHandleTag over an unsigned integer (§8.4.1), when
//              the options have a handle table; without one it does not cross
// A closure, a native function or a generator does not cross. Other tags are read through: a
// tagged item decodes as its content.
#ifndef CVC_STATE_EXEC_VALUE_CBOR_H
#define CVC_STATE_EXEC_VALUE_CBOR_H

#include <cstdint>
#include <cvc/state/state_exec/types.h>
#include <string>
#include <string_view>

namespace cvc::state_exec {

// A table of host objects, by handle (a WebAssembly guest instance's, §8.4.1).
class cbor_handles {
public:
  virtual ~cbor_handles() = default;
  // A new handle for `obj`, which the table keeps until it is dropped. Throws
  // std::invalid_argument when the table is full.
  virtual std::uint64_t put(const data_object_ptr &obj) = 0;
  // The object `handle` names; null for one that was dropped or never given.
  virtual data_object_ptr get(std::uint64_t handle) const = 0;
};

struct cbor_options {
  cbor_handles *handles = nullptr; // host objects cross as handles in it (none: they don't)
  bool symbols = false;            // a symbol crosses as itself (tag 39), not as text
};

// The CBOR of `v`. Throws std::invalid_argument for a value that does not cross (naming it), or
// one nested deeper than kMaxCborDepth.
std::string encode_cbor(const value_t &v, const cbor_options &options = {});

// The one CBOR item `data` holds. Throws std::invalid_argument on a malformed or truncated item,
// trailing bytes, an indefinite length, a map key that is not text, an integer outside int64,
// nesting deeper than kMaxCborDepth, or a handle that names nothing (or any handle, without a
// table).
value_t decode_cbor(std::string_view data, const cbor_options &options = {});

constexpr int kMaxCborDepth = 64;

// The tag a host object's handle crosses under: "arih", in CBOR's first-come range of tags.
constexpr std::uint64_t kCborHandleTag = 0x61726968;

// The tag a symbol crosses under (cbor_options::symbols): 39, IANA's "identifier".
constexpr std::uint64_t kCborSymbolTag = 39;

} // namespace cvc::state_exec

#endif
