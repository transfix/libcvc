/*
  Copyright 2025 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#ifndef __CVC_STATE_CODEC_REGISTRY_H__
#define __CVC_STATE_CODEC_REGISTRY_H__

#include <boost/any.hpp>
#include <cstdint>
#include <cvc/namespace.h>
#include <functional>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace cvc {

// ----------------
// cvc::state_codec_registry
// ----------------
// Purpose:
//   Maps libcvc type names (typically `typeid(T).name()` or the
//   value returned by `cvc::state::dataTypeName()`) to a pair of
//   functions that serialize the value to bytes and deserialize
//   bytes back into a `boost::any`. The bytes produced here are
//   what get stored in a state_payload (inline) or written to a
//   blob and referenced via state_blob_ref.
//
// Threading:
//   Thread-safe. Registration and lookup may run concurrently.
//
// Versioning:
//   Each registered codec has an opaque `codec_id` string that
//   travels in the wire envelope (state_blob_ref::codec or
//   state_mutation::type_name) so peers can pick the right
//   decoder even when type names disagree across builds.
//
// Built-in codecs:
//   register_builtin_codecs() installs encoders for the small set
//   of POD scalars the distributed-state layer needs in Phase 2
//   (bool, int32, int64, uint32, uint64, float, double, string).
//   Endianness on the wire is little-endian.
//
class state_codec_registry {
public:
  using encode_func = std::function<std::vector<unsigned char>(const boost::any &)>;
  using decode_func = std::function<boost::any(const std::vector<unsigned char> &)>;

  struct codec_entry {
    std::string codec_id;
    encode_func encode;
    decode_func decode;
  };

  state_codec_registry();

  // Register a codec keyed by `type_name`. `codec_id` is the
  // string that goes onto the wire; if empty, defaults to
  // `type_name`. Replaces any prior registration.
  void register_codec(const std::string &type_name, encode_func encode, decode_func decode,
                      const std::string &codec_id = std::string());

  // Returns true if a codec is registered for `type_name`.
  bool has(const std::string &type_name) const;

  // Encode `value` using the codec registered for `type_name`.
  // Throws std::runtime_error if no codec is registered.
  std::vector<unsigned char> encode(const std::string &type_name, const boost::any &value) const;

  // Decode `bytes` using the codec registered for `type_name`.
  // Throws std::runtime_error if no codec is registered.
  boost::any decode(const std::string &type_name, const std::vector<unsigned char> &bytes) const;

  // Look up the codec_id string for a given type. Returns empty if
  // not registered.
  std::string codec_id_for(const std::string &type_name) const;

  // Number of registered codecs.
  std::size_t size() const;

  // Install codecs for bool, int32/64, uint32/64, float, double,
  // and std::string. Idempotent.
  void register_builtin_codecs();

private:
  mutable std::mutex _mutex;
  std::unordered_map<std::string, codec_entry> _codecs;
};

} // namespace cvc

#endif // __CVC_STATE_CODEC_REGISTRY_H__
