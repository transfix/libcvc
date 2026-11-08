/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// A Unix-domain socket path for a test, short enough to bind.
//
// sockaddr_un::sun_path holds 104 bytes on macOS and the BSDs (108 on Linux),
// and state_transport_ipc::start throws "socket path too long" past it. The temp
// directory is not always short: macOS's per-user TMPDIR is already ~50 bytes,
// and `cvcpkg build` points TMPDIR at the build's own work dir
// (.../T/cvcpkg-libcvc-XXXXXXXX/tmp/), which left the tests' pid+timestamp+label
// socket names over the limit. When the name does not fit under the temp
// directory, use /tmp, the short directory every POSIX host has.

#pragma once

#include <cstddef>
#include <filesystem>
#include <string>

namespace cvc_test {

// The smallest sun_path among the platforms libcvc builds on, NUL included.
inline constexpr std::size_t kMaxSunPath = 104;

inline std::string ipc_socket_path(const std::string &name) {
  std::string p = (std::filesystem::temp_directory_path() / name).string();
  if (p.size() < kMaxSunPath)
    return p;
  return "/tmp/" + name;
}

} // namespace cvc_test
