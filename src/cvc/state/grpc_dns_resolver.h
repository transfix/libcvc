/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// Which DNS resolver gRPC uses for the transports' hostnames. Private to the gRPC state and stream
// transports, whose constructors call it before the process makes its first gRPC object.

#ifndef __CVC_STATE_GRPC_DNS_RESOLVER_H__
#define __CVC_STATE_GRPC_DNS_RESOLVER_H__

#ifdef _WIN32
#include <cstdlib>
#include <mutex>
#endif

namespace cvc {
namespace grpc_dns {

// On Windows, gRPC's c-ares resolver aborts the process on its first lookup of a hostname (gRPC
// 1.68.2 with c-ares 1.34.8: grpc_polled_fd_windows.cc "Check failed: *from_len <=
// recv_from_source_addr_len_"), so grpc://relay.example.org:7700 killed every native Windows
// client while grpc://<an IP>:7700 worked. gRPC's native resolver (getaddrinfo) has no such path.
// gRPC reads GRPC_DNS_RESOLVER once, when it first initializes, so this sets it before then, and
// only when unset: a user's own GRPC_DNS_RESOLVER=ares still wins.
inline void default_to_native_on_windows() {
#ifdef _WIN32
  static std::once_flag once;
  std::call_once(once, [] {
    if (!std::getenv("GRPC_DNS_RESOLVER"))
      _putenv_s("GRPC_DNS_RESOLVER", "native");
  });
#endif
}

} // namespace grpc_dns
} // namespace cvc

#endif // __CVC_STATE_GRPC_DNS_RESOLVER_H__
