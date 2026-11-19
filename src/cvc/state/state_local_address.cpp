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

// local_address_toward and valid_reported_address (state_identity.h). A file of its own: Asio's
// Winsock headers must not meet state_identity.cpp's <windows.h>.

#include <cctype>
#include <cstring>
#include <cvc/state/state_identity.h>
#include <string>
#if !defined(__EMSCRIPTEN__)
#include <boost/asio/io_context.hpp>
#include <boost/asio/ip/udp.hpp>
#endif

namespace cvc {

namespace {

// TARGET's host and port, from "host:port", "[v6]:port", a gRPC target ("dns:///host:port",
// "ipv4:1.2.3.4:5", "ipv6:[::1]:5") or a URL ("ws://host:port/path"). False when it names no host.
[[maybe_unused]] bool split_target(std::string t, std::string &host, std::string &port) {
  for (const char *scheme : {"dns:///", "dns:", "ipv4:", "ipv6:"})
    if (t.rfind(scheme, 0) == 0) {
      t.erase(0, std::strlen(scheme));
      break;
    }
  if (const std::size_t s = t.find("://"); s != std::string::npos)
    t.erase(0, s + 3);
  if (const std::size_t slash = t.find('/'); slash != std::string::npos)
    t.erase(slash);
  host.clear();
  port.clear();
  if (!t.empty() && t.front() == '[') {
    const std::size_t close = t.find(']');
    if (close == std::string::npos)
      return false;
    host = t.substr(1, close - 1);
    if (close + 1 < t.size() && t[close + 1] == ':')
      port = t.substr(close + 2);
  } else if (const std::size_t colon = t.rfind(':'); colon != std::string::npos) {
    host = t.substr(0, colon);
    port = t.substr(colon + 1);
  } else {
    host = t;
  }
  return !host.empty();
}

} // namespace

std::string local_address_toward(const std::string &target) {
#if defined(__EMSCRIPTEN__)
  (void)target;
  return std::string();
#else
  std::string host, port;
  if (!split_target(target, host, port))
    return std::string();
  if (port.empty())
    port = "9"; // any port: a connected UDP socket sends nothing
  try {
    boost::asio::io_context io;
    boost::asio::ip::udp::resolver resolver(io);
    boost::system::error_code ec;
    const auto endpoints = resolver.resolve(host, port, ec);
    if (ec)
      return std::string();
    for (const auto &e : endpoints) {
      boost::asio::ip::udp::socket s(io);
      s.open(e.endpoint().protocol(), ec);
      if (ec)
        continue;
      s.connect(e.endpoint(), ec);
      if (ec)
        continue;
      const auto local = s.local_endpoint(ec);
      if (ec)
        continue;
      std::string a = local.address().to_string();
      if (valid_reported_address(a))
        return a;
    }
  } catch (const std::exception &) {
  }
  return std::string();
#endif
}

bool valid_reported_address(const std::string &address) noexcept {
  if (address.empty() || address.size() > 64)
    return false;
  for (const char c : address) {
    const unsigned char u = static_cast<unsigned char>(c);
    if (!std::isalnum(u) && c != '.' && c != ':' && c != '%' && c != '_' && c != '-')
      return false;
  }
  return true;
}

} // namespace cvc
