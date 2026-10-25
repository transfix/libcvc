/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <cvc/state/state_blob_store.h> // sha256_hex
#include <cvc/state/state_identity.h>
#include <fstream>
#include <string>
#include <vector>

#if defined(_WIN32)
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#if defined(_MSC_VER)
#pragma comment(lib, "advapi32.lib")
#endif
#elif !defined(__EMSCRIPTEN__)
#include <unistd.h>
#endif

namespace cvc {

namespace {

std::string trimmed(std::string s) {
  const auto space = [](unsigned char c) { return std::isspace(c) != 0; };
  s.erase(s.begin(), std::find_if_not(s.begin(), s.end(), space));
  s.erase(std::find_if_not(s.rbegin(), s.rend(), space).base(), s.end());
  return s;
}

[[maybe_unused]] std::string first_line(const char *path) {
  std::ifstream in(path);
  std::string line;
  if (in && std::getline(in, line))
    return trimmed(line);
  return {};
}

// The OS's stable machine identifier, or "" (a wasm build has none).
std::string machine_id() {
#if defined(_WIN32)
  // MachineGuid lives in the 64-bit registry view; KEY_WOW64_64KEY reads it from a 32-bit build.
  HKEY key = nullptr;
  if (RegOpenKeyExA(HKEY_LOCAL_MACHINE, "SOFTWARE\\Microsoft\\Cryptography", 0,
                    KEY_QUERY_VALUE | KEY_WOW64_64KEY, &key) != ERROR_SUCCESS)
    return {};
  char buf[128] = {};
  DWORD size = sizeof(buf) - 1, type = 0;
  const LONG rc =
      RegQueryValueExA(key, "MachineGuid", nullptr, &type, reinterpret_cast<BYTE *>(buf), &size);
  RegCloseKey(key);
  if (rc == ERROR_SUCCESS && type == REG_SZ)
    return trimmed(buf);
  return {};
#elif defined(__APPLE__)
  uuid_t id = {};
  const timespec wait = {1, 0};
  if (gethostuuid(id, &wait) != 0)
    return {};
  static const char *hex = "0123456789abcdef";
  std::string out;
  for (unsigned char b : id) {
    out += hex[b >> 4];
    out += hex[b & 15];
  }
  return out;
#elif defined(__EMSCRIPTEN__)
  return {};
#else
  for (const char *path : {"/etc/machine-id", "/var/lib/dbus/machine-id", "/etc/hostid"}) {
    std::string id = first_line(path);
    if (!id.empty())
      return id;
  }
  return {};
#endif
}

std::string host_name() {
#if defined(_WIN32)
  char buf[256] = {};
  DWORD size = sizeof(buf);
  if (GetComputerNameExA(ComputerNameDnsHostname, buf, &size))
    return buf;
  return {};
#elif defined(__EMSCRIPTEN__)
  return {};
#else
  char buf[256] = {};
  if (gethostname(buf, sizeof(buf) - 1) == 0)
    return buf;
  return {};
#endif
}

std::string user_name() {
  for (const char *var : {"USER", "USERNAME", "LOGNAME"})
    if (const char *v = std::getenv(var); v != nullptr && *v != '\0')
      return v;
  return {};
}

} // namespace

bool constant_time_equal(const std::string &a, const std::string &b) noexcept {
  if (a.size() != b.size())
    return false;
  unsigned char diff = 0;
  for (std::size_t i = 0; i < a.size(); ++i)
    diff |= static_cast<unsigned char>(a[i] ^ b[i]);
  return diff == 0;
}

bool valid_room_token(const std::string &token) noexcept {
  if (token.size() < 16 || token.size() > 256)
    return false;
  return std::all_of(token.begin(), token.end(), [](unsigned char c) {
    return std::isalnum(c) != 0 || c == '.' || c == '_' || c == '~' || c == '-';
  });
}

bool valid_device_id(const std::string &id) noexcept {
  if (id.empty() || id.size() > 64)
    return false;
  return std::all_of(id.begin(), id.end(), [](unsigned char c) {
    return std::isalnum(c) != 0 || c == '.' || c == '_' || c == '-';
  });
}

std::string scoped_device_id(const std::string &device_id, const std::string &scope) {
  if (device_id.empty())
    return {};
  std::string in = "cvc-device-scope/v1";
  in.push_back('\0');
  in += scope;
  in.push_back('\0');
  in += device_id;
  return sha256_hex(reinterpret_cast<const unsigned char *>(in.data()), in.size()).substr(0, 32);
}

std::string default_device_id() {
  if (const char *v = std::getenv("CVC_DEVICE_ID"); v != nullptr && *v != '\0')
    return v;
#if defined(__EMSCRIPTEN__)
  return {};
#endif
  const std::string machine = machine_id(), host = host_name(), user = user_name();
  if (machine.empty() && host.empty() && user.empty())
    return {};
  // NUL-separated, so no two input triples hash the same bytes.
  std::string in = "cvc-device-id/v1";
  for (const std::string *part : {&machine, &host, &user}) {
    in.push_back('\0');
    in += *part;
  }
  return sha256_hex(reinterpret_cast<const unsigned char *>(in.data()), in.size()).substr(0, 32);
}

} // namespace cvc
