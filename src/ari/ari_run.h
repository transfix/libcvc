// ari_run.h -- the built ari, run as a launcher would, for the end-to-end tests beside it
// (ari_net_test.cpp, ari_test_command_test.cpp). CMake names the binary in ARI_BINARY.
#pragma once

#include <array>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <random>
#include <string>

#if defined(_WIN32)
#define ARI_POPEN(cmd, mode) _popen(cmd, mode)
#define ARI_PCLOSE(fp) _pclose(fp)
#else
#include <sys/stat.h>
#include <sys/wait.h>
#define ARI_POPEN(cmd, mode) popen(cmd, mode)
#define ARI_PCLOSE(fp) pclose(fp)
#endif

namespace ari_test {

namespace fs = std::filesystem;

struct AriRun {
  int status = -1;
  std::string output; // stdout and stderr
};

// `ari <args>`, with the binary CMake names in ARI_BINARY.
inline AriRun ari(const std::string &args) {
  AriRun r;
  const char *bin = std::getenv("ARI_BINARY");
  if (!bin || !*bin) {
    r.output = "ARI_BINARY is not set";
    return r;
  }
#if defined(_WIN32)
  // cmd /c strips one outer pair of quotes; wrap the whole line so the binary's own stay.
  const std::string cmd = "\"\"" + std::string(bin) + "\" " + args + " 2>&1\"";
#else
  const std::string cmd = "\"" + std::string(bin) + "\" " + args + " 2>&1";
#endif
  FILE *fp = ARI_POPEN(cmd.c_str(), "r");
  if (!fp) {
    r.output = "popen failed";
    return r;
  }
  std::array<char, 4096> buf{};
  while (std::fgets(buf.data(), static_cast<int>(buf.size()), fp))
    r.output += buf.data();
  const int status = ARI_PCLOSE(fp);
#if defined(_WIN32)
  r.status = status;
#else
  r.status = WIFEXITED(status) ? WEXITSTATUS(status) : -1;
#endif
  return r;
}

inline std::string quoted(const fs::path &p) { return "\"" + p.string() + "\""; }

// A fresh directory, removed on scope-out.
struct TempDir {
  fs::path path;
  TempDir() {
    std::random_device rd;
    path = fs::temp_directory_path() /
           ("ari-test-" + std::to_string(rd()) + "-" + std::to_string(rd()));
    fs::create_directories(path);
  }
  ~TempDir() {
    std::error_code ec;
    fs::remove_all(path, ec);
  }
  fs::path write(const std::string &name, const std::string &text) const {
    std::ofstream(path / name, std::ios::binary) << text;
    return path / name;
  }
};

} // namespace ari_test
