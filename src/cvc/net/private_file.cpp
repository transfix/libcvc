// cvc::net — files that hold a user's secrets. See private_file.h.

#include "private_file.h"

#include <cstdlib>
#include <fstream>
#include <system_error>

#if defined(_WIN32)
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#elif !defined(__EMSCRIPTEN__)
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>
#endif

namespace cvc {
namespace net {
namespace private_file {

namespace fs = std::filesystem;

namespace {
bool fail(std::string *error, std::string message) {
  if (error)
    *error = std::move(message);
  return false;
}
} // namespace

fs::path config_dir() {
#if defined(__EMSCRIPTEN__)
  return {};
#elif defined(_WIN32)
  if (const wchar_t *dir = ::_wgetenv(L"LOCALAPPDATA"); dir != nullptr && *dir != L'\0')
    return fs::path(dir) / L"cvc";
  return {};
#else
  if (const char *xdg = std::getenv("XDG_CONFIG_HOME"); xdg != nullptr && *xdg != '\0')
    return fs::path(xdg) / "cvc";
  if (const char *home = std::getenv("HOME"); home != nullptr && *home != '\0')
    return fs::path(home) / ".config" / "cvc";
  return {};
#endif
}

bool write(const fs::path &path, const std::string &text, const std::string &what,
           std::string *error) {
#if defined(__EMSCRIPTEN__)
  (void)path;
  (void)text;
  return fail(error, what + " cannot be written in a browser");
#else
  std::error_code ec;
  const fs::path dir = path.parent_path();
  if (!dir.empty() && !fs::exists(dir, ec)) {
    fs::create_directories(dir, ec);
#if !defined(_WIN32)
    ::chmod(dir.c_str(), 0700); // ours, so nobody else lists it
#endif
  }
#if defined(_WIN32)
  fs::path tmp = path;
  tmp += L".tmp." + std::to_wstring(::GetCurrentProcessId());
  {
    std::ofstream out(tmp, std::ios::binary | std::ios::trunc);
    out << text;
    if (!out.flush()) {
      fs::remove(tmp, ec);
      return fail(error, "could not write " + what + " '" + path.string() + "'");
    }
  }
  if (::MoveFileExW(tmp.c_str(), path.c_str(),
                    MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH) == 0) {
    fs::remove(tmp, ec);
    return fail(error, "could not replace " + what + " '" + path.string() + "'");
  }
  return true;
#else
  const std::string tmp = path.string() + ".tmp." + std::to_string(::getpid());
  ::unlink(tmp.c_str());
  const int fd = ::open(tmp.c_str(), O_WRONLY | O_CREAT | O_EXCL, 0600);
  if (fd < 0)
    return fail(error, "could not write " + what + " '" + path.string() + "'");
  bool ok = ::fchmod(fd, 0600) == 0; // whatever the umask
  ok = ok && ::write(fd, text.data(), text.size()) == static_cast<ssize_t>(text.size());
  ok = ok && ::fsync(fd) == 0;
  ok = ::close(fd) == 0 && ok;
  ok = ok && ::rename(tmp.c_str(), path.c_str()) == 0;
  if (!ok) {
    ::unlink(tmp.c_str());
    return fail(error, "could not write " + what + " '" + path.string() + "'");
  }
  return true;
#endif
#endif
}

bool check_owner_only(const fs::path &path, const std::string &what, std::string *error) {
#if defined(_WIN32) || defined(__EMSCRIPTEN__)
  (void)path;
  (void)what;
  (void)error;
  return true;
#else
  struct stat st {};
  if (::stat(path.c_str(), &st) != 0)
    return fail(error, "cannot read " + what + " '" + path.string() + "'");
  if (st.st_uid != ::geteuid())
    return fail(error, what + " '" + path.string() + "' belongs to another user");
  if ((st.st_mode & 077) != 0)
    return fail(error, what + " '" + path.string() +
                           "' is open to other users; it must be owner-only (chmod 600)");
  return true;
#endif
}

} // namespace private_file
} // namespace net
} // namespace cvc
