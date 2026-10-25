// cvc::process — see inc/cvc/core/process.h.
#include <cvc/core/process.h>

#if defined(__EMSCRIPTEN__)
// No processes to find or start.
#elif defined(_WIN32)
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#else
#include <climits>
#include <spawn.h>
#include <sys/types.h>
#include <unistd.h>
#if defined(__APPLE__)
#include <cstdint>
#include <mach-o/dyld.h>
#elif defined(__FreeBSD__)
#include <sys/sysctl.h>
#endif
extern char **environ;
#endif

namespace cvc {
namespace process {

namespace fs = std::filesystem;

fs::path self_path() {
#if defined(__EMSCRIPTEN__)
  return {};
#elif defined(_WIN32)
  std::wstring buf(32768, L'\0');
  const DWORD n = GetModuleFileNameW(nullptr, buf.data(), static_cast<DWORD>(buf.size()));
  if (n == 0 || n >= buf.size())
    return {};
  buf.resize(n);
  return fs::path(buf);
#elif defined(__APPLE__)
  char buf[PATH_MAX];
  uint32_t size = sizeof buf;
  if (_NSGetExecutablePath(buf, &size) != 0)
    return {};
  std::error_code ec;
  const fs::path p = fs::canonical(buf, ec);
  return ec ? fs::path(buf) : p;
#elif defined(__FreeBSD__)
  int mib[4] = {CTL_KERN, KERN_PROC, KERN_PROC_PATHNAME, -1};
  char buf[PATH_MAX];
  size_t size = sizeof buf;
  if (sysctl(mib, 4, buf, &size, nullptr, 0) != 0)
    return {};
  return fs::path(buf);
#else
  std::error_code ec;
  for (const char *link : {"/proc/self/exe", "/proc/curproc/exe"}) {
    const fs::path p = fs::read_symlink(link, ec);
    if (!ec)
      return p;
  }
  return {};
#endif
}

bool spawn_detached(const std::vector<std::string> &argv) {
  if (argv.empty())
    return false;
#if defined(__EMSCRIPTEN__)
  return false;
#elif defined(_WIN32)
  // One command line, each argument quoted (the demos' paths have no quotes of their own).
  std::wstring line;
  for (const std::string &arg : argv) {
    if (!line.empty())
      line += L' ';
    line += L'"' + fs::path(arg).wstring() + L'"';
  }
  STARTUPINFOW si{};
  si.cb = sizeof si;
  PROCESS_INFORMATION pi{};
  if (!CreateProcessW(nullptr, line.data(), nullptr, nullptr, FALSE, 0, nullptr, nullptr, &si,
                      &pi))
    return false;
  CloseHandle(pi.hThread);
  CloseHandle(pi.hProcess);
  return true;
#else
  std::vector<char *> args;
  for (const std::string &arg : argv)
    args.push_back(const_cast<char *>(arg.c_str()));
  args.push_back(nullptr);
  pid_t pid;
  return posix_spawn(&pid, args[0], nullptr, nullptr, args.data(), environ) == 0;
#endif
}

} // namespace process
} // namespace cvc
