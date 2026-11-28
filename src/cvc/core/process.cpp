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
#elif defined(__FreeBSD__) || defined(__NetBSD__)
#include <sys/sysctl.h>
#elif defined(__OpenBSD__)
#include <algorithm>
#include <cstdlib>
#include <string>
#include <sys/sysctl.h>
#include <vector>
#elif defined(__HAIKU__)
#include <image.h>
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
#elif defined(__NetBSD__)
  // kern.proc_args.<self>.pathname; procfs (/proc/curproc/exe) is not always mounted.
  int mib[4] = {CTL_KERN, KERN_PROC_ARGS, -1, KERN_PROC_PATHNAME};
  char buf[PATH_MAX];
  size_t size = sizeof buf;
  if (sysctl(mib, 4, buf, &size, nullptr, 0) == 0 && size > 1)
    return fs::path(buf);
  std::error_code ec;
  const fs::path p = fs::read_symlink("/proc/curproc/exe", ec);
  return ec ? fs::path() : p;
#elif defined(__OpenBSD__)
  // OpenBSD has no procfs and no sysctl for a process's file, only its argv: argv[0] as the
  // program was started, which is a path (absolute, or relative to the directory it started in)
  // or a name looked up on PATH. Resolved the way the shell did, as long as nothing moved since.
  int mib[4] = {CTL_KERN, KERN_PROC_ARGS, static_cast<int>(getpid()), KERN_PROC_ARGV};
  size_t size = 0;
  if (sysctl(mib, 4, nullptr, &size, nullptr, 0) != 0 || size == 0)
    return {};
  std::vector<char> buf(size);
  if (sysctl(mib, 4, buf.data(), &size, nullptr, 0) != 0)
    return {};
  char **argv = reinterpret_cast<char **>(buf.data());
  if (argv[0] == nullptr || argv[0][0] == '\0')
    return {};
  const std::string arg0 = argv[0];
  std::error_code ec;
  if (arg0.find('/') != std::string::npos) {
    const fs::path p = fs::canonical(arg0, ec);
    return ec ? fs::path() : p;
  }
  const char *path_env = std::getenv("PATH");
  std::string dirs = path_env != nullptr ? path_env : "/usr/bin:/bin:/usr/sbin:/sbin";
  size_t start = 0;
  while (start <= dirs.size()) {
    const size_t end = std::min(dirs.find(':', start), dirs.size());
    const std::string dir = dirs.substr(start, end - start);
    const fs::path candidate = fs::path(dir.empty() ? "." : dir) / arg0;
    if (access(candidate.c_str(), X_OK) == 0) {
      const fs::path p = fs::canonical(candidate, ec);
      if (!ec)
        return p;
    }
    start = end + 1;
  }
  return {};
#elif defined(__HAIKU__)
  image_info info;
  int32 cookie = 0;
  while (get_next_image_info(B_CURRENT_TEAM, &cookie, &info) == B_OK) {
    if (info.type == B_APP_IMAGE)
      return fs::path(info.name);
  }
  return {};
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
  if (!CreateProcessW(nullptr, line.data(), nullptr, nullptr, FALSE, 0, nullptr, nullptr, &si, &pi))
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
