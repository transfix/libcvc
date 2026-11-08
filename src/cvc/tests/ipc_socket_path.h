// A Unix-domain socket path for an IPC test. It goes under the temp directory, or under /tmp when
// that path would not fit in sockaddr_un::sun_path (104 bytes on macOS, 108 on Linux), which
// state_transport_ipc::start refuses ("socket path too long"). cvcpkg's builder points TMPDIR at
// a per-job directory, deep under /var/folders on macOS, where a test's socket name alone no
// longer fits.
#ifndef CVC_TESTS_IPC_SOCKET_PATH_H
#define CVC_TESTS_IPC_SOCKET_PATH_H

#include <filesystem>
#include <string>
#ifndef _WIN32
#include <sys/un.h>
#endif

namespace cvc_test {

inline std::string ipc_socket_path(const std::string &name) {
  std::filesystem::path p = std::filesystem::temp_directory_path() / name;
#ifndef _WIN32
  if (p.string().size() >= sizeof(sockaddr_un{}.sun_path))
    p = std::filesystem::path("/tmp") / name;
#endif
  return p.string();
}

} // namespace cvc_test

#endif // CVC_TESTS_IPC_SOCKET_PATH_H
