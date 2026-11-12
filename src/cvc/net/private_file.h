// cvc::net — files that hold a user's secrets (the credential store, the E2EE store): where they
// go, how they are written, and the check made before one is read. Private to src/cvc/net.
#ifndef CVC_NET_PRIVATE_FILE_H
#define CVC_NET_PRIVATE_FILE_H

#include <filesystem>
#include <string>

namespace cvc {
namespace net {
namespace private_file {

// The user's config directory for cvc, as device keys find it (state_device_key.cpp):
// %LOCALAPPDATA%\cvc on Windows, else $XDG_CONFIG_HOME/cvc or ~/.config/cvc. Empty when none can
// be named, and always in a browser.
std::filesystem::path config_dir();

// Write `text` to `path`, replacing it in one step, owner-only: a temporary file beside it
// (0600 on POSIX, synced) renamed over it, and a missing directory made 0700. `what` names the
// file in an error ("the credential store").
bool write(const std::filesystem::path &path, const std::string &text, const std::string &what,
           std::string *error);

// Before reading `path` (which exists): on POSIX it must be the user's own and open to nobody
// else. Always true on Windows, where it lives in the user's profile.
bool check_owner_only(const std::filesystem::path &path, const std::string &what,
                      std::string *error);

} // namespace private_file
} // namespace net
} // namespace cvc

#endif // CVC_NET_PRIVATE_FILE_H
