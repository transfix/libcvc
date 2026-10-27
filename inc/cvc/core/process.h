#ifndef CVC_CORE_PROCESS_H
#define CVC_CORE_PROCESS_H

// cvc::process — find the running program and start another one beside it: what a demo gallery's
// launcher needs (arigl's Demos menu, the examples' C++ demos). Native only: a WebAssembly build
// has no other program to start, so there self_path() is empty and spawn_detached() fails.

#include <filesystem>
#include <string>
#include <vector>

namespace cvc {
namespace process {

// The running program's own file: GetModuleFileNameW, _NSGetExecutablePath, the FreeBSD sysctl, or
// /proc/self/exe (Linux) and /proc/curproc/exe (NetBSD). Empty where the platform can't say.
std::filesystem::path self_path();

// Start argv[0] with the rest of argv as its arguments, and don't wait for it. argv[0] is a path
// (not looked up on PATH). True once the program is running.
bool spawn_detached(const std::vector<std::string> &argv);

} // namespace process
} // namespace cvc

#endif // CVC_CORE_PROCESS_H
