// CPU-only fallbacks for the cvc::volren CUDA entry points.
//
// This TU is compiled UNCONDITIONALLY so that raycaster.cpp can call
// raycast_cuda_available() without any #ifdef at the call site; the guard is
// on the inside, and raycast.cu supplies the real definitions when there is a
// CUDA build.
#include <cvc/volren/raycaster_cuda.h>

namespace cvc {
namespace volren {

// CVC_USING_CUDA, not CVC_ENABLE_CUDA: the latter is a CMake variable, and the
// PUBLIC compile define the cvc target actually gets is CVC_USING_CUDA
// (src/cvc/CMakeLists.txt). Guarding on the wrong name meant BOTH this TU and
// the .cu defined the symbol, which links fine on macOS (no CUDA, so the .cu is
// never compiled) and fails on every platform that has it -- see the same note
// at src/cvc/nav/drive.cpp:156.
#ifndef CVC_USING_CUDA

bool raycast_cuda_available() { return false; }

frame raycast_cuda(const raycast_cuda_request &) {
  throw volren_error("cvc::volren was built without CUDA support (no raycast_cuda backend)");
}

// The cache controls are no-ops rather than errors: raycaster::invalidate_
// device_volume() and application teardown call them unconditionally, and a
// CPU-only build simply has nothing resident to invalidate.
void raycast_cuda_set_cache_budget(std::size_t) {}
std::size_t raycast_cuda_cache_budget() { return 0; }
std::size_t raycast_cuda_cache_bytes() { return 0; }
std::uint64_t raycast_cuda_cache_upload_bytes() { return 0; }
void raycast_cuda_clear_cache() {}

#endif // !CVC_USING_CUDA

} // namespace volren
} // namespace cvc
