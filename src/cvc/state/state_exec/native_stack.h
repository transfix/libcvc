// Private: how much native stack the calling thread has left, for detail::native_nesting.
#ifndef CVC_STATE_EXEC_NATIVE_STACK_H
#define CVC_STATE_EXEC_NATIVE_STACK_H

#include <cstddef>
#include <cstdint>

namespace cvc::state_exec {
class exec_context;
}

namespace cvc::state_exec::detail {

/// The lowest address of the calling thread's stack, or 0 when the platform does not say. Asks
/// the OS (on Linux's main thread pthread_getattr_np reads /proc): exec_context::stack_low()
/// keeps what it says for the thread a context runs on.
std::uintptr_t this_thread_stack_low();

/// Bytes of native stack left below the caller on the thread running `ctx` (stacks grow down on
/// every platform libcvc builds for), or SIZE_MAX when the platform does not say where its stack
/// ends.
std::size_t native_stack_left(exec_context &ctx);

} // namespace cvc::state_exec::detail

#endif // CVC_STATE_EXEC_NATIVE_STACK_H
