// Private: how much native stack the calling thread has left, for detail::native_nesting.
#ifndef CVC_STATE_EXEC_NATIVE_STACK_H
#define CVC_STATE_EXEC_NATIVE_STACK_H

#include <cstddef>

namespace cvc::state_exec::detail {

/// Bytes of native stack left below the caller on this thread (stacks grow down on every
/// platform libcvc builds for), or SIZE_MAX when the platform does not say where its stack ends.
std::size_t native_stack_left();

} // namespace cvc::state_exec::detail

#endif // CVC_STATE_EXEC_NATIVE_STACK_H
