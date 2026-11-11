include_guard(GLOBAL)

# cvc_document_host_stack(<target>...): give each executable an 8 MB main-thread stack on Windows,
# as Linux gives every process. Loading a document and drawing it recurse once per level of its
# widget tree (each include:, each group), and at Windows' default 1 MB a document nested about 15
# levels deep through include: with args overflowed it: the host died with 0xC00000FD and no
# message. A thread made with no stack size of its own (std::thread, boost::thread's default) gets
# the executable's, so the threads that load or draw a document get it too. It is address space
# reserved, committed only as it is used. Elsewhere it does nothing: Linux's main thread already
# has 8 MB (ulimit -s), and the browser builds set their own (-sSTACK_SIZE).
set(CVC_DOCUMENT_HOST_STACK 8388608 CACHE STRING
    "Main-thread stack, in bytes, of the executables that load Ariadne documents (Windows)")

function(cvc_document_host_stack)
  foreach(_target IN LISTS ARGN)
    if(NOT TARGET ${_target})
      continue()
    endif()
    if(MSVC)
      target_link_options(${_target} PRIVATE "/STACK:${CVC_DOCUMENT_HOST_STACK}")
    elseif(MINGW)
      target_link_options(${_target} PRIVATE "-Wl,--stack,${CVC_DOCUMENT_HOST_STACK}")
    endif()
  endforeach()
endfunction()
