cmake_minimum_required(VERSION 3.15)

# Build options
option(CVC_USING_XMLRPC "Enable cvc::state network sharing with XMLRPC" OFF)

add_subdirectory(cvc)

if(CVC_USING_XMLRPC)
  add_subdirectory(xmlrpc)
endif()

# Note: CUDA is now integrated via native CMake support (3.17+)
# No special clean targets needed - handled automatically
