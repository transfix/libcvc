cmake_minimum_required(VERSION 3.15)

# Build options
option(CVC_USING_XMLRPC "Enable cvc::state network sharing with XMLRPC" OFF)

add_subdirectory(cvc)
add_subdirectory(cvc-cli)

if(CVC_USING_XMLRPC)
  add_subdirectory(xmlrpc)
endif()

# Note: CUDA is now integrated via native CMake support (3.17+)
# No special clean targets needed - handled automatically

# Note: the VolRover3 application moved to the volrover repository
# (https://github.com/transfix/volrover), where it consumes libcvc as
# an external SDK via find_package(cvc CONFIG).
