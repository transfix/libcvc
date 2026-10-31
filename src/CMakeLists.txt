cmake_minimum_required(VERSION 3.15)

# Build options
option(CVC_USING_XMLRPC "Enable cvc::state network sharing with XMLRPC" OFF)

add_subdirectory(cvc)
if(CVC_BUILD_CLI)
  add_subdirectory(cvc-cli)
endif()
# cvcMedia (src/cvcMedia): the GL-free media library, SDL3 audio output today. Its own package, like
# cvcGL, so the libcvc SDK itself never links SDL: built here only for what uses it, cvcGL (which
# links it) and ari-media (ari with sound, CVC_BUILD_ARI_MEDIA).
option(CVC_BUILD_ARI_MEDIA "Build ari-media: ari with cvcMedia (sound through SDL3's audio, no GL)" OFF)
if((CVC_BUILD_CVCGL OR CVC_BUILD_ARI_MEDIA) AND NOT TARGET cvcMedia)
  add_subdirectory(cvcMedia)
endif()
if(CVC_BUILD_ARI)
  add_subdirectory(ari)
endif()
# The conference relay needs a transport (gRPC for native clients, WebSocket for browser clients).
if(CVC_BUILD_ARISRV AND (CVC_ENABLE_GRPC OR CVC_ENABLE_WS))
  add_subdirectory(arisrv)
endif()

if(CVC_USING_XMLRPC)
  add_subdirectory(xmlrpc)
endif()

# Note: CUDA is now integrated via native CMake support (3.17+)
# No special clean targets needed - handled automatically

# Note: the VolRover3 application moved to the volrover repository
# (https://github.com/transfix/volrover), where it consumes libcvc as
# an external SDK via find_package(cvc CONFIG).
