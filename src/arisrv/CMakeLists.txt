cmake_minimum_required(VERSION 3.15)

# ---------------------------------------------------------------------------
# `arisrv` — the headless Ariadne conference RELAY (room-state relay + media SFU + WS<->gRPC
# bridge). Clients connect ONLY to this server, not to each other, so a conference works across
# home NAT. Core libcvc only (NO cvcGL/VTK/GL), so it runs on a plain headless VM. It needs a
# transport: gRPC for native clients and/or WebSocket for browser (wasm) clients — the parent
# only adds this directory when CVC_ENABLE_GRPC or CVC_ENABLE_WS is on.
#
#   In-tree    (add_subdirectory(arisrv) from the libcvc build): the cvc target already exists.
#   Standalone (a cvcpkg recipe builds ONLY this dir): find the installed libcvc SDK via
#              find_package(cvc CONFIG); the recipe points CMAKE_PREFIX_PATH at the deps prefix.
# ---------------------------------------------------------------------------
if(CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
  # Standalone (a cvcpkg recipe builds ONLY this dir): the installed cvc SDK pulls its Boost
  # dependency in via find_dependency, so the Boost imported targets exist in this scope.
  project(arisrv LANGUAGES CXX)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  find_package(cvc CONFIG REQUIRED)
else()
  # In-tree: cvc links Boost PUBLIC (CMake/SetupBoost.cmake), so cvc's INTERFACE_LINK_LIBRARIES
  # references the Boost:: imported targets. find_package created those in src/cvc's directory
  # scope, which does NOT reach this sibling directory, and SetupBoost does not promote them to
  # GLOBAL (that find_package keyword needs CMake 3.24+; the glibc-floor relay builder has 3.22).
  # Re-import the same components here so cvc's transitive link evaluates. find_package is
  # idempotent and resolves to the same deps prefix. (gRPC/protobuf are PRIVATE on cvc, so the
  # relay never needs them re-imported.)
  if(NOT TARGET Boost::filesystem)
    find_package(Boost REQUIRED COMPONENTS thread date_time regex filesystem program_options
                 OPTIONAL_COMPONENTS system chrono)
  endif()
endif()

add_executable(arisrv main.cpp)
target_compile_features(arisrv PRIVATE cxx_std_17)
if(TARGET cvc::cvc)
  target_link_libraries(arisrv PRIVATE cvc::cvc)
else()
  target_link_libraries(arisrv PRIVATE cvc)
endif()
# Installed bin/arisrv finds libcvc and its dependencies in ../lib (CMake/CvcInstallRpath.cmake).
include("${CMAKE_CURRENT_LIST_DIR}/../../CMake/CvcInstallRpath.cmake")
cvc_set_install_rpath(arisrv bin)
install(TARGETS arisrv RUNTIME DESTINATION bin COMPONENT arisrv)

# `arisrv_aoi_bench`: a headless load bench for the relay's area of interest — a relay built as
# arisrv builds it plus N synthetic clients in one process on loopback, reporting what each
# receiver got by kind, per-link rates, the journal, a late joiner's bootstrap and the probes. A
# development tool, built in-tree only (never by the standalone relay recipe) and not installed. It
# runs on whichever transports the build has (the parent adds this directory only with one).
if(NOT CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR AND (CVC_ENABLE_GRPC OR CVC_ENABLE_WS))
  add_executable(arisrv_aoi_bench aoi_bench.cpp)
  target_compile_features(arisrv_aoi_bench PRIVATE cxx_std_17)
  target_link_libraries(arisrv_aoi_bench PRIVATE cvc)
endif()
