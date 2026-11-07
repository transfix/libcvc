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
  # Standalone (the `arisrv` cvcpkg recipe builds ONLY this dir): the installed cvc SDK pulls its
  # Boost dependency in via find_dependency, so the Boost imported targets exist in this scope.
  project(arisrv LANGUAGES CXX)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  find_package(cvc CONFIG REQUIRED)
  # The relay needs a transport, as in-tree (src/CMakeLists.txt adds this directory only with
  # one). Here the SDK decides: CVC_ENABLE_GRPC and CVC_ENABLE_WS are PUBLIC definitions on cvc,
  # so they are in its exported interface when that libcvc was built with them. Every libcvc
  # through 3.7.8 was published without either (#943): a relay built on one of those has no
  # transport for a client to join by.
  get_target_property(_arisrv_cvc_defs cvc::cvc INTERFACE_COMPILE_DEFINITIONS)
  set(_arisrv_transports "")
  if("CVC_ENABLE_GRPC" IN_LIST _arisrv_cvc_defs)
    list(APPEND _arisrv_transports gRPC)
  endif()
  if("CVC_ENABLE_WS" IN_LIST _arisrv_cvc_defs)
    list(APPEND _arisrv_transports WebSocket)
  endif()
  if(NOT _arisrv_transports)
    message(FATAL_ERROR "arisrv: the libcvc SDK in ${cvc_DIR} was built without CVC_ENABLE_GRPC "
                        "and CVC_ENABLE_WS, so a relay built on it could serve no client")
  endif()
  string(REPLACE ";" " and " _arisrv_transports "${_arisrv_transports}")
  message(STATUS "arisrv: the libcvc SDK carries the ${_arisrv_transports} transports")
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

# ctest, in-tree: a server document on the relay (--doc, docs/roadmap/ARIADNE_SCRIPTING_HOSTS.md
# §6.5.3), on loopback ports no client joins. tests/echo_server.ari publishes echo and stops the
# relay with (quit 0) after five room steps; (quit 3) is arisrv's exit status; a document that
# would publish the room tree, or publish without bound writes, stops arisrv before it listens.
# What clients receive is RoomRelayLoopbackTest.ServerDocumentPublishesItsNamespaceOnly
# (src/cvc/tests/room_relay_integration_test.cpp). A libcvc without yaml-cpp skips.
if(CVC_BUILD_TESTS AND NOT CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
  set(_srv_tests "${CMAKE_CURRENT_SOURCE_DIR}/tests")
  set(_srv_listen "--room@@ctest@@--state-listen@@127.0.0.1:0@@--media-listen@@127.0.0.1:0@@--ws-listen@@127.0.0.1:0")
  add_test(NAME arisrv_doc_echo
           COMMAND ${CMAKE_COMMAND} -DARISRV=$<TARGET_FILE:arisrv>
                   "-DARGS=--doc@@${_srv_tests}/echo_server.ari@@${_srv_listen}" -DSTATUS=0
                   "-DEXPECT1=binds under ui\\.docs\\.arisrv[^\n]*publishes echo \\(to every client\\)"
                   "-DEXPECT2=the server document asked to stop \\(quit 0\\)"
                   -P "${_srv_tests}/run_arisrv.cmake")
  add_test(NAME arisrv_doc_quit_code
           COMMAND ${CMAKE_COMMAND} -DARISRV=$<TARGET_FILE:arisrv>
                   "-DARGS=--doc@@${_srv_tests}/quit_code.ari@@${_srv_listen}" -DSTATUS=3
                   "-DEXPECT1=asked to stop \\(quit 3\\)" -P "${_srv_tests}/run_arisrv.cmake")
  add_test(NAME arisrv_doc_publish_conf
           COMMAND ${CMAKE_COMMAND} -DARISRV=$<TARGET_FILE:arisrv>
                   "-DARGS=--doc@@${_srv_tests}/publish_conf.ari@@${_srv_listen}" -DSTATUS=2
                   "-DEXPECT1=server: publish: 'conf' is under conf, the relay's room tree"
                   -P "${_srv_tests}/run_arisrv.cmake")
  add_test(NAME arisrv_doc_needs_bound_writes
           COMMAND ${CMAKE_COMMAND} -DARISRV=$<TARGET_FILE:arisrv>
                   "-DARGS=--doc@@${_srv_tests}/echo_server.ari@@--bind-writes@@off@@${_srv_listen}"
                   -DSTATUS=2 "-DEXPECT1=publishes namespaces, which needs bound writes"
                   -P "${_srv_tests}/run_arisrv.cmake")
  set_tests_properties(arisrv_doc_echo arisrv_doc_quit_code arisrv_doc_publish_conf
                       arisrv_doc_needs_bound_writes PROPERTIES
                       SKIP_REGULAR_EXPRESSION "built without yaml-cpp" TIMEOUT 90)
  # A host that is not the relay ignores the server: block, and says so.
  if(TARGET ari)
    add_test(NAME ari_server_block_ignored COMMAND ari check "${_srv_tests}/echo_server.ari")
    set_tests_properties(ari_server_block_ignored PROPERTIES
                         PASS_REGULAR_EXPRESSION "server: block is for the relay that runs it"
                         SKIP_REGULAR_EXPRESSION "built without FTXUI;built without yaml-cpp")
  endif()
endif()
