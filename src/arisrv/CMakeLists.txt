cmake_minimum_required(VERSION 3.15)

# ---------------------------------------------------------------------------
# `arisrv` — the headless Ariadne conference RELAY (room-state relay + media SFU + WS<->gRPC
# bridge). Clients connect ONLY to this server, not to each other, so a conference works across
# home NAT. Core libcvc only (NO cvcGL/VTK/GL), so it runs on a plain headless VM. Its room-tree
# and media planes are gRPC, so it needs the gRPC transport; WebSocket (for browser, wasm, clients)
# is added on top when it is there, bridged onto them. The parent only adds this directory when
# CVC_ENABLE_GRPC is on. gRPC comes from cvcpkg: cvcpkg install grpc protobuf.
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
  # The relay needs the gRPC transport, as in-tree (src/CMakeLists.txt adds this directory only
  # with it): its planes are gRPC, and a WebSocket-only libcvc does not link it. Here the SDK
  # decides: CVC_ENABLE_GRPC and CVC_ENABLE_WS are PUBLIC definitions on cvc, so they are in its
  # exported interface when that libcvc was built with them. Every libcvc through 3.7.8 was
  # published without either (#943); every one since has both.
  get_target_property(_arisrv_cvc_defs cvc::cvc INTERFACE_COMPILE_DEFINITIONS)
  if(NOT "CVC_ENABLE_GRPC" IN_LIST _arisrv_cvc_defs)
    message(FATAL_ERROR "arisrv: the libcvc SDK in ${cvc_DIR} was built without CVC_ENABLE_GRPC, "
                        "and the relay's planes are gRPC. Build it against a libcvc with gRPC "
                        "(cvcpkg's has it; for your own, cvcpkg install grpc protobuf and "
                        "-DCVC_ENABLE_GRPC=ON)")
  endif()
  set(_arisrv_transports gRPC)
  if("CVC_ENABLE_WS" IN_LIST _arisrv_cvc_defs)
    list(APPEND _arisrv_transports WebSocket)
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
# A server document's load: an 8 MB stack on Windows, as on Linux (CMake/CvcHostStack.cmake).
include("${CMAKE_CURRENT_LIST_DIR}/../../CMake/CvcHostStack.cmake")
cvc_document_host_stack(arisrv)
install(TARGETS arisrv RUNTIME DESTINATION bin COMPONENT arisrv)

# arisrvx: the same relay with an embedded Python (src/ariadne_python, its SERVER part): the
# operator's program beside the relay on a script thread with an asyncio loop, ASGI apps served on
# it, and pycvc.ariadne built in (docs/roadmap/ARIADNE_SCRIPTING_HOSTS.md §6.5). Its Python needs no
# pycvc, so in-tree it is built with the bindings (CVC_BUILD_PYCVC) or on its own with
# -DARISRV_WITH_PYTHON=ON, as standalone (the arisrvx cvcpkg recipe). The stock arisrv above stays
# Python-free.
option(ARISRV_WITH_PYTHON "Also build arisrvx, arisrv with an embedded Python" OFF)
if(NOT EMSCRIPTEN AND (CVC_BUILD_PYCVC OR ARISRV_WITH_PYTHON))
  include("${CMAKE_CURRENT_LIST_DIR}/../ariadne_python/ariadne_python.cmake")
  cvc_find_ariadne_python(ARISRV_WITH_PYTHON)
  if(CVC_ARIADNE_PYTHON_FOUND)
    add_executable(arisrvx main.cpp)
    target_compile_features(arisrvx PRIVATE cxx_std_17)
    if(TARGET cvc::cvc)
      target_link_libraries(arisrvx PRIVATE cvc::cvc)
    else()
      target_link_libraries(arisrvx PRIVATE cvc)
    endif()
    cvc_ariadne_python_host(arisrvx SERVER)
    cvc_set_install_rpath(arisrvx bin)
    install(TARGETS arisrvx RUNTIME DESTINATION bin COMPONENT arisrvx)
  else()
    message(STATUS "arisrvx: not built (no Python embedding files: Python3 Development.Embed)")
  endif()
endif()

# `arisrv_aoi_bench`: a headless load bench for the relay's area of interest — a relay built as
# arisrv builds it plus N synthetic clients in one process on loopback, reporting what each
# receiver got by kind, per-link rates, the journal, a late joiner's bootstrap and the probes. A
# development tool, built in-tree only (never by the standalone relay recipe) and not installed. It
# runs on the transports the build has: gRPC (the parent adds this directory only with it), and
# WebSocket when that is on too.
if(NOT CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
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
  # The room's world (ARIADNE_WORLDS.md WD1): a --doc with a scene: is it, published before init:
  # runs; --scene and such a --doc both naming it stop arisrv; and --set comes after the stock
  # world (WD0), which the relay finds as cvc://worlds/default.ari in this source tree.
  add_test(NAME arisrv_world_from_doc
           COMMAND ${CMAKE_COMMAND} -DARISRV=$<TARGET_FILE:arisrv>
                   "-DARGS=--doc@@${_srv_tests}/world_doc.ari@@${_srv_listen}" -DSTATUS=0
                   "-DEXPECT1=world: [^\n]*world_doc\\.ari for room 'ctest': 2 objects, 1 light, shadows on"
                   -P "${_srv_tests}/run_arisrv.cmake")
  add_test(NAME arisrv_world_doc_and_scene
           COMMAND ${CMAKE_COMMAND} -DARISRV=$<TARGET_FILE:arisrv>
                   "-DARGS=--doc@@${_srv_tests}/world_doc.ari@@--scene@@large@@${_srv_listen}"
                   -DSTATUS=2 "-DEXPECT1=both name the room's world"
                   -P "${_srv_tests}/run_arisrv.cmake")
  add_test(NAME arisrv_set_after_world
           COMMAND ${CMAKE_COMMAND} -DARISRV=$<TARGET_FILE:arisrv>
                   "-DARGS=--doc@@${_srv_tests}/set_after_world.ari@@--set@@conf.rooms.ctest.scene.rev=7@@--set@@conf.rooms.ctest.scene.objects.ref.transform.pos.z=2@@${_srv_listen}"
                   -DSTATUS=0
                   "-DEXPECT1=world: cvc://worlds/default\\.ari for room 'ctest': 2 objects, 1 light, shadows on"
                   "-DEXPECT2=rules: cvc://worlds/default\\.ari: avatars are person \\(70 kg, 0\\.50 x 0\\.50 x 1\\.70 m\\) on walker@1, at most 4\\.0 m/s, gravity 9\\.81 m/s\\^2; energy on; an empty reserve rests"
                   -P "${_srv_tests}/run_arisrv.cmake")
  set_tests_properties(arisrv_set_after_world PROPERTIES
                       ENVIRONMENT "CVC_ARIADNE_PATH=${CMAKE_CURRENT_SOURCE_DIR}/../cvc/ariadne")
  # The world's rules (WD2): a --doc's world: block sets them, published before init: runs; the
  # energy flags override its policy; a kernel this build does not run stops arisrv.
  add_test(NAME arisrv_rules_from_doc
           COMMAND ${CMAKE_COMMAND} -DARISRV=$<TARGET_FILE:arisrv>
                   "-DARGS=--doc@@${_srv_tests}/world_rules.ari@@${_srv_listen}" -DSTATUS=0
                   "-DEXPECT1=rules: [^\n]*world_rules\\.ari: avatars are pilot \\(90 kg, 0\\.60 x 0\\.60 x 1\\.90 m\\) on walker@1, at most 6\\.0 m/s, gravity 9\\.81 m/s\\^2; energy on; an empty reserve stops"
                   -P "${_srv_tests}/run_arisrv.cmake")
  add_test(NAME arisrv_rules_flags_override
           COMMAND ${CMAKE_COMMAND} -DARISRV=$<TARGET_FILE:arisrv>
                   "-DARGS=--doc@@${_srv_tests}/world_rules.ari@@--avatar-energy@@off@@${_srv_listen}"
                   -DSTATUS=0 "-DEXPECT1=rules: [^\n]*world_rules\\.ari: [^\n]*; energy off"
                   -P "${_srv_tests}/run_arisrv.cmake")
  add_test(NAME arisrv_rules_unknown_kernel
           COMMAND ${CMAKE_COMMAND} -DARISRV=$<TARGET_FILE:arisrv>
                   "-DARGS=--doc@@${_srv_tests}/world_rules_bad.ari@@${_srv_listen}" -DSTATUS=2
                   "-DEXPECT1=this build has no kernel runner@1"
                   -P "${_srv_tests}/run_arisrv.cmake")
  set_tests_properties(arisrv_rules_from_doc arisrv_rules_flags_override PROPERTIES
                       ENVIRONMENT "CVC_ARIADNE_PATH=${CMAKE_CURRENT_SOURCE_DIR}/../cvc/ariadne")
  set_tests_properties(arisrv_doc_echo arisrv_doc_quit_code arisrv_doc_publish_conf
                       arisrv_doc_needs_bound_writes arisrv_world_from_doc arisrv_world_doc_and_scene
                       arisrv_set_after_world arisrv_rules_from_doc arisrv_rules_flags_override
                       arisrv_rules_unknown_kernel PROPERTIES
                       SKIP_REGULAR_EXPRESSION "built without yaml-cpp" TIMEOUT 90)
  # A host that is not the relay ignores a server: block, and says so; a document that requires
  # the relay does not run there, and the error says where it does.
  if(TARGET ari)
    add_test(NAME ari_server_block_ignored COMMAND ari check "${_srv_tests}/server_block.ari")
    set_tests_properties(ari_server_block_ignored PROPERTIES
                         PASS_REGULAR_EXPRESSION "server: block is for the relay that runs it")
    add_test(NAME ari_server_document_needs_relay COMMAND ari check "${_srv_tests}/echo_server.ari")
    set_tests_properties(ari_server_document_needs_relay PROPERTIES
                         PASS_REGULAR_EXPRESSION "a server document runs on the relay: arisrv --doc")
    set_tests_properties(ari_server_block_ignored ari_server_document_needs_relay PROPERTIES
                         SKIP_REGULAR_EXPRESSION "built without FTXUI;built without yaml-cpp")
  endif()

  # The ServerLink (src/ariadne_python/server_link.h), with no Python: the queue's coalescing,
  # order and budget, its subtree writes against a real tree, the events and the startup states.
  add_executable(arisrvx_server_link_test tests/server_link_test.cpp
                                          ../ariadne_python/server_link.cpp)
  target_include_directories(arisrvx_server_link_test PRIVATE ../ariadne_python)
  target_link_libraries(arisrvx_server_link_test PRIVATE cvc GTest::gtest GTest::gtest_main)
  include(GoogleTest)
  gtest_discover_tests(arisrvx_server_link_test DISCOVERY_TIMEOUT 120)
  # Stock arisrv refuses the Python flags, and names the host that takes them.
  add_test(NAME arisrv_python_needs_arisrvx
           COMMAND ${CMAKE_COMMAND} -DARISRV=$<TARGET_FILE:arisrv>
                   "-DARGS=--python@@main.py@@${_srv_listen}" -DSTATUS=2
                   "-DEXPECT1=--python is for arisrvx" -P "${_srv_tests}/run_arisrv.cmake")

  # arisrvx end to end, on loopback ports no client joins, with the Python it was built against:
  # tests/service.py serves a FastAPI app beside tests/service_server.ari, drives itself (requests,
  # writes the relay applies and the document reads back, a stop raised in-process as Ctrl-C would
  # be, a request in flight that finishes first), and exits 0; tests/program.py writes without an
  # ASGI app (it needs nothing installed) and stops the relay itself; --http-listen serves a program's
  # `app`; a lifespan startup that fails stops arisrvx before the relay listens; check runs a
  # program without serving and reports a missing module; a server document's python: block runs
  # on the script thread with no program, its exports called from the document's programs, typed
  # values both ways (S2), and check runs it too. The FastAPI ones skip where fastapi or uvicorn is
  # not installed, the python: block ones where pycvc is not built.
  if(TARGET arisrvx)
    set(_x "-DARISRV=$<TARGET_FILE:arisrvx>")
    set(_svc "--python@@${_srv_tests}/service.py@@--doc@@${_srv_tests}/service_server.ari")
    add_test(NAME arisrvx_serve
             COMMAND ${CMAKE_COMMAND} ${_x}
                     "-DARGS=${_svc}@@--script-write-budget@@256@@${_srv_listen}" -DSTATUS=0
                     "-DEXPECT1=selftest: the server document read the 40 items"
                     "-DEXPECT2=service: /slow finished.*service: shutdown"
                     "-DEXPECT3=python program ended \\(status 0\\): [0-9]+ write\\(s\\) applied"
                     "-DREJECT=selftest: FAILED" -P "${_srv_tests}/run_arisrv.cmake")
    add_test(NAME arisrvx_program
             COMMAND ${CMAKE_COMMAND} ${_x}
                     "-DARGS=--python@@${_srv_tests}/program.py@@${_srv_listen}" -DSTATUS=0
                     "-DEXPECT1=program: the relay applied \\{'state': 'done', 'steps': \\['1', '2', '3'\\]\\}"
                     "-DEXPECT2=python program stopped \\(status 0\\)"
                     -P "${_srv_tests}/run_arisrv.cmake")
    add_test(NAME arisrvx_http_listen
             COMMAND ${CMAKE_COMMAND} ${_x}
                     "-DARGS=--python@@${_srv_tests}/http_listen.py@@--http-listen@@127.0.0.1:0@@${_srv_listen}"
                     -DSTATUS=0 "-DEXPECT1=selftest: GET /ping 200"
                     "-DEXPECT2=python program stopped \\(status 0\\)"
                     -P "${_srv_tests}/run_arisrv.cmake")
    add_test(NAME arisrvx_startup_fails
             COMMAND ${CMAKE_COMMAND} ${_x}
                     "-DARGS=--python@@${_srv_tests}/startup_fails.py@@${_srv_listen}" -DSTATUS=1
                     "-DEXPECT1=the database is not there \\(on purpose\\)"
                     "-DEXPECT2=did not start: serve_asgi on 127\\.0\\.0\\.1:0: its startup failed"
                     "-DREJECT=relay up" -P "${_srv_tests}/run_arisrv.cmake")
    add_test(NAME arisrvx_check
             COMMAND ${CMAKE_COMMAND} ${_x}
                     "-DARGS=check@@${_srv_tests}/service.py@@--doc@@${_srv_tests}/service_server.ari@@${_srv_listen}"
                     -DSTATUS=0 "-DEXPECT1=service\\.py: ok: 1 ASGI app\\(s\\) to serve"
                     "-DREJECT=relay up|selftest" -P "${_srv_tests}/run_arisrv.cmake")
    add_test(NAME arisrvx_check_missing_module
             COMMAND ${CMAKE_COMMAND} ${_x}
                     "-DARGS=check@@${_srv_tests}/missing_module.py" -DSTATUS=1
                     "-DEXPECT1=No module named 'arisrvx_no_such_module'"
                     -P "${_srv_tests}/run_arisrv.cmake")
    # pycvc from this build, which converts the values (--python-path, as arix's tests name it).
    string(JOIN "@@" _x_path ${CVC_ARIADNE_PYTHON_TEST_PATH})
    add_test(NAME arisrvx_doc_python
             COMMAND ${CMAKE_COMMAND} ${_x}
                     "-DARGS=--doc@@${_srv_tests}/doc_python.ari@@${_x_path}@@${_srv_listen}" -DSTATUS=0
                     "-DEXPECT1=\"doc_python: init score\" 42 \"int\" \"symbol\""
                     "-DEXPECT2=\"doc_python: later\" \"int\" \"float\" 4\\.5 \"float score:\" \"float\""
                     "-DEXPECT3=\"doc_python: caught\" \"PythonError\" \"ok\""
                     -P "${_srv_tests}/run_arisrv.cmake")
    add_test(NAME arisrvx_check_doc_python
             COMMAND ${CMAKE_COMMAND} ${_x}
                     "-DARGS=check@@--doc@@${_srv_tests}/doc_python.ari@@${_x_path}@@${_srv_listen}"
                     -DSTATUS=0
                     "-DEXPECT1=\"doc_python: init score\" 42"
                     "-DEXPECT2=arisrvx check: the server document: ok"
                     "-DREJECT=relay up" -P "${_srv_tests}/run_arisrv.cmake")
    add_test(NAME arisrvx_python_interpreter
             COMMAND arisrvx python -c "import _arihost; print(_arihost.host(), _arihost.attached())")
    set_tests_properties(arisrvx_python_interpreter PROPERTIES PASS_REGULAR_EXPRESSION
                         "arisrvx False")
    set(_x_tests arisrvx_serve arisrvx_program arisrvx_http_listen arisrvx_startup_fails arisrvx_check
                 arisrvx_check_missing_module arisrvx_doc_python arisrvx_check_doc_python
                 arisrvx_python_interpreter)

    # room-call (S2, ARIADNE_SCRIPTING_HOSTS.md §6.5.5, §6.5.7): tests/relay_pair.py starts the relay
    # on loopback ports, reads its port, and runs `ari test --relay URL` with tests/call_client.ari
    # against tests/call_service.py beside tests/call_server.ari. The client checks each call's
    # value and type, keyword arguments, its pid as the principal, idempotency keys, progress in
    # its tree, the role gate, CallError, internal, the deadline, the result cap and the
    # client-only gate, and mirrors the published svc; its last call stops the relay. Over gRPC,
    # gRPC with TLS (tests/tls: a test CA and its localhost certificate) and the WebSocket, the last
    # with arix (an x host joins as ari does). A grpcs:// client without the relay's CA is refused,
    # and stock arisrv answers every call unavailable.
    set(_pair "${Python3_EXECUTABLE}" "${_srv_tests}/relay_pair.py")
    set(_call_relay --relay-arg=--python "--relay-arg=${_srv_tests}/call_service.py"
                    --relay-arg=--doc "--relay-arg=${_srv_tests}/call_server.ari"
                    --relay-arg=--room --relay-arg=ctest --relay-arg=--state-listen
                    --relay-arg=127.0.0.1:0 --relay-arg=--media-listen --relay-arg=127.0.0.1:0
                    --relay-arg=--ws-listen --relay-arg=127.0.0.1:0 --relay-arg=--call-result-max
                    --relay-arg=1024)
    foreach(_a IN LISTS CVC_ARIADNE_PYTHON_TEST_PATH)
      list(APPEND _call_relay "--relay-arg=${_a}")
    endforeach()
    set(_tls --relay-arg=--tls-cert "--relay-arg=${_srv_tests}/tls/localhost.pem"
             --relay-arg=--tls-key "--relay-arg=${_srv_tests}/tls/localhost.key")
    set(_call_expect "--expect=blocking: a sync function ran [0-9]+ ms, over --script-slow-ms 50"
                     "--expect=room-call explode from ari[a-z]*_[a-z0-9]+ raised")
    set(_call_tests)
    if(TARGET ari)
      add_test(NAME arisrvx_room_call_grpc
               COMMAND ${_pair} --relay $<TARGET_FILE:arisrvx> --client $<TARGET_FILE:ari>
                       --scheme grpc ${_call_relay} ${_call_expect} --client-arg=--relay-room
                       --client-arg=ctest "--client-arg=${_srv_tests}/call_client.ari")
      add_test(NAME arisrvx_room_call_grpcs
               COMMAND ${_pair} --relay $<TARGET_FILE:arisrvx> --client $<TARGET_FILE:ari>
                       --scheme grpcs --host localhost ${_call_relay} ${_tls} ${_call_expect}
                       --client-arg=--relay-room --client-arg=ctest --client-arg=--relay-ca
                       "--client-arg=${_srv_tests}/tls/ca.pem"
                       "--client-arg=${_srv_tests}/call_client.ari")
      add_test(NAME arisrvx_room_call_tls_needs_its_ca
               COMMAND ${_pair} --relay $<TARGET_FILE:arisrvx> --client $<TARGET_FILE:ari>
                       --scheme grpcs --host localhost ${_call_relay} ${_tls} --client-only
                       --client-status 1 "--client-expect=relay: could not reach grpcs://"
                       --client-arg=--relay-room --client-arg=ctest
                       "--client-arg=${_srv_tests}/call_client.ari")
      add_test(NAME arisrv_room_call_unavailable
               COMMAND ${_pair} --relay $<TARGET_FILE:arisrv> --client $<TARGET_FILE:ari>
                       --scheme grpc --relay-arg=--room --relay-arg=ctest --relay-arg=--state-listen
                       --relay-arg=127.0.0.1:0 --relay-arg=--media-listen --relay-arg=127.0.0.1:0
                       --relay-arg=--ws-listen --relay-arg=127.0.0.1:0 --client-only
                       --client-arg=--relay-room --client-arg=ctest
                       "--client-arg=${_srv_tests}/call_unavailable.ari")
      list(APPEND _call_tests arisrvx_room_call_grpc arisrvx_room_call_grpcs
           arisrvx_room_call_tls_needs_its_ca arisrv_room_call_unavailable)

      # The authenticate hook (S5, §6.5.4): tests/auth_service.py admits two test tokens as alice
      # (staff) and bob. alice's calls carry her principal, pass the role gate, are limited to 2 in
      # flight and stay unavailable once she is revoked (the hook refuses her, and host.disconnect
      # closes her connection); bob's staff call is forbidden. A wrong token, one the hook stalls on past --auth-timeout and one it raises on
      # are refused at once, unauthenticated.
      set(_auth_relay --relay-arg=--python "--relay-arg=${_srv_tests}/auth_service.py"
                      --relay-arg=--room --relay-arg=ctest --relay-arg=--state-listen
                      --relay-arg=127.0.0.1:0 --relay-arg=--media-listen --relay-arg=127.0.0.1:0
                      --relay-arg=--ws-listen --relay-arg=127.0.0.1:0
                      --relay-arg=--call-concurrency --relay-arg=2 --relay-arg=--auth-timeout
                      --relay-arg=1)
      foreach(_a IN LISTS CVC_ARIADNE_PYTHON_TEST_PATH)
        list(APPEND _auth_relay "--relay-arg=${_a}")
      endforeach()
      set(_auth_client --client-only --client-arg=--relay-room --client-arg=ctest)
      add_test(NAME arisrvx_auth_staff
               COMMAND ${_pair} --relay $<TARGET_FILE:arisrvx> --client $<TARGET_FILE:ari>
                       --scheme grpc ${_auth_relay} ${_auth_client}
                       --client-env=CVC_ARIADNE_ROOM_TOKEN=staff-token-0123456789
                       "--client-arg=${_srv_tests}/auth_staff.ari")
      add_test(NAME arisrvx_auth_user_ws
               COMMAND ${_pair} --relay $<TARGET_FILE:arisrvx> --client $<TARGET_FILE:ari>
                       --scheme ws ${_auth_relay} ${_auth_client}
                       --client-env=CVC_ARIADNE_ROOM_TOKEN=user-token-0123456789
                       "--client-arg=${_srv_tests}/auth_user.ari")
      foreach(_case wrong:nope-token-0123456789 slow:slow-token-0123456789
                    raises:boom-token-0123456789)
        string(REPLACE ":" ";" _case "${_case}")
        list(GET _case 0 _name)
        list(GET _case 1 _token)
        add_test(NAME arisrvx_auth_refuses_${_name}
                 COMMAND ${_pair} --relay $<TARGET_FILE:arisrvx> --client $<TARGET_FILE:ari>
                         --scheme grpc ${_auth_relay} ${_auth_client} --client-status 1
                         "--client-expect=refused this host \\(unauthenticated"
                         --client-env=CVC_ARIADNE_ROOM_TOKEN=${_token}
                         "--client-arg=${_srv_tests}/auth_user.ari")
        list(APPEND _call_tests arisrvx_auth_refuses_${_name})
      endforeach()
      # A client that joins with a host credential (--relay-credential: an OIDC sign-in's, here one
      # --credential-env supplies, bound to the relay's origin by a policy line relay_pair.py
      # writes), and dials again when the relay closes its connection.
      add_test(NAME arisrvx_auth_credential
               COMMAND ${_pair} --relay $<TARGET_FILE:arisrvx> --client $<TARGET_FILE:ari>
                       --scheme grpc ${_auth_relay} ${_auth_client}
                       --client-credential=relaytok=staff-token-0123456789
                       "--client-arg=${_srv_tests}/auth_credential.ari")
      list(APPEND _call_tests arisrvx_auth_staff arisrvx_auth_user_ws arisrvx_auth_credential)
    endif()
    if(TARGET arix)
      add_test(NAME arisrvx_room_call_ws_arix
               COMMAND ${_pair} --relay $<TARGET_FILE:arisrvx> --client $<TARGET_FILE:arix>
                       --scheme ws ${_call_relay} "--expect=room-call explode from arix_[a-z0-9]+ raised"
                       --client-arg=--relay-room --client-arg=ctest
                       "--client-arg=${_srv_tests}/call_client.ari")
      list(APPEND _call_tests arisrvx_room_call_ws_arix)
    endif()
    cvc_ariadne_python_test_properties(${_x_tests} ${_call_tests})
    set_tests_properties(${_x_tests} ${_call_tests} PROPERTIES
                         SKIP_REGULAR_EXPRESSION "No module named '(fastapi|uvicorn|starlette)';built without yaml-cpp;needs pycvc beside arisrvx;no pycvc to convert"
                         TIMEOUT 120)
  endif()
endif()
