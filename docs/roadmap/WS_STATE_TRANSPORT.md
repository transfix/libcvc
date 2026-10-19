# WebSocket `state_transport` — design + implementation plan

**Goal.** A WebSocket backend of the `cvc::state_transport` interface (`inc/cvc/state/state_transport_ws.h`) so a **browser (wasm) client joins the same `cvc::state` cluster — and therefore the same Ariadne room — as native clients**, and everyone sees/hears everyone regardless of client kind (the user's interop requirement). Browsers can't open gRPC streams or listen for connections, but they can open a WebSocket to a reachable server.

**Key idea — same wire as gRPC.** Both transports exchange the identical protobuf envelope `cvc.transport.v1.Frame` (`proto/state_transport.proto`: `oneof { Hello, Mutation, Message, Chunk*, Snapshot*, Heartbeat }`). The WS transport carries it as WebSocket **binary** frames instead of a gRPC bidi stream. Because the bytes are identical, a **relay** running both a gRPC acceptor and a WS acceptor can forward a frame received on one onto the other — that is the WS↔gRPC bridge that unifies wasm + native clients in one room.

**Topology.** Browser = client only (SFU-only). `start()` (acceptor) is native-only (Boost.Beast). `connect_to_peer()` has two client impls chosen at compile time: Boost.Beast on native, the browser WebSocket (`emscripten/websocket.h`) under `__EMSCRIPTEN__`.

**Build.** Needs the generated `Frame` **message** (protobuf), not the gRPC **service**, so it must NOT require `CVC_ENABLE_GRPC` — the wasm conference builds grpc-off. Boost.Beast (header-only) gives the native WS; link `Boost::system`.

## Status
- ✅ `inc/cvc/state/state_transport_ws.h` — the full API + design (this header).
- ✅ `src/cvc/state/state_transport_ws.cpp` — **native MVP implemented and green.** Boost.Beast
  (synchronous, one reader thread per connection), the full `state_transport` interface
  (register/publish/pump/flush) + `publish_message`, star-topology `set_relay()`, and
  backfill-on-connect. Hello / Mutation / Message frames over WS binary; chunk / snapshot / heartbeat
  are Phase 2 (base-class defaults for now).
- ✅ **Emscripten (browser) client implemented** — under `#ifdef __EMSCRIPTEN__` the connection wraps
  a browser WebSocket (`emscripten/websocket.h`): `connect_to_peer()` opens it and registers
  open/message/close/error callbacks (no reader thread — onmessage drives `deliver_inbound()`), Hello
  + backfill go out in `on_connection_open()` once the socket can send, and `write_locked()` uses
  `emscripten_websocket_send_binary`. Compiles clean under `emcc` (the native reader and the browser
  callback share one `deliver_inbound()`). The full browser round-trip needs the wasm conference
  build + a reachable relay (next).
- ✅ **WS↔gRPC bridge** — `state_transport::set_bridge(peer)` (base class): a frame received on one
  backend is, after its own relay, published on the bridged peer so it fans out to the peer's
  connections. `publish()`/`publish_message()` do not re-enter `on_inbound`, so no loop forms, and
  the receiver's dedup makes the redundant shard apply a no-op. This is how a browser (WS) and a
  native (gRPC) client share one room.
- ✅ `CVC_ENABLE_WS` CMake option — generates the proto **messages** independently of
  `CVC_ENABLE_GRPC` (a messages-only `protoc --cpp_out` when gRPC is off; the combined gRPC gen when
  on, so both-on does not double-generate), links `protobuf::libprotobuf`, rides SetupBoost's
  `Boost::system`/`Boost::thread`.
- ✅ `state_transport_ws_test` — **6 tests green** (local, grpc-on): start/stop, connect-fails, live
  mutation server→client, late-joiner backfill, **relay bridges two pure clients** (the conference
  topology), and **`BridgeWsAndGrpcClientsShareARoom`** — a native WS client and a native gRPC client
  reach each other through one bridged relay (the browser↔native interop proof). The first five also
  pass grpc-off (0 `grpc::` symbols in that `libcvc.so`).

**Teardown note (learned):** a synchronous Beast `accept()`/`read()` already blocked in a thread is
NOT interrupted by closing the socket from another thread. `stop()` therefore `shutdown()`s each
connection socket (which wakes a blocked `recv()`) and makes a throwaway loopback self-connect to
wake the blocked `accept()`, then joins.

## Implementation steps

1. **Frame codec (shared).** Extract the inline `encode_mutation` / `decode_mutation` / `encode_message` / `decode_message` + Frame build/parse from `state_transport_grpc.cpp` into a shared TU `state_frame_codec.{h,cpp}` (namespace-private, no gRPC types — just the generated message + `state_mutation`/`state_message`). Both transports use it, keeping the wire provably identical. (If a grpc-transport refactor is too risky to land with this change, the WS `.cpp` may duplicate the four functions first and the extraction follows as a separate PR.)

2. **Proto generation decoupled from gRPC.** Ensure the `Frame` *messages* are generated for this target even when `CVC_ENABLE_GRPC=OFF` (protoc `--cpp_out`, no grpc plugin). The gRPC *service* stubs stay grpc-only. Needed for the wasm build.

3. **`state_transport_ws.cpp`** (mirror `state_transport_grpc.cpp`, MVP scope):
   - `struct impl` (pimpl): native = a `boost::asio::io_context` + `tcp::acceptor` (server) and per-connection `beast::websocket::stream<tcp::socket>`; run the io_context on a small thread pool. Under `__EMSCRIPTEN__` = the browser WebSocket for the client (no server).
   - `connection`: owns the WS stream + a write mutex (+ a send queue for async writes); a read loop that parses each inbound binary frame into a `Frame` and dispatches `on_inbound_mutation` / `on_inbound_message` / (Phase 2) chunk/snapshot/heartbeat.
   - `start()`: bind the acceptor, accept loop → `admit_connection`.
   - `connect_to_peer(url)`: resolve + WS handshake (Beast native / emscripten wasm), send Hello, start the reader; block until handshake or timeout.
   - `publish()` / `publish_message()`: build a `Frame`, write it to every connection under each connection's write mutex (skip the origin); bump `_published`/`_sent_frames`.
   - `pump_shard()` / `pump_all()`: drain the shard's local-origin journal, encode each to a `Frame`, write to all connections, advance the cursor (identical to grpc).
   - `on_inbound_mutation` / `on_inbound_message`: `apply_remote` to each registered shard; if `relay()`, forward the frame to every OTHER connection (origin skipped, origin_node_id preserved → receiver dedup makes round-trips no-ops).
   - `admit_connection`: under the connection's write lock, send Hello and (if `backfill_on_connect`) replay each shard's journaled local mutations before live traffic.
   - `stop()`: close all connections, stop the io_context, join threads.

4. **CMake.** Add `state_transport_ws.cpp` to the `cvc` lib behind a `CVC_ENABLE_WS` option (default ON when Boost is present; independent of `CVC_ENABLE_GRPC`). Generate the proto messages for it. Link `Boost::system`.

5. **Test** `state_transport_ws_test` (mirror `state_transport_grpc_test`): two transports (server + client) over `ws://127.0.0.1:0`, register a shard each, publish + `pump_all`, `wait_for_received`, assert the mutation crossed byte-for-byte; a second case with the server in `relay()` mode bridging two clients.

## Phase 2 (after the native MVP is green)
- ✅ **The WS↔gRPC relay bridge** — done as `state_transport::set_bridge(peer)` (base class) + the
  `on_inbound_*` forward in both back ends; proven by `BridgeWsAndGrpcClientsShareARoom`. The relay
  server (`arisrv`) still needs to be wired to actually create both a `state_transport_grpc` and a
  `state_transport_ws` over the same shards and call `set_bridge` both ways + `start` both acceptors
  (a ~dozen lines in `ariadne_server_main.cpp`; today it starts only the gRPC acceptor). (Cleanest
  long-term remains a transport-neutral `connection` interface so one relay holds mixed connections.)
- **Wasm conference build.** Compile the conference client to wasm (grpc OFF, WS ON) with the
  emscripten client (link `-lwebsocket.js`); the browser dials `wss://` the relay → the
  cyberpcangel.com MVP. This is the remaining end-to-end step and the only one not yet exercised.
- `request_snapshot` over WS (the conference late-joiner bootstrap) + `fetch_chunk` (blob data plane) + heartbeat.
- `wss://` (TLS) + bearer-token auth (mirror `state_transport_grpc`'s `tls_config`/`auth_config`).
- Cleanup: extract the Frame codec (duplicated verbatim from `state_transport_grpc.cpp`) into a shared TU.
