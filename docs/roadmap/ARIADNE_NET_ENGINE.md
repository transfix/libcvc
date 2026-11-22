# Native connection engine: TCP and TLS streams with groups and fan-out (N1–N6)

Status: N1 (the C++ engine) is built: `inc/cvc/net/net_engine.h`, `src/cvc/net/net_engine.cpp`,
`net_engine_test`. N2 (pycvc and arisrvx) is built: `bindings/pycvc/pycvc_net.i`, `host.net`,
`test_pycvc_net.py`, `bench_pycvc_net.py`. N3-N6 and the registry tie-in are designs. What building
N1 and N2 changed in this design is marked *(N1)* and *(N2)* below.

This adds one native component to libcvc: a **connection engine** that owns raw TCP and TLS
connections on I/O threads of its own. It accepts connections and makes them, splits their bytes
into frames, queues output with backpressure, and sends one buffer to a group of connections
without a copy per receiver. It knows no protocol: the program above it decides what the frames
mean.

Three users need it:

- **Server programs in `arisrvx`** that speak a TCP protocol to clients that are not Ariadne
  clients. The first is cy-pca/sublevels, a Direct Connect (NMDC) hub. Its M1 soak
  (cy-pca/sublevels#8) measured what one asyncio loop can do: 6000 users fit (17 KiB each), but a
  message to all 6000 costs about 0.2 s of CPU, about 35 µs a connection, mostly the socket write.
  That is about 5 such messages a second, all the loop can do.
- **Documents in `ari` and `arigl`** that open TCP connections, behind a host allowlist. These are
  the X1 and X2 items of the sublevels roadmap, for nicotina's vanilla NMDC client.
- **WebAssembly guests**, later. A guest gets connections as stream channels and never owns a
  socket.

It ties into the client registry of [ARISRV_IDENTITY.md](ARISRV_IDENTITY.md) (ID1, #1293), so
connections that arrive through the engine are listed, logged and administered with the relay's
own clients.

## 0. Why not a plugin ABI, or a separate binary

The alternatives were weighed on 2026-11-16 (Joe chose this engine):

- **A dlopen plugin ABI for arisrvx.** A plugin built against libcvc's C++ headers breaks whenever
  libcvc's ABI moves. 3.7.13+cvc.4 dropped two symbols that arisrvx and pycvc +cvc.2 linked, and
  #1289 now pins every package to libcvc's major for that reason. A stable plugin ABI means a
  versioned C interface and a plugin build per platform, kept for good. arisrvx can already load
  native code: a CPython extension module is a dlopened module with its ABI fixed by the pinned
  CPython.
- **An extension inside one project** (say, sublevels' own C module). It would work, but nicotina's
  document sockets need the same engine, and so would the next TCP service.
- **A WebAssembly guest that owns sockets.** It is slow where it matters, since every write
  crosses into the host, and WASI preview 1 has no listening sockets. A guest that handles frames
  while the host does the I/O is neither: that is N6.

## 1. What exists

| Piece | Where | What it gives |
|---|---|---|
| Boost.Asio, Beast, OpenSSL | `state_transport_ws.cpp`, `utility.cpp`; `SetupBoost` | The I/O and TLS libraries the engine is built on. No new dependency. |
| Fan-out without copies | `cvc/ariadne/stream/stream_channel.h` | One producer's `frame_ptr` (a `shared_ptr`) to N bounded subscriber queues by refcount. |
| Relay to Python | `src/ariadne_python/server_link.h` | Events posted from any thread, taken by the script's waker thread without the GIL and handed to the asyncio loop. |
| The client registry | `client_registry.{h,cpp}` (ID1, #1293) | A record per client id: username, device, first and last seen, a connection log, admin, limits. |
| Wasm guests | `src/ariadne_wasm/` (G1–G4) | Guests on wasmtime with scoped WASI and gas. |

## 2. Design

### 2.1 Objects

- **`net_engine`** is caller-owned, made by a host (arisrvx's `main()`, an `ari` document host, or
  pycvc under plain `python`). It has no singleton. It runs `io_threads` Asio `io_context`s, two
  by default and at most one per core. Each connection lives on one of them for its whole life, so
  it needs no lock inside it.
- **A listener:** an address, a port, a framing, optional TLS, a backlog (1024 by default; asyncio's
  100 lost logins in sublevels' soak) and limits.
- **A connection:** a 64-bit id that is never reused within a run. It records the peer and local
  addresses, the listener, and TLS (version, cipher, SNI) if any.
- **A group:** a name and a set of connections, kept by the program (`join`, `leave`). A
  connection is in any number of groups. Closing a connection leaves all of them.

### 2.2 Framing

The engine splits each connection's input into frames, in one of these modes:

| Mode | Frame | For |
|---|---|---|
| `delimiter(byte)` | the bytes up to and including the byte | NMDC (`|`), line protocols (`\n`) |
| `length(prefix)` | a 1-, 2- or 4-byte big-endian length, then that many bytes | binary protocols |
| `raw` | whatever one read returned (at most `max_frame`) | anything else |

A `length` frame is the bytes after the prefix, without it. A frame longer than the listener's
`max_frame` closes the connection, with reason `frame_too_long`: a `delimiter` frame as soon as
`max_frame` bytes have come with no delimiter, a `length` frame as soon as its prefix says so.

A program can switch a connection's mode between frames: NMDC switches to `raw` for a compressed
block, and back. *(N1)* The I/O threads split input as it arrives, ahead of the program, so a
switch the program makes after seeing a frame comes too late for bytes the peer sent right behind
it (NMDC's `$ADCSND` and the file's bytes come in one write). So:

- `set_framing(conn, f)` applies from the first byte not yet split.
- A framing's `pause_after` names frame prefixes (`$ADCSND`). After such a frame the engine splits
  nothing more of that connection, and stops reading it, until the program's next `set_framing`
  (or `resume`). The switch then lands exactly after the frame.
- `raw(n)` delivers the next `n` bytes as raw frames and then pauses the same way, for the switch
  back.

### 2.3 Input: batches to the program

- The I/O threads post events to a queue with the same shape as ServerLink's. One waker hands
  them to the program's loop in **batches**: everything that arrived since the last batch, in
  order per connection.
- The events are `accepted(conn, info)`, `frames(conn, [bytes…])`, `closed(conn, reason)`,
  `writable(conn)` (output drained below the low mark, if asked for) and `error(listener, …)`.
- A Python batch is one call holding a list of `(conn, [memoryview…])`. Each memoryview points
  into a buffer the engine keeps alive until the call returns, so Python makes no copy unless it
  keeps a frame.
- **Limits** the engine enforces before the program sees anything: bytes per window per
  connection, frames per window, and the time a connection may take to send its first frame.
  Exceeding one closes the connection with a reason the program receives (`bytes_limit`,
  `frames_limit`, `first_frame_timeout`).
- *(N1)* In C++, `wait(batch, timeout)` and `poll(batch)` hand over everything queued as a
  `net_batch`; its frames are `string_view`s into refcounted buffers the batch holds until it is
  cleared. `set_notify(fn)` is called on an I/O thread when the queue goes from empty to not, for
  N2's waker. A read's bytes are copied once, from the I/O thread's read buffer into a buffer
  sized to them, so an idle connection holds no read buffer at all.

### 2.4 Output: sends, broadcasts and backpressure

- `send(conn, bytes)`, `send_many([conn…], bytes)` and `broadcast(group, bytes, exclude=conn)`
  each copy the bytes **once** into a refcounted buffer. Each receiver's queue holds a reference,
  as `stream_channel` does. A broadcast to 6000 costs the program one call. The I/O threads walk
  the group and write, gathering each connection's queued buffers into one `writev` (or `WSASend`)
  where they can.
- **Order.** Commands from the program apply in the order it issued them. For any one connection,
  bytes go out in the order of the sends, broadcasts and closes that reached it. A `join` made
  before a `broadcast` includes the connection.
- **Backpressure.** Each connection has a bound on queued output bytes (`max_queued`, 4 MiB by
  default). Past it, the connection closes with reason `slow_reader`. Protocols like NMDC can't
  drop messages from the middle of a stream, so dropping is not offered. A program that wants to
  slow a producer asks for `writable(conn)` at a low mark instead.
- `close(conn, flush=True)` sends what is queued, then closes. `flush=False` closes now.
  *(N1)* A flushing close ends our side once the output is out and posts `closed` then; the socket
  is read and dropped until the peer ends its side (at most `flush_timeout`, 10 s), so input the
  peer sent that nobody read does not turn the close into a reset that loses the output.
- *(N1)* The I/O threads gather: what reaches one connection in one turn of its thread (the
  commands taken together, say ten broadcasts issued in a row) goes out in one `writev`.

### 2.5 TLS

- Listener TLS from a certificate and key file, with OpenSSL through `asio::ssl`. TLS 1.2 and
  1.3 by default; the minimum is a setting.
- **Reload.** `reload_tls(listener)` loads new files for new connections without dropping open
  ones. The engine can watch the files' modification times.
- **Detection on a plain port** (optional). A listener with `detect_tls_ms` waits that long for
  the client's first byte. A TLS record (0x16) starts TLS; otherwise the connection is plain.
  Protocols where the server speaks first (NMDC) need this, because a plain client waits for the
  server. `accepted` is posted after the detection, so the program's greeting goes out on the
  right layer.
- **Client TLS** (N4): SNI, verification against the system's roots or a given CA file, and an
  optional pin (`SHA256/<base32>` of the certificate, as ADC's KEYP).
- The connection's info tells the program the TLS version and the SHA-256 of the peer's
  certificate if it sent one.

### 2.6 Identity: the client registry

- Connections from the engine have no `Hello`, and so no device id. With a listener's
  `register_clients` on, each `accepted` connection gets a record in ID1's `client_registry`:
  - transport `tcp` or `tls`, or a name the listener gives (`nmdc`, `nmdcs`);
  - the client id from ID1's fallback, a salted hash of its addresses (ARISRV_IDENTITY §2.1);
  - its username, set by the program when the protocol names it (an NMDC nick), through the
    registry's rules for uniqueness among active clients;
  - its connection log entry, opened and closed with the connection, with geo from
    `@host.geolocate` (ID4c) when arisrvx has one.
- **Admin, one surface.** ID3's `server.kick` and `server.ban` act on engine connections too: the
  registry calls back into the engine to close a connection or refuse an id at accept. The
  program can veto an accept as well (`on_accept` returning a reason), for its own bans and
  location policy.
- **Seam needed from ID1** (asked of paws, 2026-11-16): an entry point to open and close a record
  for a connection that has no Hello and no pid, keyed by the engine's connection id.

### 2.7 The Python surface (pycvc)

```python
from pycvc.ariadne import Host
from pycvc.ariadne.net import Framing

host = Host()                                   # in arisrvx, or plain python
net = host.net                                  # the host's engine; plain python gets its own

hub = net.listen("0.0.0.0", 411, framing=Framing.delimiter(b"|"),
                 backlog=1024, max_frame=65536, name="nmdc", register_clients=True)
tls = net.listen("0.0.0.0", 4111, framing=Framing.delimiter(b"|"),
                 tls=("/etc/ssl/hub.pem", "/etc/ssl/hub.key"), name="nmdcs",
                 register_clients=True)

@hub.on_accept                                  # also on tls; may return a refusal reason
def accepted(conn, info): ...

@hub.on_frames                                  # one call per batch per listener
def frames(batch):                              # [(conn, [memoryview, ...]), ...]
    ...

@hub.on_close
def closed(conn, reason): ...

net.send(conn, b"$Lock ...|")
net.join(conn, "all"); net.join(conn, "active")
net.broadcast("all", line)                      # one call, any group size
net.close(conn, flush=True)
net.stats()                                     # counters, below
```

- Every call only queues a command and returns. None blocks or waits on the I/O threads, and none
  takes long enough to matter, so the loop stays free.
- Callbacks run on the program's loop (the script thread in arisrvx), never on an I/O thread.
- Under plain `python`, `pycvc.ariadne.net` runs the same engine on its own threads, so a
  service's tests and a stand-alone run (sublevels' `run_hub.py`) work without arisrvx.

*(N2)* What was built differs from the sketch above in one way: Python gets the C++ API itself,
not a wrapper over it (Joe, 2026-11-20: "we want the full C++ api in python"). So:

- pycvc wraps `net_engine.h` with SWIG under its C++ names: `net_engine`, `engine_options`,
  `listener_options`, `framing`, `inbound_limits`, `conn_info`, `net_event`, `net_batch`,
  `net_counters`, `net_stats`, and the enums as SWIG spells them (`close_reason_slow_reader`,
  `net_event.type_frames`, `framing.mode_delimiter`). `pycvc.ariadne.net` is the same objects.
  `host.net` is a `net_engine`.
- Every command is the C++ one: `send`, `send_many`, `broadcast(group, bytes, exclude)`,
  `close(conn, flush)`, `join`, `leave`, `set_framing`, `resume`, `want_writable`, `port`,
  `stop_listening`, `stats`, `stop`, `wait(batch, ms)`, `poll(batch)`, `set_notify(fn)`.
- Additions only where Python needs them: bytes-like arguments (or a str, as UTF-8);
  `framing.delimited(b"|")` as well as an int; milliseconds as ints; the option structs take their
  members as keywords (`listener_options(port=411, frames=framing.delimited(b"|"))`) and read as
  values (assign a whole nested struct); `listen(options)` or `listen(**members)` raises OSError
  instead of returning 0; `net_counters.closed` is a dict by reason name; wait, poll, stop and the
  destruction release the GIL.
- Frames: `net_batch.events` gives each event's `frames` as memoryviews into the batch's buffers.
  Each memoryview holds a reference to those buffers, so a frame kept past the callback stays
  valid (it keeps its read's buffer alive) rather than dangling.
- The asyncio side: `listen()` returns a `net_listener`, the listener id (an int), whose
  `on_accept`, `on_frames`, `on_close`, `on_writable` and `on_error` register callbacks for the
  loop. A waker thread waits in `wait()` without the GIL and hands each batch to the loop with
  `call_soon_threadsafe`, then waits for the loop to take it, so input that arrives while the loop
  is busy comes as one larger batch. A batch goes out as accepts, then one `on_frames` call per
  listener (each connection once), then writables, closes and errors, which keeps each
  connection's order. `set_notify` is still there for a program with a loop of its own, but its
  callable runs on an I/O thread with the GIL taken, so the waker is the default.
- arisrvx makes the engine when the program first asks for `host.net` (`_arihost.net()`), with
  `--net-threads N` I/O threads (2 by default). It outlives the program; at the program's end only
  the waker stops. Under plain `python`, `Host(net_threads=2)` starts an engine of its own and stops
  it at shutdown, after its closes have reached `on_close`.

### 2.8 Documents (N5)

- Verbs for `ari` and `arigl`: `(net-connect "tcp://host:port" opts)` and `(net-send conn bytes)`,
  with frames arriving as `on_signal`, or as a stream channel (N6). Also `(net-close conn)` and,
  for active-mode protocols, `(net-listen port opts)` and UDP `(net-udp-bind port)` /
  `(net-udp-send conn host port bytes)`.
- **Off unless the host allows it.** `--net-allow tcp://host:port` (and `tcp-listen://port`,
  `udp://…`) extends the existing HTTP allowlist, and denies by default.
- Not in the browser: wasm builds have no raw sockets. A browser client reaches a TCP service
  through a WebSocket gateway on the server, which is that service's business (sublevels has one
  planned).

### 2.9 Channels and guests (N6)

- A connection can be bound to a stream channel: its frames are published to the channel (ring
  mode), and a subscriber sends by host call. This is how a wasm guest takes part. The guest
  handles each frame and calls host imports (`net.send`, `net.broadcast`, `net.join`) that queue
  engine commands, so the guest never owns a socket and its gas covers only its own logic.
- This follows ARIADNE_WORLDS Q3's order, "native kernels now, wasm guests later". It builds on
  G4's guests and on the resident guests plan (#1290).

### 2.10 Counters

`stats()` returns, per engine and per listener:
- connections open, accepted and closed by reason;
- bytes and frames in and out;
- broadcasts and the receivers they reached;
- queued output (total, maximum, closes for `slow_reader`);
- per I/O thread, busy time.

Programs publish them as they like; sublevels' analytics (its roadmap D14) draws them.
*(N1)* `messages_out` counts buffers queued to connections (one a receiver), `bytes_out` bytes
written; the per-thread counters are each thread's own, summed by `stats()`.

## 3. Parts

| Part | What | Files |
|---|---|---|
| N1 | The engine in C++: I/O threads, listeners, framing, output queues, groups, broadcast, limits, counters; ctest with TSan | `inc/cvc/net/net_engine.h`, `src/cvc/net/net_engine.cpp` (new) |
| N2 | pycvc binding and arisrvx wiring: `host.net`, batches through a ServerLink-shaped queue, plain-python mode | `src/ariadne_python/*`, `ariadne_server.py` |
| N3 | TLS: listener certificates, reload, detection on a plain port | `net_engine.*` |
| N4 | Client connections and UDP: connect with SNI, verification and pins; UDP bind and send | `net_engine.*` |
| N5 | Document verbs and `--net-allow` for `ari` and `arigl` | the verb tables, host flags |
| N6 | Stream-channel binding and guest imports | `net_engine.*`, `src/ariadne_wasm/*` |
| — | Registry tie-in (§2.6), after ID1 lands | `net_engine.*`, `client_registry.*` (with paws) |

**Order.** N1, then N2 and N3. sublevels adopts the engine then, and reruns its soak (#8)
through it. N4 and N5 follow for nicotina, and N6 after G4 and #1290.

## 4. Performance target

- **Measured on the soak's machine, with sublevels' load generator:** a message to 6000
  connections should cost the program one call, and the I/O threads under 10 ms in total. That
  would allow about 100 such messages a second, against about 5 today.
- p99 delivery to all 6000 under 50 ms with 2 I/O threads.
- Logins: 400 a second accepted with no backlog loss.
- These targets are estimates from per-write costs, not measurements. N2's soak is the check, and
  its numbers go in that PR.

*(N1, measured.)* The first target was wrong about the floor. `net_engine_test`'s fan-out
benchmark sends a 113-byte line to 6000 loopback connections. On prettyhatemachine (Xeon E5-2650
v2, Linux 7.0, retpolines), one `write()` to a loopback TCP socket costs 16.5 µs on its own,
because the sender's syscall also runs the receiving socket's side of the loopback. So 6000
receivers cost about 100 ms of CPU however the engine is written. The engine adds about 1 µs a
receiver to that:

| I/O threads | `broadcast()` call | I/O CPU a receiver | all 6000 received |
|---|---|---|---|
| 1 | 15 µs | 17.3 µs | 104 ms |
| 2 | 20 µs | 18.4 µs | 57 ms |
| 4 | 31 µs | 21.8 µs | 44 ms |
| 8 | 50 µs | 25.3 µs | 25 ms |

("All received" includes the test's own two reader threads. A bare loop of `write()`s to the same
6000 sockets takes 99-103 ms.)

What this changes:
- The program's loop pays one call of 15-50 µs, not 0.2 s. The loop is free, which was the point.
- The I/O CPU a message to all is one write a receiver: about 100 ms here over loopback. "Under
  10 ms" holds only where a write is about 1.5 µs. A real NIC's send does not run the receiver's
  stack, so it is cheaper than loopback, but by how much is for N2's soak to measure.
- Messages a second can far exceed writes a second: broadcasts that reach a thread together share
  one write a receiver (the burst line of the benchmark). A hub that sends its loop's broadcasts
  together pays per burst, not per message.
- The wall time falls with I/O threads, while the CPU a receiver rises a little (the kernel's
  locks). On the soak's machine, 4 threads would be the default to try.

*(N2, measured.)* `bindings/pycvc/bench_pycvc_net.py` drives the same fan-out from Python: 6000
loopback receivers, the 113-byte line, 2 I/O threads, 21 rounds, prettyhatemachine with other
sessions' builds running (load 10 on 32 cores), run twice, interleaved with N1's C++ benchmark:

| | C++ (`net_engine_test`) | Python (`bench_pycvc_net.py`) |
|---|---|---|
| I/O CPU a receiver | 19.3, 17.9 µs | 17.8, 20.1 µs |
| `broadcast()` call | 19.5, 23.3 µs | 30.5, 34.9 µs |
| burst of 10: I/O CPU a receiver a message | — | 2.0, 2.7 µs |
| all received | 63, 56 ms (C++ readers) | 69, 67 ms (Python readers) |

- From Python the engine costs what it costs from C++: the I/O CPU a receiver is the same within
  the runs' spread, and both sit at the write floor (17.6 µs idle in N1's table).
- The binding adds about 1 µs a call: `io_threads()` costs 0.8 µs, `send` 2.8 µs and an empty
  group's `broadcast` 5.3 µs, the engine's queueing included. The single-broadcast gap above is
  the benchmark's Python reader threads waking as the call returns.
- The program's side, which is what N2 was for: a Python loop of `sock.send()` to 6000 loopback
  sockets costs the program 33.5-34.1 µs a receiver, 0.20 s a message (sublevels' soak measured
  about 35 µs with asyncio's transports). Through the engine the program pays one call of
  30-35 µs.
- Input: 6000 receivers each send a line; all 6000 reach `on_frames` on an asyncio loop in
  116-122 ms, in 2 `on_frames` calls, and the loop thread spends 1.3-1.4 µs a frame (the waker's
  hand-off, the memoryviews, the callback).

## 5. Open questions

- **Windows and macOS.** Asio's IOCP and kqueue backends cover them. CI should run N1's tests on
  all three.
- **io_uring.** Asio has an io_uring backend on Linux. Worth measuring after N2, not before.
- **One loop's limit.** The engine moves I/O off the loop, but the protocol logic stays in Python.
  A hub busy enough to saturate the loop's logic needs several processes, which is a question
  for the service, not the engine.
- **Frames to Python.** Memoryviews into engine buffers are the plan. If keeping buffers alive
  across a callback proves awkward, the fallback is one `bytes` per batch with offsets.
