# Using libcvc from a CMake project

The libcvc binary archives (`libcvc-<version>-<os>-<arch>-<config>.{tar.gz,zip}`)
are **relocatable**: extract them anywhere and point CMake at the extracted
directory. No system installation, no `LD_LIBRARY_PATH` games.

## 1. Extract the archive

```bash
# Linux / macOS
tar xf libcvc-3.1.0-linux-x86_64-release.tar.gz   # → ./libcvc-3.1.0/
# or
unzip libcvc-3.1.0-macos-arm64-release.zip        # → ./libcvc-3.1.0/

# Windows (PowerShell)
Expand-Archive libcvc-3.1.0-windows-x86_64-release.zip
```

The archive lays out the standard GNU install tree:

```
libcvc-3.1.0/
├── bin/                   # libcvc.dll (Windows only)
├── include/cvc/           # Public headers
├── lib/
│   ├── libcvc.{so,dylib,a,lib}
│   └── cmake/cvc/
│       ├── cvcConfig.cmake
│       ├── cvcConfigVersion.cmake
│       └── cvcTargets.cmake
├── README.md
├── LICENSE
└── USAGE.md               # this file
```

## 2. Point CMake at the extracted tree

The most portable way is `CMAKE_PREFIX_PATH`:

```bash
cmake -B build -DCMAKE_PREFIX_PATH=/path/to/libcvc-3.1.0
```

You can also set `cvc_DIR` directly:

```bash
cmake -B build -Dcvc_DIR=/path/to/libcvc-3.1.0/lib/cmake/cvc
```

## 3. Use it from your `CMakeLists.txt`

```cmake
cmake_minimum_required(VERSION 3.20)
project(my_app CXX)

find_package(cvc REQUIRED)

add_executable(my_app main.cpp)
target_link_libraries(my_app PRIVATE cvc::cvc)
```

That's it — the `cvc::cvc` imported target carries every `INTERFACE_*`
property (include dirs, link dependencies, compile defs) needed to build
against libcvc, and `find_dependency()` in `cvcConfig.cmake` will locate
Boost / HDF5 / FFTW / etc. on your system.

## 4. Picking matching configurations

The Debug archive (`...-debug.{tar.gz,zip}`) contains a debug build of
`libcvc` plus debug-info side files where applicable. On MSVC, mixing a
Release consumer with a Debug `libcvc.lib` is **not** supported — pick the
matching configuration. On Linux/macOS you can mix freely if you accept the
performance cost of debug runtime checks.

## 5. Minimal smoke-test project

```cpp
// main.cpp
#include <cvc/app.h>
#include <iostream>
int main() {
    cvc::app ctx;
    std::cout << "libcvc OK, " << ctx.threads().size() << " threads\n";
}
```

```bash
cmake -B build -DCMAKE_PREFIX_PATH=/path/to/libcvc-3.1.0
cmake --build build
./build/my_app
```

---

## 6. Distributed state

libcvc includes an optional replicated state layer that synchronises an
in-process key-value tree across multiple nodes in a cluster. The layer
is assembled from composable pieces — a shard, a transport, interest
filters, conflict resolution, blob storage — but the easiest entry point
is `distributed_state_session`, which wires everything together from a
single config struct.

### 6.1 Quick start

```cpp
#include <cvc/app.h>
#include <cvc/distributed_state_session.h>
#include <cvc/state.h>

int main() {
    cvc::app ctx;

    cvc::distributed_state_config cfg;
    cfg.cluster_id = "my_cluster";
    cfg.node_id    = "node_1";
    cfg.transport  = cvc::transport_kind::ipc;
    cfg.listen_address = "/tmp/my_cluster_node1.sock";
    cfg.seeds      = {"/tmp/my_cluster_node2.sock"};

    auto session = cvc::distributed_state_session::join(ctx, cfg);

    // Write a value — it replicates to every peer in the cluster.
    cvc::state::instance(ctx)("scene.title").value(std::string("Hello"));

    // Read a value (may have arrived from a remote peer).
    std::string title = cvc::state::instance(ctx)("scene.title").value();

    session->stop();   // graceful shutdown (also called by destructor)
}
```

### 6.2 Configuration reference (`distributed_state_config`)

| Field | Type | Default | Description |
|-------|------|---------|-------------|
| `cluster_id` | `string` | *(required)* | Unique cluster name shared by all peers |
| `node_id` | `string` | *(required)* | Unique node name within the cluster |
| `root_path` | `string` | `""` | Subtree prefix to replicate (`""` = whole tree) |
| `transport` | `transport_kind` | `inproc` | `inproc`, `ipc`, or `grpc` |
| `listen_address` | `string` | `""` | Socket path (IPC) or `host:port` (gRPC) |
| `seeds` | `vector<string>` | `{}` | Peer endpoints to connect to on startup |
| `mounts` | `vector<distributed_state_mount>` | `{}` | Per-path replication modes |
| `pump_interval_ms` | `uint32` | `10` | Background replication interval (0 = no pump thread) |
| `max_inline_payload_bytes` | `uint32` | `65536` | Values larger than this go to the blob store |
| `blob_store_path` | `string` | `""` | Filesystem path for blob persistence (`""` = memory only) |
| `snapshot_on_join` | `bool` | `false` | Request a full snapshot from the first seed on join |
| `enforce_authority` | `bool` | `false` | Reject mutations owned by a different cluster |
| `enforce_write_policy` | `bool` | `false` | Consult write-policy before applying remote writes |
| `resolve_conflicts` | `bool` | `false` | Track and resolve concurrent writes (last-writer-wins) |
| `enforce_delegation` | `bool` | `false` | Respect delegation boundaries |
| `enforce_interest` | `bool` | `false` | Drop inbound mutations outside the interest set |

**gRPC-only TLS / auth fields** (ignored for `inproc` / `ipc`):

| Field | Type | Default | Description |
|-------|------|---------|-------------|
| `tls_server_cert_pem` | `string` | `""` | PEM server certificate |
| `tls_server_key_pem` | `string` | `""` | PEM server private key |
| `tls_root_ca_pem` | `string` | `""` | PEM CA certificate for peer verification |
| `tls_require_client_auth` | `bool` | `false` | Require mutual TLS |
| `require_tls` | `bool` | `false` | Throw if TLS cert/key are missing (safety guard) |
| `auth_expected_token` | `string` | `""` | Bearer token expected from inbound peers |
| `auth_outbound_token` | `string` | `""` | Bearer token sent to outbound peers |

> **Note:** identifiers (`cluster_id`, `node_id`) must follow C identifier
> rules — alphanumeric plus underscores. Hyphens are not allowed.

### 6.3 Transports

**In-process (`inproc`)** — Multiple shards in one process, no network.
Useful for tests and multi-tree applications.

**IPC (Unix domain sockets)** — Same-host, multi-process replication.
Full-duplex, automatic reconnection.

```cpp
cfg.transport      = cvc::transport_kind::ipc;
cfg.listen_address = "/tmp/cluster.sock";
cfg.seeds          = {"/tmp/peer1.sock", "/tmp/peer2.sock"};
```

**gRPC (network)** — Cross-host clustering with optional TLS and bearer-token
auth. Only available when libcvc is built with `-DCVC_ENABLE_GRPC=ON`.

```cpp
cfg.transport      = cvc::transport_kind::grpc;
cfg.listen_address = "0.0.0.0:9999";
cfg.seeds          = {"peer1.example.com:9999"};
cfg.tls_server_cert_pem = load_file("server.crt");
cfg.tls_server_key_pem  = load_file("server.key");
cfg.tls_root_ca_pem     = load_file("ca.crt");
```

### 6.4 Reading and writing state

State nodes are addressed by dot-separated paths and accessed through the
`cvc::state` API:

```cpp
auto &root = cvc::state::instance(ctx);

// Write (any type convertible to string)
root("scene.camera.fov").value(std::string("90"));
root("scene.camera.fov").value(90);

// Read
std::string fov = root("scene.camera.fov").value();
int fov_i       = root("scene.camera.fov").value<int>();

// Block until a value arrives
std::string val = root("data.result").wait_for_value<std::string>();

// Metadata
std::string name = root("scene.camera").name();        // "camera"
std::string full = root("scene.camera").fullName();     // "scene.camera"
```

### 6.5 Interest filters

In large clusters a node can limit which paths it mirrors by registering
prefix-based interest filters:

```cpp
auto &shard = session->shard();
shard.add_interest("scene.geometry");   // receive scene.geometry.*
shard.add_interest("scene.metadata");
shard.set_enforce_interest(true);       // drop everything else

// Query
bool ok = shard.path_is_of_interest("scene.geometry.mesh");  // true
auto all = shard.interests();
shard.remove_interest("scene.geometry");
```

### 6.6 Snapshots and conflict resolution

**Snapshots** let a new node catch up to the current cluster state:

```cpp
// Automatic on join:
cfg.snapshot_on_join = true;

// Manual:
auto entries = shard.snapshot("scene");  // prefix filter
```

**Conflict resolution** uses deterministic last-writer-wins ordering
(lexicographic `node_id` + sequence number):

```cpp
shard.set_resolve_conflicts(true);

// Inspect recent conflicts:
auto conflicts = shard.recent_conflicts(64);
for (auto &c : conflicts)
    std::cout << c.path << ": winner=" << c.winner_node_id << "\n";
```

### 6.7 Session lifecycle

```cpp
auto session = cvc::distributed_state_session::join(ctx, cfg);

session->is_running();    // true while active
session->status();        // replica health snapshot
session->shard();         // access the underlying shard
session->transport();     // access the transport
session->blob_store();    // access the blob store

session->stop();          // graceful shutdown
```
