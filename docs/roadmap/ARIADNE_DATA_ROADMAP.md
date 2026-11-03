# Ariadne data: a document's datasets in the state tree

Status: D1 landed (libcvc: the store, the data: block, the sources, the decoders, the scene, the
verbs, pycvc); D2 to D4 are next (see "Phases"). Owner: Ariadne. The reference for documents is
[ARIADNE.md §5.17](../ARIADNE.md).

## Why

A document can say how its scene looks (lights, shadows, sky) but not what is in it. A host loads
the data instead: GRL-SNAM's live demos find the Austin bundle on disk, load it into the scene
through `pycvc_gl.scenes.load_geometry_bundle`, then read the same files again for the navigator's
occupancy grid and terrain sampler. The document cannot name the city, swap it for another, or
load one from the network, and nothing else can share what the host loaded.

This design puts the data in the document and the loaded data in the state tree:

- a document's `data:` block names its datasets by URI: a file or directory, an `https` URL, a
  cvcpkg package (`cvcpkg://cy-pca/scene-austin-south-small`), another node's data (`state://`),
  and later another libcvc server;
- the runtime fetches each one off the frame thread, unpacks archives, decodes the files into
  typed objects (a `world_geometry`, a `height_field`, a mesh, a volume) and keeps them in one
  shared place in the state tree;
- the document's own name for a dataset is a transparent link to that place, so two documents,
  or a document and its Python host, that ask for the same data share one load;
- scene nodes take a dataset as their `source:`, and redraw when it changes;
- document logic can load another dataset or replace one at run time, and the scene follows.

## The `data:` block

```yaml
data:
  city:                                             # the document's name for it: data.city
    uri: cvcpkg://cy-pca/scene-austin-south-small   # where it comes from
    path: share/cvc-scenes/austin_south             # the part of the package to keep
    as: city_bundle                                 # how to decode it
  weights: cvcpkg://cvc/grl-snam-weights            # a bare URI: as: files, the whole package
  dem:
    uri: https://example.org/austin_dem.npy
    as: heightfield

scene:
  nodes:
    - node: city
      source: { data: city }                        # the terrain and buildings, drawn
```

Each entry's keys:

| Key | Meaning |
|---|---|
| `uri` | where the data comes from (the URI resolver's schemes, below). A bare string entry is its `uri`. |
| `path` | a directory inside the archive or package: only the files under it are kept, with it stripped from their paths. |
| `as` | the decoder (below). Default: `files` for an archive or a directory, else by the file's extension. |
| `load` | `eager` (default: at document load) or `lazy` (when something first asks for it: a scene node or `(data-load ...)`). |

A relative `uri` is relative to the document, as other URIs are.

## Where it lands: one store, linked into each document

The store is one subtree at the app root, `data`, keyed by what was asked for:

```
data.<key>                  value: the status: idle | fetching | unpacking | decoding | ready | error
  uri                       the request as written; canonical: what it resolved to
  as                        the decoder
  error                     why it failed, when it did
  progress                  0..1 while it loads
  size                      the downloaded or read size (an archive's, before unpacking)
  sha256                    the source's hash, when known
  rev                       bumped each time a load completes: the leaf a watcher polls
  files                     value: the file count
    files.<i>               value: the file's path; data(): its bytes; property size
  <the decoder's outputs>   e.g. city_bundle: world, terrain, buildings, bounds.*
```

`<key>` is a readable slug of the source plus a hash of `uri`, `path` and `as`
(`scene_austin_south_small_3f2a19c4`). The same request from anywhere is the same key, so it is
loaded once.

The document's entry `<prefix>.data.<name>` is a **transparent link** to `data.<key>`
(`state::linkTo`). A widget binds `data.city.status`, a program reads
`(state-get "data.city.progress")`, and Python asks `host.data("city")`: they all reach the same
node. Re-pointing the link (a new URI for `city`) is how a document replaces a dataset.

Bytes and objects are held by `shared_ptr` on the node's `data()`, so reading one copies a
pointer, not the payload. Watchers poll `rev`, a scalar, since `state-watch` compares values and
observers do not cross links.

## Sources

| Scheme | What it reads | Notes |
|---|---|---|
| `file://`, a bare path | a file, or a directory read as an unpacked archive | relative to the document |
| `https://` | the URL's bytes | needs the host's network (`--net-allow ORIGIN`); goes through its `NetPolicy` |
| `cvcpkg://ORG/NAME[@VERSION]` | a cvcpkg package's archive | below |
| `state://PATH?data` | another node's bytes | in-process, where the host registered the `state` scheme |
| `cvc+ws://HOST:PORT/PATH`, `cvc+grpc://...` | a subtree another libcvc server holds | phase D3 |

**`cvcpkg://`.** The runtime asks the registry (`CVCPKG_URL`, else `https://cvcpkg.org`) for the
package's newest data build, or the version given:

```
GET /v1/packages/NAME/latest?org=ORG&platform=any&arch=noarch&build_type=none&link=none[&version=V]
```

The reply names the archive (`archive_url`), its `sha256` and `size_bytes`. The runtime downloads
it through the host's network policy, checks the size and hash, and keeps it in a local cache
keyed by the hash (`CVC_DATA_CACHE`, else the user's cache directory: `%LOCALAPPDATA%\cvc\data`,
`~/Library/Caches/cvc/data`, `$XDG_CACHE_HOME/cvc/data`). A second run reads the cache and does
not touch the network. Public orgs need no token. A data package is a noarch build
(`platform: any`), so one archive serves every platform, the browser included.

## Archives

An archive is recognized by its first bytes, not its name (cvcpkg's `.tar.zst` files are gzip tars
today, and zip on Windows):

- tar (ustar, pax and GNU long names), plain or compressed with gzip or zstd;
- zip (stored and deflate).

Unpacking is safe by construction: only regular files are kept; a path that is absolute, has a
`..` or a drive letter is refused; links and devices are skipped; and the entry count and the
unpacked size are capped (`sys.data.max_entries`, default 100,000; `sys.data.max_bytes`, default
2 GiB). A tar member's bytes go straight into its `files.<i>` node; nothing is written to disk.

## Decoders (`as:`)

| `as:` | Reads | Writes |
|---|---|---|
| `files` | an archive or directory | `files.*` only |
| `bytes` | one file | `content`: its bytes on `data()` |
| `text` | one file | `text`: the text as the value |
| `json` | one JSON file | `json`: a subtree of values (objects are children, arrays are `0`, `1`, ...) |
| `mesh` | any format `read_geometry` reads (obj, ply, off, glb, gltf, ...) | `geometry`: a `cvc::geometry` |
| `volume` | any format `cvc::volume` reads (rawiv, mrc, ...) | `volume`: a `cvc::volume` |
| `heightfield` | a `terrain.json` | `terrain`: a `cvc::height_field`, and `bounds.*` |
| `city_bundle` | a city bundle's files (`terrain.json`, `buildings.glb`, ...) | `world` (`world_geometry`), `terrain` (`height_field`), `buildings` (`geometry`), `buildings_error`, `bounds.min_x` ... `max_y`, `satellite` (`satellite.png`, decoded) |

A host or library adds its own with `register_data_decoder(name, fn)`. A decoder reads files from
the store, so it works the same for a directory, a download and an archive. The path-based readers
under the mesh, volume and city decoders (Assimp, `load_city_bundle`) still open files: D1 hands
them the files they need in a temporary directory, removed after the read. In-memory readers are a
later step.

## Scene nodes take a dataset

```yaml
- node: city
  source: { data: city }                 # what data.city decoded
  body: static                           # the terrain and buildings in the physics world
- node: blocks
  source: { data: city, item: buildings } # one item of it
```

The node is created at load as an empty group with its transform. When its dataset's `rev`
changes (the first load, or a replacement) the scene rebuilds the node's children on the frame
thread, from the decoded object:

- a `world_geometry`: its items as nodes (`cvc::gl::add_world_geometry`), the terrain wearing the
  dataset's satellite image (UVs over the terrain's extent, north at the image's top), published
  as the node's height field (so `ground: city` works for `forest_trees`, `wave_sea`, ...), and,
  with `body:`, its bodies in the scene's physics world (`cvc::physics::add_world_geometry`);
- a `geometry`: a mesh node; a `height_field`: its mesh; a `volume`: a volume node.

Until then the node is empty, and the first frame never waits for a download.

## Document logic

```scheme
(data-load "city" "cvcpkg://cy-pca/scene-austin-south" "city_bundle"
           "share/cvc-scenes/austin_south")       ; replace the small city with the full one
(data-status "city")                              ; "fetching", "ready", ...
(data-files "city")                               ; the file paths
(data-file "city" "terrain.json")                 ; one file's bytes
```

`data-load` re-points the document's link at the new request's store node and starts the load.
The scene node bound to `city` rebuilds when that load completes; until then it keeps drawing the
old city.

## Python

```python
city = f"{prefix}.data.city"                 # the document's link
pycvc.data_wait(app, city)                   # the load runs on a worker thread
world = pycvc.data_world(app, city)          # the cvc::world::world_geometry the scene draws
terrain = pycvc.data_height_field(app, city) # its terrain: terrain.height(x, y) is the sampler
blocks = pycvc.data_geometry(app, city, item="buildings")
pycvc.data_files(app, city), pycvc.data_file(app, city, "satellite.png")  # the files, as bytes
```

These follow the document's link and share their arrays with the scene's nodes, so no file is
read a second time. A Python program without a document loads data itself with
`pycvc.data_load(app, uri, path=, decoder=, name=, prefix=)`, and turns the network on for it with
`pycvc.data_allow_network(app, ["https://cvcpkg.org"])`. A document host does that with
`AriRuntime.enable_net_allow([...])`, as `--net-allow` does for `ari` and `arigl`. A
`Host.data(name)` handle over these comes with the Python host (`pycvc_gl.ariadne.Host`).

## Capability tokens

A document whose data comes from the network lists `net` in `requires:`, so a host without the
network refuses it up front. Finer tokens (`data`, `data.cvcpkg`: the registry is reachable) can
come later if hosts differ there.

## Phases

- **D1 (libcvc), landed:**
  - the archive reader (`cvc/utility/archive.h`): tar, gzip, zstd and zip from memory, with its
    own inflate, so no zlib link;
  - the store and the `data:` block;
  - `file`, `https`, `cvcpkg` and `state` sources, with the download cache;
  - the `files`, `bytes`, `text`, `json`, `mesh`, `volume`, `heightfield` and `city_bundle`
    decoders;
  - `source: { data }` scene nodes, rebuilt on `rev`, with the world's bodies for `body:`;
  - the `data-*` verbs;
  - pycvc's `data_*` functions and `AriRuntime.enable_net_allow`;
  - `examples/ariadne/documents/austin_data.ari`, which draws the Austin scene from cvcpkg.org;
  - the curl client verifying TLS against the Windows certificate store, without which no native
    Windows build reached any https URL.
- **D2 (GRL-SNAM):** the live demos and ports declare their city in their documents, and the demos
  read `host.data("city")` instead of loading the bundle.
- **D3:** remote libcvc servers as sources:
  - join a distributed state session with interest in the remote path, the store node being its
    replica;
  - add state codecs for `world_geometry` and `height_field`, so a browser can take a world a
    native server loaded.
- **D4:** the browser:
  - non-blocking downloads (`resolve_async`) and decoding in slices in single-threaded wasm;
  - an IndexedDB cache;
  - the HTTP cache (`uri_http_cache`) for `https` sources.
- **Later:** lazy file bytes (an index first, a member's bytes when read); xz; signatures (cvcpkg's
  Ed25519 `signature`) checked when the registry has them; `reload:` of a changed source.
