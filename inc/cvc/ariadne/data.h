/*
  Copyright 2026 The University of Texas at Austin
  This file is part of libcvc.
  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/
#ifndef CVC_ARIADNE_DATA_H
#define CVC_ARIADNE_DATA_H
// Ariadne data (docs/ARIADNE.md §5.17, docs/roadmap/ARIADNE_DATA_ROADMAP.md): a document's
// datasets, named by URI in its `data:` block, loaded off the frame thread into one shared place in
// the state tree.
//
//   data:
//     city:
//       uri: cvcpkg://cy-pca/scene-austin-south-small   # file, https, cvcpkg, state, ...
//       path: share/cvc-scenes/austin_south             # the part of the archive to keep
//       as: city_bundle                                 # the decoder
//
// The store is the app root's `data` subtree, one node per request (DataSpec), keyed by what was
// asked for, so a request made twice is loaded once:
//
//   data.<key>              value: the status (idle, fetching, unpacking, decoding, ready, error)
//     status error progress bytes sha256 uri canonical as
//     rev                   bumped each time a load completes: the leaf a watcher polls
//     files                 value: the file count; files.<i>: value its path, data() its bytes
//     ...                   what the decoder wrote (city_bundle: world, terrain, buildings, bounds)
//
// A document's `data.<name>` is a transparent link to its request's node, so a widget binds
// `data.city.status`, a program reads `(state-get "data.city.rev")` and a host asks for
// `<prefix>.data.city`, and all of them reach the same node. Bytes and decoded objects sit on the
// nodes' data() behind shared_ptrs (DataBytes, std::shared_ptr<const T>), so a reader copies a
// pointer, never the payload.
//
// Sources: a file or directory (relative to the document), https:// (the host's network, as
// enable_net set it), cvcpkg://ORG/NAME[@VERSION] (a cvcpkg data package: the registry's newest
// noarch build, its archive checked against the registry's sha256 and kept in a local cache),
// state://PATH?data, and any scheme with a registered URI handler. Archives (tar, gzip or zstd
// tar, zip) are unpacked in memory (cvc/utility/archive.h).
//
// A dataset can be made instead of fetched (data-generate, ARIADNE_WORLDS section 18.8): a
// generator makes its files from params, and they are decoded as a fetched one's are. The
// built-in generated_world makes a world (cvc::world::generate_world) and writes it as a version 2
// city bundle, so a generated world is the same kind of dataset as a loaded city:
//
//   data:
//     world:
//       generate: generated_world
//       params: { seed: 7, half_m: 1000, preset: 1 }
//
// Saving is the app's too (data_save, a program's (data-save PATH FILE [AS])): what a state key's
// data channel holds, encoded by an encoder (register_data_encoder; the built-ins write a volume,
// point classes, a volume grid session and bytes) and written beside the document, or in a
// browser downloaded. So a node that edits data in memory (a volume_grid's point classes) reads
// and writes no file itself: the app loads its data onto the tree, and saves it from there.
#include <boost/any.hpp>
#include <cvc/utility/archive.h>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace cvc {
class app;
class state;
namespace net {
class NetPolicy;
}
namespace ariadne {

// One entry of a data: block, or one (data-load ...) call.
struct DataSpec {
  std::string name;  // the document's name for it: <prefix>.data.<name>
  std::string uri;   // where it comes from
  std::string path;  // a directory inside the archive or directory: only its files, it stripped
  std::string as;    // the decoder; empty: files for an archive or directory, else by extension
  bool lazy = false; // load: lazy: bound now, loaded when something asks for it
  std::string base;  // the document's directory, which a relative file uri is relative to
  // A dataset made, not fetched: the generator (register_data_generator) and its params, a JSON
  // object. Its files decode as city_bundle unless `as` says otherwise. `uri` may then name the
  // dataset, for one its owner remakes in place (a generated_world node's: DataStore::put);
  // without one the params name it, so the same params share one dataset.
  std::string generate;
  std::string params;
};

// A file's bytes, as a store node holds them.
using DataBytes = std::shared_ptr<const std::string>;

// The state tree's root for the store.
inline constexpr const char *kDataRoot = "data";

// What a decoder gets: the request, its files (`path:` already stripped) and where to write.
struct DataDecode {
  cvc::app &app;
  const DataSpec &spec;
  std::string node; // the store node, "data.<key>": write the outputs under it
  const std::vector<cvc::archive::file> &files;

  // The file at `path` (as in files), or null.
  DataBytes file(const std::string &path) const;
  // The first file whose name (the last path component) is `name`, or null.
  const cvc::archive::file *find(const std::string &name) const;
  // The output node `child` under node (made when absent).
  cvc::state &out(const std::string &child) const;
};
using DataDecoder = std::function<void(DataDecode &)>;

// Register (or replace) `app`'s decoder named `as` (an empty decoder removes it). Every app
// starts with the built-ins: files, bytes, text, json, mesh, heightfield, volume, city_bundle,
// point_classes (a .vgr's classes, cvc/volume/point_classes.h, at `classes`),
// volume_grid_session (a volume grid session's JSON, cvc/volume/volume_grid.h, at `session`) and
// contours (a cvc::contour_stack, cvc/geometry/contours.h, at `contours`: .ser, .contour, .cvcc and
// the other formats the app reads; a .ser's section files come from its archive or the page's
// preload).
// Thread-safe; a decoder runs on a load's worker thread and throws std::exception to fail the
// load. Another app's documents don't see it.
void register_data_decoder(cvc::app &app, const std::string &as, DataDecoder decoder);
bool has_data_decoder(cvc::app &app, const std::string &as);
std::vector<std::string> data_decoders(cvc::app &app);

// What a request with `generate:` runs in place of a fetch: it makes the dataset's files from
// d.spec.params (a version 2 city bundle's, for city_bundle), on the load's worker thread and
// holding the app as a decoder does, and may write outputs of its own under d.node (d.out), beside
// the decoder's. d.files is empty. A generator that makes no files and names no `as:` has written
// its outputs itself, and no decoder runs (cvcGL's sdf: a mesh's signed distance field, as
// `volume`). Throws std::exception to fail the load. Every app starts with
// generated_world (scene_generated_world.h: the world as a version 2 bundle, and the world itself
// as `generated`). Thread-safe, per app, as the decoders are.
using DataGenerator = std::function<std::vector<cvc::archive::file>(DataDecode &)>;
void register_data_generator(cvc::app &app, const std::string &name, DataGenerator generator);
bool has_data_generator(cvc::app &app, const std::string &name);
std::vector<std::string> data_generators(cvc::app &app);

// What an encoder gets: the payload to write (a state node's data()) and the file it is for, whose
// extension names the format where an encoder writes several (.rawiv, .mrc, ...).
struct DataEncode {
  cvc::app &app;
  const boost::any &payload;
  std::string file;
};
// The file's bytes. Throws std::exception (saying why) when it can't encode the payload.
using DataEncoder = std::function<std::string(const DataEncode &)>;

// Register (or replace) `app`'s encoder named `as` (an empty encoder removes it). Every app starts
// with: bytes (DataBytes or a std::string), volume (a cvc::volume, in any format libcvc writes, by
// the file's extension), point_classes (cvc::point_classes as .vgr), volume_grid_session (its
// JSON) and contours (a cvc::contour_stack in a one-file format: .cvcc, .contour or .pcd). Each
// takes the payload as a std::shared_ptr<const T>, a std::shared_ptr<T> or a T.
// Thread-safe and per app, as the decoders are.
void register_data_encoder(cvc::app &app, const std::string &as, DataEncoder encoder);
bool has_data_encoder(cvc::app &app, const std::string &as);
std::vector<std::string> data_encoders(cvc::app &app);

// The built-in encoder a payload takes when no `as` names one (by its type), or "".
std::string default_data_encoder(const boost::any &payload);

// Save what the state node at `path` (an absolute state path, followed through links) holds on its
// data channel as `file`, through the encoder `as` (empty: default_data_encoder). A relative file
// is the document's directory's (`base`; empty, the working directory's). A document whose
// directory is a URL (an https:// one) writes no file. In a browser the file downloads under its
// name instead. Throws std::runtime_error saying why: nothing at `path`, no encoder, the encoder's
// own error, or the file not written.
void data_save(cvc::app &app, const std::string &path, const std::string &file,
               const std::string &as = std::string(), const std::string &base = std::string());

// The store key of a request: a slug of its source and a hash of the request (uri, path, as).
std::string data_key(const DataSpec &spec);

// The local cache of downloaded archives, by hash: CVC_DATA_CACHE, else the user's cache directory
// (%LOCALAPPDATA%\cvc\data, ~/Library/Caches/cvc/data, $XDG_CACHE_HOME/cvc/data or
// ~/.cache/cvc/data). Empty where there is none (a browser): downloads are not kept.
std::string data_cache_dir();

// The app's data store: the one load per request, the workers doing them, and the links that
// name them. Shared by every document and host in the app, and kept as long as the app is: when
// the app's state tree goes, the store stops (a load still running writes nothing more) and goes.
class DataStore {
public:
  static std::shared_ptr<DataStore> of(cvc::app &app);
  ~DataStore();
  DataStore(const DataStore &) = delete;
  DataStore &operator=(const DataStore &) = delete;

  // Start loading `spec` unless it is loading or loaded (`reload` loads it again): returns its
  // store node's path, "data.<key>". Never throws: a failure is the node's status and error.
  std::string load(const DataSpec &spec, bool reload = false);

  // A document's entry: <prefix>.data.<name> becomes a transparent link to `spec`'s store node,
  // which is loaded now unless spec.lazy. Returns the store path.
  std::string bind(const std::string &prefix, const DataSpec &spec);

  // A dataset its owner has already made (a generated_world node's world, as its bundle's files):
  // `also` first (outputs of the owner's own, such as the made world), then `files` decoded as a
  // load of `spec` would decode them, now and on this thread; then <prefix>.data.<spec.name>
  // links to it. Put again under the same request, it replaces the outputs and its rev goes up.
  // With no files only `also` writes (a build with no glb writer puts the world as it was made).
  // reload() makes it again from spec's generator. Returns the store path; throws what the
  // decoder throws, leaving the node in error.
  std::string put(const std::string &prefix, const DataSpec &spec,
                  std::vector<cvc::archive::file> files, const DataDecoder &also = nullptr);

  // The store node a path names: the store path itself, or a document's link to one. "" when the
  // path is neither.
  std::string resolve(const std::string &path) const;

  // Put `payload` as the output `item` of the dataset `path` names (a store path or a document's
  // link to one), as a decoder writes its outputs, and raise its rev, so every node viewing the
  // dataset follows it: a filter's volume put back where the scene shows it. A write to
  // <prefix>.data.<name>.<item> through the document's link makes a node under the link instead;
  // this reaches the store node. `item` is one word, and not one of the store's own keys (status,
  // error, progress, bytes, sha256, uri, canonical, as, size, rev, files). Reloading the dataset
  // makes its outputs again from its source. Returns the store path; throws std::invalid_argument
  // naming the problem (no dataset there, a bad item, an empty payload).
  std::string set_output(const std::string &path, const std::string &item,
                         const boost::any &payload);

  // Load `path`'s request if it was bound lazily and is not loaded yet.
  void touch(const std::string &path);

  // Load `path`'s request again (a changed file, a newer package): its rev goes up when it is
  // done. Returns the store path, or "" when `path` names no request this store has seen.
  std::string reload(const std::string &path);

  // Wait until the node at `path` (a store path or a link to one) is ready or failed, at most
  // `timeout_seconds` (negative: as long as it takes). True when it is ready.
  bool wait(const std::string &path, double timeout_seconds = -1.0);

  // The network for https:// and cvcpkg:// sources. By default the host's (enable_net); a host
  // that vouches for its documents, or a test, sets one here (null: every origin).
  void set_network(std::shared_ptr<const cvc::net::NetPolicy> policy);

  // Stop: loads still running end without writing anything more, and their workers are left to
  // finish on their own.
  void shutdown();

  struct Impl;

private:
  explicit DataStore(cvc::app &app);
  std::shared_ptr<Impl> impl_;
};

// Readers for hosts. The bytes of the file `file_path` of the store node (or link) at `path`, or
// null; the paths of its files; and the status.
DataBytes data_file(cvc::app &app, const std::string &path, const std::string &file_path);
std::vector<std::string> data_files(cvc::app &app, const std::string &path);
std::string data_status(cvc::app &app, const std::string &path);

} // namespace ariadne
} // namespace cvc

#endif // CVC_ARIADNE_DATA_H
