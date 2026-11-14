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
// starts with the built-ins: files, bytes, text, json, mesh, heightfield, volume and city_bundle.
// Thread-safe; a decoder runs on a load's worker thread and throws std::exception to fail the
// load. Another app's documents don't see it.
void register_data_decoder(cvc::app &app, const std::string &as, DataDecoder decoder);
bool has_data_decoder(cvc::app &app, const std::string &as);
std::vector<std::string> data_decoders(cvc::app &app);

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

  // The store node a path names: the store path itself, or a document's link to one. "" when the
  // path is neither.
  std::string resolve(const std::string &path) const;

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
