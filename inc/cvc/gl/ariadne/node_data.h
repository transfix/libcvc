/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// node_data.h -- a scene node's data on the state tree (docs/roadmap/DATA_FROM_STATE_TREE.md):
// the node views data the app holds, reads no file, keeps no copy, and follows the data as they
// change. A document names the data in the node's source:
//
//   data:
//     head: head.rawiv                    # the app loads it (docs/ARIADNE.md §5.17)
//   scene:
//     nodes:
//       - { node: render, type: volren, source: { data: head } }        # a dataset's output
//       - { node: cut, type: volume_slice, source: { node: render } }   # another node's
//       - { node: work, type: volren, source: { state: work.volume } }  # a program's, a filter's
//
// These are the pieces the node types share: what a source: names (NodeDataSource), and
// VolumeFollower, the volume a node shows, followed on the tree.
#ifndef CVC_GL_ARIADNE_NODE_DATA_H
#define CVC_GL_ARIADNE_NODE_DATA_H

#include <cvc/state/followed_key.h>
#include <memory>
#include <string>
#include <vector>

namespace cvc {
class app;
class slice_source;
class volume;
namespace ariadne {
struct SceneNode;
}
namespace gl {
class GraphicsNode;
class SceneGraph;
namespace ariadne {

struct RealizedScene;

// One of a node's data, as its document names it: the state key that holds them (relative to
// the document's scope, or '/'-rooted the root's) and, for a dataset, the document's entry
// data.<name> (relative as `path` is), whose loading the scene waits for.
struct NodeDataSource {
  std::string path;
  std::string dataset;
  explicit operator bool() const { return !path.empty(); }
};

// What a node's source: names: { data: <name>, item: <output> } (the item defaults to `item`),
// { state: <key> } or { node: <id>, key: <key> } (the key defaults to `node_key`). A node id is
// the state path of the node realized by that id before this one (any depth), else
// graphics.root.children.<id>; an id with a '.' is a state path. A dataset is touched, so a lazy
// one loads now. Empty when the source: names none of them (a file, a primitive, nothing): the
// caller says what it needs.
NodeDataSource node_data_source(SceneGraph &sg, const RealizedScene &out,
                                const cvc::ariadne::SceneNode &n, const std::string &item,
                                const std::string &node_key);

// The same for a volume: item `volume`, and a node's key grid.volume for a volume_grid realized
// before this node, else volume (what volren, volume_slice and the other volume nodes publish).
NodeDataSource volume_data_source(SceneGraph &sg, const RealizedScene &out,
                                  const cvc::ariadne::SceneNode &n);

// A prop that names data the same way, { data: <name>, item } or { state: <key> } (a volume_grid's
// points: and session:). Warns and is empty for anything else: the node reads no file.
NodeDataSource prop_data_source(cvc::app &app, const std::string &prefix,
                                const cvc::ariadne::SceneNode &n, const char *prop,
                                const char *item, std::vector<std::string> *warnings);

// While a dataset among `sources` is loading, the scene has work in flight (out.pending_work), as
// long as `node` lives.
void wait_for_datasets(RealizedScene &out, cvc::app &app, std::weak_ptr<GraphicsNode> node,
                       const std::vector<NodeDataSource> &sources);

// The volume a node shows, followed on the state tree. Keys, under the node's state path:
//   source.volume   the key that holds the volume (relative to `scope`, '/'-rooted the root's),
//                   seeded from the document's source:; a program may point it elsewhere. It
//                   holds a std::shared_ptr<const cvc::volume> (a data: entry's), a
//                   std::shared_ptr<cvc::volume>, a cvc::volume or a std::shared_ptr<
//                   cvc::slice_source> (a file read plane by plane)
//   source.status   "" while a volume is shown; else what the node waits for ("waiting for a
//                   volume at <key>") or why it shows none ("error: <key> holds no volume")
//   volume          published (data channel): the std::shared_ptr<const cvc::volume> shown, the
//                   very pointer the key held, so other nodes follow it (empty for a file read
//                   plane by plane)
// The node keeps the volume it last took while the key holds nothing (a dataset loading again).
class VolumeFollower {
public:
  VolumeFollower(const std::shared_ptr<GraphicsNode> &node, std::string scope);

  // Each tick, on the node's thread: whether a new volume arrived (source() and volume() are then
  // it). The same volume written again, or a link moved to it, is no new one.
  bool tick();

  // What the node shows: the slices, and the volume (null for a file read plane by plane); null
  // until the first arrives.
  const std::shared_ptr<cvc::slice_source> &source() const { return m_source; }
  const std::shared_ptr<const cvc::volume> &volume() const { return m_volume; }
  // What a transfer function's range and histogram are taken over: the volume, or for a file read
  // plane by plane a stand-in (its middle plane down z, with the file's range). Null until the
  // first volume arrives.
  const std::shared_ptr<const cvc::volume> &histogramVolume() const { return m_histogram; }

  // The absolute state path source.volume names now, or "".
  std::string sourcePath() const;

private:
  void status(const std::string &s);

  std::weak_ptr<GraphicsNode> m_node;
  std::string m_scope;
  cvc::followed_key m_followed;
  std::shared_ptr<cvc::slice_source> m_source;
  std::shared_ptr<const cvc::volume> m_volume, m_histogram;
  std::string m_status;
  bool m_said = false; // m_status has been written once
};

} // namespace ariadne
} // namespace gl
} // namespace cvc

#endif // CVC_GL_ARIADNE_NODE_DATA_H
