/*
  Copyright 2026 The University of Texas at Austin
  This file is part of libcvc.
  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/
#ifndef CVC_ARIADNE_WORLD_MODEL_PACKAGES_H
#define CVC_ARIADNE_WORLD_MODEL_PACKAGES_H
// A world theme's model packages, fetched as datasets (cvc/ariadne/data.h) when they are not
// installed, so a themed world has its models on a host with no install: a browser page, or a
// native host with the network on and the packages missing.
//
// Each archetype names its package (world::building_archetype_rule::package). For every asset of
// the theme that the app's catalog (world::model_catalog_of) does not have, the package is loaded
// through the app's data store, as a document's `data:` entry would be:
//
//   "cy-pca/tree-palm"   -> cvcpkg://cy-pca/tree-palm, share/cvc-scenes/shared of it; in a
//                           browser cvcpkg://cy-pca/tree-palm-web, the package's web edition
//                           (its coarser LODs only: a page cannot hold the full model)
//   a URI ("https://...", a directory, "file://...") -> that, as it is
//
// Once a load is ready, every <name>.asset.json in it is added to the catalog with the load's store
// node as its data_node, so the renderer reads its glbs from memory (gl/ariadne/world_models.h). A
// load that fails (no network, no such package) is left out: the theme's archetypes for that asset
// then fall away, as for any asset that is not installed.
//
// It never blocks: poll() says whether every load has ended, so a sliced task (the world node's
// rebuild, on single-threaded wasm too) polls it a step at a time before it builds.
#include <memory>
#include <string>
#include <vector>

namespace cvc {
class app;
namespace world {
struct world_theme;
}
namespace ariadne {

// The data URI a package name fetches as: cvcpkg://<package>, with "-web" on its name in a browser;
// a URI (anything with "://", or a path) as it is.
std::string model_package_uri(const std::string &package);

class model_package_fetch {
public:
  // Starts the loads of `theme`'s packages whose assets the app's catalog lacks.
  model_package_fetch(cvc::app &app, const world::world_theme &theme);
  ~model_package_fetch();
  model_package_fetch(const model_package_fetch &) = delete;
  model_package_fetch &operator=(const model_package_fetch &) = delete;

  // True when every load has ended, ready or not; each ready one's assets are in the catalog.
  bool poll();
  std::size_t pending() const; // loads not ended yet
  std::size_t fetched() const; // assets added to the catalog so far
  // One line per load that failed or whose sidecar did not parse.
  const std::vector<std::string> &errors() const;

private:
  struct Impl;
  std::unique_ptr<Impl> d_;
};

} // namespace ariadne
} // namespace cvc

#endif // CVC_ARIADNE_WORLD_MODEL_PACKAGES_H
