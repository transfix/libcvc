/*
  Copyright 2026 The University of Texas at Austin
  This file is part of libcvc.
  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/
// world_model_packages.cpp: see cvc/ariadne/world_model_packages.h.
#include <cvc/ariadne/data.h>
#include <cvc/ariadne/world_model_packages.h>
#include <cvc/core/app.h>
#include <cvc/state/state.h>
#include <cvc/world/model_assets.h>
#include <cvc/world/world_theme.h>
#include <exception>
#include <string>
#include <vector>

namespace cvc {
namespace ariadne {
namespace {

bool isUri(const std::string &s) {
  return s.find("://") != std::string::npos || s.find('/') == 0 ||
         s.find('\\') != std::string::npos || (s.size() > 1 && s[1] == ':');
}

bool endsWith(const std::string &s, const std::string &suffix) {
  return s.size() >= suffix.size() &&
         s.compare(s.size() - suffix.size(), suffix.size(), suffix) == 0;
}

} // namespace

std::string model_package_uri(const std::string &package) {
  if (package.empty() || isUri(package))
    return package;
#ifdef __EMSCRIPTEN__
  return "cvcpkg://" + package + "-web";
#else
  return "cvcpkg://" + package;
#endif
}

struct model_package_fetch::Impl {
  cvc::app *app = nullptr;
  struct load {
    std::string uri, node;
  };
  std::vector<load> loads;
  std::size_t fetched = 0;
  std::vector<std::string> errors;
};

model_package_fetch::model_package_fetch(cvc::app &app, const world::world_theme &theme)
    : d_(std::make_unique<Impl>()) {
  d_->app = &app;
  world::model_catalog &catalog = world::model_catalog_of(app);
  std::shared_ptr<DataStore> store = DataStore::of(app);
  for (const world::theme_package &p : world::world_theme_packages(theme)) {
    if (catalog.contains(p.asset))
      continue;
    const std::string uri = model_package_uri(p.package);
    bool seen = false;
    for (const Impl::load &l : d_->loads)
      seen = seen || l.uri == uri;
    if (seen)
      continue;
    DataSpec spec;
    spec.name = "model:" + p.asset;
    spec.uri = uri;
    // A cvcpkg package keeps its models under share/cvc-scenes/shared, a web edition under
    // share/cvc-scenes/web; a URI is the models' place.
    if (uri.rfind("cvcpkg://", 0) == 0)
      spec.path = endsWith(uri, "-web") ? "share/cvc-scenes/web" : "share/cvc-scenes/shared";
    d_->loads.push_back({uri, store->load(spec)}); // never throws: a failure is the node's status
  }
}

model_package_fetch::~model_package_fetch() = default;

bool model_package_fetch::poll() {
  cvc::app &app = *d_->app;
  world::model_catalog &catalog = world::model_catalog_of(app);
  for (std::size_t i = 0; i < d_->loads.size();) {
    const Impl::load &l = d_->loads[i];
    const std::string status = data_status(app, l.node);
    if (status != "ready" && status != "error") {
      ++i;
      continue;
    }
    if (status == "error") {
      cvc::state *e = cvc::state::instance(app).findThroughLinks(l.node + ".error");
      d_->errors.push_back(l.uri + ": " + (e && !e->value().empty() ? e->value() : "failed"));
    } else {
      std::size_t found = 0;
      for (const std::string &f : data_files(app, l.node)) {
        if (!endsWith(f, ".asset.json"))
          continue;
        const DataBytes bytes = data_file(app, l.node, f);
        if (!bytes)
          continue;
        ++found; // a sidecar, parsed or not
        try {
          world::model_asset a = world::parse_model_asset(*bytes, "", l.uri + ": " + f);
          a.data_node = l.node;
          if (!catalog.contains(a.name)) { // an installed copy wins
            catalog.add(a);
            ++d_->fetched;
          }
        } catch (const std::exception &ex) {
          d_->errors.push_back(ex.what());
        }
      }
      if (found == 0)
        d_->errors.push_back(l.uri + ": no model asset (*.asset.json) in it");
    }
    d_->loads.erase(d_->loads.begin() + std::ptrdiff_t(i));
  }
  return d_->loads.empty();
}

std::size_t model_package_fetch::pending() const { return d_->loads.size(); }
std::size_t model_package_fetch::fetched() const { return d_->fetched; }
const std::vector<std::string> &model_package_fetch::errors() const { return d_->errors; }

} // namespace ariadne
} // namespace cvc
