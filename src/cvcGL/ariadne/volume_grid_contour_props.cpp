/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// The volume_grid node type's contour props. See volume_grid_contour_props.h.

#include "volume_grid_contour_props.h"

#include <algorithm>
#include <cstdlib>
#include <cvc/ariadne/bind.h>
#include <cvc/ariadne/data.h>
#include <cvc/ariadne/scene.h>
#include <cvc/ariadne/value.h>
#include <cvc/core/app.h>
#include <cvc/core/param_table.h>
#include <cvc/geometry/tiler_params.h>
#include <cvc/gl/VolumeGridNode.h>
#include <cvc/state/state.h>
#include <cvc/volume/volume_grid_contours.h>

namespace cvc {
namespace gl {
namespace ariadne {

namespace ari = cvc::ariadne;

namespace {

void warn(std::vector<std::string> *w, const std::string &m) {
  if (w)
    w->push_back(m);
}

void seed(cvc::state &key, const std::string &text) {
  if (key.value().empty() && !text.empty())
    key.value(text);
}

// A prop as a key's text: a scalar as it is, a sequence as "a,b,c" (a colour), or as a JSON array
// for a list field (`json`).
std::string prop_text(const ari::Value &v, bool json = false) {
  if (!v.is_seq())
    return v.as_string();
  std::string out = json ? "[" : "";
  for (std::size_t i = 0; i < v.items.size(); ++i) {
    out += i ? (json ? ", " : ",") : "";
    const std::string &s = v.items[i].scalar;
    char *end = nullptr;
    std::strtod(s.c_str(), &end);
    if (!json || (!s.empty() && end && *end == '\0'))
      out += s;
    else
      out += "\"" + s + "\"";
  }
  return json ? out + "]" : out;
}

// Each key a prop map gives, under `base`, warning of a key the table has no field for.
template <class P>
void seed_table(cvc::state &base, const ari::Value *props, const std::string &where,
                std::vector<std::string> *warnings) {
  if (!props)
    return;
  if (!props->is_map()) {
    warn(warnings, where + ": must be a map -- ignored");
    return;
  }
  const std::vector<std::string> names = cvc::param_names<P>();
  for (const auto &e : props->entries) {
    if (std::find(names.begin(), names.end(), e.first) == names.end()) {
      warn(warnings, where + ": '" + e.first + "' is not one of its keys -- ignored");
      continue;
    }
    seed(base(e.first), prop_text(e.second, true));
  }
}

} // namespace

std::string seed_volume_grid_contour_props(cvc::app &app, const std::string &prefix,
                                           VolumeGridNode &g, const ari::SceneNode &n,
                                           std::vector<std::string> *warnings) {
  const std::string where = "ari: scene node '" + n.id + "' (volume_grid)";
  cvc::state &grid = g.getState("grid");
  seed_table<cvc::volume_grid_contour_params>(grid("contour"), n.props.find("contour"),
                                              where + ": contour", warnings);
  seed_table<cvc::tile_params>(grid("tile"), n.props.find("tile"), where + ": tile", warnings);

  // objects: the rows a list: binds, while there are none.
  if (const ari::Value *o = n.props.find("objects")) {
    cvc::state &rows = grid("objects");
    if (!o->is_seq()) {
      warn(warnings, where + ": objects: must be a list");
    } else if (rows.value().empty() || rows.value() == "0") {
      std::size_t count = 0;
      for (std::size_t i = 0; i < o->items.size(); ++i) {
        const ari::Value &row = o->items[i];
        if (!row.is_map() || !row.has("name")) {
          warn(warnings, where + ": objects[" + std::to_string(i) + "] needs a name");
          continue;
        }
        cvc::state &r = rows(std::to_string(count++));
        r("name").value(row.str("name"));
        if (const ari::Value *col = row.find("color"))
          r("color").value(prop_text(*col));
        for (const char *k : {"interpolation", "samples", "visible"})
          if (row.has(k))
            r(k).value(row.str(k));
      }
      rows.value(std::to_string(count));
    }
  }

  // contours: { data: <name>, item } or { state: <key> }: where the grid's contours come from.
  std::string dataset;
  if (const ari::Value *c = n.props.find("contours")) {
    if (c->is_map() && c->has("data")) {
      const std::string name = c->str("data");
      dataset = name[0] == '/' ? name : std::string(ari::kDataRoot) + cvc::state::SEPARATOR + name;
      seed(grid("contour.source"), dataset + cvc::state::SEPARATOR + c->str("item", "contours"));
      ari::DataStore::of(app)->touch(ari::resolve_bind(prefix, dataset)); // a lazy one loads now
      dataset = ari::resolve_bind(prefix, dataset);
    } else if (c->is_map() && c->has("state")) {
      seed(grid("contour.source"), c->str("state"));
    } else {
      warn(warnings, where + ": contours: is { data: <name> } (a data: entry the app loads) or "
                             "{ state: <key> }; the node reads no file -- ignored");
    }
  }
  return dataset;
}

} // namespace ariadne
} // namespace gl
} // namespace cvc
