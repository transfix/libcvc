/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include "tf_editor_emit.h"

#include "../tf/json.h" // the model's own locale-proof number / string writers

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cvc/ariadne/bind.h>
#include <cvc/core/app.h>
#include <cvc/image/image.h>
#include <cvc/state/state.h>
#include <cvc/tf/editor.h>
#include <string>
#include <vector>

namespace cvc {
namespace ariadne {

namespace {

// The composed editor's preview raster: the canvas aspect of the native editor (4:1), wide enough
// for a half-block terminal to show the ramp's structure at 32 columns.
constexpr int kPreviewW = 256, kPreviewH = 64;

// The layer names a document spells (props `layers:`).
bool layer_listed(const Value *layers, const char *name) {
  if (!layers || layers->kind != Value::Kind::Sequence)
    return false;
  for (const Value &l : layers->items)
    if (l.scalar == name)
      return true;
  return false;
}

std::string fmt_num(double v) {
  char buf[48];
  std::snprintf(buf, sizeof(buf), "%.6g", v);
  return buf;
}

std::string lowered(std::string s) {
  std::transform(s.begin(), s.end(), s.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return s;
}

// A YAML boolean as the loader's flag() reads one: true / yes / on / y / 1 or false / no / off /
// n / 0, in any case. Anything else is not a boolean (false, `out` untouched).
bool parse_flag(const std::string &text, bool &out) {
  const std::string s = lowered(text);
  if (s == "true" || s == "1" || s == "yes" || s == "on" || s == "y") {
    out = true;
    return true;
  }
  if (s == "false" || s == "0" || s == "no" || s == "off" || s == "n") {
    out = false;
    return true;
  }
  return false;
}

// A props flag (interactive_updates, show_*, log_histogram, info) by the loader's flag() rule, so
// a document's `True` / `YES` / `On` means here what it means on every other widget; a value that
// is not a boolean keeps `dflt`.
bool prop_flag(const Widget &w, const char *key, bool dflt) {
  bool out = dflt;
  if (const Value *v = w.props.find(key); v && v->is_scalar())
    parse_flag(v->scalar, out);
  return out;
}

// --- props -> model JSON (the seed's inline lists, a YAML-map default:) ----------------------
//
// Props keep no YAML type (a scalar is its raw text), so each scalar is typed by where it sits in
// the model document (cvc::tf::from_string's schema), as the scene loader's transfer_function:
// reader types it, never by how the text looks: a position, colour, opacity, bound or size is a
// number, an isocontour id a string (even `01`), opacity_cubed a flag (the loader's spellings),
// domain `auto` (any case) or numbers. A scalar its slot cannot take is passed on as a JSON
// string, so from_string names the problem. Numbers go through the model's own writer: a '.'
// decimal point whatever the C locale.

enum class Slot {
  any,         // not part of the model's schema (from_string ignores it): text
  number,      // a number, or a list / object of numbers ([at,r,g,b], {at, rgb}, {min, max}, ...)
  flag,        // opacity_cubed
  domain,      // "auto" or numbers
  isocontours, // a list of isocontour
  isocontour,  // a bare position, or {id, at, rgb, opacity}
  model        // the whole document
};

// The slot of member `key` of an object in slot `parent`.
Slot member_slot(Slot parent, const std::string &key) {
  switch (parent) {
  case Slot::model:
    if (key == "isocontours")
      return Slot::isocontours;
    if (key == "domain")
      return Slot::domain;
    if (key == "opacity_cubed")
      return Slot::flag;
    if (key == "color" || key == "alpha" || key == "table" || key == "resolution" || key == "dims")
      return Slot::number;
    return Slot::any;
  case Slot::isocontour:
    return key == "id" ? Slot::any : Slot::number;
  case Slot::number:
  case Slot::domain:
    return Slot::number;
  default:
    return Slot::any;
  }
}

// The slot of an item of a list in slot `parent`.
Slot item_slot(Slot parent) {
  switch (parent) {
  case Slot::isocontours:
    return Slot::isocontour;
  case Slot::number:
  case Slot::domain:
    return Slot::number;
  default:
    return Slot::any;
  }
}

void scalar_json(const Value &v, Slot slot, std::string &out) {
  switch (slot) {
  case Slot::number:
  case Slot::isocontour: { // an isocontour given as a scalar is its position
    // Re-formatted: YAML spellings like ".5" or "+1" are not JSON.
    const double d = v.as_double(std::nan(""));
    if (std::isfinite(d)) {
      cvc::tf::json::append_exact(out, d);
      return;
    }
    break;
  }
  case Slot::flag: {
    bool b = false;
    if (parse_flag(v.scalar, b)) {
      out += b ? "true" : "false";
      return;
    }
    break;
  }
  case Slot::domain:
    if (lowered(v.scalar) == "auto") {
      out += "\"auto\"";
      return;
    }
    break;
  default:
    break;
  }
  cvc::tf::json::append_string(out, v.scalar);
}

// A Value subtree as model JSON, each scalar typed by its slot.
void typed_json(const Value &v, Slot slot, std::string &out) {
  switch (v.kind) {
  case Value::Kind::Scalar:
    scalar_json(v, slot, out);
    break;
  case Value::Kind::Sequence:
    out += '[';
    for (std::size_t i = 0; i < v.items.size(); ++i) {
      if (i)
        out += ',';
      typed_json(v.items[i], item_slot(slot), out);
    }
    out += ']';
    break;
  case Value::Kind::Map:
    out += '{';
    for (std::size_t i = 0; i < v.entries.size(); ++i) {
      if (i)
        out += ',';
      cvc::tf::json::append_string(out, v.entries[i].first);
      out += ':';
      typed_json(v.entries[i].second, member_slot(slot, v.entries[i].first), out);
    }
    out += '}';
    break;
  }
}

// --- the view -------------------------------------------------------------------------------

// The view keys of one instance, under tree.<id>.view (resolved like a window's tree.<id>.*).
struct ViewKeys {
  std::string base;
  std::string key(const char *leaf) const { return base + "." + leaf; }
};

cvc::tf::view read_view(cvc::app &app, const ViewKeys &k, const cvc::tf::view &init) {
  cvc::tf::view v;
  v.show_color = read_bool_or_seed(app, k.key("color"), init.show_color) != 0;
  v.show_opacity = read_bool_or_seed(app, k.key("opacity"), init.show_opacity) != 0;
  v.show_isocontours = read_bool_or_seed(app, k.key("isocontours"), init.show_isocontours) != 0;
  v.show_histogram = read_bool_or_seed(app, k.key("histogram"), init.show_histogram) != 0;
  v.show_contour_spectrum =
      read_bool_or_seed(app, k.key("contour_spectrum"), init.show_contour_spectrum) != 0;
  v.show_contour_tree = read_bool_or_seed(app, k.key("contour_tree"), init.show_contour_tree) != 0;
  v.log_histogram = read_bool_or_seed(app, k.key("log_histogram"), init.log_histogram) != 0;
  v.show_info = read_bool_or_seed(app, k.key("info"), init.show_info) != 0;
  double lo = 0.0, hi = 1.0;
  const std::string zoom = read_or_seed<std::string>(
      app, k.key("zoom"), cvc::tf::range_to_string(init.zoom_lo, init.zoom_hi));
  if (cvc::tf::range_from_string(zoom, lo, hi)) {
    v.zoom_lo = lo;
    v.zoom_hi = hi;
  }
  cvc::tf::clamp_zoom(v);
  return v;
}

void write_view(cvc::app &app, const ViewKeys &k, const cvc::tf::view &v) {
  write<int>(app, k.key("color"), v.show_color ? 1 : 0);
  write<int>(app, k.key("opacity"), v.show_opacity ? 1 : 0);
  write<int>(app, k.key("isocontours"), v.show_isocontours ? 1 : 0);
  write<int>(app, k.key("histogram"), v.show_histogram ? 1 : 0);
  write<int>(app, k.key("contour_spectrum"), v.show_contour_spectrum ? 1 : 0);
  write<int>(app, k.key("contour_tree"), v.show_contour_tree ? 1 : 0);
  write<int>(app, k.key("log_histogram"), v.log_histogram ? 1 : 0);
  write<int>(app, k.key("info"), v.show_info ? 1 : 0);
  write<std::string>(app, k.key("zoom"), cvc::tf::range_to_string(v.zoom_lo, v.zoom_hi));
}

// --- overlay sources ------------------------------------------------------------------------

// Where the keys beside a model live. A load: mount reaches a parent's model through a link hole,
// so the overlays are read beside the hole's TARGET (by its absolute path, the same keys a bind
// spelled through the hole reaches). The overlay request is written there only through a writable
// (rw) hole — a read-only grant keeps it local, like any other write through it.
struct OverlayBase {
  std::string read, write;
};

OverlayBase overlay_base(cvc::app &app, const std::string &path) {
  OverlayBase base{path, path};
  try {
    cvc::state &s = cvc::state::instance(app)(path);
    if (s.isLink() && s.linkMode() == cvc::state::link_mode::transparent) {
      const cvc::state::link_resolution lr = s.resolveLink();
      if (lr.kind == cvc::state::link_resolution_kind::resolved && lr.target) {
        base.read = lr.target->fullName();
        if (s.linkWritable())
          base.write = base.read;
      }
    }
  } catch (const std::exception &) {
  }
  return base;
}

// The state path an overlay is read from: props `<name>:` overrides it (a bind, resolved in the
// widget's scope), else the key beside the model.
std::string source_path(const TfEditorContext &ctx, const Widget &w, const std::string &base,
                        const char *name) {
  const std::string over = w.props.str(name);
  return over.empty() ? base + "." + name : ctx.resolve(over);
}

// Re-read one overlay string; parse it (or clear the layer) only when it changed.
template <typename Parse>
bool refresh(cvc::app &app, const std::string &path, std::string &src, Parse &&parse) {
  std::string s = read_or<std::string>(app, path, std::string());
  if (s == src)
    return false;
  src = std::move(s);
  parse(src);
  return true;
}

// --- the composed editor --------------------------------------------------------------------

// The midpoint of the widest gap between consecutive positions of a sorted list (where an "add"
// button inserts, so repeated adds spread out instead of stacking).
double widest_gap_mid(std::vector<double> at) {
  std::sort(at.begin(), at.end());
  double best = -1.0, mid = 0.5;
  for (std::size_t i = 1; i < at.size(); ++i)
    if (at[i] - at[i - 1] > best) {
      best = at[i] - at[i - 1];
      mid = 0.5 * (at[i] + at[i - 1]);
    }
  return mid;
}

template <typename List> std::vector<double> positions(const List &nodes) {
  std::vector<double> at;
  at.reserve(nodes.size() + 2);
  for (const auto &n : nodes)
    at.push_back(n.at);
  return at;
}

// A backend with no native editor (a terminal, a mock): a raster preview plus one row of sliders,
// pickers and buttons per node, the add / reset buttons and the layer toggles. Positions slide in
// data units (the isovalue a user reads is the one the renderer uses). The rows are drawn from the
// model as it was this frame and the edits land on a copy, so a re-sort (an isocontour move) or a
// delete never shifts the rows under the walk; deletes apply after every row is drawn. Every edit
// commits at once (a terminal slider commits per step).
TfEdit compose_editor(TfEditorContext &ctx, TfEditorCache &cache, const Widget &w,
                      const std::string &key, const TfEditorInput &in) {
  Backend &b = ctx.backend;
  TfEdit e;
  e.drawn = true;
  e.model = *in.model;
  e.view = in.view;
  const cvc::tf::model &shown = *in.model;
  cvc::tf::model &m = e.model;
  const double lo = in.lo, hi = in.hi;
  bool edited = false;

  if (!w.label.empty())
    b.text_line(w.label.c_str());
  // The preview raster: only a backend that shows pixels (a GL window, a kitty/Sixel/half-block
  // terminal) gets one; re-rendered only when what it shows changes.
  const Capabilities caps = b.capabilities();
  if (caps.bitmap || caps.terminal) {
    const std::string name = "tf_editor:" + key;
    const cvc::tf::view &v = in.view;
    char sig[256];
    std::snprintf(sig, sizeof(sig), "%p|%u|%u|%.17g|%.17g|%.17g|%.17g|%d%d%d%d%d%d%d",
                  static_cast<void *>(&b), cache.model_gen, cache.overlay_gen, lo, hi, v.zoom_lo,
                  v.zoom_hi, v.show_color, v.show_opacity, v.show_isocontours, v.show_histogram,
                  v.show_contour_spectrum, v.show_contour_tree, v.log_histogram);
    // The meshed interval's band is part of the picture too.
    const std::string full_sig = std::string(sig) + "\n" + in.interval_a + "\n" + in.interval_b;
    if (cache.preview_sig != full_sig) {
      cvc::tf::preview_options popt;
      popt.interval_a = in.interval_a;
      popt.interval_b = in.interval_b;
      const std::vector<std::uint8_t> px =
          cvc::tf::render_preview(shown, v, in.overlays, lo, hi, kPreviewW, kPreviewH, popt);
      if (px.size() == static_cast<std::size_t>(kPreviewW) * kPreviewH * 4)
        b.set_image(name, cvc::image(kPreviewW, kPreviewH, cvc::image::pixel_format::RGBA,
                                     cvc::image::data_type::u8, px.data()));
      cache.preview_sig = full_sig;
    }
    b.draw_image(name.c_str(), static_cast<float>(kPreviewW));
  }
  b.text_value("domain", fmt_num(lo) + " .. " + fmt_num(hi));

  b.push_id(key.c_str());
  // Colour nodes: the endpoints are pinned in position (a colour edit only).
  int drop_color = -1;
  for (std::size_t i = 0; i < shown.color.size(); ++i) {
    const std::string tag = "c" + std::to_string(i);
    b.push_id(tag.c_str());
    const bool endpoint = i == 0 || i + 1 == shown.color.size();
    const std::string row = "colour " + std::to_string(i + 1);
    const double v = cvc::tf::to_data(shown.color[i].at, lo, hi);
    if (endpoint) {
      b.text_value(row.c_str(), "at " + fmt_num(v));
    } else {
      const DoubleEdit d =
          b.slider_double(row.c_str(), (key + "#" + tag).c_str(), v, lo, hi, "%.4g");
      if (d.committed &&
          cvc::tf::move_color(m, static_cast<int>(i), cvc::tf::to_normalized(d.value, lo, hi)))
        edited = true;
    }
    const float rgb[3] = {shown.color[i].r, shown.color[i].g, shown.color[i].b};
    const ColorEdit ce = b.color("rgb", rgb);
    if (!ce.drawn)
      b.text_value("rgb", fmt_num(rgb[0]) + "," + fmt_num(rgb[1]) + "," + fmt_num(rgb[2]));
    else if (ce.committed && cvc::tf::set_color(m, static_cast<int>(i), ce.rgb))
      edited = true;
    if (!endpoint && b.button("x"))
      drop_color = static_cast<int>(i);
    b.pop_id();
  }
  // Alpha nodes: the endpoints keep their position but their opacity edits.
  int drop_alpha = -1;
  for (std::size_t i = 0; i < shown.alpha.size(); ++i) {
    const std::string tag = "a" + std::to_string(i);
    b.push_id(tag.c_str());
    const bool endpoint = i == 0 || i + 1 == shown.alpha.size();
    const std::string row = "alpha " + std::to_string(i + 1);
    const double v = cvc::tf::to_data(shown.alpha[i].at, lo, hi);
    if (!endpoint) {
      const DoubleEdit d =
          b.slider_double(row.c_str(), (key + "#" + tag + "t").c_str(), v, lo, hi, "%.4g");
      if (d.committed && cvc::tf::move_alpha(m, static_cast<int>(i),
                                             cvc::tf::to_normalized(d.value, lo, hi), m.alpha[i].a))
        edited = true;
    }
    const std::string alabel = endpoint ? row + " at " + fmt_num(v) : std::string("opacity");
    const DoubleEdit a = b.slider_double(alabel.c_str(), (key + "#" + tag + "a").c_str(),
                                         shown.alpha[i].a, 0.0, 1.0, "%.3f");
    if (a.committed &&
        cvc::tf::move_alpha(m, static_cast<int>(i), m.alpha[i].at, static_cast<float>(a.value)))
      edited = true;
    if (!endpoint && b.button("x"))
      drop_alpha = static_cast<int>(i);
    b.pop_id();
  }
  // Isocontours, by id (a move re-sorts the edited list; the rows follow the frame's snapshot).
  std::string drop_iso;
  for (const cvc::tf::isocontour &iso : shown.isocontours) {
    b.push_id(("i" + iso.id).c_str());
    const std::string row = "iso " + iso.id;
    const DoubleEdit d = b.slider_double(row.c_str(), (key + "#i" + iso.id).c_str(),
                                         cvc::tf::to_data(iso.at, lo, hi), lo, hi, "%.4g");
    if (d.committed && cvc::tf::move_isocontour(m, iso.id, cvc::tf::to_normalized(d.value, lo, hi)))
      edited = true;
    if (b.button("x"))
      drop_iso = iso.id;
    b.pop_id();
  }
  const auto drop = [&](cvc::tf::part kind, int index, const std::string &id) {
    cvc::tf::handle h;
    h.kind = kind;
    h.index = index;
    h.id = id;
    if (index >= 0 && cvc::tf::remove(m, h))
      edited = true;
  };
  if (drop_color >= 0)
    drop(cvc::tf::part::color, drop_color, std::string());
  if (drop_alpha >= 0)
    drop(cvc::tf::part::alpha, drop_alpha, std::string());
  if (!drop_iso.empty())
    drop(cvc::tf::part::isocontour, m.find_isocontour(drop_iso), drop_iso);

  if (b.button("+ colour") && cvc::tf::add_color(m, widest_gap_mid(positions(m.color))))
    edited = true;
  if (b.button("+ alpha")) {
    // On the drawn (uncubed) curve, so adding a node never changes the curve's shape.
    const double t = widest_gap_mid(positions(m.alpha));
    cvc::tf::model linear = m;
    linear.opacity_cubed = false;
    if (cvc::tf::add_alpha(m, t, linear.alpha_at(t)))
      edited = true;
  }
  if (b.button("+ isocontour")) {
    std::vector<double> at = positions(m.isocontours);
    at.push_back(0.0);
    at.push_back(1.0);
    if (!cvc::tf::add_isocontour(m, widest_gap_mid(at)).empty())
      edited = true;
  }
  if (b.button("reset")) {
    // The nodes go back to ColorTable2's default; the domain and table size are the document's
    // choice, not the curve's, so they stay.
    cvc::tf::model d = cvc::tf::model::default_model();
    d.domain = m.domain;
    d.resolution = m.resolution;
    m = d;
    edited = true;
  }
  const BoolEdit cubed = b.checkbox("opacity cubed", m.opacity_cubed);
  if (cubed.committed && cubed.value != m.opacity_cubed) {
    m.opacity_cubed = cubed.value;
    edited = true;
  }
  // The layers (the native editor keeps these in its context menu).
  const auto layer = [&](const char *label, bool &on) {
    const BoolEdit t = b.checkbox(label, on);
    if (t.committed && t.value != on) {
      on = t.value;
      e.view_changed = true;
    }
  };
  layer("histogram", e.view.show_histogram);
  layer("contour spectrum", e.view.show_contour_spectrum);
  layer("contour tree", e.view.show_contour_tree);
  if (in.overlays && !in.overlays->status.empty())
    b.text_value("overlays", in.overlays->status);
  b.pop_id();

  e.changed = e.committed = edited;
  return e;
}

} // namespace

cvc::tf::model tf_editor_seed(const Widget &w, std::string *problem) {
  cvc::tf::model m;
  bool based = false;
  if (const Value *dflt = w.props.find("default"); dflt && !dflt->empty()) {
    std::string src, err;
    if (dflt->is_scalar())
      src = dflt->scalar; // a model string: JSON text already
    else
      typed_json(*dflt, Slot::model, src);
    if (cvc::tf::from_string(src, m, &err))
      based = true;
    else if (problem)
      *problem = "default: does not parse (" + err + ")";
  }
  if (!based) {
    const std::string preset = w.props.str("preset");
    m = preset.empty() ? cvc::tf::model::default_model() : cvc::tf::model::preset(preset);
  }
  // The inline lists (the scene transfer_function: vocabulary) replace their part of that base.
  static const char *const kParts[] = {"color",  "alpha",         "isocontours",
                                       "domain", "opacity_cubed", "resolution"};
  std::string json;
  for (const char *k : kParts)
    if (const Value *v = w.props.find(k)) {
      if (!json.empty())
        json += ',';
      cvc::tf::json::append_string(json, k);
      json += ':';
      typed_json(*v, member_slot(Slot::model, k), json);
    }
  if (json.empty())
    return m;
  cvc::tf::model part;
  std::string err;
  if (!cvc::tf::from_string("{" + json + "}", part, &err)) {
    if (problem)
      *problem = "its color / alpha / isocontours do not parse (" + err + ")";
    return m;
  }
  if (w.props.has("color"))
    m.color = part.color;
  if (w.props.has("alpha"))
    m.alpha = part.alpha;
  if (w.props.has("isocontours"))
    m.isocontours = part.isocontours;
  if (w.props.has("domain"))
    m.domain = part.domain;
  if (w.props.has("opacity_cubed"))
    m.opacity_cubed = part.opacity_cubed;
  if (w.props.has("resolution"))
    m.resolution = part.resolution;
  m.normalize();
  return m;
}

cvc::tf::view tf_editor_initial_view(const Widget &w) {
  cvc::tf::view v;
  const Value *layers = w.props.find("layers");
  if (layers && layers->kind == Value::Kind::Sequence) {
    v.show_histogram = layer_listed(layers, "histogram");
    v.show_contour_spectrum = layer_listed(layers, "contour_spectrum");
    v.show_contour_tree = layer_listed(layers, "contour_tree");
  }
  v.show_color = prop_flag(w, "show_color", v.show_color);
  v.show_opacity = prop_flag(w, "show_opacity", v.show_opacity);
  v.show_isocontours = prop_flag(w, "show_isocontours", v.show_isocontours);
  v.log_histogram = prop_flag(w, "log_histogram", v.log_histogram);
  v.show_info = prop_flag(w, "info", v.show_info);
  return v;
}

std::string tf_editor_instance_id(const Widget &w) {
  if (!w.id.empty())
    return w.id;
  return w.label.empty() ? std::string("tf_editor") : w.label;
}

void emit_tf_editor(TfEditorContext &ctx, const Widget &w) {
  Backend &b = ctx.backend;
  if (w.bind.empty()) {
    b.text_value(w.label.c_str(), "(tf_editor has no bind)");
    return;
  }
  const std::string path = ctx.resolve(w.bind);
  const std::string id = tf_editor_instance_id(w);
  // The backend keys its in-progress drag on this; the bind alone is not enough, since two editors
  // may show one model (each with its own view and gesture).
  const std::string key = path + "#" + id;
  TfEditorCache &cache = ctx.caches[key];

  // The model: seeded when the key is empty (a standalone editor), else whatever the document /
  // realizer / a script put there. A malformed string keeps the last good model and warns once.
  std::string cur = read_or<std::string>(ctx.app, path, std::string());
  if (cur.empty()) {
    std::string problem;
    const cvc::tf::model seed = tf_editor_seed(w, &problem);
    if (!problem.empty())
      ctx.warn_once("ari: tf_editor '" + w.label + "': " + problem + "; seeding without it");
    cur = read_or_seed<std::string>(ctx.app, path, cvc::tf::to_string(seed));
  }
  if (cur != cache.model_src || !cache.model_ok) {
    cvc::tf::model parsed;
    std::string err;
    if (cvc::tf::from_string(cur, parsed, &err)) {
      cache.model = parsed;
      cache.model_ok = cache.src_parsed = true;
      ++cache.model_gen;
    } else {
      cache.src_parsed = false;
      ctx.warn_once("ari: tf_editor '" + w.label + "': the model at '" + path +
                    "' does not parse (" + err + "); showing the last good model");
      if (!cache.model_ok) {
        cache.model = tf_editor_seed(w);
        cache.model_ok = true;
        ++cache.model_gen;
      }
    }
    cache.model_src = cur;
  }

  // The overlays published beside the model (or where props redirect them).
  const OverlayBase base = overlay_base(ctx.app, path);
  cvc::tf::overlays &ov = cache.overlays;
  bool ov_changed = refresh(ctx.app, source_path(ctx, w, base.read, "range"), cache.range_src,
                            [&ov](const std::string &s) {
                              ov.has_range =
                                  cvc::tf::range_from_string(s, ov.range_lo, ov.range_hi);
                            });
  ov_changed |= refresh(ctx.app, source_path(ctx, w, base.read, "histogram"), cache.histogram_src,
                        [&ov](const std::string &s) {
                          if (!cvc::tf::histogram_from_string(s, ov.histogram))
                            ov.histogram.clear();
                        });
  ov_changed |= refresh(ctx.app, source_path(ctx, w, base.read, "contour_spectrum"),
                        cache.spectrum_src, [&ov](const std::string &s) {
                          if (!cvc::tf::spectrum_from_string(s, ov.spectrum))
                            ov.spectrum = {};
                        });
  ov_changed |= refresh(ctx.app, source_path(ctx, w, base.read, "contour_tree"), cache.tree_src,
                        [&ov](const std::string &s) {
                          if (!cvc::tf::tree_from_string(s, ov.tree))
                            ov.tree = {};
                        });
  if (ov_changed)
    ++cache.overlay_gen;
  ov.status = read_or<std::string>(ctx.app, base.read + ".overlay_status", std::string());
  // The interval a mesher builds from two of the model's isocontours (fe_lab publishes it), read
  // where the overlays are; resolved against the model each frame, since either id may go.
  const std::string interval =
      read_or<std::string>(ctx.app, source_path(ctx, w, base.read, "interval"), std::string());

  // This instance's view, persisted like a window's geometry (tree.<id>.view.*).
  ViewKeys vk;
  vk.base = ctx.resolve("tree." + id + ".view");
  const cvc::tf::view view = read_view(ctx.app, vk, tf_editor_initial_view(w));

  TfEditorInput in;
  in.model = &cache.model;
  in.overlays = &ov;
  cvc::tf::effective_domain(cache.model, ov.has_range, ov.range_lo, ov.range_hi, in.lo, in.hi);
  in.view = view;
  in.interactive = prop_flag(w, "interactive_updates", true);
  in.height = static_cast<float>(w.props.num("height", 0.0));
  int ia = -1, ib = -1;
  if (cvc::tf::interval_isocontours(cache.model, interval, ia, ib)) {
    in.interval_a = cache.model.isocontours[static_cast<std::size_t>(ia)].id;
    in.interval_b = cache.model.isocontours[static_cast<std::size_t>(ib)].id;
  }

  TfEdit e = b.tf_editor(w.label.c_str(), key.c_str(), in);
  if (!e.drawn)
    e = compose_editor(ctx, cache, w, key, in);

  if (e.committed) {
    cvc::tf::model m = e.model;
    m.normalize();
    const std::string s = cvc::tf::to_string(m);
    // Only a real change writes and fires `on:` (a backend may report a commit for a release that
    // moved nothing, or a Reset / preset pick that lands on the model as it is). The edit is
    // compared with the model the key holds, not with its text: a valid spelling to_string does
    // not reproduce (map nodes, "{}", a script's spacing) is the same model. A key that does not
    // parse holds no model, so any commit replaces it. A write a read-only hole refused leaves the
    // cache on the state's model.
    const bool unchanged = cache.src_parsed && s == cvc::tf::to_string(cache.model);
    if (!unchanged) {
      write<std::string>(ctx.app, path, s);
      if (read_or<std::string>(ctx.app, path, std::string()) == s) {
        cache.model = m;
        cache.model_src = s;
        cache.src_parsed = true;
        ++cache.model_gen;
        if (!w.on.empty())
          ctx.enqueue(w.on);
      }
    }
  }
  cvc::tf::view shown = view;
  if (e.view_changed) {
    shown = e.view;
    cvc::tf::clamp_zoom(shown);
    write_view(ctx.app, vk, shown);
  }

  // Ask the producer for the costly layers this instance shows but nobody has computed yet (the
  // histogram is cheap and always published). The request only ever grows, so two editors on one
  // model cannot take each other's layers away.
  unsigned want = 0;
  if (shown.show_contour_spectrum && ov.spectrum.empty())
    want |= cvc::tf::overlay_contour_spectrum;
  if (shown.show_contour_tree && ov.tree.empty())
    want |= cvc::tf::overlay_contour_tree;
  if (want) {
    const std::string req_path = base.write + ".overlay_request";
    const unsigned have =
        cvc::tf::overlay_bits_from_csv(read_or<std::string>(ctx.app, req_path, std::string()));
    if ((have | want) != have)
      write<std::string>(ctx.app, req_path, cvc::tf::overlay_bits_to_csv(have | want));
  }
}

} // namespace ariadne
} // namespace cvc
