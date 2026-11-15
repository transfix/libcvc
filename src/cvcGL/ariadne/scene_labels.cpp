/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// A scene node's `label:` and the scene's `entity_labels:` (scene_labels.h).

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cvc/ariadne/bind.h> // resolve_bind
#include <cvc/ariadne/scene.h>
#include <cvc/gl/GraphicsNode.h>
#include <cvc/gl/LabelLayer.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/ariadne/scene_labels.h>
#include <cvc/gl/ariadne/scene_realize.h>
#include <limits>

using cvc::ariadne::Value;

namespace cvc {
namespace gl {
namespace ariadne {

namespace {

// The style slots c0..c7: the chart's categorical colours (blue, orange, aqua, yellow, magenta,
// green, violet, red).
constexpr unsigned kSlots[8] = {0x2a78d6, 0xeb6834, 0x1baf7a, 0xeda100,
                                0xe87ba4, 0x008300, 0x6250d6, 0xe34948};

// [r, g, b] (0..1), "#rrggbb" or a slot c0..c7 into out; false for anything else.
bool parse_rgb(const Value &v, float out[3]) {
  if (v.is_seq() && v.items.size() >= 3) {
    for (int i = 0; i < 3; ++i)
      out[i] = static_cast<float>(std::clamp(v.items[i].as_double(0.0), 0.0, 1.0));
    return true;
  }
  if (!v.is_scalar())
    return false;
  const std::string &s = v.scalar;
  unsigned hex = 0;
  if (s.size() == 7 && s[0] == '#') {
    char *end = nullptr;
    hex = static_cast<unsigned>(std::strtoul(s.c_str() + 1, &end, 16));
    if (end != s.c_str() + 7)
      return false;
  } else if (s.size() == 2 && (s[0] == 'c' || s[0] == 'C') && s[1] >= '0' && s[1] <= '7') {
    hex = kSlots[s[1] - '0'];
  } else {
    return false;
  }
  out[0] = static_cast<float>((hex >> 16) & 0xff) / 255.0f;
  out[1] = static_cast<float>((hex >> 8) & 0xff) / 255.0f;
  out[2] = static_cast<float>(hex & 0xff) / 255.0f;
  return true;
}

bool is_off(const Value &v) {
  return v.is_scalar() &&
         (v.scalar == "none" || v.scalar == "false" || v.scalar == "off" || v.scalar == "no");
}

// `{name}`, `{id}`, ... in s filled from fields.
std::string fill(std::string s, const std::vector<std::pair<std::string, std::string>> &fields) {
  for (const auto &[k, v] : fields) {
    const std::string key = "{" + k + "}";
    for (std::size_t at = s.find(key); at != std::string::npos; at = s.find(key, at + v.size()))
      s.replace(at, key.size(), v);
  }
  return s;
}

// The label's text, style and keys (resolved against the scene's bind prefix).
struct ParsedLabel {
  bool present = false;
  std::string text;
  LabelStyle style;
};

// `focus_value` is what focus: compares its key with when it names no value: the node's id.
ParsedLabel parse_label(const Value *label, const std::string &who, const std::string &prefix,
                        const std::string &focus_value, std::vector<std::string> *warnings) {
  ParsedLabel out;
  if (!label || is_off(*label))
    return out;
  const auto warn = [&](const std::string &m) {
    if (warnings)
      warnings->push_back("ari: " + who + ": label: " + m);
  };
  out.present = true;
  if (label->is_scalar()) { // label: <text>
    out.text = label->scalar;
    return out;
  }
  if (!label->is_map()) {
    warn("is a text or a map of text, bind, offset, color, backdrop, edge, size, scale, focus "
         "and fade");
    out.present = false;
    return out;
  }
  const Value &m = *label;
  LabelStyle &s = out.style;
  out.text = m.str("text");
  if (const std::string bind = m.str("bind"); !bind.empty())
    s.text_key = cvc::ariadne::resolve_bind(prefix, bind);
  if (const Value *o = m.find("offset")) {
    if (o->is_seq() && o->items.size() >= 3)
      for (int i = 0; i < 3; ++i)
        s.offset[i] = o->items[i].as_double(0.0);
    else
      s.offset[2] = o->as_double(0.0); // a number: straight up
  }
  const std::string anchor = m.str("anchor", "top");
  if (anchor == "center" || anchor == "centre")
    s.anchor = LabelStyle::Anchor::Center;
  else if (anchor == "origin")
    s.anchor = LabelStyle::Anchor::Origin;
  else if (anchor != "top")
    warn("anchor: is top, center or origin, not '" + anchor + "'");
  if (const Value *c = m.find("color"); c && !parse_rgb(*c, s.color))
    warn("color: is [r, g, b], \"#rrggbb\" or c0..c7");
  if (const Value *b = m.find("backdrop")) {
    if (is_off(*b))
      s.backdrop = false;
    else if (!parse_rgb(*b, s.backdrop_color))
      warn("backdrop: is a colour or none");
  }
  s.backdrop_opacity =
      static_cast<float>(std::clamp(m.num("backdrop_opacity", s.backdrop_opacity), 0.0, 1.0));
  if (const Value *e = m.find("edge")) {
    if (is_off(*e))
      s.edge = false;
    else if (e->is_scalar() && (e->scalar == "true" || e->scalar == "on" || e->scalar == "yes"))
      s.edge = true;
    else if (parse_rgb(*e, s.edge_color))
      s.edge = true;
    else
      warn("edge: is a colour, true or none");
  }
  s.edge_width = static_cast<int>(std::clamp(m.num("edge_width", s.edge_width), 1.0, 16.0));
  s.size = static_cast<int>(std::clamp(m.num("size", s.size), 4.0, 256.0));
  s.bold = m.flag("bold", false);
  if (const Value *sc = m.find("scale"); sc && sc->is_scalar() && !sc->scalar.empty()) {
    const double x = sc->as_double(std::numeric_limits<double>::quiet_NaN());
    if (std::isfinite(x))
      s.scale = std::max(0.05, x);
    else
      s.scale_key = cvc::ariadne::resolve_bind(prefix, sc->scalar);
  }
  if (const Value *f = m.find("focus")) {
    // focus: <key> | { key, value, scale }: larger while the key holds the value (default: the
    // node's id).
    s.focus_value = focus_value;
    if (f->is_scalar()) {
      s.focus_key = cvc::ariadne::resolve_bind(prefix, f->scalar);
    } else if (f->is_map()) {
      if (const std::string k = f->str("key"); !k.empty())
        s.focus_key = cvc::ariadne::resolve_bind(prefix, k);
      s.focus_value = f->str("value", s.focus_value);
      s.focus_scale = std::max(0.05, f->num("scale", s.focus_scale));
    }
    if (s.focus_key.empty())
      warn("focus: names a state key");
  }
  if (const Value *fd = m.find("fade")) {
    if (fd->is_seq() && fd->items.size() >= 2) {
      s.fade_near = std::max(0.0, fd->items[0].as_double(0.0));
      s.fade_far = fd->items[1].as_double(0.0);
      if (!(s.fade_far > s.fade_near))
        warn("fade: is [near, far] with far past near");
    } else {
      warn("fade: is [near, far]");
    }
  }
  return out;
}

} // namespace

LabelStyle parse_label_style(const Value &label, const std::string &who,
                             std::vector<std::string> *warnings) {
  return parse_label(&label, who, std::string(), std::string(), warnings).style;
}

void apply_scene_label(SceneGraph &sg, const std::shared_ptr<GraphicsNode> &node,
                       const cvc::ariadne::SceneNode &n, RealizedScene &out,
                       std::vector<std::string> *warnings) {
  if (!node)
    return;
  const ParsedLabel p = parse_label(n.props.find("label"), "scene node '" + n.id + "'",
                                    out.bind_prefix, n.id, warnings);
  if (!p.present)
    return;
  // No text and no bind: the node's id.
  LabelLayerNode::of(sg)->add(node, p.text.empty() && p.style.text_key.empty() ? n.id : p.text,
                              p.style);
}

void apply_entity_label(SceneGraph &sg, const std::shared_ptr<GraphicsNode> &node,
                        const Value &style,
                        const std::vector<std::pair<std::string, std::string>> &fields,
                        const std::string &bind_prefix, std::vector<std::string> *warnings) {
  if (!node || style.empty())
    return;
  const Value *v = &style;
  Value on; // entity_labels: true -- the default look
  if (style.is_scalar() && (style.scalar == "true" || style.scalar == "on")) {
    on.kind = Value::Kind::Map;
    v = &on;
  }
  ParsedLabel p = parse_label(v, "scene entity_labels", bind_prefix, node->getName(), warnings);
  if (!p.present)
    return;
  LabelStyle s = p.style;
  std::string text = fill(p.text.empty() ? "{name}" : p.text, fields);
  if (text.empty()) // an entity with no name: its node's (vehicle_<id>)
    text = node->getName();
  s.focus_value = fill(s.focus_value, fields);
  LabelLayerNode::of(sg)->add(node, text, s);
}

} // namespace ariadne
} // namespace gl
} // namespace cvc
