/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// type: ribbon and type: draped_link (ribbon_nodes.h, docs/ARIADNE.md §5.4).

#include <algorithm>
#include <array>
#include <boost/any.hpp>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <cvc/ariadne/bind.h>                   // resolve_bind, read_node, read_effective, read_or
#include <cvc/ariadne/data.h>                   // DataStore, DataBytes, kDataRoot
#include <cvc/ariadne/scene.h>                  // SceneNode (props bag)
#include <cvc/ariadne/stream/stream.h>          // stream::registry_key
#include <cvc/ariadne/stream/stream_channel.h>  // subscription
#include <cvc/ariadne/stream/stream_registry.h> // stream_registry::for_app
#include <cvc/core/app.h>
#include <cvc/geometry/geometry.h>
#include <cvc/geometry/height_field.h>
#include <cvc/gl/DrapedLinkNode.h>
#include <cvc/gl/GraphicsNode.h>
#include <cvc/gl/HeightFieldTexture.h>
#include <cvc/gl/RibbonNode.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/ariadne/ribbon_nodes.h>
#include <cvc/gl/ariadne/scene_realize.h>
#include <cvc/state/state.h>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace sx = cvc::ariadne::stream;
using cvc::ariadne::Value;

namespace cvc {
namespace gl {
namespace ariadne {

namespace {

using Warn = std::function<void(const std::string &)>;

// --- colours -----------------------------------------------------------------

// The style slots c0..c7: the chart's categorical colours (blue, orange, aqua, yellow, magenta,
// green, violet, red), so a vehicle's track and its curve in a chart can share a slot.
constexpr unsigned kSlots[8] = {0x2a78d6, 0xeb6834, 0x1baf7a, 0xeda100,
                                0xe87ba4, 0x008300, 0x6250d6, 0xe34948};

void rgb_of(unsigned hex, float out[3]) {
  out[0] = static_cast<float>((hex >> 16) & 0xff) / 255.0f;
  out[1] = static_cast<float>((hex >> 8) & 0xff) / 255.0f;
  out[2] = static_cast<float>(hex & 0xff) / 255.0f;
}

// [r, g, b] (0..1), "#rrggbb" or a slot c0..c7. False (out untouched) for anything else.
bool parse_rgb(const Value &v, float out[3]) {
  if (v.is_seq() && v.items.size() >= 3) {
    for (int i = 0; i < 3; ++i)
      out[i] = static_cast<float>(std::clamp(v.items[i].as_double(0.0), 0.0, 1.0));
    return true;
  }
  if (!v.is_scalar())
    return false;
  const std::string &s = v.scalar;
  if (s.size() == 7 && s[0] == '#') {
    char *end = nullptr;
    const unsigned long hex = std::strtoul(s.c_str() + 1, &end, 16);
    if (end != s.c_str() + 7)
      return false;
    rgb_of(static_cast<unsigned>(hex), out);
    return true;
  }
  if (s.size() == 2 && (s[0] == 'c' || s[0] == 'C') && s[1] >= '0' && s[1] <= '7') {
    rgb_of(kSlots[s[1] - '0'], out);
    return true;
  }
  return false;
}

using Ramp = std::vector<RibbonNode::RampStop>;

RibbonNode::RampStop stop_of(float value, unsigned hex) {
  RibbonNode::RampStop s;
  s.value = value;
  float c[3];
  rgb_of(hex, c);
  s.r = c[0], s.g = c[1], s.b = c[2];
  return s;
}

// ramp: quality | heat | [[value, colour], ...] (a colour as color: takes it). A named ramp runs
// over [0, 1], or over the node's `range: [lo, hi]`.
Ramp parse_ramp(const Value &v, const Value *range, const Warn &warn) {
  Ramp out;
  if (v.is_scalar()) {
    if (v.scalar == "quality") // poor to good: red, yellow, green
      out = {stop_of(0.0f, 0xe34948), stop_of(0.5f, 0xeda100), stop_of(1.0f, 0x1baf7a)};
    else if (v.scalar == "heat") // cold to hot: blue, yellow, red
      out = {stop_of(0.0f, 0x2a78d6), stop_of(0.5f, 0xeda100), stop_of(1.0f, 0xe34948)};
    else
      warn("ramp: is quality, heat or a list of [value, colour] stops, not '" + v.scalar + "'");
    if (!out.empty() && range && range->is_seq() && range->items.size() >= 2) {
      const double lo = range->items[0].as_double(0.0), hi = range->items[1].as_double(1.0);
      for (RibbonNode::RampStop &s : out)
        s.value = static_cast<float>(lo + (hi - lo) * s.value);
    }
    return out;
  }
  if (v.is_seq())
    for (const Value &stop : v.items) {
      if (!stop.is_seq() || stop.items.size() < 2) {
        warn("ramp: a stop is [value, colour]");
        continue;
      }
      float c[3] = {1.0f, 1.0f, 1.0f};
      if (!parse_rgb(stop.items[1], c))
        warn("ramp: a stop's colour is [r, g, b], \"#rrggbb\" or c0..c7");
      RibbonNode::RampStop s;
      s.value = static_cast<float>(stop.items[0].as_double(0.0));
      s.r = c[0], s.g = c[1], s.b = c[2];
      out.push_back(s);
    }
  if (out.size() > RibbonNode::kMaxRampStops)
    warn("ramp: holds at most " + std::to_string(RibbonNode::kMaxRampStops) +
         " stops; the rest are left out");
  std::stable_sort(out.begin(), out.end(),
                   [](const RibbonNode::RampStop &a, const RibbonNode::RampStop &b) {
                     return a.value < b.value;
                   });
  return out;
}

// A value's colour through the stops: the shader's arithmetic (RibbonNode's ramp), on the CPU.
std::array<float, 3> ramp_color(const Ramp &ramp, double v) {
  if (ramp.empty())
    return {1.0f, 1.0f, 1.0f};
  const std::size_t n = std::min(ramp.size(), RibbonNode::kMaxRampStops);
  std::array<float, 3> c = {ramp[0].r, ramp[0].g, ramp[0].b};
  for (std::size_t i = 1; i < n; ++i) {
    const RibbonNode::RampStop &a = ramp[i - 1], &b = ramp[i];
    if (!(v > a.value))
      break;
    const double span = std::max(static_cast<double>(b.value - a.value), 1e-20);
    const float f = static_cast<float>(std::clamp((v - a.value) / span, 0.0, 1.0));
    c = {a.r + (b.r - a.r) * f, a.g + (b.g - a.g) * f, a.b + (b.b - a.b) * f};
  }
  return c;
}

// --- a number, or a state key read each frame ----------------------------------

struct Bound {
  bool present = false;
  bool literal = true;
  double value = 0.0; // the literal, or the default while the key holds no number
  std::string path;   // an absolute state path

  static Bound of(const Value *v, double dflt, const std::string &prefix) {
    Bound b;
    b.value = dflt;
    if (!v || !v->is_scalar() || v->scalar.empty())
      return b;
    b.present = true;
    const double x = v->as_double(std::numeric_limits<double>::quiet_NaN());
    if (std::isfinite(x)) {
      b.value = x;
    } else {
      b.literal = false;
      b.path = cvc::ariadne::resolve_bind(prefix, v->scalar);
    }
    return b;
  }
  double read(cvc::app &app) const {
    if (literal)
      return value;
    const double x = cvc::ariadne::read_or<double>(app, path, value);
    return std::isfinite(x) ? x : value;
  }
};

// --- floats from a source: ------------------------------------------------------

bool is_sep(char c) {
  return c == ',' || c == ';' || c == '[' || c == ']' || c == '(' || c == ')' ||
         std::isspace(static_cast<unsigned char>(c));
}

// The numbers in s[from, end), appended to out: separated by commas, semicolons, brackets or
// white space; anything that is not a number is skipped.
void parse_floats(const std::string &s, std::size_t from, std::vector<float> &out) {
  const char *p = s.c_str() + from, *end = s.c_str() + s.size();
  while (p < end) {
    while (p < end && is_sep(*p))
      ++p;
    if (p >= end)
      break;
    char *q = nullptr;
    const double x = std::strtod(p, &q);
    if (q == p) { // not a number: skip the token
      while (p < end && !is_sep(*p))
        ++p;
      continue;
    }
    out.push_back(static_cast<float>(x));
    p = q;
  }
}

// The first index at which a and b differ (min size when one is a prefix of the other).
std::size_t first_difference(const std::vector<float> &a, const std::vector<float> &b) {
  const std::size_t n = std::min(a.size(), b.size());
  std::size_t i = 0;
  // Whole blocks by memcmp first: a long track compares in microseconds.
  constexpr std::size_t kBlock = 256;
  while (i + kBlock <= n && std::memcmp(a.data() + i, b.data() + i, kBlock * sizeof(float)) == 0)
    i += kBlock;
  while (i < n && std::memcmp(a.data() + i, b.data() + i, sizeof(float)) == 0)
    ++i;
  return i;
}

// What a node's source: names, polled each frame for floats: a literal list (points:), a state
// key (its data(): floats, doubles, float32 bytes or a mesh's points; else its value as text), a
// dataset's output (a state key under the data store), or a stream of float32 frames.
class FloatSource {
public:
  enum class Kind { none, literal, state, stream };

  explicit FloatSource(cvc::app &app) : _app(app) {}
  ~FloatSource() { release(); }
  FloatSource(const FloatSource &) = delete;
  FloatSource &operator=(const FloatSource &) = delete;

  Kind kind() const { return _kind; }
  const std::vector<float> &floats() const { return _floats; }

  void set_literal(std::vector<float> floats) {
    _kind = Kind::literal;
    _pending = std::move(floats);
  }
  void set_state(const std::string &path) {
    _kind = Kind::state;
    _path = path;
  }
  // `token` is a stream's token (the key (stream-open) returns), or its id: opened by the
  // document (its scope, `prefix`) or at the root.
  void set_stream(const std::string &token, const std::string &prefix, bool replace,
                  std::size_t depth) {
    _kind = Kind::stream;
    _tokens = {token, sx::stream::registry_key(prefix, token), sx::stream::registry_key("", token)};
    _replace = replace;
    _depth = std::max<std::size_t>(depth, 1);
  }
  // The node's source: { state: <key> } or { stream: <token>, replace, depth } (in its props, as
  // written), or { data: <name>, item: <output> } (parsed into source_data / source_item). False
  // (and a warning) when it has none of them.
  bool configure(const cvc::ariadne::SceneNode &n, const std::string &prefix, const Warn &warn) {
    if (!n.source_data.empty())
      return configure_data(n.source_data, n.source_item, prefix);
    const Value *src = n.props.find("source");
    if (src && src->is_map()) {
      if (const std::string key = src->str("state"); !key.empty()) {
        set_state(cvc::ariadne::resolve_bind(prefix, key));
        return true;
      }
      if (const std::string token = src->str("stream"); !token.empty()) {
        set_stream(token, prefix, src->flag("replace", false),
                   static_cast<std::size_t>(std::max(1.0, src->num("depth", 4.0))));
        return true;
      }
    }
    warn("source: is { state: <key> }, { stream: <token> } or { data: <name>, item: <output> }");
    return false;
  }

  // A dataset's output, read as a state key: <prefix>.data.<name>[.<item>].
  bool configure_data(const std::string &name, const std::string &item, const std::string &prefix) {
    const std::string store = cvc::ariadne::resolve_bind(
        prefix, name[0] == '/' ? name : std::string(cvc::ariadne::kDataRoot) + "." + name);
    set_state(item.empty() ? store : store + cvc::state::SEPARATOR + item);
    cvc::ariadne::DataStore::of(_app)->touch(store); // a lazy dataset loads once asked for
    return true;
  }

  // Read the source when it changed. True when the floats did; `same` is then the index from
  // which they may differ from the floats of the last poll that returned true.
  bool poll(std::size_t &same) {
    switch (_kind) {
    case Kind::literal:
      if (_literal_done)
        return false;
      _literal_done = true;
      _floats.swap(_pending);
      same = 0;
      return true;
    case Kind::state:
      return poll_state(same);
    case Kind::stream:
      return poll_stream(same);
    default:
      return false;
    }
  }

  // A stream that would not admit this reader (its frame pool is full): said once.
  bool refused() const { return _refused; }

private:
  bool replace_with(std::vector<float> &next, std::size_t &same) {
    same = first_difference(_floats, next);
    if (same == next.size() && next.size() == _floats.size())
      return false; // the same floats again
    _floats.swap(next);
    return true;
  }

  bool poll_state(std::size_t &same) {
    cvc::state *found = cvc::ariadne::read_node(_app, _path);
    if (!found) {
      if (_floats.empty())
        return false;
      _floats.clear();
      _text.clear();
      _node = nullptr;
      same = 0;
      return true;
    }
    cvc::state &node = cvc::ariadne::read_effective(*found);
    const boost::posix_time::ptime mod = node.lastMod();
    if (&node == _node && mod == _mod)
      return false; // untouched since the last read: the per-frame fast path
    _node = &node;
    _mod = mod;
    std::vector<float> next;
    const boost::any d = node.data();
    if (!d.empty() && floats_of(d, next)) {
      _text.clear();
      return replace_with(next, same);
    }
    const std::string text = node.value();
    if (text == _text)
      return false;
    // Text that only grew past a complete number: parse just what was added.
    if (!_text.empty() && text.size() > _text.size() && text.compare(0, _text.size(), _text) == 0 &&
        (is_sep(_text.back()) || is_sep(text[_text.size()]))) {
      same = _floats.size();
      parse_floats(text, _text.size(), _floats);
      _text = text;
      return true;
    }
    parse_floats(text, 0, next);
    _text = text;
    return replace_with(next, same);
  }

  // data() as floats: std::vector<float|double> (or a shared_ptr to one), float32 bytes
  // (DataBytes), or a mesh's points (x, y, z each).
  static bool floats_of(const boost::any &d, std::vector<float> &out) {
    using fvec = std::vector<float>;
    using dvec = std::vector<double>;
    const auto from_f = [&](const fvec &v) { out.assign(v.begin(), v.end()); };
    const auto from_d = [&](const dvec &v) { out.assign(v.begin(), v.end()); };
    if (const auto *v = boost::any_cast<std::shared_ptr<const fvec>>(&d)) {
      if (*v)
        from_f(**v);
      return true;
    }
    if (const auto *v = boost::any_cast<std::shared_ptr<fvec>>(&d)) {
      if (*v)
        from_f(**v);
      return true;
    }
    if (const auto *v = boost::any_cast<fvec>(&d)) {
      from_f(*v);
      return true;
    }
    if (const auto *v = boost::any_cast<std::shared_ptr<const dvec>>(&d)) {
      if (*v)
        from_d(**v);
      return true;
    }
    if (const auto *v = boost::any_cast<dvec>(&d)) {
      from_d(*v);
      return true;
    }
    if (const auto *v = boost::any_cast<cvc::ariadne::DataBytes>(&d)) {
      if (*v) {
        out.resize((*v)->size() / sizeof(float));
        std::memcpy(out.data(), (*v)->data(), out.size() * sizeof(float));
      }
      return true;
    }
    const auto from_mesh = [&](const cvc::geometry &g) {
      const auto &pts = g.const_points();
      out.resize(3 * pts.size());
      for (std::size_t i = 0; i < pts.size(); ++i)
        for (int k = 0; k < 3; ++k)
          out[3 * i + k] = static_cast<float>(pts[i][k]);
    };
    if (const auto *v = boost::any_cast<std::shared_ptr<const cvc::geometry>>(&d)) {
      if (*v)
        from_mesh(**v);
      return true;
    }
    if (const auto *v = boost::any_cast<std::shared_ptr<cvc::geometry>>(&d)) {
      if (*v)
        from_mesh(**v);
      return true;
    }
    return false;
  }

  bool poll_stream(std::size_t &same) {
    if (!_sub) {
      if (_refused)
        return false;
      sx::stream_channel *ch = nullptr;
      for (const std::string &t : _tokens)
        if (!ch && (ch = sx::stream_registry::for_app(_app).lookup(t)))
          _token = t;
      if (!ch)
        return false; // not open yet: look again next frame
      _sub = ch->subscribe(_replace ? sx::deliver_mode::latest : sx::deliver_mode::ring, _depth);
      if (!_sub) {
        _refused = true;
        return false;
      }
    }
    const auto floats_in = [](const sx::frame &f, std::vector<float> &out) {
      const std::size_t n = f.size / sizeof(float), at = out.size();
      out.resize(at + n);
      if (n)
        std::memcpy(out.data() + at, f.data, n * sizeof(float));
    };
    if (_replace) { // each frame is the whole line
      const sx::frame_ptr f = _sub->latest();
      if (!f || f->seq == _seq)
        return false;
      _seq = f->seq;
      std::vector<float> next;
      floats_in(*f, next);
      return replace_with(next, same);
    }
    // Each frame appends its points.
    same = _floats.size();
    bool any = false;
    sx::frame_ptr f;
    while (_sub->try_pop(f)) {
      if (f)
        floats_in(*f, _floats);
      any = true;
    }
    return any;
  }

  void release() {
    if (!_sub)
      return;
    // Resolve the channel again rather than keep a pointer that a close would leave dangling
    // (StreamTextureBinding does the same).
    if (sx::stream_channel *ch = sx::stream_registry::for_app(_app).lookup(_token))
      ch->unsubscribe(_sub);
    _sub->close();
    _sub.reset();
  }

  cvc::app &_app;
  Kind _kind = Kind::none;
  std::vector<float> _floats;
  // literal
  std::vector<float> _pending;
  bool _literal_done = false;
  // state
  std::string _path;
  const cvc::state *_node = nullptr; // compared, never dereferenced
  boost::posix_time::ptime _mod;
  std::string _text;
  // stream
  std::vector<std::string> _tokens; // what to look the stream up by, in turn
  std::string _token;               // the one it was found by
  bool _replace = false;
  std::size_t _depth = 4;
  std::shared_ptr<sx::subscription> _sub;
  std::int64_t _seq = -1;
  bool _refused = false;
};

// points: [[x, y, z], ...] or [x, y, z, x, y, z, ...] — the floats as written.
std::vector<float> literal_floats(const Value &v) {
  std::vector<float> out;
  if (v.is_seq())
    for (const Value &item : v.items) {
      if (item.is_seq())
        for (const Value &x : item.items)
          out.push_back(static_cast<float>(x.as_double(0.0)));
      else
        out.push_back(static_cast<float>(item.as_double(0.0)));
    }
  return out;
}

// --- the ground ---------------------------------------------------------------------

// A height field's height at (x, y), clamped onto its grid (height() is NaN off it); `fallback`
// over a hole.
double ground_height(const cvc::height_field &f, double x, double y, double fallback) {
  x = std::clamp(x, f.x0(), f.max_x());
  y = std::clamp(y, f.y0(), f.max_y());
  const double h = f.height(x, y);
  return std::isfinite(h) ? h : fallback;
}

// The height textures links drape over, one per ground height field, shared by its links and
// made again when the ground is (a regenerated world): an app's own, held weakly.
struct GroundTextures {
  static constexpr const char *app_slot = "cvc.gl.ariadne.ribbon-ground-textures";
  struct Entry {
    std::weak_ptr<const cvc::height_field> field;
    std::weak_ptr<HeightFieldTexture> tex;
    const void *heights = nullptr; // the heights the texture holds (compared, not read)
    int rows = 0, cols = 0;
    double x0 = 0, y0 = 0, dx = 0, dy = 0;
  };
  std::mutex m;
  std::map<const cvc::height_field *, Entry> entries;

  static std::shared_ptr<const std::vector<float>> heights_of(const cvc::height_field &f) {
    auto h = std::make_shared<std::vector<float>>(f.heights());
    for (float &v : *h) // a hole would make the drape NaN
      if (!std::isfinite(v))
        v = 0.0f;
    return h;
  }

  // The texture for `field` as it is now: the same one while its grid is, its heights replaced
  // (one upload, for all its links) when they changed.
  std::shared_ptr<HeightFieldTexture> get(const std::shared_ptr<const cvc::height_field> &field) {
    std::lock_guard<std::mutex> lock(m);
    for (auto it = entries.begin(); it != entries.end();)
      it = it->second.field.expired() || it->second.tex.expired() ? entries.erase(it) : ++it;
    Entry &e = entries[field.get()];
    std::shared_ptr<HeightFieldTexture> tex = e.tex.lock();
    const cvc::height_field &f = *field;
    const bool same_grid = tex && e.field.lock() == field && e.rows == f.rows() &&
                           e.cols == f.cols() && e.x0 == f.x0() && e.y0 == f.y0() &&
                           e.dx == f.dx() && e.dy == f.dy();
    if (!same_grid) {
      tex =
          std::make_shared<HeightFieldTexture>(f.cols(), f.rows(), f.x0(), f.y0(), f.dx(), f.dy());
      tex->setHeights(heights_of(f));
      e = Entry{field, tex, &f.heights(), f.rows(), f.cols(), f.x0(), f.y0(), f.dx(), f.dy()};
    } else if (e.heights != &f.heights()) {
      tex->setHeights(heights_of(f));
      e.heights = &f.heights();
    }
    return tex;
  }
};

// A flat ground at z = 0 under the scene as it stands (a link with no ground: of its own).
std::shared_ptr<HeightFieldTexture> flat_ground(SceneGraph &sg) {
  cvc::bounding_box bb = sg.computeGraphicsBounds();
  double x0 = -1000, y0 = -1000, x1 = 1000, y1 = 1000;
  if (!bb.isNull()) {
    x0 = std::min(x0, bb.minx - 100), y0 = std::min(y0, bb.miny - 100);
    x1 = std::max(x1, bb.maxx + 100), y1 = std::max(y1, bb.maxy + 100);
  }
  return std::make_shared<HeightFieldTexture>(2, 2, x0, y0, x1 - x0, y1 - y0);
}

// The heightfield node `ground` names, realized before this node; null (and a warning) when the
// scene has none by that name yet.
std::shared_ptr<const cvc::height_field> find_ground(const RealizedScene &out,
                                                     const std::string &ground, const Warn &warn) {
  if (ground.empty())
    return nullptr;
  const auto it = out.height_fields.find(ground);
  if (it != out.height_fields.end() && it->second)
    return it->second;
  warn("ground: '" + ground +
       "' is not a heightfield or generated_world node declared before this one");
  return nullptr;
}

// A node anywhere in the scene, by id: a top-level node, else the first in the tree.
std::shared_ptr<GraphicsNode> find_graphics(SceneGraph &sg, const std::string &id) {
  if (sg.hasGraphics(id))
    return sg.getGraphics(id);
  std::vector<std::shared_ptr<GraphicsNode>> stack = {sg.getGraphicsRoot()};
  while (!stack.empty()) {
    std::shared_ptr<GraphicsNode> n = stack.back();
    stack.pop_back();
    if (!n)
      continue;
    if (n->getName() == id && n != sg.getGraphicsRoot())
      return n;
    for (const auto &c : n->getGraphicsChildren())
      stack.push_back(c);
  }
  return nullptr;
}

std::string lowered(std::string s) {
  std::transform(s.begin(), s.end(), s.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return s;
}

void flat_color(GeometryNode &g, const float rgb[3]) {
  g.setColor(rgb[0], rgb[1], rgb[2]);
  g.setAmbient(1.0);
  g.setDiffuse(0.0);
  g.setSpecular(0.0);
}

// --- type: ribbon ----------------------------------------------------------------------

struct RibbonBinding {
  explicit RibbonBinding(cvc::app &a) : app(a), src(a) {}

  cvc::app &app;
  std::weak_ptr<RibbonNode> node;
  FloatSource src;
  // fields: which float of a point is x, y, z, its width (w) and its value (v); -1: absent.
  int ix = 0, iy = 1, iz = 2, iw = -1, iv = -1;
  std::size_t stride = 3;
  float half = 1.0f, lift = 0.0f;
  std::shared_ptr<const cvc::height_field> ground;
  const void *ground_heights = nullptr; // the heights the drawn points were draped on
  std::size_t drawn = 0;                // points drawn
  // window:
  bool has_window = false, metres = false;
  Bound from, to, tail;
  double win_first = -1.0, win_last = -1.0;
  // scratch
  std::vector<float> xyz, halfs, values;

  void tick() {
    std::shared_ptr<RibbonNode> rib = node.lock();
    if (!rib)
      return;
    bool redrape = false;
    if (ground && &ground->heights() != ground_heights) { // the ground was made again
      redrape = drawn > 0;
      ground_heights = &ground->heights();
    }
    std::size_t same = 0;
    const bool changed = src.poll(same);
    if (changed || redrape)
      draw(*rib, redrape ? 0 : same);
    if (has_window)
      update_window(*rib);
  }

  // Draw the source's points from the first one at or past float `same` that is not drawn yet.
  void draw(RibbonNode &rib, std::size_t same) {
    const std::vector<float> &f = src.floats();
    const std::size_t n = f.size() / stride;
    const std::size_t d = std::min(std::min(same / stride, drawn), n); // the first point to write
    const std::size_t m = n - d;
    xyz.resize(3 * m);
    halfs.resize(iw >= 0 ? m : 0);
    values.resize(iv >= 0 ? m : 0);
    float px = 0.0f, py = 0.0f, pz = 0.0f; // the last finite point: a NaN point repeats it
    if (d > 0) {
      const std::vector<float> prev = rib.centerVertices(d - 1); // left and right of centre d-1
      px = 0.5f * (prev[0] + prev[3]), py = 0.5f * (prev[1] + prev[4]), pz = prev[2] - lift;
    }
    for (std::size_t k = 0; k < m; ++k) {
      const float *p = &f[(d + k) * stride];
      float x = p[ix], y = p[iy], z = iz >= 0 ? p[iz] : 0.0f;
      if (!std::isfinite(x) || !std::isfinite(y)) {
        x = px, y = py;
      }
      if (ground)
        z = static_cast<float>(ground_height(*ground, x, y, z));
      if (!std::isfinite(z))
        z = pz;
      px = x, py = y, pz = z;
      xyz[3 * k] = x, xyz[3 * k + 1] = y, xyz[3 * k + 2] = z + lift;
      if (iw >= 0)
        halfs[k] = std::isfinite(p[iw]) ? 0.5f * p[iw] : half;
      if (iv >= 0)
        values[k] = std::isfinite(p[iv]) ? p[iv] : 0.0f;
    }
    rib.replaceFrom(d, xyz.data(), m, iw >= 0 ? halfs.data() : nullptr,
                    iv >= 0 ? values.data() : nullptr);
    drawn = n;
  }

  // window: what part of the line is drawn, in points or metres (no upload: a draw range and
  // two uniforms).
  void update_window(RibbonNode &rib) {
    const std::size_t n = rib.centerCount();
    double first = 0.0, last = RibbonNode::kToEnd;
    const auto at = [&](double v) { return metres ? rib.centerAtArcLength(v) : std::max(0.0, v); };
    if (tail.present) {
      const double t = std::max(0.0, tail.read(app));
      first = metres ? rib.centerAtArcLength(std::max(0.0, rib.arcLength() - t))
                     : std::max(0.0, static_cast<double>(n) - 1.0 - t);
    } else {
      if (from.present)
        first = at(from.read(app));
      if (to.present)
        last = at(to.read(app));
    }
    if (first == win_first && last == win_last)
      return;
    win_first = first;
    win_last = last;
    rib.setVisibleCenters(first, last);
  }
};

// fields: the letters x, y (both needed), z, w and v, each at most once, in the order a point's
// floats come in.
bool parse_fields(const std::string &fields, RibbonBinding &b) {
  int at[5] = {-1, -1, -1, -1, -1}; // x y z w v
  const std::string letters = "xyzwv";
  for (std::size_t i = 0; i < fields.size(); ++i) {
    const std::size_t k = letters.find(fields[i]);
    if (k == std::string::npos || at[k] >= 0)
      return false;
    at[k] = static_cast<int>(i);
  }
  if (at[0] < 0 || at[1] < 0)
    return false;
  b.ix = at[0], b.iy = at[1], b.iz = at[2], b.iw = at[3], b.iv = at[4];
  b.stride = fields.size();
  return true;
}

std::shared_ptr<GraphicsNode> realize_ribbon(SceneGraph &sg, const cvc::ariadne::SceneNode &n,
                                             GraphicsNode *parent, RealizedScene &out,
                                             std::vector<std::string> *warnings) {
  const Warn warn = [&](const std::string &m) {
    if (warnings)
      warnings->push_back("ari: ribbon node '" + n.id + "': " + m);
  };
  cvc::app &app = sg.appContext();
  const Value &p = n.props;
  auto b = std::make_shared<RibbonBinding>(app);

  const std::string fields = lowered(p.str("fields", "xyz"));
  if (!parse_fields(fields, *b))
    warn("fields: '" + fields + "' is x, y and any of z, w, v, each once (xyz assumed)");
  if (const Value *pts = p.find("points"))
    b->src.set_literal(literal_floats(*pts));
  else if (!b->src.configure(n, out.bind_prefix, warn))
    warn("has no source: or points:, so it draws nothing");

  b->half = static_cast<float>(0.5 * std::max(0.0, p.num("width", 2.0)));
  b->lift = static_cast<float>(p.num("lift", 0.0));
  b->ground = find_ground(out, p.str("ground"), warn);
  const auto capacity = static_cast<std::size_t>(std::clamp(p.num("capacity", 256.0), 2.0, 1e7));

  GraphicsNode *pr = parent ? parent : sg.getGraphicsRoot().get();
  if (!parent && sg.hasGraphics(n.id)) // last wins, as for the built-ins
    sg.removeGraphics(n.id);
  std::shared_ptr<RibbonNode> rib =
      pr->addGraphicsChild<RibbonNode>(n.id, capacity, b->half, cvc::bounding_box());
  if (!parent)
    sg.registerGraphics(n.id, rib);

  if (n.has_material) {
    apply_node_material(*rib, n);
  } else {
    float rgb[3];
    rgb_of(kSlots[0], rgb);
    if (const Value *c = p.find("color"); c && !parse_rgb(*c, rgb))
      warn("color: is [r, g, b], \"#rrggbb\" or c0..c7");
    flat_color(*rib, rgb);
  }
  if (p.has("opacity"))
    rib->setOpacity(std::clamp(p.num("opacity", 1.0), 0.0, 1.0));
  if (p.has("depth_offset"))
    rib->setDepthOffset(p.num("depth_offset", 0.0));
  if (const Value *r = p.find("ramp")) {
    const Ramp ramp = parse_ramp(*r, p.find("range"), warn);
    if (b->iv < 0)
      warn("ramp: colours by each point's value, and fields: '" + fields + "' has no v");
    else if (!ramp.empty())
      rib->setValueRamp(ramp);
  }
  if (const Value *w = p.find("window"); w && w->is_map()) {
    b->has_window = true;
    const std::string units = lowered(w->str("units", "point"));
    b->metres = units == "m" || units == "metres" || units == "meters";
    if (!b->metres && units != "point" && units != "points")
      warn("window: units is point or m, not '" + units + "'");
    b->from = Bound::of(w->find("from"), 0.0, out.bind_prefix);
    const Value *to = w->find("to");
    if (!(to && to->is_scalar() && to->scalar == "end"))
      b->to = Bound::of(to, RibbonNode::kToEnd, out.bind_prefix);
    b->tail = Bound::of(w->find("tail"), 0.0, out.bind_prefix);
  }

  b->node = rib;
  b->ground_heights = b->ground ? &b->ground->heights() : nullptr;
  b->tick(); // what the source holds now
  out.custom_ticks.push_back([b](vtkRenderer *) { b->tick(); });
  return rib;
}

// --- type: draped_link ---------------------------------------------------------------

struct LinkBinding {
  explicit LinkBinding(cvc::app &a) : app(a), src(a), from(a), to(a) {}

  cvc::app &app;
  SceneGraph *sg = nullptr; // the scene the realizer ran in; outlives its nodes' ticks
  std::weak_ptr<DrapedLinkNode> node;
  std::shared_ptr<const cvc::height_field> ground;
  const void *ground_heights = nullptr; // the heights the link's texture was taken from
  // Endpoints: two nodes (between:), from:/to: (each a literal or a key), or source: (4 floats).
  std::string ends[2];
  std::weak_ptr<GraphicsNode> end_nodes[2];
  FloatSource src, from, to;
  // Style.
  float half = 1.0f, lift = 0.5f;
  float rgb[3] = {1, 1, 1};
  Ramp ramp;
  Bound value, opacity;
  // What was last handed to the node (NaN: nothing yet).
  float drawn_ends[4] = {NAN, NAN, NAN, NAN};
  float drawn_style[4] = {NAN, NAN, NAN, NAN}; // r, g, b, opacity

  bool end_of(int i, float xy[2], const std::shared_ptr<DrapedLinkNode> &link) {
    std::shared_ptr<GraphicsNode> e = end_nodes[i].lock();
    if (!e) {
      e = find_graphics(*sg, ends[i]);
      if (!e)
        return false; // not there (yet)
      end_nodes[i] = e;
    }
    const double origin[3] = {0, 0, 0};
    double world[3], local[3];
    e->localToWorld(origin, world);
    link->worldToLocal(world, local); // the link's own frame: the one it drapes in
    xy[0] = static_cast<float>(local[0]);
    xy[1] = static_cast<float>(local[1]);
    return true;
  }

  bool endpoints(float e[4], const std::shared_ptr<DrapedLinkNode> &link) {
    if (!ends[0].empty())
      return end_of(0, e, link) && end_of(1, e + 2, link);
    std::size_t same = 0;
    if (src.kind() != FloatSource::Kind::none) {
      src.poll(same);
      const std::vector<float> &f = src.floats();
      if (f.size() < 4)
        return false;
      std::copy(f.begin(), f.begin() + 4, e);
      return true;
    }
    from.poll(same);
    to.poll(same);
    if (from.floats().size() < 2 || to.floats().size() < 2)
      return false;
    e[0] = from.floats()[0], e[1] = from.floats()[1];
    e[2] = to.floats()[0], e[3] = to.floats()[1];
    return true;
  }

  void tick() {
    std::shared_ptr<DrapedLinkNode> link = node.lock();
    if (!link)
      return;
    if (ground && &ground->heights() != ground_heights) { // the ground made again
      ground_heights = &ground->heights();
      link->setHeightField(app.slot<GroundTextures>().get(ground));
    }
    float e[4];
    if (endpoints(e, link) && std::all_of(e, e + 4, [](float v) { return std::isfinite(v); }) &&
        !std::equal(e, e + 4, drawn_ends)) {
      link->setEndpoints(e[0], e[1], e[2], e[3]);
      std::copy(e, e + 4, drawn_ends);
    }
    float style[4] = {rgb[0], rgb[1], rgb[2], 1.0f};
    if (!ramp.empty()) {
      const std::array<float, 3> c = ramp_color(ramp, value.read(app));
      std::copy(c.begin(), c.end(), style);
    }
    style[3] = static_cast<float>(std::clamp(opacity.read(app), 0.0, 1.0));
    if (!std::equal(style, style + 4, drawn_style)) {
      link->setStyle(half, lift, style[0], style[1], style[2], style[3]);
      std::copy(style, style + 4, drawn_style);
    }
  }
};

// from: / to: — [x, y], or a state key holding "x, y".
bool configure_end(FloatSource &s, const Value *v, const std::string &prefix) {
  if (!v)
    return false;
  if (v->is_seq()) {
    s.set_literal(literal_floats(*v));
    return true;
  }
  if (v->is_scalar() && !v->scalar.empty()) {
    s.set_state(cvc::ariadne::resolve_bind(prefix, v->scalar));
    return true;
  }
  return false;
}

std::shared_ptr<GraphicsNode> realize_link(SceneGraph &sg, const cvc::ariadne::SceneNode &n,
                                           GraphicsNode *parent, RealizedScene &out,
                                           std::vector<std::string> *warnings) {
  const Warn warn = [&](const std::string &m) {
    if (warnings)
      warnings->push_back("ari: draped_link node '" + n.id + "': " + m);
  };
  cvc::app &app = sg.appContext();
  const Value &p = n.props;
  auto b = std::make_shared<LinkBinding>(app);
  b->sg = &sg;

  if (const Value *bt = p.find("between"); bt && bt->is_seq() && bt->items.size() == 2) {
    b->ends[0] = bt->items[0].as_string();
    b->ends[1] = bt->items[1].as_string();
    if (b->ends[0].empty() || b->ends[1].empty())
      warn("between: names two nodes");
  } else if (p.has("source") || !n.source_data.empty()) {
    b->src.configure(n, out.bind_prefix, warn);
  } else if (!configure_end(b->from, p.find("from"), out.bind_prefix) ||
             !configure_end(b->to, p.find("to"), out.bind_prefix)) {
    warn("needs between: [a, b], from: and to:, or a source: of x0, y0, x1, y1");
  }

  b->half = static_cast<float>(0.5 * std::max(0.0, p.num("width", 2.0)));
  b->lift = static_cast<float>(p.num("lift", 0.5));
  rgb_of(kSlots[0], b->rgb);
  if (const Value *c = p.find("color"); c && !parse_rgb(*c, b->rgb))
    warn("color: is [r, g, b], \"#rrggbb\" or c0..c7");
  if (const Value *r = p.find("ramp")) {
    b->ramp = parse_ramp(*r, p.find("range"), warn);
    b->value = Bound::of(p.find("value"), 0.0, out.bind_prefix);
    if (!b->value.present)
      warn("ramp: colours by value:, which this node lacks");
  }
  b->opacity = Bound::of(p.find("opacity"), 1.0, out.bind_prefix);

  b->ground = find_ground(out, p.str("ground"), warn);
  const std::shared_ptr<HeightFieldTexture> heights =
      b->ground ? app.slot<GroundTextures>().get(b->ground) : flat_ground(sg);
  const int stations = static_cast<int>(std::clamp(p.num("stations", 24.0), 2.0, 4096.0));

  GraphicsNode *pr = parent ? parent : sg.getGraphicsRoot().get();
  if (!parent && sg.hasGraphics(n.id))
    sg.removeGraphics(n.id);
  std::shared_ptr<DrapedLinkNode> link =
      pr->addGraphicsChild<DrapedLinkNode>(n.id, heights, stations);
  if (!parent)
    sg.registerGraphics(n.id, link);

  b->node = link;
  b->ground_heights = b->ground ? &b->ground->heights() : nullptr;
  b->tick();
  out.custom_ticks.push_back([b](vtkRenderer *) { b->tick(); });
  return link;
}

} // namespace

void register_ribbon_node_types(cvc::app &app) {
  NodeTypeOptions reads_data;
  reads_data.reads_data = true; // source: { data } is a ribbon's own points
  if (!has_scene_node_type(app, "ribbon"))
    register_scene_node_type(app, "ribbon", realize_ribbon, reads_data);
  if (!has_scene_node_type(app, "draped_link"))
    register_scene_node_type(app, "draped_link", realize_link, reads_data);
}

} // namespace ariadne
} // namespace gl
} // namespace cvc
