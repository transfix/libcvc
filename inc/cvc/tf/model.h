/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.

  libcvc is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

// cvc::tf::model — the renderer-agnostic 1D transfer-function model the Ariadne `tf_editor`
// widget edits (the VolumeRover2 ColorTable2 model, roadmap CVCGL-UI-DSL §10).
//
// Three independent, position-sorted node lists over a NORMALIZED scalar axis t in [0,1]:
//   color        t -> rgb      (piecewise-linear)
//   alpha        t -> opacity  (piecewise-linear, optionally cubed)
//   isocontours  t -> an isovalue with a stable string id; they do NOT feed the colour table —
//                renderers that draw isosurfaces (volren, fe_lab, ...) extract them.
// The scalar axis maps onto data units through a domain: `auto` (follow the consumer's data
// range) or an explicit [min, max]. A consumer turns t into a data value with to_data().
//
// The model rides cvc::state as ONE string (to_string / from_string, a small JSON document), so a
// widget, a document, a script and every renderer adapter read and write the same key, and it
// persists, replicates and watches like any other value. See docs/TRANSFER_FUNCTIONS.md.

#ifndef CVC_TF_MODEL_H
#define CVC_TF_MODEL_H

#include <cstdint>
#include <string>
#include <vector>

namespace cvc {
namespace tf {

// A colour control node: rgb (each channel 0..1) at normalized position `at`.
struct color_node {
  double at = 0.0;
  float r = 0.f, g = 0.f, b = 0.f;
};

// An opacity control node: opacity `a` (0..1) at normalized position `at`.
struct alpha_node {
  double at = 0.0;
  float a = 0.f;
};

// An isocontour (isosurface) at normalized position `at`. `id` is stable across edits (the
// renderer adapters and scripts key on it). The surface colour defaults to the colour ramp at
// `at` and its opacity to 1; `has_rgb` / `has_opacity` override them.
struct isocontour {
  std::string id;
  double at = 0.0;
  bool has_rgb = false;
  float r = 1.f, g = 1.f, b = 1.f;
  bool has_opacity = false;
  float opacity = 1.f;
};

// The scalar domain the normalized axis spans, in data units. `automatic` follows the consumer's
// data range (published at <tf>.range); otherwise [min, max] is explicit.
struct domain_spec {
  bool automatic = true;
  double min = 0.0, max = 1.0;
};

// One RGBA sample (each channel 0..1).
struct rgba {
  float r = 0.f, g = 0.f, b = 0.f, a = 0.f;
};

class model {
public:
  std::vector<color_node> color;       // sorted by `at`; endpoints at 0 and 1 after normalize()
  std::vector<alpha_node> alpha;       // sorted by `at`; endpoints at 0 and 1 after normalize()
  std::vector<isocontour> isocontours; // sorted by `at`; ids unique after normalize()
  domain_spec domain;
  bool opacity_cubed = false; // ColorTable2's v^3 low-opacity emphasis, applied when sampling
  int resolution = 256;       // the default baked-table size consumers use

  // ColorTable2's default: black->red->yellow->green colour, a 0/.75/.25/1 opacity ramp, no
  // isocontours. Already normalized.
  static model default_model();

  // A named preset (normalized): "default", "grayscale", "rainbow", "hot", "cool", "xray",
  // "isolines" (transparent with two isocontours). Unknown names return default_model().
  // Colours are volrover3's (grayscale black->white; rainbow blue/cyan/green/yellow/red at
  // 0/.25/.5/.75/1; hot black/red/yellow/white at 0/.33/.66/1; cool cyan->magenta). Opacity:
  // grayscale, rainbow, hot and cool start from volrover3's linear 0 / .5 / 1 ramp (the midpoint
  // is a ready handle); xray is grayscale with a faint 0 / .125 / .25 ramp so many samples
  // accumulate like a radiograph; isolines is the rainbow ramp, opacity 0 everywhere, with
  // isocontours iso1 at .25 and iso2 at .75 (drawn cyan and yellow).
  static model preset(const std::string &name);
  static const std::vector<std::string> &preset_names();

  // Sort every list, clamp positions to [0,1], insert missing colour/alpha endpoints (ColorTable2's
  // rule: a missing 0 endpoint copies the first in-range node, a missing 1 endpoint is white /
  // opacity 1), drop out-of-range colour/alpha nodes, clamp channels to [0,1], and make isocontour
  // ids non-empty and unique. Idempotent.
  // Details: of colour/alpha nodes at the same position the first in list order wins, and a NaN
  // position drops the node; isocontours may share a position. An empty id becomes
  // next_isocontour_id(); the LATER of two equal ids becomes "<id>_2" ("_3", ...). An explicit
  // domain with min > max is swapped, a non-finite one becomes automatic; a resolution < 2 is
  // reset to 256 and one above 65536 clamped to it.
  void normalize();

  // Piecewise-linear samples at normalized t (clamped to [0,1]). alpha_at() applies opacity_cubed.
  // An empty list samples as black / transparent.
  void color_at(double t, float rgb[3]) const;
  float alpha_at(double t) const;
  rgba sample(double t) const;

  // The colour / opacity an isocontour is drawn with: its override, else the ramp at its position
  // (colour) / 1 (opacity).
  void isocontour_rgb(const isocontour &iso, float rgb[3]) const;
  float isocontour_opacity(const isocontour &iso) const;

  // A baked table of `size` RGBA entries uniform over t in [0,1] (entry i <-> t = i/(size-1)).
  // size < 2 yields an empty table.
  std::vector<rgba> bake(int size) const;
  // The same, quantized to 8-bit RGBA (rounded), `size`*4 bytes.
  std::vector<std::uint8_t> bake_rgba8(int size) const;

  // A fresh isocontour id not used by any isocontour ("iso1", "iso2", ...).
  std::string next_isocontour_id() const;

  // Index of the isocontour with `id`, or -1.
  int find_isocontour(const std::string &id) const;

  // Exact comparison of what the model means: an isocontour's r/g/b (opacity) only count when
  // has_rgb (has_opacity) is set, and an automatic domain's min/max not at all -- the fields
  // to_string leaves out.
  bool operator==(const model &o) const;
  bool operator!=(const model &o) const { return !(*this == o); }
};

// The effective [lo, hi] a model spans: its explicit domain, else `range_lo/range_hi` when
// has_range (the consumer's data range), else [0, 1]. A degenerate range (hi <= lo) widens to
// lo + 1 so the mapping stays invertible.
void effective_domain(const model &m, bool has_range, double range_lo, double range_hi, double &lo,
                      double &hi);

// Normalized <-> data units over [lo, hi].
inline double to_data(double t, double lo, double hi) { return lo + (hi - lo) * t; }
inline double to_normalized(double v, double lo, double hi) {
  return hi > lo ? (v - lo) / (hi - lo) : 0.0;
}

// --- the state string -----------------------------------------------------------------------
//
// {"color":[[at,r,g,b],...],"alpha":[[at,a],...],
//  "isocontours":[{"id":"bone","at":0.6,"rgb":[1,1,1],"opacity":1},...],
//  "domain":"auto" | [min,max], "opacity_cubed":false, "resolution":256}
//
// to_string writes that compact form (rgb/opacity only when overridden). from_string also accepts
// the roadmap §10.1 map forms ({"at":t,"rgb":[r,g,b]}, {"at":t,"a":a}), "dims":1, and any missing
// key (left at its default); it normalizes the result. An empty string parses as default_model().
// Numbers: positions (colour, alpha and isocontour `at`) and an explicit domain exactly (the
// shortest of %.15g..%.17g that reads back bit-identically, so an isovalue or a node placed in
// data units survives the string even on a billions-wide range), colours / opacities %.6g; '.'
// is the decimal point whatever the global C / C++ locale, both ways. The reader also takes a
// domain as {"min":lo,"max":hi}, an isocontour as a bare position, and the §10.1 raw-table form
// {"table":{"size":N,"rgba":[r,g,b,a,...]}} (one node per entry at i/(N-1); ignored when
// "color" or "alpha" is given). A missing key keeps default_model()'s value (as the scene
// loader's transfer_function: does), so "{}" == "" == default_model(); a present key replaces it
// ("color":[] is an empty list, which normalize() turns into black->white). "dims" other than 1,
// a wrong type or a malformed node fails: false, a reason in *error ("tf model: ..."), and `out`
// left untouched.
std::string to_string(const model &m);
bool from_string(const std::string &s, model &out, std::string *error = nullptr);

// --- lowering for renderers that take raw data units -----------------------------------------
//
// Flat tables in the cvc::volren / volslice / cvc::gl::VolumeNode encodings over [lo, hi]:
//   color_table   -> value,r,g,b, value,r,g,b, ...   (one entry per colour node)
//   opacity_table -> value,a, value,a, ...            (one entry per alpha node; when opacity_cubed
//                    the cubed curve is resampled, since a cubed ramp is not piecewise-linear in
//                    the node positions: at `cubed_samples` evenly spaced points (<= 0, the
//                    default: the model's `resolution`; else at least 2) MERGED with every alpha
//                    node position in [0,1], so no node-level feature -- a one-node spike, a
//                    step -- falls between the samples; duplicate positions are written once)
// isovalues      -> the data value of each isocontour, in model order.
// Positions 0 and 1 map to exactly lo and hi (lo + (hi - lo) * 1 can miss hi by an ulp), so the
// endpoints survive from_tables bit-exactly.
// A fully transparent model (every alpha node at 0) lowers to EMPTY colour and opacity tables, so a
// renderer keeps its "no transfer function" path (volren then draws only its isosurfaces, exactly
// as a document with no transfer_function: did before the model existed).
bool transparent(const model &m);
std::vector<double> color_table(const model &m, double lo, double hi);
std::vector<double> opacity_table(const model &m, double lo, double hi, int cubed_samples = 0);
std::vector<double> isovalues(const model &m, double lo, double hi);

// The reverse: build a model from raw-unit tables over [lo, hi] (positions normalized, then
// normalize()d). `coupled` is a list of value,r,g,b,a points (the legacy scene `points:` form,
// colour and opacity at the same scalar); either table may be empty. An empty table reads the way
// the renderers sample one -- transparent black (constant black colour / constant 0 opacity) --
// so a transparent model survives a lower-and-raise round trip. A degenerate range widens like
// effective_domain's; a trailing partial entry is ignored. A hard step -- two points at the same
// value, which the renderers sample as a step -- is kept: the later point moves up by 1e-9
// (normalized; never past the next point, and for a step at or past the top end the earlier one
// moves down instead), where normalize() alone would keep only the first. Lowering gives it back
// as the pair span * 1e-9 apart, the gap tf_binding opens for any step it hands a renderer.
model from_tables(const std::vector<double> &color_table, const std::vector<double> &opacity_table,
                  double lo, double hi);
model from_coupled_points(const std::vector<double> &value_rgba, double lo, double hi);

// A dense colour ramp (sorted by `at`, e.g. a colormap sampled finely) thinned to few nodes: the
// two ends, then repeatedly the input node the thinned ramp misses by most, until it follows every
// input node within `tolerance` (per channel) or holds `max_nodes` (at least 2). An editor then
// shows a handful of colour bars to grab rather than one per sample.
std::vector<color_node> simplify_ramp(const std::vector<color_node> &dense, float tolerance,
                                      int max_nodes);

// "a,b,c" <-> doubles, the flat-CSV encoding the renderer keys use. parse_csv returns false (and
// leaves `out` cleared) on any malformed token.
// format_csv writes %.17g in the classic locale (a non-finite value as 0); parse_csv takes ""
// as an empty list and otherwise needs every comma-separated token to be one number, blanks
// around it allowed -- no empty tokens (so no trailing comma), no nan/inf, no hex.
std::string format_csv(const std::vector<double> &v);
bool parse_csv(const std::string &s, std::vector<double> &out);

} // namespace tf
} // namespace cvc

#endif // CVC_TF_MODEL_H
