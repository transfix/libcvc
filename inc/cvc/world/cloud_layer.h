/*
  Copyright 2007-2011 The University of Texas at Austin

        Authors: Joe Rivera <transfix@ices.utexas.edu>
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

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

// cloud_layer.h — a world-scale cloud deck from a few L-system cumulus maps.
//
// One cvc::lsys::cloud_field is ONE cumulus complex, windowed to zero at its grid's edges, so
// tiling it would repeat the same cloud on a lattice. The deck instead scatters clouds over a grid
// of cells: each cell holds a cloud or not (`coverage`), offset in its cell (`jitter`), and morphs
// through a cell-specific sequence of the variant maps (a linear crossfade every `morph_s`
// seconds). The cells repeat with a period of at least the world's width, so every product here
// is PERIODIC and the renderer samples it with wrap-around: the wind is a translation of the whole
// pattern (the caller's drift offset), never a re-bake.
//
// Coordinates: the PATTERN frame q = world xy - drift. Cell (i, j) spans [i, i+1) x [j, j+1)
// spacings, i, j in [0, period_cells); the pattern repeats every period_m() in x and y.
//
// What it makes, all deterministic in the params:
//   - cell_field: a cell's density at a morph, the map a near VolumeNode draws;
//   - cell_sun_tau: the same cell's optical depth toward the sun from each voxel, its self-shade;
//   - shadow_mask: the periodic ground shade, the optical depth along the sun from each point of
//     the base plane through the deck. A receiver at (x, y, z) reads it at its base-plane point,
//     base_point(); the projection is exact for any ground height below the deck;
//   - far_density: the periodic coarse opacity in `far_layers` horizontal slices, each cell
//     weighted by the caller (0 where a near volume draws it), for depth-tested far shells.
// The shadow and the far slices integrate the renderer's opacity (cloud_layer_params::opacity),
// not the raw density.
// Morphs are linear crossfades of maps that were lumped and normalized once, so the optical depth
// of a crossfade is the crossfade of the optical depths: the shadow follows the morph exactly.
//
// Which cells hold a cloud evolves (evolve()): a Dobashi-style cellular automaton [Dobashi et al.
// 2000] runs over the cell grid, one bit-packed column of `layers` bits per cell with Dobashi's
// three states (humidity, activation, cloud). Activation spreads through humid bits, up and down a
// whole column at once and, by chance, sideways to the four neighbours; cloud bits go extinct at
// random and humidity comes back, so clouds grow, last and dissipate in patches. The cells with the
// most cloud bits hold the deck's clouds, exactly round(coverage * cells) of them, so `coverage`
// still means what it says. A cell's cloud appears and fades over `fade_s` (its weight(), a
// smoothstep of its presence), and the shadow and far slices take the same weight the renderer
// gives the cell's volume.
// Orographic coupling (set_ground): air pushed up a windward flank condenses, so a cell over one
// gets more humidity and activation, and one over a lee slope, where the air sinks, loses its
// cloud sooner. The pattern drifts over the ground (set_drift), so a cloud forms as it reaches a
// mountain's windward side and dies past the crest; the deck gathers on the windward flanks while
// a high peak stands through it.
#ifndef CVC_WORLD_CLOUD_LAYER_H
#define CVC_WORLD_CLOUD_LAYER_H

#include <cstdint>
#include <vector>

namespace cvc {
namespace world {

struct cloud_layer_params {
  std::uint64_t seed = 20;
  int variants = 4; // distinct cumulus maps (cvc::lsys::cloud_field variants) cells morph through
  int n = 60, nz = 28;   // each map's grid
  double extent = 150.0; // a cloud map's half-width, metres
  double base = 74.0;    // the deck's base height, metres
  double top = 122.0;    // its top
  double spacing = 0.0;  // cell size, metres; 0 = 2.5 * extent
  double coverage = 0.6; // the fraction of cells holding a cloud
  double jitter = 0.35;  // a cloud's largest offset from its cell's centre, in spacings
  double area = 150.0;   // the half-width of the world to cover: the period is at least 2 * area
  double sun_az = -52.0; // the sun the maps are lit from and the shadows are cast along (degrees)
  double sun_el = 34.0;
  int shadow_texels = 64; // shadow-mask texels per cell (fewer when the mask would pass 512)
  int far_texels = 16;    // far-density texels per cell (fewer when a slice would pass 256)
  int far_layers = 10;    // far-density slices between base and top
  int depth = 6;          // the cloud grammar's recursion cap (cvc::lsys::cloud_params::depth)
  // The evolving coverage: seconds per automaton step at rate 1 (evolve's `rate` scales it), a
  // cloud's fade in or out (seconds), the heading the wind blows toward (degrees, as sun_az: 90 is
  // +x), and the strength of the orographic coupling (0: none; 1: a steep windward flank nearly
  // always holds a cloud).
  double evolve_s = 5.0;
  double fade_s = 20.0;
  double wind_az = 90.0;
  double orographic = 1.0;
  // The renderer's opacity per unit density, as (density, opacity) pairs in rising density,
  // linear between them: what the shadow and the far slices integrate, so they show the cloud
  // the volume shows and not the faint haze its transfer function hides. Empty: the density.
  std::vector<double> opacity;
};

struct cloud_cell {
  int i = 0, j = 0;
  bool occupied = false;     // the automaton gives this cell a cloud now
  double cx = 0.0, cy = 0.0; // the cloud's centre in the pattern frame, metres
  double phase = 0.0;        // its morph offset, in variant steps
  int jx = 0, jy = 0;        // its offset from the cell centre, in 1/far_texels of a cell
  double presence = 0.0;     // its crossfade, 0..1: moves toward `occupied` over fade_s
  double cloud = 0.0;        // the share of its column's bits that are cloud
  double forcing = 0.0;      // the orographic forcing under it at the last step, -1..1
  // The weight its cloud is drawn and shades with: a smoothstep of the presence.
  double weight() const { return presence * presence * (3.0 - 2.0 * presence); }
};

class cloud_layer {
public:
  explicit cloud_layer(const cloud_layer_params &p);

  const cloud_layer_params &params() const { return m_p; }
  int period_cells() const { return m_m; }
  double spacing() const { return m_c; }
  double period_m() const { return m_c * m_m; }
  const std::vector<cloud_cell> &cells() const { return m_cells; }
  int occupied() const;
  // Re-lay the cells for a world of half-width `area` (the period follows it). A cell's offset and
  // morph depend on its (i, j) alone. The automaton starts over on the new grid and is run until it
  // has settled (warm()), with the cells' presence snapped to it. The maps are kept, and the shadow
  // patches are re-made only when the mask's texel changes.
  void set_area(double area);

  // ── The evolving coverage
  // Run the automaton for dt seconds at `rate` (dt * rate / evolve_s steps: whole steps run, the
  // remainder carries to the next call) and move every cell's presence toward its target by
  // dt / fade_s, split at the steps so that one call of 10 s is many of 0.1 s. rate 0 freezes the
  // automaton; fades still finish.
  void evolve(double dt, double rate = 1.0);
  // One automaton step, then the ranking (no fade).
  void step();
  // Run `steps` steps and snap every presence to its cell's target (a deck that has been there).
  void warm(int steps);
  // Every presence to its target at once.
  void snap();
  long generation() const { return m_gen; }
  int layers() const { return CA_LAYERS; }
  // The pattern's offset in the world (world xy = pattern xy + drift): where the cells are over
  // the ground, for the orographic forcing.
  void set_drift(double dx, double dy) {
    m_dx = dx;
    m_dy = dy;
  }
  void set_orographic(double s) { m_p.orographic = s < 0.0 ? 0.0 : s; }
  void set_wind(double az_deg) { m_p.wind_az = az_deg; }
  // The ground under the deck, for orographic coupling: rows x cols heights, row-major, sample
  // (r, c) at (x0 + c dx, y0 + r dy) (cvc::height_field's layout). It is kept box-averaged to
  // about an eighth of a cloud. Empty: no ground, no coupling.
  void set_ground(const std::vector<float> &heights, int rows, int cols, double x0, double y0,
                  double dx, double dy);
  bool has_ground() const { return !m_gh.empty(); }
  // The orographic forcing at world (x, y): the ground's slope along the wind, averaged over a
  // cloud's footprint, over SLOPE_REF and clamped to [-1, 1]; positive on a windward flank, 0 off
  // the ground or without one. Its strength is applied in the automaton, not here.
  double forcing(double x, double y) const;
  // A cell's cloud centre in the world: its pattern centre moved by the drift, the copy nearest
  // the ground's centre (the pattern's origin without a ground).
  void cell_world(const cloud_cell &c, double &x, double &y) const;

  // The unit vector toward the sun.
  void sun_dir(double &x, double &y, double &z) const;
  // Where the ray from (x, y, z) toward the sun crosses the deck's base plane (z below the base).
  void base_point(double x, double y, double z, double &bx, double &by) const;

  // A cell's morph at the global morph clock m (variant steps): variants a -> b, weight u of b.
  struct morph_state {
    int a = 0, b = 0;
    double u = 0.0;
  };
  morph_state cell_morph(const cloud_cell &c, double m) const;
  // The variant a cell shows at morph step k (its own hashed sequence; never the same twice in a
  // row when there are two variants or more).
  int cell_variant(const cloud_cell &c, long k) const;

  // A cell's density at morph clock m: n*n*nz floats in [0, 1], cvc::lsys::cloud_index layout,
  // spanning [cx - extent, cx + extent]^2 x [base, top]. Its full cloud: the renderer applies the
  // cell's weight() (the shadow and far slices below already do).
  void cell_field(const cloud_cell &c, double m, std::vector<float> &out) const;
  // One variant's lumped, normalized map (the u = 0 field of a cell showing it).
  const std::vector<float> &variant_map(int v) const { return m_maps[v]; }
  // The cloud's own shade inside it: at each of a cell's voxels (cell_field's layout), the optical
  // depth from there toward the sun to where the ray leaves the map, the renderer's opacity
  // integrated in metres (as the shadow's). 0 on the top layer and in the open, growing down into
  // the cloud and away from the sun. A crossfade of two variants' depths, as cell_field's density.
  void cell_sun_tau(const cloud_cell &c, double m, std::vector<float> &out) const;
  const std::vector<float> &variant_sun_tau(int v) const { return m_sun[v]; }

  // The periodic shadow mask at morph clock m: shadow_size()^2 bytes, row-major (x fastest), texel
  // (tx, ty) centred on the pattern point ((tx + 0.5), (ty + 0.5)) * period_m() / shadow_size().
  // 255 is full sun; a texel's shade is floor + (1 - floor) * exp(-k * tau), tau the density
  // integrated in metres along the sun from that base-plane point, each cell's weighted by its
  // weight().
  int shadow_size() const { return m_m * m_s; }
  void shadow_mask(double m, double k, double floor, std::vector<unsigned char> &out) const;
  // The same optical depth, unshaded (for tests and tuning): shadow_size()^2 floats.
  void shadow_tau(double m, std::vector<float> &out) const;
  // shadow_mask a share at a time, for a renderer that spreads it over frames: add_shadow adds
  // cells [first, first + count) (cells() order) into tau (sized and zeroed on first use; clear
  // it to start over); shade_rows turns rows [row0, row0 + rows) of tau into bytes.
  void add_shadow(double m, std::size_t first, std::size_t count, std::vector<float> &tau) const;
  void shade_rows(const std::vector<float> &tau, double k, double floor, int row0, int rows,
                  std::vector<unsigned char> &out) const;

  // The periodic far opacity at morph clock m: far_size()^2 * far_layers bytes, x fastest, then y,
  // then the slice (bottom first), 255 the table's largest opacity. weight[k] scales cell k's
  // cloud (cells() order; missing = 1), on top of its weight().
  int far_size() const { return m_m * m_f; }
  // The opacity a far byte of 255 stands for: the opacity table's largest value (1 without one).
  double opacity_max() const { return m_amax; }
  void far_density(double m, const std::vector<float> &weight,
                   std::vector<unsigned char> &out) const;
  // far_density a share at a time (as add_shadow), then the bytes of slices
  // [slice0, slice0 + slices).
  void add_far(double m, std::size_t first, std::size_t count, const std::vector<float> &weight,
               std::vector<float> &acc) const;
  void far_bytes(const std::vector<float> &acc, int slice0, int slices,
                 std::vector<unsigned char> &out) const;

  // The automaton's layers per cell (bits in a column word).
  static constexpr int CA_LAYERS = 32;
  // The steps warm() runs after a new layout.
  static constexpr int WARM_STEPS = 200;

private:
  cloud_layer_params m_p;
  // The automaton: per cell, its column's humidity, activation and cloud bits.
  std::vector<std::uint32_t> m_hum, m_act, m_cld, m_tmp;
  long m_gen = 0;
  double m_carry = 0.0; // the step fraction evolve() has run toward the next step
  double m_dx = 0.0, m_dy = 0.0;
  // The ground, box-averaged: m_gw x m_grows samples from (m_gx0, m_gy0), (m_gsx, m_gsy) apart.
  std::vector<float> m_gh;
  int m_gw = 0, m_grows = 0;
  double m_gx0 = 0.0, m_gy0 = 0.0, m_gsx = 1.0, m_gsy = 1.0;
  void seed_automaton();
  void rank_cells();
  void fade(double share);
  double ground_at(double x, double y) const;
  int m_m = 2;         // period, in cells
  double m_c = 0;      // cell size, metres
  double m_amax = 1.0; // the opacity table's largest value
  int m_s = 64;        // shadow texels per cell
  int m_f = 16;        // far texels per cell
  std::vector<cloud_cell> m_cells;
  std::vector<std::vector<float>> m_maps; // per variant, lumped and normalized
  std::vector<std::vector<float>> m_sun;  // per variant, the optical depth toward the sun
  void build_sun();
  // Per variant: the optical depth over the base plane around the cloud (m_tau_n^2, even, centred
  // on the cloud), and its coarse slices (m_far_n^2 * far_layers, centred).
  int m_tau_n = 0, m_far_n = 0;
  std::vector<std::vector<float>> m_tau, m_far;
  double opacity_of(double v) const;
  void build_tau();
  void build_far();
};

} // namespace world
} // namespace cvc

#endif // CVC_WORLD_CLOUD_LAYER_H
