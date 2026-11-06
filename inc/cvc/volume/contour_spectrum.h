/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::compute_contour_spectrum — the contour spectrum of a scalar volume (Bajaj, Pascucci &
// Schikore, "The Contour Spectrum", IEEE Visualization 1997): signature functions of the isovalue
// w that VolumeRover2's ColorTable2 drew under its transfer function to help place isocontours.
//
//   surface_area(w)  area of the isosurface {f = w}
//   min_volume(w)    volume of the sublevel set {f < w}
//   max_volume(w)    volume of the superlevel set {f >= w}  (= total - min_volume)
//   gradient(w)      integral of |grad f|^2 over the isosurface (index-unit grad, as the original)
//
// This is a self-contained, thread-safe port of the regular-grid signature functions of libContour
// (Datareg3::compLength / compArea / compMaxArea / compGradient and the tetSurfIntegral /
// tetVolIntegral kernels; libcvc vendors that library under src/cvc/geometry/cvc-mesher/contour):
// the same five-tetrahedra cell split and the same closed-form per-tetrahedron integrals, but
// double accumulation, any sample count, any domain, any voxel type, no globals, and it builds
// everywhere (libContour needs CVC_ENABLE_MESHER, which no wasm build has). Area and volume are in
// world units (the volume's bounding box) unless options.world_units is false.
//
// Values are integrated with the domain mapped affinely onto [0, 255]: libContour separates equal
// vertex values by an absolute 1e-5, which keeps its uchar meaning on that scale whatever the data
// range. Area and volume do not change under the map; the gradient is scaled back to data units
// (by (range/255)^2). Cells with a NaN or infinite corner are skipped (and left out of
// total_volume, whatever the domain). Unlike libContour, tetrahedra below the domain count
// towards min_volume, max_volume includes a plateau at the domain maximum, and the surface area
// (so the gradient) is exact (see contour_spectrum_options::libcontour_area).

#ifndef CVC_VOLUME_CONTOUR_SPECTRUM_H
#define CVC_VOLUME_CONTOUR_SPECTRUM_H

#include <cstddef>
#include <cvc/core/exception.h>
#include <cvc/core/types.h>
#include <functional>
#include <memory>
#include <vector>

namespace cvc {
class volume;
struct contour_mesh; // cvc/volume/contour_mesh.h

CVC_DEF_EXCEPTION(contour_spectrum_error);

struct contour_spectrum {
  double domain_min = 0.0, domain_max = 0.0;
  double total_volume = 0.0;
  // All of length n; isovalue[b] = domain_min + b/(n-1) * (domain_max - domain_min).
  std::vector<double> isovalue, surface_area, min_volume, max_volume, gradient;

  std::size_t size() const { return isovalue.size(); }
  bool empty() const { return isovalue.empty(); }
  // Linear interpolation of one function (a member above) at an isovalue, clamped to the domain.
  double at(const std::vector<double> contour_spectrum::*fn, double isovalue) const;
};

struct contour_spectrum_options {
  std::size_t samples = 256; // at least 2
  // The isovalue domain; default (has_domain false) is [vol.min(), vol.max()] (the set values, or
  // the finite data extrema when unset, without writing vol's lazy min/max cache). A reversed
  // domain is swapped.
  bool has_domain = false;
  double domain_min = 0.0, domain_max = 1.0;
  // Integrate in the volume's world coordinates (bounding box) vs voxel-index units (of the grid
  // actually integrated, i.e. after max_dim resampling).
  bool world_units = true;
  // Resample (trilinear) so no axis exceeds this many voxels first (0 = full resolution; values
  // below 2 act as 2). The samples sit exactly where voxels::resize(dimension) puts them and are
  // converted to the voxel type the same way, but into the computation's own buffer: the input
  // volume is never modified. The cost is ~ 5 * cells * (1 + the samples each tetrahedron's value
  // range covers).
  uint64 max_dim = 0;
  // Called once per contour_spectrum_builder::step, after its work, with the builder's progress()
  // (so the last call of a run is 1); return false to cancel: that step throws
  // contour_spectrum_error. compute_contour_spectrum steps in 2^22-unit budgets when this is set
  // (see the builder), so it is called about every 20 ms of work.
  std::function<bool(double)> progress;
  // libContour's tetSurfIntegral weights the quadratic middle term of a tetrahedron's
  // cross-section area by s(1-s) where the exact area needs 2s(1-s) (its own tetVolIntegral uses
  // 2s(1-s)), so its surface area and gradient run low, unevenly: down to 75% of the true area of
  // a plane, depending on where the isovalue falls between voxel values, ~86% on a sphere. By
  // default the exact area is integrated; true reproduces libContour's (VolumeRover2's) curves.
  bool libcontour_area = false;
};

// The contour spectrum of `vol`. Throws contour_spectrum_error when an axis has fewer than 2
// voxels, for an Undefined voxel type, fewer than 2 samples, a non-finite domain or one too wide
// to scale (wider than 255 * sqrt(DBL_MAX), ~3.4e156, where the gradient's scale back to data
// units, (width/255)^2, overflows: [-DBL_MAX, DBL_MAX], say, or a -DBL_MAX nodata value under the
// default domain), a function value past the double range (a gradient too steep, or an area or
// volume of a bounding box too large, to represent: a returned spectrum is always finite), or on
// cancel. A degenerate domain (a constant volume under the default domain, domain_min ==
// domain_max, or one too narrow to scale) yields all-zero functions with the isovalues filled,
// and the same total_volume as any other domain (without the cells with a non-finite corner).
// Safe to call concurrently, also on one volume (it only reads it). It runs a
// contour_spectrum_builder to completion (in one step, or in
// 2^22-unit steps when opt.progress is set), so it and any stepping schedule give bit-identical
// results.
contour_spectrum compute_contour_spectrum(const volume &vol,
                                          const contour_spectrum_options &opt = {});

// The vendored libContour computation (Datareg3, 256 samples over the data's own extrema), for
// parity checks. Available only in a CVC_ENABLE_MESHER build; otherwise it throws
// contour_spectrum_error. Safe to call concurrently (libContour keeps no process-wide state). The
// arrays are libContour's own (float) results: isovalue is its sample positions and max_volume its
// "Max Volume", min_volume.back() - min_volume (not total - min_volume); total_volume is the
// grid's.
// Types other than uchar/ushort/float are converted to float first; a constant volume returns
// zeros (libContour would divide by zero). Meant for finite data.
bool contour_spectrum_libcontour_available();
contour_spectrum compute_contour_spectrum_libcontour(const volume &vol, bool world_units = true);

// compute_contour_spectrum, a slice at a time, for a caller that must not block: an Ariadne overlay
// task (cvc/tf/overlay_tasks.h) steps it a few milliseconds per frame on the app's state_exec
// scheduler, which is how single-threaded wasm computes a spectrum with no thread at all. It uses
// no threads, no OpenMP and no globals; independent builders may run on different threads.
//
// step(budget) works until it has done at least `budget` units, or the spectrum is ready, and
// returns true once it is; budget 0 runs to the end. Units are weighted so that one is about the
// time to scan one voxel (~5 ns on a desktop core) whatever the phase:
//   "domain"       1 per voxel scanned for the default domain's finite extrema (only when the
//                  domain is not given and vol's min/max cache is not fully set);
//   "resampling"   8 per voxel of the max_dim grid (eight reads, seven lerps; only when max_dim
//                  caps an axis);
//   "integrating"  1 per voxel mapped onto the integration scale; per cell 200 plus 2 per
//                  (tetrahedron, isovalue sample) pair it updates (0 to 5 * samples of them), or 2
//                  for a cell with a non-finite corner; once, `samples` for the accumulators; and 1
//                  per sample of the final pass. Under a degenerate domain a Float or Double
//                  volume's cells are only checked for a non-finite corner: 1 per voxel mapped and
//                  2 per cell (after the resample when max_dim caps an axis), no accumulators; an
//                  integer one goes straight to the final pass.
// Every loop that grows with the volume (the scan, the resample, each plane's conversion, and the
// cell loop within a z-slab and within a row) stops at the budget and resumes on the next step.
// The largest indivisible piece is one cell, so a step that does not finish does between `budget`
// and budget + max_overrun(samples) units, max_overrun = 10 * samples + 199 (2759 at the default
// 256 samples) whatever the volume's size. A budget of 4096 is then ~20 us of work, and stepping
// in such budgets costs about the same wall time as one step to the end.
//
// compute_contour_spectrum runs one to completion, so both give bit-identical results whatever the
// schedule. The builder keeps a shallow copy of the volume (the voxels are shared copy-on-write: do
// not write them through a raw pointer while it runs) and its min/max cache values, and never
// writes the caller's volume; it releases the copy once the copy is no longer read. Destroying it
// mid-way cancels it cleanly. Not thread-safe, but it may move between threads between steps.
class contour_spectrum_builder {
public:
  explicit contour_spectrum_builder(const volume &vol, const contour_spectrum_options &opt = {});
  ~contour_spectrum_builder();
  // A moved-from builder is empty: done() is true, phase() "done", progress() 1, work_done() 0,
  // and step() and take() throw contour_spectrum_error. Assigning a builder to it revives it.
  contour_spectrum_builder(contour_spectrum_builder &&) noexcept;
  contour_spectrum_builder &operator=(contour_spectrum_builder &&) noexcept;

  // Throws contour_spectrum_error where compute_contour_spectrum would (the input checks on the
  // first step, the domain's once it is known: on the first step when it is given or cached, after
  // the scan otherwise; an overflow in the final pass); after a throw the builder is done and every
  // later step rethrows the same error.
  // opt.progress is called once per step that does work, after it, with progress(), and may
  // cancel (that step then throws "cancelled"). A step on a finished builder returns true and
  // calls nothing.
  bool step(std::size_t budget);
  bool done() const; // finished, failed or moved-from
  // In [0,1], monotone: 0 before the first step, exactly 1 once the spectrum is ready, frozen on
  // failure. The phases are weighted by their units (the integration's without its sample
  // updates, which are not known in advance), so it advances roughly evenly in time.
  double progress() const;
  // The phase the next unit of work belongs to: "domain" (also before the first step),
  // "resampling", "integrating" or "done" (also after a failure).
  const char *phase() const;
  // Units of work done so far (the budget's units), e.g. to calibrate a budget against a clock.
  uint64 work_done() const;
  // The most a step can do beyond its budget (one cell's worth): 10 * samples + 199.
  static uint64 max_overrun(std::size_t samples);
  // The spectrum, once step() has returned true (moved out: call once; a second call, or a call
  // before it is ready, throws contour_spectrum_error, and after a failure it rethrows that).
  contour_spectrum take();

private:
  struct impl;
  std::unique_ptr<impl> _impl;
};

// --- on a simplicial mesh (cvc/volume/contour_mesh.h) -----------------------------------------
//
// The same four functions of a piecewise-linear field on a mesh, in the mesh's own units and data
// units (no [0, 255] map is needed: each element's integral is exact or the grid's own):
//   on tetrahedra (a solid)   the isosurface area, the volume below and above the isovalue, and the
//                             integral of |grad f|^2 over the isosurface, with each tetrahedron's
//                             exact gradient (data units per mesh unit; the grid version takes a
//                             cell-average difference per voxel). The area and volume are the grid
//                             version's per-tetrahedron integrals (libcontour_area applies);
//   on triangles (a surface)  the isocontour's length (in surface_area), the area below and above
//                             it (in min_volume / max_volume) and the integral of |grad f|^2 along
//                             it, all exact.
// total_volume is the solid's volume or the surface's area. An element with a non-finite value is
// skipped and left out of it. The domain defaults to the finite values' extrema; samples, the
// domain, progress and libcontour_area apply as on a grid; world_units and max_dim do not.
//
// Throws contour_spectrum_error for a mesh with neither tetrahedra nor triangles, a value count
// other than the point count, an element naming a point that does not exist, fewer than 2
// samples, a non-finite domain or no finite value under the default one, a function past the
// double range, or on cancel. A degenerate domain gives all-zero functions with total_volume set.
contour_spectrum compute_contour_spectrum(const contour_mesh &mesh,
                                          const contour_spectrum_options &opt = {});

// compute_contour_spectrum on a mesh, a slice at a time, like contour_spectrum_builder (an overlay
// task steps it on the state_exec scheduler). Phases: "domain" (1 unit per value scanned, only
// when the domain is not given), "integrating" (40 units per tetrahedron or 20 per triangle, plus
// 2 per isovalue sample it updates; then 1 per sample of the final pass), then "done". A step does
// at most one element past its budget. The builder holds the shared mesh until it is done.
// progress(), phase(), work_done(), take() and the failure rules are contour_spectrum_builder's.
class mesh_contour_spectrum_builder {
public:
  explicit mesh_contour_spectrum_builder(std::shared_ptr<const contour_mesh> mesh,
                                         const contour_spectrum_options &opt = {});
  ~mesh_contour_spectrum_builder();
  mesh_contour_spectrum_builder(mesh_contour_spectrum_builder &&) noexcept;
  mesh_contour_spectrum_builder &operator=(mesh_contour_spectrum_builder &&) noexcept;

  bool step(std::size_t budget);
  bool done() const;
  double progress() const;
  const char *phase() const;
  uint64 work_done() const;
  contour_spectrum take();

private:
  struct impl;
  std::unique_ptr<impl> _impl;
};

} // namespace cvc

#endif // CVC_VOLUME_CONTOUR_SPECTRUM_H
