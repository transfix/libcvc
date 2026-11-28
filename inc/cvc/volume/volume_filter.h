/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// Filters on a volume the app holds (docs/roadmap/DATA_FROM_STATE_TREE.md, decision 2): a stepper
// that makes a new, filtered volume from a shared, read-only one a few rows at a time, so the DSL
// (volume.filter: cvc/volume/volume_filter_intrinsics.h), a node or pycvc runs it as a sliced job
// beside the frame (docs/roadmap/VOLROVER-LIBRARY-PORTS-ROADMAP.md §1.4, §1.9). The input is never
// written: a program stores the result where the views follow it (a document's data entry, a
// state key), and every node viewing that key redraws on it.
//
//   cvc::volume_filter_params p;               // bilateral, VolumeRover's defaults
//   auto job = cvc::volume_filter_job(vol, p);  // vol: std::shared_ptr<const cvc::volume>
//   while (!job->step(65536)) {}                // or schedule_sliced_job
//   std::shared_ptr<const cvc::volume> out = job->take_result();
//
// The filters (VolumeRover2's Tools menu; `filter` picks one, and each reads only its own fields):
//   bilateral   VolumeRover's (VolMagick's voxels::bilateralFilter): each voxel the mean of its
//               neighbours within `radius`, weighted by a Gaussian of their distance
//               (spatial_sigma, voxels) and of their value's difference (radiometric_sigma, data
//               units, the difference taken in 256 steps of the volume's range). The same weights
//               as voxels::bilateralFilter, voxel for voxel. Each value is written in the volume's
//               own voxel type, where voxels::bilateralFilter writes every type other than float
//               and double as a byte. A constant volume comes back as it is.
//   contrast    Zeyun Yu's contrast enhancement (voxels::contrastEnhancement): local minima,
//               maxima and means propagated through the volume along each axis both ways, with
//               `contrast.resistor` in [0, 1] (how much of a neighbour's value carries over), then
//               each voxel stretched between its local minimum and maximum, and the volume's range
//               put back. Voxel for voxel voxels::contrastEnhancement's, in its voxel type.
//   anisotropic Zeyun Yu's anisotropic diffusion (voxels::anisotropicDiffusion): Perona-Malik
//               diffusion over the six face neighbours, `anisotropic.iterations` times, smoothing
//               within regions and keeping edges. Voxel for voxel voxels::anisotropicDiffusion's
//               CPU path, in its voxel type.
//   gdtv        Guoliang Xu and Qin Zhang's generalized discrete total variation flow
//               (voxels::gdtvFilter): `gdtv.iterations` Jacobi passes that pull each voxel toward
//               its neighbours, weighted through the gradient magnitude, and toward the input by
//               `gdtv.lambda`. `gdtv.exponent` p in [0, 2] is VolumeRover2's: the flow of
//               |grad u|^p, so 1 is total variation and 2 linear diffusion (voxels::gdtvFilter's q
//               is 2 - p). `gdtv.neighbours` is six (faces) or twenty_six (faces, edges and
//               corners). Voxel for voxel voxels::gdtvFilter's CPU path, in its voxel type.
// Every default is VolumeRover2's dialog's: bilateral 200, 1.5, 2; contrast 0.95; anisotropic 20
// iterations; GDTV exponent 1, lambda 1e-4, one iteration, six neighbours.
//
// No statics, no globals (§1.3): a stepper keeps its tables in itself.

#ifndef CVC_VOLUME_VOLUME_FILTER_H
#define CVC_VOLUME_VOLUME_FILTER_H

#include <boost/describe/enum.hpp>
#include <cstddef>
#include <cstdint>
#include <cvc/core/param_table.h>
#include <cvc/core/result_job.h>
#include <limits>
#include <memory>
#include <string>
#include <tuple>
#include <vector>

namespace cvc {
class volume;

enum class volume_filter_kind { bilateral, contrast, anisotropic, gdtv };
BOOST_DESCRIBE_ENUM(volume_filter_kind, bilateral, contrast, anisotropic, gdtv)

// The GDTV filter's neighbourhood: the six face neighbours (VolumeRover2's neighbourhood size 1),
// or all twenty-six (size 2).
enum class volume_gdtv_neighbours { six, twenty_six };
BOOST_DESCRIBE_ENUM(volume_gdtv_neighbours, six, twenty_six)

struct volume_filter_params {
  volume_filter_kind filter = volume_filter_kind::bilateral;
  double radiometric_sigma = 200.0;         // bilateral: data units
  double spatial_sigma = 1.5;               // bilateral: voxels
  std::int64_t radius = 2;                  // bilateral: the neighbourhood's half-width, voxels
  double contrast_resistor = 0.95;          // contrast: "contrast.resistor"
  std::int64_t anisotropic_iterations = 20; // anisotropic: "anisotropic.iterations"
  double gdtv_exponent = 1.0;               // gdtv: "gdtv.exponent"
  double gdtv_lambda = 1e-4;                // gdtv: "gdtv.lambda"
  std::int64_t gdtv_iterations = 1;         // gdtv: "gdtv.iterations"
  volume_gdtv_neighbours gdtv_neighbours = volume_gdtv_neighbours::six; // gdtv: "gdtv.neighbours"
};

template <> struct param_table<volume_filter_params> : param_fields<volume_filter_params> {
  using P = volume_filter_params;
  static constexpr double none = std::numeric_limits<double>::quiet_NaN();
  static constexpr auto fields = std::tuple{
      field("filter", &P::filter, doc("the filter: bilateral, contrast, anisotropic or gdtv")),
      field("radiometric_sigma", &P::radiometric_sigma, unit("data"), range(1e-9, none),
            doc("bilateral: how far apart two values may be and still blend (small keeps edges)")),
      field("spatial_sigma", &P::spatial_sigma, unit("voxel"), range(1e-9, none),
            doc("bilateral: how fast a neighbour's weight falls with its distance")),
      field("radius", &P::radius, unit("voxel"), range(0.0, 16.0),
            doc("bilateral: the neighbourhood's half-width (radius 2 is 5 x 5 x 5 voxels)")),
      field("contrast.resistor", &P::contrast_resistor, range(0.0, 1.0),
            doc("contrast: how much of a neighbour's local minimum, maximum and mean carries "
                "over (near 1 spreads them far)")),
      field("anisotropic.iterations", &P::anisotropic_iterations, range(1.0, 1000.0),
            doc("anisotropic: how many diffusion steps (more smooths further)")),
      field("gdtv.exponent", &P::gdtv_exponent, range(0.0, 2.0),
            doc("gdtv: the power of the gradient's magnitude the flow lowers (1 total variation, "
                "2 linear diffusion)")),
      field("gdtv.lambda", &P::gdtv_lambda, range(1e-9, none),
            doc("gdtv: the penalty factor pulling each voxel back toward the input")),
      field("gdtv.iterations", &P::gdtv_iterations, range(1.0, 1000.0),
            doc("gdtv: how many Jacobi passes")),
      field("gdtv.neighbours", &P::gdtv_neighbours,
            doc("gdtv: the neighbourhood, six (faces) or twenty_six (faces, edges, corners)")),
  };
};

// The filter, a few rows at a time: step(budget) does whole rows of voxels until about `budget`
// voxels are done (at least one row; budget 0 runs to the end), into a volume of its own. The input
// is shared and read only. phase() names the filter and, where it has more than one pass, the
// pass: "bilateral"; "contrast: mapping", "contrast: up", "contrast: down", "contrast:
// stretching", "contrast: restoring"; "anisotropic"; "gdtv: gradient", "gdtv: filtering".
// progress() rises over all of them. take() hands out the result once done. Throws
// std::invalid_argument for a null or empty input, or parameters out of range (every problem
// params_validate finds).
class volume_filter_stepper {
public:
  volume_filter_stepper(std::shared_ptr<const volume> in, const volume_filter_params &p);
  ~volume_filter_stepper();
  volume_filter_stepper(volume_filter_stepper &&) noexcept;
  volume_filter_stepper &operator=(volume_filter_stepper &&) noexcept;

  bool step(std::size_t budget);
  bool done() const;
  double progress() const;
  const char *phase() const;
  std::string status() const;
  std::shared_ptr<const volume> take();

private:
  struct impl;
  std::unique_ptr<impl> _impl;
};

// The filter as a job (cvc/core/result_job.h): for schedule_sliced_job, a node's keys or the DSL.
std::unique_ptr<result_job<std::shared_ptr<const volume>>>
volume_filter_job(std::shared_ptr<const volume> in, const volume_filter_params &p);

// The filter run to the end, here: a new volume (`in` is untouched).
volume filter_volume(const volume &in, const volume_filter_params &p);

} // namespace cvc

#endif // CVC_VOLUME_VOLUME_FILTER_H
