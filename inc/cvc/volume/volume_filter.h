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
// The filters:
//   bilateral   VolumeRover's (VolMagick's voxels::bilateralFilter): each voxel the mean of its
//               neighbours within `radius`, weighted by a Gaussian of their distance
//               (spatial_sigma, voxels) and of their value's difference (radiometric_sigma, data
//               units, the difference taken in 256 steps of the volume's range). The same weights
//               as voxels::bilateralFilter, voxel for voxel. Each value is written in the volume's
//               own voxel type, where voxels::bilateralFilter writes every type other than float
//               and double as a byte. A constant volume comes back as it is.
// Contrast enhancement, anisotropic diffusion and GDTV come later as more `filter` values.
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

enum class volume_filter_kind { bilateral };
BOOST_DESCRIBE_ENUM(volume_filter_kind, bilateral)

struct volume_filter_params {
  volume_filter_kind filter = volume_filter_kind::bilateral;
  double radiometric_sigma = 200.0; // bilateral: data units
  double spatial_sigma = 1.5;       // bilateral: voxels
  std::int64_t radius = 2;          // bilateral: the neighbourhood's half-width, voxels
};

template <> struct param_table<volume_filter_params> : param_fields<volume_filter_params> {
  using P = volume_filter_params;
  static constexpr double none = std::numeric_limits<double>::quiet_NaN();
  static constexpr auto fields = std::tuple{
      field("filter", &P::filter, doc("the filter: bilateral")),
      field("radiometric_sigma", &P::radiometric_sigma, unit("data"), range(1e-9, none),
            doc("bilateral: how far apart two values may be and still blend (small keeps edges)")),
      field("spatial_sigma", &P::spatial_sigma, unit("voxel"), range(1e-9, none),
            doc("bilateral: how fast a neighbour's weight falls with its distance")),
      field("radius", &P::radius, unit("voxel"), range(0.0, 16.0),
            doc("bilateral: the neighbourhood's half-width (radius 2 is 5 x 5 x 5 voxels)")),
  };
};

// The filter, a few rows at a time: step(budget) filters whole rows of voxels until about `budget`
// voxels are done (at least one row; budget 0 runs to the end), into a volume of its own. The input
// is shared and read only. phase() is "bilateral"; take() hands out the result once done. Throws
// std::invalid_argument for a null or empty input, or parameters out of range.
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
