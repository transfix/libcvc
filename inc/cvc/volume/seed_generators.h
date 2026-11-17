/*
  Copyright 2005-2011, 2026 The University of Texas at Austin

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

// Seed generators for the seeded segmentation (cvc/volume/segmentation.h), from VolumeRover's
// VolUtils (transfix/libcvc#123; the sources are at transfix/volrover edabb85^, src/VolUtils,
// removed from volrover's master by #19):
//
//   - capsid_critical_points: criticalPntsToSeed, which ran SegSubunit's FindCriticalPoints: the
//     local maxima of a virus map above a threshold chosen so that about 3000 T of them are kept
//     (T = h^2 + hk + k^2, the capsid's triangulation number), and, when there are few of those,
//     the local maxima of its gradient magnitude ("saddles" in the legacy code) too;
//   - rotated_seeds: SeedProduce and SeedProduceRegardingAxis, the N-fold rotational copies of a
//     set of seeds about an axis (a capsid's symmetry axis), class n holding the n-th rotation.
//
// None reads or writes a file (the legacy tools read .rawiv and .vgr files and wrote maxcrit.vgr
// and saddlecrit.vgr into the current directory); cvc::read_point_classes / write_point_classes and
// `cvc capsid-seeds` / `cvc rotate-seeds` do that. Parameters: param tables in
// cvc/volume/seed_generators_params.h.

#ifndef CVC_VOLUME_SEED_GENERATORS_H
#define CVC_VOLUME_SEED_GENERATORS_H

#include <cstddef>
#include <cvc/core/result_job.h>
#include <cvc/volume/bounding_box.h>
#include <cvc/volume/dimension.h>
#include <cvc/volume/point_classes.h>
#include <cvc/volume/volume.h>
#include <memory>
#include <string>
#include <vector>

namespace cvc {

// The legacy defaults (criticalPntsToSeed with only a volume: h 2, k 1, the threshold the
// volume's minimum).
struct capsid_seed_params {
  // Only voxels above t_low count: in the voxels' units, or with normalized_thresholds on the
  // 0..255 scale the volume is rescaled to (the legacy tool's, so the default).
  double t_low = 0.0;
  bool normalized_thresholds = true;
  // The capsid's triangulation numbers: about 3000 (h^2 + hk + k^2) maxima are kept.
  int h = 2;
  int k = 1;
  // The variable and timestep the result's classes mark.
  int variable = 0;
  int timestep = 0;
};

// The capsid seeds of `vol`: two classes, "maxima" then "saddles", their points in x-fastest order.
// The volume, rescaled to 0..255, gives its local maxima above t_low (no voxel of the 3x3x3 block
// higher); a histogram of their values gives the threshold above which about 3000 T of them lie
// (but at least t_low + 20), and the maxima above it are the "maxima" class. When those are fewer
// than min(50000, 600 T), the same is done to the gradient magnitude (GenSeg's 3x3x3 operator,
// zero where the 3x3x3 block touches a zero voxel), keeping about as many as there were maxima: the
// "saddles" class (empty otherwise). Throws segmentation_error when the volume is constant, has a
// voxel that is not finite, its voxel type is undefined, or the params are not valid.
point_classes capsid_critical_points(const volume &vol, const capsid_seed_params &params = {});

// capsid_critical_points, a slice at a time (docs/roadmap/VOLROVER-LIBRARY-PORTS-ROADMAP.md §1.9).
// Phases: "checking", "normalising" (a unit per voxel), "maxima" (the scan and the list, a unit
// per voxel) and "saddles" (the gradient, its rescale, its scan and the list, a unit per voxel).
class capsid_critical_points_stepper {
public:
  capsid_critical_points_stepper(const volume &vol, capsid_seed_params params = {});
  ~capsid_critical_points_stepper();
  capsid_critical_points_stepper(capsid_critical_points_stepper &&) noexcept;
  capsid_critical_points_stepper &operator=(capsid_critical_points_stepper &&) noexcept;

  bool step(std::size_t budget); // true once done; budget 0 runs to the end
  bool done() const;
  const char *phase() const;
  double progress() const;
  std::string status() const;
  void cancel();
  point_classes take();

private:
  struct impl;
  std::unique_ptr<impl> _impl;
};

std::unique_ptr<result_job<point_classes>>
capsid_critical_points_job(const volume &vol, capsid_seed_params params = {});

struct rotation_seed_params {
  int folds = 2; // copies at the angles 2 pi n / folds, n = 0 .. folds - 1
  // The axis's direction, and a point on it, in world units. An empty center is the target
  // volume's centre: with the default axis, SeedProduce's rotation about z through the volume's
  // centre.
  std::vector<double> axis{0.0, 0.0, 1.0};
  std::vector<double> center{};
  // The classes of this variable and timestep are rotated, and the result's classes mark them.
  int variable = 0;
  int timestep = 0;
};

// Every point of the classes of `params.variable` and `params.timestep` in `seeds`, as voxel
// indices of a grid of `from_dims` voxels over `from_box` (world = min + index * span), rotated by
// 2 pi n / folds about the axis, as the nearest voxel of `to`: class n of the result ("Class n",
// in the legacy colours) holds the n-th rotation of every point, in order. Rotations that land
// outside `to` are dropped. SeedProduce rotated seeds marked on a cut of the volume (`from`) into
// the whole volume; SeedProduceRegardingAxis about any axis. Throws segmentation_error when folds
// is below 1, the axis is zero or the vectors are not three finite numbers, or a grid is empty.
point_classes rotated_seeds(const point_classes &seeds, const dimension &from_dims,
                            const bounding_box &from_box, const volume &to,
                            const rotation_seed_params &params = {});
// The same with the seeds on `to`'s own grid.
point_classes rotated_seeds(const point_classes &seeds, const volume &to,
                            const rotation_seed_params &params = {});

} // namespace cvc

#endif // CVC_VOLUME_SEED_GENERATORS_H
