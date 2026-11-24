/*
  Copyright 2003, 2026 The University of Texas at Austin

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

// cvc::segment_seeded: seeded segmentation of a volume by fast marching, ported from VolumeRover's
// GenSeg (transfix/volrover src/Segmentation/GenSeg) per
// docs/roadmap/VOLROVER-LIBRARY-PORTS-ROADMAP.md §5. It implements §2.2 of
//
//   C. Bajaj, Z. Yu, M. Auer, "Volumetric feature extraction and visualization of tomographic
//   molecular imaging", Journal of Structural Biology 144(1-2):132-143, 2003,
//   doi:10.1016/j.jsb.2003.09.037,
//
// which GenSeg's authors ask you to cite when you use it. The method:
//   1. the volume is rescaled to 0..255, and its gradient magnitude (a 3x3x3 operator) too;
//   2. automatic seeds (seed_mode::local_maxima): the local maxima above t_high of the volume
//      smoothed twice by a 7-point average;
//   3. each point class's points start a front of its class at time 0;
//   4. the automatic seeds are classified: a threshold sweeps down from 255 to t_low, and each seed
//      takes the class whose front reaches it first through voxels above the threshold;
//   5. every labelled voxel then starts a front, and the fronts advance by fast marching through
//      voxels at or above t_low, a step into a voxel costing 0.01 exp(-gamma |grad I|) (|grad I|
//      on 0..255), so fronts slow at edges. A front stops where it meets another class's front, or
//      a voxel below t_low.
//
// The paper's own seeds (seed_mode::critical_points, critical_point_seeds(); §2.1-2.2 and §4 of
// the paper, as VolumeRover's SegMed computes them): the critical points of the volume's gradient
// field after anisotropic gradient vector diffusion (cvc/volume/gradient_vector_diffusion.h). The
// maxima seed a "feature" class and the minima a "background" class, saddles are dropped, and the
// paper's §4 thresholds move dim maxima to the background and bright minima to the feature. With no
// point classes this is the paper's fully automatic two-class segmentation; point classes add more
// groups.
//
// The result has none of the legacy library's process-wide state (its ~25 globals), reads and
// writes no file, and leaves nothing in the current directory: its labels and per-class volumes
// come back as volumes. Its labels match the legacy library's voxel for voxel
// (src/cvc/tests/segmentation_test.cpp, against tools/volrover-goldens).
//
// Resumable (§1.9): segment_stepper runs every phase a unit at a time; segment_seeded() is the
// stepper at budget 0, and segment_seeded_job() its result_job. Parameters: a param_table
// (cvc/volume/segmentation_params.h) gives the state keys, CLI flags, DSL dict and pycvc keywords.

#ifndef CVC_VOLUME_SEGMENTATION_H
#define CVC_VOLUME_SEGMENTATION_H

#include <array>
#include <cstddef>
#include <cvc/core/result_job.h>
#include <cvc/volume/gradient_vector_diffusion.h>
#include <cvc/volume/point_classes.h>
#include <cvc/volume/volume.h>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
#ifndef SWIG
#include <boost/describe/enum.hpp>
#endif

namespace cvc {

// What is wrong with a segmentation's input: the message says what, for example
// "segment_seeded: seed (12, 40, 7) of class 'membrane' is outside the 64x64x32 volume".
class segmentation_error : public std::runtime_error {
public:
  using std::runtime_error::runtime_error;
};

// The automatic seeds added to the point classes' own.
enum class seed_mode {
  none,            // only the point classes' points
  local_maxima,    // GenSeg's: the local maxima above t_high of the volume smoothed twice
  critical_points, // the paper's (SegMed's): the critical points of the diffused gradient, as two
                   // more classes, "feature" and "background" (critical_point_seeds)
};
#ifndef SWIG
BOOST_DESCRIBE_ENUM(seed_mode, none, local_maxima, critical_points)
#endif

// The field whose critical points seed_mode::critical_points finds.
enum class critical_field {
  image,              // the gradient of the volume (the paper): its maxima are the volume's
                      // maxima, its minima the volume's minima
  gradient_magnitude, // the gradient of |grad I| (SegMed): its minima are the flat regions, bright
                      // or dark, and its maxima (the edges) are dropped
};
#ifndef SWIG
BOOST_DESCRIBE_ENUM(critical_field, image, gradient_magnitude)
#endif

// The defaults are GenSeg's; VolumeGridRover's Segmentation tab had no defaults for the
// thresholds, which the user had to give.
struct seeded_segmentation_params {
  // Voxels below t_low stop every front. Automatic seeds are the smoothed volume's local maxima
  // above t_high. In the voxels' units, or with normalized_thresholds on the legacy 0..255 scale
  // the volume is rescaled to (its minimum 0, its maximum 255). Thresholds in data units are
  // rescaled the same way, so they are compared on that scale.
  double t_low = 0.0;
  double t_high = 0.0;
  bool normalized_thresholds = false;
  // A step into a voxel costs 0.01 exp(-gamma |grad I|), |grad I| rescaled to 0..255: the speed
  // F = exp(gamma |grad I|) of the paper, which slows fronts at edges for gamma < 0.
  double gamma = -0.12;
  seed_mode automatic = seed_mode::local_maxima;
  // The point classes of this variable and timestep seed the run, in their order: class n of
  // the result is the n-th of them (a class with no points labels nothing).
  int variable = 0;
  int timestep = 0;
  // Also make a volume per class (seeded_segmentation::classes).
  bool class_volumes = true;

  // seed_mode::critical_points (and critical_point_seeds). cp_iterations and cp_k default to
  // SegMed's (GVF.cpp), the rest to the paper's; SegMed's own are cp_field = gradient_magnitude,
  // cp_weights = initial and cp_sample = 20, with cp_background_threshold its tlow.
  //   The volume, rescaled to 0..255, gives a gradient field (cp_field), divided by its longest
  // vector's length, which cp_iterations of gradient vector diffusion with Eq. 3's cp_k smooth
  // (cvc::gradient_vector_diffusion, the neighbours weighed by the angles of the field cp_weights
  // names). Each voxel where no neighbour's vector points away from it is a maximum, and where
  // none points towards it a minimum (cvc::critical_point_at; around an extremum that is the
  // whole 3x3x3 block). The paper's §4: a maximum below cp_feature_threshold joins the
  // background, a minimum above cp_background_threshold the feature (in data units, or on 0..255
  // with normalized_thresholds; the defaults move none). With cp_sample > 0 each class's
  // 26-connected runs of critical voxels are thinned to one seed in every cp_sample + 1, as SegMed
  // thinned them (its first in x-fastest order, which on a steep extremum can be a voxel the march
  // cannot leave); 0, the paper's, keeps every critical voxel.
  int cp_iterations = 10;
  double cp_k = 2.0;
  double cp_feature_threshold = -std::numeric_limits<double>::infinity();
  double cp_background_threshold = std::numeric_limits<double>::infinity();
  critical_field cp_field = critical_field::image;
  diffusion_weights cp_weights = diffusion_weights::current;
  int cp_sample = 0;
};

struct seeded_segmentation {
  static constexpr unsigned char unlabelled = 255;

  explicit seeded_segmentation(app &ctx) : labels(ctx, dimension(1, 1, 1), UChar) {}

  // UChar, the input's dimensions and bounding box: each voxel's class (0, 1, ...), or
  // `unlabelled`.
  volume labels;
  // Per class, unless class_volumes is off: the input's voxels where the label is that class, 0
  // elsewhere, in the input's voxel type (the legacy <prefix>_subunit_<n>.rawiv, which held the
  // rescaled values instead).
  std::vector<volume> classes;
  // The voxels where a front stopped: next to a voxel of another class or below t_low (the
  // legacy <prefix>_indices.txt), in x-fastest order. Each is labelled with its own class.
  std::vector<std::array<int, 3>> border;
  // Per class: its name, and how many voxels it labels.
  std::vector<std::string> class_names;
  std::vector<std::size_t> class_voxels;
  // The automatic seeds found (but for those under a point class's seed), and how many of them
  // the threshold sweep gave a class (the rest were left to the final march, which may still
  // reach them). With seed_mode::critical_points: the critical points' seeds, every one of which
  // has its class (feature or background), so the two are equal.
  std::size_t automatic_seeds = 0;
  std::size_t classified_seeds = 0;
  // The thresholds as the run used them, on the 0..255 scale.
  double t_low_normalized = 0.0;
  double t_high_normalized = 0.0;
  std::vector<std::string> warnings;
};

// The segmentation of `vol` seeded by the classes of `params.variable` and `params.timestep` in
// `seeds`. Throws segmentation_error when no class has a point, a point lies outside the volume,
// there are more than 254 classes, the volume is constant or has a voxel that is not finite (NaN
// or infinite), its voxel type is undefined, or `params` is not valid (params_validate).
// Warnings, not errors: a seed below t_low (it labels only itself and its neighbours), a voxel
// that is a point of two classes (the later class has it), a class with no points.
//   With seed_mode::critical_points the critical points (critical_point_seeds) are two more
// classes after the point classes, "feature" then "background" (so at most 252 point classes),
// and the point classes may have no points at all: then the critical points alone seed the run
// (it throws when there are none either). A point class's seed wins a voxel it shares with a
// critical point.
seeded_segmentation segment_seeded(const volume &vol, const point_classes &seeds,
                                   const seeded_segmentation_params &params = {});

// segment_seeded, a slice at a time (§1.9). Phases: "checking" (the parameters and seeds, a unit
// per seed point), "normalising" (reading the voxels, the rescale and the gradient, a unit per
// voxel), "seeding" (the smoothing and the local-maximum scan, a unit per voxel; with
// seed_mode::critical_points, critical_point_seeds' gradient field, diffusion, critical points and
// thinning, a unit per voxel, voxel update or voxel visited), "classifying"
// (the threshold sweep: a unit per voxel visited, popped from a heap or stepped through),
// "marching" (the final fast marching, a unit per voxel popped) and "writing" (the labels, the
// border and the class volumes, a unit per voxel). Every unit costs about a microsecond at most,
// whatever the volume's size, but for allocating each output volume (filling its bytes with
// zeros). A failure is a segmentation_error thrown from step() (and rethrown by later steps and
// take()). cancel() is the one thread-safe member: the step running (or the next) ends the
// stepper with no result; a result_job around it ends "cancelled". Not thread-safe otherwise,
// but it may move between threads between steps. Constructing one only stores its input (the
// volume shares its voxels with the caller's copy, which must not change while the stepper
// runs).
class segment_stepper {
public:
  segment_stepper(const volume &vol, point_classes seeds, seeded_segmentation_params params = {});
  ~segment_stepper();
  segment_stepper(segment_stepper &&) noexcept;
  segment_stepper &operator=(segment_stepper &&) noexcept;

  bool step(std::size_t budget); // true once done; budget 0 runs to the end
  bool done() const;
  const char *phase() const; // as listed above, "done", or "failed"
  double progress() const;   // in [0, 1], monotone; 1 once done
  std::string status() const;
  void cancel();
  seeded_segmentation take(); // once done; moved out, so call it once

  // The labels so far, between steps, as the fronts advance (VolumeGridRover's Segmentation tab
  // draws them while it marches): each voxel's class, or seeded_segmentation::unlabelled, written
  // into `labels`, which is made a UChar volume of the input's dimensions and bounding box first
  // when it is not one. True while there are labels to show, in the "classifying" and "marching"
  // phases (the point classes' seeds, then the fronts from them); false otherwise, `labels` left
  // as it was (the finished labels are take()'s). A copy of every voxel, outside the step budget:
  // call it a few times a second, not after every step. Like step(), not thread-safe.
  bool labels_so_far(volume &labels) const;

private:
  struct impl;
  std::unique_ptr<impl> _impl;
};

std::unique_ptr<segment_stepper> segment_seeded_stepper(const volume &vol, point_classes seeds,
                                                        seeded_segmentation_params params = {});
std::unique_ptr<result_job<seeded_segmentation>>
segment_seeded_job(const volume &vol, point_classes seeds, seeded_segmentation_params params = {});

// The paper's seeds of `vol` (seed_mode::critical_points, as seeded_segmentation_params' cp_*
// fields describe): two classes, "feature" then "background", of `params.variable` and
// `params.timestep`, their points in x-fastest order. Either may be empty. Throws
// segmentation_error when the volume is constant, has a voxel that is not finite, its voxel type
// is undefined, or `params` is not valid. `params.automatic` and the march's fields (t_low, t_high,
// gamma, class_volumes) are not used.
point_classes critical_point_seeds(const volume &vol,
                                   const seeded_segmentation_params &params = {});

// critical_point_seeds, a slice at a time (§1.9), as segment_stepper is. Phases: "checking" (the
// parameters), "normalising" (reading the voxels and the rescale, a unit per voxel) and "seeding"
// (the gradient field, the diffusion, the critical points and the thinning, a unit per voxel or
// voxel visited, each about a microsecond).
class critical_point_seeds_stepper {
public:
  critical_point_seeds_stepper(const volume &vol, seeded_segmentation_params params = {});
  ~critical_point_seeds_stepper();
  critical_point_seeds_stepper(critical_point_seeds_stepper &&) noexcept;
  critical_point_seeds_stepper &operator=(critical_point_seeds_stepper &&) noexcept;

  bool step(std::size_t budget); // true once done; budget 0 runs to the end
  bool done() const;
  const char *phase() const; // as listed above, "done", or "failed"
  double progress() const;   // in [0, 1], monotone; 1 once done
  std::string status() const;
  void cancel();
  point_classes take(); // once done; moved out, so call it once

private:
  struct impl;
  std::unique_ptr<impl> _impl;
};

std::unique_ptr<result_job<point_classes>>
critical_point_seeds_job(const volume &vol, seeded_segmentation_params params = {});

} // namespace cvc

#endif // CVC_VOLUME_SEGMENTATION_H
