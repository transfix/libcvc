/*
  Copyright 2007-2011 The University of Texas at Austin

        Authors: Joe Rivera <transfix@ices.utexas.edu>
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

  This file is part of VolMagick.

  VolMagick is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.

  VolMagick is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

// CoPH-Stagewise's stage ledger, in C++: C1 of docs/roadmap/NAV-COPH-ROADMAP.md (§4.1). It is the
// ledger of the demos' own learner, crossing_demo.CrossingLearner, a maze_learning.MazeLearner
// (GRL-SNAM's grl_snam.coph):
//
//   coph_shape        an obstacle: a disk or a slab (shapes.py).
//   coph_reading, coph_transition, coph_footprint, coph_observation   the sensed records.
//   coph_stage        one stage: its box, RBF centres and weights, entry state, and the
//                     readings, transitions, footprints and obstacles it holds (Stage).
//   coph_ledger       the current stage and the previous cfg.previous, with observe and advance.
//
// MazeLearner's rule, which this implements, is not learner.Learner.advance's:
//   - It advances along whichever axis left the inner box, by size * (1 - overlap), in either
//     direction (3 x 4 m stages at 30 % overlap step 2.1 m in x and 2.8 m in y).
//   - A new stage pools the readings of every kept stage that it contains, oldest stage first,
//     with each uid once, and the footprints of every kept stage, each once.
//   - It never sets Stage.anchors, so _field_fit's overlap continuity rows are always empty in the
//     demos. coph_stage has no anchors.
//
// The geometry store (navigation.observe_geometry, MazeLearner.refresh_geometry) belongs to the
// navigation layer, which hands it to set_geometry; each stage's obstacles are the store's shapes
// that intersect its box. Learning (the field and damping fits, replay, the policy update) is C3.
//
// GRL-SNAM's golden (grl_snam.tools.coph_golden, committed as src/cvc/tests/data/coph_golden.json,
// case G4) pins every advance, centre, retention and list here bit for bit; its stage_rules text
// states the rules. coph_ledger.cpp is compiled with -ffp-contract=off.

#ifndef CVC_NAV_COPH_LEDGER_H
#define CVC_NAV_COPH_LEDGER_H

#include <array>
#include <cstdint>
#include <cvc/nav/coph_config.h>
#include <optional>
#include <string>
#include <vector>

namespace cvc {
namespace nav {

using coph_vec2 = std::array<double, 2>;
using coph_state = std::array<double, 4>; // x, y, vx, vy

// A disk (x, y, r), or a slab (x, y, r, hx, hy): every point within r of the segment from
// (x - hx, y - hy) to (x + hx, y + hy).
struct coph_shape {
  double x = 0, y = 0, r = 0, hx = 0, hy = 0;
  bool slab = false;

  static coph_shape disk(double x, double y, double r) { return {x, y, r, 0.0, 0.0, false}; }
  static coph_shape segment(double x, double y, double r, double hx, double hy) {
    return {x, y, r, hx, hy, true};
  }
  // A radius about (x, y) that contains the shape: r for a disk, r + hypot(hx, hy) for a slab.
  double reach() const;

  bool operator==(const coph_shape &o) const {
    return slab == o.slab && x == o.x && y == o.y && r == o.r && hx == o.hx && hy == o.hy;
  }
  bool operator!=(const coph_shape &o) const { return !(*this == o); }
};

// A point reading of the risk field. The ledger keeps uid as given; the cooperative learner
// prefixes its own readings with its agent id ("<agent>:<uid>"). A uid ending in a multiple of 5
// is a validation reading (C3).
struct coph_reading {
  std::string uid;
  double time = 0;
  coph_vec2 q{};
  double value = 0;
};

// One plant step: the state x, the force u held over it, the state after, and the learner's
// version when it was taken.
struct coph_transition {
  std::int64_t uid = 0;
  double time = 0;
  coph_state x{};
  coph_vec2 u{};
  coph_state after{};
  std::int64_t version = 0;
};

// Where the sensor looked: its centre, radius and the time.
struct coph_footprint {
  double x = 0, y = 0, radius = 0, time = 0;
  bool operator==(const coph_footprint &o) const {
    return x == o.x && y == o.y && radius == o.radius && time == o.time;
  }
  bool operator!=(const coph_footprint &o) const { return !(*this == o); }
};

// What the ledger takes from one Observation. Its obstacles go to the navigation layer, which
// owns the geometry store.
struct coph_observation {
  double time = 0;
  std::vector<coph_reading> readings;
  std::optional<coph_vec2> sensor_center; // no footprint without it
  double sensor_radius = 0;
};

// One stage (records.Stage, as new_stage builds it).
struct coph_stage {
  std::int64_t id = 0;
  coph_vec2 center{};
  coph_state entry{}; // the state it was entered at
  coph_vec2 target{}; // (min(goal.x, center.x + stage_width / 2 - 0.1), goal.y)
  std::vector<coph_vec2>
      centers;                 // rbf_side^2 RBF centres, x outer: [(x, y) for x in xs for y in ys]
  std::vector<double> weights; // the field weights; -1 / rbf_side^2 when made
  std::vector<coph_reading> readings;       // the last sample_cap
  std::vector<coph_transition> transitions; // the last transition_cap (the newest stage takes them)
  std::vector<coph_footprint> footprints;   // the last footprint_cap
  std::vector<coph_shape> obstacles;        // the geometry store's shapes that intersect the box
  std::int64_t version = 0;                 // the learner's version when it was made

  // |q - center| within half the stage's size, plus 1e-10, on both axes.
  bool contains(const coph_vec2 &q, const coph_config &cfg) const;
};

// new_stage: a stage centred on `center`, entered at `entry`, its RBF centres on numpy.linspace
// grids over the box and its weights all -1 / rbf_side^2.
coph_stage make_coph_stage(std::int64_t id, const coph_vec2 &center, const coph_state &entry,
                           const coph_vec2 &goal, const coph_config &cfg);

// navigation.intersects: d = max(|shape.xy - center| - (stage_width, stage_height) / 2, 0), and
// d.d <= (reach + 0.12 + safety_margin)^2 + 1e-12. The 0.12 is the agent's body radius.
bool coph_intersects(const coph_shape &shape, const coph_vec2 &center, const coph_config &cfg);

// The ledger of one agent's stages.
class coph_ledger {
public:
  // The first stage is centred on initial's position, entered at initial, with version 0.
  // Validates cfg, and throws std::invalid_argument if it is refused or outside the demos'
  // profile (D7).
  coph_ledger(coph_config cfg, const coph_state &initial, const coph_vec2 &goal);

  const coph_config &config() const { return cfg_; }
  const coph_vec2 &goal() const { return goal_; }
  // The kept stages, oldest first; the last is the current stage. At most cfg.previous + 1.
  const std::vector<coph_stage> &stages() const { return stages_; }
  const coph_stage &current() const { return stages_.back(); }
  const std::vector<coph_shape> &geometry() const { return geometry_; }

  // Learner.observe's ledger part, on each kept stage in order:
  //   1. with a sensor centre, the footprint (centre, radius, time) is appended to every stage
  //      whose box the sensor disk intersects, which then keeps its last footprint_cap;
  //   2. each reading the stage contains and whose uid it does not already hold is appended, in
  //      order, and the stage keeps its last sample_cap;
  //   3. a transition is appended to the current stage, which keeps its last transition_cap.
  void observe(const coph_observation &observation, const coph_transition *transition = nullptr);

  // The navigation layer's geometry store, in its order. Every stage's obstacles become the shapes
  // that intersect it, in that order.
  void set_geometry(std::vector<coph_shape> store);

  // MazeLearner.advance's ledger part. From the current stage s, delta = |x.xy - s.center| and
  // size = (stage_width, stage_height). Nothing happens unless delta[k] > (size[k] * (1 -
  // overlap)) / 2 for some k. Then axis = argmax(delta / size) (x on a tie), and the new stage,
  // with id s.id + 1, is centred on s.center moved by sign(x[axis] - center[axis]) * (size[axis] *
  // (1 - overlap)) along that axis. It is entered at x with the learner's `version`, and takes:
  //   - every kept stage's readings that it contains, oldest stage first, each uid at its first
  //     place, then the last sample_cap;
  //   - every kept stage's footprints, oldest first, each once, then the last footprint_cap;
  //   - no transitions.
  // The ledger then keeps the last previous + 1 stages, and every stage's obstacles are taken
  // from the geometry store again. Returns whether it advanced.
  bool advance(const coph_state &x, std::int64_t version);

private:
  void refresh_obstacles();

  coph_config cfg_;
  coph_vec2 goal_;
  std::vector<coph_stage> stages_;
  std::vector<coph_shape> geometry_;
};

} // namespace nav
} // namespace cvc

#endif
