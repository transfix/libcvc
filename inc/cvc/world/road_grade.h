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

// cvc::world road grading — roads that shape the ground instead of hugging it.
//
//   std::vector<double> z = grade_road_profile(net, ground, p);  // a height per road node
//   auto graded = std::make_shared<graded_terrain>(ground_ptr, net, z, p);
//   graded->sample(x, y);  // the ground with the roads cut and filled into it
//
// A road laid on rough ground as it is rises and falls with every bump and tilts with every side
// slope, which a vehicle feels. Here each road node gets a height: the ground under it, kept above
// the water, then relaxed along the network so the grade changes gradually from one segment to the
// next, and held to max_grade; a road passing close to another it does not meet is held to within
// side_slope of it over the gap between them, so the ground between the two never has to step. A
// segment's surface is the straight run between its nodes' heights and level across, so the road is
// flat side to side and smooth end to end. graded_terrain is that surface pressed into the ground:
// within a road's half width (plus flat_margin) the ground is the road; past it the ground blends
// back to its own height over a cut or an embankment of side_slope. Where several roads reach a
// point, the ground blends toward all of their surfaces, the strongest influence dominating, so it
// stays continuous between them. Where a road runs along or over shallow water, the embankment is
// the shore it stands on.

#ifndef CVC_WORLD_ROAD_GRADE_H
#define CVC_WORLD_ROAD_GRADE_H

#include <cvc/world/roads.h>
#include <cvc/world/terrain_source.h>
#include <limits>
#include <memory>
#include <vector>

namespace cvc {
namespace world {

struct road_grade_params {
  double max_grade = 0.12;      // the steepest a road climbs along its length (rise over run)
  double junction_grade = 0.05; // ... on a segment meeting a junction, so a crossing is near level
  int smooth_passes = 60;       // relaxation passes along the network: grade changes spread out
  double smooth = 0.5;          // how far a pass moves a node toward its neighbours' mean height
  double hold = 0.04; // ... and back toward the ground under it, so it stays near the terrain
  // Roads stay water_clearance above this (-inf: no water).
  double water_level = -std::numeric_limits<double>::infinity();
  double water_clearance = 0.8;
  double flat_margin = 1.0;  // level ground past each road edge (about a terrain cell)
  double side_slope = 0.5;   // the cut or embankment back to the ground (rise over run)
  double min_shoulder = 2.0; // ... over at least this many metres
  double max_shoulder = 30.0;
};

struct road_segment_grid; // the roads near a point (road_grade.cpp)

// A height per node of `net` (nodes no segment uses keep the ground's): see the file header.
std::vector<double> grade_road_profile(const road_network &net, const terrain_source &ground,
                                       const road_grade_params &p = {});

// The ground with the roads graded into it (see the file header). The network and heights are
// copied in; the base ground is shared.
class graded_terrain : public terrain_source {
public:
  graded_terrain(std::shared_ptr<const terrain_source> ground, road_network net,
                 std::vector<double> node_z, const road_grade_params &p = {});

  double sample(double x, double y) const override;
  terrain_bounds bounds() const override { return ground_->bounds(); }
  double sea_level_m() const override { return ground_->sea_level_m(); }
  double relief_m() const override { return ground_->relief_m(); }
  bool has_mask(terrain_mask m) const override { return ground_->has_mask(m); }
  double mask(terrain_mask m, double x, double y) const override { return ground_->mask(m, x, y); }

  // True where a road may change the ground (within its reach: corridor and shoulder); elsewhere
  // sample() is the base ground's. A cheap test, for a caller re-sampling only near the roads.
  bool graded_at(double x, double y) const;
  // The road surface along segment `seg` at t in [0, 1].
  double road_height(int seg, double t) const;
  const terrain_source &ground() const { return *ground_; }
  const road_network &roads() const { return net_; }
  const std::vector<double> &node_heights() const { return z_; }

private:
  std::shared_ptr<const terrain_source> ground_;
  road_network net_;
  std::vector<double> z_;
  road_grade_params p_;
  double reach_ = 0.0; // the furthest any road changes the ground, from its centreline
  std::shared_ptr<const road_segment_grid> grid_;
  std::vector<int> run_; // each segment's road: segments joined through two-road nodes
};

} // namespace world
} // namespace cvc

#endif
