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
//   road_network dense = subdivide_roads(net, 8.0);              // nodes at most 8 m apart
//   std::vector<double> z = grade_road_profile(dense, ground, p); // a height per road node
//   auto graded = std::make_shared<graded_terrain>(ground_ptr, dense, z, p);
//   graded->sample(x, y);  // the ground with the roads cut and filled into it
//
// A road laid on rough ground as it is rises and falls with every bump and tilts with every side
// slope, which a vehicle feels. Here each road node gets a height: the ground under it, kept above
// the water; then, along the network, the grade is held to max_grade and its change to
// max_curvature per metre. Only what breaks a limit moves, so a road keeps to a gentle hill and
// cuts through a steep one: a node is relaxed toward its neighbours only by what its bend exceeds,
// and a climb too steep comes down at its top more than it rises at its foot (cut_share), so roads
// sit in cuts rather than up on banks. A road passing close to another it does not meet is held to
// within side_slope of it over the gap between them. A segment's surface is the straight run
// between its nodes' heights, level across, so grade a network whose nodes are close
// (subdivide_roads): a long segment would bridge a dip or cut a hill whole.
//
// graded_terrain presses that surface into the ground as earthworks do: within a road's half width
// (plus flat_margin) the ground is the road; past it the ground keeps its own height unless it
// rises above the cut or falls below the embankment climbing or falling side_slope from the road's
// edge (steeper where it must, to meet the ground within max_shoulder). Where several roads reach a
// point, the ground keeps within all their cuts and banks; where those cannot all hold (two roads
// at very different heights), it settles between them, nearer the nearer road's. So the ground
// beside a road is dug below it only by another road's cut, and a road over a hill stands on the
// hill. Where a road runs along or over shallow water, the embankment is the shore it stands on.

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
  // The most the grade changes per metre along a road: a crest or dip takes 13 m to turn a 20%
  // change of grade (a vehicle at 15 m/s feels 3.4 m/s^2 over it); a bend in the ground gentler
  // than this is followed as it is.
  double max_curvature = 0.015;
  double cut_share = 0.75; // of a climb too steep, the share taken off its top (the rest raises
                           // its foot): roads cut through hills more than they bank over dips
  int smooth_passes = 400; // the most passes the limits take to settle
  double smooth = 0.5;     // how far a pass moves a node by what its bend exceeds
  // Roads stay water_clearance above this (-inf: no water).
  double water_level = -std::numeric_limits<double>::infinity();
  double water_clearance = 0.8;
  double flat_margin = 1.0;   // level ground past each road edge (about a terrain cell)
  double side_slope = 0.5;    // the cut or embankment back to the ground (rise over run)
  double min_shoulder = 2.0;  // roads whose corridors come within 2x this are held to agree
  double max_shoulder = 40.0; // a cut or bank meets the ground within this, steeper if it must
};

struct road_segment_grid; // the roads near a point (road_grade.cpp)

// `net` with every segment longer than `max_len` split evenly into pieces no longer: the same
// roads (widths, classes) through extra two-road nodes, which are appended after the original
// nodes (so those keep their indices).
road_network subdivide_roads(const road_network &net, double max_len);

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
