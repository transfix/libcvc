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

// CoPH-Stagewise's configuration records (GRL-SNAM's grl_snam.coph.records.Config and
// grl_snam.coph.cooperative.CommunicationConfig), in C++: C1 of docs/roadmap/NAV-COPH-ROADMAP.md
// (§4.1).
//
//   coph_config                the learner's settings, with Python's defaults. validate() is
//                              Config.__post_init__: the same refusals, with the same messages, in
//                              the same order, and it fills replay_update_cap the same way.
//   coph_communication_config  the packet settings; validate() is CommunicationConfig's.
//   outside_profile / require_profile   the mode fields that differ from the demos' profile.
//                              The C++ twin implements that profile only and refuses the rest
//                              (decision D7).
//
// GRL-SNAM's golden (grl_snam.tools.coph_golden, committed as src/cvc/tests/data/coph_golden.json)
// pins every acceptance, refusal and value here (case G1).

#ifndef CVC_NAV_COPH_CONFIG_H
#define CVC_NAV_COPH_CONFIG_H

#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace cvc {
namespace nav {

// Config. Every double is SI; the defaults are Python's. The modes are strings, as in Python, so
// that a mode Python refuses is refused here with Python's message.
struct coph_config {
  std::string stage_blend_mode = "legacy";
  std::string route_field_mode = "anchors";
  std::string controller_mode = "legacy";
  std::string shaping_mode = "legacy";
  double shaping_k = 1.0;
  double shaping_ell = 0.5;
  double shaping_eta = 0.5;
  double safety_margin = 0.05;
  int obstacle_cap = 256;
  int footprint_cap = 120;
  int route_expansion_cap = 4096;
  std::string navigation_target = "mission";
  std::string replay_objective = "legacy";
  std::string replay_domain = "single";
  std::string replay_horizon_mode = "fixed";
  std::optional<int> replay_update_cap; // None in Python; validate() may fill it with 1024
  int boundary_max_steps = 32;
  int boundary_follow_steps = 4;
  double dt = 0.12;
  int steps = 180;
  double stage_width = 3.0;
  double stage_height = 4.0;
  double overlap = 0.30;
  int previous = 2;
  int sample_cap = 240;
  int transition_cap = 120;
  int anchor_cap = 12;
  int rbf_side = 4;
  double rbf_width = 0.8;
  int update_every = 8;
  int replay_steps = 8;
  int solver_steps = 8;
  double damping_initial = 0.6;
  std::pair<double, double> policy_initial{1.0, 2.0};
  std::pair<double, double> damping_bounds{0.15, 1.5};
  std::pair<double, double> policy_bounds{0.0, 10.0};
  double damping_trust = 0.15;
  double policy_trust = 0.25;
  double fit_ridge = 0.02;
  double validation_slack = 0.003;
  double sensor_radius = 1.25;
  double force_limit = 3.0;

  // Config.__post_init__. Throws std::invalid_argument with Python's message at the first check
  // that fails. On success it fills replay_update_cap with 1024 where Python does: for any
  // controller mode other than legacy, and for supported_boundary horizons. On a throw the config
  // is unchanged. A NaN passes the scale checks wherever it passes Python's min(), as upstream's
  // does.
  void validate();

  // The mode fields that differ from the demos' profile, in coph_profile_modes()' order.
  std::vector<std::string> outside_profile() const;
};

// The demos' profile (crossing_demo.configuration, hazard_demo's): each mode field and its value.
// Returned by value: libcvc keeps no process-wide state, a function-local static included.
inline std::vector<std::pair<std::string, std::string>> coph_profile_modes() {
  return {
      {"stage_blend_mode", "legacy"},   {"route_field_mode", "anchors"},
      {"controller_mode", "routed"},    {"shaping_mode", "progress"},
      {"navigation_target", "mission"}, {"replay_objective", "mission"},
      {"replay_domain", "retained"},    {"replay_horizon_mode", "supported_boundary"},
  };
}

// The profile as the demos build it: Config(controller_mode='routed', shaping_mode='progress',
// replay_objective='mission', replay_domain='retained',
// replay_horizon_mode='supported_boundary', replay_update_cap=1024), validated. The demos also set
// steps (2500 crossing, 1800 hazard).
coph_config coph_profile();

// CommunicationConfig. The defaults are Python's; the demos run coph_communication_profile().
struct coph_communication_config {
  int ensemble_size = 8;
  int anchors = 4;
  int received_cap = 8;
  double ttl = 2.0;
  double temporal_window = 3.0;
  double covariance_floor = 0.0025;
  double covariance_scale = 1.0;
  int byte_budget = 240000;
  double byte_price = 0.00001;
  double processing_price = 0.0001;
  double rate_regularizer = 0.01;
  double max_rate = 4.0;
  int delay_ticks = 0;
  std::string prediction_variant = "legacy";

  // CommunicationConfig.__post_init__: throws std::invalid_argument("invalid prediction_variant")
  // unless it is legacy, baseline, compact, cross_stage or both.
  void validate() const;

  // {"prediction_variant"} unless it is legacy, the demos' variant.
  std::vector<std::string> outside_profile() const;
};

// CommunicationConfig(byte_budget=4000000, ttl=300.), both demos' communication, validated.
coph_communication_config coph_communication_profile();

// Throws std::invalid_argument naming the fields outside the demos' profile, if any (D7). It does
// not validate.
void require_profile(const coph_config &cfg);
void require_profile(const coph_communication_config &comm);

} // namespace nav
} // namespace cvc

#endif
