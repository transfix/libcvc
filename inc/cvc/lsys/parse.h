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

// parse.h — the `.lsys` text format reader.
//
// Format (v1):
//
//   # cvc lsystem v1                     <- comment lines preserved for round-trip
//   name: pine_monopodial
//   kind: plant                          <- plant|terrain|cloud|rock|building
//   mode: parallel                       <- parallel|sequential
//   contain: none                        <- none|strict
//   angle: 22.5                          <- default +/-/&/^ turn (degrees)
//   tilt: 30                             <- "T" (degrees)
//   roll: 90                             <- "R" (degrees)
//   step: 5.0                            <- default "F" length
//   width: 0.7                           <- default "!" width
//   taper: 0.1                           <- per-segment radius decay
//   preview_gen: 6
//   build_gen: 10
//   ignore: + - [ ]                      <- symbols skipped during context match
//   param r = 0.7                        <- a named global scalar
//   axiom: A(1)
//   A(s) : s > 0.05 -> F(s) [ +(25) A(s*r) ] [ -(25) A(s*r) ]
//   B < A > C : true -> F                <- context-sensitive form
//   A -> (0.4) F A                       <- (prob) after -> is the production probability
//
// A predecessor is `Name` or `Name(f1,f2)` with FORMAL parameter names bound
// from the matched module. A module in an axiom/successor is an operator char or
// an identifier, optionally with a parenthesised expression list.

#ifndef CVC_LSYS_PARSE_H
#define CVC_LSYS_PARSE_H

#include <cvc/lsys/grammar.h>
#include <string>
#include <vector>

namespace cvc {
namespace lsys {

struct parse_result {
  ruleset rs;
  std::vector<diagnostic> diags;
  bool ok = false;
};

parse_result parse_lsys(const std::string &src);

} // namespace lsys
} // namespace cvc

#endif // CVC_LSYS_PARSE_H
