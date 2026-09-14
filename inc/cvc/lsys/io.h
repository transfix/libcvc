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

// io.h — the `.lsys` canonical writer + file helpers.
//
// write_lsys emits a canonical form; a file authored in that form round-trips
// byte-for-byte (write(parse(canonical)) == canonical). parse(write(rs)) is
// always semantically equal to rs.

#ifndef CVC_LSYS_IO_H
#define CVC_LSYS_IO_H

#include <cvc/lsys/grammar.h>
#include <cvc/lsys/parse.h>
#include <string>

namespace cvc {
namespace lsys {

std::string write_lsys(const ruleset &rs);

// File convenience (throws std::runtime_error on I/O failure).
parse_result parse_lsys_file(const std::string &path);
void write_lsys_file(const std::string &path, const ruleset &rs);

} // namespace lsys
} // namespace cvc

#endif // CVC_LSYS_IO_H
