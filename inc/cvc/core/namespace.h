/*
  Copyright 2011 The University of Texas at Austin

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

#ifndef __CVCNAMESPACE_H__
#define __CVCNAMESPACE_H__

#include <cvc/core/config.h>

// Backward-compat: many older consumers (F2Dock, volrover, TexMol) reference
// symbols as `CVC::Foo`. Provide `CVC` as a *real* nested namespace with a
// using-directive into the canonical lowercase namespace so those references
// resolve transparently — and so consumer code is still free to reopen
// `namespace CVC { ... }` to add its own aliases (which would be forbidden
// if CVC were a namespace *alias*).
namespace cvc {}
namespace CVC {
using namespace cvc;
}

#endif
