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

#include <cvc/config.h>

#ifndef CVC_NAMESPACE
#define CVC_NAMESPACE cvc
#endif

// Backward-compat alias so legacy code that uses `CVC::` continues to work.
// (Older code in F2Dock / volrover / TexMol references CVC::Foo.)
namespace CVC_NAMESPACE {}
namespace CVC = CVC_NAMESPACE;

#endif
