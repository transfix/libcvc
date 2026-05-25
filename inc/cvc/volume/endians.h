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

#ifndef __CVC_ENDIANS_H__
#define __CVC_ENDIANS_H__

#include <cvc/core/namespace.h>

#define SWAP_64(a)                                                                                 \
  {                                                                                                \
    unsigned char tmp[8];                                                                          \
    unsigned char *ch;                                                                             \
    ch = (unsigned char *)(a);                                                                     \
    tmp[0] = ch[0];                                                                                \
    tmp[1] = ch[1];                                                                                \
    tmp[2] = ch[2];                                                                                \
    tmp[3] = ch[3];                                                                                \
    tmp[4] = ch[4];                                                                                \
    tmp[5] = ch[5];                                                                                \
    tmp[6] = ch[6];                                                                                \
    tmp[7] = ch[7];                                                                                \
    ch[0] = tmp[7];                                                                                \
    ch[1] = tmp[6];                                                                                \
    ch[2] = tmp[5];                                                                                \
    ch[3] = tmp[4];                                                                                \
    ch[4] = tmp[3];                                                                                \
    ch[5] = tmp[2];                                                                                \
    ch[6] = tmp[1];                                                                                \
    ch[7] = tmp[0];                                                                                \
  }
#define SWAP_32(a)                                                                                 \
  {                                                                                                \
    unsigned char tmp[4];                                                                          \
    unsigned char *ch;                                                                             \
    ch = (unsigned char *)(a);                                                                     \
    tmp[0] = ch[0];                                                                                \
    tmp[1] = ch[1];                                                                                \
    tmp[2] = ch[2];                                                                                \
    tmp[3] = ch[3];                                                                                \
    ch[0] = tmp[3];                                                                                \
    ch[1] = tmp[2];                                                                                \
    ch[2] = tmp[1];                                                                                \
    ch[3] = tmp[0];                                                                                \
  }
#define SWAP_16(a)                                                                                 \
  {                                                                                                \
    unsigned char d;                                                                               \
    unsigned char *ch;                                                                             \
    ch = (unsigned char *)(a);                                                                     \
    d = ch[0];                                                                                     \
    ch[0] = ch[1];                                                                                 \
    ch[1] = d;                                                                                     \
  }

namespace cvc {
static inline int big_endian() {
  long one = 1;
  return !(*((char *)(&one)));
}
} // namespace cvc

#endif
