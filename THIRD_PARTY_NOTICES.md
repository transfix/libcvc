# Third-party notices

libcvc is licensed under the GNU Lesser General Public License, version 2.1
(see [LICENSE](LICENSE)). This file records the code in this repository -- and
the code compiled into its libraries from a dependency -- that comes from, or
is derived from, authors other than The University of Texas at Austin, each
with its notice exactly as the source carries it. It is installed with the
documentation by the libcvc and cvcGL installs and ships in the libcvc-examples
and libcvc-examples-web packages.

Compiled into libcvc:

- [stb_image / stb_image_write](#stb_image-230-and-stb_image_write-116)
- [nanoflann](#nanoflann-201)
- [PocketFFT](#pocketfft)
- [XmlRpc++](#xmlrpc)
- [readvtk / writevtk](#readvtk--writevtk-technical-university-of-denmark)
- [The contour library](#the-contour-library) and
  [kazlib's dictionary](#kazlib-dictionary)
- [sdf](#sdf-signed-distance-function) and [mtxlib](#mtxlib)
- [VTK](#vtk-the-visualization-toolkit) (also compiled into libcvcGL)
- [Eigen](#eigen-501) and [libigl](#libigl-264-core-module), when built with
  `CVC_ENABLE_LIBIGL`; without it, colormap samples taken from libigl's tables
  (see [libigl](#libigl-264-core-module))
- The permissive geometry libraries that replace CGAL's GPL packages for the VolumeRover
  ports (`docs/roadmap/VOLROVER-LIBRARY-PORTS-ROADMAP.md` §1.2): [CDT](#cdt-201), with
  [Lenthe's predicates](#cdts-predicates-william-c-lenthe) and
  [libc++'s nth_element](#cdts-portable_nth_element-llvm-libc), and
  [Shewchuk's predicates](#shewchuks-robust-predicates), vendored; [Qhull](#qhull-802) and
  [Clipper2](#clipper2-201), linked when built with `CVC_WITH_QHULL` / `CVC_WITH_CLIPPER2`
  (default ON when the packages are found)

Compiled into the cvcGL examples: [ABYSSAL](#abyssal).

Installed with cvcGL (`share/libcvc/fonts`) and embedded in the browser hosts (arigl-web, cyprty):
the fonts the ImGui overlay falls back on, [DejaVu Sans Mono](#dejavu-sans-mono-237) and
[Twemoji Mozilla](#twemoji-mozilla-070).

In the build scripts only: [CMake modules](#cmake-modules-build-scripts-only).

## stb_image 2.30 and stb_image_write 1.16

`src/cvc/image/third_party/stb_image.h` and `stb_image_write.h` (Sean Barrett
and contributors), compiled into libcvc by `src/cvc/image/stb_io.cpp`. Both
files end with this notice:

```
This software is available under 2 licenses -- choose whichever you prefer.
------------------------------------------------------------------------------
ALTERNATIVE A - MIT License
Copyright (c) 2017 Sean Barrett
Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
of the Software, and to permit persons to whom the Software is furnished to do
so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
------------------------------------------------------------------------------
ALTERNATIVE B - Public Domain (www.unlicense.org)
This is free and unencumbered software released into the public domain.
Anyone is free to copy, modify, publish, use, compile, sell, or distribute this
software, either in source code form or as a compiled binary, for any purpose,
commercial or non-commercial, and by any means.
In jurisdictions that recognize copyright laws, the author or authors of this
software dedicate any and all copyright interest in the software to the public
domain. We make this dedication for the benefit of the public at large and to
the detriment of our heirs and successors. We intend this dedication to be an
overt act of relinquishment in perpetuity of all present and future rights to
this software under copyright law.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
```

## nanoflann 2.0.1

`src/cvc/third_party/nanoflann/nanoflann.hpp` (Marius Muja, David G. Lowe, Jose Luis Blanco),
copied unchanged from github.com/jlblancoc/nanoflann at tag `2.0.1`. Being header-only, its code is
compiled into libcvc by `src/cvc/nav/spatial.cpp` and `src/cvc/geometry/project_verts.cpp`. Its
notice (`src/cvc/third_party/nanoflann/COPYING`):

```
Software License Agreement (BSD License)

Copyright 2008-2009  Marius Muja (mariusm@cs.ubc.ca). All rights reserved.
Copyright 2008-2009  David G. Lowe (lowe@cs.ubc.ca). All rights reserved.
Copyright 2011-2026  Jose L. Blanco (joseluisblancoc@gmail.com). All rights reserved.

THE BSD LICENSE

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions
are met:

1. Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
```

## CDT 2.0.1

`src/cvc/third_party/CDT/` (Artem Amirkhanov and contributors): the headers of
github.com/artem-ogre/CDT at tag `2.0.1` (source tarball sha256
`5e3c45715e55b5f2d4390082c8c1ac026f635ddee233516fdcf94b197954367d`), header-only, for 2D
constrained Delaunay triangulation. They are used under the Mozilla Public License 2.0, whose full
text is `src/cvc/third_party/CDT/LICENSE`; the vendored files, modified or not, stay under it, and
that directory is their Source Code Form. Two files are modified for libcvc (2026-11-09, by
libcvc's maintainers, so the vendored code holds no mutable process-wide state; see
`src/cvc/third_party/README.md`): `Triangulation.hpp` (a never-written function-local `static`
made `const`) and `predicates.h` (its error-bound constants initialised from constant
expressions, with the same values). Each MPL-2.0 file carries:

```
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.
```

### CDT's predicates (William C. Lenthe)

`src/cvc/third_party/CDT/predicates.h`, the robust predicates CDT uses
(`src/cvc/third_party/CDT/LICENSE-predicates.txt`):

```
Copyright (c) 2019, William C. Lenthe
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
```

### CDT's portable_nth_element (LLVM libc++)

`src/cvc/third_party/CDT/portable_nth_element.hpp`, copied by CDT from LLVM's libc++, under the
Apache License v2.0 with LLVM Exceptions (full text: `src/cvc/third_party/CDT/LICENSE-LLVM.txt`).
The exception waives the attribution conditions for code embedded in object form, and settles
compatibility with the GPLv2. The file carries:

```
Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
See https://llvm.org/LICENSE.txt for license information.
SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
```

## Shewchuk's robust predicates

`src/cvc/third_party/shewchuk/predicates.hpp`: Jonathan Richard Shewchuk's adaptive exact
orientation and in-circle / in-sphere predicates (predicates.c, May 18, 1996,
www.cs.cmu.edu/~quake/robust.html), turned into a reentrant C++ header by
`src/cvc/third_party/shewchuk/gen_predicates.py` (its changes are listed at the top of the
header). The source carries:

```
Placed in the public domain by
Jonathan Richard Shewchuk
School of Computer Science
Carnegie Mellon University
5000 Forbes Avenue
Pittsburgh, Pennsylvania  15213-3891
jrs@cs.cmu.edu
```

## Qhull 8.0.2

Not vendored: the reentrant library `libqhull_r` comes from the cvcpkg `qhull` package
(github.com/qhull/qhull, tag `v8.0.2`, source tarball sha256
`8774e9a12c70b0180b95d6b0b563c5aa4bea8d5960c15e18ae3b6d2521d64f8b`), unmodified. A shared libcvc
links the shared `libqhull_r`; a static one (wasm) links `libqhullstatic_r`, so its code is
compiled into the programs that link libcvc. The original source code may be obtained from
www.qhull.org. Qhull's `COPYING.txt`:

```
                    Qhull, Copyright (c) 1993-2020
                    
                            C.B. Barber
                           Arlington, MA 
                          
                               and

       The National Science and Technology Research Center for
        Computation and Visualization of Geometric Structures
                        (The Geometry Center)
                       University of Minnesota

                       email: qhull@qhull.org

This software includes Qhull from C.B. Barber and The Geometry Center.  
Files derived from Qhull 1.0 are copyrighted by the Geometry Center.  The
remaining files are copyrighted by C.B. Barber.  Qhull is free software 
and may be obtained via http from www.qhull.org.  It may be freely copied, 
modified, and redistributed under the following conditions:

1. All copyright notices must remain intact in all files.

2. A copy of this text file must be distributed along with any copies 
   of Qhull that you redistribute; this includes copies that you have 
   modified, or copies of programs or other software products that 
   include Qhull.

3. If you modify Qhull, you must include a notice giving the
   name of the person performing the modification, the date of
   modification, and the reason for such modification.

4. When distributing modified versions of Qhull, or other software 
   products that include Qhull, you must provide notice that the original 
   source code may be obtained as noted above.

5. There is no warranty or other guarantee of fitness for Qhull, it is 
   provided solely "as is".  Bug reports or fixes may be sent to 
   qhull_bug@qhull.org; the authors may or may not act on them as 
   they desire.
```

## Clipper2 2.0.1

Not vendored: the static `Clipper2` library comes from the cvcpkg `clipper2` package
(github.com/AngusJohnson/Clipper2, tag `Clipper2_2.0.1`, source tarball sha256
`2a3693aceab4aed3e39b743e038d87701acc53cf05ed7b2013aab3e0aec5287e`, Angus Johnson), so its code
is compiled into libcvc. The package patches one line (`Triangulate` initialises an error code it
ORs into). Clipper2's `LICENSE`:

```
Boost Software License - Version 1.0 - August 17th, 2003

Permission is hereby granted, free of charge, to any person or organization
obtaining a copy of the software and accompanying documentation covered by
this license (the "Software") to use, reproduce, display, distribute,
execute, and transmit the Software, and to prepare derivative works of the
Software, and to permit third-parties to whom the Software is furnished to
do so, all subject to the following:

The copyright notices in the Software and this entire statement, including
the above license grant, this restriction and the following disclaimer,
must be included in all copies of the Software, in whole or in part, and
all derivative works of the Software, unless such copies or derivative
works are solely in the form of machine-executable object code generated by
a source language processor.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT
SHALL THE COPYRIGHT HOLDERS OR ANYONE DISTRIBUTING THE SOFTWARE BE LIABLE
FOR ANY DAMAGES OR OTHER LIABILITY, WHETHER IN CONTRACT, TORT OR OTHERWISE,
ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
DEALINGS IN THE SOFTWARE.
```

## PocketFFT

Not vendored: `pocketfft_hdronly.h` comes from the cvcpkg `pocketfft` package
(github.com/mreineck/pocketfft, `cpp` branch, commit
`c90e55b3d529f8efa40ed01a20de22405f45fc65`). Being header-only, its code is
compiled into libcvc (`src/cvc/volume/volume_ops.cpp`) whenever
`CVC_FFT_PROVIDER=pocketfft`, the default. The header's notice:

```
This file is part of pocketfft.

Copyright (C) 2010-2024 Max-Planck-Society
Copyright (C) 2019-2020 Peter Bell

For the odd-sized DCT-IV transforms:
  Copyright (C) 2003, 2007-14 Matteo Frigo
  Copyright (C) 2003, 2007-14 Massachusetts Institute of Technology

For the prev_good_size search:
  Copyright (C) 2024 Tan Ping Liang, Peter Bell

For the safeguards against integer overflow in good_size search:
  Copyright (C) 2024 Cris Luengo

Authors: Martin Reinecke, Peter Bell

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice, this
  list of conditions and the following disclaimer in the documentation and/or
  other materials provided with the distribution.
* Neither the name of the copyright holder nor the names of its contributors may
  be used to endorse or promote products derived from this software without
  specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
```

## XmlRpc++

The bundled XmlRpc++ sources under `inc/xmlrpc/` and `src/xmlrpc/`, built only
with `CVC_USING_XMLRPC=ON` (default OFF), are used under the GNU Lesser General
Public License, version 2.1 or later. The headers in `inc/xmlrpc/` carry this
notice (the `.cpp` files carry none of their own):

```
// XmlRpc++ Copyright (c) 2002-2003 by Chris Morley
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.
//
// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307
```

Its `inc/xmlrpc/base64.h` carries only:

```
//  base64.hpp
//  Autor Konstantin Pilipchuk
//  mailto:lostd@ukr.net
```

## readvtk / writevtk (Technical University of Denmark)

The legacy `.vtk` reader and writer in `src/cvc/volume/vtk_io.cpp`, compiled
into libcvc, carry this notice and no license terms of their own:

```
/* readvtk/writevtk:
 * Copyright (c) The Technical University of Denmark
 * Author: Ojaswa Sharma
 * E-mail: os@imm.dtu.dk
 * File: vtkIO.h
 * .vtk I/O
 */
```

## The contour library

`src/cvc/geometry/cvc-mesher/contour/` (with copies of its `cubes.h` in
`cvc-mesher/LBIE/` and `SDF/SignDistanceFunction_v2/`), compiled into libcvc
with `CVC_ENABLE_MESHER` (default ON). Its files carry these lines and no
license terms of their own; files without a header belong to the same library:

```
Copyright (c) 1997 Dan Schikore
Copyright (c) 1997 Dan Schikore - modified by Emilio Camahort, 1999
Copyright (c) 1997 Dan Schikore - updated by Emilio Camahort, 1998
Copyright (c) 1997 Dan Schikore - updated by Emilio Camahort, 1999
Copyright (c) 1998 Emilio Camahort
Copyright (c) 1998 Emilio Camahort, Dan Schikore
Copyright (c) 1998 Emilio Camahort - ecamahor@cs.utexas.edu
Copyright (c) 1999 Emilio Camahort
```

and, in the priority-queue headers (`ipqueue.h` and its siblings):

```
 * Author:      Dan Schikore
 *
 * Adapted from pqueue.h
 * Ported to C++ by Raymund Merkert - June 1995
 * Changes by Fausto Bernardini - Sept 1995
```

## kazlib dictionary

`src/cvc/geometry/cvc-mesher/contour/dict.c` and `dict.h`, compiled with the
contour library:

```
/*
 * Dictionary Abstract Data Type
 * Copyright (C) 1997 Kaz Kylheku <kaz@ashi.footprints.net>
 *
 * Free Software License:
 *
 * All rights are reserved by the author, with the following exceptions:
 * Permission is granted to freely reproduce and distribute this software,
 * possibly in exchange for a fee, provided that this copyright notice appears
 * intact. Permission is also granted to adapt this software to produce
 * derivative works, as long as the modified versions carry this copyright
 * notice and additional notices stating that the work has been modified.
 * This source code may be translated into executable form and incorporated
 * into proprietary software; there is no requirement for such software to
 * contain a copyright notice related to this source.
```

## sdf (signed distance function)

`src/cvc/geometry/SDF/SignDistanceFunction_v2/`, compiled into libcvc with
`CVC_ENABLE_SDF` (default ON), is used under the GNU Lesser General Public
License, version 2.1. Its files that carry a header carry:

```
  Copyright (c): Xiaoyu Zhang (xiaoyu@csusm.edu)

  This file is part of sdf (signed distance function).

  sdf is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.

  sdf is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
```

and, in `diskio.h` and `geom.h`, `(c) 2000 Xiaoyu Zhang`.

## mtxlib

`src/cvc/geometry/SDF/SignDistanceFunction_v2/mtxlib.h` and `mtxlib.cpp` (C++
Matrix Library 2.6), compiled with sdf. Portions Copyright (C) Dante Treglia II
and Mark A. DeLoura, 2000. The notice, as `mtxlib.h` carries it (`mtxlib.cpp`
has the same text):

```
/* Copyright (C) Dante Treglia II and Mark A. DeLoura, 2000.
 * All rights reserved worldwide.
 *
 * This software is provided "as is" without express or implied
 * warranties. You may freely copy and compile this source into
 * applications you distribute provided that the copyright text
 * below is included in the resulting source code, for example:
 * "Portions Copyright (C) Dante Treglia II and Mark A. DeLoura, 2000"
 */
```

## VTK (The Visualization Toolkit)

- Portions of `src/cvcGL/LowMemoryPolyDataMapper.cpp` (libcvcGL) -- the
  regions between `BEGIN VTK 9.5.0-DERIVED` and `END VTK 9.5.0-DERIVED` -- are
  derived from VTK 9.5.0 (`Rendering/OpenGL2`: `vtkGLSLModCoincidentTopology.cxx`,
  `vtkOpenGLLowMemoryPolyDataMapper.cxx`, `vtkOpenGLLowMemoryCellTypeAgent.cxx`,
  `vtkOpenGLLowMemoryVerticesAgent.cxx`, `vtkOpenGLLowMemoryLinesAgent.cxx`,
  `vtkOpenGLLowMemoryPolygonsAgent.cxx`). The same notice is reproduced at the
  top of that source file.
- The marching-cubes triangulation table in `inc/cvc/volren/detail/mc_tables.h`
  (libcvc's volume renderer) is VTK's `triCases` from `vtkMarchingCubesCases.h`,
  by way of volrover's libiso.

Both are used under VTK's license:

```
SPDX-FileCopyrightText: Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
SPDX-License-Identifier: BSD-3-Clause

Copyright (c) 1993-2015 Ken Martin, Will Schroeder, Bill Lorensen
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

 * Neither name of Ken Martin, Will Schroeder, or Bill Lorensen nor the names
   of any contributors may be used to endorse or promote products derived
   from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS ``AS IS''
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHORS OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
```

## Eigen 5.0.1

Not vendored: the headers come from the cvcpkg `eigen` package (gitlab.com/libeigen/eigen, tag
`5.0.1`, source tarball sha256
`e9c326dc8c05cd1e044c71f30f1b2e34a6161a3b6ecf445d56b53ff1669e3dec`). Being header-only, its
code is compiled into libcvc by the libigl-backed geometry sources (`src/cvc/geometry/igl/`)
whenever libcvc is built with `CVC_ENABLE_LIBIGL` (default ON when the package is found). The
Source Code Form is the package's `include/eigen3` tree or that tarball, and the package ships
Eigen's full license texts (`COPYING.MPL2`, `COPYING.APACHE`, `COPYING.BSD`, ...) in
`share/licenses/eigen/`. Eigen's `COPYING.README`:

```
Eigen is primarily MPL2 licensed. See COPYING.MPL2 and these links:
  http://www.mozilla.org/MPL/2.0/
  http://www.mozilla.org/MPL/2.0/FAQ.html

Some files contain third-party code under BSD or other MPL2-compatible licenses,
whence the other COPYING.* files here.
```

Its MPL-2.0 files carry, after their own copyright lines:

```
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
```

The files under other terms that libcvc's build reaches are listed below. The list comes from
the include closure of the four `src/cvc/geometry/igl/*.cpp` translation units, computed over
the Eigen 5.0.1 and libigl 2.6.4 sources; it follows every `#include` regardless of `#if`, so it
can only overcount. Through `<Eigen/Sparse>` and libigl's `min_quad_with_fixed.h` that closure
includes Eigen's sparse solvers (SimplicialCholesky, SparseLU, SparseQR, the AMD and COLAMD
orderings) and `unsupported/Eigen/SparseExtra`. A file is listed even where libcvc may not emit
its code, and a condition is noted where one applies.

`Eigen/src/Core/arch/Default/BFloat16.h` is included unconditionally by `Eigen/Core` and is
under the Apache License, Version 2.0 (full text: Eigen's `COPYING.APACHE`). Whether compiling
it into LGPL-2.1-only libcvc is acceptable is an open owner decision; see
`docs/roadmap/LIBIGL-GEOMETRY-FE-ROADMAP.md` §6. Its notice:

```
/* Copyright 2017 The TensorFlow Authors. All Rights Reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
==============================================================================*/
```

`Eigen/src/Core/arch/Default/Half.h`, also included unconditionally, carries the MPL-2.0 notice
above followed by:

```
// The conversion routines are Copyright (c) Fabian Giesen, 2016.
// The original license follows:
//
// Copyright (c) Fabian Giesen, 2016
// All rights reserved.
// Redistribution and use in source and binary forms, with or without
// modification, are permitted.
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
```

`Eigen/src/Geometry/AlignedBox.h` (reached through libigl's `AABB.h`) carries the MPL-2.0 notice
followed by, for its `AlignedBox::transform()`:

```
// Function void Eigen::AlignedBox::transform(const Transform& transform)
// is provided under the following license agreement:
//
// Software License Agreement (BSD License)
//
// Copyright (c) 2011-2014, Willow Garage, Inc.
// Copyright (c) 2014-2015, Open Source Robotics Foundation
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of Open Source Robotics Foundation nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
```

`Eigen/src/OrderingMethods/Eigen_Colamd.h` (the COLAMD column ordering behind
`COLAMDOrdering`). libigl's `min_quad_with_fixed` keeps a `SparseLU` and a `SparseQR` with that
ordering, and libcvc's sparse solves (`cvc::fem`, `cvc::smooth`'s implicit step, the geodesic
solver) instantiate it. Unlike `Amd.h` (below), the file carries no MPL relicensing
grant. After its MPL-2.0 header it reads:

```
// This file is modified from the colamd/symamd library. The copyright is below

//   The authors of the code itself are Stefan I. Larimore and Timothy A.
//   Davis (davis@cise.ufl.edu), University of Florida.  The algorithm was
//   developed in collaboration with John Gilbert, Xerox PARC, and Esmond
//   Ng, Oak Ridge National Laboratory.
//
//     Date:
//
//   September 8, 2003.  Version 2.3.
//
//     Acknowledgements:
//
//   This work was supported by the National Science Foundation, under
//   grants DMS-9504974 and DMS-9803599.
//
//     Notice:
//
//   Copyright (c) 1998-2003 by the University of Florida.
//   All Rights Reserved.
//
//   THIS MATERIAL IS PROVIDED AS IS, WITH ABSOLUTELY NO WARRANTY
//   EXPRESSED OR IMPLIED.  ANY USE IS AT YOUR OWN RISK.
//
//   Permission is hereby granted to use, copy, modify, and/or distribute
//   this program, provided that the Copyright, this License, and the
//   Availability of the original version is retained on all copies and made
//   accessible to the end-user of any code or package that includes COLAMD
//   or any modified version of COLAMD.
//
//     Availability:
//
//   The colamd/symamd library is available at
//
//       http://www.suitesparse.com
```

`Eigen/src/SparseCore/SparseColEtree.h` and the SuperLU-derived kernels of Eigen's `SparseLU`
(`Eigen/src/SparseLU/SparseLU_{Memory,column_bmod,column_dfs,copy_to_ucol,heap_relax_snode,panel_bmod,panel_dfs,pivotL,pruneL,relax_snode}.h`),
reached the same way (`SparseQR` uses the column elimination tree as well). Each file says it is
a modified version of the SuperLU file it names ("NOTE: This file is the modified version of
... in SuperLU", or "This file is a modified version of ..."), which is the modification notice
the terms ask for; `SparseLU_Structs.h` comes from SuperLU's `slu_[s,d,c,z]defs.h` too, without a
notice of its own. The notice, as `SparseColEtree.h` carries it after its MPL-2.0 header (the
other files differ only in the SuperLU file named, its version and its date):

```
 * NOTE: This file is the modified version of sp_coletree.c file in SuperLU

 * -- SuperLU routine (version 3.1) --
 * Univ. of California Berkeley, Xerox Palo Alto Research Center,
 * and Lawrence Berkeley National Lab.
 * August 1, 2008
 *
 * Copyright (c) 1994 by Xerox Corporation.  All rights reserved.
 *
 * THIS MATERIAL IS PROVIDED AS IS, WITH ABSOLUTELY NO WARRANTY
 * EXPRESSED OR IMPLIED.  ANY USE IS AT YOUR OWN RISK.
 *
 * Permission is hereby granted to use or copy this program for any
 * purpose, provided the above notices are retained on all copies.
 * Permission to modify the code and to distribute modified code is
 * granted, provided the above notices are retained, and a notice that
 * the code was modified is included with the above copyright notice.
```

`Eigen/src/LU/arch/InverseSize4.h` is included by `Eigen/LU` (and so by `Eigen/Dense`) on SSE and
NEON targets, which covers x86-64 and arm64. Its code is emitted only for fixed-size 4x4
inverses. After its MPL-2.0 header it reads:

```
// The SSE code for the 4x4 float and double matrix inverse in former (deprecated) \src\LU\Inverse_SSE.h
// comes from the following Intel's library:
// http://software.intel.com/en-us/articles/optimized-matrix-library-for-use-with-the-intel-pentiumr-4-processors-sse2-instructions/
//
// Here is the respective copyright and license statement:
//
//   Copyright (c) 2001 Intel Corporation.
//
// Permission is granted to use, copy, distribute and prepare derivative works
// of this library for any purpose and without fee, provided, that the above
// copyright notice and this statement appear in all copies.
// Intel makes no representations about the suitability of this software for
// any purpose, and specifically disclaims all warranties.
// See LEGAL.TXT for all the legal information.
```

The Intel BLAS/LAPACKE/MKL backend headers are BSD-3-Clause. `Eigen/src/Core/util/MKL_support.h`
is included by `Eigen/Core` (macro definitions only unless `EIGEN_USE_MKL*` is defined); the
backends themselves (`Core/products/*_BLAS.h`, `*_LAPACKE.h`, `Core/Assign_MKL.h`,
`misc/lapacke.h`) compile only with `EIGEN_USE_BLAS`, `EIGEN_USE_LAPACKE` or `EIGEN_USE_MKL*`,
which libcvc does not define. The notice, as `MKL_support.h` carries it (`Assign_MKL.h` adds
`Copyright (C) 2015 Gael Guennebaud`, `BDCSVD_LAPACKE.h` adds
`Copyright (C) 2022 Melven Roehrig-Zoellner`, and `misc/lapacke.h` reads
`Copyright (c) 2010, Intel Corp.`):

```
 Copyright (c) 2011, Intel Corporation. All rights reserved.

 Redistribution and use in source and binary forms, with or without modification,
 are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.
 * Neither the name of Intel Corporation nor the names of its contributors may
   be used to endorse or promote products derived from this software without
   specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
```

Other files in the closure name an outside origin but are under the MPL-2.0 alone, as each one
states: `Eigen/src/OrderingMethods/Amd.h` (from CSparse) and
`Eigen/src/SparseCholesky/SimplicialCholesky_impl.h` (from LDL), whose author, Timothy A. Davis,
executed a license with Google LLC permitting their distribution as part of Eigen under the
MPL-2.0; `IterativeLinearSolvers/IncompleteLUT.h` (SPARSKIT's ILUT, relicensed to MPL-2.0 with
Yousef Saad's permission); `Eigenvalues/EigenSolver.h` and `RealSchur.h` (algorithms from the
public-domain JAMA); and `unsupported/Eigen/SparseExtra` (`MarketIO.h`,
`MatrixMarketIterator.h`, `RandomSetter.h`, `SparseInverse.h`), which libigl's
`min_quad_with_fixed.h` includes. Some MPL-2.0 files carry copyright lines of other contributors
(Intel, NVIDIA, Arm, Julien Pommier's SSE math functions) and no other terms. No MINPACK-derived
file (`unsupported/Eigen/NonLinearOptimization`, `LevenbergMarquardt`) is reached.

## libigl 2.6.4 (core module)

Not vendored: the headers come from the cvcpkg `libigl` package (github.com/libigl/libigl, tag
`v2.6.4`, commit `7100764c2a2833284a8bdfa9b948d2b3bf124624`, source tarball sha256
`09fa1b9b44e0ecbd8fbaa17ec8d9ac0cb4efcedcb2427970f8a4e90bc96e0970`). Only the core module -- the
top-level files of `include/igl` -- is packaged and used; none of libigl's copyleft
(`include/igl/copyleft/`) or restricted (`triangle/`, `matlab/`, `mosek/`) modules is built,
packaged or compiled into libcvc. Being header-only, its code is compiled into libcvc by
`src/cvc/geometry/igl/` whenever libcvc is built with `CVC_ENABLE_LIBIGL`. The Source Code Form
is the package's `include/igl` tree or that tarball; the package ships `LICENSE.MPL2` in
`share/licenses/libigl/`. The core files carry, after their own copyright lines:

```
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.
```

Third-party code inside the core files libcvc compiles (the include closure of
`src/cvc/geometry/igl/*.cpp`, as for Eigen above):

`include/igl/FastWindingNumberForSoups.h` (behind `igl::fast_winding_number`, used for the sign
of signed distances and for winding numbers):

```
// MIT License

// Copyright (c) 2018 Side Effects Software Inc.

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
```

`include/igl/raytri.c` (ray-triangle tests), in the public domain. `AABB.h` includes it
through `ray_mesh_intersect.cpp`; its functions are `inline`, so they are emitted only where a
ray query uses them:

```
/* Ray-Triangle Intersection Test Routines          */
/* Different optimizations of my and Ben Trumbore's */
/* code from journals of graphics tools (JGT)       */
/* http://www.acm.org/jgt/                          */
/* by Tomas Moller, May 2000                        */


// Alec: this file is listed as "Public Domain"
// http://fileadmin.cs.lth.se/cs/Personal/Tomas_Akenine-Moller/code/
```

The colormap tables in `include/igl/colormap.cpp` (behind `cvc::colormap_rgb`). The file's
own note covers the matplotlib maps (viridis, magma, plasma, inferno):

```
// One of the new matplotlib colormaps by Nathaniel J.Smith, Stefan van der Walt, and (in the case of viridis) Eric Firing.
// Released under the CC0 license / public domain dedication
```

The same file also holds a `turbo` table, which it links to Google's 2019 Turbo colormap
(`https://ai.googleblog.com/2019/08/turbo-improved-rainbow-colormap-for.html`) and also uses for
`jet`, and a `parula` table. libigl carries no attribution for either beyond the file's MPL-2.0
header. Their provenance is an open item in the roadmap's licensing section.

These tables also reach builds **without** `CVC_ENABLE_LIBIGL`. There
`src/cvc/geometry/mesh_ops.cpp` serves the colormaps from 17 evenly spaced 8-bit samples of six
of libigl's tables (viridis, magma, plasma, inferno, turbo, parula), so the attributions above,
and the turbo/parula provenance question, apply to every libcvc build. `JET` is not among
them: libcvc's `JET` is its own analytic MATLAB-style formula in every build, and never libigl's
`jet` (which renders turbo).

Packaged in the libigl bundle but **not** compiled into libcvc today: the 3x3 SVD kernels
`include/igl/Singular_Value_Decomposition_{Preamble,Kernel_Declarations,Main_Kernel_Body,Givens_QR_Factorization_Kernel,Jacobi_Conjugation_Kernel}.hpp`.
Only `svd3x3`, `polar_svd3x3` and `fit_rotations` include them, and no libcvc source reaches
those. The notice is kept here because a later phase (ARAP deformation, roadmap P4.3) would:

```
//#####################################################################
// Copyright (c) 2010-2011, Eftychios Sifakis.
//
// Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or
//     other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING,
// BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
// SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//#####################################################################
```

Also packaged but not reached: `exact_geodesic.cpp` and `tri_tri_intersect.h` (MIT),
`tinyply.h` (public domain) and `marching_cubes.cpp` (adapted from public-domain code). Their
notices belong here once libcvc compiles them.

## ABYSSAL

The GPU FFT ocean in `src/cvcGL/ocean/OceanFFT.h` and `OceanFFT.cpp` -- its
spectrum, time evolution, butterfly IFFT and assembly -- is a C++/VTK port of
ABYSSAL (github.com/Token-Gremlin/natural-disasters). It is built into the
`lsystem_coast` example (the libcvc-examples packages) and the `cvcgl_ocean_fft`
test. Upstream's `LICENSE`:

```
MIT License

Copyright (c) 2026 Davi (Token-Gremlin)

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```

## DejaVu Sans Mono 2.37

`src/cvcGL/fonts/DejaVuSansMono.ttf`, unchanged from DejaVu 2.37
(github.com/dejavu-fonts/dejavu-fonts). Upstream's `LICENSE`:

```
Fonts are (c) Bitstream (see below). DejaVu changes are in public domain.
Glyphs imported from Arev fonts are (c) Tavmjong Bah (see below)


Bitstream Vera Fonts Copyright
------------------------------

Copyright (c) 2003 by Bitstream, Inc. All Rights Reserved. Bitstream Vera is
a trademark of Bitstream, Inc.

Permission is hereby granted, free of charge, to any person obtaining a copy
of the fonts accompanying this license ("Fonts") and associated
documentation files (the "Font Software"), to reproduce and distribute the
Font Software, including without limitation the rights to use, copy, merge,
publish, distribute, and/or sell copies of the Font Software, and to permit
persons to whom the Font Software is furnished to do so, subject to the
following conditions:

The above copyright and trademark notices and this permission notice shall
be included in all copies of one or more of the Font Software typefaces.

The Font Software may be modified, altered, or added to, and in particular
the designs of glyphs or characters in the Fonts may be modified and
additional glyphs or characters may be added to the Fonts, only if the fonts
are renamed to names not containing either the words "Bitstream" or the word
"Vera".

This License becomes null and void to the extent applicable to Fonts or Font
Software that has been modified and is distributed under the "Bitstream
Vera" names.

The Font Software may be sold as part of a larger software package but no
copy of one or more of the Font Software typefaces may be sold by itself.

THE FONT SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
OR IMPLIED, INCLUDING BUT NOT LIMITED TO ANY WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT OF COPYRIGHT, PATENT,
TRADEMARK, OR OTHER RIGHT. IN NO EVENT SHALL BITSTREAM OR THE GNOME
FOUNDATION BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, INCLUDING
ANY GENERAL, SPECIAL, INDIRECT, INCIDENTAL, OR CONSEQUENTIAL DAMAGES,
WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
THE USE OR INABILITY TO USE THE FONT SOFTWARE OR FROM OTHER DEALINGS IN THE
FONT SOFTWARE.

Except as contained in this notice, the names of Gnome, the Gnome
Foundation, and Bitstream Inc., shall not be used in advertising or
otherwise to promote the sale, use or other dealings in this Font Software
without prior written authorization from the Gnome Foundation or Bitstream
Inc., respectively. For further information, contact: fonts at gnome dot
org.

Arev Fonts Copyright
------------------------------

Copyright (c) 2006 by Tavmjong Bah. All Rights Reserved.

Permission is hereby granted, free of charge, to any person obtaining
a copy of the fonts accompanying this license ("Fonts") and
associated documentation files (the "Font Software"), to reproduce
and distribute the modifications to the Bitstream Vera Font Software,
including without limitation the rights to use, copy, merge, publish,
distribute, and/or sell copies of the Font Software, and to permit
persons to whom the Font Software is furnished to do so, subject to
the following conditions:

The above copyright and trademark notices and this permission notice
shall be included in all copies of one or more of the Font Software
typefaces.

The Font Software may be modified, altered, or added to, and in
particular the designs of glyphs or characters in the Fonts may be
modified and additional glyphs or characters may be added to the
Fonts, only if the fonts are renamed to names not containing either
the words "Tavmjong Bah" or the word "Arev".

This License becomes null and void to the extent applicable to Fonts
or Font Software that has been modified and is distributed under the 
"Tavmjong Bah Arev" names.

The Font Software may be sold as part of a larger software package but
no copy of one or more of the Font Software typefaces may be sold by
itself.

THE FONT SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO ANY WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT
OF COPYRIGHT, PATENT, TRADEMARK, OR OTHER RIGHT. IN NO EVENT SHALL
TAVMJONG BAH BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
INCLUDING ANY GENERAL, SPECIAL, INDIRECT, INCIDENTAL, OR CONSEQUENTIAL
DAMAGES, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
FROM, OUT OF THE USE OR INABILITY TO USE THE FONT SOFTWARE OR FROM
OTHER DEALINGS IN THE FONT SOFTWARE.

Except as contained in this notice, the name of Tavmjong Bah shall not
be used in advertising or otherwise to promote the sale, use or other
dealings in this Font Software without prior written authorization
from Tavmjong Bah. For further information, contact: tavmjong @ free
. fr.

TeX Gyre DJV Math
-----------------
Fonts are (c) Bitstream (see below). DejaVu changes are in public domain.

Math extensions done by B. Jackowski, P. Strzelczyk and P. Pianowski
(on behalf of TeX users groups) are in public domain.

Letters imported from Euler Fraktur from AMSfonts are (c) American
Mathematical Society (see below).
Bitstream Vera Fonts Copyright
Copyright (c) 2003 by Bitstream, Inc. All Rights Reserved. Bitstream Vera
is a trademark of Bitstream, Inc.

Permission is hereby granted, free of charge, to any person obtaining a copy
of the fonts accompanying this license (“Fonts”) and associated
documentation
files (the “Font Software”), to reproduce and distribute the Font Software,
including without limitation the rights to use, copy, merge, publish,
distribute,
and/or sell copies of the Font Software, and to permit persons  to whom
the Font Software is furnished to do so, subject to the following
conditions:

The above copyright and trademark notices and this permission notice
shall be
included in all copies of one or more of the Font Software typefaces.

The Font Software may be modified, altered, or added to, and in particular
the designs of glyphs or characters in the Fonts may be modified and
additional
glyphs or characters may be added to the Fonts, only if the fonts are
renamed
to names not containing either the words “Bitstream” or the word “Vera”.

This License becomes null and void to the extent applicable to Fonts or
Font Software
that has been modified and is distributed under the “Bitstream Vera”
names.

The Font Software may be sold as part of a larger software package but
no copy
of one or more of the Font Software typefaces may be sold by itself.

THE FONT SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS
OR IMPLIED, INCLUDING BUT NOT LIMITED TO ANY WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT OF COPYRIGHT, PATENT,
TRADEMARK, OR OTHER RIGHT. IN NO EVENT SHALL BITSTREAM OR THE GNOME
FOUNDATION
BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, INCLUDING ANY GENERAL,
SPECIAL, INDIRECT, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, WHETHER IN AN
ACTION
OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF THE USE OR
INABILITY TO USE
THE FONT SOFTWARE OR FROM OTHER DEALINGS IN THE FONT SOFTWARE.
Except as contained in this notice, the names of GNOME, the GNOME
Foundation,
and Bitstream Inc., shall not be used in advertising or otherwise to promote
the sale, use or other dealings in this Font Software without prior written
authorization from the GNOME Foundation or Bitstream Inc., respectively.
For further information, contact: fonts at gnome dot org.

AMSFonts (v. 2.2) copyright

The PostScript Type 1 implementation of the AMSFonts produced by and
previously distributed by Blue Sky Research and Y&Y, Inc. are now freely
available for general use. This has been accomplished through the
cooperation
of a consortium of scientific publishers with Blue Sky Research and Y&Y.
Members of this consortium include:

Elsevier Science IBM Corporation Society for Industrial and Applied
Mathematics (SIAM) Springer-Verlag American Mathematical Society (AMS)

In order to assure the authenticity of these fonts, copyright will be
held by
the American Mathematical Society. This is not meant to restrict in any way
the legitimate use of the fonts, such as (but not limited to) electronic
distribution of documents containing these fonts, inclusion of these fonts
into other public domain or commercial font collections or computer
applications, use of the outline data to create derivative fonts and/or
faces, etc. However, the AMS does require that the AMS copyright notice be
removed from any derivative versions of the fonts which have been altered in
any way. In addition, to ensure the fidelity of TeX documents using Computer
Modern fonts, Professor Donald Knuth, creator of the Computer Modern faces,
has requested that any alterations which yield different font metrics be
given a different name.

$Id$
```

## Twemoji Mozilla 0.7.0

`src/cvcGL/fonts/Twemoji.Mozilla.ttf`, unchanged from Mozilla's twemoji-colr v0.7.0
(github.com/mozilla/twemoji-colr): the Twemoji emoji art (github.com/twitter/twemoji), Copyright
Twitter, Inc. and other contributors, licensed under CC-BY 4.0, as a COLR/CPAL font. Upstream's
`LICENSE.md`:

```
## License for the Code

Copyright 2016-2018, Mozilla Foundation

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.



## License for the Visual Design

The Emoji art in the twe-svg.zip archive comes from [Twemoji](https://twitter.github.io/twemoji),
and is used and redistributed under the CC-BY-4.0 [license terms](https://github.com/twitter/twemoji#license)
offered by the Twemoji project.

### Creative Commons Attribution 4.0 International (CC BY 4.0)
https://creativecommons.org/licenses/by/4.0/legalcode
or for the human readable summary: https://creativecommons.org/licenses/by/4.0/


#### You are free to:
**Share** — copy and redistribute the material in any medium or format

**Adapt** — remix, transform, and build upon the material for any purpose, even commercially.

The licensor cannot revoke these freedoms as long as you follow the license terms.


#### Under the following terms:    
**Attribution** — You must give appropriate credit, provide a link to the license,
and indicate if changes were made.
You may do so in any reasonable manner, but not in any way that suggests the licensor endorses you or your use.

**No additional restrictions** — You may not apply legal terms or **technological measures**
that legally restrict others from doing anything the license permits.

#### Notices:
You do not have to comply with the license for elements of the material in the public domain
or where your use is permitted by an applicable exception or limitation. No warranties are given.
The license may not give you all of the permissions necessary for your intended use.
For example, other rights such as publicity, privacy, or moral rights may limit how you use the material.
```

## CMake modules (build scripts only)

These run at configure time; nothing from them is compiled or installed.

`CMake/BundleUtilities.cmake`:

```
#=============================================================================
# CMake - Cross Platform Makefile Generator
# Copyright 2000-2009 Kitware, Inc., Insight Software Consortium
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# * Redistributions of source code must retain the above copyright
#   notice, this list of conditions and the following disclaimer.
#
# * Redistributions in binary form must reproduce the above copyright
#   notice, this list of conditions and the following disclaimer in the
#   documentation and/or other materials provided with the distribution.
#
# * Neither the names of Kitware, Inc., the Insight Software Consortium,
#   nor the names of their contributors may be used to endorse or promote
#   products derived from this software without specific prior written
#   permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#=============================================================================
```

`CMake/GetPrerequisites.cmake` (the `Copyright.txt` it refers to is not in this
repository):

```
#=============================================================================
# Copyright 2008-2009 Kitware, Inc.
#
# Distributed under the OSI-approved BSD License (the "License");
# see accompanying file Copyright.txt for details.
#
# This software is distributed WITHOUT ANY WARRANTY; without even the
# implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See the License for more information.
#=============================================================================
# (To distribute this file outside of CMake, substitute the full
#  License text for the above reference.)
```

`CMake/FindPETSc.cmake` (the `COPYING-CMAKE-SCRIPTS` it refers to is not in this
repository):

```
# Redistribution and use is allowed according to the terms of the BSD license.
# For details see the accompanying COPYING-CMAKE-SCRIPTS file.
```

`CMake/cuda/FindCUDA.cmake` and `CMake/cuda/FindCUDA/{make2cmake,parse_cubin,run_nvcc}.cmake`
(`run_nvcc.cmake` names NVIDIA only):

```
#  James Bigler, NVIDIA Corp (nvidia.com - jbigler)
#  Abe Stephens, SCI Institute -- http://www.sci.utah.edu/~abe/FindCuda.html
#
#  Copyright (c) 2008 - 2009 NVIDIA Corporation.  All rights reserved.
#
#  Copyright (c) 2007-2009
#  Scientific Computing and Imaging Institute, University of Utah
#
#  This code is licensed under the MIT License.  See the FindCUDA.cmake script
#  for the text of the license.

# The MIT License
#
# License for the specific language governing rights and limitations under
# Permission is hereby granted, free of charge, to any person obtaining a
# copy of this software and associated documentation files (the "Software"),
# to deal in the Software without restriction, including without limitation
# the rights to use, copy, modify, merge, publish, distribute, sublicense,
# and/or sell copies of the Software, and to permit persons to whom the
# Software is furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included
# in all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
# OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
# DEALINGS IN THE SOFTWARE.
```
