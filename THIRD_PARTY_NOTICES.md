# Third-party notices

libcvc is licensed under the GNU Lesser General Public License, version 2.1
(see [LICENSE](LICENSE)). This file records the code in this repository -- and
the code compiled into its libraries from a dependency -- that comes from, or
is derived from, authors other than The University of Texas at Austin, each
with its notice exactly as the source carries it. It is installed with the
documentation by the libcvc and cvcGL installs and ships in the cvcgl-examples
package.

Compiled into libcvc:

- [stb_image / stb_image_write](#stb_image-230-and-stb_image_write-116)
- [PocketFFT](#pocketfft)
- [XmlRpc++](#xmlrpc)
- [readvtk / writevtk](#readvtk--writevtk-technical-university-of-denmark)
- [The contour library](#the-contour-library) and
  [kazlib's dictionary](#kazlib-dictionary)
- [sdf](#sdf-signed-distance-function) and [mtxlib](#mtxlib)
- [VTK](#vtk-the-visualization-toolkit) (also compiled into libcvcGL)

Compiled into the cvcGL examples: [ABYSSAL](#abyssal).

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

## ABYSSAL

The GPU FFT ocean in `src/cvcGL/examples/OceanFFT.h` and `OceanFFT.cpp` -- its
spectrum, time evolution, butterfly IFFT and assembly -- is a C++/VTK port of
ABYSSAL (github.com/Token-Gremlin/natural-disasters). It is built into the
`lsystem_coast` example (the cvcgl-examples package) and the `cvcgl_ocean_fft`
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
