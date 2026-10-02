# Third-party notices

libcvc is licensed under the GNU Lesser General Public License, version 2.1
(see [LICENSE](LICENSE)). Some of its source is derived from, or bundles, code
under other licenses. Those notices are recorded here and installed with the
documentation.

## XmlRpc++

The bundled XmlRpc++ sources under `inc/xmlrpc/` and `src/xmlrpc/` are
Copyright (c) 2002-2003 Chris Morley and are used under the GNU Lesser General
Public License, version 2.1 or later.

## VTK (The Visualization Toolkit) 9.5.0

Portions of `src/cvcGL/LowMemoryPolyDataMapper.cpp` -- the regions between
`BEGIN VTK 9.5.0-DERIVED` and `END VTK 9.5.0-DERIVED` -- are derived from
VTK 9.5.0 (`Rendering/OpenGL2`: `vtkGLSLModCoincidentTopology.cxx`,
`vtkOpenGLLowMemoryPolyDataMapper.cxx`, `vtkOpenGLLowMemoryCellTypeAgent.cxx`,
`vtkOpenGLLowMemoryVerticesAgent.cxx`, `vtkOpenGLLowMemoryLinesAgent.cxx`,
`vtkOpenGLLowMemoryPolygonsAgent.cxx`). They are used under VTK's license:

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

The same notice is reproduced at the top of that source file.
