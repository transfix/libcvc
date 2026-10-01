/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// Test-only GL traffic counter for the cvcgl_streaming_* tests: swaps VTK's
// glad function pointers (glad_gl*) for counting wrappers, so a test can assert
// what actually reached the driver -- how many buffer/texture uploads and
// bytes, buffers/textures created, draws -- independently of the streaming
// node's own bookkeeping. Native only (the wasm spike counted in JS).
#ifndef CVCGL_TEST_GL_UPLOAD_COUNTER_H
#define CVCGL_TEST_GL_UPLOAD_COUNTER_H

#include <string>

namespace cvcgl_test {

enum GLKey {
  K_BUFDATA,   // glBufferData calls
  K_BUFDATA_B, //   bytes (a null-data allocation counts 0)
  K_BUFSUB,    // glBufferSubData calls
  K_BUFSUB_B,  //   bytes
  K_TEXIMG,    // glTexImage2D calls
  K_TEXIMG_B,  //   bytes (w * h * bytes per texel, null data counts 0)
  K_TEXSUB,    // glTexSubImage2D calls
  K_TEXSUB_B,  //   bytes
  K_COPYBUF,   // glCopyBufferSubData calls
  K_COPYBUF_B, //   bytes
  K_GENBUF,    // buffers created
  K_GENTEX,    // textures created
  K_DRAWS,     // draw calls (arrays / elements / range / instanced)
  K_N
};

struct GLCount {
  double v[K_N] = {};
  GLCount operator-(const GLCount &o) const {
    GLCount r;
    for (int i = 0; i < K_N; ++i)
      r.v[i] = v[i] - o.v[i];
    return r;
  }
  double uploadCalls() const { return v[K_BUFDATA] + v[K_BUFSUB] + v[K_TEXIMG] + v[K_TEXSUB]; }
  double uploadBytes() const {
    return v[K_BUFDATA_B] + v[K_BUFSUB_B] + v[K_TEXIMG_B] + v[K_TEXSUB_B] + v[K_COPYBUF_B];
  }
  bool createdNothing() const {
    return v[K_BUFDATA] == 0 && v[K_TEXIMG] == 0 && v[K_GENBUF] == 0 && v[K_GENTEX] == 0;
  }
};

// (Re)install the counting wrappers. Call once a GL context exists (VTK has
// loaded glad), and again after anything that may reload it; idempotent.
void glcountInstall();
GLCount glcountRead();
std::string glcountStr(const GLCount &d);

} // namespace cvcgl_test

#endif
