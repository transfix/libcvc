/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// Test-only GL call counter: swaps VTK's glad function pointers (glad_gl*) for
// counting wrappers, so a test can see every GL call the scene issues, by entry
// point. Uniform uploads are also checked against the last value sent to the
// same (program, location): a re-send of an unchanged value counts as
// redundant. Desktop GL only (the wasm VTK calls GLES directly, not via glad).
#ifndef CVCGL_TEST_GL_CALL_COUNTER_H
#define CVCGL_TEST_GL_CALL_COUNTER_H

#include <string>
#include <vector>

namespace cvcgl_test {

struct GLCalls {
  std::vector<double> n;       // per entry point (index = glcallsName(i))
  double uniformRedundant = 0; // uniform uploads that repeated the current value

  GLCalls operator-(const GLCalls &o) const;
  GLCalls &operator+=(const GLCalls &o);
  double total() const;
  double count(const std::string &entry) const; // entry without the "gl" prefix
  double uniforms() const;                      // every glUniform*
  // Calls that make WebGL wait for the GPU process: glGet*, glIsEnabled,
  // glGetError, glGetUniformLocation, glCheckFramebufferStatus, glReadPixels...
  double sync() const;
  // Same count for every entry point; otherwise the first difference.
  bool sameAs(const GLCalls &o, std::string *firstDifference = nullptr) const;
  // "total T (uniforms U, redundant R, sync S) | Entry=k ..." with every number
  // divided by `per`; the `top` largest entries.
  std::string str(double per = 1.0, int top = 14) const;
};

// (Re)install the wrappers. Call once a GL context exists (VTK has loaded glad),
// and again after anything that may reload it; idempotent.
void glcallsInstall();
GLCalls glcallsRead();
int glcallsEntries();
const char *glcallsName(int i);

} // namespace cvcgl_test

#endif
