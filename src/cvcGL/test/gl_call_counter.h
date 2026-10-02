/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// Test-only GL call counter and tracer: swaps VTK's glad function pointers
// (glad_gl*) for wrappers, so a test can see every GL call the scene issues --
// as counts per entry point, and (while a trace is open) as an ordered trace of
// (entry point, bound program / active texture unit, argument bytes, uniform
// values). Uniform uploads are also checked against the last value sent to the
// same (program, location): a re-send of an unchanged value counts as
// redundant. Every entry point VTK 9.5.0's Rendering/OpenGL2 and
// RenderingVolumeOpenGL2 and cvcGL call is wrapped, plus the GLES3-core
// families they may grow into (glUniform*ui*, glClearBuffer*, glTexStorage*,
// ...), so frame totals are complete. Desktop GL only (the wasm VTK calls GLES
// directly, not via glad). glcallsUninstall() puts VTK's pointers back, for
// timing without the wrappers' cost.
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
  // Desktop round-trips (upper bound): calls that wait on the driver on desktop
  // GL -- glGet*, glIs*, glGetError, glGet*Location, glCheckFramebufferStatus,
  // glReadPixels, glMapBuffer*, query results, glFinish. NOT a WebGL census:
  // Firefox answers some of them (cached limits, glIsProgram, ...) in the
  // content process, so on WebGL this is an upper bound.
  double roundTrips() const;
  // Same count for every entry point; otherwise the first difference.
  bool sameAs(const GLCalls &o, std::string *firstDifference = nullptr) const;
  // "total T (uniforms U, redundant R, desktop round-trips (upper bound) S) |
  // Entry=k ..." with every number divided by `per`; the `top` largest entries.
  std::string str(double per = 1.0, int top = 14) const;
};

// One traced GL call, or a marker the test inserted (glcallsMark).
struct TraceRec {
  int entry = 0;    // glcallsName(entry); >= kTraceMarkBase: marker kTraceMarkBase + kind
  unsigned ctx = 0; // the bound program for glUniform* and draw calls, the active texture
                    // unit (0-based) for texture-state calls, 0 for the rest; a marker's arg
  std::string args; // the arguments' bytes as passed (a pointer: whether it is null), the
                    // values a glUniform*v / glUniformMatrix* sends, the name a
                    // glGet*Location looks up
  bool marker() const;
  bool operator==(const TraceRec &o) const {
    return entry == o.entry && ctx == o.ctx && args == o.args;
  }
  bool operator!=(const TraceRec &o) const { return !(*this == o); }
};
using Trace = std::vector<TraceRec>;
constexpr int kTraceMarkBase = 1 << 20;
// Marker kinds glcallsTraceStart() puts first: the GL state the trace starts
// from that vtkOpenGLState caches, args = the float value.
constexpr int kTraceInitPointSize = 1000, kTraceInitLineWidth = 1001;

// (Re)install the wrappers. Call once a GL context exists (VTK has loaded glad),
// and again after anything that may reload it; idempotent.
void glcallsInstall();
// Put VTK's own function pointers back (no wrapper on any call); idempotent.
void glcallsUninstall();
GLCalls glcallsRead();
int glcallsEntries();
const char *glcallsName(int i);

// Start recording every wrapped call in order (clears the previous trace; the
// trace opens with kTraceInitPointSize / kTraceInitLineWidth markers); stop and
// take it.
void glcallsTraceStart();
Trace glcallsTraceStop();
// Insert a marker at this point of the open trace (no-op when none is open).
void glcallsMark(int kind, int arg);
// glDraw* / glDispatchCompute.
bool glcallsIsDraw(int entry);
// The index of an entry point ("PointSize"), or -1.
int glcallsEntry(const std::string &name);
// "glUniform1i prog=3 [05 00 00 00 01 00 00 00]" / "mark 5 (2)".
std::string glcallsDescribe(const TraceRec &r);

} // namespace cvcgl_test

#endif
