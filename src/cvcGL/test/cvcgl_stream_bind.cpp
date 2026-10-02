// Phase-2 PR8: the (gl-bind-stream NODE-ID TOKEN) verb SEAM. This proves the genuinely new part
// of gl-bind-stream — registration, the per-document routing through ictx.document, argument
// parsing (positional NODE-ID + a token that may be a bare string OR a (stream-open) handle dict),
// the success/err value_t dicts, and the error paths — WITHOUT any GL: a neutral cvc::ariadne
// Runtime drives a tick-resident program that calls the verb, and a mock StreamBindingSink stands
// in for the cvcGL GlSceneAdapter, recording what the verb routed to it. The REAL GL binding path
// (GlSceneAdapter::bind_stream -> StreamTextureBinding applied to a scene node) is covered by
// cvcgl_stream_texture.cpp + cvcgl_stream_node.cpp, which exercise the same StreamTextureBinding in
// a live scene; here we own the verb <-> sink contract.

// cvcpkg builds tests Release (NDEBUG), which would make every assert() vacuous.
#undef NDEBUG
#include <cassert>
#include <cstdio>
#include <cvc/ariadne/ariadne.h>            // Runtime, Runtime::document_scope()
#include <cvc/core/app.h>                   // cvc::app
#include <cvc/core/state.h>                 // cvc::state::instance
#include <cvc/core/state_exec/intrinsics.h> // document_scope::slot
#include <cvc/gl/ariadne/stream_verbs.h>    // StreamBindingSink / GlStreamSinkHandle / register
#include <string>
#include <utility>
#include <vector>

using cvc::ariadne::Runtime;
namespace gla = cvc::gl::ariadne;

// A stand-in for the cvcGL GlSceneAdapter: records every (node, token) the verb routed to it, and
// returns a configurable result ("" = bound; non-empty = the verb surfaces it as the error).
struct MockSink : gla::StreamBindingSink {
  std::vector<std::pair<std::string, std::string>> calls;
  std::string next_error; // "" => success
  std::string bind_stream(const std::string &node, const std::string &token) override {
    calls.emplace_back(node, token);
    return next_error;
  }
};

// Install `sink` as THIS runtime's GL sink (what AriRuntime's ctor does), then run `prog` as the
// document's on:tick body (a backend-free action lane) until it writes "<prefix>.done", draining a
// few frames to let the resident submit + fire.
static void run_tick(Runtime &rt, cvc::app &app, const std::string &prefix,
                     gla::StreamBindingSink *sink, const std::string &prog) {
  if (sink)
    rt.document_scope().slot<gla::GlStreamSinkHandle>(gla::kGlStreamSinkSlot)->sink = sink;
  rt.set_tick_program(prog);
  for (int i = 0; i < 6 && cvc::state::instance(app)(prefix + ".done").value().empty(); ++i)
    rt.drain();
}

static std::string get(cvc::app &app, const std::string &path) {
  return cvc::state::instance(app)(path).value();
}

int main() {
  cvc::app app;
  gla::register_gl_stream_intrinsics(); // append the process-global verb provider (once is enough)

  // ── 1. success: the verb parses (NODE-ID TOKEN), routes to THIS doc's sink, returns the dict ──
  {
    Runtime rt(app, "t1");
    MockSink sink;
    run_tick(rt, app, "t1", &sink,
             "(begin"
             "  (set h (gl-bind-stream \"screen\" \"streams.cam0\"))"
             "  (state-set \"ok\" (get-attr h \"ok\"))"
             "  (state-set \"node\" (get-attr h \"node\"))"
             "  (state-set \"token\" (get-attr h \"token\"))"
             "  (state-set \"done\" \"1\"))");
    assert(get(app, "t1.ok") == "true" && "verb should report ok");
    assert(get(app, "t1.node") == "screen");
    assert(get(app, "t1.token") == "streams.cam0");
    assert(sink.calls.size() == 1 && "verb routed exactly one bind to this document's sink");
    assert(sink.calls[0].first == "screen" && sink.calls[0].second == "streams.cam0");
  }

  // ── 2. token may be a (stream-open) HANDLE DICT; the verb reads its "token" field ──
  {
    Runtime rt(app, "t2");
    MockSink sink;
    run_tick(rt, app, "t2", &sink,
             "(begin"
             "  (set h (gl-bind-stream \"screen\" (dict \"token\" \"demo.streams.cam0\")))"
             "  (state-set \"ok\" (get-attr h \"ok\"))"
             "  (state-set \"done\" \"1\"))");
    assert(get(app, "t2.ok") == "true");
    assert(sink.calls.size() == 1 && sink.calls[0].second == "demo.streams.cam0" &&
           "handle-dict token should resolve to its 'token' field");
  }

  // ── 3. the sink's error is surfaced verbatim (no node / wrong type / pool full live here) ──
  {
    Runtime rt(app, "t3");
    MockSink sink;
    sink.next_error = "gl-bind-stream: no GeometryNode 'ghost' in the scene";
    run_tick(rt, app, "t3", &sink,
             "(begin"
             "  (set h (gl-bind-stream \"ghost\" \"streams.cam0\"))"
             "  (state-set \"ok\" (get-attr h \"ok\"))"
             "  (state-set \"err\" (get-attr h \"error\"))"
             "  (state-set \"done\" \"1\"))");
    assert(get(app, "t3.ok") == "false");
    assert(get(app, "t3.err").find("no GeometryNode 'ghost'") != std::string::npos);
  }

  // ── 4. a document with NO GL sink (no cvcGL host) fails cleanly, not a crash ──
  {
    Runtime rt(app, "t4");
    run_tick(rt, app, "t4", /*sink=*/nullptr,
             "(begin"
             "  (set h (gl-bind-stream \"screen\" \"streams.cam0\"))"
             "  (state-set \"ok\" (get-attr h \"ok\"))"
             "  (state-set \"err\" (get-attr h \"error\"))"
             "  (state-set \"done\" \"1\"))");
    assert(get(app, "t4.ok") == "false");
    assert(get(app, "t4.err").find("no GL scene sink") != std::string::npos);
  }

  // ── 5. missing TOKEN arg is rejected with the usage message, sink untouched ──
  {
    Runtime rt(app, "t5");
    MockSink sink;
    run_tick(rt, app, "t5", &sink,
             "(begin"
             "  (set h (gl-bind-stream \"screen\"))"
             "  (state-set \"ok\" (get-attr h \"ok\"))"
             "  (state-set \"err\" (get-attr h \"error\"))"
             "  (state-set \"done\" \"1\"))");
    assert(get(app, "t5.ok") == "false");
    assert(get(app, "t5.err").find("expects (gl-bind-stream NODE-ID TOKEN)") != std::string::npos);
    assert(sink.calls.empty() && "a malformed call must not reach the sink");
  }

  std::printf("cvcgl_stream_bind: OK\n");
  return 0;
}
