// bundles.h — the registration bundles an Ariadne GL host (arigl, the cvcGL examples) makes before
// it loads a document.
//
// Ariadne's registries (widget types, scene node types, program-lane intrinsics, ...) are
// populated by explicit host calls. The example hosts each made their own subset of these calls;
// these functions are the named sets, so every host agrees on what a document can use. All are
// safe to call more than once (the node-type and widget registrars are idempotent, and a repeated
// intrinsics provider rebinds the same verbs).
//
//   register_gl_basics       cvcGL's node types and shader presets (register_cvcgl_extensions)
//                            and the `labeled` custom widget hello.ari uses.
//   register_streaming       the stream verbs (stream-open/info/close), gl-bind-stream, the media
//                            verbs (stream-capture-camera / -audio, stream-play-audio) and the
//                            `type: stream` scene node.
//   register_window_widgets  the ImGui-only widgets: `stream_view`.
//   register_conference      a conference document's set: register_streaming, the room verbs
//                            (room-join / roster / leave) and `stream_view`. The gRPC clustering
//                            (state / media transports, the shard, stream_bridge) is not a
//                            registration and stays with the host.
//
// Terminal hosts (`arigl --terminal`) make the first two; the ImGui ones are window-only.
#pragma once

namespace cvc {
class app;
namespace gl {
class ImGuiBackend;
namespace ariadne {

void register_gl_basics(cvc::app &app);
void register_streaming(cvc::app &app);
void register_window_widgets(cvc::gl::ImGuiBackend &backend, cvc::app &app);
void register_conference(cvc::app &app, cvc::gl::ImGuiBackend &backend);

} // namespace ariadne
} // namespace gl
} // namespace cvc
