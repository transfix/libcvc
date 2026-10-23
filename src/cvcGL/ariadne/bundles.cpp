// bundles.cpp — see inc/cvc/gl/ariadne/bundles.h.

#include <cvc/ariadne/ariadne.h>                    // register_widget_type, text, text_bound
#include <cvc/ariadne/conference/room_intrinsics.h> // register_room_intrinsics
#include <cvc/ariadne/stream_intrinsics.h>          // register_stream_intrinsics
#include <cvc/gl/ariadne/bundles.h>
#include <cvc/gl/ariadne/extensions.h>         // register_cvcgl_extensions
#include <cvc/gl/ariadne/media_verbs.h>        // register_media_intrinsics
#include <cvc/gl/ariadne/stream_node.h>        // register_stream_node_type
#include <cvc/gl/ariadne/stream_verbs.h>       // register_gl_stream_intrinsics
#include <cvc/gl/ariadne/stream_view_widget.h> // register_stream_view_widget

namespace cvc {
namespace gl {
namespace ariadne {

namespace ari = cvc::ariadne;

void register_gl_basics(cvc::app &app) {
  register_cvcgl_extensions(app);
  // A custom widget composed from built-ins (a caption and a read-only value), so it draws on every
  // backend:   - type: labeled
  //              title: Belief
  //              bind: demo.belief
  ari::register_widget_type("labeled", [](const ari::Widget &w, const ari::WidgetEmitContext &ctx) {
    ctx.emit(ari::text(w.props.str("title", w.label)));
    ctx.emit(ari::text_bound("  =", w.bind));
  });
}

void register_streaming(cvc::app &app) {
  ari::register_stream_intrinsics(app); // (stream-open / stream-info / stream-close)
  register_gl_stream_intrinsics();      // (gl-bind-stream)
  register_media_intrinsics();          // (stream-capture-camera / -audio, stream-play-audio)
  register_stream_node_type();          // type: stream (video on a quad)
}

void register_window_widgets(cvc::gl::ImGuiBackend &backend, cvc::app &app) {
  register_stream_view_widget(backend, app); // type: stream_view
}

void register_conference(cvc::app &app, cvc::gl::ImGuiBackend &backend) {
  register_streaming(app);
  ari::conference::register_room_intrinsics(app); // (room-join / room-roster / room-leave)
  register_window_widgets(backend, app);
}

} // namespace ariadne
} // namespace gl
} // namespace cvc
