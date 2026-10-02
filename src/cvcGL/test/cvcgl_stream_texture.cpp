// Phase-2 PR2: StreamTextureBinding drives a GeometryNode's texture from a
// cvc::ariadne::stream video subscription. This proves the END-TO-END ZERO-COPY path with
// no GL context and no hardware: a frame published on the stream is, on tick(),
// aliased into the node's vtkTexture whose scalar array points at the SAME bytes
// the stream frame borrows (no memcpy). Also covers the seq dirty-check (a tick
// with no newer frame is a no-op) and prompt unsubscribe on node teardown (so the
// pool slab + the channel's committed slots are released, not leaked for the
// scene's lifetime). setTexture() runs inline on the calling thread here (no pump),
// so the texture state is observable immediately — same discipline as
// cvcgl_texture_zerocopy.cpp.

// cvcpkg builds tests Release (NDEBUG), which would make every assert() vacuous.
#undef NDEBUG
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cvc/ariadne/stream/frame.h>
#include <cvc/ariadne/stream/stream.h>
#include <cvc/ariadne/stream/stream_channel.h>
#include <cvc/core/app.h>
#include <cvc/geometry/geometry.h>
#include <cvc/gl/GeometryNode.h>
#include <cvc/gl/ariadne/stream_texture_binding.h>
#include <memory>
#include <vtkActor.h>
#include <vtkDataArray.h>
#include <vtkImageData.h>
#include <vtkPointData.h>
#include <vtkTexture.h>

using cvc::gl::GeometryNode;
using cvc::gl::ariadne::StreamTextureBinding;
using namespace cvc;

// Expose the protected getProp() so the test can inspect the actor/texture.
class TestGeomNode : public GeometryNode {
public:
  explicit TestGeomNode(cvc::app &a) : GeometryNode(a, "test.streamtex", "streamtex") {}
  vtkProp *prop() { return getProp(); }
};

static geometry uv_quad() {
  geometry g;
  g.points() = {{{-1, -1, 0}}, {{1, -1, 0}}, {{1, 1, 0}}, {{-1, 1, 0}}};
  g.uvs() = {{{0.0, 0.0}}, {{1.0, 0.0}}, {{1.0, 1.0}}, {{0.0, 1.0}}};
  g.tris() = {{{0, 1, 2}}, {{0, 2, 3}}};
  return g;
}

static cvc::ariadne::stream::stream_params video_params(const char *id, int w, int h) {
  cvc::ariadne::stream::stream_params p;
  p.id = id;
  p.format.kind = cvc::ariadne::stream::frame_kind::video_raw;
  p.format.codec = "rgba8";
  p.format.w = w;
  p.format.h = h;
  p.format.stride = w * 4; // dense -> frame_bytes = w*4*h, so open() accepts it
  p.subscriber_depth = 2;
  p.expected_subscribers = 1;
  return p;
}

static const void *publish_tagged(cvc::ariadne::stream::stream &s,
                                  cvc::ariadne::stream::subscription &sub, std::uint8_t tag) {
  auto l = s.channel().pool().acquire();
  assert(l.has_value() && "pool exhausted");
  l->data[0] = tag;
  s.channel().publish(*l, l->cap, 0.0);
  return sub.latest()->data; // the slab the freshest frame borrows
}

static vtkDataArray *texture_scalars(TestGeomNode &node) {
  vtkActor *actor = vtkActor::SafeDownCast(node.prop());
  assert(actor && "no actor");
  vtkTexture *tex = actor->GetTexture();
  assert(tex && "no texture attached");
  vtkImageData *id = vtkImageData::SafeDownCast(tex->GetInput());
  assert(id && "texture has no image data");
  vtkDataArray *scalars = id->GetPointData()->GetScalars();
  assert(scalars && "texture image data has no scalars");
  return scalars;
}

int main() {
  cvc::app app;
  const int W = 8, H = 8;

  auto s = cvc::ariadne::stream::stream::open(app, video_params("vid", W, H));
  assert(s && "stream::open failed");
  auto sub = s->channel().subscribe(cvc::ariadne::stream::deliver_mode::latest, 2);
  assert(sub && "subscribe failed");

  auto node = std::make_shared<TestGeomNode>(app);
  node->setUseSingleColor(false);
  node->setGeometry(uv_quad());

  // The binding resolves the channel by the stream's canonical registry key.
  StreamTextureBinding binding(app, s->token(), node, sub);
  assert(binding.last_seq() == -1 && "fresh binding should have no applied frame");

  // 1. Publish a frame; a tick aliases it into the texture ZERO-COPY.
  const void *frame0 = publish_tagged(*s, *sub, 0xA1);
  binding.tick();
  assert(binding.last_seq() == 0 && "tick did not apply the first frame");
  vtkDataArray *scalars = texture_scalars(*node);
  assert(scalars->GetVoidPointer(0) == frame0 &&
         "zero-copy end-to-end FAILED: texture does not alias the frame buffer");
  assert(scalars->GetNumberOfComponents() == 4 && "expected RGBA scalars");

  // 2. Seq dirty-check: a tick with no newer frame is a no-op.
  binding.tick();
  assert(binding.last_seq() == 0 && "tick re-applied a frame with no new seq");

  // 3. A newer frame advances the seq and re-aliases the new buffer.
  const void *frame1 = publish_tagged(*s, *sub, 0xB2);
  binding.tick();
  assert(binding.last_seq() == 1 && "tick did not apply the second frame");
  assert(texture_scalars(*node)->GetVoidPointer(0) == frame1 &&
         "second frame not aliased into the texture");

  // 4. Node teardown: the next tick unsubscribes, releasing the committed slots.
  assert(s->channel().subscriber_count() == 1 && "expected one subscriber");
  node.reset(); // drops the texture (its m_textureStorage ref to the last slab);
                // the binding holds only a weak_ptr
  binding.tick();
  assert(!binding.subscribed() && "binding did not unsubscribe after node teardown");
  assert(s->channel().subscriber_count() == 0 &&
         "subscription leaked after node teardown (committed slots not released)");

  // 5. The last slab is still pinned by the test's own surviving subscription
  //    ref (its latest_ register); dropping it recycles the slab -> pool empty.
  //    This proves the slab (not just the slot accounting) is released.
  assert(s->channel().pool().in_use() == 1 && "expected the last frame still pinned by `sub`");
  sub.reset();
  assert(s->channel().pool().in_use() == 0 && "pool slab not recycled after last ref dropped");

  std::printf("cvcgl_stream_texture: OK\n");
  return 0;
}
