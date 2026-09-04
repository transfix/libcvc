// Phase-5 zero-copy texture path: GeometryNode::setTexture(img) (zeroCopy
// default) makes the actor's vtkTexture ALIAS the cvc::image's RGBA8 pixel
// buffer — no memcpy. This test proves the aliasing (the texture's scalar array
// points at the SAME memory), that an in-place edit of the image's bytes is seen
// by the texture WITHOUT a re-set, that texture_modified() bumps the texture
// MTime, and that the TCoords' V is flipped (so no pixel flip-copy is needed).
// Pure data-structure checks — no GL context.

// These tests assert(), and cvcpkg builds them Release -- where NDEBUG makes
// assert() expand to nothing and every check below would pass vacuously.
// Undefine it before <cassert> so the assertions actually run.
#undef NDEBUG
#include <cassert>
#include <cstdio>
#include <cvc/core/app.h>
#include <cvc/geometry/geometry.h>
#include <cvc/gl/GeometryNode.h>
#include <cvc/image/image.h>
#include <vtkActor.h>
#include <vtkDataArray.h>
#include <vtkImageData.h>
#include <vtkMapper.h>
#include <vtkPointData.h>
#include <vtkPolyData.h>
#include <vtkTexture.h>
#include <vtkUnsignedCharArray.h>

using cvc::gl::GeometryNode;

using namespace cvc;

// Expose the protected getProp() so the test can inspect the VTK actor/texture.
class TestGeomNode : public GeometryNode {
public:
  explicit TestGeomNode(cvc::app &a) : GeometryNode(a, "test.texzc", "texzc") {}
  vtkProp *prop() { return getProp(); }
};

static geometry uv_quad() {
  geometry g;
  g.points() = {{{-1, -1, 0}}, {{1, -1, 0}}, {{1, 1, 0}}, {{-1, 1, 0}}};
  g.uvs() = {{{0.0, 0.0}}, {{1.0, 0.0}}, {{1.0, 1.0}}, {{0.0, 1.0}}};
  g.tris() = {{{0, 1, 2}}, {{0, 2, 3}}};
  return g;
}

int main() {
  cvc::app app;
  TestGeomNode node(app);
  node.setUseSingleColor(false);
  node.setGeometry(uv_quad());

  vtkActor *actor = vtkActor::SafeDownCast(node.prop());
  assert(actor && "no actor");

  // An RGBA8 image with a recognizable first pixel.
  image img(8, 8, image::pixel_format::RGBA, image::data_type::u8);
  img.storage().get()[0] = 11;
  img.storage().get()[1] = 22;
  img.storage().get()[2] = 33;
  img.storage().get()[3] = 44;

  // Record the exact buffer the node should alias.
  unsigned char *bufBefore = img.storage().get();

  node.setTexture(img); // zero-copy default

  vtkTexture *tex = actor->GetTexture();
  assert(tex && "setTexture did not attach a texture");
  vtkImageData *id = vtkImageData::SafeDownCast(tex->GetInput());
  assert(id && "texture has no image data input");
  vtkDataArray *scalars = id->GetPointData()->GetScalars();
  assert(scalars && "texture image data has no scalars");

  // 1. ALIASING: the texture's scalar array points at the image's OWN buffer —
  //    proof there was no copy.
  assert(scalars->GetVoidPointer(0) == static_cast<void *>(bufBefore) &&
         "zero-copy setTexture did NOT alias the image buffer (a copy was made)");
  assert(scalars->GetNumberOfComponents() == 4 && "expected RGBA scalars");

  // 2. LIVE EDIT: mutate the image's bytes in place (through storage(), no COW
  //    detach) — the texture's scalars reflect it WITHOUT a new setTexture.
  img.storage().get()[0] = 200;
  img.storage().get()[2] = 250;
  vtkUnsignedCharArray *uc = vtkUnsignedCharArray::SafeDownCast(scalars);
  assert(uc && "scalars are not a uchar array");
  assert(uc->GetValue(0) == 200 && "in-place pixel edit not visible in the aliased texture");
  assert(uc->GetValue(2) == 250 && "in-place pixel edit not visible in the aliased texture");

  // 3. texture_modified() bumps the texture MTime so VTK re-samples on next render.
  vtkMTimeType before = tex->GetMTime();
  node.texture_modified();
  assert(tex->GetMTime() > before && "texture_modified() did not bump the texture MTime");

  // 4. TCoords' V is flipped when a texture is active (no pixel flip-copy): the
  //    quad's uv (0,0) -> (0,1), uv (1,1) -> (1,0).
  vtkPolyData *pd = vtkPolyData::SafeDownCast(actor->GetMapper()->GetInput());
  assert(pd && "no polydata");
  vtkDataArray *tc = pd->GetPointData()->GetTCoords();
  assert(tc && "UVs did not reach SetTCoords");
  double uv0[2];
  tc->GetTuple(0, uv0); // vertex 0: original (0,0)
  assert(uv0[0] == 0.0 && uv0[1] == 1.0 && "TCoords V not flipped for active texture");

  // 5. clearTexture drops the texture, the aliased buffer, and un-flips TCoords.
  node.clearTexture();
  assert(actor->GetTexture() == nullptr && "clearTexture did not remove the texture");
  tc = pd->GetPointData()->GetTCoords();
  assert(tc && "TCoords should still exist (geometry has uvs)");
  tc->GetTuple(0, uv0);
  assert(uv0[0] == 0.0 && uv0[1] == 0.0 && "clearTexture did not restore un-flipped TCoords");

  // 6. Non-RGBA8 image uses the copy fallback (no aliasing, but still textures).
  image gray(4, 4, image::pixel_format::GRAY, image::data_type::u8);
  node.setTexture(gray);
  assert(actor->GetTexture() && "fallback setTexture did not attach a texture");

  printf("CVCGL_TEXTURE_ZEROCOPY_OK\n");
  return 0;
}
