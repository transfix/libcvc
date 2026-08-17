// Phase-4 cvcGL texture support: geometry UVs reach the polydata's SetTCoords
// slot, and setTexture(cvc::image) attaches an RGBA vtkTexture to the actor
// (clearTexture removes it). Pure data-structure checks — no GL context needed.

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
#include <vtkMapper.h>
#include <vtkPointData.h>
#include <vtkPolyData.h>
#include <vtkTexture.h>

using namespace cvc;

// Expose the protected getProp() so the test can inspect the VTK actor/polydata.
class TestGeomNode : public GeometryNode {
public:
  explicit TestGeomNode(cvc::app &a) : GeometryNode(a, "test.tex", "tex") {}
  vtkProp *prop() { return getProp(); }
};

static geometry uv_triangle() {
  geometry g;
  g.points() = {{{0, 0, 0}}, {{1, 0, 0}}, {{0, 1, 0}}};
  g.uvs() = {{{0.0, 0.0}}, {{1.0, 0.0}}, {{0.0, 1.0}}};
  g.tris() = {{{0, 1, 2}}};
  return g;
}

int main() {
  cvc::app app;
  TestGeomNode node(app);
  node.setUseSingleColor(false);

  // 1. geometry WITH uvs -> polydata TCoords populated
  node.setGeometry(uv_triangle());
  vtkActor *actor = vtkActor::SafeDownCast(node.prop());
  assert(actor && "no actor");
  vtkPolyData *pd = vtkPolyData::SafeDownCast(actor->GetMapper()->GetInput());
  assert(pd && "no polydata");
  vtkDataArray *tc = pd->GetPointData()->GetTCoords();
  assert(tc && "UVs did not reach SetTCoords");
  assert(tc->GetNumberOfTuples() == 3 && "wrong tcoord count");
  assert(tc->GetNumberOfComponents() == 2 && "tcoords must be 2-component");
  double uv[2];
  tc->GetTuple(1, uv);
  assert(uv[0] == 1.0 && uv[1] == 0.0 && "tcoord value wrong");

  // 2. geometry WITHOUT uvs -> TCoords cleared
  geometry g2;
  g2.points() = {{{0, 0, 0}}, {{1, 0, 0}}, {{0, 1, 0}}};
  g2.tris() = {{{0, 1, 2}}};
  node.setGeometry(g2);
  assert(pd->GetPointData()->GetTCoords() == nullptr && "TCoords not cleared when no uvs");

  // 3. setTexture attaches a vtkTexture; clearTexture removes it
  node.setGeometry(uv_triangle());
  assert(actor->GetTexture() == nullptr && "unexpected texture before setTexture");
  image tex(4, 4, image::pixel_format::RGBA, image::data_type::u8);
  node.setTexture(tex);
  vtkTexture *vt = actor->GetTexture();
  assert(vt && "setTexture did not attach a vtkTexture");
  assert(vt->GetInput() && "texture has no image data");
  node.clearTexture();
  assert(actor->GetTexture() == nullptr && "clearTexture did not remove the texture");

  // 4. an empty image clears any existing texture
  node.setTexture(tex);
  node.setTexture(image());
  assert(actor->GetTexture() == nullptr && "empty image should clear the texture");

  printf("CVCGL_TEXTURE_OK\n");
  return 0;
}
