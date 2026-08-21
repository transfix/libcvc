// Tests for the SDF v2 internals under src/cvc/geometry/SDF/SignDistanceFunction_v2:
// mtxlib (vector2/3/4, matrix33/44), FaceVertSet3D, BufferedIO, the parsers
// (Geom3DParser, RawivParser) and DistanceTransform construction + geometric
// predicates.
//
// Include order matters here, and is enforced against clang-format. reg3data.h
// (pulled in by DistanceTransform.h) defines a function-like macro `error(x)`.
// boost/parameter/parameters.hpp — reached transitively through cvc/core/app.h
// and gtest via boost.signals2 — contains a bare `error();` call, which that
// macro clobbers into a syntax error. So the boost-pulling headers must be
// parsed BEFORE the SDF v2 headers define the macro.
//
// The repo .clang-format is IncludeBlocks:Regroup with quoted includes sorted
// ahead of angle-bracket ones, so it would hoist the SDF headers above the
// cvc/boost includes and re-break the build — unless a non-include line splits
// them into separate, individually-sorted blocks. The #undef barriers below do
// exactly that (and are correct on their own merits). Do not merge these blocks.
#include <cmath>
#include <cstdio>
#include <cstring>
#include <cvc/core/app.h>
#include <gtest/gtest.h>
#include <string>
#include <vector>

#ifdef error
#undef error // no-op here; keeps this block separate from the SDF block below
#endif

#include "DistanceTransform.h" // pulls in FaceVertSet3D.h, reg3data.h, RawivParser.h
#include "Geom3DParser.h"
#include "bufferedio.h"
#include "mtxlib.h"

#ifdef error
#undef error // reg3data.h's error(x) macro must not leak into the test body
#endif

namespace {

const float kEps = 1e-5f;

std::string tmpPath(const char *name) { return ::testing::TempDir() + "mtxlib_test_" + name; }

long fileSize(const std::string &path) {
  FILE *f = fopen(path.c_str(), "rb");
  if (!f)
    return -1;
  fseek(f, 0, SEEK_END);
  long sz = ftell(f);
  fclose(f);
  return sz;
}

// Unit cube [-1,1]^3 with outward-facing triangle normals (same winding as the
// public SDF flip-normals tests in geometry_test.cpp).
void buildCube(FaceVertSet3D &fvs) {
  const float V[8][3] = {{-1, -1, -1}, {1, -1, -1}, {1, 1, -1}, {-1, 1, -1},
                         {-1, -1, 1},  {1, -1, 1},  {1, 1, 1},  {-1, 1, 1}};
  for (int i = 0; i < 8; i++)
    fvs.addVert(V[i][0], V[i][1], V[i][2]);
  const int T[12][3] = {{0, 2, 1}, {0, 3, 2},  // bottom (-Z)
                        {4, 5, 6}, {4, 6, 7},  // top (+Z)
                        {0, 1, 5}, {0, 5, 4},  // front (-Y)
                        {2, 3, 7}, {2, 7, 6},  // back (+Y)
                        {0, 4, 7}, {0, 7, 3},  // left (-X)
                        {1, 2, 6}, {1, 6, 5}}; // right (+X)
  for (int i = 0; i < 12; i++)
    fvs.AddTri(TriId3i(T[i][0], T[i][1], T[i][2]));
  fvs.buildBBox();
  fvs.computeTriNormals();
}

void expectMat44Near(const matrix44 &a, const matrix44 &b, float tol) {
  for (unsigned c = 0; c < 4; c++)
    for (unsigned r = 0; r < 4; r++)
      EXPECT_NEAR(a[c][r], b[c][r], tol) << "col " << c << " row " << r;
}

} // namespace

// ===========================================================================
// vector2
// ===========================================================================

TEST(MtxlibVector2, BasicOps) {
  vector2 a(1.0f, 2.0f);
  vector2 b(a);
  EXPECT_EQ(a, b);
  EXPECT_FLOAT_EQ(a[0], 1.0f);
  EXPECT_FLOAT_EQ(a[1], 2.0f);
  const vector2 &ca = a;
  EXPECT_FLOAT_EQ(ca[1], 2.0f);

  b.set(3.0f, 4.0f);
  EXPECT_TRUE(a != b);
  EXPECT_FLOAT_EQ(b.length(), 5.0f);
  EXPECT_FLOAT_EQ(b.lengthSqr(), 25.0f);
  EXPECT_FALSE(b.isZero());
  EXPECT_TRUE(vector2(0.0f, 0.0f).isZero());

  vector2 c = a + b;
  EXPECT_FLOAT_EQ(c.x, 4.0f);
  EXPECT_FLOAT_EQ(c.y, 6.0f);
  c = b - a;
  EXPECT_FLOAT_EQ(c.x, 2.0f);
  EXPECT_FLOAT_EQ(c.y, 2.0f);
  c = -a;
  EXPECT_FLOAT_EQ(c.x, -1.0f);
  c = a * 2.0f;
  EXPECT_FLOAT_EQ(c.y, 4.0f);
  c = 2.0f * a;
  EXPECT_FLOAT_EQ(c.x, 2.0f);
  c = b / 2.0f;
  EXPECT_FLOAT_EQ(c.x, 1.5f);

  c = a;
  c += b;
  EXPECT_FLOAT_EQ(c.x, 4.0f);
  c -= b;
  EXPECT_FLOAT_EQ(c.x, 1.0f);
  c *= 3.0f;
  EXPECT_FLOAT_EQ(c.y, 6.0f);
  c /= 3.0f;
  EXPECT_FLOAT_EQ(c.y, 2.0f);
}

TEST(MtxlibVector2, NormalizeAndHelpers) {
  vector2 v(3.0f, 4.0f);
  v.normalize();
  EXPECT_NEAR(v.x, 0.6f, kEps);
  EXPECT_NEAR(v.y, 0.8f, kEps);

  vector2 z(0.0f, 0.0f);
  z.normalize(); // zero-length branch
  EXPECT_TRUE(z.isZero());

  vector2 n = Normalized(vector2(0.0f, 5.0f));
  EXPECT_NEAR(n.y, 1.0f, kEps);

  EXPECT_FLOAT_EQ(DotProduct(vector2(1, 2), vector2(3, 4)), 11.0f);

  vector2 a(1, 2), b(3, 4);
  SwapVec(a, b);
  EXPECT_FLOAT_EQ(a.x, 3.0f);
  EXPECT_FLOAT_EQ(b.x, 1.0f);

  EXPECT_TRUE(NearlyEquals(vector2(1, 1), vector2(1.001f, 1), 0.01f));
  EXPECT_FALSE(NearlyEquals(vector2(1, 1), vector2(2, 1), 0.01f));
}

// ===========================================================================
// vector3
// ===========================================================================

TEST(MtxlibVector3, BasicOps) {
  vector3 a(1.0f, 2.0f, 3.0f);
  vector3 b(a);
  EXPECT_EQ(a, b);
  EXPECT_FLOAT_EQ(a[2], 3.0f);
  const vector3 &ca = a;
  EXPECT_FLOAT_EQ(ca[0], 1.0f);

  vector3 fromV2(vector2(5.0f, 6.0f));
  EXPECT_FLOAT_EQ(fromV2.x, 5.0f);
  EXPECT_FLOAT_EQ(fromV2.z, 0.0f);
  vector3 assigned;
  assigned = vector2(7.0f, 8.0f);
  EXPECT_FLOAT_EQ(assigned.y, 8.0f);
  EXPECT_FLOAT_EQ(assigned.z, 0.0f);

  b.set(2.0f, 3.0f, 6.0f);
  EXPECT_TRUE(a != b);
  EXPECT_FLOAT_EQ(b.length(), 7.0f);
  EXPECT_FLOAT_EQ(b.lengthSqr(), 49.0f);
  EXPECT_FALSE(b.isZero());
  EXPECT_TRUE(vector3().isZero());

  vector3 c = a + b;
  EXPECT_FLOAT_EQ(c.z, 9.0f);
  c = b - a;
  EXPECT_FLOAT_EQ(c.z, 3.0f);
  c = -a;
  EXPECT_FLOAT_EQ(c.x, -1.0f);
  c = a * 2.0f;
  EXPECT_FLOAT_EQ(c.z, 6.0f);
  c = 2.0f * a;
  EXPECT_FLOAT_EQ(c.y, 4.0f);
  c = b / 2.0f;
  EXPECT_FLOAT_EQ(c.z, 3.0f);

  c = a;
  c += b;
  EXPECT_FLOAT_EQ(c.x, 3.0f);
  c -= b;
  EXPECT_FLOAT_EQ(c.x, 1.0f);
  c *= 2.0f;
  EXPECT_FLOAT_EQ(c.y, 4.0f);
  c /= 2.0f;
  EXPECT_FLOAT_EQ(c.y, 2.0f);
}

TEST(MtxlibVector3, NormalizeCrossAndHelpers) {
  vector3 v(0.0f, 3.0f, 4.0f);
  v.normalize();
  EXPECT_NEAR(v.y, 0.6f, kEps);
  EXPECT_NEAR(v.z, 0.8f, kEps);

  vector3 z;
  z.normalize(); // zero-length branch
  EXPECT_TRUE(z.isZero());

  vector3 n = Normalized(vector3(2.0f, 0.0f, 0.0f));
  EXPECT_NEAR(n.x, 1.0f, kEps);

  EXPECT_FLOAT_EQ(DotProduct(vector3(1, 2, 3), vector3(4, 5, 6)), 32.0f);

  vector3 cx = CrossProduct(vector3(1, 0, 0), vector3(0, 1, 0));
  EXPECT_NEAR(cx.x, 0.0f, kEps);
  EXPECT_NEAR(cx.y, 0.0f, kEps);
  EXPECT_NEAR(cx.z, 1.0f, kEps);

  vector3 a(1, 2, 3), b(4, 5, 6);
  SwapVec(a, b);
  EXPECT_FLOAT_EQ(a.x, 4.0f);
  EXPECT_FLOAT_EQ(b.z, 3.0f);

  EXPECT_TRUE(NearlyEquals(vector3(1, 1, 1), vector3(1, 1, 1.001f), 0.01f));
  EXPECT_FALSE(NearlyEquals(vector3(1, 1, 1), vector3(1, 1, 2), 0.01f));
}

// ===========================================================================
// vector4
// ===========================================================================

TEST(MtxlibVector4, BasicOps) {
  vector4 a(1.0f, 2.0f, 3.0f, 4.0f);
  vector4 b(a);
  EXPECT_EQ(a, b);
  EXPECT_FLOAT_EQ(a[0], 1.0f);
  EXPECT_FLOAT_EQ(a[1], 2.0f);
  EXPECT_FLOAT_EQ(a[2], 3.0f);
  EXPECT_FLOAT_EQ(a[3], 4.0f);
  const vector4 &ca = a;
  EXPECT_FLOAT_EQ(ca[3], 4.0f);
  a[3] = 5.0f;
  EXPECT_FLOAT_EQ(a.w, 5.0f);
  a[3] = 4.0f;

  vector4 fromV3(vector3(1, 2, 3));
  EXPECT_FLOAT_EQ(fromV3.w, 0.0f);
  vector4 fromV2(vector2(1, 2));
  EXPECT_FLOAT_EQ(fromV2.z, 0.0f);
  EXPECT_FLOAT_EQ(fromV2.w, 0.0f);
  vector4 assigned;
  assigned = vector3(9, 8, 7);
  EXPECT_FLOAT_EQ(assigned.x, 9.0f);
  EXPECT_FLOAT_EQ(assigned.w, 0.0f);
  assigned = vector2(5, 6);
  EXPECT_FLOAT_EQ(assigned.y, 6.0f);
  EXPECT_FLOAT_EQ(assigned.z, 0.0f);

  b.set(2.0f, 2.0f, 2.0f, 2.0f);
  EXPECT_TRUE(a != b);
  EXPECT_FLOAT_EQ(b.length(), 4.0f);
  EXPECT_FLOAT_EQ(b.lengthSqr(), 16.0f);
  EXPECT_FALSE(b.isZero());
  EXPECT_TRUE(vector4(0, 0, 0, 0).isZero());

  vector4 c = a + b;
  EXPECT_FLOAT_EQ(c.w, 6.0f);
  c = a - b;
  EXPECT_FLOAT_EQ(c.w, 2.0f);
  c = -a;
  EXPECT_FLOAT_EQ(c.w, -4.0f);
  c = a * 2.0f;
  EXPECT_FLOAT_EQ(c.w, 8.0f);
  c = 2.0f * a;
  EXPECT_FLOAT_EQ(c.x, 2.0f);
  c = b / 2.0f;
  EXPECT_FLOAT_EQ(c.w, 1.0f);

  c = a;
  c += b;
  EXPECT_FLOAT_EQ(c.x, 3.0f);
  c -= b;
  EXPECT_FLOAT_EQ(c.x, 1.0f);
  c *= 2.0f;
  EXPECT_FLOAT_EQ(c.z, 6.0f);
  c /= 2.0f;
  EXPECT_FLOAT_EQ(c.z, 3.0f);
}

TEST(MtxlibVector4, NormalizeAndHelpers) {
  vector4 v(0.0f, 0.0f, 3.0f, 4.0f);
  v.normalize();
  EXPECT_NEAR(v.z, 0.6f, kEps);
  EXPECT_NEAR(v.w, 0.8f, kEps);

  vector4 z(0, 0, 0, 0);
  z.normalize(); // zero-length branch
  EXPECT_TRUE(z.isZero());

  vector4 n = Normalized(vector4(2, 0, 0, 0));
  EXPECT_NEAR(n.x, 1.0f, kEps);

  EXPECT_FLOAT_EQ(DotProduct(vector4(1, 2, 3, 4), vector4(5, 6, 7, 8)), 70.0f);

  vector4 a(1, 2, 3, 4), b(5, 6, 7, 8);
  SwapVec(a, b);
  EXPECT_FLOAT_EQ(a.x, 5.0f);
  EXPECT_FLOAT_EQ(b.w, 4.0f);

  EXPECT_TRUE(NearlyEquals(vector4(1, 1, 1, 1), vector4(1, 1, 1, 1.001f), 0.01f));
  EXPECT_FALSE(NearlyEquals(vector4(1, 1, 1, 1), vector4(1, 1, 1, 3), 0.01f));
}

// ===========================================================================
// matrix33
// ===========================================================================

TEST(MtxlibMatrix33, ConstructorsAndIdentity) {
  matrix33 filled(2.0f);
  for (unsigned c = 0; c < 3; c++)
    for (unsigned r = 0; r < 3; r++)
      EXPECT_FLOAT_EQ(filled[c][r], 2.0f);

  matrix33 fromVecs(vector3(1, 2, 3), vector3(4, 5, 6), vector3(7, 8, 9));
  EXPECT_FLOAT_EQ(fromVecs[1][2], 6.0f);
  matrix33 copy(fromVecs);
  EXPECT_EQ(copy, fromVecs);

  matrix33 assigned;
  assigned = fromVecs;
  EXPECT_EQ(assigned, fromVecs);

  matrix33 id = IdentityMatrix33();
  EXPECT_FLOAT_EQ(id[0][0], 1.0f);
  EXPECT_FLOAT_EQ(id[1][1], 1.0f);
  EXPECT_FLOAT_EQ(id[2][2], 1.0f);
  EXPECT_FLOAT_EQ(id[1][0], 0.0f);
}

TEST(MtxlibMatrix33, ArithmeticOps) {
  matrix33 a(1.0f), b(2.0f);
  matrix33 sum = a + b;
  EXPECT_FLOAT_EQ(sum[0][0], 3.0f);
  matrix33 diff = b - a;
  EXPECT_FLOAT_EQ(diff[2][2], 1.0f);
  matrix33 scaled = a * 4.0f;
  EXPECT_FLOAT_EQ(scaled[1][1], 4.0f);
  scaled = 4.0f * a;
  EXPECT_FLOAT_EQ(scaled[1][2], 4.0f);

  matrix33 acc(1.0f);
  acc += b;
  EXPECT_FLOAT_EQ(acc[0][1], 3.0f);
  acc -= b;
  EXPECT_FLOAT_EQ(acc[0][1], 1.0f);
  acc *= 2.0f;
  EXPECT_FLOAT_EQ(acc[0][1], 2.0f);

  EXPECT_TRUE(a != b);
  EXPECT_TRUE(a == matrix33(1.0f));
}

TEST(MtxlibMatrix33, RotateMultiplyAndVectors) {
  const float kPi = 3.14159265358979f;
  matrix33 r90 = RotateRadMatrix33(kPi / 2.0f);
  vector3 rx = r90 * vector3(1, 0, 0);
  EXPECT_NEAR(rx.x, 0.0f, kEps);
  EXPECT_NEAR(rx.y, 1.0f, kEps);
  EXPECT_NEAR(rx.z, 0.0f, kEps);

  // Composition: Rot(a)*Rot(b) == Rot(a+b)
  matrix33 ab = RotateRadMatrix33(0.3f) * RotateRadMatrix33(0.5f);
  matrix33 expect = RotateRadMatrix33(0.8f);
  for (unsigned c = 0; c < 3; c++)
    for (unsigned r = 0; r < 3; r++)
      EXPECT_NEAR(ab[c][r], expect[c][r], kEps);

  // v * m computes DotProduct against columns
  matrix33 m(vector3(1, 2, 3), vector3(4, 5, 6), vector3(7, 8, 9));
  vector3 v(1, 1, 1);
  vector3 vm = v * m;
  EXPECT_FLOAT_EQ(vm.x, 6.0f);
  EXPECT_FLOAT_EQ(vm.y, 15.0f);
  EXPECT_FLOAT_EQ(vm.z, 24.0f);
}

TEST(MtxlibMatrix33, TransposeAndInvert) {
  matrix33 m(vector3(1, 2, 3), vector3(4, 5, 6), vector3(7, 8, 9));
  matrix33 t = TransposeMatrix33(m);
  EXPECT_FLOAT_EQ(t[0][1], 4.0f);
  EXPECT_FLOAT_EQ(t[1][0], 2.0f);
  t.transpose();
  EXPECT_EQ(t, m);

  matrix33 s = ScaleMatrix33(2.0f, 4.0f, 5.0f);
  matrix33 sinv = InvertMatrix33(s);
  EXPECT_NEAR(sinv[0][0], 0.5f, kEps);
  EXPECT_NEAR(sinv[1][1], 0.25f, kEps);
  EXPECT_NEAR(sinv[2][2], 0.2f, kEps);

  // Permutation matrix exercises the pivot-swap path; it is its own inverse.
  matrix33 perm(vector3(0, 1, 0), vector3(1, 0, 0), vector3(0, 0, 1));
  matrix33 pinv = InvertMatrix33(perm);
  matrix33 prod = perm * pinv;
  matrix33 id = IdentityMatrix33();
  for (unsigned c = 0; c < 3; c++)
    for (unsigned r = 0; r < 3; r++)
      EXPECT_NEAR(prod[c][r], id[c][r], kEps);

  // Singular matrix returns identity.
  matrix33 sing(1.0f);
  sing.invert();
  EXPECT_EQ(sing, IdentityMatrix33());
}

TEST(MtxlibMatrix33, TranslateAndScaleFactories) {
  matrix33 tr = TranslateMatrix33(5.0f, -3.0f);
  vector3 p = tr * vector3(1.0f, 2.0f, 1.0f); // homogeneous 2D point
  EXPECT_NEAR(p.x, 6.0f, kEps);
  EXPECT_NEAR(p.y, -1.0f, kEps);
  EXPECT_NEAR(p.z, 1.0f, kEps);

  matrix33 sc = ScaleMatrix33(2.0f, 3.0f); // default z = 1
  vector3 q = sc * vector3(1.0f, 1.0f, 1.0f);
  EXPECT_NEAR(q.x, 2.0f, kEps);
  EXPECT_NEAR(q.y, 3.0f, kEps);
  EXPECT_NEAR(q.z, 1.0f, kEps);
}

// ===========================================================================
// matrix44
// ===========================================================================

TEST(MtxlibMatrix44, ConstructorsAndIdentity) {
  matrix44 filled(3.0f);
  EXPECT_FLOAT_EQ(filled[3][3], 3.0f);

  matrix44 fromVecs(vector4(1, 2, 3, 4), vector4(5, 6, 7, 8), vector4(9, 10, 11, 12),
                    vector4(13, 14, 15, 16));
  EXPECT_FLOAT_EQ(fromVecs[2][1], 10.0f);
  matrix44 copy(fromVecs);
  EXPECT_EQ(copy, fromVecs);
  matrix44 assigned;
  assigned = fromVecs;
  EXPECT_EQ(assigned, fromVecs);

  float arr[16];
  for (int i = 0; i < 16; i++)
    arr[i] = (float)i;
  matrix44 fromArr(arr);
  EXPECT_FLOAT_EQ(fromArr[0][0], 0.0f);
  EXPECT_FLOAT_EQ(fromArr[1][0], 4.0f); // column-major layout
  EXPECT_FLOAT_EQ(fromArr[3][3], 15.0f);

  float out[16];
  fromArr.toArray(out);
  for (int i = 0; i < 16; i++)
    EXPECT_FLOAT_EQ(out[i], arr[i]);

  matrix33 m3(vector3(1, 2, 3), vector3(4, 5, 6), vector3(7, 8, 9));
  matrix44 fromM3(m3);
  EXPECT_FLOAT_EQ(fromM3[1][2], 6.0f);
  EXPECT_FLOAT_EQ(fromM3[3][3], 1.0f);
  matrix44 assignedM3;
  assignedM3 = m3;
  EXPECT_FLOAT_EQ(assignedM3[0][1], 2.0f);
  EXPECT_FLOAT_EQ(assignedM3[3][3], 1.0f);

  matrix44 id = IdentityMatrix44();
  EXPECT_FLOAT_EQ(id[0][0], 1.0f);
  EXPECT_FLOAT_EQ(id[3][3], 1.0f);
  EXPECT_FLOAT_EQ(id[2][0], 0.0f);
}

TEST(MtxlibMatrix44, ArithmeticOps) {
  matrix44 a(1.0f), b(2.0f);
  matrix44 sum = a + b;
  EXPECT_FLOAT_EQ(sum[0][0], 3.0f);
  matrix44 diff = b - a;
  EXPECT_FLOAT_EQ(diff[3][3], 1.0f);
  matrix44 scaled = a * 5.0f;
  EXPECT_FLOAT_EQ(scaled[2][2], 5.0f);
  scaled = 5.0f * a;
  EXPECT_FLOAT_EQ(scaled[1][3], 5.0f);

  matrix44 acc(1.0f);
  acc += b;
  EXPECT_FLOAT_EQ(acc[0][1], 3.0f);
  acc -= b;
  EXPECT_FLOAT_EQ(acc[0][1], 1.0f);
  acc *= 2.0f;
  EXPECT_FLOAT_EQ(acc[0][1], 2.0f);

  EXPECT_TRUE(a != b);
  EXPECT_TRUE(a == matrix44(1.0f));
}

TEST(MtxlibMatrix44, VectorMultiply) {
  matrix44 tr = TranslateMatrix44(1.0f, 2.0f, 3.0f);

  // vector4 with w=1 picks up the translation
  vector4 p = tr * vector4(0.0f, 0.0f, 0.0f, 1.0f);
  EXPECT_NEAR(p.x, 1.0f, kEps);
  EXPECT_NEAR(p.y, 2.0f, kEps);
  EXPECT_NEAR(p.z, 3.0f, kEps);
  EXPECT_NEAR(p.w, 1.0f, kEps);

  // m * vector3 promotes with w=0, so it transforms as a direction
  vector3 d = tr * vector3(1.0f, 0.0f, 0.0f);
  EXPECT_NEAR(d.x, 1.0f, kEps);
  EXPECT_NEAR(d.y, 0.0f, kEps);

  // v * m computes DotProduct against columns
  matrix44 m(vector4(1, 0, 0, 0), vector4(0, 1, 0, 0), vector4(0, 0, 1, 0), vector4(1, 2, 3, 1));
  vector4 vm = vector4(1, 1, 1, 1) * m;
  EXPECT_FLOAT_EQ(vm.x, 1.0f);
  EXPECT_FLOAT_EQ(vm.w, 7.0f);

  vector3 v3m = vector3(1, 1, 1) * m;
  EXPECT_FLOAT_EQ(v3m.x, 1.0f);
  EXPECT_FLOAT_EQ(v3m.z, 1.0f);

  matrix44 prod = tr * TranslateMatrix44(-1.0f, -2.0f, -3.0f);
  expectMat44Near(prod, IdentityMatrix44(), kEps);
}

TEST(MtxlibMatrix44, TransposeAndInvert) {
  matrix44 m(vector4(1, 2, 3, 4), vector4(5, 6, 7, 8), vector4(9, 10, 11, 12),
             vector4(13, 14, 15, 16));
  matrix44 t = TransposeMatrix44(m);
  EXPECT_FLOAT_EQ(t[0][1], 5.0f);
  EXPECT_FLOAT_EQ(t[1][0], 2.0f);
  t.transpose();
  EXPECT_EQ(t, m);

  matrix44 tr = TranslateMatrix44(1.0f, 2.0f, 3.0f);
  matrix44 trInv = InvertMatrix44(tr);
  EXPECT_NEAR(trInv[3][0], -1.0f, kEps);
  EXPECT_NEAR(trInv[3][1], -2.0f, kEps);
  EXPECT_NEAR(trInv[3][2], -3.0f, kEps);

  matrix44 combo =
      TranslateMatrix44(1, -2, 3) * ScaleMatrix44(2, 4, 8) * RotateRadMatrix44('z', 0.4f);
  matrix44 comboInv = InvertMatrix44(combo);
  expectMat44Near(combo * comboInv, IdentityMatrix44(), 1e-4f);

  matrix44 sing(0.0f);
  sing.invert(); // singular -> identity
  EXPECT_EQ(sing, IdentityMatrix44());
}

TEST(MtxlibMatrix44, RotateFactories) {
  const float kPi = 3.14159265358979f;

  vector3 rx = RotateRadMatrix44('x', kPi / 2.0f) * vector3(0, 1, 0);
  EXPECT_NEAR(rx.x, 0.0f, kEps);
  EXPECT_NEAR(rx.y, 0.0f, kEps);
  EXPECT_NEAR(rx.z, 1.0f, kEps);

  vector3 ry = RotateRadMatrix44('y', kPi / 2.0f) * vector3(0, 0, 1);
  EXPECT_NEAR(ry.x, 1.0f, kEps);
  EXPECT_NEAR(ry.y, 0.0f, kEps);
  EXPECT_NEAR(ry.z, 0.0f, kEps);

  vector3 rz = RotateRadMatrix44('z', kPi / 2.0f) * vector3(1, 0, 0);
  EXPECT_NEAR(rz.x, 0.0f, kEps);
  EXPECT_NEAR(rz.y, 1.0f, kEps);

  // Capital axis letters share the same code path
  matrix44 rX = RotateRadMatrix44('X', 0.3f);
  expectMat44Near(rX, RotateRadMatrix44('x', 0.3f), kEps);

  // Arbitrary-axis rotation about z axis matches the char version
  matrix44 axisZ = RotateRadMatrix44(vector3(0, 0, 2), 0.7f); // non-unit axis
  expectMat44Near(axisZ, RotateRadMatrix44('z', 0.7f), kEps);

  // Rotation about (1,1,1) by 2*pi/3 permutes the basis: x -> y
  matrix44 diag = RotateRadMatrix44(vector3(1, 1, 1), 2.0f * kPi / 3.0f);
  vector3 dx = diag * vector3(1, 0, 0);
  EXPECT_NEAR(dx.x, 0.0f, 1e-4f);
  EXPECT_NEAR(dx.y, 1.0f, 1e-4f);
  EXPECT_NEAR(dx.z, 0.0f, 1e-4f);
}

TEST(MtxlibMatrix44, TranslateScaleLookAt) {
  matrix44 tr = TranslateMatrix44(4.0f, 5.0f, 6.0f);
  EXPECT_FLOAT_EQ(tr[3][0], 4.0f);
  EXPECT_FLOAT_EQ(tr[3][1], 5.0f);
  EXPECT_FLOAT_EQ(tr[3][2], 6.0f);

  matrix44 sc = ScaleMatrix44(2.0f, 3.0f, 4.0f, 5.0f);
  EXPECT_FLOAT_EQ(sc[0][0], 2.0f);
  EXPECT_FLOAT_EQ(sc[3][3], 5.0f);

  // NOTE: the header declares LookAtMatrix44(camPos, camUp, target) but the
  // implementation's parameter order is (camPos, target, camUp). We test the
  // implemented behavior: second argument is the target.
  matrix44 look = LookAtMatrix44(vector3(0, 0, 5), vector3(0, 0, 0), vector3(0, 1, 0));
  vector4 origin = look * vector4(0, 0, 0, 1);
  EXPECT_NEAR(origin.x, 0.0f, kEps);
  EXPECT_NEAR(origin.y, 0.0f, kEps);
  EXPECT_NEAR(origin.z, -5.0f, kEps);
  EXPECT_NEAR(origin.w, 1.0f, kEps);
  vector4 right = look * vector4(1, 0, 0, 1);
  EXPECT_NEAR(right.x, 1.0f, kEps);
  EXPECT_NEAR(right.y, 0.0f, kEps);
}

TEST(MtxlibMatrix44, ProjectionFactories) {
  matrix44 fr = FrustumMatrix44(-1.0f, 1.0f, -1.0f, 1.0f, 1.0f, 3.0f);
  EXPECT_NEAR(fr[0][0], 1.0f, kEps);
  EXPECT_NEAR(fr[1][1], 1.0f, kEps);
  EXPECT_NEAR(fr[2][2], -2.0f, kEps);
  EXPECT_NEAR(fr[2][3], -1.0f, kEps);
  EXPECT_NEAR(fr[3][2], -3.0f, kEps);
  EXPECT_NEAR(fr[3][3], 0.0f, kEps);

  matrix44 pr = PerspectiveMatrix44(90.0f, 2.0f, 1.0f, 3.0f);
  EXPECT_NEAR(pr[0][0], 0.5f, 1e-4f); // cot(45deg)/aspect
  EXPECT_NEAR(pr[1][1], 1.0f, 1e-4f);
  EXPECT_NEAR(pr[2][2], -2.0f, kEps);
  EXPECT_NEAR(pr[2][3], -1.0f, kEps);
  EXPECT_NEAR(pr[3][2], -3.0f, kEps);

  // OrthoMatrix44: l=0 r=4 b=0 t=2 n=1 f=5 -> width=4 height=2 depth=4
  matrix44 ortho = OrthoMatrix44(0.0f, 4.0f, 0.0f, 2.0f, 1.0f, 5.0f);
  EXPECT_NEAR(ortho[0][0], 0.5f, kEps);
  EXPECT_NEAR(ortho[0][1], 0.0f, kEps);
  EXPECT_NEAR(ortho[1][1], 1.0f, kEps);
  EXPECT_NEAR(ortho[2][2], -0.5f, kEps);
  EXPECT_NEAR(ortho[3][0], -1.0f, kEps);
  EXPECT_NEAR(ortho[3][2], -1.5f, kEps);
  EXPECT_NEAR(ortho[3][3], 1.0f, kEps);
  // KNOWN BUG (characterized, not fixed): OrthoMatrix44 writes the y
  // translation term -(t+b)/height into ret[1][3] instead of ret[3][1],
  // leaving ret[3][1] uninitialized. We assert the misplaced write and do not
  // read ret[3][1].
  EXPECT_NEAR(ortho[1][3], -1.0f, kEps);

  matrix44 basis = OrthoNormalMatrix44(vector3(1, 0, 0), vector3(0, 1, 0), vector3(0, 0, 1));
  EXPECT_FLOAT_EQ(basis[0][0], 1.0f);
  EXPECT_FLOAT_EQ(basis[1][1], 1.0f);
  EXPECT_FLOAT_EQ(basis[2][2], 1.0f);
  EXPECT_FLOAT_EQ(basis[0][3], 0.0f); // w components zeroed via vector4(vector3)
  EXPECT_FLOAT_EQ(basis[3][3], 1.0f);
}

TEST(MtxlibDebugPrint, AllTypesWriteToFile) {
  FILE *f = tmpfile();
  ASSERT_NE(f, nullptr);
  vector2(1, 2).fprint(f, "v2 ");
  vector3(1, 2, 3).fprint(f, "v3 ");
  vector4(1, 2, 3, 4).fprint(f, "v4 ");
  IdentityMatrix33().fprint(f, "m33 ");
  IdentityMatrix44().fprint(f, "m44 ");
  fflush(f);
  long sz = ftell(f);
  EXPECT_GT(sz, 0);
  rewind(f);
  char head[16] = {0};
  ASSERT_EQ(fread(head, 1, 10, f), 10u);
  EXPECT_EQ(strncmp(head, "v2 vector2", 10), 0);
  fclose(f);
}

// ===========================================================================
// FaceVertSet3D
// ===========================================================================

TEST(FaceVertSet3DTest, CtorWithArraysAndBBox) {
  Point3f verts[3] = {Point3f(0, 0, 0), Point3f(2, 0, 0), Point3f(0, 3, 0)};
  TriId3i tris[1] = {TriId3i(0, 1, 2)};
  Vector3f norms[3] = {Vector3f(0, 0, 1), Vector3f(0, 0, 1), Vector3f(0, 0, 1)};
  FaceVertSet3D fvs(3, 1, verts, tris, norms);
  EXPECT_EQ(fvs.vertCount(), 3);
  EXPECT_EQ(fvs.triCount(), 1);
  BoundingBox box = fvs.getExtent();
  EXPECT_FLOAT_EQ(box.lower[0], 0.0f);
  EXPECT_FLOAT_EQ(box.upper[0], 2.0f);
  EXPECT_FLOAT_EQ(box.upper[1], 3.0f);

  Point3f v0, v1, v2;
  fvs.getTriVerts(0, v0, v1, v2);
  EXPECT_FLOAT_EQ(v1[0], 2.0f);
  TriId3i id = fvs.getTriId(0);
  EXPECT_EQ(id[2], 2u);

  fvs.computeTriNormals();
  Vector3f n;
  fvs.getTriNormal(0, n);
  EXPECT_NEAR(n[2], 1.0f, kEps);
  fvs.flipTriNormals();
  fvs.getTriNormal(0, n);
  EXPECT_NEAR(n[2], -1.0f, kEps);

  BoundingBox custom(-5, -5, -5, 5, 5, 5);
  fvs.setExtent(custom);
  EXPECT_FLOAT_EQ(fvs.getExtent().upper[2], 5.0f);
}

TEST(FaceVertSet3DTest, AddVertsTrisAndUnique) {
  FaceVertSet3D fvs;
  EXPECT_EQ(fvs.vertCount(), 0);
  EXPECT_EQ(fvs.triCount(), 0);

  EXPECT_EQ(fvs.addVert(0, 0, 0), 0);
  EXPECT_EQ(fvs.AddVert(Point3f(1, 0, 0), Vector3f(0, 0, 1)), 1);
  EXPECT_EQ(fvs.addVert(0, 1, 0), 2);
  EXPECT_EQ(fvs.AddTri(TriId3i(0, 1, 2)), 0);
  fvs.buildBBox();
  EXPECT_FLOAT_EQ(fvs.getExtent().upper[0], 1.0f);

  // AddVertUnique: same position twice returns the same id
  int first = fvs.AddVertUnique(Point3f(2, 2, 2), Vector3f(0, 0, 1));
  EXPECT_EQ(first, 3);
  int second = fvs.AddVertUnique(Point3f(2, 2, 2), Vector3f(0, 0, 1));
  EXPECT_EQ(second, 3);
  int third = fvs.AddVertUnique(Point3f(3, 3, 3), Vector3f(0, 0, 1));
  EXPECT_EQ(third, 4);
  EXPECT_EQ(fvs.vertCount(), 5);

  // compact() drops the uniqueness set; a later AddVertUnique rebuilds it and
  // no longer knows about previous vertices.
  fvs.compact();
  int again = fvs.AddVertUnique(Point3f(2, 2, 2), Vector3f(0, 0, 1));
  EXPECT_EQ(again, 5);
}

TEST(FaceVertSet3DTest, CubeNormalsAndComputeNormal) {
  FaceVertSet3D fvs;
  buildCube(fvs);
  EXPECT_EQ(fvs.vertCount(), 8);
  EXPECT_EQ(fvs.triCount(), 12);

  BoundingBox box = fvs.getExtent();
  EXPECT_FLOAT_EQ(box.lower[0], -1.0f);
  EXPECT_FLOAT_EQ(box.upper[2], 1.0f);

  Vector3f n;
  fvs.getTriNormal(0, n); // bottom face -> -Z
  EXPECT_NEAR(n[0], 0.0f, kEps);
  EXPECT_NEAR(n[1], 0.0f, kEps);
  EXPECT_NEAR(n[2], -1.0f, kEps);
  fvs.getTriNormal(2, n); // top face -> +Z
  EXPECT_NEAR(n[2], 1.0f, kEps);
  fvs.getTriNormal(8, n); // left face -> -X
  EXPECT_NEAR(n[0], -1.0f, kEps);

  // ComputeNormal(false) with existing normals is a no-op; force recompute
  // exercises the averaging path; invertNormal flips vertex normals in place.
  fvs.ComputeNormal(false);
  fvs.ComputeNormal(true);
  fvs.invertNormal();
  fvs.ComputeNormal(true);
  SUCCEED();
}

// ===========================================================================
// BufferedIO
// ===========================================================================

TEST(BufferedIOTest, WriteReadAllTypes) {
  std::string p = tmpPath("bio_all_types.bin");
  std::remove(p.c_str());

  char cv[3] = {'a', 'b', 'c'};
  unsigned char ucv[2] = {7, 250};
  short sv[4] = {-5, 6, 300, -1000};
  unsigned short usv[2] = {65000, 12};
  int iv[5] = {1, -2, 100000, -100000, 42};
  long lv[3] = {123456789L, -987654321L, 0L};
  int64 llv[3] = {1234567890123LL, -9876543210LL, 77LL};
  float fv[6] = {0.5f, -1.25f, 3.75f, 0.0f, 1e6f, -1e-6f};
  double dv[4] = {3.14159265358979, -2.71828, 0.0, 1e100};

  {
    BufferedIO w(p.c_str(), DiskIO::WRITE);
    EXPECT_EQ(w.mode(), DiskIO::WRITE);
    ASSERT_TRUE(w.open());
    EXPECT_TRUE(w.open()); // double open is a no-op returning true
    w.put(cv, 3);
    w.put(ucv, 2);
    w.put(sv, 4);
    w.put(usv, 2);
    w.put(iv, 5);
    w.put(lv, 3);
    w.put(llv, 3);
    w.put(fv, 6);
    w.put(dv, 4);
    EXPECT_TRUE(w.close()); // fill=true pads to the 4096 block boundary
  }
  EXPECT_EQ(fileSize(p), 4096L);

  {
    BufferedIO r(p.c_str()); // default READ mode
    ASSERT_TRUE(r.open());
    char rcv[3];
    unsigned char rucv[2];
    short rsv[4];
    unsigned short rusv[2];
    int riv[5];
    long rlv[3];
    int64 rllv[3];
    float rfv[6];
    double rdv[4];
    EXPECT_EQ(r.get(rcv, 3), 3);
    EXPECT_EQ(r.get(rucv, 2), 2);
    EXPECT_EQ(r.get(rsv, 4), 4);
    EXPECT_EQ(r.get(rusv, 2), 2);
    EXPECT_EQ(r.get(riv, 5), 5);
    EXPECT_EQ(r.get(rlv, 3), 3);
    EXPECT_EQ(r.get(rllv, 3), 3);
    EXPECT_EQ(r.get(rfv, 6), 6);
    EXPECT_EQ(r.get(rdv, 4), 4);
    EXPECT_EQ(memcmp(rcv, cv, sizeof cv), 0);
    EXPECT_EQ(memcmp(rucv, ucv, sizeof ucv), 0);
    EXPECT_EQ(memcmp(rsv, sv, sizeof sv), 0);
    EXPECT_EQ(memcmp(rusv, usv, sizeof usv), 0);
    EXPECT_EQ(memcmp(riv, iv, sizeof iv), 0);
    EXPECT_EQ(memcmp(rlv, lv, sizeof lv), 0);
    EXPECT_EQ(memcmp(rllv, llv, sizeof llv), 0);
    EXPECT_EQ(memcmp(rfv, fv, sizeof fv), 0);
    EXPECT_EQ(memcmp(rdv, dv, sizeof dv), 0);
    EXPECT_FALSE(r.eof());
    EXPECT_TRUE(r.close());
  }
  std::remove(p.c_str());
}

TEST(BufferedIOTest, MultiBlockAndEofHandling) {
  std::string p = tmpPath("bio_multiblock.bin");
  std::remove(p.c_str());

  const int N = 3000; // 24000 bytes: spans multiple 4096-byte disk blocks
  std::vector<double> big(N);
  for (int i = 0; i < N; i++)
    big[i] = i * 0.25;
  {
    BufferedIO w(p.c_str(), DiskIO::WRITE);
    ASSERT_TRUE(w.open());
    w.put(big.data(), N); // exercises putraw's flush loop
    EXPECT_TRUE(w.close(false));
  }
  EXPECT_EQ(fileSize(p), (long)(N * sizeof(double)));

  {
    BufferedIO r(p.c_str(), DiskIO::READ, 2); // 8192-byte buffer
    ASSERT_TRUE(r.open());
    std::vector<double> got(N, 0.0);
    EXPECT_EQ(r.get(got.data(), N), N);
    EXPECT_DOUBLE_EQ(got[0], 0.0);
    EXPECT_DOUBLE_EQ(got[1024], 256.0);
    EXPECT_DOUBLE_EQ(got[N - 1], (N - 1) * 0.25);
    double extra[4];
    EXPECT_EQ(r.get(extra, 4), 0); // fully consumed
    EXPECT_TRUE(r.eof());
    EXPECT_TRUE(r.close());
  }
  std::remove(p.c_str());
}

TEST(BufferedIOTest, PartialReadAtEof) {
  std::string p = tmpPath("bio_partial.bin");
  std::remove(p.c_str());
  {
    BufferedIO w(p.c_str(), DiskIO::WRITE);
    ASSERT_TRUE(w.open());
    char ten[10] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
    w.put(ten, 10);
    EXPECT_TRUE(w.close(false));
  }
  EXPECT_EQ(fileSize(p), 10L);

  BufferedIO r(p.c_str());
  ASSERT_TRUE(r.open());
  char buf[20];
  EXPECT_EQ(r.get(buf, 20), 10); // returns what is available
  EXPECT_EQ(buf[9], 9);
  EXPECT_TRUE(r.eof());
  EXPECT_EQ(r.get(buf, 5), 0);
  EXPECT_TRUE(r.close());
  std::remove(p.c_str());
}

TEST(BufferedIOTest, SeekReadAndWriteModes) {
  std::string p = tmpPath("bio_seek.bin");
  std::remove(p.c_str());
  const int SZ = 8192;
  {
    std::vector<char> pattern(SZ);
    for (int i = 0; i < SZ; i++)
      pattern[i] = (char)(i % 251);
    BufferedIO w(p.c_str(), DiskIO::WRITE);
    ASSERT_TRUE(w.open());
    w.put(pattern.data(), SZ);
    EXPECT_TRUE(w.close(false));
  }
  {
    BufferedIO r(p.c_str());
    ASSERT_TRUE(r.open());
    char b[16];
    EXPECT_EQ(r.get(b, 16), 16);
    // FROM_HERE within the in-memory buffer
    EXPECT_TRUE(r.seek(100, DiskIO::FROM_HERE));
    EXPECT_EQ(r.get(b, 1), 1);
    EXPECT_EQ(b[0], (char)(116 % 251));
    // negative FROM_HERE still within the buffer
    EXPECT_TRUE(r.seek(-3, DiskIO::FROM_HERE));
    EXPECT_EQ(r.get(b, 1), 1);
    EXPECT_EQ(b[0], (char)(114 % 251));
    // FROM_HERE beyond the buffer forces an fseek
    EXPECT_TRUE(r.seek(6000, DiskIO::FROM_HERE));
    EXPECT_EQ(r.get(b, 1), 1);
    EXPECT_EQ(b[0], (char)((115 + 6000) % 251));
    // FROM_START and FROM_END
    EXPECT_TRUE(r.seek(5, DiskIO::FROM_START));
    EXPECT_EQ(r.get(b, 1), 1);
    EXPECT_EQ(b[0], (char)(5 % 251));
    EXPECT_TRUE(r.seek(-10, DiskIO::FROM_END));
    EXPECT_EQ(r.get(b, 1), 1);
    EXPECT_EQ(b[0], (char)((SZ - 10) % 251));
    EXPECT_TRUE(r.close());
  }
  std::remove(p.c_str());

  // WRITE-mode seeks flush pending data and reposition the file pointer.
  std::string q = tmpPath("bio_seek_write.bin");
  std::remove(q.c_str());
  {
    BufferedIO w(q.c_str(), DiskIO::WRITE);
    ASSERT_TRUE(w.open());
    char aaaa[4] = {'A', 'A', 'A', 'A'};
    w.put(aaaa, 4);
    EXPECT_TRUE(w.seek(0, DiskIO::FROM_START));
    char bb[2] = {'B', 'B'};
    w.put(bb, 2);
    EXPECT_TRUE(w.seek(1, DiskIO::FROM_HERE));
    char d = 'D';
    w.put(&d, 1);
    EXPECT_TRUE(w.seek(0, DiskIO::FROM_END));
    char e = 'E';
    w.put(&e, 1);
    EXPECT_TRUE(w.close(false));
  }
  {
    FILE *f = fopen(q.c_str(), "rb");
    ASSERT_NE(f, nullptr);
    char content[8] = {0};
    size_t n = fread(content, 1, 8, f);
    fclose(f);
    ASSERT_EQ(n, 5u);
    EXPECT_EQ(memcmp(content, "BBADE", 5), 0);
  }
  std::remove(q.c_str());
}

TEST(BufferedIOTest, FilePointerCtorAndSetMode) {
  FILE *fp = tmpfile();
  ASSERT_NE(fp, nullptr);
  BufferedIO io(fp, DiskIO::WRITE);
  int vals[4] = {10, 20, 30, 40};
  io.put(vals, 4);
  EXPECT_TRUE(io.setMode(DiskIO::READ)); // flush and switch to reading
  EXPECT_TRUE(io.setMode(DiskIO::READ)); // same mode returns immediately
  EXPECT_EQ(io.mode(), DiskIO::READ);
  EXPECT_TRUE(io.seek(0, DiskIO::FROM_START));
  int back[4] = {0, 0, 0, 0};
  EXPECT_EQ(io.get(back, 4), 4);
  EXPECT_EQ(memcmp(back, vals, sizeof vals), 0);
  EXPECT_TRUE(io.setMode(DiskIO::WRITE)); // back to writing (fseek sync path)
  int more = 50;
  io.put(&more, 1);
  EXPECT_TRUE(io.close(false)); // also fcloses the tmpfile
}

TEST(BufferedIOTest, ReopenFlushAndOpenFailure) {
  std::string pa = tmpPath("bio_reopen_a.bin");
  std::string pb = tmpPath("bio_reopen_b.bin");
  std::remove(pa.c_str());
  std::remove(pb.c_str());

  BufferedIO w(pa.c_str(), DiskIO::WRITE);
  ASSERT_TRUE(w.open());
  char abc[3] = {'a', 'b', 'c'};
  w.put(abc, 3);
  // reopen(fname): flush (padded) to pa, then switch to pb
  EXPECT_TRUE(w.reopen(pb.c_str()));
  w.put(abc, 3);
  EXPECT_TRUE(w.close());
  EXPECT_EQ(fileSize(pa), 4096L);
  EXPECT_EQ(fileSize(pb), 4096L);

  // reopen() with a closed WRITE stream appends
  EXPECT_TRUE(w.reopen());
  w.put(abc, 3);
  EXPECT_TRUE(w.close(false));
  EXPECT_EQ(fileSize(pb), 4099L);
  // reopen() while already open returns true immediately
  EXPECT_TRUE(w.reopen());
  EXPECT_TRUE(w.reopen());
  EXPECT_TRUE(w.close(false));

  {
    // READ-mode flush skips to the next disk-block boundary
    BufferedIO r(pb.c_str());
    ASSERT_TRUE(r.open());
    char b[4];
    EXPECT_EQ(r.get(b, 2), 2);
    r.flush();
    EXPECT_EQ(r.get(b, 3), 3); // now reading the appended bytes at offset 4096
    EXPECT_EQ(memcmp(b, "abc", 3), 0);
    // READ-mode reopen() of a closed stream. NOTE (library quirk): reopen()
    // does not reset the buffered EOF state, so a read right after reopen
    // returns 0; a seek clears the stale state.
    EXPECT_TRUE(r.close());
    EXPECT_TRUE(r.reopen());
    EXPECT_TRUE(r.seek(0, DiskIO::FROM_START));
    EXPECT_EQ(r.get(b, 1), 1);
    EXPECT_EQ(b[0], 'a');
    EXPECT_TRUE(r.close());
  }

  // open() failure paths
  BufferedIO badRead("/nonexistent_dir_mtxlib_test/x.bin");
  EXPECT_FALSE(badRead.open());
  BufferedIO badWrite("/nonexistent_dir_mtxlib_test/y.bin", DiskIO::WRITE);
  EXPECT_FALSE(badWrite.open());

  std::remove(pa.c_str());
  std::remove(pb.c_str());
}

// ===========================================================================
// Geom3DParser
// ===========================================================================

TEST(Geom3DParserTest, ParseValidRawFile) {
  std::string p = tmpPath("geom3d_ok.raw");
  {
    FILE *f = fopen(p.c_str(), "w");
    ASSERT_NE(f, nullptr);
    fprintf(f, "3 1\n0 0 0\n2 0 0\n0 2 0\n0 1 2\n");
    fclose(f);
  }
  FaceVertSet3D fvs;
  Geom3DParser parser;
  EXPECT_TRUE(parser.ParseRawFile(fvs, p.c_str()));
  EXPECT_EQ(fvs.vertCount(), 3);
  EXPECT_EQ(fvs.triCount(), 1);
  BoundingBox box = fvs.getExtent();
  EXPECT_FLOAT_EQ(box.upper[0], 2.0f);
  EXPECT_FLOAT_EQ(box.upper[1], 2.0f);
  Vector3f n;
  fvs.getTriNormal(0, n);
  EXPECT_NEAR(n[2], 1.0f, kEps);
  std::remove(p.c_str());
}

TEST(Geom3DParserTest, ParseFailurePaths) {
  Geom3DParser parser;
  FaceVertSet3D fvs;
  EXPECT_FALSE(parser.ParseRawFile(fvs, "/nonexistent_dir_mtxlib_test/g.raw"));

  // Empty file: header read fails
  std::string pEmpty = tmpPath("geom3d_empty.raw");
  {
    FILE *f = fopen(pEmpty.c_str(), "w");
    ASSERT_NE(f, nullptr);
    fclose(f);
  }
  FaceVertSet3D fvs2;
  EXPECT_FALSE(parser.ParseRawFile(fvs2, pEmpty.c_str()));
  std::remove(pEmpty.c_str());

  // Header promises vertices that never appear
  std::string pNoVerts = tmpPath("geom3d_noverts.raw");
  {
    FILE *f = fopen(pNoVerts.c_str(), "w");
    ASSERT_NE(f, nullptr);
    fprintf(f, "5 2\n");
    fclose(f);
  }
  FaceVertSet3D fvs3;
  EXPECT_FALSE(parser.ParseRawFile(fvs3, pNoVerts.c_str()));
  std::remove(pNoVerts.c_str());

  // Vertices present but triangles missing
  std::string pNoTris = tmpPath("geom3d_notris.raw");
  {
    FILE *f = fopen(pNoTris.c_str(), "w");
    ASSERT_NE(f, nullptr);
    fprintf(f, "3 1\n0 0 0\n1 0 0\n0 1 0\n");
    fclose(f);
  }
  FaceVertSet3D fvs4;
  EXPECT_FALSE(parser.ParseRawFile(fvs4, pNoTris.c_str()));
  std::remove(pNoTris.c_str());
}

// ===========================================================================
// RawivParser + Reg3Data
// ===========================================================================

TEST(RawivParserTest, WriteThenParseRestoresData) {
  int dim[3] = {3, 4, 5};
  const int nverts = dim[0] * dim[1] * dim[2];
  std::vector<float> values(nverts);
  for (int i = 0; i < nverts; i++)
    values[i] = i * 0.5f;
  Reg3Data<float> src(dim, values.data());
  float orig[3] = {0.5f, 1.0f, 2.0f};
  float span[3] = {0.25f, 0.5f, 1.0f};
  src.setOrig(orig);
  src.setSpan(span);

  EXPECT_EQ(src.getNVerts(), nverts);
  EXPECT_EQ(src.getNCells(), 2 * 3 * 4);
  EXPECT_FLOAT_EQ(src.getFuncMin(), 0.0f);
  EXPECT_FLOAT_EQ(src.getFuncMax(), (nverts - 1) * 0.5f);

  std::string p = tmpPath("reg3.rawiv");
  std::remove(p.c_str());
  RawivParser parser;
  ASSERT_TRUE(parser.write(src, p.c_str()));
  EXPECT_GT(fileSize(p), 0L);

  Reg3Data<float> dst;
  ASSERT_TRUE(parser.parse(&dst, p.c_str()));
  int gotDim[3];
  dst.getDim(gotDim);
  EXPECT_EQ(gotDim[0], 3);
  EXPECT_EQ(gotDim[1], 4);
  EXPECT_EQ(gotDim[2], 5);
  float gotOrig[3], gotSpan[3];
  dst.getOrig(gotOrig);
  dst.getSpan(gotSpan);
  for (int i = 0; i < 3; i++) {
    EXPECT_FLOAT_EQ(gotOrig[i], orig[i]);
    EXPECT_FLOAT_EQ(gotSpan[i], span[i]);
  }
  for (int i = 0; i < nverts; i++)
    EXPECT_FLOAT_EQ(dst.getValue(i), values[i]) << "voxel " << i;
  EXPECT_FLOAT_EQ(dst.getFuncMin(), 0.0f);
  EXPECT_FLOAT_EQ(dst.getFuncMax(), (nverts - 1) * 0.5f);
  std::remove(p.c_str());
}

TEST(RawivParserTest, ParseRejectsBadInputs) {
  RawivParser parser;
  Reg3Data<float> data;
  EXPECT_FALSE(parser.parse(&data, "not_a_rawiv.txt")); // wrong extension
  EXPECT_FALSE(parser.parse(&data, tmpPath("missing_file.rawiv").c_str()));
}

// ===========================================================================
// DistanceTransform
// ===========================================================================

class DistanceTransformTest : public ::testing::Test {
protected:
  void SetUp() override { buildCube(cube); }

  cvc::app ctx;
  FaceVertSet3D cube;
};

TEST_F(DistanceTransformTest, CtorFromDimSetsUpGrid) {
  int dim[3] = {16, 16, 16};
  DistanceTransform dt(ctx, cube, dim);
  const Reg3Data<float> &grid = dt.getReg3Data();
  int gotDim[3];
  grid.getDim(gotDim);
  EXPECT_EQ(gotDim[0], 16);
  EXPECT_EQ(gotDim[1], 16);
  EXPECT_EQ(gotDim[2], 16);
  float orig[3], span[3];
  grid.getOrig(orig);
  grid.getSpan(span);
  // bbox [-1,1]^3, max_ext=2, scale 2 -> orig=-2, span=4/15
  for (int i = 0; i < 3; i++) {
    EXPECT_NEAR(orig[i], -2.0f, kEps);
    EXPECT_NEAR(span[i], 4.0f / 15.0f, kEps);
  }
  // After construction, near-surface verts are 0 and the rest MAX_FLOAT.
  float fmin, fmax;
  grid.getFuncMinMax(fmin, fmax);
  EXPECT_FLOAT_EQ(fmin, 0.0f);
  EXPECT_FLOAT_EQ(fmax, DistanceTransform::MAX_FLOAT);
}

TEST_F(DistanceTransformTest, CtorFromReg3Data) {
  int dim[3] = {16, 16, 16};
  std::vector<float> zeros(16 * 16 * 16, 0.0f);
  Reg3Data<float> reg(dim, zeros.data());
  float orig[3] = {-2.0f, -2.0f, -2.0f};
  float span[3] = {4.0f / 15.0f, 4.0f / 15.0f, 4.0f / 15.0f};
  reg.setOrig(orig);
  reg.setSpan(span);

  DistanceTransform dt(ctx, cube, reg); // copies the Reg3Data
  const Reg3Data<float> &grid = dt.getReg3Data();
  int gotDim[3];
  grid.getDim(gotDim);
  EXPECT_EQ(gotDim[0], 16);
  float gotSpan[3];
  grid.getSpan(gotSpan);
  EXPECT_NEAR(gotSpan[1], 4.0f / 15.0f, kEps);
}

TEST_F(DistanceTransformTest, GridTransformSignsAndRawivRoundtrip) {
  int dim[3] = {16, 16, 16};
  DistanceTransform dt(ctx, cube, dim);
  dt.transform();
  const Reg3Data<float> &grid = dt.getReg3Data();

  // Grid point (8,8,8) is at (0.133,0.133,0.133): inside the cube -> negative,
  // roughly -(1 - 0.133).
  float inside = grid.getValue(8, 8, 8);
  EXPECT_LT(inside, 0.0f);
  EXPECT_NEAR(inside, -0.8667f, 0.3f);

  // Grid corner (0,0,0) is at (-2,-2,-2): outside -> positive, roughly
  // sqrt(3) from the cube corner.
  float outside = grid.getValue(0, 0, 0);
  EXPECT_GT(outside, 0.0f);
  EXPECT_NEAR(outside, 1.7321f, 0.4f);

  // Every value should be finite and well below MAX_FLOAT after transform.
  float fmin, fmax;
  grid.getFuncMinMax(fmin, fmax);
  EXPECT_LT(fmin, 0.0f);
  EXPECT_GT(fmax, 0.0f);
  EXPECT_LT(fmax, 10.0f);

  // writeRawiv + parse roundtrip through RawivParser/BufferedIO.
  std::string p = tmpPath("dt_out.rawiv");
  std::remove(p.c_str());
  dt.writeRawiv(p.c_str());
  Reg3Data<float> back;
  RawivParser parser;
  ASSERT_TRUE(parser.parse(&back, p.c_str()));
  int gotDim[3];
  back.getDim(gotDim);
  EXPECT_EQ(gotDim[0], 16);
  EXPECT_FLOAT_EQ(back.getValue(8, 8, 8), inside);
  EXPECT_FLOAT_EQ(back.getValue(0, 0, 0), outside);
  std::remove(p.c_str());
}

TEST_F(DistanceTransformTest, DistanceToEdgeBranches) {
  int dim[3] = {16, 16, 16};
  DistanceTransform dt(ctx, cube, dim);

  Point3f v1(0, 0, 0), v2(2, 0, 0), ne;
  // Before v1
  double d = dt.distance2Edge(Point3f(-1, 1, 0), v1, v2, ne);
  EXPECT_NEAR(d, std::sqrt(2.0), 1e-5);
  EXPECT_FLOAT_EQ(ne[0], 0.0f);
  // Past v2
  d = dt.distance2Edge(Point3f(3, 1, 0), v1, v2, ne);
  EXPECT_NEAR(d, std::sqrt(2.0), 1e-5);
  EXPECT_FLOAT_EQ(ne[0], 2.0f);
  // Interior projection
  d = dt.distance2Edge(Point3f(1, 1, 0), v1, v2, ne);
  EXPECT_NEAR(d, 1.0, 1e-5);
  EXPECT_FLOAT_EQ(ne[0], 1.0f);
  EXPECT_FLOAT_EQ(ne[1], 0.0f);
}

TEST_F(DistanceTransformTest, PointInTriangleProjections) {
  int dim[3] = {16, 16, 16};
  DistanceTransform dt(ctx, cube, dim);

  // Left-face triangle 8 = {0,4,7}, normal (-1,0,0): projects to the y-z plane
  Point3f v0, v1, v2;
  Vector3f n;
  cube.getTriVerts(8, v0, v1, v2);
  cube.getTriNormal(8, n);
  EXPECT_TRUE(dt.pointInTriangle(Point3f(-1, 0, 0.5f), v0, v1, v2, n));
  EXPECT_FALSE(dt.pointInTriangle(Point3f(-1, 0.5f, 0), v0, v1, v2, n)); // alpha < 0
  EXPECT_FALSE(dt.pointInTriangle(Point3f(-1, -2, 0), v0, v1, v2, n));   // beta < 0
  EXPECT_FALSE(dt.pointInTriangle(Point3f(-1, -1, 2), v0, v1, v2, n));   // alpha+beta > 1

  // Front-face triangle 4 = {0,1,5}, normal (0,-1,0): projects to x-z
  cube.getTriVerts(4, v0, v1, v2);
  cube.getTriNormal(4, n);
  EXPECT_TRUE(dt.pointInTriangle(Point3f(0, -1, -0.5f), v0, v1, v2, n));

  // Bottom-face triangle 0 = {0,2,1}, normal (0,0,-1): projects to x-y
  cube.getTriVerts(0, v0, v1, v2);
  cube.getTriNormal(0, n);
  EXPECT_TRUE(dt.pointInTriangle(Point3f(0.5f, 0.2f, -1), v0, v1, v2, n));
}

TEST_F(DistanceTransformTest, DistanceToTriangleAndRayIntersect) {
  int dim[3] = {16, 16, 16};
  DistanceTransform dt(ctx, cube, dim);

  // Plane-projection branch: point straight out from the left face interior
  Point3f nearest;
  double d = dt.distance2Triangle(Point3f(-2, 0, 0.5f), 8, nearest);
  EXPECT_NEAR(d, 1.0, 1e-5);
  EXPECT_NEAR(nearest[0], -1.0f, kEps);

  // Corner branch: closest point is vertex (-1,-1,-1)
  d = dt.distance2Triangle(Point3f(-2, -2, -2), 8, nearest);
  EXPECT_NEAR(d, std::sqrt(3.0), 1e-5);

  // Edge branch: projection falls outside, closest point on the diagonal edge
  d = dt.distance2Triangle(Point3f(-2, 0.5f, 0.2f), 8, nearest);
  EXPECT_NEAR(d, std::sqrt(1.045), 1e-4);

  // Ray crossing the left face at t=0.5
  double t = dt.rayTriangleIntersection(8, Point3f(-2, 0, 0.5f), Point3f(0, 0, 0.5f));
  EXPECT_NEAR(t, 0.5, 1e-5);
  // Ray on one side of the plane -> no intersection
  t = dt.rayTriangleIntersection(8, Point3f(-3, 0, 0.5f), Point3f(-2, 0, 0.5f));
  EXPECT_FLOAT_EQ((float)t, DistanceTransform::MAX_FLOAT);
  // Crossing the plane outside the triangle -> no intersection
  t = dt.rayTriangleIntersection(8, Point3f(-2, 0.5f, 0), Point3f(0, 0.5f, 0));
  EXPECT_FLOAT_EQ((float)t, DistanceTransform::MAX_FLOAT);
  // Begin exactly on the triangle -> 0
  t = dt.rayTriangleIntersection(8, Point3f(-1, 0, 0.5f), Point3f(1, 0, 0.5f));
  EXPECT_NEAR(t, 0.0, 1e-6);
  // End exactly on the triangle -> 1
  t = dt.rayTriangleIntersection(8, Point3f(-2, 0, 0.5f), Point3f(-1, 0, 0.5f));
  EXPECT_NEAR(t, 1.0, 1e-6);
}

TEST_F(DistanceTransformTest, CellPredicatesAndNearDistance) {
  int dim[3] = {16, 16, 16};
  DistanceTransform dt(ctx, cube, dim);

  Point3f v0, v1, v2;
  Vector3f n;
  cube.getTriVerts(8, v0, v1, v2);
  cube.getTriNormal(8, n);
  // Cell (3,8,8) spans x in [-1.2,-0.933]: the left-face plane x=-1 crosses it
  EXPECT_TRUE(dt.intersectCell(v0, n, 3, 8, 8));
  // Cell (8,8,8) is strictly inside the cube; no plane crossing
  EXPECT_FALSE(dt.intersectCell(v0, n, 8, 8, 8));

  EXPECT_TRUE(dt.nearSurface(4, 8, 8));
  EXPECT_FALSE(dt.nearSurface(1, 1, 1));

  // Triangle/cube overlap tests
  EXPECT_TRUE(
      dt.TriangleCubeIntersection(8, Point3f(-1.2f, -0.2f, -0.2f), Point3f(-0.8f, 0.2f, 0.2f)));
  EXPECT_FALSE(dt.TriangleCubeIntersection(8, Point3f(5, 5, 5), Point3f(6, 6, 6)));

  // computeNearDistance at a grid vertex just inside the left face:
  // pos x = -0.933..., distance to the x=-1 face is ~0.0667, negative inside.
  ASSERT_TRUE(dt.nearSurface(4, 8, 8));
  Point3f closest;
  float dNear = dt.computeNearDistance(4, 8, 8, closest);
  EXPECT_NEAR(dNear, -0.0667f, 5e-3f);
  EXPECT_NEAR(closest[0], -1.0f, 1e-4f);
}

TEST_F(DistanceTransformTest, InsideOutsideClassification) {
  int dim[3] = {16, 16, 16};
  DistanceTransform dt(ctx, cube, dim);

  dynamic_array<int> bottomPair;
  bottomPair.insert(0);
  bottomPair.insert(1);

  // Point below the bottom face is outside (+1)
  EXPECT_EQ(dt.inOrOut(bottomPair, Point3f(0, 0, -3), Point3f(0, 0, -1)), 1);
  // Cube center is inside (-1)
  EXPECT_EQ(dt.inOrOut(bottomPair, Point3f(0, 0, 0), Point3f(0, 0, -1)), -1);
}

TEST_F(DistanceTransformTest, NearestPlaneSelection) {
  int dim[3] = {16, 16, 16};
  DistanceTransform dt(ctx, cube, dim);

  // Coplanar pair (both bottom-face triangles): the first candidate wins.
  dynamic_array<int> coplanar;
  coplanar.insert(0);
  coplanar.insert(1);
  EXPECT_EQ(dt.nearestPlane(coplanar, Point3f(0.5f, -0.5f, -2.0f)), 0);

  // Non-coplanar pair (bottom tri 0 and front tri 4): for a point far in -y,
  // the front face plane is the nearest -> index 1.
  dynamic_array<int> ridge;
  ridge.insert(0);
  ridge.insert(4);
  EXPECT_EQ(dt.nearestPlane(ridge, Point3f(0, -3, 0)), 1);

  // Degenerate query point exactly on both planes falls through to the
  // warning path and returns 0.
  EXPECT_EQ(dt.nearestPlane(coplanar, Point3f(0.5f, -0.5f, -1.0f)), 0);
}

TEST_F(DistanceTransformTest, Transform1DEnvelope) {
  int dim[3] = {16, 16, 16};
  DistanceTransform dt(ctx, cube, dim);
  const float MAXF = DistanceTransform::MAX_FLOAT;

  {
    // Single source at index 0: squared distances grow quadratically.
    float f[5] = {0, MAXF, MAXF, MAXF, MAXF};
    float d[5] = {0, 0, 0, 0, 0};
    int parent[5] = {0, 1, 2, 3, 4};
    dt.transform1D(5, f, d, parent, 1.0f);
    EXPECT_FLOAT_EQ(d[0], 0.0f);
    EXPECT_FLOAT_EQ(d[1], 1.0f);
    EXPECT_FLOAT_EQ(d[2], 4.0f);
    EXPECT_FLOAT_EQ(d[3], 9.0f);
    EXPECT_FLOAT_EQ(d[4], 16.0f);
    for (int i = 0; i < 5; i++)
      EXPECT_EQ(parent[i], 0);
  }
  {
    // Source in the middle; span scales squared distances by span^2.
    float f[5] = {MAXF, MAXF, 0, MAXF, MAXF};
    float d[5] = {0, 0, 0, 0, 0};
    int parent[5] = {0, 1, 2, 3, 4};
    dt.transform1D(5, f, d, parent, 2.0f);
    EXPECT_FLOAT_EQ(d[0], 16.0f);
    EXPECT_FLOAT_EQ(d[1], 4.0f);
    EXPECT_FLOAT_EQ(d[2], 0.0f);
    EXPECT_FLOAT_EQ(d[3], 4.0f);
    EXPECT_FLOAT_EQ(d[4], 16.0f);
    for (int i = 0; i < 5; i++)
      EXPECT_EQ(parent[i], 2);
  }
  {
    // All-MAX input: early return leaves d as the copied input.
    float f[4] = {MAXF, MAXF, MAXF, MAXF};
    float d[4] = {0, 0, 0, 0};
    int parent[4] = {0, 1, 2, 3};
    dt.transform1D(4, f, d, parent, 1.0f);
    for (int i = 0; i < 4; i++)
      EXPECT_FLOAT_EQ(d[i], MAXF);
  }
  {
    // Two competing sources: nearest one wins per position.
    float f[6] = {0, MAXF, MAXF, MAXF, MAXF, 0};
    float d[6];
    int parent[6] = {10, 11, 12, 13, 14, 15};
    dt.transform1D(6, f, d, parent, 1.0f);
    EXPECT_FLOAT_EQ(d[0], 0.0f);
    EXPECT_FLOAT_EQ(d[1], 1.0f);
    EXPECT_FLOAT_EQ(d[2], 4.0f);
    EXPECT_FLOAT_EQ(d[3], 4.0f);
    EXPECT_FLOAT_EQ(d[4], 1.0f);
    EXPECT_FLOAT_EQ(d[5], 0.0f);
    EXPECT_EQ(parent[0], 10);
    EXPECT_EQ(parent[5], 15);
  }
}
