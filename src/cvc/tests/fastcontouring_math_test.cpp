/*
  Unit tests for the FastContouring math helpers and contour containers:
    - Tuple, Vector, Ray, Quaternion, Matrix (Quaternion.cpp, Ray.cpp,
      Vector.cpp, Matrix.cpp, Tuple.cpp)
    - ContourGeometry (tri/quad buffer management + addToGeometry)
    - MarchingCubesBuffers
    - ContourExtractor (FastContouring.cpp marching cubes driver)

  NOTE: FastContouring.h does `#include <VolMagickCompat.h>`, but that header
  lives in src/cvc/geometry/cvc-mesher/ which is a PRIVATE include dir of the
  cvc target and is NOT on this test target's include path (only the
  FastContouring/ subdir is).  We therefore include the compat header by a
  quoted relative path and reproduce the tiny FastContouring.h declarations
  (TriSurf + ContourExtractor) verbatim below instead of including it.
*/

#include <cmath>
#include <cvc/core/app.h>
#include <cvc/core/types.h>
#include <cvc/volume/volume.h>
#include <gtest/gtest.h>
#include <memory>
#include <vector>

// Private-include-dir workaround (see file comment above).
#include "../geometry/cvc-mesher/VolMagickCompat.h"
#include "MarchingCubesBuffers.h"
#include "Matrix.h"
#include "Quaternion.h"
#include "Ray.h"
#include "Tuple.h"
#include "Vector.h"

// --- Begin verbatim reproduction of FastContouring.h declarations ---------
namespace FastContouring {
// dead simple contour surface description
struct TriSurf {
  std::vector<double> verts;
  std::vector<double> normals;
  std::vector<double> colors;
  std::vector<unsigned int> tris;
};

class ContourExtractor {
public:
  ContourExtractor(cvc::app &ctx) : m_Data(ctx) {}
  ContourExtractor(const ContourExtractor &copy)
      : m_Data(copy.m_Data), m_Buffers(copy.m_Buffers), m_SaveMatrix(copy.m_SaveMatrix) {}
  ~ContourExtractor() {}

  ContourExtractor &operator=(const ContourExtractor &copy) {
    m_Data = copy.m_Data;
    m_Buffers = copy.m_Buffers;
    m_SaveMatrix = copy.m_SaveMatrix;
    return *this;
  }

  void setVolume(const VolMagick::Volume &vol);
  const VolMagick::Volume &getVolume() const { return m_Data; }

  TriSurf extractContour(double isovalue, double R = 1.0, double G = 1.0, double B = 1.0);

private:
  void classifyVertices(unsigned int k, unsigned int *cacheMemory, float isovalue) const;
  void getNormal(unsigned int i, unsigned int j, unsigned int k, float &nx, float &ny,
                 float &nz) const;
  unsigned int determineCase(unsigned int *offsetTable, unsigned int index) const;

  VolMagick::Volume m_Data;
  // buffers used to speed up marching cubes
  MarchingCubesBuffers m_Buffers;
  Matrix m_SaveMatrix;
};
} // namespace FastContouring
// --- End verbatim reproduction ---------------------------------------------

#include "ContourGeometry.h"

namespace FC = FastContouring;

namespace {

const float kPi = 3.14159265358979323846f;

void expectQuatNear(const FC::Quaternion &a, const FC::Quaternion &b, float tol) {
  for (unsigned int i = 0; i < 4; ++i)
    EXPECT_NEAR(a[i], b[i], tol) << "component " << i;
}

void expectVecNear(const FC::Vector &a, float x, float y, float z, float tol) {
  EXPECT_NEAR(a[0], x, tol);
  EXPECT_NEAR(a[1], y, tol);
  EXPECT_NEAR(a[2], z, tol);
}

// Build an implicit-sphere scalar volume: f(x,y,z) = r - distance_to_center.
cvc::volume makeSphereVolume(cvc::app &ctx, unsigned int n, double radius) {
  cvc::bounding_box bbox(-1.0, -1.0, -1.0, 1.0, 1.0, 1.0);
  cvc::volume v(ctx, cvc::dimension(n, n, n), cvc::Float, bbox);
  const double dx = 2.0 / double(n - 1);
  for (unsigned int k = 0; k < n; ++k)
    for (unsigned int j = 0; j < n; ++j)
      for (unsigned int i = 0; i < n; ++i) {
        double x = -1.0 + i * dx;
        double y = -1.0 + j * dx;
        double z = -1.0 + k * dx;
        double d = std::sqrt(x * x + y * y + z * z);
        v(i, j, k, radius - d);
      }
  return v;
}

// Linear ramp along X in a [0,1]^3 box with arbitrary (non-cubic) dims.
cvc::volume makeRampVolume(cvc::app &ctx, unsigned int nx, unsigned int ny, unsigned int nz) {
  cvc::bounding_box bbox(0.0, 0.0, 0.0, 1.0, 1.0, 1.0);
  cvc::volume v(ctx, cvc::dimension(nx, ny, nz), cvc::Float, bbox);
  for (unsigned int k = 0; k < nz; ++k)
    for (unsigned int j = 0; j < ny; ++j)
      for (unsigned int i = 0; i < nx; ++i)
        v(i, j, k, double(i) / double(nx - 1));
  return v;
}

} // namespace

// ===========================================================================
// Tuple
// ===========================================================================

TEST(FCTupleMath, ConstructorsAndIndexing) {
  FC::Tuple def;
  EXPECT_FLOAT_EQ(def[0], 0.0f);
  EXPECT_FLOAT_EQ(def[1], 0.0f);
  EXPECT_FLOAT_EQ(def[2], 0.0f);
  EXPECT_FLOAT_EQ(def[3], 0.0f);

  FC::Tuple t(1.0f, 2.0f, 3.0f, 4.0f);
  EXPECT_FLOAT_EQ(t[0], 1.0f);
  EXPECT_FLOAT_EQ(t[3], 4.0f);

  const FC::Tuple &ct = t;
  EXPECT_FLOAT_EQ(ct[2], 3.0f);

  t[1] = 9.0f; // non-const operator[]
  EXPECT_FLOAT_EQ(t[1], 9.0f);

  FC::Tuple copy(t);
  EXPECT_FLOAT_EQ(copy[1], 9.0f);

  FC::Tuple assigned;
  assigned = t;
  EXPECT_FLOAT_EQ(assigned[0], 1.0f);

  // self-assignment hits the this!=&copy guard
  FC::Tuple &alias = assigned;
  assigned = alias;
  EXPECT_FLOAT_EQ(assigned[3], 4.0f);

  float arr[4] = {5.0f, 6.0f, 7.0f, 8.0f};
  t.set(arr);
  EXPECT_FLOAT_EQ(t[0], 5.0f);
  EXPECT_FLOAT_EQ(t[3], 8.0f);

  t.set(-1.0f, -2.0f, -3.0f, -4.0f);
  EXPECT_FLOAT_EQ(t[2], -3.0f);

  t.set(copy);
  EXPECT_FLOAT_EQ(t[1], 9.0f);
  t.set(t); // self-set guard
  EXPECT_FLOAT_EQ(t[1], 9.0f);
}

// ===========================================================================
// Vector
// ===========================================================================

TEST(FCVectorMath, ConstructorsAndSet) {
  FC::Vector def;
  EXPECT_TRUE(def.isBad()); // all-zero default is the "bad" vector

  float arr[4] = {1.0f, 2.0f, 3.0f, 4.0f};
  FC::Vector fromArray(arr);
  EXPECT_FLOAT_EQ(fromArray[1], 2.0f);

  FC::Vector v(1.0f, 2.0f, 3.0f, 0.0f);
  FC::Vector copy(v);
  EXPECT_FLOAT_EQ(copy[2], 3.0f);

  FC::Vector assigned;
  assigned = v;
  EXPECT_FLOAT_EQ(assigned[0], 1.0f);
  FC::Vector &alias = assigned;
  assigned = alias; // self-assignment guard
  EXPECT_FLOAT_EQ(assigned[0], 1.0f);

  assigned.set(9.0f, 8.0f, 7.0f, 6.0f);
  EXPECT_FLOAT_EQ(assigned[3], 6.0f);
  assigned.set(arr);
  EXPECT_FLOAT_EQ(assigned[3], 4.0f);
  assigned.set(v);
  EXPECT_FLOAT_EQ(assigned[1], 2.0f);

  std::unique_ptr<FC::Vector> cloned(v.clone());
  ASSERT_TRUE(cloned != nullptr);
  EXPECT_FLOAT_EQ((*cloned)[0], 1.0f);
  EXPECT_FLOAT_EQ((*cloned)[2], 3.0f);
}

TEST(FCVectorMath, CrossAndDot) {
  FC::Vector x(1.0f, 0.0f, 0.0f, 0.0f);
  FC::Vector y(0.0f, 1.0f, 0.0f, 0.0f);

  FC::Vector z = x.cross(y);
  expectVecNear(z, 0.0f, 0.0f, 1.0f, 1e-6f);
  EXPECT_FLOAT_EQ(z[3], 0.0f);

  FC::Vector zNeg = y.cross(x); // anticommutative
  expectVecNear(zNeg, 0.0f, 0.0f, -1.0f, 1e-6f);

  FC::Vector w(1.0f, 0.0f, 0.0f, 0.0f);
  w.crossEquals(y);
  expectVecNear(w, 0.0f, 0.0f, 1.0f, 1e-6f);

  // NOTE: FastContouring::Vector::dot includes the w component.
  FC::Vector a(1.0f, 2.0f, 3.0f, 4.0f);
  FC::Vector b(5.0f, 6.0f, 7.0f, 8.0f);
  EXPECT_FLOAT_EQ(a.dot(b), 70.0f);
}

TEST(FCVectorMath, AddSubScaleNegate) {
  FC::Vector a(1.0f, 2.0f, 3.0f, 1.0f);
  FC::Vector b(10.0f, 20.0f, 30.0f, 0.0f);

  FC::Vector sum = a + b;
  expectVecNear(sum, 11.0f, 22.0f, 33.0f, 1e-6f);
  EXPECT_FLOAT_EQ(sum[3], 1.0f); // w adds

  FC::Vector diff = b - a;
  expectVecNear(diff, 9.0f, 18.0f, 27.0f, 1e-6f);
  EXPECT_FLOAT_EQ(diff[3], -1.0f);

  FC::Vector acc(1.0f, 1.0f, 1.0f, 0.0f);
  acc += b;
  expectVecNear(acc, 11.0f, 21.0f, 31.0f, 1e-6f);
  acc -= b;
  expectVecNear(acc, 1.0f, 1.0f, 1.0f, 1e-6f);

  // scalar multiply scales xyz but preserves w
  FC::Vector scaled = a * 2.0f;
  expectVecNear(scaled, 2.0f, 4.0f, 6.0f, 1e-6f);
  EXPECT_FLOAT_EQ(scaled[3], 1.0f);
  FC::Vector scaledInPlace(a);
  scaledInPlace *= 3.0f;
  expectVecNear(scaledInPlace, 3.0f, 6.0f, 9.0f, 1e-6f);
  EXPECT_FLOAT_EQ(scaledInPlace[3], 1.0f);

  // unary minus negates xyz but preserves w
  FC::Vector neg = -a;
  expectVecNear(neg, -1.0f, -2.0f, -3.0f, 1e-6f);
  EXPECT_FLOAT_EQ(neg[3], 1.0f);
}

TEST(FCVectorMath, NormalizeNormBadAndBlend) {
  // w == 0 branch: euclidean normalization
  FC::Vector dir(3.0f, 4.0f, 0.0f, 0.0f);
  EXPECT_FLOAT_EQ(dir.norm(), 5.0f);
  dir.normalize();
  expectVecNear(dir, 0.6f, 0.8f, 0.0f, 1e-6f);
  EXPECT_FLOAT_EQ(dir[3], 0.0f);

  // w != 0 branch: homogeneous divide
  FC::Vector homo(2.0f, 4.0f, 6.0f, 2.0f);
  homo.normalize();
  expectVecNear(homo, 1.0f, 2.0f, 3.0f, 1e-6f);
  EXPECT_FLOAT_EQ(homo[3], 1.0f);

  FC::Vector bad = FC::Vector::badVector();
  EXPECT_TRUE(bad.isBad());
  FC::Vector notBad(0.0f, 0.0f, 0.0f, 1.0f);
  EXPECT_FALSE(notBad.isBad());

  FC::Vector a(1.0f, 2.0f, 3.0f, 4.0f);
  FC::Vector b(5.0f, 6.0f, 7.0f, 8.0f);
  FC::Vector mid = FC::Vector::interpolate(a, b, 0.25f);
  expectVecNear(mid, 2.0f, 3.0f, 4.0f, 1e-5f);
  EXPECT_NEAR(mid[3], 5.0f, 1e-5f);

  // cubicInterpolate is documented (in the source) to fall back to linear
  FC::Vector c(9.0f, 10.0f, 11.0f, 12.0f);
  FC::Vector d(13.0f, 14.0f, 15.0f, 16.0f);
  FC::Vector cubic = FC::Vector::cubicInterpolate(a, b, c, d, 0.5f);
  FC::Vector linear = FC::Vector::interpolate(b, c, 0.5f);
  expectVecNear(cubic, linear[0], linear[1], linear[2], 1e-6f);
}

// ===========================================================================
// Quaternion
// ===========================================================================

TEST(FCQuaternionMath, ConstructorsAndSet) {
  FC::Quaternion identity;
  EXPECT_FLOAT_EQ(identity[0], 1.0f); // w first
  EXPECT_FLOAT_EQ(identity[1], 0.0f);
  EXPECT_FLOAT_EQ(identity[2], 0.0f);
  EXPECT_FLOAT_EQ(identity[3], 0.0f);

  FC::Quaternion q(0.5f, 0.1f, 0.2f, 0.3f);
  FC::Quaternion copy(q);
  EXPECT_FLOAT_EQ(copy[3], 0.3f);

  FC::Quaternion assigned;
  assigned = q;
  EXPECT_FLOAT_EQ(assigned[1], 0.1f);
  FC::Quaternion &alias = assigned;
  assigned = alias; // self-assignment guard
  EXPECT_FLOAT_EQ(assigned[1], 0.1f);

  assigned.set(1.0f, 2.0f, 3.0f, 4.0f);
  EXPECT_FLOAT_EQ(assigned[2], 3.0f);
  float arr[4] = {4.0f, 3.0f, 2.0f, 1.0f};
  assigned.set(arr);
  EXPECT_FLOAT_EQ(assigned[0], 4.0f);
  assigned.set(q);
  EXPECT_FLOAT_EQ(assigned[0], 0.5f);
}

TEST(FCQuaternionMath, AddSubNegateScalarOps) {
  FC::Quaternion a(1.0f, 2.0f, 3.0f, 4.0f);
  FC::Quaternion b(0.5f, 0.5f, 0.5f, 0.5f);

  FC::Quaternion sum = a + b;
  expectQuatNear(sum, FC::Quaternion(1.5f, 2.5f, 3.5f, 4.5f), 1e-6f);
  FC::Quaternion diff = a - b;
  expectQuatNear(diff, FC::Quaternion(0.5f, 1.5f, 2.5f, 3.5f), 1e-6f);
  FC::Quaternion neg = -a;
  expectQuatNear(neg, FC::Quaternion(-1.0f, -2.0f, -3.0f, -4.0f), 1e-6f);

  FC::Quaternion scaled = a * 2.0f;
  expectQuatNear(scaled, FC::Quaternion(2.0f, 4.0f, 6.0f, 8.0f), 1e-6f);
  FC::Quaternion divided = a / 2.0f;
  expectQuatNear(divided, FC::Quaternion(0.5f, 1.0f, 1.5f, 2.0f), 1e-6f);

  FC::Quaternion inPlace(a);
  inPlace *= 4.0f;
  expectQuatNear(inPlace, FC::Quaternion(4.0f, 8.0f, 12.0f, 16.0f), 1e-6f);
  inPlace /= 4.0f;
  expectQuatNear(inPlace, a, 1e-6f);
}

TEST(FCQuaternionMath, HamiltonProductAndPrePostMultiply) {
  FC::Quaternion qi(0.0f, 1.0f, 0.0f, 0.0f);
  FC::Quaternion qj(0.0f, 0.0f, 1.0f, 0.0f);
  FC::Quaternion qk(0.0f, 0.0f, 0.0f, 1.0f);

  // i*j = k (standard Hamilton convention)
  expectQuatNear(qi * qj, qk, 1e-6f);
  // j*i = -k
  expectQuatNear(qj * qi, -qk, 1e-6f);
  // i*i = -1
  expectQuatNear(qi * qi, FC::Quaternion(-1.0f, 0.0f, 0.0f, 0.0f), 1e-6f);

  FC::Quaternion identity;
  expectQuatNear(identity * qi, qi, 1e-6f);

  FC::Quaternion post(qi);
  post.postMultiply(qj); // == i*j
  expectQuatNear(post, qk, 1e-6f);

  FC::Quaternion pre(qi);
  pre.preMultiply(qj); // == j*i
  expectQuatNear(pre, -qk, 1e-6f);
}

TEST(FCQuaternionMath, NormConjugateInverseNormalize) {
  FC::Quaternion q(1.0f, 2.0f, 2.0f, 4.0f);
  EXPECT_FLOAT_EQ(q.norm(), 5.0f);

  FC::Quaternion conj = q.conjugate();
  expectQuatNear(conj, FC::Quaternion(1.0f, -2.0f, -2.0f, -4.0f), 1e-6f);

  // q * conjugate == (norm^2, 0, 0, 0)
  FC::Quaternion qc = q * conj;
  expectQuatNear(qc, FC::Quaternion(25.0f, 0.0f, 0.0f, 0.0f), 1e-4f);

  FC::Quaternion normalized(q);
  normalized.normalize();
  EXPECT_NEAR(normalized.norm(), 1.0f, 1e-6f);

  // for a unit quaternion, inverse() is a true inverse
  FC::Quaternion rot = FC::Quaternion::rotation(0.9f, 1.0f, -2.0f, 0.5f);
  FC::Quaternion prod = rot * rot.inverse();
  expectQuatNear(prod, FC::Quaternion(1.0f, 0.0f, 0.0f, 0.0f), 1e-5f);
}

TEST(FCQuaternionMath, RotationFactoryAndApplyRotation) {
  const float half = kPi / 4.0f;
  FC::Quaternion q = FC::Quaternion::rotation(kPi / 2.0f, 0.0f, 0.0f, 1.0f);
  expectQuatNear(q, FC::Quaternion(std::cos(half), 0.0f, 0.0f, std::sin(half)), 1e-6f);

  // axis overload produces the same quaternion
  FC::Quaternion qAxis = FC::Quaternion::rotation(kPi / 2.0f, FC::Vector(0.0f, 0.0f, 1.0f, 0.0f));
  expectQuatNear(qAxis, q, 1e-6f);

  // axis does not need to be unit length
  FC::Quaternion qScaledAxis = FC::Quaternion::rotation(kPi / 2.0f, 0.0f, 0.0f, 7.0f);
  expectQuatNear(qScaledAxis, q, 1e-6f);

  // degenerate zero axis returns identity in both overloads
  FC::Quaternion zeroAxis = FC::Quaternion::rotation(1.0f, 0.0f, 0.0f, 0.0f);
  expectQuatNear(zeroAxis, FC::Quaternion(), 1e-6f);
  FC::Quaternion zeroAxisVec = FC::Quaternion::rotation(1.0f, FC::Vector(0.0f, 0.0f, 0.0f, 0.0f));
  expectQuatNear(zeroAxisVec, FC::Quaternion(), 1e-6f);

  // 90 degrees about +z maps +x to +y; w passes through
  FC::Vector v(1.0f, 0.0f, 0.0f, 0.5f);
  FC::Vector rotated = q.applyRotation(v);
  expectVecNear(rotated, 0.0f, 1.0f, 0.0f, 1e-6f);
  EXPECT_FLOAT_EQ(rotated[3], 0.5f);

  // unit rotation preserves vector norms
  FC::Vector arbitrary(0.3f, -0.7f, 0.2f, 0.0f);
  FC::Quaternion qArb = FC::Quaternion::rotation(0.77f, 1.0f, 2.0f, -1.0f);
  FC::Vector arbRotated = qArb.applyRotation(arbitrary);
  EXPECT_NEAR(arbRotated.norm(), arbitrary.norm(), 1e-5f);

  // ray rotation rotates both origin and direction
  FC::Ray ray(FC::Vector(1.0f, 0.0f, 0.0f, 1.0f), FC::Vector(0.0f, 1.0f, 0.0f, 0.0f));
  FC::Ray rotatedRay = q.applyRotation(ray);
  expectVecNear(rotatedRay.m_Origin, 0.0f, 1.0f, 0.0f, 1e-6f);
  EXPECT_FLOAT_EQ(rotatedRay.m_Origin[3], 1.0f);
  expectVecNear(rotatedRay.m_Dir, -1.0f, 0.0f, 0.0f, 1e-6f);
  EXPECT_FLOAT_EQ(rotatedRay.m_Dir[3], 0.0f);
}

TEST(FCQuaternionMath, BuildMatrixMatchesApplyRotation) {
  FC::Quaternion q = FC::Quaternion::rotation(0.83f, 1.0f, 2.0f, 3.0f);
  FC::Matrix m = q.buildMatrix();

  const FC::Vector samples[3] = {FC::Vector(1.0f, 0.0f, 0.0f, 0.0f),
                                 FC::Vector(0.0f, 1.0f, 0.0f, 0.0f),
                                 FC::Vector(0.3f, -0.2f, 0.5f, 0.0f)};
  for (const FC::Vector &v : samples) {
    FC::Vector viaMatrix = m * v;
    FC::Vector viaQuat = q.applyRotation(v);
    expectVecNear(viaMatrix, viaQuat[0], viaQuat[1], viaQuat[2], 1e-5f);
  }

  // rotation-only matrix: last row/column are identity
  EXPECT_FLOAT_EQ(m.get(3, 3), 1.0f);
  EXPECT_FLOAT_EQ(m.get(0, 3), 0.0f);
  EXPECT_FLOAT_EQ(m.get(3, 0), 0.0f);
}

TEST(FCQuaternionMath, RotateAndPower) {
  FC::Quaternion q;
  q.rotate(kPi / 3.0f, 0.0f, 0.0f, 1.0f);
  expectQuatNear(q, FC::Quaternion::rotation(kPi / 3.0f, 0.0f, 0.0f, 1.0f), 1e-6f);

  // power on a rotation quaternion scales the rotation angle
  FC::Quaternion doubled = q.power(2.0);
  expectQuatNear(doubled, FC::Quaternion::rotation(2.0f * kPi / 3.0f, 0.0f, 0.0f, 1.0f), 1e-5f);

  // theta==0 branch (w >= 0.9999) with zero vector part
  FC::Quaternion identityPow = FC::Quaternion().power(0.5);
  expectQuatNear(identityPow, FC::Quaternion(1.0f, 0.0f, 0.0f, 0.0f), 1e-6f);

  // theta==2*pi branch (w <= -0.9999), zero vector part
  FC::Quaternion negIdent(-1.0f, 0.0f, 0.0f, 0.0f);
  FC::Quaternion negPow = negIdent.power(0.5);
  expectQuatNear(negPow, FC::Quaternion(-1.0f, 0.0f, 0.0f, 0.0f), 1e-5f);

  // theta==2*pi branch with a non-zero vector part
  FC::Quaternion negAxis(-1.0f, 0.6f, 0.0f, 0.0f);
  FC::Quaternion negAxisPow = negAxis.power(0.25);
  expectQuatNear(negAxisPow, FC::Quaternion(0.0f, 1.0f, 0.0f, 0.0f), 1e-5f);
}

TEST(FCQuaternionMath, LogAndExponent) {
  // log of the identity is zero (theta==0 and zero-axis branches)
  FC::Quaternion logIdent = FC::Quaternion::log(FC::Quaternion());
  expectQuatNear(logIdent, FC::Quaternion(0.0f, 0.0f, 0.0f, 0.0f), 1e-6f);

  // log normalizes its input first: log(2 * identity) is still zero
  FC::Quaternion logScaled = FC::Quaternion::log(FC::Quaternion(2.0f, 0.0f, 0.0f, 0.0f));
  expectQuatNear(logScaled, FC::Quaternion(0.0f, 0.0f, 0.0f, 0.0f), 1e-6f);

  // log of w<=-0.9999 quaternion with zero axis
  FC::Quaternion logNeg = FC::Quaternion::log(FC::Quaternion(-1.0f, 0.0f, 0.0f, 0.0f));
  expectQuatNear(logNeg, FC::Quaternion(0.0f, 0.0f, 0.0f, 0.0f), 1e-6f);

  // log of a rotation about z: (0, 0, 0, theta/2)
  FC::Quaternion rot = FC::Quaternion::rotation(kPi / 2.0f, 0.0f, 0.0f, 1.0f);
  FC::Quaternion logRot = FC::Quaternion::log(rot);
  expectQuatNear(logRot, FC::Quaternion(0.0f, 0.0f, 0.0f, kPi / 4.0f), 1e-5f);

  // exponent inverts log for unit rotations
  FC::Quaternion expLog = FC::Quaternion::exponent(logRot);
  expectQuatNear(expLog, rot, 1e-5f);

  // exponent of zero quaternion = identity (zero-axis branch)
  FC::Quaternion expZero = FC::Quaternion::exponent(FC::Quaternion(0.0f, 0.0f, 0.0f, 0.0f));
  expectQuatNear(expZero, FC::Quaternion(1.0f, 0.0f, 0.0f, 0.0f), 1e-6f);

  // exponent with non-zero scalar part: e^w * (cos|v|, u sin|v|)
  FC::Quaternion expFull = FC::Quaternion::exponent(FC::Quaternion(0.5f, 0.0f, 0.0f, 0.3f));
  const float ew = std::exp(0.5f);
  expectQuatNear(expFull, FC::Quaternion(ew * std::cos(0.3f), 0.0f, 0.0f, ew * std::sin(0.3f)),
                 1e-5f);

  // round trip exp(log(q)) for a generic rotation
  FC::Quaternion generic = FC::Quaternion::rotation(1.0f, 1.0f, 1.0f, 0.0f);
  FC::Quaternion roundTrip = FC::Quaternion::exponent(FC::Quaternion::log(generic));
  expectQuatNear(roundTrip, generic, 1e-5f);
}

TEST(FCQuaternionMath, InterpolateSlerp) {
  FC::Quaternion a = FC::Quaternion::rotation(0.2f, 0.0f, 0.0f, 1.0f);
  FC::Quaternion b = FC::Quaternion::rotation(1.0f, 0.0f, 0.0f, 1.0f);

  expectQuatNear(FC::Quaternion::interpolate(a, b, 0.0f), a, 1e-5f);
  expectQuatNear(FC::Quaternion::interpolate(a, b, 1.0f), b, 1e-4f);
  expectQuatNear(FC::Quaternion::interpolate(a, b, 0.5f),
                 FC::Quaternion::rotation(0.6f, 0.0f, 0.0f, 1.0f), 1e-4f);
}

TEST(FCQuaternionMath, CubicBlends) {
  FC::Quaternion a = FC::Quaternion::rotation(0.2f, 0.0f, 0.0f, 1.0f);
  FC::Quaternion b = FC::Quaternion::rotation(0.4f, 0.0f, 0.0f, 1.0f);
  FC::Quaternion c = FC::Quaternion::rotation(0.6f, 0.0f, 0.0f, 1.0f);
  FC::Quaternion d = FC::Quaternion::rotation(0.8f, 0.0f, 0.0f, 1.0f);

  // endpoints
  expectQuatNear(FC::Quaternion::cubicInterpolate(a, b, c, d, 0.0f), b, 1e-4f);
  expectQuatNear(FC::Quaternion::cubicInterpolate(a, b, c, d, 1.0f), c, 1e-4f);

  // midpoint of evenly spaced coaxial rotations reduces to slerp
  expectQuatNear(FC::Quaternion::cubicInterpolate(a, b, c, d, 0.5f),
                 FC::Quaternion::rotation(0.5f, 0.0f, 0.0f, 1.0f), 1e-3f);

  // start/end variants pin their endpoint
  expectQuatNear(FC::Quaternion::startCubicInterpolate(a, b, c, 0.0f), a, 1e-4f);
  expectQuatNear(FC::Quaternion::endCubicInterpolate(a, b, c, 1.0f), c, 1e-4f);
}

// ===========================================================================
// Matrix
// ===========================================================================

TEST(FCMatrixMath, ConstructorsAndColumnMajorStorage) {
  FC::Matrix identity;
  for (int r = 0; r < 4; ++r)
    for (int col = 0; col < 4; ++col)
      EXPECT_FLOAT_EQ(identity.get(r, col), (r == col) ? 1.0f : 0.0f);

  // ctor takes row-major arguments, stores column-major
  FC::Matrix a(1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f, 9.0f, 10.0f, 11.0f, 12.0f, 13.0f,
               14.0f, 15.0f, 16.0f);
  EXPECT_FLOAT_EQ(a.get(0, 0), 1.0f);
  EXPECT_FLOAT_EQ(a.get(0, 1), 2.0f);
  EXPECT_FLOAT_EQ(a.get(1, 0), 5.0f);
  EXPECT_FLOAT_EQ(a.get(3, 3), 16.0f);

  const float *raw = a.getMatrix();
  EXPECT_FLOAT_EQ(raw[0], 1.0f); // m00
  EXPECT_FLOAT_EQ(raw[1], 5.0f); // m10: column-major layout
  EXPECT_FLOAT_EQ(raw[4], 2.0f); // m01

  FC::Matrix copy(a);
  EXPECT_FLOAT_EQ(copy.get(2, 1), 10.0f);

  FC::Matrix assigned;
  assigned = a;
  EXPECT_FLOAT_EQ(assigned.get(1, 2), 7.0f);

  FC::Matrix viaSet;
  viaSet.set(a);
  EXPECT_FLOAT_EQ(viaSet.get(3, 0), 13.0f);
  viaSet.set(viaSet); // self-set guard
  EXPECT_FLOAT_EQ(viaSet.get(3, 0), 13.0f);

  viaSet.set(1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f,
             0.0f, 1.0f);
  EXPECT_FLOAT_EQ(viaSet.get(0, 0), 1.0f);
  EXPECT_FLOAT_EQ(viaSet.get(3, 0), 0.0f);

  FC::Matrix reset(a);
  reset.reset();
  EXPECT_FLOAT_EQ(reset.get(0, 1), 0.0f);
  EXPECT_FLOAT_EQ(reset.get(2, 2), 1.0f);
}

TEST(FCMatrixMath, VectorRayAndMatrixProducts) {
  FC::Matrix t = FC::Matrix::translation(1.0f, 2.0f, 3.0f);
  FC::Matrix tVec = FC::Matrix::translation(FC::Vector(1.0f, 2.0f, 3.0f, 1.0f));
  for (int r = 0; r < 4; ++r)
    for (int col = 0; col < 4; ++col)
      EXPECT_FLOAT_EQ(t.get(r, col), tVec.get(r, col));

  // points (w=1) translate; directions (w=0) do not
  FC::Vector point(1.0f, 1.0f, 1.0f, 1.0f);
  FC::Vector moved = t * point;
  expectVecNear(moved, 2.0f, 3.0f, 4.0f, 1e-6f);
  EXPECT_FLOAT_EQ(moved[3], 1.0f);
  FC::Vector dir(1.0f, 1.0f, 1.0f, 0.0f);
  FC::Vector movedDir = t * dir;
  expectVecNear(movedDir, 1.0f, 1.0f, 1.0f, 1e-6f);

  FC::Ray ray(point, dir);
  FC::Ray movedRay = t * ray;
  expectVecNear(movedRay.m_Origin, 2.0f, 3.0f, 4.0f, 1e-6f);
  expectVecNear(movedRay.m_Dir, 1.0f, 1.0f, 1.0f, 1e-6f);

  FC::Matrix s = FC::Matrix::scale(2.0f, 3.0f, 4.0f);
  EXPECT_FLOAT_EQ(s.get(0, 0), 2.0f);
  EXPECT_FLOAT_EQ(s.get(1, 1), 3.0f);
  EXPECT_FLOAT_EQ(s.get(2, 2), 4.0f);

  // (T*S)*v == T*(S*v)
  FC::Matrix ts = t * s;
  FC::Vector composed = ts * point;
  FC::Vector stepwise = t * (s * point);
  expectVecNear(composed, stepwise[0], stepwise[1], stepwise[2], 1e-5f);
  expectVecNear(composed, 3.0f, 5.0f, 7.0f, 1e-5f);

  // postMultiplication(B) == this*B ; preMultiplication(B) == B*this
  FC::Matrix post(t);
  post.postMultiplication(s);
  FC::Matrix pre(s);
  pre.preMultiplication(t);
  for (int r = 0; r < 4; ++r)
    for (int col = 0; col < 4; ++col) {
      EXPECT_FLOAT_EQ(post.get(r, col), ts.get(r, col));
      EXPECT_FLOAT_EQ(pre.get(r, col), ts.get(r, col));
    }
}

TEST(FCMatrixMath, InverseInverseTransposeDeterminant) {
  EXPECT_NEAR(FC::Matrix::scale(2.0f, 3.0f, 4.0f).determinant(), 24.0f, 1e-4f);
  EXPECT_NEAR(FC::Matrix::rotationZ(0.7f).determinant(), 1.0f, 1e-5f);

  FC::Matrix m = FC::Matrix::translation(1.0f, 2.0f, 3.0f);
  m.postMultiplication(FC::Matrix::rotationZ(0.3f));
  m.postMultiplication(FC::Matrix::scale(2.0f, 1.5f, 4.0f));

  FC::Matrix inv = m.inverse();
  FC::Matrix shouldBeIdentity = m * inv;
  for (int r = 0; r < 4; ++r)
    for (int col = 0; col < 4; ++col)
      EXPECT_NEAR(shouldBeIdentity.get(r, col), (r == col) ? 1.0f : 0.0f, 1e-3f);

  // inverseTranspose == transpose of inverse
  FC::Matrix invT = m.inverseTranspose();
  for (int r = 0; r < 4; ++r)
    for (int col = 0; col < 4; ++col)
      EXPECT_NEAR(invT.get(r, col), inv.get(col, r), 1e-4f);

  // singular matrices: inverse and inverseTranspose return identity
  FC::Matrix singular = FC::Matrix::scale(1.0f, 1.0f, 0.0f);
  EXPECT_NEAR(singular.determinant(), 0.0f, 1e-6f);
  FC::Matrix singularInv = singular.inverse();
  FC::Matrix singularInvT = singular.inverseTranspose();
  for (int r = 0; r < 4; ++r)
    for (int col = 0; col < 4; ++col) {
      EXPECT_FLOAT_EQ(singularInv.get(r, col), (r == col) ? 1.0f : 0.0f);
      EXPECT_FLOAT_EQ(singularInvT.get(r, col), (r == col) ? 1.0f : 0.0f);
    }
}

TEST(FCMatrixMath, AxisRotations) {
  const float a = kPi / 2.0f;

  // NOTE: these factories use the transpose of the usual right-handed
  // rotation matrices; assertions record actual behavior.
  FC::Vector rx = FC::Matrix::rotationX(a) * FC::Vector(0.0f, 1.0f, 0.0f, 0.0f);
  expectVecNear(rx, 0.0f, 0.0f, -1.0f, 1e-6f);

  FC::Vector ry = FC::Matrix::rotationY(a) * FC::Vector(1.0f, 0.0f, 0.0f, 0.0f);
  expectVecNear(ry, 0.0f, 0.0f, 1.0f, 1e-6f);

  FC::Vector rz = FC::Matrix::rotationZ(a) * FC::Vector(1.0f, 0.0f, 0.0f, 0.0f);
  expectVecNear(rz, 0.0f, -1.0f, 0.0f, 1e-6f);

  // orthonormal: inverse equals transpose, norms preserved
  FC::Matrix r = FC::Matrix::rotationY(0.42f);
  FC::Matrix rInv = r.inverse();
  for (int row = 0; row < 4; ++row)
    for (int col = 0; col < 4; ++col)
      EXPECT_NEAR(rInv.get(row, col), r.get(col, row), 1e-5f);

  FC::Vector v(0.3f, -0.4f, 0.5f, 0.0f);
  FC::Vector rotated = r * v;
  EXPECT_NEAR(rotated.norm(), v.norm(), 1e-5f);
}

// ===========================================================================
// Ray
// ===========================================================================

TEST(FCRayGeometry, DefaultsAndPointOnRay) {
  FC::Ray def;
  expectVecNear(def.m_Origin, 0.0f, 0.0f, 0.0f, 1e-6f);
  EXPECT_FLOAT_EQ(def.m_Origin[3], 1.0f);
  expectVecNear(def.m_Dir, 0.0f, 0.0f, 1.0f, 1e-6f);
  EXPECT_FLOAT_EQ(def.m_Dir[3], 0.0f);

  FC::Vector p = def.getPointOnRay(2.5f);
  expectVecNear(p, 0.0f, 0.0f, 2.5f, 1e-6f);

  FC::Ray ray(FC::Vector(1.0f, 2.0f, 3.0f, 1.0f), FC::Vector(1.0f, 0.0f, 0.0f, 0.0f));
  FC::Vector q = ray.getPointOnRay(-2.0f);
  expectVecNear(q, -1.0f, 2.0f, 3.0f, 1e-6f);
}

TEST(FCRayGeometry, NearestToCoordinateAxes) {
  // ray parallel to z through (5,3,*): closest approach to X/Y axes at z=0
  FC::Ray rz(FC::Vector(5.0f, 3.0f, 4.0f, 1.0f), FC::Vector(0.0f, 0.0f, 1.0f, 0.0f));

  EXPECT_NEAR(rz.nearestTOnXAxis(), -4.0f, 1e-5f);
  FC::Vector px = rz.nearestPointOnXAxis();
  expectVecNear(px, 5.0f, 0.0f, 0.0f, 1e-5f);
  EXPECT_NEAR(rz.distanceToXAxis(), 3.0f, 1e-5f);

  EXPECT_NEAR(rz.nearestTOnYAxis(), -4.0f, 1e-5f);
  FC::Vector py = rz.nearestPointOnYAxis();
  expectVecNear(py, 0.0f, 3.0f, 0.0f, 1e-5f);
  EXPECT_NEAR(rz.distanceToYAxis(), 5.0f, 1e-5f);

  // ray parallel to x through (*,2,5): closest approach to Z axis at x=0
  FC::Ray rx(FC::Vector(1.0f, 2.0f, 5.0f, 1.0f), FC::Vector(1.0f, 0.0f, 0.0f, 0.0f));
  EXPECT_NEAR(rx.nearestTOnZAxis(), -1.0f, 1e-5f);
  FC::Vector pz = rx.nearestPointOnZAxis();
  expectVecNear(pz, 0.0f, 0.0f, 5.0f, 1e-5f);
  EXPECT_NEAR(rx.distanceToZAxis(), 2.0f, 1e-5f);
}

TEST(FCRayGeometry, NearestToShiftedAxes) {
  FC::Ray rz(FC::Vector(5.0f, 3.0f, 4.0f, 1.0f), FC::Vector(0.0f, 0.0f, 1.0f, 0.0f));
  FC::Vector origin(1.0f, 1.0f, 1.0f, 1.0f);

  // X axis through (1,1,1): shifted ray origin is (4,2,3)
  EXPECT_NEAR(rz.nearestTOnXAxis(origin), -3.0f, 1e-5f);
  FC::Vector px = rz.nearestPointOnXAxis(origin);
  expectVecNear(px, 5.0f, 1.0f, 1.0f, 1e-5f);
  EXPECT_NEAR(rz.distanceToXAxis(origin), 2.0f, 1e-5f);

  EXPECT_NEAR(rz.nearestTOnYAxis(origin), -3.0f, 1e-5f);
  FC::Vector py = rz.nearestPointOnYAxis(origin);
  expectVecNear(py, 1.0f, 3.0f, 1.0f, 1e-5f);
  EXPECT_NEAR(rz.distanceToYAxis(origin), 4.0f, 1e-5f);

  FC::Ray rx(FC::Vector(1.0f, 2.0f, 5.0f, 1.0f), FC::Vector(1.0f, 0.0f, 0.0f, 0.0f));
  EXPECT_NEAR(rx.nearestTOnZAxis(origin), 0.0f, 1e-5f);
  FC::Vector pz = rx.nearestPointOnZAxis(origin);
  expectVecNear(pz, 1.0f, 1.0f, 5.0f, 1e-5f);
  EXPECT_NEAR(rx.distanceToZAxis(origin), 1.0f, 1e-5f);
}

// ===========================================================================
// ContourGeometry
// ===========================================================================

TEST(FCContourGeometryBuffers, TriangleMeshBuildAndAddToGeometry) {
  FC::ContourGeometry cg;
  ASSERT_TRUE(cg.allocateVertexBuffers(2));
  ASSERT_TRUE(cg.allocateTriangleBuffers(1));
  cg.setIsovalue(0.5f);
  EXPECT_FALSE(cg.useColors());

  unsigned int id = 999;
  // edge 0: interpolates to the midpoint (densities 0 -> 1, iso 0.5)
  ASSERT_TRUE(cg.addEdge(id, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f,
                         0.0f, 1.0f));
  EXPECT_EQ(id, 0u);
  // edge 1
  ASSERT_TRUE(cg.addEdge(id, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f,
                         0.0f, 1.0f));
  EXPECT_EQ(id, 1u);
  // edge 2: equal densities, stays at its first endpoint; triggers
  // doubleVertexBuffers (allocated 2)
  ASSERT_TRUE(cg.addEdge(id, 2.0f, 3.0f, 4.0f, 1.0f, 0.0f, 0.0f, 9.0f, 9.0f, 9.0f, 0.0f, 1.0f, 0.0f,
                         0.25f, 0.25f));
  EXPECT_EQ(id, 2u);

  ASSERT_TRUE(cg.addTriangle(0, 1, 2));
  ASSERT_TRUE(cg.addTriangle(2, 1, 0)); // triggers doubleTriangleBuffers

  EXPECT_EQ(cg.getNumVerts(), 3);
  EXPECT_EQ(cg.getNumTris(), 2);

  FC::TriSurf surf;
  FC::Matrix identity;
  int nextVert = 0, nextTri = 0;
  cg.addToGeometry(surf, identity, nextVert, nextTri);

  EXPECT_EQ(nextVert, 3);
  EXPECT_EQ(nextTri, 2);
  ASSERT_EQ(surf.verts.size(), 9u);
  ASSERT_EQ(surf.normals.size(), 9u);
  EXPECT_TRUE(surf.colors.empty()); // useColors() == false
  ASSERT_EQ(surf.tris.size(), 6u);

  // vertex 0 interpolated to midpoint
  EXPECT_NEAR(surf.verts[0], 0.5, 1e-6);
  EXPECT_NEAR(surf.verts[1], 0.0, 1e-6);
  EXPECT_NEAR(surf.verts[2], 0.0, 1e-6);
  EXPECT_NEAR(surf.normals[0], 0.0, 1e-6);
  EXPECT_NEAR(surf.normals[1], 0.5, 1e-6);
  EXPECT_NEAR(surf.normals[2], 0.5, 1e-6);
  // vertex 1 interpolated to midpoint
  EXPECT_NEAR(surf.verts[3], 0.0, 1e-6);
  EXPECT_NEAR(surf.verts[4], 0.5, 1e-6);
  EXPECT_NEAR(surf.verts[5], 0.0, 1e-6);
  // vertex 2 untouched (equal densities)
  EXPECT_NEAR(surf.verts[6], 2.0, 1e-6);
  EXPECT_NEAR(surf.verts[7], 3.0, 1e-6);
  EXPECT_NEAR(surf.verts[8], 4.0, 1e-6);
  EXPECT_NEAR(surf.normals[6], 1.0, 1e-6);

  const unsigned int expectedTris[6] = {0, 1, 2, 2, 1, 0};
  for (int i = 0; i < 6; ++i)
    EXPECT_EQ(surf.tris[i], expectedTris[i]);

  // second export with a vertex offset; interpolation is already done
  FC::TriSurf surf2;
  nextVert = 7;
  nextTri = 0;
  cg.addToGeometry(surf2, identity, nextVert, nextTri);
  EXPECT_EQ(nextVert, 10);
  ASSERT_EQ(surf2.tris.size(), 6u);
  EXPECT_EQ(surf2.tris[0], 7u);
  EXPECT_EQ(surf2.tris[2], 9u);
  EXPECT_NEAR(surf2.verts[0], 0.5, 1e-6); // values unchanged on re-export
}

TEST(FCContourGeometryBuffers, ColorMeshAndSingleColor) {
  FC::ContourGeometry cg;
  ASSERT_TRUE(cg.allocateVertexBuffers(4));
  ASSERT_TRUE(cg.allocateTriangleBuffers(4));
  cg.setUseColors(true);
  EXPECT_TRUE(cg.useColors());
  cg.setIsovalue(0.25f);

  unsigned int id = 0;
  ASSERT_TRUE(cg.addEdge(id, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 0.0f, 0.0f, 2.0f, 0.0f, 0.0f,
                         0.0f, 0.0f, 1.0f, 0.0f, 1.0f, 0.0f, 0.0f, 1.0f));
  ASSERT_TRUE(cg.addEdge(id, 0.0f, 2.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f,
                         0.0f, 0.0f, 1.0f, 1.0f, 1.0f, 0.0f, 0.0f, 1.0f));
  ASSERT_TRUE(cg.addEdge(id, 0.0f, 0.0f, 2.0f, 0.0f, 0.0f, 1.0f, 0.5f, 0.5f, 0.5f, 0.0f, 0.0f, 0.0f,
                         0.0f, 0.0f, 1.0f, 1.0f, 0.0f, 1.0f, 0.5f, 0.5f));
  ASSERT_TRUE(cg.addTriangle(0, 1, 2));

  FC::TriSurf surf;
  FC::Matrix identity;
  int nextVert = 0, nextTri = 0;
  cg.addToGeometry(surf, identity, nextVert, nextTri);

  ASSERT_EQ(surf.verts.size(), 9u);
  ASSERT_EQ(surf.colors.size(), 9u);
  // interpolation moved vertex 0 a quarter of the way along the edge...
  EXPECT_NEAR(surf.verts[0], 0.5, 1e-6);
  // ...but colors are NOT interpolated by doInterpolation (it always calls
  // the color-less interpArray): each vertex keeps its first-endpoint color.
  EXPECT_NEAR(surf.colors[0], 1.0, 1e-6);
  EXPECT_NEAR(surf.colors[1], 0.0, 1e-6);
  EXPECT_NEAR(surf.colors[2], 0.0, 1e-6);
  EXPECT_NEAR(surf.colors[3], 0.0, 1e-6);
  EXPECT_NEAR(surf.colors[5], 1.0, 1e-6);

  // setSingleColor repaints every vertex
  cg.setSingleColor(0.2f, 0.3f, 0.4f);
  FC::TriSurf repainted;
  nextVert = 0;
  nextTri = 0;
  cg.addToGeometry(repainted, identity, nextVert, nextTri);
  ASSERT_EQ(repainted.colors.size(), 9u);
  for (unsigned int v = 0; v < 3; ++v) {
    EXPECT_NEAR(repainted.colors[v * 3 + 0], 0.2, 1e-6);
    EXPECT_NEAR(repainted.colors[v * 3 + 1], 0.3, 1e-6);
    EXPECT_NEAR(repainted.colors[v * 3 + 2], 0.4, 1e-6);
  }

  // re-allocation with a smaller size reuses the buffers and resets counts
  ASSERT_TRUE(cg.allocateVertexBuffers(1));
  ASSERT_TRUE(cg.allocateTriangleBuffers(1));
  EXPECT_EQ(cg.getNumVerts(), 0);
  EXPECT_EQ(cg.getNumTris(), 0);
}

TEST(FCContourGeometryBuffers, EmptyAndUnallocatedExport) {
  // addToGeometry on a fresh object (no buffers) is a no-op
  FC::ContourGeometry unallocated;
  FC::TriSurf surf;
  FC::Matrix identity;
  int nextVert = 5, nextTri = 7;
  unallocated.addToGeometry(surf, identity, nextVert, nextTri);
  EXPECT_EQ(nextVert, 5);
  EXPECT_EQ(nextTri, 7);
  EXPECT_TRUE(surf.verts.empty());
  EXPECT_EQ(unallocated.getNumVerts(), 0);
  EXPECT_EQ(unallocated.getNumTris(), 0);

  // allocated but empty: export produces empty arrays
  FC::ContourGeometry emptyMesh;
  ASSERT_TRUE(emptyMesh.allocateVertexBuffers(4));
  ASSERT_TRUE(emptyMesh.allocateTriangleBuffers(4));
  nextVert = 0;
  nextTri = 0;
  emptyMesh.addToGeometry(surf, identity, nextVert, nextTri);
  EXPECT_EQ(nextVert, 0);
  EXPECT_EQ(nextTri, 0);
  EXPECT_TRUE(surf.verts.empty());
  EXPECT_TRUE(surf.tris.empty());

  // destroy paths are safe to call explicitly and repeatedly
  emptyMesh.destroyVertexBuffers();
  emptyMesh.destroyTriangleBuffers();
  emptyMesh.destroyQuadBuffers();
  emptyMesh.destroyVertexBuffers();
}

TEST(FCContourGeometryBuffers, QuadMeshSmoothNormals) {
  FC::ContourGeometry cg;
  // tiny initial allocation so addQuadVertex has to double repeatedly
  ASSERT_TRUE(cg.allocateVertexBuffers(1));

  // unit square in the XY plane (+ one orphan vertex for the
  // zero-length-normal branch of the normalizer)
  ASSERT_TRUE(cg.addQuadVertex(0.0f, 0.0f, 0.0f, 9.0f, 9.0f, 9.0f, 1.0f, 0.0f, 0.0f));
  ASSERT_TRUE(cg.addQuadVertex(1.0f, 0.0f, 0.0f, 9.0f, 9.0f, 9.0f, 0.0f, 1.0f, 0.0f));
  ASSERT_TRUE(cg.addQuadVertex(1.0f, 1.0f, 0.0f, 9.0f, 9.0f, 9.0f, 0.0f, 0.0f, 1.0f));
  ASSERT_TRUE(cg.addQuadVertex(0.0f, 1.0f, 0.0f, 9.0f, 9.0f, 9.0f, 1.0f, 1.0f, 0.0f));
  ASSERT_TRUE(cg.addQuadVertex(5.0f, 5.0f, 5.0f, 9.0f, 9.0f, 9.0f, 0.0f, 1.0f, 1.0f));
  EXPECT_EQ(cg.getNumVerts(), 5);

  ASSERT_TRUE(cg.allocateQuadBuffers(1));
  ASSERT_TRUE(cg.addQuad(0, 1, 2, 3));
  ASSERT_TRUE(cg.addQuad(0, 1, 2, 3)); // triggers doubleQuadBuffers

  cg.CalculateQuadSmoothNormals();
  EXPECT_EQ(cg.getNumVerts(), 5);

  // reuse path for quad buffers, then normals with zero quads: every
  // accumulated normal is zero and the normalizer's degenerate branch runs
  ASSERT_TRUE(cg.allocateQuadBuffers(1));
  cg.CalculateQuadSmoothNormals();
  EXPECT_EQ(cg.getNumVerts(), 5);

  cg.setSingleColor(0.5f, 0.6f, 0.7f);
  EXPECT_EQ(cg.getNumVerts(), 5);
}

// ===========================================================================
// MarchingCubesBuffers
// ===========================================================================

TEST(FCMarchingCubesBufferHandling, AllocateSwapCopy) {
  FC::MarchingCubesBuffers buffers;
  EXPECT_TRUE(buffers.m_EdgeCaches[0].get() == nullptr);

  ASSERT_TRUE(buffers.allocateEdgeBuffers(4, 3));
  for (unsigned int i = 0; i < 5; ++i)
    EXPECT_TRUE(buffers.m_EdgeCaches[i].get() != nullptr);
  for (unsigned int i = 0; i < 2; ++i)
    EXPECT_TRUE(buffers.m_VertClassifications[i].get() != nullptr);

  // fill classifications so we can observe the swap
  buffers.m_VertClassifications[0][0] = 111u;
  buffers.m_VertClassifications[1][0] = 222u;
  unsigned int *before0 = buffers.m_VertClassifications[0].get();
  unsigned int *before1 = buffers.m_VertClassifications[1].get();

  buffers.swapEdgeBuffers();
  EXPECT_EQ(buffers.m_VertClassifications[0].get(), before1);
  EXPECT_EQ(buffers.m_VertClassifications[1].get(), before0);
  EXPECT_EQ(buffers.m_VertClassifications[0][0], 222u);
  EXPECT_EQ(buffers.m_VertClassifications[1][0], 111u);

  // re-allocation with a smaller footprint reuses the existing buffers
  unsigned int *keep = buffers.m_EdgeCaches[0].get();
  ASSERT_TRUE(buffers.allocateEdgeBuffers(2, 2));
  EXPECT_EQ(buffers.m_EdgeCaches[0].get(), keep);

  // copy constructor and assignment deep-copy the buffers
  buffers.m_EdgeCaches[0][0] = 314u;
  FC::MarchingCubesBuffers copy(buffers);
  ASSERT_TRUE(copy.m_EdgeCaches[0].get() != nullptr);
  EXPECT_NE(copy.m_EdgeCaches[0].get(), buffers.m_EdgeCaches[0].get());
  EXPECT_EQ(copy.m_EdgeCaches[0][0], 314u);

  FC::MarchingCubesBuffers assigned;
  assigned.allocateEdgeBuffers(2, 2);
  assigned = buffers;
  ASSERT_TRUE(assigned.m_VertClassifications[0].get() != nullptr);
  EXPECT_EQ(assigned.m_VertClassifications[0][0], 222u);

  buffers.destroyEdgeBuffers();
  EXPECT_TRUE(buffers.m_EdgeCaches[0].get() == nullptr);
}

// ===========================================================================
// ContourExtractor
// ===========================================================================

class FCContourExtractorTest : public ::testing::Test {
protected:
  cvc::app ctx;
};

TEST_F(FCContourExtractorTest, SphereSurfaceBasics) {
  cvc::volume v = makeSphereVolume(ctx, 16, 0.5);
  FC::ContourExtractor ex(ctx);
  ex.setVolume(v);

  FC::TriSurf surf = ex.extractContour(0.0, 0.25, 0.5, 0.75);

  ASSERT_FALSE(surf.verts.empty());
  ASSERT_FALSE(surf.tris.empty());
  ASSERT_EQ(surf.verts.size() % 3, 0u);
  ASSERT_EQ(surf.tris.size() % 3, 0u);
  EXPECT_EQ(surf.normals.size(), surf.verts.size());
  EXPECT_EQ(surf.colors.size(), surf.verts.size());

  const unsigned int numVerts = surf.verts.size() / 3;
  for (unsigned int t : surf.tris)
    ASSERT_LT(t, numVerts);

  // vertices lie near the r=0.5 sphere, in object space
  for (unsigned int i = 0; i < numVerts; ++i) {
    double x = surf.verts[i * 3 + 0];
    double y = surf.verts[i * 3 + 1];
    double z = surf.verts[i * 3 + 2];
    double d = std::sqrt(x * x + y * y + z * z);
    EXPECT_NEAR(d, 0.5, 0.1) << "vertex " << i;
  }

  // constant per-vertex color
  for (unsigned int i = 0; i < numVerts; ++i) {
    EXPECT_NEAR(surf.colors[i * 3 + 0], 0.25, 1e-6);
    EXPECT_NEAR(surf.colors[i * 3 + 1], 0.5, 1e-6);
    EXPECT_NEAR(surf.colors[i * 3 + 2], 0.75, 1e-6);
  }

  // normals are non-degenerate
  for (unsigned int i = 0; i < numVerts; ++i) {
    double nx = surf.normals[i * 3 + 0];
    double ny = surf.normals[i * 3 + 1];
    double nz = surf.normals[i * 3 + 2];
    EXPECT_GT(nx * nx + ny * ny + nz * nz, 0.0) << "normal " << i;
  }
}

TEST_F(FCContourExtractorTest, IsovalueOutsideRangeGivesEmptySurface) {
  cvc::volume v = makeSphereVolume(ctx, 16, 0.5);
  FC::ContourExtractor ex(ctx);
  ex.setVolume(v);

  FC::TriSurf below = ex.extractContour(-5.0);
  EXPECT_TRUE(below.verts.empty());
  EXPECT_TRUE(below.tris.empty());
  EXPECT_TRUE(below.normals.empty());

  FC::TriSurf above = ex.extractContour(5.0);
  EXPECT_TRUE(above.verts.empty());
  EXPECT_TRUE(above.tris.empty());
}

TEST_F(FCContourExtractorTest, PlaneInNonCubicVolume) {
  // 6x9x13 ramp along x in [0,1]^3; iso 0.45 sits inside cell i=2
  cvc::volume v = makeRampVolume(ctx, 6, 9, 13);
  FC::ContourExtractor ex(ctx);
  ex.setVolume(v);

  FC::TriSurf surf = ex.extractContour(0.45);
  ASSERT_FALSE(surf.verts.empty());
  const unsigned int numVerts = surf.verts.size() / 3;
  for (unsigned int i = 0; i < numVerts; ++i) {
    EXPECT_NEAR(surf.verts[i * 3 + 0], 0.45, 1e-4) << "vertex " << i;
    EXPECT_GE(surf.verts[i * 3 + 1], -1e-6);
    EXPECT_LE(surf.verts[i * 3 + 1], 1.0 + 1e-6);
    EXPECT_GE(surf.verts[i * 3 + 2], -1e-6);
    EXPECT_LE(surf.verts[i * 3 + 2], 1.0 + 1e-6);
  }
  // the plane cuts every one of the 8x12 cell columns into >= 2 triangles
  EXPECT_GE(surf.tris.size() / 3, 96u * 2u);
  // default color is white
  ASSERT_EQ(surf.colors.size(), surf.verts.size());
  EXPECT_NEAR(surf.colors[0], 1.0, 1e-6);
  EXPECT_NEAR(surf.colors[surf.colors.size() - 1], 1.0, 1e-6);
}

TEST_F(FCContourExtractorTest, SphereThroughVolumeBordersStaysInBounds) {
  // radius 1.2 pokes through all six faces of the [-1,1]^3 box, driving the
  // border branches of getNormal at i/j/k == 0 and dim-1
  cvc::volume v = makeSphereVolume(ctx, 8, 1.2);
  FC::ContourExtractor ex(ctx);
  ex.setVolume(v);

  FC::TriSurf surf = ex.extractContour(0.0);
  ASSERT_FALSE(surf.verts.empty());
  for (double coord : surf.verts) {
    EXPECT_GE(coord, -1.0 - 1e-6);
    EXPECT_LE(coord, 1.0 + 1e-6);
  }
  EXPECT_EQ(surf.normals.size(), surf.verts.size());
}

TEST_F(FCContourExtractorTest, SetVolumeTwiceAndCopySemantics) {
  cvc::volume sphere = makeSphereVolume(ctx, 16, 0.5);
  cvc::volume smallvol = makeSphereVolume(ctx, 8, 0.5);

  FC::ContourExtractor ex(ctx);
  ex.setVolume(sphere);
  EXPECT_EQ(ex.getVolume().XDim(), 16u);
  FC::TriSurf first = ex.extractContour(0.0);
  ASSERT_FALSE(first.verts.empty());

  // swap in a smaller volume; edge buffers get reused/reallocated
  ex.setVolume(smallvol);
  EXPECT_EQ(ex.getVolume().XDim(), 8u);
  FC::TriSurf second = ex.extractContour(0.0);
  ASSERT_FALSE(second.verts.empty());
  EXPECT_LT(second.verts.size(), first.verts.size());

  // copy constructor and assignment keep an independent, working extractor
  FC::ContourExtractor copy(ex);
  FC::TriSurf fromCopy = copy.extractContour(0.0);
  EXPECT_EQ(fromCopy.verts.size(), second.verts.size());

  FC::ContourExtractor assigned(ctx);
  assigned = ex;
  FC::TriSurf fromAssigned = assigned.extractContour(0.0);
  EXPECT_EQ(fromAssigned.verts.size(), second.verts.size());
}
