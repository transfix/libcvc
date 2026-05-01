// originally written by Sangmin Park, modified by Joe!

#include <algorithm>
#include <boost/foreach.hpp>
#include <boost/format.hpp>
#include <boost/scoped_array.hpp>
#include <cmath>
#include <cvc/app.h>
#include <cvc/geometry.h>
#include <limits>
#include <map>
#include <vector>

#define maxIndex 20
#define SHAREDT 15

// #define       FIX_BOUNDARY
// #define       PERTURB_1
#define GEOMETRIC_FLOW
#define SMOOTHING
// #define       PERTURB_2

using std::isnan;

namespace {
// calculate normal at v0
void crossproduct(float v0[3], float v1[3], float v2[3], float *normal) {

  float v01[3], v02[3], g;

  int i;
  for (i = 0; i < 3; i++) {
    v01[i] = v1[i] - v0[i];
    v02[i] = v2[i] - v0[i];
  }

  normal[0] = v01[1] * v02[2] - v02[1] * v01[2];
  normal[1] = v01[2] * v02[0] - v02[2] * v01[0];
  normal[2] = v01[0] * v02[1] - v02[0] * v01[1];

  g = normal[0] * normal[0] + normal[1] * normal[1] + normal[2] * normal[2];
  if (g < 0.0) // avoid NaN
    for (i = 0; i < 3; i++)
      normal[i] = 0.0;
  else
    for (i = 0; i < 3; i++)
      normal[i] /= (float)sqrt(g);
}

// calculate the area for a triangle
float area_tri(float v0[3], float v1[3], float v2[3]) {

  float a, b, c, p, area;
  int i;

  a = 0.0;
  b = 0.0;
  c = 0.0;
  for (i = 0; i < 3; i++) {
    a += (v1[i] - v0[i]) * (v1[i] - v0[i]);
    b += (v2[i] - v1[i]) * (v2[i] - v1[i]);
    c += (v0[i] - v2[i]) * (v0[i] - v2[i]);
  }
  a = (float)sqrt(a);
  b = (float)sqrt(b);
  c = (float)sqrt(c);

  p = (a + b + c) * 0.5;
  area = (float)sqrt(p * (p - a) * (p - b) * (p - c));

  return area;
}

// return length
float Normalize(float *Vec3) {
  float Tempf;

  Tempf = sqrt(Vec3[0] * Vec3[0] + Vec3[1] * Vec3[1] + Vec3[2] * Vec3[2]);
  if (Tempf < 1e-6) {
    Vec3[0] = 0.0;
    Vec3[1] = 0.0;
    Vec3[2] = 0.0;
  } else {
    Vec3[0] /= Tempf;
    Vec3[1] /= Tempf;
    Vec3[2] /= Tempf;
  }

  return Tempf;
}

// Uniform Radom Number Generator from 0 to 1
float ranf() {
  double UniformRandomNum;

  UniformRandomNum = (double)rand();
  UniformRandomNum /= (double)RAND_MAX;
  return (float)UniformRandomNum; // from 0 to 1
}

// Uniform Radom Number Generator from Minf to Maxf
float ranf(float Minf, float Maxf) {
  double UniformRandomNum;

  UniformRandomNum = (double)rand();
  UniformRandomNum /= (double)RAND_MAX;
  UniformRandomNum *= (Maxf - Minf);
  UniformRandomNum += Minf;

  return (float)UniformRandomNum; // from 0 to 1
}

//
// Normal Random Variate Generator
// Mean=m, Standard deviation=S
//
float GaussianRandomNum(float m, float s) {
  float x1, x2, w, y1;
  static float y2;
  static int use_last = 0;

  // use value from previous call
  if (use_last) {
    y1 = y2;
    use_last = 0;
  } else {
    do {
      x1 = 2.0 * ranf() - 1.0;
      x2 = 2.0 * ranf() - 1.0;
      w = x1 * x1 + x2 * x2;
    } while (w >= 1.0);

    w = sqrt((-2.0 * log(w)) / w);
    y1 = x1 * w;
    y2 = x2 * w;
    use_last = 1;
  }

  return (m + y1 * s);
}

float Distance(float *Pt1, float *Pt2) {
  float Dist_f;
  double Dist_d;

  Dist_d = (Pt2[0] - Pt1[0]) * (Pt2[0] - Pt1[0]) + (Pt2[1] - Pt1[1]) * (Pt2[1] - Pt1[1]) +
           (Pt2[2] - Pt1[2]) * (Pt2[2] - Pt1[2]);
  Dist_f = (float)sqrt(Dist_d);
  return Dist_f;
}

void smooth_geometry(CVC_NAMESPACE::app &ctx, CVC_NAMESPACE::geometry &geo, float delta,
                     bool fix_boundary, bool perturb_1, bool geometric_flow, bool smoothing_enabled,
                     bool perturb_2) {
  using namespace std;
  using namespace boost;
  using namespace CVC_NAMESPACE;

  vector<float> Vertices_gf;
  vector<float> VerticesBackup_gf;
  vector<float> VertexNormal_gf;
  vector<int> FaceIndex_gi;

  typedef vector<int> TriangleIndexVec;
  typedef vector<TriangleIndexVec> SharedTriangleIndexVec;
  SharedTriangleIndexVec SharedTriangleIndex_gi; // Shared triangle indexes of a vertex

  vector<unsigned char> FixedVertices_guc;

  int NumVertices_i, NumTriangles_i, i, j, k, v0, v1, v2;
  int itri, index, NumNeighborVertices_i;
  float vx, vy, vz, Pt0[3], Pt1[3], Pt2[3], WeightedCenter[3];
  float sum_0, sum_1[3], Normal_f[3], delta_t, area, t;
  float Min_f[3], Max_f[3], Half_f[3], Length_f;
  float AveCenter_f[3];

  thread_info ti(ctx, BOOST_CURRENT_FUNCTION);

  delta_t = delta;

  NumVertices_i = geo.num_points();
  NumTriangles_i = geo.num_tris();

  if (NumVertices_i == 0 || NumTriangles_i == 0)
    return;

  // initMem
  Vertices_gf.resize(NumVertices_i * 3);
  BOOST_FOREACH (float &v, Vertices_gf)
    v = 0.0f;
  VerticesBackup_gf.resize(NumVertices_i * 3);
  BOOST_FOREACH (float &v, VerticesBackup_gf)
    v = 0.0f;
  VertexNormal_gf.resize(NumVertices_i * 3);

  // Initialize normals from existing geometry normals if available, otherwise zero
  const auto &existing_normals = geo.const_normals();
  if (existing_normals.size() >= static_cast<size_t>(NumVertices_i)) {
    // Use existing normals as starting point
    for (i = 0; i < NumVertices_i; i++) {
      VertexNormal_gf[i * 3 + 0] = static_cast<float>(existing_normals[i][0]);
      VertexNormal_gf[i * 3 + 1] = static_cast<float>(existing_normals[i][1]);
      VertexNormal_gf[i * 3 + 2] = static_cast<float>(existing_normals[i][2]);
    }
  } else {
    // Initialize to zero - normals will be computed from faces later
    BOOST_FOREACH (float &v, VertexNormal_gf)
      v = 0.0f;
  }
  FaceIndex_gi.resize(NumTriangles_i * 3);
  BOOST_FOREACH (int &v, FaceIndex_gi)
    v = -1;
  FixedVertices_guc.resize(NumVertices_i);
  BOOST_FOREACH (unsigned char &v, FixedVertices_guc)
    v = 0;
  SharedTriangleIndex_gi.resize(NumVertices_i);

  for (k = 0; k < 3; k++)
    Min_f[k] = numeric_limits<float>::max();
  for (k = 0; k < 3; k++)
    Max_f[k] = -numeric_limits<float>::max();

  for (i = 0; i < NumVertices_i; i++) {

    vx = geo.const_points()[i][0];
    vy = geo.const_points()[i][1];
    vz = geo.const_points()[i][2];

    Vertices_gf[i * 3 + 0] = VerticesBackup_gf[i * 3 + 0] = vx;
    Vertices_gf[i * 3 + 1] = VerticesBackup_gf[i * 3 + 1] = vy;
    Vertices_gf[i * 3 + 2] = VerticesBackup_gf[i * 3 + 2] = vz;

    for (k = 0; k < 3; k++)
      if (Min_f[k] > Vertices_gf[i * 3 + k])
        Min_f[k] = Vertices_gf[i * 3 + k];
    for (k = 0; k < 3; k++)
      if (Max_f[k] < Vertices_gf[i * 3 + k])
        Max_f[k] = Vertices_gf[i * 3 + k];
  }

  for (k = 0; k < 3; k++)
    Half_f[k] = (Min_f[k] + Max_f[k]) / 2;

  ctx.log(3, str(boost::format("Min = %.4f %.4f %.4f\n") % Min_f[0] % Min_f[1] % Min_f[2]));
  ctx.log(3, str(boost::format("Max = %.4f %.4f %.4f\n") % Max_f[0] % Max_f[1] % Max_f[2]));
  ctx.log(3, str(boost::format("Half = %.4f %.4f %.4f\n") % Half_f[0] % Half_f[1] % Half_f[2]));

  for (i = 0; i < NumTriangles_i; i++) {

    v0 = geo.const_tris()[i][0];
    v1 = geo.const_tris()[i][1];
    v2 = geo.const_tris()[i][2];

    FaceIndex_gi[i * 3 + 0] = v0;
    FaceIndex_gi[i * 3 + 1] = v1;
    FaceIndex_gi[i * 3 + 2] = v2;

    SharedTriangleIndex_gi[v0].push_back(i);
    SharedTriangleIndex_gi[v1].push_back(i);
    SharedTriangleIndex_gi[v2].push_back(i);
  }

  if (fix_boundary) {
    thread_info ti(ctx, "fix_boundary");
    map<int, unsigned char> NeighborVertices_m;
    map<int, unsigned char>::iterator NeighborVertices_it;
    int NumFixedVertices_i = 0, NumNeighborTriangles_i;

    // Check if boundary markers are explicitly set in the geometry
    bool has_explicit_boundary =
        !geo.const_boundary().empty() && geo.const_boundary().size() == NumVertices_i;

    if (has_explicit_boundary) {
      // Use explicit boundary markers from geometry
      ctx.log(3, "Using explicit boundary markers...\n");
      for (i = 0; i < NumVertices_i; i++) {
        FixedVertices_guc[i] = geo.const_boundary()[i];
        if (FixedVertices_guc[i])
          NumFixedVertices_i++;
      }
    } else {
      // Auto-detect boundary vertices topologically
      ctx.log(3, "Finding fixed vertices ...\n");
      for (i = 0; i < NumVertices_i; i++) {
        NumNeighborTriangles_i = 0;
        NeighborVertices_m.clear();

        for (TriangleIndexVec::const_iterator j = SharedTriangleIndex_gi[i].begin();
             j != SharedTriangleIndex_gi[i].end(); j++) {
          itri = *j;
          v0 = FaceIndex_gi[itri * 3 + 0];
          v1 = FaceIndex_gi[itri * 3 + 1];
          v2 = FaceIndex_gi[itri * 3 + 2];
          NeighborVertices_m[v0] = 1;
          NeighborVertices_m[v1] = 1;
          NeighborVertices_m[v2] = 1;
          NumNeighborTriangles_i++;
        }

        if ((int)NeighborVertices_m.size() - 1 == NumNeighborTriangles_i)
          FixedVertices_guc[i] = 0;
        else {
          FixedVertices_guc[i] = 1; // Fixed vertex
          NumFixedVertices_i++;
        }
      }
    }
    ctx.threadProgress(float(NumFixedVertices_i) / float(NumVertices_i));
  } else {
    for (i = 0; i < NumVertices_i; i++)
      FixedVertices_guc[i] = 0;
  }

  if (perturb_1) {
    thread_info ti(ctx, "PERTURB_1");
    // calculate the normal for each vertex
    for (i = 0; i < NumVertices_i; i++) {
      for (k = 0; k < 3; k++)
        VertexNormal_gf[i * 3 + k] = 0.0f;
    }
    for (i = 0; i < NumVertices_i; i++) {

      if (FixedVertices_guc[i] == 1)
        continue;

      for (TriangleIndexVec::const_iterator j = SharedTriangleIndex_gi[i].begin();
           j != SharedTriangleIndex_gi[i].end(); j++) {
        itri = *j;
        v0 = FaceIndex_gi[itri * 3 + 0];
        v1 = FaceIndex_gi[itri * 3 + 1];
        v2 = FaceIndex_gi[itri * 3 + 2];
        for (k = 0; k < 3; k++) {
          Pt0[k] = Vertices_gf[v0 * 3 + k];
          Pt1[k] = Vertices_gf[v1 * 3 + k];
          Pt2[k] = Vertices_gf[v2 * 3 + k];
        }
        crossproduct(Pt0, Pt1, Pt2, Normal_f);
        for (k = 0; k < 3; k++)
          VertexNormal_gf[i * 3 + k] += Normal_f[k];
      }

      Length_f = Normalize(&VertexNormal_gf[i * 3]);
      if (fabs(Length_f) < 1e-4) {
        ctx.log(3, "Error! Length is equal to zero\n");
      }
    }

    float EdgeLength_f[3], MaxDist_f, Perturb_f;
    for (i = 0; i < NumVertices_i; i++) {

      for (k = 0; k < 3; k++) {
        Pt0[k] = Vertices_gf[v0 * 3 + k];
        Pt1[k] = Vertices_gf[v1 * 3 + k];
        Pt2[k] = Vertices_gf[v2 * 3 + k];
      }
      EdgeLength_f[0] = Distance(Pt0, Pt1);
      EdgeLength_f[1] = Distance(Pt0, Pt2);
      EdgeLength_f[2] = Distance(Pt1, Pt2);
      MaxDist_f = -1.0;
      for (k = 0; k < 3; k++) {
        if (MaxDist_f < EdgeLength_f[k])
          MaxDist_f = EdgeLength_f[k];
      }

      Perturb_f = ranf(-MaxDist_f / 2, MaxDist_f / 2);
      for (k = 0; k < 3; k++) {
        Vertices_gf[i * 3 + k] += Perturb_f * VertexNormal_gf[i * 3 + k];
      }
    }
  }

  if (geometric_flow) {
    // Rounding sharp edges
    thread_info ti(ctx, "GEOMETRIC_FLOW");

    // Adjusting each triangle size
    for (index = 0; index < maxIndex; index++) {

      // calculate the normal for each vertex
#ifdef _OPENMP
#pragma omp parallel for schedule(static)
#endif
      for (i = 0; i < NumVertices_i; i++) {
        for (k = 0; k < 3; k++)
          VertexNormal_gf[i * 3 + k] = 0.0f;
      }
#ifdef _OPENMP
#pragma omp parallel for private(itri, v0, v1, v2, k, Pt0, Pt1, Pt2, Normal_f, Length_f)           \
    schedule(dynamic)
#endif
      for (i = 0; i < NumVertices_i; i++) {

        for (TriangleIndexVec::const_iterator j = SharedTriangleIndex_gi[i].begin();
             j != SharedTriangleIndex_gi[i].end(); j++) {
          itri = *j;
          v0 = FaceIndex_gi[itri * 3 + 0];
          v1 = FaceIndex_gi[itri * 3 + 1];
          v2 = FaceIndex_gi[itri * 3 + 2];
          for (k = 0; k < 3; k++) {
            Pt0[k] = Vertices_gf[v0 * 3 + k];
            Pt1[k] = Vertices_gf[v1 * 3 + k];
            Pt2[k] = Vertices_gf[v2 * 3 + k];
          }
          crossproduct(Pt0, Pt1, Pt2, Normal_f);
          for (k = 0; k < 3; k++)
            VertexNormal_gf[i * 3 + k] += Normal_f[k];
        }

        Length_f = Normalize(&VertexNormal_gf[i * 3]);
        if (fabs(Length_f) < 1e-4) {
          ctx.log(3, "Error! Length is equal to zero\n");
        }
      }

#ifdef _OPENMP
#pragma omp parallel for private(itri, v0, v1, v2, k, Pt0, Pt1, Pt2, area, sum_0, sum_1,           \
                                     WeightedCenter, t) schedule(dynamic)
#endif
      for (i = 0; i < NumVertices_i; i++) {

        if (FixedVertices_guc[i] == 1)
          continue;

        // calculate the mass center WeightedCenter[3]
        sum_0 = 0.0f;
        for (k = 0; k < 3; k++) {
          WeightedCenter[k] = 0.0f;
          sum_1[k] = 0.0f;
        }

        for (TriangleIndexVec::const_iterator j = SharedTriangleIndex_gi[i].begin();
             j != SharedTriangleIndex_gi[i].end(); j++) {
          itri = *j;
          v0 = FaceIndex_gi[itri * 3 + 0];
          v1 = FaceIndex_gi[itri * 3 + 1];
          v2 = FaceIndex_gi[itri * 3 + 2];
          for (k = 0; k < 3; k++) {
            Pt0[k] = Vertices_gf[v0 * 3 + k];
            Pt1[k] = Vertices_gf[v1 * 3 + k];
            Pt2[k] = Vertices_gf[v2 * 3 + k];
          }

          area = area_tri(Pt0, Pt1, Pt2);
          sum_0 += area;
          for (k = 0; k < 3; k++) {
            WeightedCenter[k] += (Pt0[k] + Pt1[k] + Pt2[k]) * area / 3.0f;
          }
        }

        for (k = 0; k < 3; k++)
          WeightedCenter[k] /= sum_0;

        // calculate the new position in tangent direction
        // xi+1 = xi + delta_t*((m-xi) - (n, m-xi)n))
        t = 0.0f;
        for (k = 0; k < 3; k++) {
          WeightedCenter[k] -= Vertices_gf[i * 3 + k];
          t += WeightedCenter[k] * VertexNormal_gf[i * 3 + k];
        }
        for (k = 0; k < 3; k++) {
          Vertices_gf[i * 3 + k] += delta_t * WeightedCenter[k]; // mass center
          // Vertices_gf[i*3 + k] += delta_t*(WeightedCenter[k] - t*VertexNormal_gf[i*3 + k]);//
          // tangent movement
        }
      }

    } // end of index loop
  }

  if (smoothing_enabled) {
    thread_info ti(ctx, "SMOOTHING");

    // Smoothing
    for (index = 0; index < maxIndex; index++) {

      // calculate the normal for each vertex
#ifdef _OPENMP
#pragma omp parallel for schedule(static)
#endif
      for (i = 0; i < NumVertices_i; i++) {
        for (k = 0; k < 3; k++)
          VertexNormal_gf[i * 3 + k] = 0.0f;
      }
#ifdef _OPENMP
#pragma omp parallel for private(itri, v0, v1, v2, k, Pt0, Pt1, Pt2, Normal_f, Length_f)           \
    schedule(dynamic)
#endif
      for (i = 0; i < NumVertices_i; i++) {

        for (TriangleIndexVec::const_iterator j = SharedTriangleIndex_gi[i].begin();
             j != SharedTriangleIndex_gi[i].end(); j++) {
          itri = *j;
          v0 = FaceIndex_gi[itri * 3 + 0];
          v1 = FaceIndex_gi[itri * 3 + 1];
          v2 = FaceIndex_gi[itri * 3 + 2];
          for (k = 0; k < 3; k++) {
            Pt0[k] = Vertices_gf[v0 * 3 + k];
            Pt1[k] = Vertices_gf[v1 * 3 + k];
            Pt2[k] = Vertices_gf[v2 * 3 + k];
          }
          crossproduct(Pt0, Pt1, Pt2, Normal_f);
          for (k = 0; k < 3; k++)
            VertexNormal_gf[i * 3 + k] += Normal_f[k];
        }

        Length_f = Normalize(&VertexNormal_gf[i * 3]);
        if (fabs(Length_f) < 1e-4) {
          ctx.log(3, "Error! Length is equal to zero\n");
        }
      }

#ifdef _OPENMP
#pragma omp parallel for private(itri, v0, v1, v2, k, Pt0, Pt1, Pt2, AveCenter_f,                  \
                                     NumNeighborVertices_i) schedule(dynamic)
#endif
      for (i = 0; i < NumVertices_i; i++) {

        if (FixedVertices_guc[i] == 1)
          continue;
        NumNeighborVertices_i = 0;
        for (k = 0; k < 3; k++)
          AveCenter_f[k] = 0.0f;

        for (TriangleIndexVec::const_iterator j = SharedTriangleIndex_gi[i].begin();
             j != SharedTriangleIndex_gi[i].end(); j++) {
          itri = *j;
          v0 = FaceIndex_gi[itri * 3 + 0];
          v1 = FaceIndex_gi[itri * 3 + 1];
          v2 = FaceIndex_gi[itri * 3 + 2];
          for (k = 0; k < 3; k++) {
            Pt0[k] = Vertices_gf[v0 * 3 + k];
            Pt1[k] = Vertices_gf[v1 * 3 + k];
            Pt2[k] = Vertices_gf[v2 * 3 + k];
          }

          for (k = 0; k < 3; k++) {
            AveCenter_f[k] += (Pt0[k] + Pt1[k] + Pt2[k]) / 3.0f;
          }
          NumNeighborVertices_i++;
        }

        for (k = 0; k < 3; k++) {
          AveCenter_f[k] /= (float)NumNeighborVertices_i;
        }

        for (k = 0; k < 3; k++) {
          Vertices_gf[i * 3 + k] = AveCenter_f[k];
        }
      }

    } // end of index loop
  }

  if (perturb_2) {
    thread_info ti(ctx, "PERTURB_2");
    // calculate the normal for each vertex
#ifdef _OPENMP
#pragma omp parallel for schedule(static)
#endif
    for (i = 0; i < NumVertices_i; i++) {
      for (k = 0; k < 3; k++)
        VertexNormal_gf[i * 3 + k] = 0.0f;
    }
#ifdef _OPENMP
#pragma omp parallel for private(itri, v0, v1, v2, k, Pt0, Pt1, Pt2, Normal_f, Length_f)           \
    schedule(dynamic)
#endif
    for (i = 0; i < NumVertices_i; i++) {

      if (FixedVertices_guc[i] == 1)
        continue;

      for (TriangleIndexVec::const_iterator j = SharedTriangleIndex_gi[i].begin();
           j != SharedTriangleIndex_gi[i].end(); j++) {
        itri = *j;
        v0 = FaceIndex_gi[itri * 3 + 0];
        v1 = FaceIndex_gi[itri * 3 + 1];
        v2 = FaceIndex_gi[itri * 3 + 2];
        for (k = 0; k < 3; k++) {
          Pt0[k] = Vertices_gf[v0 * 3 + k];
          Pt1[k] = Vertices_gf[v1 * 3 + k];
          Pt2[k] = Vertices_gf[v2 * 3 + k];
        }
        crossproduct(Pt0, Pt1, Pt2, Normal_f);
        for (k = 0; k < 3; k++)
          VertexNormal_gf[i * 3 + k] += Normal_f[k];
      }

      Length_f = Normalize(&VertexNormal_gf[i * 3]);
      if (fabs(Length_f) < 1e-4) {
        ctx.log(3, "Error! Length is equal to zero\n");
      }
    }

    float EdgeLength_f[3], MaxDist_f, Perturb_f;
#ifdef _OPENMP
#pragma omp parallel for private(k, Pt0, Pt1, Pt2, EdgeLength_f, MaxDist_f, Perturb_f)             \
    schedule(dynamic)
#endif
    for (i = 0; i < NumVertices_i; i++) {

      for (k = 0; k < 3; k++) {
        Pt0[k] = Vertices_gf[v0 * 3 + k];
        Pt1[k] = Vertices_gf[v1 * 3 + k];
        Pt2[k] = Vertices_gf[v2 * 3 + k];
      }
      EdgeLength_f[0] = Distance(Pt0, Pt1);
      EdgeLength_f[1] = Distance(Pt0, Pt2);
      EdgeLength_f[2] = Distance(Pt1, Pt2);
      MaxDist_f = -1.0;
      for (k = 0; k < 3; k++) {
        if (MaxDist_f < EdgeLength_f[k])
          MaxDist_f = EdgeLength_f[k];
      }

      Perturb_f = ranf(-MaxDist_f / 5, MaxDist_f / 5);
      for (k = 0; k < 3; k++) {
        Vertices_gf[i * 3 + k] += Perturb_f * VertexNormal_gf[i * 3 + k];
      }
    }
  }

  points_t &pts = geo.points();
  normals_t &norm = geo.normals();

  // Ensure normals array is sized to match points
  // This prevents crashes when geometry has no normals or mismatched sizes
  if (norm.size() != static_cast<size_t>(NumVertices_i))
    norm.resize(NumVertices_i);

#ifdef _OPENMP
#pragma omp parallel for schedule(static)
#endif
  for (i = 0; i < NumVertices_i; i++) {
    // use old vertex if smoothing produced garbage
    if (isnan(Vertices_gf[i * 3]) || isnan(Vertices_gf[i * 3 + 1]) ||
        isnan(Vertices_gf[i * 3 + 2])) {
      pts[i][0] = VerticesBackup_gf[i * 3 + 0];
      pts[i][1] = VerticesBackup_gf[i * 3 + 1];
      pts[i][2] = VerticesBackup_gf[i * 3 + 2];
    } else {
      pts[i][0] = Vertices_gf[i * 3 + 0];
      pts[i][1] = Vertices_gf[i * 3 + 1];
      pts[i][2] = Vertices_gf[i * 3 + 2];
    }

    norm[i][0] = VertexNormal_gf[i * 3 + 0];
    norm[i][1] = VertexNormal_gf[i * 3 + 1];
    norm[i][2] = VertexNormal_gf[i * 3 + 2];
  }

  geo.points() = pts;
  geo.normals() = norm;
}
}; // namespace

namespace CVC_NAMESPACE {
geometry &geometry::smoothing(float delta, bool fix_boundary, bool perturb_1, bool geometric_flow,
                              bool smoothing_enabled, bool perturb_2) {
  smooth_geometry(ctx(), *this, delta, fix_boundary, perturb_1, geometric_flow, smoothing_enabled,
                  perturb_2);
  return *this;
}
} // namespace CVC_NAMESPACE
