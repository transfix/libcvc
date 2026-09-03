// Quadratic-B-spline (de Boor) gradient over the 4x4x4 voxel neighborhood --
// the port of vrSplineNorm.
//
// The legacy implementation kept the neighborhood cache as fields of the
// shared VolRenEnv (hoisted out of function statics "for reentrancy"), which
// is exactly what forced vrCopyEnv per OpenMP thread.  Here the cache is a
// plain stack object owned by each ray: reentrancy by construction, no
// copies, no shared mutable state.
//
// The arithmetic is kept identical to the legacy code (same difference
// tensors, same de Boor weights) so the port shades like the original.  The
// result is the UNNORMALIZED gradient in value-units-per-cell; callers use
// its magnitude for the gradient-opacity ramp and its direction (normalized)
// as the shading normal.
// Derived from volrover's volren/libiso (C) 2000-2005 University of Texas at
// Austin (Park/Zhang/Rivera, advisor Bajaj), LGPL 2.1 -- the same license as
// libcvc.
#ifndef CVC_VOLREN_DETAIL_SPLINE_GRADIENT_H
#define CVC_VOLREN_DETAIL_SPLINE_GRADIENT_H

#include <cvc/volren/detail/sampler.h>
#include <cvc/volren/types.h>

#include <cstdint>

namespace cvc {
namespace volren {
namespace detail {

class spline_gradient_cache {
public:
  // Gradient at local weights w[3] inside cell idx of `grid`.  The 4^3
  // neighborhood and difference tensors are cached and refreshed only when
  // the cell changes -- cheap for the many samples a ray takes in one cell.
  vec3d evaluate(const grid_sampler &grid, const std::int64_t idx[3], const float w[3]) {
    if (idx[0] != _cell[0] || idx[1] != _cell[1] || idx[2] != _cell[2]) {
      _cell[0] = idx[0];
      _cell[1] = idx[1];
      _cell[2] = idx[2];
      refresh(grid);
    }

    float d[4][4][2];
    float normal[3];
    // For each gradient component l: de Boor collapse of degree 2 along axis
    // l, degree 1 along the other two axes.
    for (int l = 0; l < 3; ++l) {
      for (int a = 1; a < 3; ++a) {
        for (int b = 1; b < 3; ++b) {
          float delta = 0.5f * (1.0f - w[l]);
          d[a][b][0] = delta * _deriv[a][b][0][l] + (1.0f - delta) * _deriv[a][b][1][l];
          delta = 0.5f * (2.0f - w[l]);
          d[a][b][1] = delta * _deriv[a][b][1][l] + (1.0f - delta) * _deriv[a][b][2][l];

          delta = 1.0f - w[l];
          d[a][b][0] = delta * d[a][b][0] + (1.0f - delta) * d[a][b][1];
        }
      }
      int m = (l + 1) % 3;
      for (int a = 1; a < 3; ++a) {
        const float delta = 1.0f - w[m];
        d[a][0][0] = delta * d[a][1][0] + (1.0f - delta) * d[a][2][0];
      }
      m = (l + 2) % 3;
      const float delta = 1.0f - w[m];
      normal[l] = delta * d[1][0][0] + (1.0f - delta) * d[2][0][0];
    }
    return {normal[0], normal[1], normal[2]};
  }

private:
  void refresh(const grid_sampler &grid) {
    // _val[z][y][x] holds voxels (cell + offset - 1) for offsets 0..3,
    // edge-clamped like the legacy fill.
    for (int k = -1; k <= 2; ++k)
      for (int j = -1; j <= 2; ++j)
        for (int i = -1; i <= 2; ++i)
          _val[k + 1][j + 1][i + 1] =
              grid.at_clamped(_cell[0] + i, _cell[1] + j, _cell[2] + k);

    // Forward-difference tensors, one per gradient axis, laid out exactly as
    // the legacy code stored them (the evaluate() index dance depends on it):
    //   _deriv[z][y][xgap][0] : x-differences
    //   _deriv[x][z][ygap][1] : y-differences
    //   _deriv[y][x][zgap][2] : z-differences
    for (int g = 0; g < 3; ++g)
      for (int a = 1; a < 3; ++a)
        for (int b = 1; b < 3; ++b)
          _deriv[a][b][g][0] = _val[a][b][g + 1] - _val[a][b][g];

    for (int g = 1; g < 3; ++g)
      for (int j = 0; j < 3; ++j)
        for (int a = 1; a < 3; ++a)
          _deriv[g][a][j][1] = _val[a][j + 1][g] - _val[a][j][g];

    for (int g = 1; g < 3; ++g)
      for (int b = 1; b < 3; ++b)
        for (int i = 0; i < 3; ++i)
          _deriv[b][g][i][2] = _val[i + 1][b][g] - _val[i][b][g];
  }

  std::int64_t _cell[3] = {-1, -1, -1};
  float _val[4][4][4] = {};
  float _deriv[4][4][3][3] = {};
};

} // namespace detail
} // namespace volren
} // namespace cvc

#endif // CVC_VOLREN_DETAIL_SPLINE_GRADIENT_H
