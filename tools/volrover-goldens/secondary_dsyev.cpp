// LAPACK's dsyev_ for legacy SecondaryStructures (helix.cpp fits each helix's axis with it), so
// this build needs no LAPACK: the eigenvalues of a symmetric n x n matrix, ascending in w, and with
// jobz 'V' its eigenvectors as a's columns (column-major, leading dimension lda), by cyclic Jacobi
// rotations. A workspace query (lwork -1) answers 1. An eigenvector's sign is LAPACK's to choose,
// and differs here; the axis it gives is the same line. Part of tools/volrover-goldens: never built
// into libcvc.

#include <algorithm>
#include <cmath>
#include <numeric>
#include <vector>

extern "C" void dsyev_(char *jobz, char *uplo, int *n, double *a, int *lda, double *w, double *work,
                       int *lwork, int *info) {
  *info = 0;
  if (*lwork == -1) {
    work[0] = 1;
    return;
  }
  const int N = *n, L = *lda;
  const bool upper = (*uplo == 'U' || *uplo == 'u');
  std::vector<double> m(std::size_t(N) * N), v(std::size_t(N) * N, 0.0);
  auto M = [&](int i, int j) -> double & { return m[std::size_t(i) * N + j]; };
  auto V = [&](int i, int j) -> double & { return v[std::size_t(i) * N + j]; };
  for (int j = 0; j < N; ++j)
    for (int i = 0; i < N; ++i) {
      const bool stored = upper ? i <= j : i >= j;
      const double x = stored ? a[i + std::size_t(j) * L] : a[j + std::size_t(i) * L];
      M(i, j) = x;
    }
  for (int i = 0; i < N; ++i)
    V(i, i) = 1.0;
  for (int sweep = 0; sweep < 64; ++sweep) {
    double off = 0, diag = 0;
    for (int i = 0; i < N; ++i) {
      diag += std::fabs(M(i, i));
      for (int j = i + 1; j < N; ++j)
        off += std::fabs(M(i, j));
    }
    if (off == 0 || off <= 1e-18 * diag)
      break;
    for (int p = 0; p < N - 1; ++p)
      for (int q = p + 1; q < N; ++q) {
        if (M(p, q) == 0)
          continue;
        const double theta = (M(q, q) - M(p, p)) / (2.0 * M(p, q));
        const double t =
            std::fabs(theta) > 1e150
                ? 0.5 / theta
                : (theta >= 0 ? 1.0 : -1.0) / (std::fabs(theta) + std::sqrt(theta * theta + 1));
        const double c = 1.0 / std::sqrt(t * t + 1), s = t * c;
        const double apq = M(p, q);
        M(p, p) -= t * apq;
        M(q, q) += t * apq;
        M(p, q) = M(q, p) = 0.0;
        for (int r = 0; r < N; ++r) {
          if (r == p || r == q)
            continue;
          const double arp = M(r, p), arq = M(r, q);
          M(r, p) = M(p, r) = c * arp - s * arq;
          M(r, q) = M(q, r) = c * arq + s * arp;
        }
        for (int r = 0; r < N; ++r) {
          const double vrp = V(r, p), vrq = V(r, q);
          V(r, p) = c * vrp - s * vrq;
          V(r, q) = s * vrp + c * vrq;
        }
      }
  }
  std::vector<int> order(static_cast<std::size_t>(N));
  std::iota(order.begin(), order.end(), 0);
  std::stable_sort(order.begin(), order.end(), [&](int x, int y) { return M(x, x) < M(y, y); });
  const bool vectors = (*jobz == 'V' || *jobz == 'v');
  for (int k = 0; k < N; ++k) {
    w[k] = M(order[std::size_t(k)], order[std::size_t(k)]);
    if (vectors)
      for (int i = 0; i < N; ++i)
        a[i + std::size_t(k) * L] = V(i, order[std::size_t(k)]);
  }
}
