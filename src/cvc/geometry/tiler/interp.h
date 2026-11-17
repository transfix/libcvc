#ifndef __INTERP_H__
#define __INTERP_H__

#include <ContourTiler/common.h>

CONTOURTILER_BEGIN_NAMESPACE

Number_type interpolate(const Polygon_2& P, const Point_3& p);

template <typename Poly_iter, typename Iter>
Number_type interpolate(Poly_iter poly_begin, Poly_iter poly_end, Iter points_begin, Iter points_end, Number_type zmid);

CONTOURTILER_END_NAMESPACE

#endif
