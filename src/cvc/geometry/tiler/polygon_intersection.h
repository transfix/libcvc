#ifndef __POLYGON_INTERSECTION_H__
#define __POLYGON_INTERSECTION_H__

#include <ContourTiler/common.h>

CONTOURTILER_BEGIN_NAMESPACE

template <typename Out_iter>
void polygon_intersection(const Polygon_2& P, const Polygon_2& Q, 
			Out_iter out);

// template <typename Out_iter>
// void polygon_intersection(const Polygon_with_holes_2& P, const Polygon_with_holes_2& Q, 
// 			Out_iter P_out, Out_iter Q_out);

CONTOURTILER_END_NAMESPACE

#endif
