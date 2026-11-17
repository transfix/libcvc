#ifndef __OFFSET_POLYGON_H__
#define __OFFSET_POLYGON_H__

#include <ContourTiler/common.h>

CONTOURTILER_BEGIN_NAMESPACE

// Polygon_with_holes_2 offset_polygon(const Polygon_2& polygon, Number_type offset);

Polygon_with_holes_2 offset_polygon_positive(const Polygon_2& polygon, Number_type offset);

template <typename Out_iter>
void offset_polygon_negative(const Polygon_2& polygon, Number_type offset, Out_iter out);

// template <typename Out_iter>
// void offset_polygon(const Polygon_with_holes_2& polygon, Number_type offset, Out_iter out);

// template <typename Out_iter>
// void offset_polygon(const Polygon_2& polygon, Number_type offset, Out_iter out)
// {
//   offset_polygon(Polygon_with_holes_2(polygon), offset, out);
// }

CONTOURTILER_END_NAMESPACE

#endif
