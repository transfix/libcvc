#ifndef __KERNEL_UTILS_H__
#define __KERNEL_UTILS_H__

#include <ContourTiler/common.h>
#include <ContourTiler/bso_rational_nt.h>

CONTOURTILER_BEGIN_NAMESPACE

//---------------------------------------------------------------------
// Useful kernels with their defined polygons etc
//---------------------------------------------------------------------

// bso
struct Bso_kernel : public CGAL::Cartesian<Bso_number_type> {};
typedef Bso_kernel::Point_2                            Bso_point_2;
typedef Bso_kernel::Segment_2                          Bso_segment_2;
typedef CGAL::Polygon_2<Bso_kernel>                    Bso_polygon_2;
typedef CGAL::Polygon_with_holes_2<Bso_kernel>         Bso_polygon_with_holes_2;
typedef Bso_kernel::FT                                 Bso_FT;

// exact predicates inexact constructions
typedef CGAL::Exact_predicates_inexact_constructions_kernel Epic_kernel;
typedef Epic_kernel::Point_2                                Epic_point_2;
typedef CGAL::Polygon_2<Epic_kernel>                        Epic_polygon_2;
typedef CGAL::Polygon_with_holes_2<Epic_kernel>             Epic_polygon_with_holes_2;
typedef Epic_kernel::FT                                     Epic_FT;


//---------------------------------------------------------------------
// Useful conversion routines
//---------------------------------------------------------------------

template <typename K>
typename K::Point_2 change_kernel(const Point_2& p)
{
  return typename K::Point_2(p.x(), p.y());
}

template <typename K>
typename K::Segment_2 change_kernel(const Segment_2& s)
{
  return typename K::Segment_2(change_kernel<K>(s.source()), change_kernel<K>(s.target()));
}

template <typename K>
typename K::Ray_2 change_kernel(const Ray_2& r)
{
  return typename K::Ray_2(change_kernel<K>(r.source()), change_kernel<K>(r.point(1)));
}

template <typename K>
CGAL::Polygon_2<K> change_kernel(const Polygon_2& P)
{
  CGAL::Polygon_2<K> new_P;
  for (Polygon_2::Vertex_iterator it = P.vertices_begin(); it != P.vertices_end(); ++it) {
    new_P.push_back(typename K::Point_2(it->x(), it->y()));
  }
  return new_P;
}

template <typename K>
CGAL::Polygon_with_holes_2<K> change_kernel(const Polygon_with_holes_2& P)
{
  CGAL::Polygon_with_holes_2<K> new_P(change_kernel<K>(P.outer_boundary()));
  for (Polygon_with_holes_2::Hole_const_iterator it = P.holes_begin(); it != P.holes_end(); ++it) {
    new_P.add_hole(change_kernel<K>(*it));
  }
  return new_P;
}

template <typename K>
Point_2 to_common(const typename K::Point_2& s, Number_type z)
{
  return Point_2(CGAL::to_double(s.x()), CGAL::to_double(s.y()), z);
}

template <typename K>
Segment_2 to_common(const typename K::Segment_2& s, Number_type z)
{
  return Segment_2(to_common<K>(s.source(), z), to_common<K>(s.target(), z));
}

template <typename K>
Ray_2 to_common(const typename K::Ray_2& r, Number_type z)
{
  return Ray_2(to_common<K>(r.source(), z), to_common(r.point(1), z));
}

template <typename K>
Polygon_2 to_common(const CGAL::Polygon_2<K>& P, Number_type z)
{
  Polygon_2 p;
  for (typename CGAL::Polygon_2<K>::Vertex_iterator it = P.vertices_begin(); it != P.vertices_end(); ++it) {
    p.push_back(Point_2(CGAL::to_double(it->x()), CGAL::to_double(it->y()), z));
  }
  return p;
}

template <typename K>
Polygon_with_holes_2 to_common(const CGAL::Polygon_with_holes_2<K>& P, Number_type z)
{
  Polygon_with_holes_2 p(to_common(P.outer_boundary(), z));
  for (typename CGAL::Polygon_with_holes_2<K>::Hole_const_iterator it = P.holes_begin(); it != P.holes_end(); ++it) {
    p.add_hole(to_common(*it, z));
  }
  return p;
}

CONTOURTILER_END_NAMESPACE

#endif
