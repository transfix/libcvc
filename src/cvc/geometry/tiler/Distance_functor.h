#ifndef __DISTANCE_FUNCTOR_H__
#define __DISTANCE_FUNCTOR_H__

#include <ContourTiler/common.h>

CONTOURTILER_BEGIN_NAMESPACE

//------------------------------------------------------------------------------
// Distance_functor class
//------------------------------------------------------------------------------
template <typename Point>
class Distance_functor
{
public:
  Distance_functor()
  {}

  Distance_functor(const Point& s)
  { _source = s; }

  bool operator()( const Point& a, const Point& b ) const
  { return CGAL::has_larger_distance_to_point(_source, b, a); }

private:
  Point _source;
};

//------------------------------------------------------------------------------
// dist_functor
//
/// Utility function to create a distance functor based on a point.
//------------------------------------------------------------------------------
template <typename Point>
Distance_functor<Point> dist_functor(const Point& point)
{
  return Distance_functor<Point>(point);
}

typedef Distance_functor<Point_2> Distance_functor_2;
typedef Distance_functor<Point_3> Distance_functor_3;

CONTOURTILER_END_NAMESPACE

#endif
