#ifndef __XY_EQUAL_PREDICATE_H__
#define __XY_EQUAL_PREDICATE_H__

#include <ContourTiler/common.h>

CONTOURTILER_BEGIN_NAMESPACE

//------------------------------------------------------------------------------
// XY_equal_predicate class
//------------------------------------------------------------------------------
template <typename Point>
class XY_equal_predicate
{
public:
  XY_equal_predicate()
  {}

  XY_equal_predicate(const Point& s)
  { _source = s; }

  bool operator()(const Point& p) const
  { return xy_equal(p, _source); }

private:
  Point _source;
};

//------------------------------------------------------------------------------
// dist_functor
//
/// Utility function to create a distance functor based on a point.
//------------------------------------------------------------------------------
template <typename Point>
XY_equal_predicate<Point> xy_equal_predicate(const Point& point)
{
  return XY_equal_predicate<Point>(point);
}

CONTOURTILER_END_NAMESPACE

#endif
