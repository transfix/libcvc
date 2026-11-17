#include <ContourTiler/segment_utils.h>

CONTOURTILER_BEGIN_NAMESPACE

Segment_2 lexicographically_ordered(const Segment_2& s)
{
  if (CGAL::lexicographically_xy_smaller(s.source(), s.target()))
    return s;
  return s.opposite();
}

Segment_3 lexicographically_ordered(const Segment_3& s)
{
  if (CGAL::lexicographically_xyz_smaller(s.source(), s.target()))
    return s;
  return s.opposite();
}

bool equal_ignore_order(const Segment_3& a, const Segment_3& b)
{
  return lexicographically_ordered(a) == lexicographically_ordered(b);
}

bool equal_ignore_order(const Segment_2& a, const Segment_3& b)
{
  return lexicographically_ordered(project_3(a)) == lexicographically_ordered(b);
}

Segment_3 project_3(const Segment_2& s)
{
  return Segment_3(s.source().point_3(), s.target().point_3());
}

Segment_2 project_2(const Segment_3& s)
{
  return Segment_2(s.source().point_2(), s.target().point_2());
}

CONTOURTILER_END_NAMESPACE
