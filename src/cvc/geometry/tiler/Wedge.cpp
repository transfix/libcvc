#include <stdexcept>

#include <CGAL/Direction_2.h>

#include <ContourTiler/Wedge.h>
#include <ContourTiler/print_utils.h>
#include <ContourTiler/projection.h>
#include <ContourTiler/Point_tiling_region.h>

CONTOURTILER_BEGIN_NAMESPACE

HWedge Wedge::whitelist(const Wedge& wedge, const Point_3& p)
{ 
  HTiling_region whitelisted(new Point_tiling_region(p, false, true, wedge.z_home_nothrow()));
  return HWedge(new Wedge(wedge, whitelisted)); 
}

typedef CGAL::Direction_2<Kernel> Direction_2;

bool Wedge::contains(const Point_3& point3) const
{
  Point_2 point = point3.point_2();

  if (_ray1.has_on(point) || _ray2.has_on(point))
    return _closed || whitelisted(point3);

  Ray_2 r(_ray1.source(), point);
  Direction_2 d = r.direction();
  return d.counterclockwise_in_between(_ray1.direction(), _ray2.direction());
}

bool Wedge::contains(const Segment_3& segment3) const
{
  Segment_2 segment = projection_z(segment3);

//   if (!xy_equal(segment.source(), vertex()) && !xy_equal(segment.target(), vertex()))
//     throw std::logic_error("At least one end of the segment must be on the wedge vertex");

  if (xy_equal(_ray1.source(), segment.source()))
    return contains(segment.target()) && (_closed || whitelisted(segment3.source()));
  if (xy_equal(_ray1.source(), segment.target()))
    return contains(segment.source()) && (_closed || whitelisted(segment3.target()));

//   if (_closed)
//   {
//     if (xy_equal(_ray1.source(), segment.source()))
//       return contains(segment.target());
//     if (xy_equal(_ray1.source(), segment.target()))
//       return contains(segment.source());
//   }

  if (!contains(segment.source()) || !contains(segment.target()))
    return false;
    
  int intersections = 0;
  CGAL::Object result;
  Point_2 ipoint;

  result = CGAL::intersection(_ray1, segment);
  if (CGAL::assign(ipoint, result))
  {
    if (!_closed || !is_boundary(ipoint, segment, _ray1))
      ++intersections;
  }
  result = CGAL::intersection(_ray2, segment);
  if (CGAL::assign(ipoint, result))
  {
    if (!_closed || !is_boundary(ipoint, segment, _ray2))
      ++intersections;
  }
  if (intersections > 0)
    return false;

  // One more check: it's possible that if _closed == true that both points
  // may be on the boundaries but the interior of the segment is out of bounds.
  if (_closed)
  {
    Point_2 mid((segment.source().x() + segment.target().x())/2.0, (segment.source().y() + segment.target().y())/2.0);
    return contains(mid);
  }
  return true;
}

HTiling_region Wedge::get_complement() const
{
  return HTiling_region(new Wedge(_ray2, _ray1, !_closed, z_home_nothrow()));
}

bool Wedge::is_empty_intersection(const Wedge& w) const
{
  if (!xy_equal(vertex(), w.vertex()))
    throw std::logic_error("Wedges must have common vertices");
  if (_closed != w._closed)
    throw std::logic_error("Wedges must both be closed or both be open");

  if (is_empty() || w.is_empty()) return true;
  
  if (_ray1 == w._ray1 || _ray2 == w._ray2) return false;

  if (_closed && (_ray1 == w._ray2 || _ray2 == w._ray1)) return false;

  Direction_2 d11 = _ray1.direction();
  Direction_2 d12 = _ray2.direction();
  Direction_2 d21 = w._ray1.direction();
  Direction_2 d22 = w._ray2.direction();

  if (d21.counterclockwise_in_between(d11, d12) ||
      d22.counterclockwise_in_between(d11, d12))
    return false;

  if (d11.counterclockwise_in_between(d21, d22) ||
      d12.counterclockwise_in_between(d21, d22))
    return false;

  return true;
}

// Wedge Wedge::intersection(const Wedge& w) const
// {
//   if (!xy_equal(vertex(), w.vertex()))
//     throw std::logic_error("Wedges must have common vertices");
//   if (_closed != w._closed)
//     throw std::logic_error("Wedges must both be closed or both be open");

//   Direction_2 d11 = _ray1.direction();
//   Direction_2 d12 = _ray2.direction();
//   Direction_2 d21 = w._ray1.direction();
//   Direction_2 d22 = w._ray2.direction();

//   if ((d21.counterclockwise_in_between(d11, d12) || w._ray1 == _ray1) && 
//       (d22.counterclockwise_in_between(d11, d12) || w._ray2 == _ray2)) {
//     return w;
//   }
//   if (d21.counterclockwise_in_between(d11, d12) || w._ray1 == _ray1) {
//     return Wedge(w._ray1, _ray2, _closed);
//   }
//   if (d22.counterclockwise_in_between(d11, d12) || w._ray2 == _ray2) {
//     return Wedge(_ray1, w._ray2, _closed);
//   }
//   if (d11.counterclockwise_in_between(d21, d22)) {
//     if (!d12.counterclockwise_in_between(d21, d22))
//       throw std::logic_error("Unexpected case");
//     return *this;
//   }
// //   return Wedge(Point_2(1, 0), Point_2(0, 0), Point_2(1, 0), false);
//   return Wedge::EMPTY();
// }

bool Wedge::is_boundary(const Point_2& p, const Segment_2& segment, const Ray_2& ray) const
{
  return xy_equal(p, segment.source()) || xy_equal(p, segment.target()) || xy_equal(p, ray.source());
}

bool Wedge::whitelisted(const Point_3& point3) const
{
  if (_whitelisted)
    return _whitelisted->contains(point3);
  return false;
}

bool operator==(const Wedge& a, const Wedge& b)
{
  return a._ray1 == b._ray1 && a._ray2 == b._ray2 && a._closed == b._closed;
}

std::ostream& Wedge::print(std::ostream& out) const
{
  out << *this;
  return out;
}

std::ostream& operator<<(std::ostream& out, const Wedge& wedge)
{
  out <<    
    "p = " << pp(wedge._ray2.point(1)) << 
    " q = " << pp(wedge.vertex()) << 
    " r = " << pp(wedge._ray1.point(1)) <<
    " closed = " << wedge._closed;
  return out;
}

CONTOURTILER_END_NAMESPACE

