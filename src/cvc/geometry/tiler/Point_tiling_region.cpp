#include <stdexcept>

#include <ContourTiler/Point_tiling_region.h>
#include <ContourTiler/print_utils.h>

CONTOURTILER_BEGIN_NAMESPACE

bool Point_tiling_region::contains(const Point_3& point) const
{
  if (_z)
    return _inverse == !xyz_equal(point, _point);
  return _inverse == !xy_equal(point, _point);
}

bool Point_tiling_region::contains(const Segment_3& segment) const
{
  if (_z) {
    if (_inverse)
      return !xyz_equal(segment.source(), _point) && !xyz_equal(segment.target(), _point);
    return xyz_equal(segment.source(), _point) || xyz_equal(segment.target(), _point);
  }
  if (_inverse)
    return !xy_equal(segment.source(), _point) && !xy_equal(segment.target(), _point);
  return xy_equal(segment.source(), _point) || xy_equal(segment.target(), _point);
}

HTiling_region Point_tiling_region::get_complement() const
{
  return HTiling_region(new Point_tiling_region(_point, !_inverse, _z, z_home_nothrow()));
}
std::ostream& Point_tiling_region::print(std::ostream& out) const
{
  out << *this;
  return out;
}

std::ostream& operator<<(std::ostream& out, const Point_tiling_region& region)
{
  out <<    
    "p = " << pp(region._point) << 
    " inverse = " << region._inverse;
  return out;
}

CONTOURTILER_END_NAMESPACE

