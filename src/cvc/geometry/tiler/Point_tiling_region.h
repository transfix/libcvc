#ifndef __POINT_TILING_REGION_H__
#define __POINT_TILING_REGION_H__

#include <boost/shared_ptr.hpp>

#include <CGAL/Cartesian.h>
#include <CGAL/enum.h>

#include <ContourTiler/common.h>
#include <ContourTiler/Tiling_region.h>

CONTOURTILER_BEGIN_NAMESPACE

class Point_tiling_region;
typedef boost::shared_ptr<Point_tiling_region> HPoint_tiling_region;

class Point_tiling_region : public Tiling_region
{
public:
  Point_tiling_region(Number_type z_home) : Tiling_region(z_home) {}

  Point_tiling_region(const Point_2& p, bool inverse, bool z, Number_type z_home) 
    : Tiling_region(z_home), _point(p), _inverse(inverse), _z(z) 
  {
  }

  virtual ~Point_tiling_region() {}

  virtual bool contains(const Point_3& point) const;

  virtual bool contains(const Segment_3& segment) const;

  virtual HTiling_region get_complement() const;

  virtual std::ostream& print(std::ostream& out) const;

  friend std::ostream& operator<<(std::ostream& out, const Point_tiling_region& region);

private:
  Point_2 _point;
  bool _inverse;
  bool _z;
};

CONTOURTILER_END_NAMESPACE

#endif
