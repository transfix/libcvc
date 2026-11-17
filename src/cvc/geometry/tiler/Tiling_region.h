#ifndef __TILING_REGION_H__
#define __TILING_REGION_H__

#include <limits>

#include <boost/shared_ptr.hpp>

#include <CGAL/Cartesian.h>
#include <CGAL/enum.h>

#include <ContourTiler/common.h>

CONTOURTILER_BEGIN_NAMESPACE

class Tiling_region;
typedef boost::shared_ptr<const Tiling_region> HTiling_region;

class Tiling_region
{
public:
  static const Number_type AMBIGUOUS_Z_HOME;

  static HTiling_region overlapping_vertex(const Point_2& p, const Point_2& q, const Point_2& r,
					   const Point_2& a, const Point_2& b, const Point_2& c);


public:
  Tiling_region(Number_type z_home);

  virtual ~Tiling_region();

  virtual bool contains(const Point_3& point) const = 0;

  virtual bool contains(const Segment_3& segment) const = 0;

  virtual HTiling_region get_complement() const = 0;

  bool has_z_home() const;

  Number_type z_home() const;

  // This version returns NaN if there is no z_home.
  Number_type z_home_nothrow() const;

  virtual std::ostream& print(std::ostream& out) const = 0;

  friend HTiling_region operator&(HTiling_region a, HTiling_region b);

  friend HTiling_region operator|(HTiling_region a, HTiling_region b);

  friend std::ostream& operator<<(std::ostream& out, const Tiling_region& region)
  { region.print(out); return out; }

private:
  Number_type _z_home;
};

CONTOURTILER_END_NAMESPACE

#endif
