#ifndef __COMPOSITE_TILING_REGION_H__
#define __COMPOSITE_TILING_REGION_H__

#include <boost/shared_ptr.hpp>

#include <CGAL/Cartesian.h>
#include <CGAL/enum.h>

#include <ContourTiler/common.h>
#include <ContourTiler/Tiling_region.h>

CONTOURTILER_BEGIN_NAMESPACE

class Composite_tiling_region : public Tiling_region
{
public:
  static HTiling_region composite(HTiling_region a, HTiling_region b, bool and_, Number_type z_home)
  { return HTiling_region(new Composite_tiling_region(a, b, and_, z_home)); }

public:
  Composite_tiling_region(Number_type z_home);

  Composite_tiling_region(HTiling_region a, HTiling_region b, bool and_, Number_type z_home);

public:
  virtual ~Composite_tiling_region();

  virtual bool contains(const Point_3& point) const;

  virtual bool contains(const Segment_3& segment) const;

  virtual HTiling_region get_complement() const;

  virtual std::ostream& print(std::ostream& out) const;

  friend std::ostream& operator<<(std::ostream& out, const Composite_tiling_region& region);

private:
  HTiling_region _A;
  HTiling_region _B;
  bool _and;
};

CONTOURTILER_END_NAMESPACE

#endif
