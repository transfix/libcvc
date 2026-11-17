#ifndef __ALL_TILING_REGION_H__
#define __ALL_TILING_REGION_H__

#include <boost/shared_ptr.hpp>

#include <CGAL/Cartesian.h>
#include <CGAL/enum.h>

#include <ContourTiler/common.h>
#include <ContourTiler/Tiling_region.h>

CONTOURTILER_BEGIN_NAMESPACE

class All_tiling_region : public Tiling_region
{
public:
  static HTiling_region empty(Number_type z_home) { return HTiling_region(new All_tiling_region(true, z_home)); }

public:
  All_tiling_region(Number_type z_home);

  All_tiling_region(bool empty, Number_type z_home);

public:
  virtual ~All_tiling_region();

  virtual bool contains(const Point_3& point) const;

  virtual bool contains(const Segment_3& segment) const;

  virtual HTiling_region get_complement() const;

  virtual std::ostream& print(std::ostream& out) const;

  friend std::ostream& operator<<(std::ostream& out, const All_tiling_region& region);

private:
  bool _empty;
};

CONTOURTILER_END_NAMESPACE

#endif
