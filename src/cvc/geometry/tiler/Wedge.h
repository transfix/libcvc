#ifndef __WEDGE_H__
#define __WEDGE_H__

#include <boost/shared_ptr.hpp>

#include <CGAL/Cartesian.h>
#include <CGAL/enum.h>

#include <ContourTiler/common.h>
#include <ContourTiler/Tiling_region.h>

CONTOURTILER_BEGIN_NAMESPACE

class Wedge;
typedef boost::shared_ptr<Wedge> HWedge;

class Wedge : public Tiling_region
{
public:
  static HWedge RS(const Point_2& p, const Point_2& q, const Point_2& r, Number_type z_home)
  { return HWedge(new Wedge(p, q, r, false, z_home)); }

  static HWedge LS(const Point_2& p, const Point_2& q, const Point_2& r, Number_type z_home)
  { return HWedge(new Wedge(r, q, p, false, z_home)); }

  static HWedge whitelist(const Wedge& wedge, HTiling_region whitelisted)
  { return HWedge(new Wedge(wedge, whitelisted)); }

  static HWedge whitelist(const Wedge& wedge, const Point_3& p);

  Wedge(Number_type z_home) : Tiling_region(z_home) {}

  Wedge(const Point_2& p, const Point_2& q, const Point_2& r, bool closed, Number_type z_home) 
    : Tiling_region(z_home), _ray1(Ray_2(q, p)), _ray2(Ray_2(q, r)), _closed(closed) 
  {
  }

  Wedge(const Ray_2& ray1, const Ray_2& ray2, bool closed, Number_type z_home) 
    : Tiling_region(z_home), _ray1(ray1), _ray2(ray2), _closed(closed) 
  {
    if (ray1.source() != ray2.source())
      throw std::logic_error("Rays forming a wedge must have the same source");
  }

  Wedge(const Wedge& wedge, HTiling_region whitelisted)
    : Tiling_region(wedge.z_home_nothrow()), _ray1(wedge._ray1), _ray2(wedge._ray2), _closed(wedge._closed) , _whitelisted(whitelisted)
  {
  }

  virtual ~Wedge() {}

  virtual bool contains(const Point_3& point) const;

  virtual bool contains(const Segment_3& segment) const;

  virtual HTiling_region get_complement() const;

  virtual std::ostream& print(std::ostream& out) const;

  bool is_empty_intersection(const Wedge& w) const;

  Point_2 vertex() const
  { return _ray1.source(); }

  bool is_empty() const
  { return !_closed && _ray1 == _ray2; }

  friend bool operator==(const Wedge& a, const Wedge& b);
  friend std::ostream& operator<<(std::ostream& out, const Wedge& wedge);

private:
  bool is_boundary(const Point_2& p, const Segment_2& segment, const Ray_2& ray) const;

  bool whitelisted(const Point_3& point3) const;

private:
  Ray_2 _ray1;
  Ray_2 _ray2;
  bool _closed;
  HTiling_region _whitelisted;
};

CONTOURTILER_END_NAMESPACE

#endif
