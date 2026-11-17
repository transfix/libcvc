#ifndef __COLORED_POINT_H__
#define __COLORED_POINT_H__

#include <ContourTiler/common.h>
#include <ContourTiler/Color.h>

CONTOURTILER_BEGIN_NAMESPACE

template <typename Point>
class Colored_point
{
public:
  Colored_point() {}
  Colored_point(const Point& point, const Color& color) 
    : _point(point), _color(color) {}
  Colored_point(const Point& point, double r, double g, double b) 
    : _point(point), _color(r, g, b) {}
  ~Colored_point() {}

  Point& point() { return _point; }
  const Point& point() const { return _point; }

  Color& color() { return _color; }
  const Color& color() const { return _color; }

  const Number_type& x() const { return _point.x(); }
  const Number_type& y() const { return _point.y(); }
  const Number_type& z() const { return _point.z(); }

  double r() { return _color.r(); }
  double g() { return _color.g(); }
  double b() { return _color.b(); }

  bool operator==(const Colored_point<Point>& p) const
  { return _point == p._point && _color == p._color; }

private:  
  Point _point;
  Color _color;
};

template <typename Point>
std::size_t hash_value(const Colored_point<Point>& point)
{
  std::size_t seed = 0;
  boost::hash_combine(seed, hash_value(point.point()));
  boost::hash_combine(seed, hash_value(point.color()));
  return seed;
}

typedef Colored_point<Point_2> Colored_point_2;
typedef Colored_point<Point_3> Colored_point_3;

CONTOURTILER_END_NAMESPACE

#endif
