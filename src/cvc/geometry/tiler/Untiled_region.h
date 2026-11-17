#ifndef __UNTILED_REGION_H__
#define __UNTILED_REGION_H__

#include <ContourTiler/common.h>
#include <list>

CONTOURTILER_BEGIN_NAMESPACE

/// Basically is a collection of points representing a polygon.
/// Useful when it needs to support points directly above another.
class Untiled_region
{
private:
  typedef std::list<Point_3> Container;
public:
  typedef Container::iterator iterator;
  typedef Container::const_iterator const_iterator;

public:
  Untiled_region() {}

  template <typename PointIterator>
  Untiled_region(PointIterator begin, PointIterator end)
  {
    insert(begin, end);
  }

  ~Untiled_region() {}

  template <typename PointIterator>
  void insert(PointIterator begin, PointIterator end)
  {
    for (PointIterator it = begin; it != end; ++it)
      push_back(it->point_3());
  }

  void push_back(const Point_3& point) { _points.push_back(point); }

  iterator begin() { return _points.begin(); }
  const_iterator begin() const { return _points.begin(); }
  iterator end() { return _points.end(); }
  const_iterator end() const { return _points.end(); }

  Container::reverse_iterator rbegin() { return _points.rbegin(); }
  Container::const_reverse_iterator rbegin() const { return _points.rbegin(); }
  Container::reverse_iterator rend() { return _points.rend(); }
  Container::const_reverse_iterator rend() const { return _points.rend(); }

  void reverse_orientation()
  {
    _points = Container(_points.rbegin(), _points.rend());
  }

  size_t size() const { return _points.size(); }
  bool empty() const { return _points.empty(); }

private:
  Container _points;
};

CONTOURTILER_END_NAMESPACE

#endif
