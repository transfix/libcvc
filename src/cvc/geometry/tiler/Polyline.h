#ifndef __POLYLINE_H__
#define __POLYLINE_H__

#include <boost/functional/hash.hpp>

#include <ContourTiler/common.h>

CONTOURTILER_BEGIN_NAMESPACE

template <typename Point>
class Polyline
{
private:
  typedef typename std::vector<Point> Container;
public:
  typedef typename Container::iterator iterator;
  typedef typename Container::const_iterator const_iterator;
  typedef typename Container::reverse_iterator reverse_iterator;
  typedef typename Container::const_reverse_iterator const_reverse_iterator;

public:
  Polyline() {}

  Polyline(const Point_3& p)
  { _vertices.push_back(p); }

  template <typename Vertex_iter>
  Polyline(Vertex_iter beg, Vertex_iter end)
  { 
    _vertices.insert(_vertices.end(), beg, end); 
    check_valid(); 
  }

  ~Polyline() {}

  void push_back(const Point& v)
  { _vertices.push_back(v); check_valid(); }

  template <typename Vertex_iter>
  void insert(iterator loc, Vertex_iter beg, Vertex_iter end)
  { _vertices.insert(loc, beg, end); check_valid(); }

  const Point& source() const { return _vertices[0]; }
  const Point& target() const { return _vertices[size()-1]; }

  iterator begin() { return _vertices.begin(); }
  const_iterator begin() const { return _vertices.begin(); }
  iterator end() { return _vertices.end(); }
  const_iterator end() const { return _vertices.end(); }

  reverse_iterator rbegin() { return _vertices.rbegin(); }
  const_reverse_iterator rbegin() const { return _vertices.rbegin(); }
  reverse_iterator rend() { return _vertices.rend(); }
  const_reverse_iterator rend() const { return _vertices.rend(); }

  Point& operator[](int i) { return _vertices[i]; }
  const Point& operator[](int i) const { return _vertices[i]; }

  size_t size() const { return _vertices.size(); }

  bool operator==(const Polyline& p) const
  { return _vertices == p._vertices; }

private:
  void check_valid() const;

private:
  Container _vertices;
};

typedef Polyline<Point_2> Polyline_2;
typedef Polyline<Point_3> Polyline_3;

inline std::size_t hash_value(const Polyline_2& p)
{
  std::size_t seed = 0;
  for (Polyline_2::const_iterator it = p.begin(); it != p.end(); ++it)
    boost::hash_combine(seed, hash_value(*it));
  return seed;
}

inline std::size_t hash_value(const Polyline_3& p)
{
  std::size_t seed = 0;
  for (Polyline_3::const_iterator it = p.begin(); it != p.end(); ++it)
    boost::hash_combine(seed, hash_value(*it));
  return seed;
}

CONTOURTILER_END_NAMESPACE

#endif
