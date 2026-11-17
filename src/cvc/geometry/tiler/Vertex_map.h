#ifndef __VERTEX_MAP_H__
#define __VERTEX_MAP_H__

#include <vector>
#include <map>
#include <boost/unordered_map.hpp>
#include <ContourTiler/CGAL_hash.h>

#include <ContourTiler/Contour.h>

CONTOURTILER_BEGIN_NAMESPACE

template <typename T>
class Vertex_map
{
  //--------------------------------------------------------
  // Vertex_map class
  //--------------------------------------------------------
private:
  typedef std::vector<T> Values;

public:
  typedef typename Values::iterator iterator;
  typedef typename Values::const_iterator const_iterator;
  
public:
  Vertex_map() : _values(10000) {}
  Vertex_map(size_t size) : _values(size) 
  {
    _values.resize(size);
  }
  ~Vertex_map() 
  {
  }

  typename Values::const_reference operator[](const Point_3& vertex) const
  {
    return _values[vertex.id()];
  }

  typename Values::reference operator[](const Point_3& vertex)
  {
    _values.reserve((vertex.id()+1)*2);
    return _values[vertex.id()];
  }

  bool contains(const Point_3& vertex) const
  {
    return vertex.id() < _values.size();
  }

  void set(const Point_3& vertex, const T& value)
  {
    _values.reserve(vertex.id()+1);
    _values[vertex.id()] = value;
  }

  iterator begin() { return _values.begin(); }
  const_iterator begin() const { return _values.begin(); }
  iterator end() { return _values.end(); }
  const_iterator end() const { return _values.end(); }

private:
  Values _values;
};

CONTOURTILER_END_NAMESPACE

#endif
