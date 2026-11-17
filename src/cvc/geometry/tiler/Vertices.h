#ifndef __VERTICES_H__
#define __VERTICES_H__

#include <list>

#include <boost/functional/hash.hpp>

#include <ContourTiler/Contour.h>
#include <ContourTiler/contour_utils.h>
#include <ContourTiler/Hierarchy.h>
#include <ContourTiler/Vertex_map.h>

CONTOURTILER_BEGIN_NAMESPACE

class Vertices
{
private:
  typedef std::vector<Point_3> Container;
  
public:
  typedef Container::iterator iterator;
  typedef Container::const_iterator const_iterator;
//   typedef std::vector<Mid_vertex>::iterator mid_iterator;
//   typedef std::vector<Mid_vertex>::const_iterator const_mid_iterator;
  typedef std::set<Number_type>::iterator z_iterator;
  typedef std::set<Number_type>::const_iterator const_z_iterator;

public:
  template <typename ContourIterator>
  static Vertices create(ContourIterator contours_begin, ContourIterator contours_end)
  {
    Vertices vertices;
    size_t contour_offset = 0;
    for (ContourIterator it = contours_begin; it != contours_end; ++it)
    {
      Contour_handle contour = *it;
      vertices._contour_offsets[contour] = contour_offset;
      Polygon_2& polygon = contour->polygon();
      size_t i = 0;
      for (Polygon_2::Vertex_iterator vit = polygon.vertices_begin();
	   vit != polygon.vertices_end();
	   ++vit, ++i)
//       for (size_t i = 0; i < polygon.size(); ++i)
      {
	vit->z() = contour->slice();
	vit->id() = contour_offset + i;
// 	Point_3 v(contour, i, contour_offset);
// 	Point_3 v(polygon[i]);
	vertices.push_back(*vit);
	vertices._contours[*vit] = contour;
      }
      contour_offset += contour->size();
    }
    return vertices;
  }

  Vertices() : _next_unique_id(0) {}
  ~Vertices() {}

//   const Mid_vertex& add(const Point_3& point)
//   {
//     _mid_vertices.push_back(Mid_vertex(point, size())); 
//     return _mid_vertices.back();
//   }

  size_t size() const { return _vertices.size(); }// + _mid_vertices.size(); }

  const_iterator begin() const { return _vertices.begin(); }
  const_iterator end() const { return _vertices.end(); }

//   const_mid_iterator begin_mid() const { return _mid_vertices.begin(); }
//   const_mid_iterator end_mid() const { return _mid_vertices.end(); }

  Point_3 operator[](size_t idx) const
  {
//     if (idx < _vertices.size())
      return _vertices[idx];//.point_3();
    
//     return _mid_vertices[idx - _vertices.size()].point_3(); 
  }

  const Point_3& get_contour_vertex(size_t idx) const
  { return _vertices[idx]; }

  const Point_3& get(Contour_handle contour, size_t vertex_idx) const
  { return _vertices[offset(contour) + vertex_idx]; }

  const Point_3& get(Contour_handle contour, Polygon_2::Vertex_circulator ci) const
  { return _vertices[offset(contour) + vertex_idx(contour, ci)]; }

  const Point_3& get(Contour_handle contour, Polygon_2::Vertex_iterator it) const
  { return _vertices[offset(contour) + vertex_idx(contour, it)]; }

  /// Returns all vertices for the given contour
  template <typename OutputIterator>
  void get_vertices(Contour_handle contour, OutputIterator vertices) const
  { 
    for (size_t i = 0; i < contour->size(); ++i)
    {
      *vertices = get(contour, i);
      ++vertices;
    }
  }

  const_z_iterator z_values_begin() const { return _z_values.begin(); }
  const_z_iterator z_values_end() const { return _z_values.end(); }

//   Contour_handle contour(const Point_3& v) { if (v.id() == DEFAULT_ID()) return Contour_handle(); return _contours[v]; }
  Contour_handle contour(const Point_3& v) const 
  { 
    if (v.id() == DEFAULT_ID() || !_contours.contains(v)) 
      return Contour_handle(); 
    return _contours[v]; 
  }

  Point_3 ccw(const Point_3& v) const
  { 
    Contour_handle _contour = contour(v);
    size_t _contour_offset = _contour_offsets.find(_contour)->second;
    size_t _vertex_idx = v.unique_id() - _contour_offset;
    size_t idx = _contour_offset + ((_vertex_idx + 1) % _contour->size());
//     return Point_3(_contour, idx, _contour_offset); 
    return _vertices[idx];
  }

  Point_3 cw(const Point_3& v) const
  { 
    Contour_handle _contour = contour(v);
    size_t _contour_offset = _contour_offsets.find(_contour)->second;
    size_t _vertex_idx = v.unique_id() - _contour_offset;
    size_t idx = _contour_offset + ((_vertex_idx > 0) ? _vertex_idx - 1 : _contour->size() - 1);
//     return Point_3(_contour, idx, _contour_offset);
    return _vertices[idx];
  }

  bool adjacent(const Point_3& u, const Point_3& v) const
  { 
    Contour_handle cu = contour(u);
    Contour_handle cv = contour(v);
    if (cu != cv)
      return false;
    return u == cw(v) || u == ccw(v);
  }

  Point_3 adjacent(const Point_3& v, Walk_direction dir, const Hierarchies& hierarchies) const
  { 
    return adjacent(v, dir, hierarchies.find(v.z())->second);
  }

  Point_3 adjacent(const Point_3& v, Walk_direction dir, const Hierarchy& hierarchy) const
  { 
    Contour_handle _contour = contour(v);
    if (hierarchy.is_CCW(_contour))
      return (dir == Walk_direction::FORWARD) ? ccw(v) : cw(v);
    return (dir == Walk_direction::FORWARD) ? cw(v) : ccw(v);
  }

  size_t unique_id() {
    return _next_unique_id++;
  }

private:
  void push_back(const Point_3& v) 
  { 
    _vertices.push_back(v); 
    _z_values.insert(v.z());
    _next_unique_id = max(_next_unique_id, v.id()+1);
  }
  size_t offset(Contour_handle contour) const { return _contour_offsets.find(contour)->second; }

private:
  Container _vertices;
  std::map<Contour_handle, size_t> _contour_offsets;
  std::set<Number_type> _z_values;
//   std::vector<Mid_vertex> _mid_vertices;
  Vertex_map<Contour_handle> _contours;
  size_t _next_unique_id;
};

CONTOURTILER_END_NAMESPACE

#endif
