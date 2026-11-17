#ifndef __BOUNDARY_SLICE_CHORD_H__
#define __BOUNDARY_SLICE_CHORD_H__

#include <ContourTiler/common.h>
#include <ContourTiler/CGAL_hash.h>
#include <ContourTiler/Vertex_map.h>
#include <ContourTiler/Correspondences.h>
#include <ContourTiler/Tiling_region.h>
#include <ContourTiler/Vertex_completion_map.h>

CONTOURTILER_BEGIN_NAMESPACE

/// Boundary slice chord
class Boundary_slice_chord
{
public:
  friend std::size_t hash_value(const Boundary_slice_chord& bsc)
  {
    return CGAL::hash_value(bsc._segment);
  }

  bool operator==(const Boundary_slice_chord& bsc) const
  {
    return _segment == bsc._segment;
  }

  bool operator!=(const Boundary_slice_chord& bsc) const
  {
    return !(*this == bsc);
  }

  friend std::ostream& operator<<(std::ostream& out, const Boundary_slice_chord& bsc)
  {
    out << bsc.segment();
//     Contour_vertex temp1, temp2;
//     boost::tie(temp1, temp2) = bsc.vertices();
//     out << temp1 << "  " << temp2;
    return out;
  }

public:
  Boundary_slice_chord() : _dir(Walk_direction::FORWARD) {}

  Boundary_slice_chord(const Segment_3& segment, Walk_direction dir, Number_type seg_z)
    : _segment(segment), _dir(dir), _seg_z(seg_z)
  { 
    if (segment.source().z() == segment.target().z())
      throw std::logic_error("segment points must lie on different slices");
//     assert_order(); 
  }

  ~Boundary_slice_chord() {}

  Boundary_slice_chord opposite() const
  {
    return Boundary_slice_chord(_segment.opposite(), _dir, _seg_z);
  }

  const Segment_3& segment() const { return _segment; }
  Walk_direction direction() const { return _dir; }

  bool is_source(const Point_3& vertex) const
  {
    return vertex == _segment.source();
  }

  /// slice that the contour segment of the adjoining tile is on.
  Number_type seg_z() const
  { return _seg_z; }

//   bool is_endpoint(const Point_3& vertex) const
//   {
//     return vertex == _segment.source() || vertex == _segment.target();
// //     return xyz_equal(vertex, boost::get<0>(_vertices)) || xyz_equal(vertex, boost::get<1>(_vertices));
//   }

private:
//   void assert_order()
//   {
//     if (boost::get<0>(_vertices).z() >= boost::get<1>(_vertices).z())
//       throw std::logic_error("first vertex must be on the bottom in a boundary slice chord");
//   }

private:
  Segment_3 _segment;
  Walk_direction _dir;
  Number_type _seg_z;
};

CONTOURTILER_END_NAMESPACE

#endif
