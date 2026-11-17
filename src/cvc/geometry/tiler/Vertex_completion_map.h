#ifndef __VERTEX_COMPLETION_MAP_H__
#define __VERTEX_COMPLETION_MAP_H__

#include <boost/unordered_map.hpp>

#include <ContourTiler/Tile.h>

CONTOURTILER_BEGIN_NAMESPACE

class Vertex_completion_map
{
public:
  Vertex_completion_map() {}
  ~Vertex_completion_map() {}

  /// The segment must be in contour ordering
  void put(const Segment_3& segment)
  {
    //     -----------------*-----------------      slice i
    //                     / \
    //                    /   \
    //                   /     \
    //                  /       \
    //                 /         \
    //                /           \
    //               /             \
    // *------------*===============*------------*  slice j
    //              ^               ^
    //           source          target
    // 
    // source is forward-complete
    // target is backward-complete
    _forward[segment.source()];
    _backward[segment.target()];
  }

  bool is_complete(const Segment_3& segment)
  {
    return (_forward.find(segment.source()) != _forward.end() && _backward.find(segment.target()) != _backward.end());
  }

  bool is_complete(const Point_3& vertex)
  {
    return (_forward.find(vertex) != _forward.end() && _backward.find(vertex) != _backward.end());
  }

  bool is_complete(const Point_3& vertex, Walk_direction dir)
  {
    return (dir == Walk_direction::FORWARD) ? (_forward.find(vertex) != _forward.end()) : (_backward.find(vertex) != _backward.end());
  }

private:
  boost::unordered_map<Point_3, bool> _forward;
  boost::unordered_map<Point_3, bool> _backward;
};

CONTOURTILER_END_NAMESPACE

#endif
