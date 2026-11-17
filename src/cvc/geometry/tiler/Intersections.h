#ifndef __INTERSECTIONS_H__
#define __INTERSECTIONS_H__

#include <ContourTiler/common.h>
#include <ContourTiler/mtiler_operations.h>
#include <ContourTiler/Segment_3_undirected.h>
#include <ContourTiler/print_utils.h>
#include <ContourTiler/Polyline.h>
#include <ContourTiler/Distance_functor.h>
#include <ContourTiler/triangle_utils.h>
// #include "Z_home.h"

CONTOURTILER_BEGIN_NAMESPACE

//------------------------------------------------------------------------------
// Z_home_functor class
//------------------------------------------------------------------------------
class Z_home_functor
{
public:
  Z_home_functor()
  {}

  Z_home_functor(Number_type z_home)
  { _z = z_home; }

  /// Returns true if a is further away from z_home than b.
  bool operator()( const Point_3& a, const Point_3& b ) const
  { return (abs(a.z()-_z) < abs(b.z()-_z)); }

private:
  Number_type _z;
};

//------------------------------------------------------------------------------
// typedefs
//------------------------------------------------------------------------------
typedef boost::shared_ptr<Point_3> Point_3_handle;
typedef Segment_3_undirected Segment_3_;

//------------------------------------------------------------------------------
// Point_info class
//------------------------------------------------------------------------------
class Point_info
{
private:
  typedef std::set<Point_3, Distance_functor_3> Ordered_points;
  typedef boost::unordered_map<Segment_3, Ordered_points> Segment2points;
  typedef boost::unordered_map<Point_2, Point_2> Point2opp_point;
  typedef boost::unordered_map<Point_2, Point_3> Point2new_point;
  typedef boost::unordered_map<Point_3, Segment_3> Point2segment;

public:
  // The edge on which the point lies.
  list<Segment_3_> _edges;
  // There can be more than one opposite point, all of which will
  // have identical (x,y) values.  This list is ordered by 
  // z value in the direction from the opposite z_home to z_home.
  std::set<Point_3, Z_home_functor> _opposite;
  // If the intersection is 3D then the point will have a new 
  // location.
//   Point_3_handle _new;
  // Whether the point is in conflict or not
  bool _conflict;
  Number_type _z_home;
};

//------------------------------------------------------------------------------
// Edge_info class
//------------------------------------------------------------------------------
template <typename Tile_handle>
class Edge_info
{
private:
  typedef std::set<Point_3, Distance_functor_3> Ordered_points;
  typedef boost::unordered_map<Segment_3, Ordered_points> Segment2points;
  typedef boost::unordered_map<Point_2, Point_2> Point2opp_point;
  typedef boost::unordered_map<Point_2, Point_3> Point2new_point;
  typedef boost::unordered_map<Point_3, Segment_3> Point2segment;

public:
  Ordered_points _points;
  std::list<Tile_handle> _tiles;
};

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// Intersections class
//
/// Stores information on 2D intersections between line segments.  This includes
/// the yellow and green points (which are xy_equal but not necessarily
/// xyz_equal), their respective tile edges on which they lie, etc.  It also
/// provides utility methods such as finding neighbors of intersection points.
/// This also stores locations of intersection between an endpoint of a line
/// segment with another segment and between two endpoints of segments.
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
template <typename Tile_handle>
class Intersections
{
private:
  typedef std::set<Point_3, Distance_functor_3> Ordered_points;
  typedef boost::unordered_map<Segment_3, Ordered_points> Segment2points;
  typedef boost::unordered_map<Point_2, Point_2> Point2opp_point;
  typedef boost::unordered_map<Point_2, Point_3> Point2new_point;
  typedef boost::unordered_map<Point_3, Segment_3> Point2segment;

private:
  typedef boost::unordered_map<Segment_3_, Edge_info<Tile_handle> > Edge_map;
  typedef boost::unordered_map<Point_3, Point_info> Point_map;

public:
  //--------------------------------------
  // constructor
  //--------------------------------------
  Intersections() 
    : /*_z_home((Number_type[2]){0,0}),*/ ilogger(log4cplus::Logger::getInstance("intersection"))
  {
  }

  //--------------------------------------
  // constructor
  //--------------------------------------
//   Intersections(Number_type z0, Number_type z1)
//   Intersections(const Z_home& z_home)
//     : /*_z_home(z_home),*/ ilogger(log4cplus::Logger::getInstance("intersection"))
//   { 
// //     set_z_home(z0, z1); 
//   }

  //--------------------------------------
  // destructor
  //--------------------------------------
  ~Intersections() {}

//   //--------------------------------------
//   // set_z_home
//   //--------------------------------------
//   void set_z_home(Number_type z0, Number_type z1)
//   { 
//     if (z0 == z1)
//     {
//       stringstream ss;
//       ss << "z_home values cannot be the same: " << z0;
//       throw logic_error(ss.str());
//     }

//     _z_home[0] = z0; 
//     _z_home[1] = z1; 
//   }

  //--------------------------------------
  // insert
  //
  /// p lies on s and opposite is an intersecting point.  i is the component
  /// that p belongs to.  z_home is p's z_home.
  //--------------------------------------
  void insert(const Point_3& p, const Point_3& opposite, int i, Number_type z_home)
  {
//     check_z();

    if (!xy_equal(p, opposite))
    {
      LOG4CPLUS_WARN(ilogger, "Point and its opposite counterpart are not xy equal: " << pp(p) << ", " << pp(opposite));
      throw logic_error("Point and its opposite counterpart are not xy equal");
    }

    if (!p.is_valid() || !opposite.is_valid())
    {
      LOG4CPLUS_ERROR(ilogger, "Point or its opposite counterpart: no valid id: " << pp(p) << ", " << pp(opposite));
      throw logic_error("Point or its opposite counterpart: no valid id");
    }

    Point_info& info = _point_map[i][p];
    info._z_home = z_home;

    // Initialize the opposite comparator if uninitialized
    if (info._opposite.empty()) {
//       info._opposite = set<Point_3, Z_home_functor>(Z_home_functor(_z_home[i]));
      info._opposite = set<Point_3, Z_home_functor>(Z_home_functor(z_home));
    }

    info._opposite.insert(opposite);
  }

  // @param p the intersection point
  // @param e the segment that p lies on
  // @param opposite the intersection on the other tile
  // @param i index for p
  // @param z_home z_home value for p
  void insert(const Point_3& p, const Segment_3_& e, const Point_3& opposite, int i, Number_type z_home)
  {
    insert(p, opposite, i, z_home);

    Point_info& info = _point_map[i][p];
    info._edges.push_back(e);

    // Now the edge -> point map
    Edge_info<Tile_handle>& einfo = _edge_map[i][e];
    // Initialize the comparator if uninitialized
    if (einfo._points.empty())
      einfo._points = Ordered_points(dist_functor(e.segment().source()));
    einfo._points.insert(p);
  }

  //--------------------------------------
  // set
  //
  /// s is an edge in t.  i is the component that s belongs to.
  //--------------------------------------
  void insert(const Segment_3_& e, Tile_handle t, int i)
  {
    _edge_map[i][e]._tiles.push_back(t);
  }

//   //--------------------------------------
//   // get_3D_intersections
//   //
//   /// Adds all points that have a 3D intersection to the 
//   /// output iterator points.
//   //--------------------------------------
//   template <typename Point_iter>
//   void get_3D_intersections(int i, Point_iter points) const
//   {
//     for (boost::unordered_set<Point_3>::const_iterator it = _3D[i].begin();
// 	 it != _3D[i].end();
// 	 ++it)
//     {
//       *points = *it;
//       ++points;
//     }
//   }

  //--------------------------------------
  // neighbors
  //
  /// Adds all points that are immediate neighbors to the
  /// output iterator n.  At most
  /// two points will be added.  Neighbors are points that
  /// are next to p on p's tile edge.
  //--------------------------------------
  template <typename Point_iter>
  void neighbors(const Point_3& p, int i, const Segment_3_& e, Tile_handle tile, Point_iter n) const
  {
    const Ordered_points& points = info(e, i)._points;
    Ordered_points::const_iterator p_iter = points.find(p);
    if (p_iter == points.end())
    {
      LOG4CPLUS_WARN(ilogger, "Unexpectedly didn't find point in edge's points: " << pp(p));
      for (Ordered_points::const_iterator it = points.begin(); it != points.end(); ++it)
      {	LOG4CPLUS_WARN(ilogger, "  " << pp(*it)); }
      throw logic_error("Unexpectedly didn't find point in edge's points");
    }

    if (p_iter != points.begin())
    {
      Ordered_points::const_iterator prev = p_iter;
      --prev;
      *n++ = *prev;
    }
    Ordered_points::const_iterator next = p_iter;
    ++next;
    if (next != points.end())
    {
      *n++ = *next;
    }
  }

  template <typename Point_iter>
  void neighbors(const Point_3& p, int i, Tile_handle tile, Point_iter n) const
  {
    for (list<Segment_3_>::const_iterator it = edges_begin(p, i); it != edges_end(p, i); ++it)
      neighbors(p, i, *it, tile, n);
  }

  //--------------------------------------
  // intersections
  //
  /// Given an edge, finds all intersections
  /// between the edge and the given tile.
  //--------------------------------------
  template <typename Point_iter>
  void intersections(const Segment_3_& ye, int yi, Tile_handle gt, Point_iter n) const
  {
    int gi = 1-yi;
    const Ordered_points& points = info(ye, yi)._points;
    Ordered_points::const_iterator p_iter = points.begin();
    for (; p_iter != points.end(); ++p_iter)
    {
      const Point_3& gp = first_opposite(*p_iter, yi);
      list<Tile_handle> gtiles;
      tiles(gp, gi, back_inserter(gtiles));
      if (find(gtiles.begin(), gtiles.end(), gt) != gtiles.end())
      {
	*n = gp;
	++n;
      }
    }
  }

  //--------------------------------------
  // edge
  //--------------------------------------
  list<Segment_3_>::const_iterator edges_begin(const Point_3& p, int i) const
  { return info(p, i)._edges.begin(); }

  list<Segment_3_>::const_iterator edges_end(const Point_3& p, int i) const
  { return info(p, i)._edges.end(); }

//   //--------------------------------------
//   // edge_from_new_point
//   //
//   /// p is a new point
//   //--------------------------------------
//   const Segment_3_& edge_from_new_point(const Point_3& p, int i) const
//   { 
//     if (has_point(p, i))
//       return info(p, i)._edge; 
//     return edge(_new2old.find(p)->second, i);
//   }

  //--------------------------------------
  // first_opposite
  //
  /// Returns the point opposite p that is furthers from the opposite
  /// component's z_home. 
  //--------------------------------------
  const Point_3& first_opposite(const Point_3& p, int i) const
  { return *info(p, i)._opposite.begin(); }

  //--------------------------------------
  // tiles
  // 
  /// Returns all tiles touching e
  //--------------------------------------
  const list<Tile_handle>& tiles(const Segment_3_& e, int i) const
  { return info(e, i)._tiles; }

  //--------------------------------------
  // tiles
  // 
  /// Returns all tiles touching e
  //--------------------------------------
  template <typename Tile_iter>
  void tiles(const Segment_3_& e, int i, Tile_iter t) const
  { 
    const list<Tile_handle>& t_ = info(e, i)._tiles; 
    for (typename list<Tile_handle>::const_iterator it = t_.begin(); it != t_.end(); ++it)
    {
      *t = *it;
      ++t;
    }
  }

  //--------------------------------------
  // tiles
  //
  /// Returns all tiles touching p
  //--------------------------------------
  template <typename Tile_iter>
  void tiles(const Point_3& p, int i, Tile_iter tiles) const
  { 
    for (list<Segment_3_>::const_iterator it = edges_begin(p, i); it != edges_end(p, i); ++it)
    {
      const Segment_3_& e = *it;
      for (typename list<Tile_handle>::const_iterator t_it = info(e, i)._tiles.begin(); 
	   t_it != info(e, i)._tiles.end();
	   ++t_it)
      {
	*tiles++ = *t_it;
      }
    }
  }

  //--------------------------------------
  // has_point
  //
  /// Returns true if information for the
  /// given point is here
  //--------------------------------------
  bool has_point(const Point_3& p, int i) const
  {
    return (_point_map[i].find(p) != _point_map[i].end());
  }

  //--------------------------------------
  // has_edge
  //
  /// Returns true if the given point has
  /// an associated edge
  //--------------------------------------
  bool has_edge(const Point_3& p, int i) const
  {
    return edges_begin(p, i) != edges_end(p, i);
  }

  //--------------------------------------
  // has_edge
  //
  /// Returns true if e intersects with
  /// another component
  //--------------------------------------
  bool has_edge(const Segment_3_& e, int i) const
  {
    return (_edge_map[i].find(e) != _edge_map[i].end());
  }

  template <typename Point_iter>
  void all_intersections(Point_iter intersections) const
  {
    for(Point_map::const_iterator it = _point_map[0].begin(); it != _point_map[0].end(); ++it)
      *intersections++ = it->first;
  }

  Number_type z_home(const Point_3& p, int i) const
  {
    return info(p, i)._z_home;
  }

private:
//   void check_z()
//   { 
//     if (_z_home[0] == _z_home[1])
//       throw logic_error("z_home values are uninitialized");
//   }

  const Point_info& info(const Point_3& p, int i) const
  { 
    Point_map::const_iterator it = _point_map[i].find(p);
    if (it == _point_map[i].end())
    {
      LOG4CPLUS_WARN(ilogger, "Point not found in map: " << pp(p) << ", i = " << i);
      for (typename Point_map::const_iterator it = _point_map[i].begin(); it != _point_map[i].end(); ++it)
	LOG4CPLUS_TRACE(ilogger, "  " << pp(it->first));
      throw logic_error("Point not found in map");
    }
    return it->second;
  }

  const Edge_info<Tile_handle>& info(const Segment_3_& e, int i) const
  { 
    typename Edge_map::const_iterator it = _edge_map[i].find(e);
    if (it == _edge_map[i].end())
    {
      LOG4CPLUS_WARN(ilogger, "Edge not found in map: " << pp(e) << ", i = " << i);
      for (typename Edge_map::const_iterator it = _edge_map[i].begin(); it != _edge_map[i].end(); ++it)
	LOG4CPLUS_TRACE(ilogger, "  " << pp(it->first));
      throw logic_error("Edge not found in map");
    }
    return it->second;
  }

private:
  Edge_map _edge_map[2];
  Point_map _point_map[2];
  // All points involved in a 3D intersection
//   boost::unordered_set<Point_3> _3D[2];
//   boost::unordered_map<Point_3, Point_3> _new2old;
//   Number_type _z_home[2];
//   Z_home _z_home;

  mutable log4cplus::Logger ilogger;

public:
  boost::unordered_map<Tile_handle, int> tile2idx;
};

CONTOURTILER_END_NAMESPACE

#endif
