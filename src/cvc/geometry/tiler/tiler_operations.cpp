#include <ContourTiler/tiler_operations.h>

#include <sstream>
#include <limits>
#include <ostream>

#include <boost/shared_array.hpp>

#include <log4cplus/logger.h>
#include <log4cplus/loggingmacros.h>

#include <CGAL/intersections.h>
#include <CGAL/squared_distance_2.h>
#include <CGAL/Direction_2.h>

#include <ContourTiler/Boundary_slice_chords.h>
#include <ContourTiler/print_utils.h>
#include <ContourTiler/intersection/Segment_3_Segment_3.h>
#include <ContourTiler/skeleton.h>
#include <ContourTiler/theorems.h>
#include <ContourTiler/Untiled_region.h>
#include <ContourTiler/augment.h>
#include <ContourTiler/Distance_functor.h>
#include <ContourTiler/xy_pred.h>
#include <ContourTiler/Tiling_region.h>
#include <ContourTiler/Composite_tiling_region.h>
#include <ContourTiler/All_tiling_region.h>
#include <ContourTiler/Wedge.h>
#include <ContourTiler/polygon_intersection.h>

using namespace log4cplus;

CONTOURTILER_BEGIN_NAMESPACE

/// Modifies the given polygons in place so that the contours are augmented
void augment(Polygon_2& P, Polygon_2& Q)
{
  boost::tie(P, Q) = augment1(P, Q);
}

Walk_direction opposite(Walk_direction dir)
{
  if (dir == Walk_direction::FORWARD)
    return Walk_direction::BACKWARD;
  return Walk_direction::FORWARD;
}

bool is_positive(const Point_3& v, const Contour_handle& c, const Hierarchy& h, const Hierarchy& opp_h)
{
  return h.orientation(c) != opp_h.orientation(opp_h.NEC(v));
}

Point_2 my_midpoint(const Point_2& p, const Point_2& q)
{
  return Point_2((p.x()+q.x())/2.0, (p.y()+q.y())/2.0, (p.z()+q.z())/2.0);
}

bool is_backwards_new(const Point_2& u2, const Point_2& u3, 
		  const Point_2& v2, const Point_2& v3)
{
//   if (!xy_equal(u2, v2))
//     return false;

  if (CGAL::left_turn(u3, u2, v3))
    return (CGAL::angle(u3, u2, v3) == CGAL::OBTUSE);
  if (CGAL::left_turn(v3, u2, u3))
    return (CGAL::angle(v3, u2, u3) == CGAL::OBTUSE);
  return true;
}

// bool is_backwards_new(const Point_3& u2, const Point_3& v2, 
// 		      const Vertices& vertices, const Hierarchies& h)
// {
//   Walking_dir dir = Walking_dir::FORWARD;
//   Point_3 u1 = vertices.adjacent(u2, opposite(dir), h);
//   Point_3 u3 = vertices.adjacent(u2, dir, h);
//   Point_3 v1 = vertices.adjacent(v2, opposite(dir), h);
//   Point_3 v3 = vertices.adjacent(v2, dir, h);

//   bool bf = is_backwards_new(u2, u3, v2, v3);
//   bool bb = is_backwards_new(u2, u1, v2, v1);

//   if (bf && bb) return true;
  
// }

bool is_backwards(const Point_3& u1, const Point_3& u2, const Point_3& u3, 
		  const Point_3& v1, const Point_3& v2, const Point_3& v3, 
		  const Contour_handle ucontour, const Contour_handle vcontour,
		  const Hierarchy& cur_h, const Hierarchy& opp_h)
{
  if (!xy_equal(u2, v2))
    return false;

  if (xy_equal(u1, v3) || xy_equal(u3, v1))
    return true;

//   const Contour_handle contour = vertices.contour(u2);
  const Polygon_2& vpoly = vcontour->polygon();
  Point_2 m1 = my_midpoint(u1, u2);
  Point_2 m3 = my_midpoint(u2, u3);
  bool pos1 = is_positive(m1, ucontour, cur_h, opp_h);
  bool pos2 = is_positive(u2, ucontour, cur_h, opp_h);
  bool pos3 = is_positive(m3, ucontour, cur_h, opp_h);
  bool on1 = find_if(vpoly.vertices_begin(), vpoly.vertices_end(), xy_pred(u1)) != vpoly.vertices_end();
  bool on3 = find_if(vpoly.vertices_begin(), vpoly.vertices_end(), xy_pred(u3)) != vpoly.vertices_end();
  return pos1 && pos2 && pos3 && !on1 && !on3;
}

bool is_backwards(const Point_3& p, const Contour_handle opposite_contour,
		  const Vertices& vertices, const Hierarchies& h)
{
  const Polygon_2& op = opposite_contour->polygon();
  Polygon_2::const_iterator qit = find_if(op.vertices_begin(), op.vertices_end(), xy_pred(p));
  if (qit != op.vertices_end()) {
    Walk_direction dir = Walk_direction::FORWARD;
    Point_3 u2 = p;
    Point_3 v2 = *qit;
    Point_3 u1 = vertices.adjacent(u2, opposite(dir), h);
    Point_3 u3 = vertices.adjacent(u2, dir, h);
    Point_3 v1 = vertices.adjacent(v2, opposite(dir), h);
    Point_3 v3 = vertices.adjacent(v2, dir, h);
    const Hierarchy& cur_h = h.find(u2.z())->second;
    const Hierarchy& opp_h = h.find(v2.z())->second;
    return is_backwards(u1, u2, u3, v1, v2, v3, 
			vertices.contour(u2), opposite_contour,
			cur_h, opp_h);
  }
  return false;
}

/// See unit test31.
bool test31a(const Point_3& u2, const Point_3& v2, const Vertices& vertices, const Hierarchies& h)
{
  return !is_backwards(u2, vertices.contour(v2), vertices, h) ||
    !is_backwards(v2, vertices.contour(u2), vertices, h);
//   if (!xy_equal(u2, v2)) return true;

//   Walk_direction dir = Walk_direction::FORWARD;
//   Point_3 u1 = vertices.adjacent(u2, opposite(dir), h);
//   Point_3 u3 = vertices.adjacent(u2, dir, h);
//   Point_3 v1 = vertices.adjacent(v2, opposite(dir), h);
//   Point_3 v3 = vertices.adjacent(v2, dir, h);

//   return (!xy_equal(u1, v3) && !xy_equal(u3, v1));

//   if (xy_equal(u1, v3)) {
// //     w.vertices.contour(v2)
//     return target != u1;
//   }
//   else if (xy_equal(u3, v1)) {
//     return target != u3;
//   }
//   return true;
}

bool is_illegal(const Point_3& p, const Contour_handle opposite_contour,
		const Vertices& vertices, const Hierarchies& h)
{
  const Polygon_2& op = opposite_contour->polygon();
  Polygon_2::const_iterator qit = find_if(op.vertices_begin(), op.vertices_end(), xy_pred(p));
  if (qit != op.vertices_end()) {
    Walk_direction dir = Walk_direction::FORWARD;
    Point_3 u2 = p;
    Point_3 v2 = *qit;
    Point_3 u1 = vertices.adjacent(u2, opposite(dir), h);
    Point_3 u3 = vertices.adjacent(u2, dir, h);
    Point_3 v1 = vertices.adjacent(v2, opposite(dir), h);
    Point_3 v3 = vertices.adjacent(v2, dir, h);
    if (xy_equal(u1, v3) || xy_equal(u3, v1))
      return false;

    const Hierarchy& cur_h = h.find(u2.z())->second;
    const Hierarchy& opp_h = h.find(v2.z())->second;
    const Contour_handle contour = vertices.contour(p);
    Point_2 m1 = my_midpoint(u1, u2);
    Point_2 m3 = my_midpoint(u2, u3);
    bool pos1 = is_positive(m1, contour, cur_h, opp_h);
    bool pos2 = is_positive(u2, contour, cur_h, opp_h);
    bool pos3 = is_positive(m3, contour, cur_h, opp_h);
    bool on1 = find_if(op.vertices_begin(), op.vertices_end(), xy_pred(m1)) != op.vertices_end();
    bool on3 = find_if(op.vertices_begin(), op.vertices_end(), xy_pred(m3)) != op.vertices_end();
    return pos1 && pos2 && pos3 && !on1 && !on3;
  }
  return false;
}

bool is_illegal(const Point_3& u2, const Point_3& v2,
		const Vertices& vertices, const Hierarchies& h)
{
  const Polygon_2& op = vertices.contour(v2)->polygon();
//   Polygon_2::const_iterator qit = find_if(op.vertices_begin(), op.vertices_end(), xy_pred(p));
//   if (qit != op.vertices_end()) {
    Walk_direction dir = Walk_direction::FORWARD;
//     Point_3 u2 = p;
//     Point_3 v2 = *qit;
    Point_3 u1 = vertices.adjacent(u2, opposite(dir), h);
    Point_3 u3 = vertices.adjacent(u2, dir, h);
    Point_3 v1 = vertices.adjacent(v2, opposite(dir), h);
    Point_3 v3 = vertices.adjacent(v2, dir, h);
    if (xy_equal(u1, v3) || xy_equal(u3, v1))
      return false;

    const Hierarchy& cur_h = h.find(u2.z())->second;
    const Hierarchy& opp_h = h.find(v2.z())->second;
    const Contour_handle contour = vertices.contour(u2);
    Point_2 m1 = my_midpoint(u1, u2);
    Point_2 m3 = my_midpoint(u2, u3);
    bool pos1 = is_positive(m1, contour, cur_h, opp_h);
    bool pos2 = is_positive(u2, contour, cur_h, opp_h);
    bool pos3 = is_positive(m3, contour, cur_h, opp_h);
    bool on1 = find_if(op.vertices_begin(), op.vertices_end(), xy_pred(m1)) != op.vertices_end();
    bool on3 = find_if(op.vertices_begin(), op.vertices_end(), xy_pred(m3)) != op.vertices_end();
    return pos1 && pos2 && pos3 && !on1 && !on3;
//   }
  return false;
}

template <typename ContourIterator>
OTV_table build_OTV_table(ContourIterator contours_begin, ContourIterator contours_end,
			  ContourIterator c1_begin, ContourIterator c1_end,
			  ContourIterator c2_begin, ContourIterator c2_end,
			  const Vertices& vertices,
			  const Correspondences& correspondences,
			  const Vertex_map<HTiling_region>& tiling_regions,
			  const Hierarchies& h,
			  Banned& banned)
{
  Logger logger = Logger::getInstance("tiler.build_OTV_table");

  typedef Correspondences::const_iterator corr_iterator;
  typedef Polygon_2::Vertex_iterator Vertex_iterator;

  using namespace std;

  OTV_table otv_table;

  for (ContourIterator it = contours_begin; it != contours_end; ++it)
  {
    Contour_handle contour = *it;

    // get vertices of all contours that correspond to the current contour
    vector<Point_3> points;
    for (corr_iterator cit = correspondences.begin(contour); cit != correspondences.end(contour); ++cit)
      vertices.get_vertices(*cit, back_inserter(points));

    // loop through each vertex v on current contour; sort all corresponding vertices
    // according to distance from v.
    for (Vertex_iterator vit = contour->polygon().vertices_begin(); vit != contour->polygon().vertices_end(); ++vit)
    {
      Point_3 vertex = vertices.get(contour, vit);
      LOG4CPLUS_TRACE(logger, "Finding OTV for " << pp(vertex));

      sort(points.begin(), points.end(), Distance_functor_2(vertex));

      LOG4CPLUS_TRACE(logger, "  Candidates in order: ");
      for (vector<Point_3>::iterator test_it = points.begin(); test_it != points.end(); ++test_it)
	LOG4CPLUS_TRACE(logger, "    " << test_it->id());

      // loop through sorted vertices until we find a valid point
      bool found = false;
      for (typename vector<Point_3>::const_iterator pit = points.begin(); !found && pit != points.end(); ++pit)
      {
	const Point_3& test_vertex = *pit;
	LOG4CPLUS_TRACE(logger, "  Testing " << test_vertex.id());
	Point_2 test_point = test_vertex.point_2();
// 	found = (test_point == vertex.point_2());
	found = (test_point == vertex.point_2() && tiling_regions[vertex]->contains(test_point));

// 	if (found) {
// 	  if (is_illegal(test_point, vertex, vertices, h)) {
// 	    banned.insert(test_point);
// 	    banned.insert(vertex);
// 	  }
// 	}
	if (!found)
	{
	  Segment_2 segment(vertex.point_2(), test_point);
	  found = (tiling_regions[vertex]->contains(test_point) &&
		   !intersects_proper(segment, c1_begin, c1_end) && 
		   !intersects_proper(segment, c2_begin, c2_end));
	}
// 	found = found && test31a(test_point, vertex, vertices, h);
// 	found = found && 
// 	  !is_backwards(test_point, vertices.contour(vertex), vertices, h) &&
// 	  !is_backwards(vertex, vertices.contour(test_point), vertices, h);
	if (found)
	{
	  otv_table[vertex] = test_vertex;
	  LOG4CPLUS_TRACE(logger, "  Assigning " << test_vertex.id());
	  if (vertex.z() == test_vertex.z())
	  {
	    LOG4CPLUS_ERROR(logger, "OTV is in the same slice.  Points: " << pp(vertex) << " " << pp(test_vertex));
	    throw std::logic_error("OTV is in the same slice");
	  }
	}
      }
    }
  }

  return otv_table;
}

template OTV_table build_OTV_table(std::vector<Contour_handle>::iterator contours_begin, std::vector<Contour_handle>::iterator contours_end,
				   std::vector<Contour_handle>::iterator c1_begin, std::vector<Contour_handle>::iterator c1_end,
				   std::vector<Contour_handle>::iterator c2_begin, std::vector<Contour_handle>::iterator c2_end,
				   const Vertices& vertices,
				   const Correspondences& correspondences,
				   const Vertex_map<HTiling_region>& tiling_regions,
				   const Hierarchies& h,
				   Banned& banned);
template OTV_table build_OTV_table(std::vector<Contour_handle>::const_iterator contours_begin, std::vector<Contour_handle>::const_iterator contours_end,
				   std::vector<Contour_handle>::const_iterator c1_begin, std::vector<Contour_handle>::const_iterator c1_end,
				   std::vector<Contour_handle>::const_iterator c2_begin, std::vector<Contour_handle>::const_iterator c2_end,
				   const Vertices& vertices,
				   const Correspondences& correspondences,
				   const Vertex_map<HTiling_region>& tiling_regions,
				   const Hierarchies& h,
				   Banned& banned);

// theorem 6.2
bool can_tile_disjoint(Contour_handle c1, Contour_handle c2, const Hierarchy& h1, const Hierarchy& h2)
{
  typedef Polygon_2::Vertex_iterator Vertex_iterator;

  if (h1.orientation(c1) == h2.orientation(c2))
    return false;

  const Polygon_2& polygon1 = c1->polygon();
  const Polygon_2& polygon2 = c2->polygon();
  CGAL::Orientation orientation1 = h1.orientation(c1);
  CGAL::Orientation orientation2 = h2.orientation(c2);
  Contour_handle NEC1 = h1.NEC(c1), NEC2 = h2.NEC(c2);

  bool has_neg_vertex1 = false, has_neg_vertex2 = false;
  bool insulated1 = true, insulated2 = true;
  Vertex_sign sign = Vertex_sign::POSITIVE;
  Contour_handle NEC;

  for (Vertex_iterator it = polygon1.vertices_begin(); it != polygon1.vertices_end(); ++it)
  {
    boost::tie(sign, NEC) = h2.vertex_sign(*it, orientation1);
    if (sign == Vertex_sign::NEGATIVE)
      has_neg_vertex1 = true;
    if (NEC == NEC2)
      insulated1 = false;
  }

  for (Vertex_iterator it = polygon2.vertices_begin(); it != polygon2.vertices_end(); ++it)
  {
    boost::tie(sign, NEC) = h1.vertex_sign(*it, orientation2);
    if (sign == Vertex_sign::NEGATIVE)
      has_neg_vertex2 = true;
    if (NEC == NEC1)
      insulated2 = false;
  }

  return (has_neg_vertex1 && has_neg_vertex2 && !insulated1 && !insulated2);
}

// theorem 6.3
// Precondition: c1 is inside c2
bool can_tile_nested(Contour_handle c1, Contour_handle c2, const Hierarchy& h1, const Hierarchy& h2)
{
  typedef Polygon_2::Vertex_iterator Vertex_iterator;

  if (h1.orientation(c1) != h2.orientation(c2))
    return false;

  const Polygon_2& polygon1 = c1->polygon();
  const Polygon_2& polygon2 = c2->polygon();
  CGAL::Orientation orientation1 = h1.orientation(c1);
  CGAL::Orientation orientation2 = h2.orientation(c2);
  Contour_handle NEC1 = h1.NEC(c1), NEC2 = h2.NEC(c2);

  bool has_neg_vertex1 = false, has_pos_vertex2 = false;
  bool insulated1 = true, insulated2 = true;
  Vertex_sign sign = Vertex_sign::POSITIVE;
  Contour_handle NEC;

  for (Vertex_iterator it = polygon1.vertices_begin(); it != polygon1.vertices_end(); ++it)
  {
    boost::tie(sign, NEC) = h2.vertex_sign(*it, orientation1);
    if (sign == Vertex_sign::NEGATIVE)
      has_neg_vertex1 = true;
  }
  insulated1 = (c2 != h2.NEC(c1));

  for (Vertex_iterator it = polygon2.vertices_begin(); it != polygon2.vertices_end(); ++it)
  {
    boost::tie(sign, NEC) = h1.vertex_sign(*it, orientation2);
    if (sign == Vertex_sign::POSITIVE)
      has_pos_vertex2 = true;
    if (NEC == NEC1)
      insulated2 = false;
  }

  return (has_neg_vertex1 && has_pos_vertex2 && !insulated1 && !insulated2);
}

/// Returns true if a tiling tile can exist between two contours 
/// according to the requirements given in Bajaj96, theorem 6.
bool can_tile(Contour_handle c1, Contour_handle c2, const Hierarchy& h1, const Hierarchy& h2, Number_type overlap)
{
  Logger logger = Logger::getInstance("tiler.can_tile");
  LOG4CPLUS_TRACE(logger, "Testing if tiling can exist: " << pp(c1->polygon()) << " " << pp(c2->polygon()));

  typedef Polygon_2::Vertex_iterator Vertex_iterator;

  Polygon_relation rel = relation(c1->polygon(), c2->polygon(), false);

  bool ret;

  // Theorem 6.1
  if (rel == Polygon_relation::BOUNDARY_INTERSECT)
  {
    ret = true;
    if (overlap > 0) {
      list<Polygon_with_holes_2> intersections;
      polygon_intersection(c1->polygon(), c2->polygon(), back_inserter(intersections));
      Number_type area = 0;
      for (list<Polygon_with_holes_2>::const_iterator it = intersections.begin(); it != intersections.end(); ++it) {
	area += it->outer_boundary().area();
      }
      ret = area > overlap;
    }
    if (ret)
      LOG4CPLUS_TRACE(logger, "Boundaries intersect");
  }
  // Theorem 6.2
  else if (rel == Polygon_relation::SIBLING)
  {
    ret = can_tile_disjoint(c1, c2, h1, h2);
    LOG4CPLUS_TRACE(logger, "Sibling");
  }
  // Theorem 6.3
  else if (rel == Polygon_relation::PARENT)
  {
    ret = can_tile_nested(c2, c1, h2, h1);
    LOG4CPLUS_TRACE(logger, "Parent");
  }
  // Theorem 6.3
  else
  {
    ret = can_tile_nested(c1, c2, h1, h2);
    LOG4CPLUS_TRACE(logger, "Child");
  }

  return ret;
}

HTiling_region tiling_region(Polygon_2::Vertex_circulator vertex, /*const Contour_handle contour, */
			     const Hierarchy& h, const Hierarchy& h_opp,
			     Number_type z, Number_type z_opp)
{
  Logger logger = Logger::getInstance("tiler.tiling_region");

  typedef Polygon_2::Vertex_circulator Vertex_circulator;

  Vertex_circulator prev = vertex - 1;
  Vertex_circulator next = vertex + 1;

  HTiling_region region;

  Contour_handle overlapping_contour;
  Contour_handle nec;
  Polygon_2::Vertex_circulator V_opp;
  boost::tie(overlapping_contour, nec, V_opp) = h_opp.is_overlapping(*vertex);
  if (!overlapping_contour)
  {
    if (h_opp.is_CW(nec)) {
      region = Wedge::RS(*prev, *vertex, *next, z);
      region = region->get_complement();
    }
    else {
      region = Wedge::LS(*prev, *vertex, *next, z_opp);
      region = region->get_complement();
    }
  }
  else
  {
    region = Tiling_region::overlapping_vertex(*prev, *vertex, *next, 
					       *(V_opp-1), *V_opp, *(V_opp+1));
  }
  
  LOG4CPLUS_TRACE(logger, pp(*vertex) << " -- " << region->z_home_nothrow());

  return region;
}

template <typename ContourIterator>
Vertex_map<HTiling_region> find_tiling_regions(ContourIterator start1, ContourIterator end1, const Hierarchy& h1, 
					       ContourIterator start2, ContourIterator end2, const Hierarchy& h2,
					       Number_type z1, Number_type z2,
					       const Vertices& vertices)
{
  typedef Polygon_2::Vertex_circulator Vertex_circulator;

  Vertex_map<HTiling_region> regions(10000);

//   Number_type z1 = (*start1)->slice();
//   Number_type z2 = (*start2)->slice();

  for (ContourIterator it1 = start1; it1 != end1; ++it1)
  {
    Contour_handle contour = *it1;
    Vertex_circulator beg = contour->polygon().vertices_circulator();
    Vertex_circulator it = beg;
    do
    {
      Point_3 v = vertices.get(contour, it);
      regions[v] = tiling_region(it, /*contour,*/ h1, h2, z1, z2);
      ++it;
    } while (it != beg);
  }

  for (ContourIterator it2 = start2; it2 != end2; ++it2)
  {
    Contour_handle contour = *it2;
    Vertex_circulator beg = contour->polygon().vertices_circulator();
    Vertex_circulator it = beg;
    do
    {
      Point_3 v = vertices.get(contour, it);
      regions[v] = tiling_region(it, /*contour,*/ h2, h1, z2, z1);
      ++it;
    } while (it != beg);
  }

  return regions;
}

template
Vertex_map<HTiling_region> find_tiling_regions(std::vector<Contour_handle>::iterator start1, std::vector<Contour_handle>::iterator end1, 
					       const Hierarchy& h1, 
					       std::vector<Contour_handle>::iterator start2, std::vector<Contour_handle>::iterator end2, 
					       const Hierarchy& h2,
					       Number_type z1, Number_type z2,
					       const Vertices& vertices);
template
Vertex_map<HTiling_region> find_tiling_regions(std::vector<Contour_handle>::const_iterator start1, std::vector<Contour_handle>::const_iterator end1, 
					       const Hierarchy& h1, 
					       std::vector<Contour_handle>::const_iterator start2, std::vector<Contour_handle>::const_iterator end2, 
					       const Hierarchy& h2,
					       Number_type z1, Number_type z2,
					       const Vertices& vertices);

bool is_OTV_pair(const Point_3& v1, const Point_3& v2, const OTV_table& otv_table)
{
  return v1.is_valid() && v2.is_valid() && otv_table[v1] == v2 && otv_table[v2] == v1;
}

OTV_pair find_OTV_pair(const Tiler_workspace& w)
{
  for (Vertex_iterator it = w.vertices.begin(); it != w.vertices.end(); ++it)
//   for (OTV_table::const_iterator it = otv_table.begin(); it != otv_table.end(); ++it)
  {
    const Point_3& v0 = *it;
    const Point_3& v1 = w.otv_table[v0];
    if (is_OTV_pair(v0, v1, w.otv_table))
      return OTV_pair(v0, v1);
  }
  return OTV_pair(Point_3(), Point_3());
}

template <typename OutputIterator>
void find_all_OTV_pairs(const Tiler_workspace& w, OutputIterator pairs)
{
  Number_type slice = w.vertices_begin()->z();
  for (Vertex_iterator it = w.vertices_begin(); it != w.vertices_end(); ++it)
//   size_t slice = otv_table.begin()->first.contour()->info().z();
//   for (OTV_table::const_iterator it = otv_table.begin(); it != otv_table.end(); ++it)
  {
    const Point_3& v0 = *it;
    if (it->z() == slice)
    {
      const Point_3& v1 = (w.otv_table)[v0];
      if (is_OTV_pair(v0, v1, w.otv_table))
      {
	*pairs = OTV_pair(v0, v1);
	++pairs;
      }
    }
//     if (it->first.contour()->info().z() == slice && is_OTV_pair(it->first, it->second, otv_table))
//     {
//       *pairs = *it;
//       ++pairs;
//     }
  }
}

void find_banned(Tiler_workspace& w)
{
//   std::list<OTV_pair> otv_pairs;
//   find_all_OTV_pairs(w, back_inserter(otv_pairs));
//   for (std::list<OTV_pair>::iterator it = otv_pairs.begin(); it != otv_pairs.end(); ++it)
//   {
//     // u = top
//     // v = bottom
//     Point_3 u2 = it->top();
//     Point_3 v2 = it->bottom();
    
//     if (xy_equal(u2, v2)) {
//       Walk_direction dir = Walk_direction::FORWARD;
//       Point_3 u1 = w.vertices.adjacent(u2, opposite(dir), w.hierarchies);
//       Point_3 u3 = w.vertices.adjacent(u2, dir, w.hierarchies);
//       Point_3 v1 = w.vertices.adjacent(v2, opposite(dir), w.hierarchies);
//       Point_3 v3 = w.vertices.adjacent(v2, dir, w.hierarchies);

//       bool banned = is_illegal(u2, v2, w.vertices, w.hierarchies);

// //       bool banned = (xy_equal(u1, v3) || xy_equal(u3, v1));
// //       if (!banned) {
      
// //       }
//       if (banned) {
// 	w.add_banned(u2);
// 	w.add_banned(v2);
//       }
//     }
//   }
}

int optimality(const Point_3& u2, const Point_3& u3, const Point_3& v2, const Point_3& v3, const Tiler_workspace& w)
{
  const OTV_table& otv_table = w.otv_table;

  if (w.contour(u2) != w.contour(u3) || w.contour(v2) != w.contour(v3))
    throw std::logic_error("u2,u3 and v2,v3 must lie on respective contours");

  if (is_OTV_pair(u2, v2, otv_table) && 
      is_OTV_pair(u3, v3, otv_table))
    return 1;

//   if (u2.point() == v2.point())
  if (xy_equal(u2, v2))
    return 2;

  if (is_OTV_pair(u2, v2, otv_table) && 
      v2 == otv_table[u3])
    return 3;

  if (is_OTV_pair(u2, v2, otv_table) && 
      v2 != otv_table[u3] &&
      w.contour(v2) == w.contour(otv_table[u3]))
    return 4;

  if (!is_OTV_pair(u2, v2, otv_table) && 
      w.contour(v2) == w.contour(otv_table[u2]) &&
      w.contour(v2) == w.contour(otv_table[u3]))
    return 5;

  return 6;
}

void add_chord(const Boundary_slice_chord& chord, Boundary_slice_chords& bscs)
{
  bscs.put(chord);
}

string pp(size_t a, size_t b, size_t c)
{
  std::stringstream ss;
  if (a > b)
    swap(a, b);
  if (b > c)
    swap(b, c);
  if (a > b)
    swap(a, b);
  ss << a << " " << b << " " << c;
  return ss.str();
}

string pp(const Point_3& a, const Point_3& b, const Point_3& c)
{
//   return pp(a.id(), b.id(), c.id());
  std::stringstream ss;
  ss << a.id() << " " << b.id() << " " << c.id();// << " " << a.id() << " " << b.id();
  return ss.str();
}

string pp(const Segment_3& s, const Point_3& p)
{
  return pp(s.source(), s.target(), p);
}

string pp(Walk_direction dir)
{
  if (dir == Walk_direction::FORWARD)
    return "forward";
  return "backward";
}

/// See unit test31.
bool test31(const Point_3& u2, const Point_3& target, const Point_3& v2, Tiler_workspace& w)
{
  if (!xy_equal(u2, v2)) return true;

  Walk_direction dir = Walk_direction::FORWARD;
  Point_3 u1 = w.vertices.adjacent(u2, opposite(dir), w.hierarchies);
  Point_3 u3 = w.vertices.adjacent(u2, dir, w.hierarchies);
  Point_3 v1 = w.vertices.adjacent(v2, opposite(dir), w.hierarchies);
  Point_3 v3 = w.vertices.adjacent(v2, dir, w.hierarchies);

  if (xy_equal(u1, v3)) {
//     w.vertices.contour(v2)
    return target != u1;
  }
  else if (xy_equal(u3, v1)) {
    return target != u3;
  }
  return true;
}

/// See unit test37
bool test37(const Segment_3& segment, const Point_3& opposite, Tiler_workspace& w)
{
  return !is_illegal(segment.source(), w.vertices.contour(opposite), w.vertices, w.hierarchies) &&
    !is_illegal(segment.target(), w.vertices.contour(opposite), w.vertices, w.hierarchies) &&
    !is_illegal(opposite, w.vertices.contour(segment.source()), w.vertices, w.hierarchies);
}

/// See unit test31.
bool test31(const Segment_3& segment, const Point_3& opposite, Tiler_workspace& w)
{
  return test31(segment.source(), segment.target(), opposite, w) &&
    test31(segment.target(), segment.source(), opposite, w);
}

/// @param segment contour segment in contour order, that is, if the contour is CCW, then the
///   segment should be given in CCW order
/// @param opposite vertex opposite the segment, that is, vertex lying in the other slice
bool add_if_possible(const Segment_3& segment, const Point_3& opposite, Tiler_workspace& w)
{
  Logger logger = Logger::getInstance("tiler.add_if_possible");
  
  if (w.vertices.adjacent(segment.source(), Walk_direction::FORWARD, w.hierarchies) != segment.target())
    throw std::logic_error("segment must be in contour ordering");

  LOG4CPLUS_TRACE(logger, "Testing " << pp(segment, opposite));

  if (w.completion_map.is_complete(segment)) {
    LOG4CPLUS_TRACE(logger, "Failed to add tile " << pp(segment, opposite) << ": segment is already part of a tile" );
    return false;
  }

//   if (!test31(segment, opposite, w)) {
//     LOG4CPLUS_TRACE(logger, "Failed to add tile " << pp(segment, opposite) << ": test31 failed" );
//     return false;
//   }
//   if (!test31(segment, opposite, w)) {
//     LOG4CPLUS_TRACE(logger, "Failed to add tile " << pp(segment, opposite) << ": test31 failed" );
//     return false;
//   }
//   if (w.is_banned(segment.source())) {
//     LOG4CPLUS_TRACE(logger, "Failed to add tile " << pp(segment, opposite) << ": " << pp(segment.source()) << " is banned");
//     return false;
//   }
//   if (w.is_banned(segment.target())) {
//     LOG4CPLUS_TRACE(logger, "Failed to add tile " << pp(segment, opposite) << ": " << pp(segment.target()) << " is banned");
//     return false;
//   }
//   if (w.is_banned(opposite)) {
//     LOG4CPLUS_TRACE(logger, "Failed to add tile " << pp(segment, opposite) << ": " << pp(opposite) << " is banned");
//     return false;
//   }

  // The slice chord between the source and opposite goes from bottom slice to top slice.
  // The slice chord between the target and opposite goes from top slice to bottom slice.
  // Assume that opposite is on the top and then switch the slice chord ordering if that
  // is not the case.
  Segment_3 source_chord(segment.source(), opposite);
  Segment_3 target_chord(opposite, segment.target());
  if (opposite.z() < segment.source().z())
  {
    source_chord = source_chord.opposite();
    target_chord = target_chord.opposite();
  }

  bool pass2s = test_theorem2(source_chord, w);
  bool pass2t = test_theorem2(target_chord, w);
  bool pass3 = test_theorem3(segment, opposite, w);
  bool pass4 = test_theorem4(segment, opposite, w);
  bool pass5 = test_theorem5(source_chord, target_chord, w.bscs);
  bool pass8 = test_theorem8(segment, opposite, w);
  bool pass9 = test_theorem9(segment, opposite, w);
  bool pass10 = !w.bscs.retired(source_chord) && !w.bscs.retired(target_chord);
  bool add = pass2s && pass2t && pass3 && pass4 && pass5 && pass8 && pass9 && pass10;

  if (add)
  {
    LOG4CPLUS_TRACE(logger, "Added tile " << pp(segment, opposite));
    w.add_tile(segment.source(), segment.target(), opposite);
    w.completion_map.put(segment);
    Boundary_slice_chord bsc0(source_chord, Walk_direction::BACKWARD, segment.source().z());
    Boundary_slice_chord bsc1(target_chord, Walk_direction::FORWARD, segment.source().z());
    add_chord(bsc0, w.bscs);
    add_chord(bsc1, w.bscs);
  }
  else
    LOG4CPLUS_TRACE(logger, "Failed to add tile " << pp(segment, opposite) << ": " 
		    << pass2s << " " << pass2t << " " << pass3 << " " 
		    << pass4 << " " << pass5 << " " 
		    << pass8 << " " << pass9 << " " << pass10);

//   LOG4CPLUS_TRACE(logger, "add_if_possible returning: " << add);
  return add;
}

bool add_if_possible(const Point_3& v0, const Point_3& v1, const Point_3& v2,
		     Tiler_workspace& w)
{
  Segment_3 segment(v0, v1);
  if (w.vertices.adjacent(v0, Walk_direction::FORWARD, w.hierarchies) != v1)
    segment = Segment_3(v1, v0);
  return add_if_possible(segment, v2, w);
}

bool add_if_possible(Point_3* points, Tiler_workspace& w)
{
  return add_if_possible(points[0], points[1], points[2], w);
}

void test31_swap(const Point_3& u2, const Point_3& v2, Point_3& u3, Point_3& v3, 
		 Walk_direction dir, const Tiler_workspace& w)
{
  if (!xy_equal(u2, v2)) return;

  Point_3 u1 = w.vertices.adjacent(u2, opposite(dir), w.hierarchies);
  Point_3 v1 = w.vertices.adjacent(v2, opposite(dir), w.hierarchies);

  if (xy_equal(u1, v3)) {
    u3 = u1;
  }
  else if (xy_equal(v1, u3)) {
    v3 = v1;
  }
}

/// Pass 1, handling cases 1-3
/// u2 is always the top vertex
void case1_3(const Point_3& u2, const Point_3& v2, Walk_direction dir,
	   Tiler_workspace& w)
{
  static Logger logger = Logger::getInstance("tiler.stage1.case1_3");

  Point_3 u1 = w.vertices.adjacent(u2, opposite(dir), w.hierarchies);
  Point_3 v1 = w.vertices.adjacent(v2, opposite(dir), w.hierarchies);
  Point_3 u3 = w.vertices.adjacent(u2, dir, w.hierarchies);
  Point_3 v3 = w.vertices.adjacent(v2, dir, w.hierarchies);

//   if (is_OTV_pair(u3, v1, w.otv_table)) {
//     LOG4CPLUS_TRACE(logger, "swapping v3");
//     swap(v1, v3);
//   }

//   if (is_backwards_new(u2, u3, v2, v3)) {
//     LOG4CPLUS_TRACE(logger, "     \\/  OTV pair");
//     LOG4CPLUS_TRACE(logger, "u2: " << setw(2) << u2.id() << " u3: " << setw(2) << u3.id());
//     LOG4CPLUS_TRACE(logger, "v2: " << setw(2) << v2.id() << " v3: " << setw(2) << v3.id());
//     v3 = w.vertices.adjacent(v2, opposite(dir), w.hierarchies);
//     LOG4CPLUS_TRACE(logger, "swapping v3");
//   }

//   test31_swap(u2, v2, u3, v3, dir, w);

  LOG4CPLUS_TRACE(logger, "     \\/  OTV pair");
//   LOG4CPLUS_TRACE(logger, "u2: " << pp(u2) << " u3: " << pp(u3));
//   LOG4CPLUS_TRACE(logger, "v2: " << pp(v2) << " v3: " << pp(v3));
  LOG4CPLUS_TRACE(logger, "u2: " << setw(2) << u2.id() << " u3: " << setw(2) << u3.id());
  LOG4CPLUS_TRACE(logger, "v2: " << setw(2) << v2.id() << " v3: " << setw(2) << v3.id());
//   LOG4CPLUS_TRACE(logger, "u2: " << pp(u2) << " v2: " << pp(v2) << " (OTV pair)");
//   LOG4CPLUS_TRACE(logger, "u3: " << pp(u3) << " v3: " << pp(v3));
  LOG4CPLUS_TRACE(logger, "dir: " << pp(dir));

  int optimal = optimality(u2, u3, v2, v3, w);
  int optimal_r = optimality(v2, v3, u2, u3, w);

  LOG4CPLUS_TRACE(logger, "opt1: " << optimal << " opt2: " << optimal_r);

  Number_type v2u3 = CGAL::squared_distance(v2.point_2().point_2(), u3.point_2().point_2());
  Number_type u2v3 = CGAL::squared_distance(u2.point_2().point_2(), v3.point_2().point_2());
  bool v2u3_shorter = (v2u3 < u2v3);
  if (v2u3 == u2v3) {
    // If u3 and v3 are OTV and have already been tiled, then we must use
    // the chord created between u3-v2 or v3-u2.
    if (w.bscs.contains(u3, v2))
      v2u3_shorter = true;
    if (w.bscs.contains(v3, u2))
      v2u3_shorter = true;
  }
  // Whether to tile the upper-(left|right) and lower-(left|right) 
  // tiles.  See figure 12 in bajaj96.
  bool ul = false, ll = false, ur = false, lr = false;
  if (optimal == 1)
  {
    ul = lr = v2u3_shorter;
    ll = ur = !v2u3_shorter;
  }
  else if (optimal == 2 || (optimal == 3 && optimal_r == 3))
  {
//     ul = v2u3_shorter;
//     ll = !v2u3_shorter;
    if (v2u3_shorter)
    {
      if (!add_if_possible(u2, u3, v2, w))
	add_if_possible(v2, v3, u2, w);
    }
    else
    {
      if (!add_if_possible(v2, v3, u2, w))
	add_if_possible(u2, u3, v2, w);
    }
  }
  else if (optimal == 3)
  {
//     ul = true;
    if (!add_if_possible(u2, u3, v2, w) && optimal_r == 3)
      add_if_possible(v2, v3, u2, w);
  }
  else if (optimal_r == 3)
    ll = true;

  bool added = false;
  if (ll)
    added = add_if_possible(v2, v3, u2, w);
  if (ur)
    added = added || add_if_possible(u3, u2, v3, w);
  if (ul)
    added = added || add_if_possible(u2, u3, v2, w);
  if (lr)
    added = added || add_if_possible(v3, v2, u3, w);

}

void init(boost::shared_array<Point_3>& pts, const Point_3& p0, const Point_3& p1, const Point_3& p2)
{
  pts.reset(new Point_3[3]);
  pts[0] = p0;
  pts[1] = p1;
  pts[2] = p2;
}

pair<Point_3, Point_3> get_u3v3(const Point_3& u2, const Point_3& v2, Number_type seg_z, 
	     Walk_direction dir, Tiler_workspace& w)
{
  static Logger logger = Logger::getInstance("tiler.stage1.case2_4.get_u3v3");

  Point_3 u3 = w.vertices.adjacent(u2, dir, w.hierarchies);
  Point_3 v3 = w.vertices.adjacent(v2, dir, w.hierarchies);
//   bool backwards = is_backwards(u2, w.vertices.contour(v2), w.vertices, w.hierarchies) ||
//     is_backwards(v2, w.vertices.contour(u2), w.vertices, w.hierarchies);

//   if (is_backwards(u2, u3, v2, v3))
//     v3 = w.vertices.adjacent(v2, opposite(dir), w.hierarchies);

//   if (u2.z() == seg_z && is_backwards(
//       is_backwards(v2, w.vertices.contour(u2), w.vertices, w.hierarchies) &&
//       xy_equal(u2, v2))
//   {
//     v3 = w.vertices.adjacent(v2, opposite(dir), w.hierarchies);
//     LOG4CPLUS_TRACE(logger, "Changing v3.  u2 = " << pp(u2) << " v2 = " << pp(v2) << " seg_z = " << seg_z);
//   }
//   else if (v2.z() == seg_z && 
// 	   is_backwards(u2, w.vertices.contour(v2), w.vertices, w.hierarchies) &&
// 	   xy_equal(u2, v2))
//   {
//     u3 = w.vertices.adjacent(u2, opposite(dir), w.hierarchies);
//     LOG4CPLUS_TRACE(logger, "Changing u3.  u2 = " << pp(u2) << " v2 = " << pp(v2) << " seg_z = " << seg_z);
//   }
  if (u2.z() == seg_z && 
      is_backwards(v2, w.vertices.contour(u2), w.vertices, w.hierarchies) &&
      xy_equal(u2, v2))
  {
    v3 = w.vertices.adjacent(v2, opposite(dir), w.hierarchies);
    LOG4CPLUS_TRACE(logger, "Changing v3.  u2 = " << pp(u2) << " v2 = " << pp(v2) << " seg_z = " << seg_z);
  }
  else if (v2.z() == seg_z && 
	   is_backwards(u2, w.vertices.contour(v2), w.vertices, w.hierarchies) &&
	   xy_equal(u2, v2))
  {
    u3 = w.vertices.adjacent(u2, opposite(dir), w.hierarchies);
    LOG4CPLUS_TRACE(logger, "Changing u3.  u2 = " << pp(u2) << " v2 = " << pp(v2) << " seg_z = " << seg_z);
  }
  return make_pair(u3, v3);
}

/// Passes 2-4, handling cases 4-6.
bool case2_4(int target_optimality, const Point_3& u2, const Point_3& v2, Number_type seg_z, 
	     Walk_direction dir, Tiler_workspace& w)
{
  static Logger logger = Logger::getInstance("tiler.stage1.case2_4");

//   Point_3 u3, v3;
//   boost::tie(u3, v3) = get_u3v3(u2, v2, seg_z, dir, w);
  Point_3 u3 = w.vertices.adjacent(u2, dir, w.hierarchies);
  Point_3 v3 = w.vertices.adjacent(v2, dir, w.hierarchies);

//   if (is_backwards_new(u2, u3, v2, v3)) {
//     if (u2.z() == seg_z) {
//       LOG4CPLUS_TRACE(logger, "swapping v3");
//       v3 = w.vertices.adjacent(v2, opposite(dir), w.hierarchies);
//     }
//     else {
//       LOG4CPLUS_TRACE(logger, "swapping u3");
//       u3 = w.vertices.adjacent(u2, opposite(dir), w.hierarchies);
//     }
//   }
//   test31_swap(u2, v2, u3, v3, dir, w);

  int optimal = optimality(u2, u3, v2, v3, w);
  int optimal_r = optimality(v2, v3, u2, u3, w);
  bool v2u3_shorter = (CGAL::squared_distance(v2.point_2().point_2(), u3.point_2().point_2()) <
		       CGAL::squared_distance(u2.point_2().point_2(), v3.point_2().point_2()));
  bool u2_complete = w.completion_map.is_complete(u2, dir);
  bool v2_complete = w.completion_map.is_complete(v2, dir);
  // Whether to tile the upper or lower tile.  See figure 12 in bajaj96.
  bool upper = false, lower = false;
  bool added = false;

//   LOG4CPLUS_TRACE(logger, "u2: " << pp(u2) << " u3: " << pp(u3));
//   LOG4CPLUS_TRACE(logger, "v2: " << pp(v2) << " v3: " << pp(v3));
  LOG4CPLUS_TRACE(logger, "u2: " << setw(2) << u2.id() << " u3: " << setw(2) << u3.id());
  LOG4CPLUS_TRACE(logger, "v2: " << setw(2) << v2.id() << " v3: " << setw(2) << v3.id());
  LOG4CPLUS_TRACE(logger, "dir: " << pp(dir) << " seg_z point: " << ((u2.z() == seg_z) ? u2.id() : v2.id()));
  LOG4CPLUS_TRACE(logger, "optimal: " << optimal << " optimal_r: " << optimal_r);
  LOG4CPLUS_TRACE(logger, "u2_complete: " << u2_complete << " v2_complete: " << v2_complete);
  LOG4CPLUS_TRACE(logger, "v2u3_shorter: " << v2u3_shorter);


  boost::shared_array<Point_3> preferred, secondary;

  if (optimal <= target_optimality && optimal_r <= target_optimality)
  {
    if (!u2_complete)
      init(preferred, u3, u2, v2);
    if (!v2_complete)
      init(secondary, v2, v3, u2);
    if (!v2u3_shorter)
      std::swap(preferred, secondary);
  }
  else if (optimal <= target_optimality)
    upper = !u2_complete;
  else if (optimal_r <= target_optimality)
    lower = !v2_complete;

  if (lower)
  {
    added = add_if_possible(v2, v3, u2, w);
//     LOG4CPLUS_TRACE(logger, "1: " << added);
  }
  if (upper)
  {
    added = added || add_if_possible(u3, u2, v2, w);
//     LOG4CPLUS_TRACE(logger, "2: " << added);
  }

  if (preferred)
  {
    added = added || add_if_possible(preferred.get(), w);
//     LOG4CPLUS_TRACE(logger, "3: " << added);
  }
  if (secondary)
  {
//     LOG4CPLUS_TRACE(logger, "4a: " << added);
    bool tb = add_if_possible(secondary.get(), w);
//     LOG4CPLUS_TRACE(logger, "4b: " << tb);
    added = added || tb;//add_if_possible(secondary.get(), w);
//     LOG4CPLUS_TRACE(logger, "4c: " << added);
  }
  
  return added;
}

/// pass 2, 3 or 4
void pass2_4(size_t case_num, Tiler_workspace& w)
{
  static Logger logger = Logger::getInstance("tiler.stage1.pass2_4");

//   Boundary_slice_chord chords
  list<Boundary_slice_chord> chords;
  w.bscs.all(back_inserter(chords));

  for (list<Boundary_slice_chord>::iterator it = chords.begin(); it != chords.end(); ++it)
  {
    const Boundary_slice_chord& bsc = *it;
    if (w.bscs.contains(bsc))
    {
      LOG4CPLUS_TRACE(logger, "Checking boundary slice chord: " << pp(bsc.segment()));
      Walk_direction dir = bsc.direction();
      Segment_3 segment = bsc.segment();
      Point_3 u2 = segment.source();
      Point_3 v2 = segment.target();
      Number_type seg_z = bsc.seg_z();
      // stop when we either run out of incomplete segments or we fail to add one
      bool keep_going = true;
      while (keep_going)
      {
	LOG4CPLUS_TRACE(logger, "Continuing from last addition: " << pp(Segment_2(u2, v2)));

	bool u2_complete = w.completion_map.is_complete(u2, dir);
	bool v2_complete = w.completion_map.is_complete(v2, dir);

	const Boundary_slice_chord& tmp_bsc = w.bscs.get_chord(u2, v2);
	seg_z = tmp_bsc.seg_z();

	keep_going = case2_4(case_num, u2, v2, seg_z, dir, w);

	if (keep_going)
	{
	  bool u2b = w.completion_map.is_complete(u2, dir);
	  bool v2b = w.completion_map.is_complete(v2, dir);
	  bool u2_completed = !u2_complete && u2b;
	  bool v2_completed = !v2_complete && v2b;
	  u2_complete = u2b;
	  v2_complete = v2b;

	  if (u2_completed)
	    u2 = w.vertices.adjacent(u2, dir, w.hierarchies);
	  if (v2_completed)
	    v2 = w.vertices.adjacent(v2, dir, w.hierarchies);

	  keep_going = !w.completion_map.is_complete(u2, dir)
	    || !w.completion_map.is_complete(v2, dir);

	  keep_going = keep_going && w.bscs.contains(u2, v2);
	}
      }
    }
    else
    {
      LOG4CPLUS_TRACE(logger, "Boundary slice chord already removed: " << pp(bsc.segment()));
    }
  }
}

/// Searches backwards through the untiled region until a point is found that
/// is not on the boundary of the given polygon.  The relation of this point
/// to the polygon is given.
template <typename PointIterator>
CGAL::Oriented_side side(PointIterator begin, PointIterator end, const Polygon_2& polygon)
{
  for (PointIterator it = begin; it != end; ++it)
  {
    CGAL::Oriented_side s = polygon.oriented_side(it->point_2());
    if (s != CGAL::ON_ORIENTED_BOUNDARY)
      return s;
  }
  return CGAL::ON_ORIENTED_BOUNDARY;
}

class Point_on_polygon
{
public:
  Point_on_polygon() {}
  Point_on_polygon(const Polygon_2& P) : _P(P) {}
  bool operator()(const Point_2& p) {
    return find(_P.vertices_begin(), _P.vertices_end(), p) != _P.vertices_end();
  }
private:
  Polygon_2 _P;
};

class Point_not_on_polygon
{
public:
  Point_not_on_polygon() {}
  Point_not_on_polygon(const Polygon_2& P) : _P(P) {}
  bool operator()(const Point_2& p) {
    return find(_P.vertices_begin(), _P.vertices_end(), p) == _P.vertices_end();
  }
private:
  Polygon_2 _P;
};

/// Searches backwards through the untiled region until a point is found that
/// is not on the boundary of the given polygon.
template <typename PointIterator>
PointIterator last_not_on(PointIterator begin, PointIterator end, const Polygon_2& polygon)
{
  static Logger logger = Logger::getInstance("tiler.last_not_on");
  return find_if(begin, end, Point_on_polygon(polygon));
}

// /// Rules:
// ///  If coming from a boundary chord, first try to stay on the same polygon.  If
// ///    the adjacent vertex has already been visited, then take the next available
// ///    boundary chord.
// ///  If coming from an adjacent vertex, take a boundary chord if there is one available.
// ///    If not, stay on the polygon.
// ///  To choose between available boundary chords, choose the one whose destination
// ///    point is on the same oriented side as the last point that appeared in that slice.
// list<Untiled_region> trace_untiled_regions_bak(Tiler_workspace& w)
// {
//   static Logger logger = Logger::getInstance("tiler.trace_untiled_regions");

//   list<Untiled_region> polygons;

//   // Trace untiled regions bounded by boundary slice chords
//   while (w.bscs.size() > 0)
//   {
//     LOG4CPLUS_TRACE(logger, "Beginning untiled trace");

//     Boundary_slice_chord first_chord = w.bscs.first();

//     w.bscs.erase(first_chord);

//     Walk_direction dir = first_chord.direction();
//     Point_3 first = first_chord.segment().source();
//     Point_3 cur = first_chord.segment().target();
//     Number_type midz = (first.z() + cur.z()) / 2.0;
//     Untiled_region vertices;
//     vertices.push_back(first);
//     LOG4CPLUS_TRACE(logger, pp(first));
//     bool from_chord = true;

//     while (!xyz_equal(cur, first))
//     {
//       vertices.push_back(cur);
//       LOG4CPLUS_TRACE(logger, pp(cur));

//       Point_3 next = w.vertices.adjacent(cur, dir, w.hierarchies);
//       std::vector<Boundary_slice_chord> chords = w.bscs.on_vertex(cur);

//       // Determine whether to stay on the same slice or use a boundary slice chord
//       if (from_chord)
//       {
// 	// Can't stay on the same slice because the slice going forward
// 	// is complete.
// 	if (w.completion_map.is_complete(next, opposite(dir)))
// 	  next = Point_3();
//       }
//       else
//       {
// 	// Always take a chord to the opposite slice if not coming
// 	// from a chord already.
// 	if (chords.size() > 0)
// 	  next = Point_3();
//       }

//       // If we need to get the next from the boundary slice chords,
//       // choose which chord.
//       if (next.id() == DEFAULT_ID())
//       {
// 	Boundary_slice_chord chord(chords[0]);
// 	if (chords.size() > 1)
// 	{
// 	  const Polygon_2& polygon = w.vertices.contour(cur)->polygon();
// 	  CGAL::Oriented_side s = side(vertices.rbegin(), vertices.rend(), polygon);
// 	  for (std::vector<Boundary_slice_chord>::const_iterator it = chords.begin();
// 	       it != chords.end();
// 	       ++it)
// 	  {
// 	    if (polygon.oriented_side(it->segment().target()) == s)
// 	      chord = *it;
// 	  }
// 	}
// 	next = chord.segment().target();
// 	w.bscs.erase(chord);
// 	dir = opposite(dir);
// 	from_chord = true;
//       }
//       else
//       {
// 	from_chord = false;
// 	if (dir == Walk_direction::FORWARD)
// 	  w.completion_map.put(Segment_3(cur, next));
// 	else
// 	  w.completion_map.put(Segment_3(next, cur));
//       }
//       cur = next;
//     }

//     LOG4CPLUS_TRACE(logger, "Ending untiled trace");
//     polygons.push_back(vertices);
//   }

//   // Any remaining regions should be in contours that are completely untiled
//   for (std::vector<Contour_handle>::const_iterator it = w.bottom.begin();
//        it != w.bottom.end();
//        ++it)
//   {
//     const Polygon_2& polygon = (*it)->polygon();
//     if (!w.completion_map.is_complete(polygon[0]))
//     {
//       Untiled_region poly;
//       poly.insert(polygon.vertices_begin(), polygon.vertices_end());
//       polygons.push_back(poly);
//     }
//   }
//   for (std::vector<Contour_handle>::const_iterator it = w.top.begin();
//        it != w.top.end();
//        ++it)
//   {
//     const Polygon_2& polygon = (*it)->polygon();
//     if (!w.completion_map.is_complete(polygon[0]))
//     {
//       Untiled_region poly;
//       poly.insert(polygon.vertices_begin(), polygon.vertices_end());
//       poly.reverse_orientation();
//       polygons.push_back(poly);
//     }
//   }

//   return polygons;
// }

Untiled_region trace_untiled_region(Point_3 cur, Untiled_region vertices, 
				     Walk_direction dir, Tiler_workspace& w)
{
  static Logger logger = Logger::getInstance("tiler.trace_untiled_region");

  typedef std::vector<Boundary_slice_chord>::const_iterator BSC_iter;

  LOG4CPLUS_TRACE(logger, "cur = " << pp(cur));
  LOG4CPLUS_TRACE(logger, "  dir = " << dir);

  // Check if we've completed the loop
  if (vertices.size() > 1) {
    // Completed correctly
    if (cur.id() == vertices.begin()->id()) {
      return vertices;
    }
    // Illegal cycle
    else if (find(vertices.begin(), vertices.end(), cur) != vertices.end()) {
      LOG4CPLUS_TRACE(logger, "Illegal cycle detected in trace_untiled_region: " << pp(cur));
      throw logic_error("Illegal cycle detected in trace_untiled_region");
    }
  }

  vertices.push_back(cur);

  // All chords whose source is the current vertex cur
  std::vector<Boundary_slice_chord> chords = w.bscs.on_vertex(cur);

  Point_3 next;
  if (chords.empty()) {
    LOG4CPLUS_TRACE(logger, "contour segment");
    next = w.vertices.adjacent(cur, dir, w.hierarchies);
    Segment_3 segment;
    if (dir == Walk_direction::FORWARD) {
      segment = Segment_3(cur, next);
    }
    else {
      segment = Segment_3(next, cur);
    }
    vertices = trace_untiled_region(next, vertices, dir, w);
    w.completion_map.put(segment);
    return vertices;
  }
  else {
    LOG4CPLUS_TRACE(logger, "chord");
    for (BSC_iter it = chords.begin(); it != chords.end(); ++it) {
      next = it->segment().target();
      try {
	vertices = trace_untiled_region(next, vertices, opposite(dir), w);
	w.bscs.erase(*it);
	return vertices;
      }
      catch (logic_error& e) {
	// Swallow and try the next one
      }
    }
    LOG4CPLUS_TRACE(logger, "No chord was legal");
    throw logic_error("No chord was legal");
  }
}

/// Rules:
///  If coming from a boundary chord, first try to stay on the same polygon.  If
///    the adjacent vertex has already been visited, then take the next available
///    boundary chord.
///  If coming from an adjacent vertex, take a boundary chord if there is one available.
///    If not, stay on the polygon.
///  To choose between available boundary chords, choose the one whose destination
///    point is on the same oriented side as the last point that appeared in that slice.
list<Untiled_region> trace_untiled_regions(Tiler_workspace& w)
{
  static Logger logger = Logger::getInstance("tiler.trace_untiled_regions");

  typedef std::vector<Boundary_slice_chord>::const_iterator BSC_iter;

  list<Untiled_region> polygons;

  // Trace untiled regions bounded by boundary slice chords
  while (w.bscs.size() > 0)
  {
    LOG4CPLUS_TRACE(logger, "Beginning untiled trace");

    Boundary_slice_chord first_chord = w.bscs.first();

    w.bscs.erase(first_chord);

    Walk_direction dir = first_chord.direction();
    Point_3 first = first_chord.segment().source();
    Point_3 cur = first_chord.segment().target();
    Number_type midz = (first.z() + cur.z()) / 2.0;
    Untiled_region vertices;
    // vertices.push_back(first);
    LOG4CPLUS_TRACE(logger, "cur = " << pp(first));
    LOG4CPLUS_TRACE(logger, "  dir = " << dir);

    vertices.push_back(first);
    try {
      vertices = trace_untiled_region(cur, vertices, dir, w);
    }
    catch (logic_error& e) {
      LOG4CPLUS_ERROR(logger, "Failed to trace untiled region: " << e.what());
      return polygons;
    }

    LOG4CPLUS_TRACE(logger, "Ending untiled trace");
    polygons.push_back(vertices);
  }

  // Any remaining regions should be in contours that are completely untiled
  for (std::vector<Contour_handle>::const_iterator it = w.bottom.begin();
       it != w.bottom.end();
       ++it)
  {
    const Polygon_2& polygon = (*it)->polygon();
    if (!w.completion_map.is_complete(polygon[0]))
    {
      Untiled_region poly;
      poly.insert(polygon.vertices_begin(), polygon.vertices_end());
      polygons.push_back(poly);
    }
  }
  for (std::vector<Contour_handle>::const_iterator it = w.top.begin();
       it != w.top.end();
       ++it)
  {
    const Polygon_2& polygon = (*it)->polygon();
    if (!w.completion_map.is_complete(polygon[0]))
    {
      Untiled_region poly;
      poly.insert(polygon.vertices_begin(), polygon.vertices_end());
      poly.reverse_orientation();
      polygons.push_back(poly);
    }
  }

  return polygons;
}

void build_tiling_table(Tiler_workspace& w)
{
  build_tiling_table_phase1(w);
  build_tiling_table_phase2(w);
}

void build_tiling_table_phase1(Tiler_workspace& w)
{
  Logger logger = Logger::getInstance("tiler.build_tiling_table_phase1");

  LOG4CPLUS_TRACE(logger, "Pass 1 (cases 1-3)...");

  // pass 1
  std::list<OTV_pair> otv_pairs;
  find_all_OTV_pairs(w, back_inserter(otv_pairs));
  for (std::list<OTV_pair>::iterator it = otv_pairs.begin(); it != otv_pairs.end(); ++it)
  {
    // u = top
    // v = bottom
    Point_3 u2 = it->top();
    Point_3 v2 = it->bottom();
    LOG4CPLUS_TRACE(logger, "OTV pair " << u2.id() << " " << v2.id() << " FORWARD");
    case1_3(u2, v2, Walk_direction::FORWARD, w);
    LOG4CPLUS_TRACE(logger, "OTV pair " << u2.id() << " " << v2.id() << " BACKWARD");
    case1_3(u2, v2, Walk_direction::BACKWARD, w);
  }

  LOG4CPLUS_TRACE(logger, "Pass 2 (case 4)...");
  pass2_4(4, w);
  LOG4CPLUS_TRACE(logger, "Pass 3 (case 5)...");
  pass2_4(5, w);
  LOG4CPLUS_TRACE(logger, "Pass 4 (case 6)...");
  pass2_4(6, w);

}

void build_tiling_table_phase2(Tiler_workspace& w)
{
  Logger logger = Logger::getInstance("tiler.build_tiling_table_phase2");

  // trace out new contours
  LOG4CPLUS_TRACE(logger, "Tracing untiled contours...");
  list<Untiled_region> contours = trace_untiled_regions(w);

  LOG4CPLUS_TRACE(logger, "Tiling untiled regions...");
  int idx = 0;
  for (list<Untiled_region>::iterator it = contours.begin(); it != contours.end(); ++it)
  {
    const Untiled_region& region = *it;
    w._callback->untiled_region(region);

    // if (region.size() < 3) {
    //   LOG4CPLUS_WARN(logger, "Untiled region of size " << region.size() << " -- skipping");
    // }
    // else if (region.size() == 3) {
    if (region.size() == 3) {
      Untiled_region::const_iterator i1 = region.begin();
      Untiled_region::const_iterator i2 = i1; i2++;
      Untiled_region::const_iterator i3 = i2; i3++;
      w.add_tile(Tile_handle(new Tile(Triangle(*i1, *i2, *i3))));
    }
    else {
      list<Tile> new_tiles;
      medial_axis_stable(region, w.midz, back_inserter(new_tiles), w.vertices, w);
    
      for (list<Tile>::iterator i = new_tiles.begin(); i != new_tiles.end(); ++i) {
	w.add_tile(Tile_handle(new Tile(*i)));
      }
    }

    ++idx;
  }
}

CONTOURTILER_END_NAMESPACE
