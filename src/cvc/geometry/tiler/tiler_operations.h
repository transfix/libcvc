#ifndef __TILER_OPERATIONS_H__
#define __TILER_OPERATIONS_H__

#include <list>
#include <map>
#include <iostream>
#include <ContourTiler/contour_utils.h>
#include <ContourTiler/Hierarchy.h>
#include <ContourTiler/Tiling_region.h>
#include <ContourTiler/Vertex_map.h>
#include <ContourTiler/Contour.h>
#include <boost/unordered_map.hpp>
#include <ContourTiler/Correspondences.h>

#include <CGAL/intersections.h>
#include <boost/shared_ptr.hpp>

#include <ContourTiler/Tiles.h>
#include <ContourTiler/Vertex_completion_map.h>
#include <ContourTiler/tiler_defines.h>
#include <ContourTiler/Tiler_workspace.h>

CONTOURTILER_BEGIN_NAMESPACE

/// Augments the given polygons in place
void augment(Polygon_2& P, Polygon_2& Q);

/// Outputs augmented contours
/// ContourIterator - iterator over contours
template <typename ContourIterator, typename OutputIterator>
bool augment(ContourIterator bottom_start, ContourIterator bottom_end, ContourIterator top_start, ContourIterator top_end, 
	     OutputIterator bottom_aug, OutputIterator top_aug)
{
  using namespace std;

  typedef list<Contour_handle> Container;
  typedef Container::iterator iterator;

  bool ret = false;

  list<Contour_handle> top, bottom;
  for (ContourIterator it = top_start; it != top_end; ++it)
  {
    Contour_handle copy = (*it)->copy();
    top.push_back(copy);
    *top_aug = copy;
    ++top_aug;
  }
  for (ContourIterator it = bottom_start; it != bottom_end; ++it)
  {
    Contour_handle copy = (*it)->copy();
    bottom.push_back(copy);
    *bottom_aug = copy;
    ++bottom_aug;
  }

  for (iterator top_it = top.begin(); top_it != top.end(); ++top_it) {
    for (iterator bottom_it = bottom.begin(); bottom_it != bottom.end(); ++bottom_it) {
      Polygon_2& P = (*bottom_it)->polygon();
      Polygon_2& Q = (*top_it)->polygon();
      Polygon_2 oldp = P;
      Polygon_2 oldq = Q;
      augment(P, Q);
      ret = ret || (oldp.size() < P.size() || oldq.size() < Q.size());
    }
  }

  return ret;
}

/// Returns true if a tile can exist between two contours 
/// according to the requirements given in Bajaj96, theorem 6.
// template <typename Contour_handle>
bool can_tile(Contour_handle c1, Contour_handle c2, const Hierarchy& h1, const Hierarchy& h2, Number_type overlap);

/// Finds all contours that can be tiled to the given contour
/// Yes, I know the return value looks scary, but it is simply a map of contour handles to a 
/// list of contour handles:
template <typename ContourIterator>
Correspondences find_correspondences(ContourIterator start1, ContourIterator end1, const Hierarchy& h1, 
		     ContourIterator start2, ContourIterator end2, const Hierarchy& h2, Number_type overlap)
{
  static log4cplus::Logger logger = log4cplus::Logger::getInstance("tiler.find_correspondences");

  Correspondences correspondences;
  size_t count = 0;
  for (ContourIterator it1 = start1; it1 != end1; ++it1)
    for (ContourIterator it2 = start2; it2 != end2; ++it2)
      if (can_tile(*it1, *it2, h1, h2, overlap)) {
	correspondences.add(*it1, *it2);
	++count;
      }
  LOG4CPLUS_TRACE(logger, "Number of correspondences: " << count);
  return correspondences;
}

// template <typename Vertex_circulator, typename Hierarchy>
// Tiling_region tiling_region(Polygon_2::Vertex_circulator vertex, const Hierarchy& h);

/// ContourIterator iterates over contours
template <typename ContourIterator>
Vertex_map<HTiling_region> find_tiling_regions(ContourIterator start1, ContourIterator end1, const Hierarchy& h1, 
					       ContourIterator start2, ContourIterator end2, const Hierarchy& h2,
					       Number_type z1, Number_type z2,
					       const Vertices& vertices);
// {
//   typedef Polygon_2::Vertex_circulator Vertex_circulator;

//   Vertex_map<Tiling_region> regions(1000);

//   for (ContourIterator it1 = start1; it1 != end1; ++it1)
//   {
//     Contour_handle contour = *it1;
//     Vertex_circulator beg = contour->polygon().vertices_circulator();
//     Vertex_circulator it = beg;
//     do
//     {
//       Point_3 v = vertices.get(contour, it);
//       regions[v] = tiling_region(it, h2);
//       ++it;
//     } while (it != beg);
//   }

//   for (ContourIterator it2 = start2; it2 != end2; ++it2)
//   {
//     Contour_handle contour = *it2;
//     Vertex_circulator beg = contour->polygon().vertices_circulator();
//     Vertex_circulator it = beg;
//     do
//     {
//       Point_3 v = vertices.get(contour, it);
//       regions[v] = tiling_region(it, h1);
//       ++it;
//     } while (it != beg);
//   }

//   return regions;
// }

// Used to compare distances from 2 points to another point
class Dist_cmp
{
public:
  Dist_cmp(const Point_2& p) : _point(p) {}
//   Dist_cmp(const Point_3& v) : _point(v.point()) {}

  bool operator()(const Point_2& a, const Point_2& b)
  {
    return CGAL::has_smaller_distance_to_point(_point, a, b);
  }

//   bool operator()(const Point_3& a, const Point_3& b)
//   {
//     return CGAL::has_smaller_distance_to_point(_point, a.point(), b.point());
//   }

private:
  Point_2 _point;
};

/// ContourIterator iterates over contours
template <typename ContourIterator>
OTV_table build_OTV_table(ContourIterator contours_begin, ContourIterator contours_end,
			  ContourIterator c1_begin, ContourIterator c1_end,
			  ContourIterator c2_begin, ContourIterator c2_end,
			  const Vertices& vertices,
			  const Correspondences& correspondences,
			  const Vertex_map<HTiling_region>& tiling_regions,
			  const Hierarchies& h,
			  Banned& banned);


// {
//   typedef Correspondences::const_iterator corr_iterator;
//   typedef Polygon_2::Vertex_iterator Vertex_iterator;

//   using namespace std;

//   OTV_table otv_table;

//   for (ContourIterator it = contours_begin; it != contours_end; ++it)
//   {
//     Contour_handle contour = *it;

//     // get vertices of all contours that corresponding to the current contour
//     vector<Point_3> points;
//     for (corr_iterator cit = correspondences.begin(contour); cit != correspondences.end(contour); ++cit)
//       vertices.get_vertices(*cit, back_inserter(points));

//     // loop through each vertex v on current contour; sort all corresponding vertices
//     // according to distance from v.
//     for (Vertex_iterator vit = contour->polygon().vertices_begin(); vit != contour->polygon().vertices_end(); ++vit)
//     {
//       Point_3 vertex = vertices.get(contour, vit);
//       sort(points.begin(), points.end(), Dist_cmp(vertex));

//       // loop through sorted vertices until we find a valid point
//       bool found = false;
//       for (typename vector<Point_3>::const_iterator pit = points.begin(); !found && pit != points.end(); ++pit)
//       {
// 	const Point_3& test_vertex = *pit;
// 	Point_2 test_point = test_vertex;//test_vertex.point();
// 	found = (test_point == vertex.point());
// 	if (!found)
// 	{
// 	  Segment_2 segment(vertex.point(), test_point);
// 	  found = (tiling_regions[vertex].contains(test_point) &&
// 		   !intersects_proper(segment, c1_begin, c1_end) && 
// 		   !intersects_proper(segment, c2_begin, c2_end));
// 	}
// 	if (found)
// 	  otv_table[vertex] =test_vertex;
//       }
//     }
//   }

//   return otv_table;
// }

bool is_OTV_pair(const Point_3& v1, const Point_3& v2, const OTV_table& otv_table);

/// See discussion of 6 cases of optimality in bajaj96.
int optimality(const Point_3& u2, const Point_3& u3, const Point_3& v2, const Point_3& v3, const Tiler_workspace& w);

void find_banned(Tiler_workspace& w);

template <typename ContourIterator>
boost::shared_ptr<Tiler_workspace> pre_tile(ContourIterator bottom_start, ContourIterator bottom_end, 
					    ContourIterator top_start, ContourIterator top_end, 
					    bool do_augment, Number_type overlap)
{
  typedef std::vector<Contour_handle> Contours;

  Contours bottom, top, all;
  Hierarchy bottom_h, top_h;
  Hierarchies hierarchies;
  Correspondences correspondences;
  Vertices vertices;
  Vertex_map<HTiling_region> tiling_regions;
  OTV_table otv_table;
  
  Number_type bottomz;
  if (bottom_start != bottom_end) {
    bottomz = (*bottom_start)->polygon()[0].z();
  }
  else {
    bottomz = (*top_start)->polygon()[0].z() - 1;
  }
  Number_type topz;
  if (top_start != top_end) {
    topz = (*top_start)->polygon()[0].z();
  }
  else {
    topz = bottomz + 1;
  }

//   Number_type bottomz = (*bottom_start)->polygon()[0].z();
//   Number_type topz = (*top_start)->polygon()[0].z();
  Number_type midz = (topz + bottomz) / 2.0;

  static log4cplus::Logger logger = log4cplus::Logger::getInstance("tiler.pre_tile");

  if (do_augment)
  {
    LOG4CPLUS_TRACE(logger, "Augmenting contours...");
    augment(bottom_start, bottom_end, top_start, top_end, back_inserter(bottom), back_inserter(top));
  }
  else
  {
    bottom.insert(bottom.end(), bottom_start, bottom_end);
    top.insert(top.end(), top_start, top_end);
  }

  all.insert(all.end(), bottom.begin(), bottom.end());
  all.insert(all.end(), top.begin(), top.end());

  LOG4CPLUS_TRACE(logger, "Creating hierarchies...");
  LOG4CPLUS_TRACE(logger, "Creating bottom augmented hierarchy...");
  bottom_h = Hierarchy(bottom.begin(), bottom.end(), Hierarchy_policy::FORCE_CCW);
  LOG4CPLUS_TRACE(logger, "Creating top augmented hierarchy...");
  top_h = Hierarchy(top.begin(), top.end(), Hierarchy_policy::FORCE_CCW);

//   hierarchies[bottom[0]->slice()] = bottom_h;
//   hierarchies[top[0]->slice()] = top_h;
  hierarchies[bottomz] = bottom_h;
  hierarchies[topz] = top_h;

  LOG4CPLUS_TRACE(logger, "Creating vertices...");
  vertices = Vertices::create(all.begin(), all.end());

  // Get correspondences
  LOG4CPLUS_TRACE(logger, "Finding correspondences...");
  correspondences = find_correspondences(bottom.begin(), bottom.end(), bottom_h, top.begin(), top.end(), top_h, overlap);

  // Get the tiling regions
  LOG4CPLUS_TRACE(logger, "Finding tiling regions...");
  tiling_regions = find_tiling_regions(bottom.begin(), bottom.end(), bottom_h, 
				       top.begin(), top.end(), top_h, 
				       bottomz, topz,
				       vertices);
    
  Banned banned;

  // Build the OTV table
  LOG4CPLUS_TRACE(logger, "Building OTV table...");
  otv_table = build_OTV_table(all.begin(), all.end(), bottom.begin(), bottom.end(), top.begin(), top.end(), vertices, correspondences, tiling_regions, hierarchies, banned);
    
  TW_handle w = Tiler_workspace::create(all, bottom, top, correspondences, vertices, tiling_regions, otv_table, hierarchies, midz);

  // Finding banned vertices
  LOG4CPLUS_TRACE(logger, "Finding banned vertices...");
  w->set_banned(banned);
//   find_banned(*w);
    
  return w;
//   return boost::shared_ptr<Tiler_workspace>(w);
  
}

void build_tiling_table(Tiler_workspace& workspace);

void build_tiling_table_phase1(Tiler_workspace& workspace);

void build_tiling_table_phase2(Tiler_workspace& workspace);

template <typename ContourIterator>
Tiles build_tiling_table(ContourIterator s1_begin, ContourIterator s1_end,
			 ContourIterator s2_begin, ContourIterator s2_end,
			 const Correspondences& correspondences,
			 const Vertex_map<Tiling_region>& tiling_regions,
			 const OTV_table& otv_table, const Hierarchies& hierarchies)
{
  Tiles tiles;
  Tiler_workspace workspace(correspondences, tiling_regions, otv_table, hierarchies, tiles);
  build_tiling_table(workspace);
//   for (ContourIterator cit = s1_begin; cit != s1_end; ++cit)
//   {
//     build_tiling_table(*cit, correspondences, tiling_regions, otv_table, hierarchies, tiles);
//   }
  return tiles;
}

CONTOURTILER_END_NAMESPACE

#endif
