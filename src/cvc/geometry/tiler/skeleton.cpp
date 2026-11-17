#include <limits>

#include <boost/foreach.hpp>

#include <ContourTiler/common.h>
#include <ContourTiler/print_utils.h>
#include <float.h>
#include <ContourTiler/theorems.h>
#include <ContourTiler/sweep_line_visitors.h>
#include <ContourTiler/triangle_utils.h>
#include <ContourTiler/xy_pred.h>
#include <ContourTiler/Vertices.h>
#include <ContourTiler/interp.h>

#include <time.h>
//-------------------------------------------------
// <CGAL>
#include <boost/shared_ptr.hpp>
#include <boost/mpl/equal_to.hpp>

#include <CGAL/basic.h>
#include <CGAL/Cartesian.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Straight_skeleton_builder_2.h>
#include <CGAL/Polygon_offset_builder_2.h>
#include <CGAL/compute_outer_frame_margin.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Triangulation_vertex_base_with_info_2.h>
#include <CGAL/centroid.h>
#include <CGAL/partition_2.h>
#include <CGAL/Bbox_2.h>
#include <CGAL/utility.h>
#include <CGAL/natural_neighbor_coordinates_2.h>
#include <CGAL/Delaunay_triangulation_2.h>

typedef CGAL::Straight_skeleton_2<Kernel> Ss;

typedef Ss::Face_iterator Face_iterator;
typedef Ss::Face_handle Face_handle;
typedef Ss::Halfedge_iterator Halfedge_iterator;
typedef Ss::Halfedge_handle   Halfedge_handle;
typedef Ss::Vertex_handle     Vertex_handle;

typedef CGAL::Straight_skeleton_builder_traits_2<Kernel>      SsBuilderTraits;
typedef CGAL::Straight_skeleton_builder_2<SsBuilderTraits,Ss> SsBuilder;

typedef CGAL::Polygon_offset_builder_traits_2<Kernel>                  OffsetBuilderTraits;
typedef CGAL::Polygon_offset_builder_2<Ss,OffsetBuilderTraits,Polygon_2> OffsetBuilder;

typedef CGAL::Triangulation_vertex_base_2<Kernel>                     Vb;
typedef CGAL::Triangulation_vertex_base_with_info_2<bool, Kernel, Vb>     Info;
typedef CGAL::Constrained_triangulation_face_base_2<Kernel>           Fb;
typedef CGAL::Triangulation_data_structure_2<Info,Fb>              TDS;
typedef CGAL::Exact_predicates_tag                               Itag;
typedef CGAL::Constrained_Delaunay_triangulation_2<Kernel, TDS, Itag> CDT;

// </CGAL>
//-------------------------------------------------

typedef CGAL::Partition_traits_2<Kernel> PT;
typedef std::set<Point_2, bool(*)(const Point_2&, const Point_2&)> Overlapping;

extern string out_dir;

CONTOURTILER_BEGIN_NAMESPACE

void attempt_insert(const Point_3& p0, const Point_3& p1, const Point_3& p2, Tiler_workspace& w)
{
  if (test_all(Segment_3(p0, p1), w) && test_all(Segment_3(p0, p2), w) && test_all(Segment_3(p1, p2), w))
    w.add_tile(p0, p1, p2);
}

template <typename Polygon_t>
void triangulate(const Polygon_t& polygon, const Overlapping& doubles, Number_type midz, Tiler_workspace& w, bool reverse)
{
  static log4cplus::Logger logger_skel = log4cplus::Logger::getInstance("tiler.skeleton");

  CDT cdt;
  cdt.insert(polygon.vertices_begin(), polygon.vertices_end());

  // Loop through the triangulation and store the vertices of each triangle
  for (CDT::Finite_faces_iterator ffi = cdt.finite_faces_begin();
       ffi != cdt.finite_faces_end();
       ++ffi)
  {
    Point_2 p[3];
    for (int i = 0; i < 3; ++i)
    {
      p[i] = ffi->vertex(i)->point();
      LOG4CPLUS_TRACE(logger_skel, "Triangulation from skeleton: " << pp(p[i]));
    }

    if (reverse)
      attempt_insert(p[2].point_3(), p[1].point_3(), p[0].point_3(), w);
    else
      attempt_insert(p[0].point_3(), p[1].point_3(), p[2].point_3(), w);
  }
}

// void ensure_same_contour(Polygon_2& sub_polygon, double midz, const Overlapping& doubles, Tiler_workspace& w)
// {
// }

void check_z(Polygon_2& sub_polygon, Number_type midz, const Overlapping& doubles, Tiler_workspace& w)
{
  // Find the two points on contours.  We are guaranteed that there will be only
  // two as these are generated from skeletonization of a polygon.
  //                 _____________
  //                /p1          m1
  //               /
  // _____________/
  // m0          p0
  Point_2 point[2];
  Polygon_2::Vertex_circulator pnt[2];
  Polygon_2::Vertex_circulator midpnt[2];
  Polygon_2::Vertex_circulator first = sub_polygon.vertices_circulator();
  Polygon_2::Vertex_circulator it = first;
  // Find the two points and the points next to them
  do
  {
    if (it->z() != midz)
    {
      pnt[0] = it;
      if ((it+1)->z() != midz)
      {
	pnt[1] = it + 1;
	midpnt[0] = it - 1;
	midpnt[1] = it + 2;
      }
      else
      {
	pnt[1] = it - 1;
	midpnt[0] = it + 1;
	midpnt[1] = it - 2;
      }
      break;
    }
    ++it;
  } while (it != first);

  point[0] = *pnt[0];
  point[1] = *pnt[1];

  // Let's do some sanity checks
  {
    Polygon_2::Vertex_circulator cfirst = sub_polygon.vertices_circulator();
    Polygon_2::Vertex_circulator cit = first;
    // Find the two points and the points next to them
    do
    {
      if (*it != *pnt[0] && *it != *pnt[1])
      {
	if (it->z() != midz)
	  throw std::logic_error("Unexpected point in slice");
      }
    } while (cit != cfirst);
    if (midpnt[0]->z() != midz || midpnt[1]->z() != midz)
      throw std::logic_error("Unexpected point in slice");
    if (pnt[0]->z() == midz || pnt[1]->z() == midz)
      throw std::logic_error("Unexpected point in mid slice");
  } // end sanity checks

  Overlapping::const_iterator dbl[2];
  dbl[0] = doubles.find(*(pnt[0]));
  dbl[1] = doubles.find(*(pnt[1]));

  // both doubles
  if (dbl[0] != doubles.end() && dbl[1] != doubles.end())
  {
    // Use the z value for which the two points are adjacent on the contour
//     if (w.vertices->contour(*pnt[0]) == w.vertices->contour(*dbl[1]))
    if (w.vertices.adjacent(*pnt[0], *dbl[1]))
      sub_polygon.set(pnt[1], *dbl[1]);
//     else if (w.vertices->contour(*dbl[0]) == w.vertices->contour(*pnt[1]))
    else if (w.vertices.adjacent(*dbl[0], *pnt[1]))
      sub_polygon.set(pnt[0], *dbl[0]);
//     else if (w.vertices->contour(*dbl[0]) == w.vertices->contour(*dbl[1]))
    else if (w.vertices.adjacent(*dbl[0], *dbl[1]))
    {
      sub_polygon.set(pnt[0], *dbl[0]);
      sub_polygon.set(pnt[1], *dbl[1]);
    }
  }
  else
  {
    for (int i = 0; i < 2; ++i)
    {
      if (dbl[i] != doubles.end())
      {
	if (pnt[i]->z() != pnt[1-i]->z())
	  sub_polygon.set(pnt[i], *(dbl[i]));
      }
    }
  }

  for (int i = 0; i < 2; ++i)
  {
    if (dbl[i] != doubles.end())
    {
//        attempt_insert(*pnt[i], *dbl[i], *midpnt[i], w);
      attempt_insert(point[i], *dbl[i], *midpnt[i], w);
    }
  }
}

template <typename Polygon_iterator>
void get_sub_polygons(boost::shared_ptr<Ss> ss, Number_type midz, Polygon_iterator polygons, const Overlapping& doubles, Tiler_workspace& w)
{
  for (Face_iterator fi = ss->faces_begin(); fi != ss->faces_end(); ++fi)
  {
    Polygon_2 polygon;
    Halfedge_handle halfedge = fi->halfedge();
    Halfedge_handle first = halfedge;
    do
    {
      Vertex_handle v = halfedge->vertex();
      Point_2 point(v->point());
      if (!v->is_contour())
	point.z() = midz;
      polygon.push_back(point);

      halfedge = halfedge->next();
    } while (halfedge != first);
    check_z(polygon, midz, doubles, w);
    *polygons = polygon;
    ++polygons;
  }
}

std::list<PT::Polygon_2> convex_partition(const Polygon_2& polygon)
{
  std::list<PT::Polygon_2> output_polys;
  CGAL::approx_convex_partition_2(polygon.vertices_begin(), polygon.vertices_end(), 
				  std::back_inserter(output_polys));
  return output_polys;
}

int compare_2d(const Point_2& a, const Point_2& b)
{
  // Can't just return a.x()-b.x() because the value is rounded
  // to an int, and if a.x()-b.x() == -0.5 then 0 will be returned.
  if (a.x() < b.x())
    return -1;
  if (a.x() > b.x())
    return 1;
  if (a.y() < b.y())
    return -1;
  if (a.y() > b.y())
    return 1;
  return 0;
}

bool less_than_2d(const Point_2& a, const Point_2& b)
{
  return compare_2d(a, b) < 0;
}

bool equals_2d(const Point_2& a, const Point_2& b)
{
  return compare_2d(a, b) == 0;
}

// void plot_polygon(const Polygon_2& polygon)
// {
//   static int count = 0;
//   std::stringstream ss_out;
//   ss_out << out_dir << "/out_skel_" << count << ".g";
//   ++count;
//   ofstream out(ss_out.str().c_str());
//   gnuplot_print_polygon(out, polygon);
//   out.close();
// }

/// Steps:
///  1. Convert contour to a polygon_2
///   1a. Build a map of all co-incident 2D vertices (vertices with same x,y but different z) -
///       only include one of them in the polygon_2
///  2. Skeletonize the polygon_2
///  3. For each sub-polygon_2 P generated during skeletonization
///   3a. For each convex sub-sub-polygon_2 Q in P
///    3ai. Triangulate Q
void triangulate_to_medial_axis(const Untiled_region& contour, Vertices& vertices, Tiler_workspace& w)
{
  static log4cplus::Logger logger_skel = log4cplus::Logger::getInstance("tiler.skeleton");
  using namespace std;

  // Step 1.  Convert contour to a polygon
  Polygon_2 polygon;
  Number_type maxz = -1;
  Number_type minz = DBL_MAX;
  Overlapping doubles(less_than_2d);
  for (Untiled_region::const_iterator it = contour.begin(); it != contour.end(); ++it)
  {
    Point_2 p = it->point_2();
    if (polygon.size() > 0 && equals_2d(polygon[polygon.size()-1], p))
    {
      LOG4CPLUS_TRACE(logger_skel, "Adding to doubles: " << pp(p));
      doubles.insert(p); // Step 1a
    }
    else
      polygon.push_back(p);
//     LOG4CPLUS_TRACE(logger_skel, "Adding to polygon to be triangulated: " << pp(p));
    maxz = (maxz > p.z()) ? maxz : p.z();
    minz = (minz < p.z()) ? minz : p.z();
  }
  LOG4CPLUS_TRACE(logger_skel, "Polygon to be triangulated: " << pp(polygon));

//   Number_type midz = (minz + maxz) / 2.0;
  Number_type midz = w.midz;

  // Contour may possibly have bad orientation
  bool reverse = false;
  if (polygon.is_clockwise_oriented())
  {
    reverse = true;
    polygon.reverse_orientation();
  }

  // Step 2. Skeletonize polygon and get all sub-polygons
  SsBuilder ssb;
  ssb.enter_contour(polygon.vertices_begin(), polygon.vertices_end());
  boost::shared_ptr<Ss> ss = ssb.construct_skeleton();
  std::list<Polygon_2> sub_polygons;
  if (ss)
    get_sub_polygons(ss, midz, back_inserter(sub_polygons), doubles, w);
  else
  {
    if (!polygon.is_convex() || !polygon.is_simple())
    {
      LOG4CPLUS_TRACE(logger_skel, "Issues getting skeleton");
      std::list<PT::Polygon_2> test;
      CGAL::approx_convex_partition_2(polygon.vertices_begin(), polygon.vertices_end(), 
				      std::back_inserter(test));
      for (std::list<PT::Polygon_2>::iterator it = test.begin(); it != test.end(); ++it)
      {
	Polygon_2 t(it->vertices_begin(), it->vertices_end());
	LOG4CPLUS_TRACE(logger_skel, pp(t));
      }
      throw std::runtime_error("Failed to skeletonize and unable to recover: " + pp(polygon));
    }
    sub_polygons.push_back(polygon);
  }

  // Print out sub-polygons
//   std::for_each(sub_polygons.begin(), sub_polygons.end(), plot_polygon);

  // Step 3
  for (std::list<Polygon_2>::iterator it = sub_polygons.begin();
       it != sub_polygons.end();
       ++it)
  {

    // TODO: We may be triangulating non-convex polygons which may account for some of our
    // normal problems.

    // Step 3a
//     std::list<PT::Polygon_2> sub_sub_polygons;
//     CGAL::approx_convex_partition_2(it->vertices_begin(), it->vertices_end(), 
// 				    std::back_inserter(sub_sub_polygons));
//     a += (clock() - t);

//     t = clock();
//     for (std::list<PT::Polygon_2>::iterator ssp_it = sub_sub_polygons.begin();
// 	 ssp_it != sub_sub_polygons.end();
// 	 ++ssp_it)
    for (std::list<Polygon_2>::iterator ssp_it = sub_polygons.begin();
	 ssp_it != sub_polygons.end();
	 ++ssp_it)
    {
      // Step 3ai
      triangulate(*ssp_it, doubles, midz, w, reverse);
    }
  }

}

Point_3 centroid(const Polygon_2& P, Number_type z)
{
  Number_type A = P.area();
  Number_type xsum = 0, ysum = 0;
  Polygon_2::Vertex_circulator start = P.vertices_circulator();
  Polygon_2::Vertex_circulator ci = start;
  Polygon_2::Vertex_circulator ci_next = start;
  ++ci_next;
  do
  {
    const Point_2& cur = *ci;
    const Point_2& next = *ci_next;
    xsum += (cur.x() + next.x()) * (cur.x() * next.y() - next.x() * cur.y());
    ysum += (cur.y() + next.y()) * (cur.x() * next.y() - next.x() * cur.y());
    ++ci;
    ++ci_next;
  } while (ci != start);

  Number_type x = xsum / (6 * A);
  Number_type y = ysum / (6 * A);

//   typedef CGAL::Delaunay_triangulation_2<Kernel> DT;
//   typedef std::vector<std::pair<Point_2, Number_type> > Point_coordinate_vector;

//   DT cdt;
//   cdt.insert(P.vertices_begin(), P.vertices_end());

//   Point_coordinate_vector coords;
//   CGAL::Triple<
//     std::back_insert_iterator<Point_coordinate_vector>, 
//     Number_type, bool> result = 
//     CGAL::natural_neighbor_coordinates_2(cdt, Point_2(x, y),
// 					 std::back_inserter(coords));
//   if(!result.third){
//     std::cout << "The coordinate computation was not successful." 
// 	      << std::endl;
//     std::cout << "The point (" << pp(Point_2(x,y)) << ") lies outside the convex hull."
// 	      << std::endl;
//   }
//   Number_type norm = result.second;

//   Triangle tri(coords[0].first, coords[1].first, coords[2].first);
//   z = get_z(tri, Point_2(x, y));

  return Point_3(x, y, z);
}

Untiled_region::const_iterator next(Untiled_region::const_iterator it, const Untiled_region& region)
{
  ++it;
  if (it == region.end())
    it = region.begin();
  return it;
}

Untiled_region::const_iterator prev(Untiled_region::const_iterator it, const Untiled_region& region)
{
  if (it == region.begin())
    it = region.end();
  --it;
  return it;
}

Polygon_2 to_polygon(const Untiled_region& region, map<Point_2, Point_2>& old2new)
{
  static log4cplus::Logger logger = log4cplus::Logger::getInstance("tiler.to_polygon");

  Polygon_2 polygon;
  for (Untiled_region::const_iterator it = region.begin(); it != region.end(); ++it) {
    Point_2 p = *it;
    Point_2 n = *(next(it, region));
    if (!xy_equal(p, n)) {
      LOG4CPLUS_TRACE(logger, "Adding to polygon: " << pp(p));
      polygon.push_back(p);
    }
  }
  return polygon;
}

//------------------------------------------------------------------------------
// point_intersection_2
//
/// Returns true if segment a intersects with line b at a point in two
/// dimensions.  The intersection must occur on the projection onto the z plane.
/// Returns false if the two segments either don't intersect or if they 
/// intersect on a segment.
//------------------------------------------------------------------------------
// THIS FUNCTION IS NOT GENERAL!
bool mid_int(const Segment_2& a, const Segment_2& b, Point_2& ia)
{
  Number_type numa = 
    (b.target().x() - b.source().x()) * (a.source().y() - b.source().y()) -
    (b.target().y() - b.source().y()) * (a.source().x() - b.source().x());
  Number_type numb = 
    (a.target().x() - a.source().x()) * (a.source().y() - b.source().y()) -
    (a.target().y() - a.source().y()) * (a.source().x() - b.source().x());
  Number_type den = 
    (b.target().y() - b.source().y()) * (a.target().x() - a.source().x()) -
    (b.target().x() - b.source().x()) * (a.target().y() - a.source().y());

  if (numa == 0 && numb == 0 && den == 0)
    // Intersection is a segment
    return false;

  if (den == 0)
    return false;
  Number_type ua = numa / den;
  Number_type ub = numb / den;
  // SEE?  NOT GENERAL!
//   bool intersects = ua > 0 && ua < 1;
  bool intersects = ua > 0.1 && ua < 0.9;

  if (intersects)
  {
    ia = Point_2(a.source().x() + ua * (a.target().x() - a.source().x()),
		 a.source().y() + ua * (a.target().y() - a.source().y()));
  }

  return intersects;
}

Point_3 midpoint(const Segment_2_undirected& s, const Segment_2& cross, Number_type z)
{
  log4cplus::Logger logger = log4cplus::Logger::getInstance("tiler.medial_axis_stable");
//   LOG4CPLUS_TRACE(logger, "Midpoint: " << pp(s.lesser()) << " " << pp(s.greater()));
  Point_2 ia;
  if (mid_int(s.segment(), cross, ia))
    return Point_3(ia.x(), ia.y(), z);
  return Point_3((s.lesser().x() + s.greater().x()) / 2.0,
		 (s.lesser().y() + s.greater().y()) / 2.0,
		 z);
}

// typedef boost::unordered_map<Point_2, list<Segment_2_undirected> > Vertex2extras;
typedef boost::unordered_map<Point_2, list<Point_2> > Vertex2extras;

bool valid_as_start(Untiled_region::const_iterator it, const Untiled_region& region,
		    const Vertex2extras& vertex2extras, map<Point_2, Point_2>& old2new)
{
  Point_2 p = *it;
  Point_2 n = *(next(it, region));
  Segment_2 edge(p, n);
  const Point_2& source = old2new[edge.source()];
  const Point_2& target = old2new[edge.target()];

  return (!xy_equal(source, target) &&
	  vertex2extras.find(source) == vertex2extras.end());
}

// Segment_2 as_source(const Segment_2_undirected& segment, const Point_2& p)
// {
//   Segment_2 s = segment.segment();
//   if (p == s.source())
//     return s;
//   if (p != s.target())
//     throw logic_error("p not equal to either endpoint of segment");
//   return s.opposite();
// }

class Angle_functor
{
public:
  Angle_functor(const Segment_2& segment) : _segment(segment) {}
  ~Angle_functor() {}

//   bool operator()(const Segment_2_undirected& a, const Segment_2_undirected& b)
//   {
//     typedef CGAL::Direction_2<Kernel> Direction_2;
//     Direction_2 pd(as_source(a, _segment.source()).direction());
//     Direction_2 qd(as_source(b, _segment.source()).direction());
//     Direction_2 d(_segment.direction());
//     return pd.counterclockwise_in_between(qd, d);
//   }

  bool operator()(const Point_2& a, const Point_2& b)
  {
    static const Number_type EPSILON = 0.00000000001;
    if (abs(a.x()-b.x())<EPSILON && abs(a.y()-b.y())<EPSILON) {
      return a.id() < b.id();
    }

    typedef CGAL::Direction_2<Kernel> Direction_2;
    Direction_2 pd(Segment_2(_segment.source(), a).direction());
    Direction_2 qd(Segment_2(_segment.source(), b).direction());
    Direction_2 d(_segment.direction());
    return pd.counterclockwise_in_between(qd, d);
  }

private:
  Segment_2 _segment;
};

Triangle make_triangle(Point_3 a, Point_3 b, Point_3 c, bool reverse, 
                       map<Point_2, Point_2>& new2old)
{
  static log4cplus::Logger logger = log4cplus::Logger::getInstance("tiler.make_triangle");

  if (new2old.find(a) != new2old.end()) {
    LOG4CPLUS_TRACE(logger, "changing " << pp(a) << " to " << pp(new2old[a]));
    a = new2old[a];
  }
  if (new2old.find(b) != new2old.end()) b = new2old[b];
  if (new2old.find(c) != new2old.end()) c = new2old[c];

  if (reverse)
    return Triangle(c, b, a);
  return Triangle(a, b, c);
}

template <typename Out_iter>
Number_type partition(const Polygon_2& P, Out_iter out)
{
  log4cplus::Logger logger = log4cplus::Logger::getInstance("tiler.partition");
//   if (P.is_convex()) {
//     *out++ = P;
//     return;
//   }
//   LOG4CPLUS_TRACE(logger, "Not convex: " << pp(P));

  LOG4CPLUS_TRACE(logger, "partition(): " << pp(P));

  if (!P.is_simple()) {
    LOG4CPLUS_ERROR(logger, "Polygon given to partition is not simple" << pp(P));
    throw logic_error("Polygon given to partition is not simple");
  }

  // Split the polygon into convex polygons
  typedef std::list<PT::Polygon_2> Partition_polygons;
  Partition_polygons partition_polygons;
  bool valid = true;
  // try {
    CGAL::approx_convex_partition_2(P.vertices_begin(), P.vertices_end(), 
				    std::back_inserter(partition_polygons));
  // }
  // catch(...) {
  //   valid = false;
  // }

  valid = valid && CGAL::convex_partition_is_valid_2(P.vertices_begin(), P.vertices_end(),
						     partition_polygons.begin(), partition_polygons.end());

  if (!valid)
  {
    LOG4CPLUS_DEBUG(logger, "Approximate convex partition failed: " << pp(P) << ".  Attempting Greene's algorithm.");

    partition_polygons.clear();
    valid = true;
    CGAL::greene_approx_convex_partition_2(P.vertices_begin(), P.vertices_end(), 
					   std::back_inserter(partition_polygons));
    valid = valid && CGAL::convex_partition_is_valid_2(P.vertices_begin(), P.vertices_end(),
						       partition_polygons.begin(), partition_polygons.end());
  }

  if (!valid)
  {
    // Failed again.  Bail.
    LOG4CPLUS_ERROR(logger, "Convex partition failed: " << pp(P));
    throw logic_error("Convex partition failed");
  }

  list<Polygon_2> polygons;
  list<Segment_2> segments;
  Number_type max_area = 0;
  for (Partition_polygons::iterator it = partition_polygons.begin(); it != partition_polygons.end(); ++it)
  {
    Polygon_2 p(it->vertices_begin(), it->vertices_end());
    if (!p.is_convex()) {
      LOG4CPLUS_ERROR(logger, "Somehow a non-convex polygon made it here: " << pp(p) << " Original polygon: " << pp(P));
      throw logic_error("Somehow a non-convex polygon made it here");
    }
    *out++ = p;
    if (p.area() > max_area) {
      max_area = p.area();
    }
//     partition(p, out);

//     if (!p.is_convex()) {
//       LOG4CPLUS_ERROR(logger, "Somehow a non-convex polygon made it here: " << pp(p) << " Original polygon: " << pp(P));
//       throw logic_error("Somehow a non-convex polygon made it here");
//     }
//     polygons.push_back(p);
//     LOG4CPLUS_TRACE(logger, "Convex polygon: " << pp(p));

//     segments.insert(segments.end(), p.edges_begin(), p.edges_end());
  }
  return max_area;
}

Point_3 find_bad_vertex(const Untiled_region& r) {
  log4cplus::Logger logger = log4cplus::Logger::getInstance("tiler.find_bad_vertex");

  list<SL_intersection> intersections;
  map<Point_2, Point_2> old2new;
  Polygon_2 P = to_polygon(r, old2new);
  get_intersections(P.edges_begin(), P.edges_end(), back_inserter(intersections), true, false);
  boost::unordered_set<Segment_2> seg_set;
  Segment_2 main_seg;
  Point_2 bad_vertex;
  for (list<SL_intersection>::iterator it = intersections.begin(); it != intersections.end(); ++it) {
    const list<Segment_2>& interiors = it->interiors();
    if (interiors.size() == 0) {
      LOG4CPLUS_ERROR(logger, "Can't currently handle end-end intersections");
      throw logic_error("Can't currently handle end-end intersections");
    }
    if (interiors.size() == 2) {
      LOG4CPLUS_ERROR(logger, "Can't currently handle int-int intersections");
      throw logic_error("Can't currently handle int-int intersections");
    }
    for (list<Segment_2>::const_iterator in = interiors.begin(); in != interiors.end(); ++in) {
      if (seg_set.find(*in) != seg_set.end()) {
	LOG4CPLUS_TRACE(logger, "Main segment: " << pp(*in));
	seg_set.erase(*in);
	main_seg = *in;
      }
      else {
	seg_set.insert(*in);
      }
    }
    const list<Segment_2>& ends = it->ends();
    for (list<Segment_2>::const_iterator end = ends.begin(); end != ends.end(); ++end) {
      seg_set.insert(*end);
    }
    LOG4CPLUS_TRACE(logger, "Bad vertex: " << pp(it->point()));
    bad_vertex = it->point();
  }
  
  if (seg_set.empty()) {
    LOG4CPLUS_ERROR(logger, "No intersection found");
    throw logic_error("No intersection found");
  }

  Line_2 line = main_seg.supporting_line();
  Segment_2 seg = *seg_set.begin();
  Point_2 off_point = seg.source();
  if (off_point == bad_vertex) {
    off_point = seg.target();
  }
  CGAL::Vector_2<Kernel> v(bad_vertex, off_point);
  double len = sqrt(v.squared_length());
  double frac = 0.001/len;
  CGAL::Direction_2<Kernel> d = v.direction();

  Polygon_2 Q(P);
  // Polygon_2::Vertex_iterator it = find(Q.vertices_begin(), Q.vertices_end(), bad_vertex);
  Untiled_region rr;
  for (Untiled_region::const_iterator it = r.begin(); it != r.end(); ++it) {
    if (it->id() == bad_vertex.id()) {
      // rr.push_back(Point_3(it->x()+0.001, it->y(), it->z()));
      rr.push_back(Point_3(it->x()+d.dx()*frac, it->y()+d.dy()*frac, it->z()));
      LOG4CPLUS_TRACE(logger, "  xxx: " << pp(Point_3(it->x()+d.dx()*frac, it->y()+d.dy()*frac, it->z())));
    }
    else {
      rr.push_back(*it);
      LOG4CPLUS_TRACE(logger, "  xxx: " << pp(*it));
    }
  }
  // Untiled_region::iterator it = find(rr.begin(), rr.end(), bad_vertex);
  // LOG4CPLUS_TRACE(logger, "Found bad: " << pp((Point_2)*it));
  // it->x() += 0.02;
  // LOG4CPLUS_TRACE(logger, "Fixed: " << pp(rr));

  return Point_3();

  // Polygon_2 Q;
  // Polygon_2::Vertex_const_circulator first = P.vertices_circulator();
  // Polygon_2::Vertex_const_circulator c = P.vertices_circulator();
  // Polygon_2::Vertex_const_circulator bad = first;
  // do {
  //   Q.push_back(*c);
  //   LOG4CPLUS_TRACE(logger, "Temp polygon size: " << Q.size());
  //   if (Q.size() >= 3) {
  //     if (!Q.is_simple()) {
  // 	LOG4CPLUS_TRACE(logger, "Not simple");
  // 	bad = c;
  //     }
  //   }
  //   c++;
  // } while (c != first && bad == first);

  // if (c == first) {
  //   throw logic_error("Should have reached bad vertex");
  // }

  // return *c;

  // first = bad;
  // c = bad;
  
  // do {
  //   Q.push_back(*c);
  //   if (Q.size() >= 3) {
  //     if (!Q.is_simple()) {
  // 	bad = c;
  //     }
  //   }
  //   c--;
  // } while (c != first && bad == first);

  // if (c == first) {
  //   throw logic_error("Should have reached bad vertex");
  // }
}

string pp(const Triangle& T)
{
  stringstream ss;
  for (int i = 0; i < 3; ++i) {
    ss << i << ": " << pp(T[i]) << " ";
  }
  return ss.str();
}

template <typename Tile_iterator, typename ID_factory>
void medial_axis_stable(const Untiled_region& r, Number_type zmid, Tile_iterator tiles, 
			ID_factory& id_factory, Tiler_workspace& tw)
{
  log4cplus::Logger logger = log4cplus::Logger::getInstance("tiler.medial_axis_stable");

  Untiled_region region(r);
  map<Point_2, Point_2> old2new, new2old;

  // Simplify the region into a polygon with no degenerate points.
  Polygon_2 P = to_polygon(region, old2new);

  LOG4CPLUS_TRACE(logger, "medial_axis_stable called with: " << pp(P));

  // Basic error checking
  if (!P.is_simple()) {
    LOG4CPLUS_ERROR(logger, "Polygon being given to the medial axis splitter is not simple: " << pp(P));
    try {
      LOG4CPLUS_ERROR(logger, "Bad point: " << pp(find_bad_vertex(region)));
    } catch (logic_error& e) {
      LOG4CPLUS_ERROR(logger, "Failed to find bad point: " << e.what());
    }
    return;
  }

  if (P.size() == 0) {
    LOG4CPLUS_WARN(logger, "Polygon being given to the medial axis splitter is empty");
    return;
  }

  bool planar = true;
  Number_type planar_z = P[0].z();
  for (auto v_it = P.vertices_begin(); v_it != P.vertices_end(); ++v_it) {
    const Point_2& v = *v_it;
    planar = (v.z() == planar_z);
  }
  LOG4CPLUS_TRACE(logger, "planar = " << planar);
  
  try {
    tw.propagate_z_home(P.vertices_begin(), P.vertices_end());
  }
  catch (logic_error& e) {
    LOG4CPLUS_ERROR(logger, "Failed to propagate z_home - aborting medial axis tile: " << pp(P));
    return;
  }

  Number_type z_home;
  for (auto v_it = P.vertices_begin(); v_it != P.vertices_end(); ++v_it) {
    const Point_2& v = *v_it;
    if (tw.has_z_home(v)) {
      z_home = tw.z_home(v);
      break;
    }
  }

  // Ensure validity (ccw)
  bool reverse = P.is_clockwise_oriented();
  if (reverse) {
    LOG4CPLUS_TRACE(logger, "Reversing untiled region polygon");
    region.reverse_orientation();
    old2new.clear();
    P = to_polygon(region, old2new);
  }

  // P = remove_duplicate_vertices(P);

  // Adjust points that are close to another edge, making the possibility of
  // nonsimple polygons.
  Polygon_2 old_P = P;
  P = adjust_nonsimple_polygon(P, 0.00001, old2new);
  for (auto p_it = old_P.vertices_begin(); p_it != old_P.vertices_end(); ++p_it) {
    const Point_2& p = *p_it;
    if (old2new.find(p) == old2new.end()) {
      old2new[p] = p;
    }
    else {
      LOG4CPLUS_TRACE(logger, "old = " << pp(p) << " new = " << pp(old2new[p]));
    }
    new2old[old2new[p]] = p;
  }

  LOG4CPLUS_TRACE(logger, "Splitting polygon: " << pp(P));

  // Split the polygon into convex polygons
  list<Polygon_2> polygons;
  list<Segment_2> segments;
  try {
    Number_type max_area = partition(P, std::back_inserter(polygons));
  }
  catch (logic_error& e) {
    LOG4CPLUS_ERROR(logger, "Failed to partition polygon - aborting medial axis tile: " << pp(P));
    LOG4CPLUS_ERROR(logger, "Original: " << pp(old_P));
    return;
  }
  for (const auto& p : polygons) {
    segments.insert(segments.end(), p.edges_begin(), p.edges_end());
    LOG4CPLUS_TRACE(logger, "convex polygon area = " << p.area());
  }

  const Number_type zdiff = tw.zmax() - tw.zmin();

  // * anchor
  // + polygon center
  // x midpnt
  //
  //       *------------------------*
  //        \                       |
  //         \                      |
  //          \                     |
  //           \          +         |
  //            \         |         |
  //             \        |         |
  //              \       |         |
  //               *------x---------*
  //              /       |         |
  //             /        |         |
  //            /         |         |
  //           /          +         |
  //          /                     |
  //         /                      |
  //        *-----------------------*
  //
  //   edge2center maps *--*  ---->  +
  // vertex2extras maps   *   ---->  + x

  list<Point_2> centers;
  list<Point_2> anchors(P.vertices_begin(), P.vertices_end());

  // Create a map of edges and their associated polygon center points.  The center
  // points are the center of the convex polygon the edge belongs to.
  boost::unordered_map<Segment_2, Point_3> edge2center;
  Number_type largest_area = std::numeric_limits<Number_type>::min();
  Point_2 largest_centroid;
  for (const auto& p : polygons) {
    Number_type area = p.area();
    Point_3 center = centroid(p, zmid);
    const size_t id = id_factory.unique_id();

    if (!p.has_on_bounded_side(center)) {
      LOG4CPLUS_TRACE(logger, "Not simple.  Polygon: " << pp(p));
      LOG4CPLUS_TRACE(logger, "Not simple.  Area: " << p.area());
      CGAL::Bbox_2 bb = p.bbox();
      center = Point_3((bb.xmin() + bb.xmax())/2.0, (bb.ymin() + bb.ymax())/2.0, zmid);
    }
    center.id() = id;
    tw.set_z_home(center, z_home);
    LOG4CPLUS_TRACE(logger, "center: " << pp(center) << " " << pp(p));

    for (Polygon_2::Edge_const_iterator e_it = p.edges_begin(); e_it != p.edges_end(); ++e_it) {
      edge2center[*e_it] = center;
    }

    // If we're tiling an end contour (only one contour tiling to the z midpoint)
    // then we're going to interpolate using the polygon and the centroid of the
    // largest sub-polygon, which will be at the midpoint.
    if (planar) {
      if (p.area() > largest_area) {
	if (largest_area > std::numeric_limits<Number_type>::min()) {
	  centers.push_back(largest_centroid);
	}
	largest_centroid = center;
	largest_area = p.area();
      }
      else {
	centers.push_back(center);
      }
    }
    else {
      centers.push_back(center);
    }
  }
  
  boost::unordered_map<Point_2, Number_type> point2z;
  if (planar) {
    anchors.push_back(largest_centroid);
    point2z[largest_centroid] = largest_centroid.z();
  }

  // Find shared edges (cuts) between convex polygons
  list<SL_intersection> intersections;
  get_intersections(segments.begin(), segments.end(), back_inserter(intersections), true, true);

  Vertex2extras vertex2extras;
  for (const auto& i : intersections) {
    const list<Segment_2>& ends = i.ends();
    if (ends.size() == 2) {
      Segment_2_undirected a(*ends.begin());
      Segment_2_undirected b(*ends.rbegin());
      if (a == b) {
	const Segment_2& edge1 = *ends.begin();
	const Segment_2& edge2 = *ends.rbegin();
	const Point_3& center1 = edge2center[edge1];
	const Point_3& center2 = edge2center[edge2];
// 	Point_3 midpnt = midpoint(a, Segment_2(center1, center2), zmid);
	Point_3 midpnt = midpoint(a, Segment_2(center1, center2), (center1.z() + center2.z())/2.0);
	midpnt.id() = id_factory.unique_id();
        LOG4CPLUS_TRACE(logger, "midpnt: " << pp(midpnt) << " " << pp(Segment_2(center1, center2)));
	tw.set_z_home(midpnt, z_home);
	centers.push_back(midpnt);

	vertex2extras[a.lesser()].push_back(midpnt);
	vertex2extras[a.lesser()].push_back(center1);
	vertex2extras[a.lesser()].push_back(center2);

	vertex2extras[a.greater()].push_back(midpnt);
	vertex2extras[a.greater()].push_back(center1);
	vertex2extras[a.greater()].push_back(center2);

	LOG4CPLUS_TRACE(logger, "Inserting common edge: " << pp(a.orig_segment()));
      }
    }
  }

  Number_type anchor_min = std::numeric_limits<Number_type>::max();
  Number_type anchor_max = std::numeric_limits<Number_type>::min();
  for (list<Point_2>::iterator it = anchors.begin(); it != anchors.end(); ++it) {
    anchor_min = min(anchor_min, it->z());
    anchor_max = max(anchor_max, it->z());
  }
  const Number_type anchor_mid = (anchor_min + anchor_max)/2.0;
  // Draw points slightly in toward center to avoid numerical problem causing
  // center points on slices.
  for (list<Point_2>::iterator it = anchors.begin(); it != anchors.end(); ++it) {
    it->z() = it->z() * 0.99 + anchor_mid * 0.01;
  }
  
  interpolate(anchors.begin(), anchors.end(), centers.begin(), centers.end(), zmid);

  for (const auto& c : centers) {
    if (c.z() > anchor_max || c.z() < anchor_min) {
      LOG4CPLUS_ERROR(logger, "Bad interpolation: failing.  z = " << c.z()
        	      << ", min = " << anchor_min << ", max = " << anchor_max << "  Anchors:");
      for (const auto& a : anchors) {
        LOG4CPLUS_DEBUG(logger, "  Anchors = " << pp(a));
      }
      for (const auto& p : polygons) {
        LOG4CPLUS_DEBUG(logger, "  Polygon = " << pp(p));
      }
      LOG4CPLUS_DEBUG(logger, "  Original polygon = " << pp(P));
      return;
      // LOG4CPLUS_TRACE(logger, "Bad interpolation: failing.  z = " << c.z()
      //   	      << ", min = " << anchor_min << ", max = " << anchor_max << "  Anchors:");
      // for (const auto& a : anchors) {
      //   LOG4CPLUS_TRACE(logger, "  " << pp(a));
      // }
      // for (const auto& p : polygons) {
      //   LOG4CPLUS_TRACE(logger, "  " << pp(p));
      // }
      // point2z[c] = anchor_mid;
    }
    else {
      point2z[c] = c.z();
    }
  }

  // Sort the cuts in ccw order
  typedef Untiled_region::const_iterator Iter;
  for (Iter it = region.begin(); it != region.end(); ++it) {
    Point_2 vertex = old2new[*it];
    if (vertex2extras.find(vertex) != vertex2extras.end()) {
      Point_2 previous = *prev(it, region);
      Segment_2 segment(vertex, previous);
      list<Point_2>& extras = vertex2extras[vertex];
      extras.sort(Angle_functor(segment));
      extras.unique();

      // debug
      if (vertex.id() == 1397 && previous.id() == 1398) {
        for (const auto& p : extras) {
          LOG4CPLUS_TRACE(logger, "Extra: " << pp(p));
        }
      }
      // /debug

      // The first point will be taken care of by standard adding
      extras.pop_front();
    }
  }

  // Create the tiles by iterating over the original untiled region.
  Iter begin = region.begin();
  while (!valid_as_start(begin, region, vertex2extras, old2new)) {
    LOG4CPLUS_TRACE(logger, "Not valid as start: " << pp(*begin));
    ++begin;
  }

  Iter it = begin;
  Point_3 cur;
  do {
    Point_2 p = old2new[*it];
    Point_2 n = old2new[*(next(it, region))];
    Segment_2 edge(p, n);
    Segment_2_undirected edge_u(edge);
    const Point_2& source = edge.source();
    const Point_2& target = edge.target();

    LOG4CPLUS_TRACE(logger, "Tiling edge: " << pp(edge));

    // If the source and target are equal, it's a double: tile
    // to the current point (centroid of the polygon)
    if (xy_equal(source, target)) {
      try {
        Triangle T = make_triangle(source, target, cur, reverse, new2old);
        if (!T.is_degenerate()) {
          *tiles++ = T;
        }
        else {
          LOG4CPLUS_WARN(logger, "Refused to add degenerate triangle: " << pp(T));
        }
	LOG4CPLUS_TRACE(logger, "Made triangle: " << pp(T));
      } catch(std::logic_error& e) {
	LOG4CPLUS_WARN(logger, "Failed to make triangle: " << pp(source) <<
                       " " << pp(target) << " " << pp(cur) <<
                       " " << e.what());
      }
    }
    
    // Otherwise:
    //   1) tile from the current edge to the centroid
    //   2) tile from the target to extra to centroid
    else {
      cur = edge2center[edge];
      cur = Point_3(cur.x(), cur.y(), point2z[cur], cur.id());
      try {
        Triangle T = make_triangle(source, target, cur, reverse, new2old);
        if (!T.is_degenerate()) {
          *tiles++ = T;
        }
        else {
          LOG4CPLUS_WARN(logger, "Refused to add degenerate triangle: " << pp(T));
        }
	LOG4CPLUS_TRACE(logger, "Made triangle: " << pp(T));
      } catch(std::logic_error& e) {
	LOG4CPLUS_WARN(logger, "Failed to make triangle: " << pp(source) << 
                       " " << pp(target) << " " << pp(cur) <<
                       " " << e.what());
      }

      if (vertex2extras.find(target) != vertex2extras.end()) {
	// const list<Point_2>& extras = vertex2extras[target];
	// for (list<Point_2>::const_iterator e_it = extras.begin(); e_it != extras.end(); ++e_it) {
	//   Point_3 next = *e_it;
	const vector<Point_2> extras(vertex2extras[target].begin(), vertex2extras[target].end());
        for (int i = 0; i < extras.size(); ++i) {
	  Point_3 next = extras[i];
	  next = Point_3(next.x(), next.y(), point2z[next], next.id());
// 	  next.z() = point2z[next];

	  try {
            Triangle T = make_triangle(target, next, cur, reverse, new2old);
            if (!T.is_degenerate()) {
              *tiles++ = T;
            }
            else {
              LOG4CPLUS_WARN(logger, "Refused to add degenerate triangle: " << pp(T));
            }
            LOG4CPLUS_TRACE(logger, "Made triangle: " << pp(T));
	  } catch(std::logic_error& e) {
	    LOG4CPLUS_WARN(logger, "Failed to make triangle: " << pp(target) << 
                       " " << pp(next) << " " << pp(cur) <<
                       " " << e.what());
	  }
	  cur = next;
	}
      }
    }
    it = next(it, region);
  } while (it != begin);
}


template
void medial_axis_stable(const Untiled_region& region, Number_type zmid, 
			back_insert_iterator<list<Triangle> > tiles, Vertices& id_factory, Tiler_workspace& tw);

template
void medial_axis_stable(const Untiled_region& region, Number_type zmid, 
			back_insert_iterator<vector<Triangle> > tiles, Vertices& id_factory, Tiler_workspace& tw);

// void to_simple(const Untiled_region& region)
// {
//   log4cplus::Logger logger = log4cplus::Logger::getInstance("tiler.to_simple");

//   LOG4CPLUS_TRACE(logger, "Beginning polygon");

//   Polygon_2 P = to_polygon(region);
//   typedef Polygon_2::iterator Iter;
//   for (Iter it = P.vertices_begin(); it != P.vertices_end(); ++it) {
//     Iter next = it;
//     ++next;
//     if (find_if(next, P.vertices_end(), xy_pred(*it)) != P.vertices_end()) 
//       LOG4CPLUS_TRACE(logger, pp(*it));
//   }
// }

CONTOURTILER_END_NAMESPACE
