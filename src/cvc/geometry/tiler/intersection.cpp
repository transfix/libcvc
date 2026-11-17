#include <ContourTiler/intersection.h>
#include <ContourTiler/common.h>
#include <list>
#include <ContourTiler/triangle_utils.h>
#include <ContourTiler/print_utils.h>
#include <ContourTiler/segment_utils.h>
#include <ContourTiler/XY_equal_predicate.h>
#include <ContourTiler/sweep_line_visitors.h>
#include <ContourTiler/cut.h>

CONTOURTILER_BEGIN_NAMESPACE

using namespace std;

template <typename Iter>
void parameter_dump(log4cplus::Logger& logger, Iter begin, Iter end)
{
  for (Iter it = begin; it != end; ++it)
  {
    string name = *it;
    string value = *(++it);
    if (name != "")
      LOG4CPLUS_DEBUG(logger, "  " << name << " = " << value);
  }
}

void parameter_dump(log4cplus::Logger& logger, 
		    const string& n1, const string& p1, const string& n2, const string& p2,
		    const string& n3, const string& p3, const string& n4, const string& p4,
		    const string& n5, const string& p5, const string& n6, const string& p6,
		    const string& n7, const string& p7, const string& n8, const string& p8,
		    const string& n9, const string& p9)
{
  string values[] = { n1, p1, n2, p2, n3, p3, n4, p4, n5, p5, n6, p6, n7, p7, n8, p8, n9, p9 };
  parameter_dump(logger, values, values+18);
}

void parameter_dump(log4cplus::Logger& logger, 
		    const string& n1, const string& p1, const string& n2, const string& p2,
		    const string& n3, const string& p3, const string& n4, const string& p4,
		    const string& n5, const string& p5, const string& n6, const string& p6,
		    const string& n7, const string& p7, const string& n8, const string& p8)
{
  string values[] = { n1, p1, n2, p2, n3, p3, n4, p4, n5, p5, n6, p6, n7, p7, n8, p8 };
  parameter_dump(logger, values, values+16);
}

void parameter_dump(log4cplus::Logger& logger, 
		    const string& n1, const string& p1, const string& n2, const string& p2,
		    const string& n3, const string& p3, const string& n4, const string& p4,
		    const string& n5, const string& p5, const string& n6, const string& p6,
		    const string& n7, const string& p7)
{
  string values[] = { n1, p1, n2, p2, n3, p3, n4, p4, n5, p5, n6, p6, n7, p7 };
  parameter_dump(logger, values, values+14);
}

void parameter_dump(log4cplus::Logger& logger, 
		    const string& n1, const string& p1, const string& n2, const string& p2,
		    const string& n3, const string& p3, const string& n4, const string& p4,
		    const string& n5, const string& p5, const string& n6, const string& p6)
{
  string values[] = { n1, p1, n2, p2, n3, p3, n4, p4, n5, p5, n6, p6 };
  parameter_dump(logger, values, values+12);
}

void parameter_dump(log4cplus::Logger& logger, 
		    const string& n1, const string& p1, const string& n2, const string& p2,
		    const string& n3, const string& p3, const string& n4, const string& p4,
		    const string& n5, const string& p5)
{
  string values[] = { n1, p1, n2, p2, n3, p3, n4, p4, n5, p5 };
  parameter_dump(logger, values, values+10);
}

//------------------------------------------------------------------------------
// point_intersection_2
//
/// Returns true if the two segments intersect at a point in two dimensions.
/// The intersection must occur on the projection onto the z plane.
/// Returns false if the two segments either don't intersect or if they 
/// intersect on a segment.
//------------------------------------------------------------------------------
bool point_intersection_2(const Segment_3& a, const Segment_3& b, Point_3& ia, Point_3& ib)
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
  bool intersects = ua >= 0 && ua <= 1 && ub >= 0 && ub <= 1;
//   bool intersects = ua >  0 && ua <  1 && ub >  0 && ub <  1;

  if (intersects)
  {
    ia = Point_3(a.source().x() + ua * (a.target().x() - a.source().x()),
		 a.source().y() + ua * (a.target().y() - a.source().y()),
		 a.source().z() + ua * (a.target().z() - a.source().z()));
    ib = Point_3(ia.x(),//b.source().x() + ub * (b.target().x() - b.source().x()),
		 ia.y(),//b.source().y() + ub * (b.target().y() - b.source().y()),
		 b.source().z() + ub * (b.target().z() - b.source().z()));
  }

  return intersects;
}

//------------------------------------------------------------------------------
// point_intersection_2
//
/// Returns true if segment a intersects with line b at a point in two
/// dimensions.  The intersection must occur on the projection onto the z plane.
/// Returns false if the two segments either don't intersect or if they 
/// intersect on a segment.
//------------------------------------------------------------------------------
bool point_intersection_2(const Segment_3& a, const Segment_3& b, Point_3& ia)
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
  bool intersects = ua > 0 && ua < 1;

  if (intersects)
  {
    ia = Point_3(a.source().x() + ua * (a.target().x() - a.source().x()),
		 a.source().y() + ua * (a.target().y() - a.source().y()),
		 a.source().z() + ua * (a.target().z() - a.source().z()));
  }

  return intersects;
}

Segment_2 to_2(const Segment_3& s)
{ return Segment_2(s.source(), s.target()); }

Segment_3 to_3(const Segment_2& s)
{ return Segment_3(s.source(), s.target()); }

// template <typename Out_iter>
// void remove_intersections(TW_handle tw_yellow,
// 			  TW_handle tw_green,
// 			  Number_type yz, Number_type gz,
// 			  Out_iter new_yellow, Out_iter new_green, Number_type epsilon)
// {
// }

// edge2points is a map of any tile edges on slices that may have been modified.
//   The points will have to be induced on these edges in other tilings.  For example,
//   if edge e on slice 4 has an induced point when tiling slices 3 and 4, then that
//   same edge will need an induced point when tiling slices 4 and 5.
template <typename Tile_iter, typename Out_iter>
void remove_intersections(TW_handle tw_yellow,
			  TW_handle tw_green,
			  Tile_iter yellow_begin, Tile_iter yellow_end,
			  Tile_iter green_begin, Tile_iter green_end,
			  Number_type yz, Number_type gz,
			  Out_iter new_yellow, Out_iter new_green, Number_type epsilon,
			  boost::unordered_map<Segment_3_undirected, list<Point_3> >& edge2points)
{
  static log4cplus::Logger logger = log4cplus::Logger::getInstance("intersection.multitiler");

  typedef typename iterator_traits<Tile_iter>::value_type Tile_handle;

  LOG4CPLUS_TRACE(logger, "removing intersections; home_z = " << yz << " " << gz);
  Z_adjustments<Tile_handle> z_adjustments(yz, gz, epsilon);
  Intersections<Tile_handle> ints = get_intersections(tw_yellow, tw_green, yellow_begin, yellow_end, green_begin, green_end, z_adjustments);

  TW_handle tws[2];
  tws[0] = tw_yellow;
  tws[1] = tw_green;

  boost::unordered_map<Tile_handle, boost::unordered_set<Polyline_2> > cuts;
  get_polyline_cuts(tw_yellow, tw_green, ints, cuts, z_adjustments);

  list<Tile_handle> new_tiles[2];
  new_tiles[0].insert(new_tiles[0].end(), yellow_begin, yellow_end);
  new_tiles[1].insert(new_tiles[1].end(), green_begin, green_end);

  list<Segment_3_> lines;
  boost::unordered_set<Tile_handle> bad_tiles;
  typedef typename boost::unordered_map<Tile_handle, boost::unordered_set<Polyline_2> >::iterator cuts_iter;
  for (cuts_iter it = cuts.begin(); it != cuts.end(); ++it)
    bad_tiles.insert(it->first);

  bad_tiles.insert(z_adjustments.tiles_begin(), z_adjustments.tiles_end());

  for (typename boost::unordered_set<Tile_handle>::iterator it = bad_tiles.begin();
       it != bad_tiles.end();
       ++it)
  {
    Tile_handle tile = *it;
    boost::unordered_set<Polyline_2>& c = cuts[tile];
    int i = ints.tile2idx[tile];
    new_tiles[i].remove(tile);

    LOG4CPLUS_TRACE(logger, "Cutting tile for component " << i << ": " << pp_tri(*tile));

    // debug
    LOG4CPLUS_TRACE(logger, "  Cuts:");
    for (boost::unordered_set<Polyline_2>::const_iterator c_it = c.begin(); c_it != c.end(); ++c_it) {
      LOG4CPLUS_TRACE(logger, "   " << pp(*c_it));
    }

    typedef boost::unordered_set<Segment_3_undirected> EdgeSet;
    typedef boost::unordered_map<Point_3, EdgeSet> Point2Edges;

    list<Polygon_2> polygons;
    Point2Edges point2edges;
    cut_tile_with_polylines(tile, c.begin(), c.end(), back_inserter(polygons), ints, i, point2edges);

    Number_type zmin = tw_yellow->zmin();
    Number_type zmax = tw_yellow->zmax();
    for (Point2Edges::iterator p2e_it = point2edges.begin(); p2e_it != point2edges.end(); ++p2e_it) {
      for (EdgeSet::iterator e_it = p2e_it->second.begin(); e_it != p2e_it->second.end(); ++e_it) {
	const Segment_3_undirected& edge = *e_it;
	if (edge.lesser().z() == edge.greater().z()) {
	  if (edge.lesser().z() == zmin || edge.lesser().z() == zmax) {
	    edge2points[*e_it].push_back(p2e_it->first);
	  }
	}
      }
    }

    for (typename list<Polygon_2>::iterator p_it = polygons.begin(); p_it != polygons.end(); ++p_it)
    {
      // Adjust z values
      Polygon_2& p = *p_it;
      Number_type z_home = tws[i]->z_home(p.vertices_begin(), p.vertices_end());

      LOG4CPLUS_TRACE(logger, "Polygon to triangulate for component " << i << ": " << pp(p));
//       if (!p.is_simple()) {
// 	LOG4CPLUS_ERROR(logger, "  Polygon is not simple");
//       }
      list<Triangle> nt;
      try {
	triangulate_safe(p, back_inserter(nt), point2edges);
      }
      catch (logic_error& e) {
	LOG4CPLUS_WARN(logger, "Failed to cut " << pp(p) << " into triangles: " << e.what());
      }
      for (list<Triangle>::iterator t_it = nt.begin(); t_it != nt.end(); ++t_it)
      {
	Tile_handle t(new Triangle(*t_it));
	t = z_adjustments.update(i, t);

	for (int j = 0; j < 3; ++j) {
	  Point_3& vertex = (*t)[j];
	  if (!vertex.is_valid())
	    vertex.id() = tws[i]->vertices.unique_id();
	  tws[i]->set_z_home(vertex, z_home);
	}

	LOG4CPLUS_TRACE(logger, "Final tile for component " << i << ": " << pp_tri(*t));
	new_tiles[i].push_back(t);
      }
    }
  }

  static int total_num_tiles = 0;
  for (typename list<Tile_handle>::iterator it = new_tiles[0].begin(); it != new_tiles[0].end(); ++it)
  {
    *new_yellow = *it;
    ++new_yellow;
    total_num_tiles++;
  }
  for (typename list<Tile_handle>::iterator it = new_tiles[1].begin(); it != new_tiles[1].end(); ++it)
  {
    *new_green = *it;
    ++new_green;
    total_num_tiles++;
  }
//   LOG4CPLUS_DEBUG(logger, "Final total number of tiles: " << total_num_tiles);
}


//------------------------------------------------------------------------------
// get_intersections
//
//------------------------------------------------------------------------------
template <typename Tile_iter>
Intersections<typename std::iterator_traits<Tile_iter>::value_type>
get_intersections(TW_handle tw_yellow, TW_handle tw_green,
		  Tile_iter ybegin, Tile_iter yend, Tile_iter gbegin, Tile_iter gend, 
		  Z_adjustments<typename std::iterator_traits<Tile_iter>::value_type>& z_adjustments)
{
  static log4cplus::Logger logger = log4cplus::Logger::getInstance("intersection.get_intersections");

  typedef typename iterator_traits<Tile_iter>::value_type Tile_handle;
  typedef boost::unordered_set<Tile_handle>               Tile_set;
  typedef boost::unordered_set<Segment_3_>                Edge_set;

  Intersections<Tile_handle> ret;
  Tile_iter begin[] = { ybegin, gbegin };
  Tile_iter end[] = { yend, gend };
  Edge_set edge_set[2];
  list<Segment_2> edges[2];
  boost::unordered_map<Point_3, list<Tile_handle> > vertex2tiles;
  int num_tiles[] = {0, 0};
  int num_intersections = 0;

  // Compile all edges of each component (yellow and green) into two
  // sets.
  for (int i = 0; i < 2; ++i) {
    for (Tile_iter it = begin[i]; it != end[i]; ++it) {
      Tile_handle tile_h = *it;
      for (int j = 0; j < 3; ++j) {
	Segment_3_ e = edge(j, *tile_h);
	ret.insert(e, tile_h, i);
 	edge_set[i].insert(e);
// 	edges[i].push_back(to_2(e.orig_segment()));
	Point_3 v = vertex(j, *tile_h);
	vertex2tiles[v].push_back(tile_h);
      }
      ret.tile2idx[tile_h] = i;
      ++num_tiles[i];
    }

    for (Edge_set::const_iterator it = edge_set[i].begin(); it != edge_set[i].end(); ++it) {
      edges[i].push_back(to_2(it->orig_segment()));
    }
  }

  boost::unordered_set<Point_2> temp;

  int yi = 0;
  int gi = 1;

  TW_handle tws[2];
  tws[yi] = tw_yellow;
  tws[gi] = tw_green;

  // Get and iterate through all edge-edge intersections
  list<SL_intersection> intersections;
  get_intersections(edges[0].begin(), edges[0].end(),
		    edges[1].begin(), edges[1].end(),
		    back_inserter(intersections), true, true);

  for (list<SL_intersection>::iterator it = intersections.begin(); it != intersections.end(); ++it) {
    const SL_intersection& intersection = *it;
    const Point_2& point = intersection.point();

    Segment_3_ segments[] = { Segment_3_(to_3(intersection.segment(0))), 
			      Segment_3_(to_3(intersection.segment(1))) };
    Segment_3 ys = segments[0].segment();
    Point_3 p[2];
    // The line sweep catches intersections between segments that overlap along a segment
    // (rather than at a single point).  We want to weed these out, so do another check
    // with our own point intersection function.
    if (point_intersection_2(segments[0].segment(), segments[1].segment(), p[0], p[1])) {

      // edge0 and edge1 intersect at points p0 and p1
      if (!xy_equal(p[0], p[1])) {
	LOG4CPLUS_WARN(logger, "Intersection points are not xy-equal: " << pp(p[0]) << " " << pp(p[1]));
	throw logic_error("Intersection points are not xy-equal");
      }

      Point_3_handle new_p[2];

      Point_3& py = p[yi];
      Point_3& pg = p[gi];

      // The tiles touching the intersecting segments
      list<Tile_handle> tiles[2];
      for (int i = 0; i < 2; ++i) {
	ret.tiles(segments[i], i, back_inserter(tiles[i]));
      }

      list<Tile_handle>& ytiles = tiles[yi];
      list<Tile_handle>& gtiles = tiles[gi];

      Tile_handle ytile = *ytiles.begin();
      Tile_handle gtile = *gtiles.begin();
      Number_type y_z_home = tw_yellow->z_home(ytile);
      Number_type g_z_home = tw_green->z_home(gtile);
      if (y_z_home == g_z_home) {
	LOG4CPLUS_ERROR(logger, " y_z_home = " << y_z_home << " g_z_home = " << g_z_home);
	LOG4CPLUS_ERROR(logger, "tiles: " << pp_tri(*ytile) << " " << pp_tri(*gtile));
	Number_type ty_z_home = tw_yellow->z_home(ytile);
	Number_type tg_z_home = tw_green->z_home(gtile);
	throw logic_error("y_z_home == g_z_home");
      }

      py.id() = tw_yellow->vertices.unique_id();
      pg.id() = tw_green->vertices.unique_id();

      tw_yellow->set_z_home(py, y_z_home);
      tw_green->set_z_home(pg, g_z_home);

      Point_3 qy = ys.source();
      if (!in_between(y_z_home, py.z(), qy.z())) {
	qy = ys.target();
      }

      bool intersects_3d = !z_adjustments.is_legal(py, pg, qy, y_z_home, g_z_home);
      if (intersects_3d) {
	LOG4CPLUS_TRACE(logger, "Intersection (3d): " << pp(p[yi]) << " " << pp(p[gi]) << " "
			<< y_z_home << " " << g_z_home);
	temp.insert(p[0]);

	z_adjustments.add(py, pg, yi, gi, qy, ytiles.begin(), ytiles.end(), gtiles.begin(), gtiles.end(), y_z_home, g_z_home);
	++num_intersections;
      }
      else {
	LOG4CPLUS_TRACE(logger, "Intersection (2d): " << pp(p[0]) << " " << pp(p[1]));
      }

      // Associate each 2D point (xy_equal(p[0], p[1]) == true) with the two
      // edges forming the intersection.
      for (int i = 0; i < 2; ++i) {
	ret.insert(p[i], segments[i], p[1-i], i, tws[i]->z_home(*(tiles[i].begin())));
	LOG4CPLUS_TRACE(logger, "  Segment: " << pp(segments[i]));
      }
    }
  }

  // Test for interior intersections
  for (int i = 0; i < 2; ++i)
  {
    int tyi = i;
    int tgi = 1-tyi;
    for (Tile_iter y_it = begin[tyi]; y_it != end[tyi]; ++y_it)
    {
      Tile_handle yt = *y_it;
      for (int j = 0; j < 3; ++j)
      {
	const Point_3& yv = vertex(j, *yt);
	for (Tile_iter g_it = begin[tgi]; g_it != end[tgi]; ++g_it)
	{
	  Tile_handle gt = *g_it;
	  CGAL::Triangle_2<Kernel> gtri((*gt)[0], (*gt)[1], (*gt)[2]);
	  if (gtri.orientation() == CGAL::CLOCKWISE)
	    gtri = gtri.opposite();
	  if (gtri.has_on_positive_side(yv))
	  {
	    Point_3 pg = Point_3(yv.x(), yv.y(), get_z(*gt, yv));
	    LOG4CPLUS_TRACE(logger, "Testing for inside intersection yv: " << pp(yv) << " pg: " << pp(pg) << " gt: " << pp_tri(*gt));

	    Point_3 py = yv;
	    Number_type y_z_home = tws[tyi]->z_home(yt);
	    Number_type g_z_home = tws[tgi]->z_home(gt);

	    pg.id() = tws[tgi]->vertices.unique_id();
	    tws[tgi]->set_z_home(pg, g_z_home);

	    ret.insert(pg, py, tgi, g_z_home);
	    ret.insert(py, pg, tyi, y_z_home);
	    
	    Point_3 qy = vertex(0, *yt);
	    if (!in_between(y_z_home, py.z(), qy.z()))
	      qy = vertex(1, *yt);
	    if (!in_between(y_z_home, py.z(), qy.z()))
	      qy = vertex(2, *yt);

	    if (!z_adjustments.is_legal(py, pg, qy, y_z_home, g_z_home)) {
	      list<Tile_handle> ytiles, gtiles;
	      ytiles.insert(ytiles.end(), vertex2tiles[yv].begin(), vertex2tiles[yv].end());
	      gtiles.push_back(gt);

	      z_adjustments.add(py, pg, tyi, tgi, qy, ytiles.begin(), ytiles.end(), gtiles.begin(), gtiles.end(), y_z_home, g_z_home);
	      LOG4CPLUS_TRACE(logger, "Intersection (3d - inside): " << pp(py) << " " << pp(pg) << " " << y_z_home << " " << g_z_home);
	      temp.insert(py);
	      ++num_intersections;
	    }
	  }
	}
      }
    }
  }

  LOG4CPLUS_TRACE(logger, "Num tiles:  [0] = " << num_tiles[0] << " [1] = " << num_tiles[1]);
  LOG4CPLUS_TRACE(logger, "Num intersections:  [0] = " << num_intersections);

  static int total_num_tiles = 0;
  static int total_num_intersections = 0;
  total_num_tiles += num_tiles[0] + num_tiles[1];
  total_num_intersections += num_intersections;
//   LOG4CPLUS_DEBUG(logger, "Total num tiles:  " << total_num_tiles);
  LOG4CPLUS_DEBUG(logger, "Total num intersections:  " << total_num_intersections);

  return ret;
}

//------------------------------------------------------------------------------
// neighbor
//
/// Given two tiles, green and yellow, and an intersection point (where the 
/// intersection point lies on the green tile in 3D) between the
/// two, finds the neighbor of yintersection that also intersects with the green
/// tile, if there is one.  Returns true if there is such an intersection.
//------------------------------------------------------------------------------
template <typename Tile_handle>
bool neighbor(Tile_handle green, Tile_handle yellow, const Point_3& gp, int g, Point_3& gn, const Intersections<Tile_handle>& intersections)
{
  static log4cplus::Logger logger = log4cplus::Logger::getInstance("intersection.neighbor");

  int y = 1-g;
  const Point_2& yp = intersections.first_opposite(gp, g);
  list<Point_3> yneighbors;
  intersections.neighbors(yp, y, yellow, back_inserter(yneighbors));

  for (list<Point_3>::iterator n_it = yneighbors.begin(); n_it != yneighbors.end(); ++n_it)
  {
    const Point_3& yn = *n_it;
    gn = intersections.first_opposite(yn, y);
//     LOG4CPLUS_TRACE(logger, "Testing neighbor yn: " << pp(yn) << "  gn: " << pp(gn));
//     LOG4CPLUS_TRACE(logger, "  green tile: " << pp_tri(*green));
    list<Tile_handle> tiles;
    intersections.tiles(gn, g, back_inserter(tiles));

//     for (typename list<Tile_handle>::const_iterator it = tiles.begin(); it != tiles.end(); ++it)
//       LOG4CPLUS_TRACE(logger, "  gn tile: " << pp_tri(**it));

    if (find(tiles.begin(), tiles.end(), green) != tiles.end())
      return true;
  }
  LOG4CPLUS_TRACE(logger, "No neighbor found");
  return false;
}

//------------------------------------------------------------------------------
// neighbor
//
/// Given two tiles, green and yellow, and an intersection point (where the 
/// intersection point lies on the green tile in 3D) between the
/// two, finds the neighbor of yp that is inside the green
/// tile, if there is one.  Returns true if there is such a point.
//------------------------------------------------------------------------------
template <typename Tile_handle>
bool neighbor_inside(Tile_handle gt, Tile_handle yt, const Point_3& gp, int gi, Point_3& yn, Segment_3_& yseg, const Intersections<Tile_handle>& ints)
{
  static log4cplus::Logger logger = log4cplus::Logger::getInstance("intersection.neighbor");

  int yi = 1-gi;
  const Point_2& yp = ints.first_opposite(gp, gi);

  CGAL::Triangle_2<Kernel> cgal_tri((*gt)[0], (*gt)[1], (*gt)[2]);
  if (cgal_tri.is_degenerate())
    return false;

  if (cgal_tri.orientation() == CGAL::CLOCKWISE)
    cgal_tri = cgal_tri.opposite();

  list<Segment_3_> ye_list(ints.edges_begin(yp, yi), ints.edges_end(yp, yi));
  for (list<Segment_3_>::iterator it = ye_list.begin(); it != ye_list.end(); ++it)
  {
    yseg = *it;
    // Make sure the segment is on the tile
    if (index(yseg, *yt) > -1)
    {
      for (int i = 0; i < 2; ++i)
      {
	Point_3 p(yseg[i]);
	int idx = index(p, *yt);
	if (cgal_tri.has_on_positive_side(p) && 
	    idx > -1 && xyz_equal(p, (*yt)[idx]) &&
	    !xy_equal(p, yp))
	{
	  LOG4CPLUS_TRACE(logger, "Found point inside: " << pp(p) << " seg = " << pp(yseg) << " yp = " << pp(yp));
	  yn = p;
	  return true;
	}
      }
    }
  }
  return false;
}

//------------------------------------------------------------------------------
// find_edge_intersection
//
/// Given two tiles, green and yellow, and a yellow edge, returns true if the
/// edge intersects with the green tile.  If it does, gp is assigned the
/// intersection point.
//------------------------------------------------------------------------------
template <typename Tile_handle>
bool find_edge_intersection(Tile_handle gt, const Segment_3_& ye, int yi, Point_3& gp, const Intersections<Tile_handle>& ints)
{
  static log4cplus::Logger logger = log4cplus::Logger::getInstance("intersection.find_edge_intersection");

  if (ints.has_edge(ye, yi))
  {
    // there is an intersection with this edge
    vector<Point_3> points;
    ints.intersections(ye, yi, gt, back_inserter(points));
    if (points.size() == 0)
    {
      LOG4CPLUS_TRACE(logger, "Intersection but no points found for " << pp(ye) << " " << yi << " gt = " << pp_tri(*gt));
      return false;
    }
    gp = points[0];
    LOG4CPLUS_TRACE(logger, "Intersection found for " << pp(ye) << " " << yi << ": " << pp(gp));
    return true;
  }
  LOG4CPLUS_TRACE(logger, "No intersection found for " << pp(ye) << " " << yi);
  return false;
}

//------------------------------------------------------------------------------
// get_new_point
//
/// Returns a pair: the original or induced green point and the z-adjusted
/// green point.
//------------------------------------------------------------------------------
template <typename Tile_handle>
Point_3 get_new_point(TW_handle tw_yellow, TW_handle tw_green, Tile_handle gt, Tile_handle yt, const Point_3& py, int yi, const Intersections<Tile_handle>& ints, 
		      Z_adjustments<Tile_handle>& z_adjustments)
{
  static log4cplus::Logger logger = log4cplus::Logger::getInstance("intersection.get_new_point");

  int gi = 1 - yi;
  Point_3 pg;
  if (ints.has_point(py, yi))
  {
    pg = ints.first_opposite(py, yi);
  }
  else
  {
    pg = py;
    // Same as py but with z adjusted so that pg lies on the green tile
    pg = Point_3(pg.x(), pg.y(), get_z(*gt, pg));

    LOG4CPLUS_TRACE(logger, "Created new point inside triangle: " << pp(pg));
    LOG4CPLUS_TRACE(logger, "                        Component: " << gi);
    LOG4CPLUS_TRACE(logger, "       Opposite component's point: " << pp(py));

//     Number_type y_z_home;
//     Number_type g_z_home;
//     try {
//       y_z_home = tw_yellow->z_home(yt);
//       g_z_home = tw_green->z_home(gt);
//     } catch(logic_error& e) {
//       g_z_home = tw_yellow->z_home(gt);
//       y_z_home = tw_green->z_home(yt);
//     }

    // TODO: replace
//     Point_3 qy = z_adjustments.get_qy(py, pg, yi, gi, yt, gt, y_z_home);
//     if (!z_adjustments.is_legal(py, pg, qy, y_z_home, g_z_home)) {
//       LOG4CPLUS_TRACE(logger, "Intersection (3d - inside): " << pp(py) << " " << pp(pg) << " " << y_z_home << " " << g_z_home);
//       z_adjustments.add(py, pg, yi, gi, yt, gt, y_z_home, g_z_home);
//     }
  }
  return pg;
}

template <typename Tile_handle>
bool is_vertical(Tile_handle t)
{
//   return CGAL::collinear((Point_2)vertex(0, *t), (Point_2)vertex(1, *t), (Point_2)vertex(2, *t));
  return (xy_equal((*t)[0], (*t)[1]) ||
	  xy_equal((*t)[0], (*t)[2]) ||
	  xy_equal((*t)[1], (*t)[2]));
}

// Finds the vertex of t that is nearest the segment.  If it is very far away
// then throw.
template <typename Tile_handle>
Point_2 find_nearest_vertex(Tile_handle t, const Segment_3& seg)
{
  Segment_2 s(seg.source(), seg.target());
  int closest = 0;
  int dist = CGAL::squared_distance(s, (Point_2)(*t)[0]);
  for (int i = 1; i < 2; ++i)
  {
    if (CGAL::squared_distance(s, (Point_2)(*t)[i]) < dist)
    {
      dist = CGAL::squared_distance(s, (Point_2)(*t)[i]);
      closest = i;
    }
  }
  if (dist > 0.0000001)
    throw logic_error("Expected nearest vertex to be closer");

  return (*t)[closest];
}

bool xy_close(const Point_2& a, const Point_2& b, Number_type epsilon)
{
  return (abs(a.x() - b.x()) < epsilon &&
	  abs(a.y() - b.y()) < epsilon);
}

//------------------------------------------------------------------------------
// find_exit
//
/// Given two tiles, green and yellow, and an intersection point (where the 
/// intersection point lies on the green tile in 3D) between the
/// two, find the polyline from the intersection point through the interior
/// of the green tile to the intersection point where the polyline exits
/// the green tile.  The points of the polyline are on the yellow tile in 3D.
//------------------------------------------------------------------------------
template <typename Tile_handle>
Polyline_2 find_exit(TW_handle tw_yellow, TW_handle tw_green, Tile_handle tg, Tile_handle ty, const Point_3& pg, int ig, const Intersections<Tile_handle>& ints,
		     Z_adjustments<Tile_handle>& z_adjustments)
{
  static log4cplus::Logger logger = log4cplus::Logger::getInstance("intersection.find_exit");

  Polyline_2 ret;
  int yi = 1 - ig;
  const Point_3& yp = ints.first_opposite(pg, ig);
  Point_3 gn0 = pg;
  Number_type epsilon = 0.00000001;

  LOG4CPLUS_TRACE(logger, "Adding original point: " << pp(pg));
  ret.push_back(pg);

  // This case handles where the green tile's 2D projection is a line.
//   if (CGAL::collinear((Point_2)vertex(0, *tg), (Point_2)vertex(1, *tg), (Point_2)vertex(2, *tg)))
  if (is_vertical(tg))
  {
    LOG4CPLUS_TRACE(logger, "Green tile's 2D projection is collinear");
    return ret;
  }

  // This case handles where the yellow tile's 2D projection is a line.
//   if (CGAL::collinear((Point_2)vertex(0, *ty), (Point_2)vertex(1, *ty), (Point_2)vertex(2, *ty)))
  if (is_vertical(ty))
  {
    LOG4CPLUS_TRACE(logger, "Yellow tile's 2D projection is collinear");
    return ret;
  }

  // The intersection point is a vertex on either the yellow or green tiles.
//   if (index(yp, *ty) > -1 || index(pg, *tg) > -1)
//   {
//     LOG4CPLUS_TRACE(logger, "Intersection point is a vertex");
//     return ret;
//   }

  Point_3 gn1;
  if (neighbor(tg, ty, pg, ig, gn1, ints))
  {
    if (!xy_close(gn0, gn1, epsilon))
    {
      LOG4CPLUS_TRACE(logger, "Adding exit point: " << pp(gn1));
      ret.push_back(gn1);
    }
    else
    {
      LOG4CPLUS_TRACE(logger, "Entry and exit points are identical.  Returning single point " << pp(gn1));
    }
  }
  else
  {
    //            /
    //           /yseg0/ye0
    //      /\  /
    //     /  \/gn0
    //    /   /\
    //   /yn1/  \
    //  / gn1\   \
    // /      \   \
    // --------\---
    //       gn2\ 
    //           \yseg1/ye1
    //            \

    Segment_3_ yseg0;
    Segment_3 ye0;
    // If the intersection is a glancing blow with a vertex of the green
    // then return.
    Point_3 yn1;
    if (!neighbor_inside(tg, ty, pg, ig, yn1, yseg0, ints))
      return ret;

    ye0 = edge(index(yseg0, *ty), *ty);
    // There exists some yellow vertex (yn0) inside the green tile.
    Point_3 gn1 = get_new_point(tw_yellow, tw_green, tg, ty, yn1, yi, ints, z_adjustments);
    LOG4CPLUS_TRACE(logger, "Adding interior vertex point: " << pp(gn1) << ", yn1 = " << pp(yn1));
    if (xy_close(gn0, gn1, epsilon))
    {
      LOG4CPLUS_WARN(logger, "Exit point error: gn0 == gn1: skipping point");
      parameter_dump(logger, "gn0", pp(gn0), "gn1", pp(gn1),
		     "ye0", pp(ye0), "ty", pp_tri(*ty), "tg", pp_tri(*tg));
    }
    else
      ret.push_back(gn1);

    int ye1_idx = other_edge(yseg0, yn1, *ty);
    Segment_3 ye1 = edge(ye1_idx, *ty);
    LOG4CPLUS_TRACE(logger, "Examining ye1: " << pp(ye1) << " of tile " << pp_tri(*ty) << " index = " << ye1_idx);
    Point_3 gn2;
    if (find_edge_intersection(tg, ye1, yi, gn2, ints))
    {
      LOG4CPLUS_TRACE(logger, "Adding exit point: " << pp(gn2));
      if (gn1 == gn2)
      {
	LOG4CPLUS_WARN(logger, "Exit point error: gn1 == gn2: skipping point");
	parameter_dump(logger, "gn0", pp(gn0), "gn1", pp(gn1), "gn2", pp(gn2),
		       "ye0", pp(ye0), "ye1", pp(ye1), "ty", pp_tri(*ty), "tg", pp_tri(*tg));
      }
      else if (gn0 == gn2)
      {
	LOG4CPLUS_WARN(logger, "Exit point error: gn0 == gn2 -- returning polyline with single point");
	parameter_dump(logger, "gn0", pp(gn0), "gn1", pp(gn1), "gn2", pp(gn2),
		       "ye0", pp(ye0), "ye1", pp(ye1), "ty", pp_tri(*ty), "tg", pp_tri(*tg));
	return Polyline_2(gn2);
      }
      else
	ret.push_back(gn2);
    }
    else
    {
      //                /
      //               /yseg0/ye0
      //          /\  /
      //         /  \/gn0
      //        /   /\
      //       /yn1/  \
      //      / gn1\   \
      //     /      \   \
      //    /     ye1\   \
      //   /          \   \   ye2
      //  /         yn2\---\---------
      // /          gn2     \gn3
      ///--------------------\

      int yn2_idx = other_vertex(yn1, ye1, *ty);
      Point_3 yn2 = vertex(yn2_idx, *(ty));
      Point_3 gn2 = get_new_point(tw_yellow, tw_green, tg, ty, yn2, yi, ints, z_adjustments);
      LOG4CPLUS_TRACE(logger, "Adding second interior vertex point: " << pp(gn2));
      ret.push_back(gn2);

      if (xy_equal(gn2, (*tg)[0]) || xy_equal(gn2, (*tg)[1]) || xy_equal(gn2, (*tg)[2]))
	return ret;

      int ye2_idx = other_edge(ye1, yn2, *ty);
      Segment_3 ye2 = edge(ye2_idx, *ty);
      LOG4CPLUS_TRACE(logger, "Examining edge: " << pp(ye2) << " of tile " << pp_tri(*ty) << " index = " << ye2_idx);
      Point_3 gn3;
      if (!find_edge_intersection(tg, ye2, yi, gn3, ints))
      {
	LOG4CPLUS_ERROR(logger, "Found no exit point for " << pp(pg));
	parameter_dump(logger, "gn0", pp(gn0), "gn1", pp(gn1), "gn2", pp(gn2), "gn3", pp(gn3), 
		     "ye0", pp(ye0), "ye1", pp(ye1), "ye2", pp(ye2), "ty", pp_tri(*ty), "tg", pp_tri(*tg));
	gn3 = find_nearest_vertex(tg, ye2);
// 	throw logic_error("There should be an intersection");
      }
      LOG4CPLUS_TRACE(logger, "Adding exit point: " << pp(gn3));
      if (gn3 == gn1)
      {
	LOG4CPLUS_WARN(logger, "Exit point error: gn3 == gn1: failing");
	parameter_dump(logger, "gn0", pp(gn0), "gn1", pp(gn1), "gn2", pp(gn2), "gn3", pp(gn3), 
		     "ye0", pp(ye0), "ye1", pp(ye1), "ye2", pp(ye2), "ty", pp_tri(*ty), "tg", pp_tri(*tg));
	throw logic_error("Exit point error: gn3 == gn2 || gn3 == gn1");
      }
      else if (gn3 == gn2)
      {
	LOG4CPLUS_WARN(logger, "Exit point error: gn3 == gn2: skipping");
	parameter_dump(logger, "gn0", pp(gn0), "gn1", pp(gn1), "gn2", pp(gn2), "gn3", pp(gn3), 
		     "ye0", pp(ye0), "ye1", pp(ye1), "ye2", pp(ye2), "ty", pp_tri(*ty), "tg", pp_tri(*tg));
      }
      else if (gn0 == gn3)
      {
	// *********** THIS MAY NEED TO CHANGE BACK **************
	LOG4CPLUS_WARN(logger, "Exit point error: gn0 == gn3: NOT skipping");
	parameter_dump(logger, "gn0", pp(gn0), "gn1", pp(gn1), "gn2", pp(gn2), "gn3", pp(gn3), 
		       "ye0", pp(ye0), "ye1", pp(ye1), "ye2", pp(ye2), "ty", pp_tri(*ty), "tg", pp_tri(*tg));
	ret.push_back(gn3);
      }
      else
	ret.push_back(gn3);
    }
  }
  return ret;
}

//------------------------------------------------------------------------------
// trace_tile
//
/// Debug utility
//------------------------------------------------------------------------------
template <typename Tile_handle>
void trace_tile(Tile_handle t, string prefix)
{
  static log4cplus::Logger logger = log4cplus::Logger::getInstance("intersection");

  for (int i = 0; i < 3; ++i)
    LOG4CPLUS_TRACE(logger, prefix << i << ": " << pp(edge(i, *t)));
}

//------------------------------------------------------------------------------
// get_tile
//
/// Returns the tile that contains both points (on different edges)
//------------------------------------------------------------------------------
template <typename Tile_handle>
Tile_handle get_tile(const Intersections<Tile_handle>& intersections, const Point_3& a, const Point_3& b, int i)
{
  static log4cplus::Logger logger = log4cplus::Logger::getInstance("intersection");

  const list<Tile_handle>& tiles_a = intersections.tiles(a, i);
  const list<Tile_handle>& tiles_b = intersections.tiles(b, i);
  for (typename list<Tile_handle>::const_iterator it = tiles_a.begin(); it != tiles_a.end(); ++it)
  {
    if (find(tiles_b.begin(), tiles_b.end(), *it) != tiles_b.end())
      return *it;
  }

  LOG4CPLUS_WARN(logger, "Points don't share a tile: " << pp(a) << ", " << pp(b));
  for (typename list<Tile_handle>::const_iterator it = tiles_a.begin(); it != tiles_a.end(); ++it)
    trace_tile(*it, " a: ");
  for (typename list<Tile_handle>::const_iterator it = tiles_b.begin(); it != tiles_b.end(); ++it)
    trace_tile(*it, " b: ");
  throw logic_error("Points don't share a tile");
}

//------------------------------------------------------------------------------
// other_tile
//
/// Returns the tile touching p that isn't t.  Returns a null handle if
/// there is only one tile touching p and it is t.
//------------------------------------------------------------------------------
template <typename Tile_handle>
Tile_handle other_tile(const Intersections<Tile_handle>& intersections, const Segment_3_& e, int i, Tile_handle t)
{
  static log4cplus::Logger logger = log4cplus::Logger::getInstance("intersection");

//   LOG4CPLUS_TRACE(logger, "Searching for other tile: " << pp(p));
  const list<Tile_handle>& tiles = intersections.tiles(e, i);
  for (typename list<Tile_handle>::const_iterator it = tiles.begin(); it != tiles.end(); ++it)
    if (*it != t)
      return *it;
  return Tile_handle();
}

template <typename Tile_handle>
void induce_last(const Polyline_2& cut, int gi, Tile_handle tg, const Intersections<Tile_handle>& ints,
		 Z_adjustments<Tile_handle>& z_adjustments,
		 boost::unordered_map<Tile_handle, boost::unordered_set<Polyline_2> >& cuts)
{
  static log4cplus::Logger logger = log4cplus::Logger::getInstance("intersection.induce_adjacent");
  if (cut.size() > 1)
  {
    {
    // Find adjacent green tiles and induce the last cut point on them.
    list<Segment_3_> all(ints.edges_begin(cut.target(), gi), ints.edges_end(cut.target(), gi));
    for (list<Segment_3_>::iterator it = all.begin(); it != all.end(); ++it)
    {
      const Segment_3_& adj_e = *it;
      Tile_handle adj = other_tile(ints, adj_e, gi, tg);
      if (adj)
      {
	LOG4CPLUS_TRACE(logger, "Adjacent tile for " << pp(cut.target()) << ": " << pp_tri(*adj));
	cuts[adj].insert(Polyline_2(cut.target()));
	z_adjustments.add(adj);
      }
      else
	LOG4CPLUS_TRACE(logger, "No adjacent tile for " << pp(cut.target()));
    }
    }

    {
    // Find adjacent green tiles and induce the last cut point on them.
    list<Segment_3_> all(ints.edges_begin(cut.source(), gi), ints.edges_end(cut.source(), gi));
    for (list<Segment_3_>::iterator it = all.begin(); it != all.end(); ++it)
    {
      const Segment_3_& adj_e = *it;
      Tile_handle adj = other_tile(ints, adj_e, gi, tg);
      if (adj)
      {
	LOG4CPLUS_TRACE(logger, "Adjacent tile for " << pp(cut.source()) << ": " << pp_tri(*adj));
	cuts[adj].insert(Polyline_2(cut.source()));
	z_adjustments.add(adj);
      }
      else
	LOG4CPLUS_TRACE(logger, "No adjacent tile for " << pp(cut.source()));

    }
    }
  }
}

template <typename Tile_handle>
bool is_required(const Polyline_2& cut, int ci,
		       Z_adjustments<Tile_handle>& z_adjustments)
{
  for (Polyline_2::const_iterator it = cut.begin(); it != cut.end(); ++it)
  {
    if (z_adjustments.contains(*it, ci))
      return true;
  }
  return false;
}

template <typename Tile_handle>
bool contains(Tile_handle t, const Polyline_2& cut) {
  Polygon_2 P;
  P.push_back(vertex(0, *t));
  P.push_back(vertex(1, *t));
  P.push_back(vertex(2, *t));
  for (Polyline_2::const_iterator it = cut.begin(); it != cut.end(); ++it)
  {
    const Point_2& p = *it;
    if (P.has_on_negative_side(p))
      return false;
  }
  return true;
}

//------------------------------------------------------------------------------
// get_polyline_cuts
//
//------------------------------------------------------------------------------
template <typename Tile_handle>
void get_polyline_cuts(TW_handle tw_yellow, TW_handle tw_green, Intersections<Tile_handle>& ints, 
		       boost::unordered_map<Tile_handle, boost::unordered_set<Polyline_2> >& cuts,
		       Z_adjustments<Tile_handle>& z_adjustments)
{
  static log4cplus::Logger logger = log4cplus::Logger::getInstance("intersection.get_polyline_cuts");

  typedef typename boost::unordered_set<Tile_handle> Tile_set;
  typedef typename Tile_set::const_iterator Tile_set_iter;

  // Cache all the points since new ones will be added during the course of
  // this function
  boost::unordered_set<Point_3> int_pts;
  ints.all_intersections(inserter(int_pts, int_pts.end()));

  boost::unordered_set<Point_2> added_points;
  int iy = 0;
  int ig = 1;

  // Loop through all 3D intersections.
  int total = int_pts.size();
  int cur = 1;
  for (boost::unordered_set<Point_3>::iterator it = int_pts.begin(); it != int_pts.end(); ++it)
  {
    const Point_3& py = *it;
    const Point_3& pg = ints.first_opposite(py, iy);
    LOG4CPLUS_TRACE(logger, "Getting polyline cut for " << 
		    pp(py) << "(" << iy << ") and " << 
		    pp(pg) << "(" << ig << ")");
      
    LOG4CPLUS_TRACE(logger, "  Intersection check " << cur++ << " of " << total);
      
    boost::unordered_set<Tile_handle> ytiles, gtiles;
    ints.tiles(py, iy, inserter(ytiles, gtiles.end()));
    ints.tiles(pg, ig, inserter(gtiles, gtiles.end()));
    typedef typename boost::unordered_set<Tile_handle>::const_iterator tile_it;
    for (tile_it tg_it = gtiles.begin(); tg_it != gtiles.end(); ++tg_it) {
      Tile_handle tg = *tg_it;
      for (tile_it ty_it = ytiles.begin(); ty_it != ytiles.end(); ++ty_it) {
	Tile_handle ty = *ty_it;
	LOG4CPLUS_TRACE(logger, "  Yellow tile: " << pp_tri(*ty));
	LOG4CPLUS_TRACE(logger, "  Green tile: " << pp_tri(*tg));
	if (added_points.find(pg) == added_points.end()) {
	  // Get yellow's cut through the green tile
	  try {
	    Polyline_2 cut = find_exit(tw_yellow, tw_green, tg, ty, pg, ig, ints, z_adjustments);
	    LOG4CPLUS_TRACE(logger, "Found polyline cut (green tile) for " << pp(pg) << ": " << pp(cut));
	    LOG4CPLUS_TRACE(logger, "      tile = " << pp_tri(*tg));
	    if (is_required(cut, ig, z_adjustments)) {
// 	      if (!contains(tg, cut)) {
// 		LOG4CPLUS_ERROR(logger, "  Cut not contained in tile (" << cut.size() << ")");
// 		LOG4CPLUS_ERROR(logger, "    Cut = " << pp(cut));
// 		LOG4CPLUS_ERROR(logger, "    tg = " << pp_tri(*tg));
// 		LOG4CPLUS_ERROR(logger, "    ty = " << pp_tri(*ty));
// 		LOG4CPLUS_ERROR(logger, "    pg = " << pp(pg));
// 	      }
	      LOG4CPLUS_TRACE(logger, "Inserting polyline cut (green tile) for " << pp(pg) << ": " << pp(cut));
// 	      for (Polyline_2::iterator cut_iter = cut.begin(); cut_iter != cut.end(); ++cut_iter) {
// 		if (!cut_iter->is_valid()) {
// 		  cut_iter->id() = tw_green->vertices.unique_id();
// 		}
// 	      }
// 	      tw_green->ensure_z_home(cut, ints.z_home(cut[0], ig));
	      for (Polyline_2::iterator cit = cut.begin(); cit != cut.end(); ++cit) {
		if (!cit->is_valid()) {
		  LOG4CPLUS_ERROR(logger, "Cut point: no valid id(): " << pp(*cit));
		}
	      }
	      LOG4CPLUS_TRACE(logger, "Green unique " << tw_green->vertices.unique_id());
	      cuts[tg].insert(cut);
	      induce_last(cut, ig, tg, ints, z_adjustments, cuts);
	    }
	    else {
	      LOG4CPLUS_TRACE(logger, "  Polyline cut is not required");
	    }
	  } catch (std::logic_error& e)
	  { LOG4CPLUS_ERROR(logger, e.what());}
	}
	if (added_points.find(py) == added_points.end()) {
	  // Get green's cut through the yellow tile
	  try {
	    Polyline_2 cut = find_exit(tw_yellow, tw_green, ty, tg, py, iy, ints, z_adjustments);
	    LOG4CPLUS_TRACE(logger, "Found polyline cut (yellow tile) for " << pp(py) << ": " << pp(cut));
	    LOG4CPLUS_TRACE(logger, "      tile = " << pp_tri(*ty));
	    if (is_required(cut, iy, z_adjustments)) {
// 	      if (!contains(ty, cut)) {
// 		LOG4CPLUS_ERROR(logger, "  Cut not contained in tile (" << cut.size() << ")");
// 		LOG4CPLUS_ERROR(logger, "    Cut = " << pp(cut));
// 		LOG4CPLUS_ERROR(logger, "    ty = " << pp_tri(*ty));
// 		LOG4CPLUS_ERROR(logger, "    tg = " << pp_tri(*tg));
// 		LOG4CPLUS_ERROR(logger, "    py = " << pp(py));
// 	      }
	      LOG4CPLUS_TRACE(logger, "Inserting polyline cut (yellow tile) for " << pp(py) << ": " << pp(cut));
// 	      for (Polyline_2::iterator cut_iter = cut.begin(); cut_iter != cut.end(); ++cut_iter) {
// 		if (!cut_iter->is_valid()) {
// 		  cut_iter->id() = tw_yellow->vertices.unique_id();
// 		}
// 	      }
// 	      tw_yellow->ensure_z_home(cut, ints.z_home(cut[0], iy));
	      for (Polyline_2::iterator cit = cut.begin(); cit != cut.end(); ++cit) {
		if (!cit->is_valid()) {
		  LOG4CPLUS_ERROR(logger, "Cut point: no valid id(): " << pp(*cit));
		}
	      }
	      LOG4CPLUS_TRACE(logger, "Yellow unique " << tw_yellow->vertices.unique_id());
	      cuts[ty].insert(cut);
	      induce_last(cut, iy, ty, ints, z_adjustments, cuts);
	    }
	    else {
	      LOG4CPLUS_TRACE(logger, "  Polyline cut is not required");
	    }
	  } catch (std::logic_error& e)
	  { LOG4CPLUS_ERROR(logger, e.what());}
	}
      }
    }
  }
}

template <typename Tile_handle, typename Cuts_iter>
boost::unordered_map<Point_3, boost::unordered_set<Segment_3_undirected> > 
map_point2edges(Tile_handle tile,
		Cuts_iter cuts_begin, Cuts_iter cuts_end,
		const Intersections<Tile_handle>& ints, int i)
{
  boost::unordered_map<Point_3, boost::unordered_set<Segment_3_undirected> > point2edges;
  // First get all the induced points from the cuts and make a polygon out of
  // the tile
  for (Cuts_iter it = cuts_begin; it != cuts_end; ++it)
  {
    const Polyline_2& cut = *it;
    for (Polyline_2::const_iterator p_it = cut.begin(); p_it != cut.end(); ++p_it)
    {
      const Point_2& p = *p_it;
      if (ints.has_point(p, i))
	point2edges[p].insert(ints.edges_begin(p, i), ints.edges_end(p, i));
    }
  }

  for (int i = 0; i < 3; ++i)
  {
    int prev = prev_idx(i);
    point2edges[(*tile)[i]].insert(Segment_3_(edge(i, *tile)));
    point2edges[(*tile)[i]].insert(Segment_3_(edge(prev, *tile)));
  }

  return point2edges;
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// *** Implementations ***
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
typedef boost::shared_ptr<Triangle> Tile_handle_int;

template
bool neighbor(Tile_handle_int green, Tile_handle_int yellow, const Point_3& gp, int g, Point_3& gn, const Intersections<Tile_handle_int>& intersections);

template
Polyline_2 find_exit(TW_handle tw_yellow, TW_handle green, Tile_handle_int gt, Tile_handle_int yt, const Point_3& gp, int gi, 
		     const Intersections<Tile_handle_int>& ints,
		     Z_adjustments<Tile_handle_int>& z_adjustments);

template
void get_polyline_cuts(TW_handle tw_yellow, TW_handle green, Intersections<Tile_handle_int>& intersections, 
		       boost::unordered_map<Tile_handle_int, boost::unordered_set<Polyline_2> >& cuts,
		       Z_adjustments<Tile_handle_int>& z_adjustments);

template Intersections<Tile_handle_int>
get_intersections(TW_handle tw_yellow, TW_handle tw_green,
		  std::list<Tile_handle_int>::iterator ybegin, std::list<Tile_handle_int>::iterator yend,
		   std::list<Tile_handle_int>::iterator gbegin, std::list<Tile_handle_int>::iterator gend,
		  Z_adjustments<Tile_handle_int>& z_adjustments);

template Intersections<Tile_handle_int>
get_intersections(TW_handle tw_yellow, TW_handle tw_green,
		  std::vector<Tile_handle_int>::iterator ybegin, std::vector<Tile_handle_int>::iterator yend,
		   std::vector<Tile_handle_int>::iterator gbegin, std::vector<Tile_handle_int>::iterator gend,
		  Z_adjustments<Tile_handle_int>& z_adjustments);

template
boost::unordered_map<Point_3, boost::unordered_set<Segment_3_undirected> > 
map_point2edges(Tile_handle_int tile,
		list<Polyline_2>::iterator cuts_begin, list<Polyline_2>::iterator cuts_end,
		const Intersections<Tile_handle_int>& ints, int i);

typedef boost::shared_ptr<Triangle> Triangle_handle;
typedef list<Triangle_handle>::const_iterator clti;
typedef list<Triangle_handle>::iterator lti;
typedef vector<Triangle_handle>::const_iterator cvti;
typedef vector<Triangle_handle>::iterator vti;
typedef std::back_insert_iterator<std::vector<Triangle_handle> > bii;

// template
// void remove_intersections(TW_handle tw_yellow, TW_handle tw_green,
// 			  Number_type yz, Number_type gz,
// 			  bii new_yellow, bii new_green, Number_type epsilon);

template
void remove_intersections(TW_handle tw_yellow, TW_handle tw_green,
			  lti yellow_begin, lti yellow_end,
			  lti green_begin, lti green_end,
			  Number_type yz, Number_type gz,
			  bii new_yellow, bii new_green, Number_type epsilon,
			  boost::unordered_map<Segment_3_undirected, list<Point_3> >& edge2points);

template
void remove_intersections(TW_handle tw_yellow, TW_handle tw_green,
			  clti yellow_begin, clti yellow_end,
			  clti green_begin, clti green_end,
			  Number_type yz, Number_type gz,
			  bii new_yellow, bii new_green, Number_type epsilon,
			  boost::unordered_map<Segment_3_undirected, list<Point_3> >& edge2points);


CONTOURTILER_END_NAMESPACE
