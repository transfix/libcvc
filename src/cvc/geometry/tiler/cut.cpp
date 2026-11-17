#include <ContourTiler/cut.h>
#include <ContourTiler/triangle_utils.h>
#include <ContourTiler/print_utils.h>
#include <ContourTiler/segment_utils.h>
#include <ContourTiler/Distance_functor.h>

CONTOURTILER_BEGIN_NAMESPACE

using namespace std;

//------------------------------------------------------------------------------
// make_poly
//
/// Makes a polygon out of a triangle along with extra points lying on the 
/// triangle segments.
//------------------------------------------------------------------------------
Polygon_2 make_poly(const Triangle& triangle, const boost::unordered_map<Segment_3_, boost::unordered_set<Point_3> >& seg2points)
{
  log4cplus::Logger cut_logger = log4cplus::Logger::getInstance("intersection.cut.make_poly");

  Polygon_2 poly;

  for (int i = 0; i < 3; ++i)
  {
    const Point_3& s = vertex(i, triangle);
    const Point_3& t = vertex(i+1, triangle);
    Segment_3_ seg(s, t);

    vector<Point_3> points;
    points.push_back(s);

    boost::unordered_map<Segment_3_, boost::unordered_set<Point_3> >::const_iterator it = seg2points.find(seg);
    if (it != seg2points.end())
    {
      boost::unordered_set<Point_3> seg_pts = it->second;
      LOG4CPLUS_TRACE(cut_logger, "  Augmenting tile with points.  Segment = " << pp(seg.segment()));
      seg_pts.erase(seg.segment().source());
      seg_pts.erase(seg.segment().target());
      for (boost::unordered_set<Point_3>::iterator it = seg_pts.begin(); it != seg_pts.end(); ++it)
	LOG4CPLUS_TRACE(cut_logger, "    Point: " << pp(*it));
      points.insert(points.end(), seg_pts.begin(), seg_pts.end());
    }

    sort(points.begin(), points.end(), dist_functor(s));
    poly.insert(poly.vertices_end(), points.begin(), points.end());
  }
  return poly;
}

//------------------------------------------------------------------------------
// make_poly
//
/// Makes a polygon out of a triangle along with extra points lying on the 
/// triangle segments.
//------------------------------------------------------------------------------
Polygon_2 make_poly(const Triangle& triangle, const boost::unordered_map<Point_3, Segment_3>& pt2seg)
{
  boost::unordered_map<Segment_3_, boost::unordered_set<Point_3> > seg2points;
  boost::unordered_map<Point_3, Segment_3>::const_iterator it = pt2seg.begin();
  for (; it != pt2seg.end(); ++it)
    seg2points[Segment_3_(it->second)].insert(it->first);
  return make_poly(triangle, seg2points);
}

//------------------------------------------------------------------------------
// make_poly
//
/// Makes a polygon out of a triangle along with extra points lying on the 
/// triangle segments.
//------------------------------------------------------------------------------
Polygon_2 make_poly(const Triangle& triangle, const boost::unordered_map<Point_3, Segment_3_>& pt2seg)
{
  boost::unordered_map<Segment_3_, boost::unordered_set<Point_3> > seg2points;
  boost::unordered_map<Point_3, Segment_3_>::const_iterator it = pt2seg.begin();
  for (; it != pt2seg.end(); ++it)
    seg2points[it->second].insert(it->first);
  return make_poly(triangle, seg2points);
}

//------------------------------------------------------------------------------
// cut_polygon_with_line
//
/// Cuts a polygon with a segment and returns the resulting two polygons
/// Precondition: both endpoints of cut are vertices in the polygon
/// Precondition: at least one non-collinear point in the polygon lies between
///               the endpoints of cut
//------------------------------------------------------------------------------
pair<Polygon_2, Polygon_2> cut_polygon_with_line(const Polygon_2& p, const Segment_3& cut)
{
  typedef Polygon_2::Vertex_iterator Iter;

  const Point_3& s = cut.source();
  const Point_3& t = cut.target();

  Polygon_2 ret0, ret1;

  Iter source = find(p.vertices_begin(), p.vertices_end(), s);
  Iter target = find(p.vertices_begin(), p.vertices_end(), t);
  if (find(p.vertices_begin(), source, *target) != source)
    swap(source, target);

  for (Iter it = p.vertices_begin(); it != source; ++it)
    ret0.push_back(*it);
  ret0.push_back(*source);
  for (Iter it = target; it != p.vertices_end(); ++it)
    ret0.push_back(*it);

  for (Iter it = source; it != target; ++it)
    ret1.push_back(*it);
  ret1.push_back(*target);

  return make_pair(ret0, ret1);
}

//------------------------------------------------------------------------------
// trim_forward
//
//------------------------------------------------------------------------------
Polyline_2 trim_forward(const Polygon_2& p, const Polyline_2& c)
{
  typedef Polygon_2::Vertex_const_iterator Polygon_iter;
  typedef Polyline_2::const_iterator Polyline_2_iter;

  Polyline_2_iter first = c.begin();
  Polyline_2_iter last = c.end() - 1;
  Polygon_iter poly_first = find(p.vertices_begin(), p.vertices_end(), *first);
  Polygon_iter poly_last = find(p.vertices_begin(), p.vertices_end(), *last);

  while (*next(poly_first, p) == *(first+1))
  {
    poly_first = next(poly_first, p);
    ++first;
    if (first == last)
      return Polyline_2();
  }
  while (*prev(poly_last, p) == *(last-1))
  {
    poly_last = prev(poly_last, p);
    --last;
  }

  Polyline_2 ret(first, last);
  ret.push_back(*last);

  return ret;
}

//------------------------------------------------------------------------------
// trim_backward
//
//------------------------------------------------------------------------------
Polyline_2 trim_backward(const Polygon_2& p, const Polyline_2& c)
{
  typedef Polygon_2::Vertex_const_iterator Polygon_iter;
  typedef Polyline_2::const_iterator Polyline_2_iter;

  Polyline_2_iter first = c.begin();
  Polyline_2_iter last = c.end() - 1;
  Polygon_iter poly_first = find(p.vertices_begin(), p.vertices_end(), *first);
  Polygon_iter poly_last = find(p.vertices_begin(), p.vertices_end(), *last);

  while (*prev(poly_first, p) == *(first+1))
  {
    poly_first = prev(poly_first, p);
    ++first;
    if (first == last)
      return Polyline_2();
  }
  while (*next(poly_last, p) == *(last-1))
  {
    poly_last = next(poly_last, p);
    --last;
  }

  Polyline_2 ret(first, last);
  ret.push_back(*last);

  return ret;
}

bool adjacent(const Polygon_2& P, const Point_2& a, const Point_2& b)
{
  typedef Polygon_2::Vertex_const_iterator Polygon_iter;
  Polygon_iter ai = find(P.vertices_begin(), P.vertices_end(), a);
  Polygon_iter bi = find(P.vertices_begin(), P.vertices_end(), b);
  return ai == next(bi, P) ||
    bi == next(ai, P);
}

template <typename Polyline_iter>
Polyline_iter find_last_adj(Polyline_iter begin, Polyline_iter end, const Polygon_2& p)
{
  Polyline_iter it = begin;
  Polyline_iter next = it + 1;
  while (next != end && adjacent(p, *it, *next))
  {
    ++it;
    ++next;
  }
  return it;
}

template <typename Polyline_iter>
Polyline_iter find_first_on(Polyline_iter begin, Polyline_iter end, const Polygon_2& p)
{
  Polyline_iter it = begin;
  while (it != end && find(p.vertices_begin(), p.vertices_end(), *it) == p.vertices_end())
    ++it;
  return it;
}

template <typename Out_iter>
void/*Polyline_2*/ trim(const Polygon_2& p, const Polyline_2& c, Out_iter cuts)
{
  log4cplus::Logger cut_logger = log4cplus::Logger::getInstance("intersection.cut.trim");

  typedef Polygon_2::Vertex_const_iterator Polygon_iter;
  typedef Polyline_2::const_iterator Polyline_2_iter;
  
  Polyline_2_iter beg = find_last_adj(c.begin(), c.end(), p);
  while (beg != c.end())
  {
    Polyline_2_iter end = find_first_on(beg+1, c.end(), p);
    if (end != c.end())
    {
//     if (end == c.end())
//       throw logic_error("No other point in the cut is on the polygon");
      ++end;
//     if (end != c.end())
//       throw logic_error("Cut has intermediate adjacencies");

      *cuts++ = Polyline_2(beg, end);
      LOG4CPLUS_TRACE(cut_logger, "trim adding cut: " << pp(Polyline_2(beg, end)));
    }
    beg = find_last_adj(end, c.end(), p);
  }

//   for (; it != c.end(); ++it)
//   {
//     Polyline_2_iter next = it + 1;
//     if (next == c.end() || !adjacent(p, *it, *next))
//       break;
//   }
//   c = Polyline_2(it, c.end());

//   c = Polyline_2(c.rbegin(), c.rend());
//   it = c.begin();
//   for (; it != c.end(); ++it)
//   {
//     Polyline_2_iter next = it + 1;
//     if (next == c.end() || !adjacent(p, *it, *next))
//       break;
//   }
//   c = Polyline_2(it, c.end());

//   c = Polyline_2(c.rbegin(), c.rend());
//   return c;
}


// If target appears before source when iterating through polygon
// vertices, then switch direction of cut.
Polyline_2 order(const Polygon_2& p, Polyline_2 cut)
{
  log4cplus::Logger cut_logger = log4cplus::Logger::getInstance("intersection.cut.order");

  typedef Polygon_2::Vertex_iterator Iter;
  Iter source = find(p.vertices_begin(), p.vertices_end(), cut.source());
  Iter target = find(p.vertices_begin(), p.vertices_end(), cut.target());

  // If target appears before source when iterating through polygon
  // vertices, then switch direction of cut.
  // 
  // Post-condition: Cut source appears before cut target when iterating
  // through polygon vertices.
  if (find(p.vertices_begin(), source, *target) != source)
  {
    cut = Polyline_2(cut.rbegin(), cut.rend());
    swap(source, target);
    LOG4CPLUS_TRACE(cut_logger, "Swapping source and target");
  }

  return cut;
}

//------------------------------------------------------------------------------
// cut_polygon_with_polyline
//
/// Cuts a polygon with a polyline and returns the resulting two polygons
/// Precondition: both endpoints of cut are vertices in the polygon
/// Precondition: at least one non-collinear point in the polygon lies between
///               the endpoints of cut
//------------------------------------------------------------------------------
pair<Polygon_2, Polygon_2> cut_polygon_with_polyline(const Polygon_2& p, const Polyline_2& c)
{
  log4cplus::Logger cut_logger = log4cplus::Logger::getInstance("intersection.cut.cut_polygon_with_polyline");

  LOG4CPLUS_TRACE(cut_logger, "Cutting polygon " << pp(p) << " with cut " << pp(c));

  typedef Polygon_2::Vertex_iterator Iter;
  typedef Polygon_2::Vertex_circulator Circulator;
  
  Polygon_2 ret0, ret1;
  Polyline_2 cut = order(p, c);//trim(p, order(p, c));
  
  list<Polyline_2> trimmed;
  trim(p, cut, back_inserter(trimmed));
//   if (cut != *trimmed.begin())
//     throw logic_error("Cut has not been trimmed");

  LOG4CPLUS_TRACE(cut_logger, "After trimming: " << pp(cut));

  if (cut.size() > 1)
  {
    Iter source = find(p.vertices_begin(), p.vertices_end(), cut.source());
    Iter target = find(p.vertices_begin(), p.vertices_end(), cut.target());

    for (Iter it = next(target, p); it != source; it = next(it, p))
      ret0.push_back(*it);
    ret0.insert(ret0.vertices_end(), cut.begin(), cut.end());

    for (Iter it = next(source, p); it != target; it = next(it, p))
      ret1.push_back(*it);
    ret1.insert(ret1.vertices_end(), cut.rbegin(), cut.rend());
  }

  LOG4CPLUS_TRACE(cut_logger, "  Results: ");
  LOG4CPLUS_TRACE(cut_logger, "    " << pp(ret0));
  LOG4CPLUS_TRACE(cut_logger, "    " << pp(ret1));

  return make_pair(ret0, ret1);
}

// See intersection30 test in lab book page 45
template <typename Out_iter>
void simplify(const Polygon_2& p, Out_iter sub_polygons)
{
  for (Polygon_2::Vertex_const_iterator it = p.vertices_begin(); it != p.vertices_end(); ++it)
  {
    Polygon_2::Vertex_const_iterator duplicate = find(it+1, p.vertices_end(), *it);
    if (duplicate != p.vertices_end())
    {
      Polygon_2 sub(p.vertices_begin(), it);
      sub.insert(sub.vertices_end(), duplicate, p.vertices_end());
      if (sub.size() > 2)
      {
	*sub_polygons++ = sub;
	if (!sub.is_simple()) throw logic_error("sub is not simple");
      }

      Polygon_2::Vertex_const_iterator next = it;
      ++next;
      Polygon_2::Vertex_const_iterator next_duplicate = find(next+1, p.vertices_end(), *next);
      if (next_duplicate != p.vertices_end())
      {
	duplicate = next_duplicate;
	it = next;
      }

      sub = Polygon_2(it, duplicate);
      if (sub.size() > 2)
      {
	*sub_polygons++ = sub;
	if (!sub.is_simple()) throw logic_error("sub is not simple");
      }

    }
  }
}

//------------------------------------------------------------------------------
// cut_polygon_with_polylines
//
/// Cuts a polygon into multiple polygons with the given cuts
/// Precondition: no two cuts intersect
/// Precondition: see preconditions to other cut function
//------------------------------------------------------------------------------
template <typename Polyline_iter, typename Polygon_iter>
void cut_polygon_with_polylines(const Polygon_2& p, Polyline_iter cuts_begin, Polyline_iter cuts_end, Polygon_iter polys)
{
  log4cplus::Logger cut_logger = log4cplus::Logger::getInstance("intersection.cut.cut_polygon_with_polylines");

  LOG4CPLUS_TRACE(cut_logger, "Cutting " << pp(p) << " into polygons");
  
  list<Polygon_2> sum;
  sum.push_back(p);

  list<Polyline_2> cuts(cuts_begin, cuts_end);
  while (!cuts.empty())
  {
    Polyline_2 c = cuts.front();
    cuts.pop_front();
    LOG4CPLUS_TRACE(cut_logger, "  Cut: " << pp(c));
    if (c.size() > 1)
    {
      for (list<Polygon_2>::iterator p_it = sum.begin(); p_it != sum.end(); ++p_it)
      {
	const Polygon_2& polygon = *p_it;
	LOG4CPLUS_TRACE(cut_logger, "  Polygon: " << pp(polygon));
	if (find(p_it->vertices_begin(), p_it->vertices_end(), c.source()) != p_it->vertices_end() &&
	    find(p_it->vertices_begin(), p_it->vertices_end(), c.target()) != p_it->vertices_end())
	{
	  vector<Polyline_2> subcuts;
	  trim(polygon, c, back_inserter(subcuts));
	  LOG4CPLUS_TRACE(cut_logger, "Number of cuts returned from trim(): " << subcuts.size());
	  for (int i = 1; i < subcuts.size(); ++i)
	  {
	    cuts.push_back(subcuts[i]);
	    LOG4CPLUS_TRACE(cut_logger, "Adding subcut: " << pp(subcuts[i]));
	  }

	  if (subcuts.size() > 0)
	  {
	    const Polyline_2& cut = subcuts[0];
	    Polygon_2 a, b;
	    boost::tie(a, b) = cut_polygon_with_polyline(polygon, cut);
// 	    if (a.size() > 2)
	    {
	      if (a.size() < 3 || b.size() < 3)// || !a.is_simple() || !b.is_simple())
		throw logic_error("Error in cutting polygons");
	      sum.erase(p_it);
	      sum.push_back(a);
	      sum.push_back(b);
	    }
	    break;
	  }
	}
      }
    }
  }

  for (list<Polygon_2>::iterator p_it = sum.begin(); p_it != sum.end(); ++p_it)
  {
    *polys = *p_it;
    ++polys;
  }
}

bool xy_close(const Point_3& p1, const Point_3& p2)
{
  return CGAL::abs(p1.x()-p2.x()) < 0.000001 && CGAL::abs(p1.y()-p2.y()) < 0.000001;
    
}

bool xyz_close(const Point_3& p1, const Point_3& p2)
{
  return CGAL::abs(p1.x()-p2.x()) < 0.000001 && 
    CGAL::abs(p1.y()-p2.y()) < 0.000001 &&
    CGAL::abs(p1.z()-p2.z()) < 0.000001;
    
}

bool xy_close(const Point_3& p, const Triangle& t)
{
  for (int i = 0; i < 3; ++i)
  {
    if (xy_close(p, t[i]))
      return true;
  }
  return false;
}

bool xyz_close(const Point_3& p, const Triangle& t)
{
  for (int i = 0; i < 3; ++i)
  {
    if (xyz_close(p, t[i]))
      return true;
  }
  return false;
}

//------------------------------------------------------------------------------
// cut_tile_with_polylines
//
/// Induces points onto the tile to generate a polygon and then uses the given
/// cuts to cut the polygon into multiple sub-polygons and then triangulates
/// from there.
//------------------------------------------------------------------------------
template <typename Tile_handle, typename Cuts_iter, typename Poly_iter>
void cut_tile_with_polylines(Tile_handle tile,
		       Cuts_iter cuts_begin, Cuts_iter cuts_end,
		       Poly_iter new_polys,
		       const Intersections<Tile_handle>& ints, int i,
		       boost::unordered_map<Point_3, boost::unordered_set<Segment_3_undirected> >& point2edges)
{
  log4cplus::Logger cut_logger = log4cplus::Logger::getInstance("intersection.cut.cut_tile_with_polylines");

  LOG4CPLUS_TRACE(cut_logger, "Cutting tile: " << pp_tri(*tile));

  // First get all the induced points from the cuts and make a polygon out of
  // the tile
  boost::unordered_map<Point_3, Segment_3_> pt2seg;
  for (Cuts_iter it = cuts_begin; it != cuts_end; ++it)
  {
    const Polyline_2& cut = *it;

    LOG4CPLUS_TRACE(cut_logger, "Updating polygon with cut = " << pp(cut));

    for (Polyline_2::const_iterator p_it = cut.begin(); p_it != cut.end(); ++p_it)
    {
      const Point_2& p = *p_it;
      if (ints.has_point(p, i) && !xyz_close(p, *tile))
      {
	LOG4CPLUS_TRACE(cut_logger, "  Adding point " << pp(p));
	pt2seg[p] = *ints.edges_begin(p, i);
	point2edges[p].insert(ints.edges_begin(p, i), ints.edges_end(p, i));
      }
    }
  }

  for (int i = 0; i < 3; ++i)
  {
    int prev = prev_idx(i);
    point2edges[(*tile)[i]].insert(Segment_3_(edge(i, *tile)));
    point2edges[(*tile)[i]].insert(Segment_3_(edge(prev, *tile)));
  }

  Polygon_2 poly = make_poly(*tile, pt2seg);
  cut_polygon_with_polylines(poly, cuts_begin, cuts_end, new_polys);
}

//------------------------------------------------------------------------------
// cut_into_triangles
//
/// Cuts a polygon into multiple triangles with the given cuts
/// Precondition: no two cuts intersect
/// Precondition: see preconditions to other cut function
//------------------------------------------------------------------------------
template <typename Polyline_iter, typename Triangle_iter>
void cut_into_triangles(const Polygon_2& p, Polyline_iter cuts_begin, Polyline_iter cuts_end, Triangle_iter triangles)
{
  log4cplus::Logger cut_logger = log4cplus::Logger::getInstance("intersection.cut.cut_into_triangles");

  LOG4CPLUS_TRACE(cut_logger, "Cutting " << pp(p) << " into triangles");

  list<Polygon_2> polygons;
  cut_polygon_with_polylines(p, cuts_begin, cuts_end, back_inserter(polygons));
  
  boost::unordered_map<Point_3, boost::unordered_set<Segment_3_undirected> > map;
  for (list<Polygon_2>::iterator it = polygons.begin(); it != polygons.end(); ++it) {
    try {
      triangulate_safe(*it, triangles, map);
    }
    catch (logic_error& e) {
      LOG4CPLUS_WARN(cut_logger, "Failed to cut " << pp(p) << " into triangles: " << e.what());
    }
  }
}

//------------------------------------------------------------------------------
// cut_into_triangles
//
/// Induces points onto the tile to generate a polygon and then uses the given
/// cuts to cut the polygon into multiple sub-polygons and then triangulates
/// from there.
//------------------------------------------------------------------------------
template <typename Tile_handle, typename Cuts_iter, typename Tiles_iter>
void cut_into_triangles(Tile_handle tile,
	 Cuts_iter cuts_begin, Cuts_iter cuts_end,
	 Tiles_iter new_tiles,
	 const Intersections<Tile_handle>& ints, int i)
{
  // First get all the induced points from the cuts and make a polygon out of
  // the tile
  boost::unordered_map<Point_3, Segment_3_> pt2seg;
  for (Cuts_iter it = cuts_begin; it != cuts_end; ++it)
  {
    const Polyline_2& cut = *it;
    pt2seg[cut.source()] = *ints.edges_begin(cut.source(), i);
    if (cut.size() > 1)
      pt2seg[cut.target()] = *ints.edges_begin(cut.target(), i);
  }
  Polygon_2 poly = make_poly(*tile, pt2seg);
  vector<Triangle> triangles;
  cut_into_triangles(poly, cuts_begin, cuts_end, back_inserter(triangles));

  for (vector<Triangle>::iterator it = triangles.begin(); it != triangles.end(); ++it)
  {
    *new_tiles = Tile_handle(new Triangle(*it));
    ++new_tiles;
  }
}

std::pair<Triangle, Triangle> decompose_triangle(const Triangle& triangle, 
						 const Segment_3& edge,
						 const Point_3& point)
{
  log4cplus::Logger cut_logger = log4cplus::Logger::getInstance("intersection.cut.decompose_triangle");
  LOG4CPLUS_TRACE(cut_logger, "Decomposing " << pp_tri(triangle) << " into triangles");
  int e = index(edge, triangle);
  int i0 = e;
  int i1 = next_idx(i0);
  int i2 = next_idx(i1);
  const Point_3& v0 = vertex(i0, triangle);
  const Point_3& v1 = vertex(i1, triangle);
  const Point_3& v2 = vertex(i2, triangle);
  Triangle t1(v0, point, v2);
  Triangle t2(point, v1, v2);
  return make_pair(t1, t2);
}

template <typename Out_iter>
void decompose_triangle(const Triangle& triangle, 
			const boost::unordered_map<Segment_3_undirected, list<Point_3> >& edge2points,
			Out_iter output)
{
  log4cplus::Logger cut_logger = log4cplus::Logger::getInstance("intersection.cut.decompose_triangle");
  LOG4CPLUS_TRACE(cut_logger, "Decomposing " << pp_tri(triangle) << " into triangles");

  int edge_idx = -1;
  for (int i = 0; i < 3; ++i) {
    Segment_3_undirected e(edge(i, triangle));
    if (edge2points.find(e) != edge2points.end()) {
      if (edge_idx > -1) {
	throw logic_error("Triangle unexpectedly has decomposing points on two edges");
      }
      edge_idx = i;
    }
  }

  if (edge_idx == -1) return;

  Segment_3_undirected e(edge(edge_idx, triangle));
  int i0 = edge_idx;
  int i1 = next_idx(i0);
  int i2 = next_idx(i1);
  const Point_3& v0 = vertex(i0, triangle);
  const Point_3& v1 = vertex(i1, triangle);
  const Point_3& v2 = vertex(i2, triangle);

  typedef list<Point_3>::iterator Iter;

  list<Point_3> points = edge2points.find(e)->second;
  points.sort(Distance_functor<Point_3>(vertex(i0, triangle)));
  points.push_back(v1);

  Iter end = remove(points.begin(), points.end(), v0);
  end = unique(points.begin(), end);

  Point_3 cur = v0;
  for (Iter it = points.begin(); it != end; ++it) {
    Triangle t(cur, *it, v2);
    *output++ = t;
    cur = *it;
  }
}

template
void decompose_triangle(const Triangle& triangle, 
			const boost::unordered_map<Segment_3_undirected, list<Point_3> >& edge2points,
			back_insert_iterator<list<Triangle> > output);

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// *** Implementations ***
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
typedef boost::shared_ptr<Triangle> Tile_handle_int;

template
void cut_polygon_with_polylines(const Polygon_2& p, 
		       boost::unordered_set<Polyline_2>::const_iterator cuts_begin, boost::unordered_set<Polyline_2>::const_iterator cuts_end, 
		       back_insert_iterator<list<Polygon_2> > polys);

// template
// void cut_polygon_with_polylines(const Polygon_2& p, 
// 		       boost::unordered_set<Polyline_2>::iterator cuts_begin, boost::unordered_set<Polyline_2>::iterator cuts_end, 
// 		       back_insert_iterator<list<Polygon_2> > polys);

template
void cut_polygon_with_polylines(const Polygon_2& p, 
		       boost::unordered_set<Polyline_2>::iterator cuts_begin, boost::unordered_set<Polyline_2>::iterator cuts_end, 
		       back_insert_iterator<vector<Polygon_2> > polys);

template
void cut_polygon_with_polylines(const Polygon_2& p, 
		       Polyline_2* cuts_begin, Polyline_2* cuts_end, 
		       back_insert_iterator<list<Polygon_2> > polys);

template
void cut_tile_with_polylines(Tile_handle_int tile,
		       boost::unordered_set<Polyline_2>::iterator cuts_begin, boost::unordered_set<Polyline_2>::iterator cuts_end,
		       back_insert_iterator<list<Polygon_2> > new_polys,
		       const Intersections<Tile_handle_int>& ints, int i,
		       boost::unordered_map<Point_3, boost::unordered_set<Segment_3_undirected> >& point2edges);

template
void cut_into_triangles(const Polygon_2& p, 
			list<Polyline_2>::iterator cuts_begin, list<Polyline_2>::iterator cuts_end, 
			back_insert_iterator<list<Triangle> > triangles);

template
void cut_into_triangles(const Polygon_2& p, 
			list<Polyline_2>::iterator cuts_begin, list<Polyline_2>::iterator cuts_end, 
			back_insert_iterator<vector<Triangle> > triangles);

template
void cut_into_triangles(Tile_handle_int tile,
			list<Polyline_2>::iterator cuts_begin, list<Polyline_2>::iterator cuts_end,
			back_insert_iterator<list<Tile_handle_int> > new_tiles,
			const Intersections<Tile_handle_int>& ints, int i);
CONTOURTILER_END_NAMESPACE
