#include <ContourTiler/theorems.h>
#include <ContourTiler/projection.h>
#include <ContourTiler/intersection/Segment_3_Segment_3.h>
#include <ContourTiler/print_utils.h>
#include <ContourTiler/sweep_line_visitors.h>

CONTOURTILER_BEGIN_NAMESPACE

bool test_all(const Segment_3& chord, const Tiler_workspace& w)
{
  bool t2 = true;//test_theorem2(chord, w);
  bool t5 = test_theorem5(chord, w.bscs);
//   bool t8 = test_theorem8(chord, w);
  return t2 && t5;
}

bool test_theorem2(const Segment_3& chord, const Tiler_workspace& w)
{
  log4cplus::Logger logger = log4cplus::Logger::getInstance("tiler.theorems.2");
  LOG4CPLUS_TRACE(logger, pp_id(chord));

  Point_3 v0 = chord.source();
  Point_3 v1 = chord.target();
//   Segment_2 chord_2 = projection_z(chord);
  return (w.tiling_regions[v0]->contains(chord) && 
	  w.tiling_regions[v1]->contains(chord));
}

bool test_theorem3(const Segment_3& segment, const Point_3& opposite, Tiler_workspace& w)
{
  log4cplus::Logger logger = log4cplus::Logger::getInstance("tiler.theorems.3");
  LOG4CPLUS_TRACE(logger, pp_id(segment) << " " << opposite.id());

  if (xy_equal(opposite, segment.source()) ||
      xy_equal(opposite, segment.target()))
    return true;

  const Hierarchy& h = w.hierarchies.find(opposite.z())->second;
  Point_2 opposite_2 = opposite.point_2();
  Line_2 line = projection_z(segment).supporting_line();

  bool pass = true;
  for (int i = 0; pass && i < 2; ++i)
  {
    Point_3 v = segment[i];
    Contour_handle overlap_contour;
    Contour_handle nec;
    Polygon_2::Vertex_circulator ci;
    boost::tie(overlap_contour, nec, ci) = h.is_overlapping(v.point_2());
    bool overlapping = (overlap_contour != NULL);
    if (!overlapping)
    {
      // RS = negative side
      // LS = positive side
      if (h.is_CW(nec))
	pass = !line.has_on_negative_side(opposite_2);
      else
	pass = !line.has_on_positive_side(opposite_2);
    }
  }

  return pass;
}

// This has one subtle difference with theorem4 in the bajaj96 paper: if the chord
// has ANY intersection with any point on a contour segment then that segment is
// tested for connectivity with the chord.  Connectivity, in this sense, means that
// there exists a path from the intersection point to one of the chord's endpoints 
// and that every vertex on this path also intersects with the chord.  An example
// of an intersection rendered illegal by this change that would be legal in the 
// original paper is this:
//
//  ____________________________________
// |                                 __/| e
// |                            d __/   |
// |     A                     __/----  |
// |         --------       __/ |     | |
// |        |        |   __/    | C   | |
// |        |   B    |__/       |     | |
// |         -------- f        c -----  |
// |                                    |
// |                                    |
// |                                    |
// |                                    |
// | a                                b |
// |____________________________________|
//
// The chord from B to A only grazes C.  The original theorem 4 allows this.  It
// is disallowed here because since our algorithm can fail.  For example, suppose
// the chord is allowed and then we end up with an untiled region abcdef.  This region
// is a degenerate polygon in 2D (because of the collinearity of def).
bool test_theorem4(const Segment_3& segment, const Point_3& opposite, Tiler_workspace& w)
{
  log4cplus::Logger logger = log4cplus::Logger::getInstance("tiler.theorems.4");
  LOG4CPLUS_TRACE(logger, pp_id(segment) << " " << opposite.id());

  typedef Tiler_workspace::Contours::const_iterator Iter;
  typedef list<SL_intersection>::const_iterator I_iter;
  typedef set<Segment_2_undirected>::const_iterator S_it;

  list<Segment_2> chords;
  chords.push_back(Segment_2(segment.source(), opposite));
  chords.push_back(Segment_2(segment.target(), opposite));

  bool intersects = false;

  list<Segment_2> contour_segments;
  for (Iter it = w.contours.begin(); it != w.contours.end(); ++it) {
    const Polygon_2& polygon = (*it)->polygon();
    contour_segments.insert(contour_segments.end(), polygon.edges_begin(), polygon.edges_end());
  }

  // end-int intersections between chords and contour segments
  list<SL_intersection> intersections;
  // contour segments that intersect the chord
  set<Segment_2_undirected> segs; 

  // component 0 is the chord
  // component 1 is the set of contour segments
  get_intersections(chords.begin(), chords.end(), 
		    contour_segments.begin(), contour_segments.end(), 
		    back_inserter(intersections), true, false);

  if (intersections.size() > 0)
    return false;

  for (I_iter it = intersections.begin(); it != intersections.end(); ++it) {
    const SL_intersection& i = *it;
    const list<Segment_2>& interiors = i.interiors();
    // Check if there is an interior-interior crossing.  If so, return false.
    if (interiors.size() == 2) {
      return false;
    }
    // It is a crossing of chord interior with segment end.
    segs.insert(i.ends().front());
  }

  for (S_it it = segs.begin(); it != segs.end(); ++it) {
    LOG4CPLUS_TRACE(logger, "Contour segment intersects with chord: " << pp(it->segment()));
    S_it n = it;
    ++n;
    if (n != segs.end()) {
      if (it->greater() != n->lesser())
	return false;
    }
  }

  if (!segs.empty()) {
    set<Point_3> points;
    points.insert(segment.source());
    points.insert(segment.target());
    points.insert(opposite);
    if (points.find(segs.begin()->lesser()) == points.end() &&
	points.find(segs.rbegin()->greater()) == points.end())
      return false;
    LOG4CPLUS_TRACE(logger, "Endpoint matches");
  }

  return true;
}

/// Returns false if the chords fail the test of Theorem 5, described in bajaj96.
bool test_theorem5(const Segment_3& chord0, const Segment_3& chord1)
{
  log4cplus::Logger logger = log4cplus::Logger::getInstance("tiler.theorems.5");

  CGAL::Object result;
  CGAL::Point_2<Kernel> ipoint;
  CGAL::Point_3<Kernel> ipoint_3;
  CGAL::Segment_2<Kernel> iseg;
  CGAL::Segment_3<Kernel> iseg_3;

  LOG4CPLUS_TRACE(logger, "Testing: " << pp_id(chord0) << "  " << pp_id(chord1));

  if (chord0 == chord1 || chord0 == chord1.opposite()) return true;
  if (chord0.source() == chord1.source() ||
      chord0.source() == chord1.target() ||
      chord0.target() == chord1.source() ||
      chord0.target() == chord1.target())
    return true;

  // Probably not necessary.  Doesn't appear to mess anything up, though.
//   if ((xy_equal(chord0.source(), chord1.source()) &&
//        xy_equal(chord0.target(), chord1.target())) ||
//       (xy_equal(chord0.source(), chord1.target()) &&
//        xy_equal(chord0.target(), chord1.source())))
//   {
//     LOG4CPLUS_TRACE(logger, "Projections are equal: " << pp_id(chord0) << "  " << pp_id(chord1));
//     return false;
//   }

  bool endpoint_intersection_2D = 
    (xy_equal(chord0.source(), chord1.source()) ||
     xy_equal(chord0.source(), chord1.target()) ||
     xy_equal(chord0.target(), chord1.source()) ||
     xy_equal(chord0.target(), chord1.target()));

  Segment_2 proj0(projection_z(chord0));
  result = CGAL::intersection(proj0, projection_z(chord1));
  if (!endpoint_intersection_2D && CGAL::assign(ipoint, result))
  {
    LOG4CPLUS_TRACE(logger, "2D intersection: " << pp_id(chord0) << "  " << pp_id(chord1));
    return false;
  }
  if (CGAL::assign(iseg, result))
  {
    // Check 3D intersection by projecting along the x and y axes
    result = CEP::intersection::intersection(chord0, chord1);
    if (CGAL::assign(ipoint_3, result) || CGAL::assign(iseg_3, result))
    {
      LOG4CPLUS_TRACE(logger, "3D intersection: " << pp_id(chord0) << "  " << pp_id(chord1));
      return false;
    }
  }

  LOG4CPLUS_TRACE(logger, "No intersection: " << pp_id(chord0) << "  " << pp_id(chord1));

  return true;
}

bool test_theorem5(const Segment_3& chord, const Boundary_slice_chords& bscs)
{
  log4cplus::Logger logger = log4cplus::Logger::getInstance("tiler.theorems.5");
  LOG4CPLUS_TRACE(logger, pp_id(chord));

  list<Boundary_slice_chord> chords;
  bscs.all(back_inserter(chords));
  for (list<Boundary_slice_chord>::iterator it = chords.begin(); it != chords.end(); ++it)
  {
    const Segment_3& test_chord = it->segment();
    if (!test_theorem5(chord, test_chord))
      return false;
  }
  return true;
}

bool test_theorem5(const Segment_3& chord0, const Segment_3& chord1, const Boundary_slice_chords& bscs)
{
  log4cplus::Logger logger = log4cplus::Logger::getInstance("tiler.theorems.5");
  LOG4CPLUS_TRACE(logger, pp_id(chord0) << " " << pp_id(chord1));

  list<Boundary_slice_chord> chords;
  bscs.all(back_inserter(chords));
  for (list<Boundary_slice_chord>::iterator it = chords.begin(); it != chords.end(); ++it)
  {
    const Segment_3& test_chord = it->segment();
    if (!test_theorem5(chord0, test_chord) || !test_theorem5(chord1, test_chord))
      return false;
  }
  return true;
}

/// This is a new test not presented in the paper
bool test_theorem8(const Segment_3& segment, const Point_3& opposite, const Tiler_workspace& w)
{
  return true;

  log4cplus::Logger logger = log4cplus::Logger::getInstance("tiler.theorems.8");
  LOG4CPLUS_TRACE(logger, pp_id(segment) << " " << opposite.id());

  Point_3 source = segment.source();
  Point_3 target = segment.target();
  const Hierarchy& h = w.hierarchies.find(source.z())->second;
  const Hierarchy& h_ = w.hierarchies.find(opposite.z())->second;
  Contour_handle contour = w.contour(source);

  Contour_handle overlap_contour1, overlap_contour2;
  Contour_handle nec1, nec2;
  Polygon_2::Vertex_circulator ci1, ci2;
  boost::tie(overlap_contour1, nec1, ci1) = h_.is_overlapping(source.point_2());
  boost::tie(overlap_contour2, nec2, ci2) = h_.is_overlapping(target.point_2());
  bool overlapping1 = (overlap_contour1 != NULL);
  bool overlapping2 = (overlap_contour2 != NULL);
  if (overlapping1 && overlapping2)
  {
    if (target != w.vertices.ccw(source) && target != w.vertices.cw(source))
      return false;
    Point_3 source_ = ci1->point_3();
    Point_3 target_ = ci2->point_3();
    if ((xy_equal(opposite, source) || xy_equal(opposite, target)) && 
	target_ != w.vertices.ccw(source_) && target_ != w.vertices.cw(source_))
      return false;
//     Polygon_2::Vertex_circulator next = ci1, prev = ci1;
//     next++;
//     prev--;
//     if (*next != *ci2 && *prev != *ci2)
//       return false;
  }
  return true;
}

/// If u is overlapping and v is not, then the tile is legal only if
/// v is positive.
bool test_theorem9(const Segment_3& segment, const Point_3& opposite, Tiler_workspace& w)
{
  return true;

  log4cplus::Logger logger = log4cplus::Logger::getInstance("tiler.theorems.9");
  LOG4CPLUS_TRACE(logger, pp_id(segment) << " " << opposite.id());

  const Hierarchy& h = w.hierarchies.find(segment.source().z())->second;
  const Hierarchy& h_ = w.hierarchies.find(opposite.z())->second;
  Contour_handle contour = w.vertices.contour(segment[0]);
  CGAL::Orientation orientation = h.orientation(contour);
//   Point_2 opposite_2 = opposite.point_2();
//   Line_2 line = projection_z(segment).supporting_line();

//   bool pass = true;
  for (int i = 0; i < 2; ++i)
  {
    Point_3 u = segment[i];
    Point_3 v = segment[1-i];
    Contour_handle u_overlap_contour, v_overlap_contour;
    Contour_handle nec;
    Polygon_2::Vertex_circulator ci;
    boost::tie(u_overlap_contour, nec, ci) = h_.is_overlapping(u.point_2());
    boost::tie(v_overlap_contour, nec, ci) = h_.is_overlapping(v.point_2());
    bool u_overlaps = (u_overlap_contour != NULL);
    bool v_overlaps = (v_overlap_contour != NULL);
    if (u_overlaps && !v_overlaps && xy_equal(u, opposite))
    {
      Vertex_sign sign = Vertex_sign::POSITIVE;
      Contour_handle c_;
      boost::tie(sign, c_) = h_.vertex_sign(v.point_2(), orientation);
      if (sign != Vertex_sign::POSITIVE)
	return false;
    }
  }

  return true;
}


CONTOURTILER_END_NAMESPACE
