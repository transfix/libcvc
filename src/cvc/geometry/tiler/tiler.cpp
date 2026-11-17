#include <boost/lexical_cast.hpp>
#include <boost/foreach.hpp>

#include <ContourTiler/tiler.h>
#include <ContourTiler/tiler_operations.h>
#include <ContourTiler/print_utils.h>
#include <ContourTiler/sweep_line_visitors.h>
#include <ContourTiler/set_utils.h>
#include <ContourTiler/tiler_output.h>
#include <ContourTiler/intersection.h>
#include <ContourTiler/cut.h>
#include <ContourTiler/perturb.h>

CONTOURTILER_BEGIN_NAMESPACE

template <typename Contour_iter>
TW_handle tile(Contour_iter bottom_begin, Contour_iter bottom_end,
	       Contour_iter top_begin, Contour_iter top_end,
	       const Tiler_options& options);

//------------------------------------------------------------------------------
// tile_one_component
//
/// Tiles one component across two slices
//------------------------------------------------------------------------------
template <typename Contour_iter>
TW_handle tile_one_component(Contour_iter bottom_begin, Contour_iter bottom_end,
	       Contour_iter top_begin, Contour_iter top_end,
	       const Tiler_options& options)
{
  using namespace std;

  output_pre_pre_tile(bottom_begin, bottom_end, top_begin, top_end, options);

  // Pre-tile: augment, generate hierarchy, get correspondences, build otv table
  TW_handle workspace = pre_tile(bottom_begin, bottom_end, top_begin, top_end, false, options.correspondence_overlap());
  Tiler_workspace& w = *workspace;
  output_pre_tile(w, options);

  // Perform tiling phase 1
  boost::shared_ptr<Callback> callback(new Callback(options));
  w.set_callback(callback);
  build_tiling_table_phase1(w);
  output_phase1(w, options);

  // Perform tiling phase 2
  build_tiling_table_phase2(w);//, options.interp_untiled_regions());
  output_phase2(w, options);

  return workspace;

}

template
TW_handle tile_one_component(vector<Contour_handle>::iterator bottom_begin, vector<Contour_handle>::iterator bottom_end,
	       vector<Contour_handle>::iterator top_begin, vector<Contour_handle>::iterator top_end,
	       const Tiler_options& options);

class Tiler_intersection
{
public:
  Tiler_intersection(size_t a_id, size_t b_id, Number_type a_z, Number_type b_z)
     : _a_id(a_id), _b_id(b_id), _a_z(a_z), _b_z(b_z) {}
//   {
//     if (a_id > b_id) {
//       swap(a_id, b_id);
//     }
//     _a_id = a_id;
//     _b_id = b_id;
//   }
  ~Tiler_intersection() {}
  
  size_t a_id() const { return _a_id; }
  size_t b_id() const { return _b_id; }
  Number_type a_z() const { return _a_z; }
  Number_type b_z() const { return _b_z; }

  bool operator==(const Tiler_intersection& t) const
  {
    return _a_id == t._a_id &&
      _b_id == t._b_id &&
      _a_z == t._a_z &&
      _a_z == t._a_z;
  }

private:
  size_t _a_id, _b_id;
  Number_type _a_z, _b_z;
};

std::size_t hash_value(const Tiler_intersection& t)
{
  std::size_t seed = 0;
  boost::hash_combine(seed, t.a_id());
  boost::hash_combine(seed, t.b_id());
  boost::hash_combine(seed, t.a_z());
  boost::hash_combine(seed, t.b_z());
  return seed;
}

//------------------------------------------------------------------------------
// get_intersections
//
/// Gets intersections between multiple components using the sweep line
/// algorithm.  pairs will contain pairs of component IDs representing components
/// that intersect each other.  The component IDs match the iteration order of
/// tw_begin and tw_end.
//------------------------------------------------------------------------------
template <typename TW_iter, typename Out_iter>
void get_intersections(TW_iter tw_begin, TW_iter tw_end, Out_iter pairs)
{
  static log4cplus::Logger logger = log4cplus::Logger::getInstance("get_intersections");

  vector<Segment_2> segments;
  vector<SL_point> points;
  size_t comp_id = 0;
  for (TW_iter it = tw_begin; it != tw_end; ++it)
  {
    TW_handle tw = *it;
    for (Tile_iterator tile_it = tw->tiles_begin(); tile_it != tw->tiles_end(); ++tile_it)
    {
      const Triangle& tile = **tile_it;
      for (int i = 0; i < 3; ++i)
      {
	Segment_3 s_3 = edge(i, tile);
	Segment_2 s_2(s_3.source(), s_3.target());
	Segment_2_undirected s_u(s_2);
	segments.push_back(s_2);
	points.push_back(SL_point(s_2.source(), segments.size()-1, s_u, comp_id));
      }
    }
    ++comp_id;
  }

  sort(points.begin(), points.end());
  for (vector<SL_point>::iterator it = points.begin(); it != points.end(); ++it) {
    LOG4CPLUS_TRACE(logger, it->point());
  }

  typedef list<SL_intersection> Intersection_list;
  typedef Intersection_visitor<back_insert_iterator<Intersection_list> > Visitor;
  Intersection_list intersections;
  Visitor v(back_inserter(intersections));
  sweep_line_multi_comp(points.begin(), points.end(), segments, comp_id, v);
  
  vector<TW_handle> tws(tw_begin, tw_end);
  for (list<SL_intersection>::iterator it = intersections.begin(); it != intersections.end(); ++it)
  {
    size_t a_id, b_id;
    boost::tie(a_id, b_id) = it->components();
    Point_2 intersection_point = it->point();
    Segment_2 a_seg = it->segment(a_id);
    Segment_2 b_seg = it->segment(b_id);

//     Number_type a_z = a_seg.source().z();
//     if (xy_equal(a_seg.source(), a_seg.target()))
//       throw logic_error("Vertical chord cannot intersect another component");
//     a_z = tws[a_id]->zhome(a_seg.source(), a_seg.target());

//     if (a_z != (int)a_z)
//     {
//       LOG4CPLUS_ERROR(logger, "z_home returned not on a slice: " << a_z << " " << pp(a_seg) << " " << a_id);
//       a_z = tws[a_id]->zhome(a_seg.source(), a_seg.target());
//     }

//     Number_type b_z = b_seg.source().z();
//     if (xy_equal(a_seg.source(), a_seg.target()))
//       throw logic_error("Vertical chord cannot intersect another component");
//     b_z = tws[b_id]->zhome(b_seg.source(), b_seg.target());

//     if (b_z != (int)b_z)
//       LOG4CPLUS_ERROR(logger, "z_home returned not on a slice: " << b_z << " " << pp(b_seg) << " " << b_id);

//     LOG4CPLUS_TRACE(logger, "z_home for: " << a_id << " = " << a_z);
//     LOG4CPLUS_TRACE(logger, "z_home for: " << b_id << " = " << b_z);

//     Tiler_intersection ti(a_id, b_id, a_z, b_z);
    Tiler_intersection ti(a_id, b_id, 0, 1);
    *pairs++ = ti;
  }
}

//------------------------------------------------------------------------------
// augment_slices
//
/// Augments contours in-place across multiple slices.
//------------------------------------------------------------------------------
template <typename Slice_iter>
bool augment_slices(Slice_iter slices_begin, Slice_iter slices_end)
{
  static log4cplus::Logger logger = log4cplus::Logger::getInstance("augment_slices");

  Slice_iter it0 = slices_begin;
  Slice_iter it1 = it0;
  ++it1;

  bool augmented_some_point = false;
  while (it1 != slices_end)
  {
    Slice& bottom = *it0;
    Slice& top = *it1;

    bottom.validate();
    top.validate();

    // Get all components in the two slices
    vector<string> components;
    {
      boost::unordered_set<string> bottom_components, top_components;
      bottom.components(inserter(bottom_components, bottom_components.end()));
      top.components(inserter(top_components, top_components.end()));
      set_intersection(bottom_components, top_components, back_inserter(components));
    }

    // For each component, augment the component's contours.
    for (vector<string>::const_iterator it = components.begin(); it != components.end(); ++it)
    {
      const string& component = *it;

      // Put all of the component's contours in 
      // bc (bottom) and tc (top).
      list<Contour_handle> bc, tc;
      bc.insert(bc.end(), bottom.begin(component), bottom.end(component));
      tc.insert(tc.end(), top.begin(component), top.end(component));

      // Augment the contours, putting the results in bac and tac.
      list<Contour_handle> bac, tac;
      bool b = augment(bc.begin(), bc.end(), tc.begin(), tc.end(), back_inserter(bac), back_inserter(tac));
      augmented_some_point = augmented_some_point || b;

      // Update the slices with the augmented contours.
      bottom.replace(component, bac.begin(), bac.end());
      top.replace(component, tac.begin(), tac.end());
    }
    bottom.validate();
    top.validate();
    ++it0;
    ++it1;
  }
  return augmented_some_point;
}

Number_type round(Number_type d, int dec) {
  d *= pow((Number_type)10, (Number_type)dec);
  d = (d < 0) ? ceil(d-0.5) : floor(d+0.5);
  d /= pow((Number_type)10, (Number_type)dec);
  return d;
}

void round(Polygon_2& p, int dec) {
  Polygon_2::Vertex_iterator vit;
  for (vit = p.vertices_begin(); vit != p.vertices_end(); ++vit) {
    Point_2 pnt = *vit;
    pnt = Point_25_<Kernel>(round(pnt.x(), dec), round(pnt.y(), dec), pnt.z(), pnt.id());
    p.set(vit, pnt);
  }

  p = Polygon_2(p.vertices_begin(), unique(p.vertices_begin(), p.vertices_end()));
}

void round(Slice& slice, int dec) {
  list<string> components;
  slice.components(back_inserter(components));
  for (list<string>::const_iterator it = components.begin();
       it != components.end();
       ++it)
  {
    for (Slice::Contour_iterator cit = slice.begin(*it);
	 cit != slice.end(*it);
	 ++cit)
    {
      round((*cit)->polygon(), dec);
    }
  }
}

struct Colored_triangle
{
  Colored_triangle(const Triangle& triangle, const Color& color)
    : _triangle(triangle), _color(color) {}
  Triangle _triangle;
  Color _color;
};

//------------------------------------------------------------------------------
// tile
//
/// Tiles multiple components across multiple slices
//------------------------------------------------------------------------------
template <typename Slice_iter>
void tile(Slice_iter slices_begin, Slice_iter slices_end, 
	  const boost::unordered_map<string, Color>& comp2color, 
	  const Tiler_options& options)
{
  using namespace std;
  static log4cplus::Logger logger = log4cplus::Logger::getInstance("tiler.tile");

  typedef boost::shared_ptr<Triangle> Triangle_handle;

  vector<TW_handle> workspaces;
  boost::unordered_map<string, list<Colored_triangle> > triangles_by_component;
//   list<Colored_point_3> all_points;
//   boost::unordered_map<string, list<Colored_point_3> > points_by_component;

  Slice_iter it0 = slices_begin;
  Slice_iter it1 = it0;
  ++it1;
  int i = 0;

  // Contours after intersection removal may be very slightly different
  // even if there were no intersections since intersection removal
  // performs an expansion and contraction of each contour.
  LOG4CPLUS_INFO(logger, "Removing 2D contour intersections");
  for (Slice_iter it = slices_begin; it != slices_end; ++it) {
    Slice& s = *it;
    LOG4CPLUS_TRACE(logger, "Preprocessing slice: " << s.to_string());
    if (!s.empty()) {
      LOG4CPLUS_DEBUG(logger, "Removing 2D contour intersections in slice " << s.z());
    }
    s.validate();
    s.remove_collinear(options.collinear_epsilon());
    s.validate();
    LOG4CPLUS_TRACE(logger, "Removed collinear points in slice " << s.z());

    if (options.contour_curation_delta() > 0) {
      LOG4CPLUS_TRACE(logger, "ri1: " << s.to_string());
      s.remove_intersections(options.contour_curation_delta());
      LOG4CPLUS_TRACE(logger, "ri2: " << s.to_string());
      s.validate();
      LOG4CPLUS_TRACE(logger, "Removed intersections in slice " << s.z());

      // round(s, 5);
      // perturb(s, 0.000001);
      s.validate();
      LOG4CPLUS_TRACE(logger, "Rounded in slice " << s.z());
    }
    LOG4CPLUS_TRACE(logger, "Finished preprocessing slice: " << s.to_string());
  }

  int aug = 1;
  LOG4CPLUS_DEBUG(logger, "Augmenting contours - iteration " << aug++);
  while (augment_slices(slices_begin, slices_end)) {
    LOG4CPLUS_DEBUG(logger, "Augmenting contours - iteration " << aug++);
  }

  // Iterate through the slices
  boost::unordered_map<Segment_3_undirected, list<Point_3> > edge2points;
  while (it1 != slices_end)
  {
    const Slice& bottom = *it0;
    const Slice& top = *it1;

    if (!bottom.empty() || !top.empty()) {
      if (bottom.empty()) {
        LOG4CPLUS_INFO(logger, "Tiling slice " << top.z());
      }
      else if (bottom.empty()) {
        LOG4CPLUS_INFO(logger, "Tiling slice " << bottom.z());
      }
      else {
        LOG4CPLUS_INFO(logger, "Tiling slices " << bottom.z() << " - " << top.z());
      }
    }

    it0->augment(edge2points);

    boost::unordered_set<string> bottom_components, top_components;
    bottom.components(inserter(bottom_components, bottom_components.end()));
    top.components(inserter(top_components, top_components.end()));
    vector<string> components;
    set_union(bottom_components, top_components, back_inserter(components));

    vector<TW_handle> slice_workspaces;
    vector<list<Triangle_handle> > tiles;

    // Tile each component in between these two slices.  Add resultant workspaces to 
    // slice_workspaces and resultant tiles to tiles.
    for (vector<string>::const_iterator it = components.begin(); it != components.end(); ++it)
    {
      const string& component = *it;

      Tiler_options sub_options = options;
      sub_options.base_name() = sub_options.base_name() + component + boost::lexical_cast<string>(i);

      list<Contour_handle> contours[2];
      contours[0].insert(contours[0].end(), bottom.begin(component), bottom.end(component));
      contours[1].insert(contours[1].end(), top.begin(component), top.end(component));

      LOG4CPLUS_DEBUG(logger, "Tiling " << component << " slices " 
		      << bottom.z() << " - " << top.z());
      TW_handle w = tile_one_component(bottom.begin(component), bottom.end(component), 
			 top.begin(component), top.end(component), 
			 sub_options);

      slice_workspaces.push_back(w);
      list<Triangle_handle> t(w->tiles_begin(), w->tiles_end());
      tiles.push_back(t);
    }

    // After tiling is complete, remove any intersections between slices
    // that were created.
    if (options.remove_intersections())
    {
      if (!bottom.empty() && !top.empty()) {
        LOG4CPLUS_INFO(logger, "Removing intersections between slices " << bottom.z() << " - " << top.z());
      }

      // Find all tiles that intersect between components
      boost::unordered_set<Tiler_intersection> intersections;
      get_intersections(slice_workspaces.begin(), slice_workspaces.end(), inserter(intersections, intersections.end()));

      // For each intersecting component pair, remove intersections
      // for (boost::unordered_set<Tiler_intersection>::iterator it = intersections.begin(); it != intersections.end(); ++it)
      // {
	// size_t a = it->a_id(), b = it->b_id();
      for (const auto& ti : intersections) {
	size_t a = ti.a_id(), b = ti.b_id();
        TW_handle twa = slice_workspaces[a];
        TW_handle twb = slice_workspaces[b];

	vector<Triangle_handle> new_yellow, new_green;
	// Number_type yz = it->a_z();
	// Number_type gz = it->b_z();
	Number_type yz = ti.a_z();
	Number_type gz = ti.b_z();
	// Number_type zmin = slice_workspaces[a]->zmin();
	// Number_type zmax = slice_workspaces[a]->zmax();
	Number_type zmin = min(twa->zmin(), twb->zmin());
	Number_type zmax = max(twa->zmax(), twb->zmax());
 	Number_type zdelta = fabs(zmax - zmin) * options.contour_curation_delta() / options.z_scale();
	LOG4CPLUS_TRACE(logger, "z values: " << fabs(zmax-zmin) << " " 
			<< options.contour_curation_delta() << " " 
			<< options.z_scale() << " " << zdelta);

	LOG4CPLUS_DEBUG(logger, "Removing intersections between " 
			<< components[a] << " - " << components[b] << " slices " 
			<< zmin << " - " << zmax);

	if (zmin != zmax) {
	  try {
	    remove_intersections(slice_workspaces[a], slice_workspaces[b],
				 tiles[a].begin(), tiles[a].end(),
				 tiles[b].begin(), tiles[b].end(),
				 yz, gz,
				 back_inserter(new_yellow), back_inserter(new_green),
				 zdelta,
				 edge2points);

	    tiles[a].clear();
	    tiles[a].insert(tiles[a].end(), new_yellow.begin(), new_yellow.end());
	    tiles[b].clear();
	    tiles[b].insert(tiles[b].end(), new_green.begin(), new_green.end());
	  } catch (std::logic_error& e) {
	    LOG4CPLUS_ERROR(logger, "Removing intersections between " 
			  << components[a] << " - " << components[b] << " slices " 
			  << zmin << " - " << zmax);
	    LOG4CPLUS_ERROR(logger, "Failed to remove intersections: " << e.what());
	  }
	}
	else {
	  LOG4CPLUS_ERROR(logger, "Removing intersections between " 
			  << components[a] << " - " << components[b] << " slices " 
			  << zmin << " - " << zmax);
          TW_handle twa = slice_workspaces[a];
          TW_handle twb = slice_workspaces[b];
	  LOG4CPLUS_ERROR(logger, twa->zmin() << " - " << twa->zmax());
	  LOG4CPLUS_ERROR(logger, twb->zmin() << " - " << twb->zmax());
	  LOG4CPLUS_ERROR(logger, "Intersection occurred on same slice.  Failing to remove intersections. ");
	}
      }
    }

    workspaces.insert(workspaces.end(), slice_workspaces.begin(), slice_workspaces.end());

    int total_tiles = 0;
    for (int j = 0; j < slice_workspaces.size(); ++j)
    {
      const list<Triangle_handle>& t = tiles[j];
      const string& component = components[j];
      Color color(1,1,1);
      if (options.color())
	color = Color(options.color_r() / 255.0, options.color_g() / 255.0, options.color_b() / 255.0);
      else if (comp2color.find(component) != comp2color.end())
	color = comp2color.find(component)->second;
      for (list<Triangle_handle>::const_iterator th_it = t.begin(); th_it != t.end(); ++th_it)
      {
	const Triangle& tri = **th_it;
	triangles_by_component[component].push_back(Colored_triangle(tri, color));
// 	for (int k = 0; k < 3; ++k) {
// 	  all_points.push_back(Colored_point_3(tri[k], color));
// 	  points_by_component[component].push_back(Colored_point_3(tri[k], color));
// 	}
	total_tiles++;
      }
    }

    LOG4CPLUS_DEBUG(logger, "Total number of tiles for slices " 
		    << bottom.z() << " - " << top.z() << ": " << total_tiles);

    ++it0;
    ++it1;
    ++i;
  }

  // Put tiles into all_points which is a list of points with
  // tiles appearing as point triples:
  //   (all_points[0], all_points[1], all_points[2]) -- tile 0
  //   (all_points[3], all_points[4], all_points[5]) -- tile 1
  //   ...
  //
  // This takes into consideration any tiles that need to be augmented
  // and decomposed due to inducing points on an edge that exists on 
  // a slice.
  list<Colored_point_3> all_points;
  boost::unordered_map<string, list<Colored_point_3> > points_by_component;

  typedef boost::unordered_map<string, list<Colored_triangle> >::iterator MIter;
  typedef list<Colored_triangle>::iterator CIter;
  for (MIter mit = triangles_by_component.begin(); mit != triangles_by_component.end(); ++mit) {
    const string& component = mit->first;
    list<Colored_triangle>& triangles = mit->second;
    for (CIter it = triangles.begin(); it != triangles.end(); ++it) {
      const Triangle& tri = it->_triangle;
      const Color& color = it->_color;

      // Augment and decompose triangles as necessary.  New triangles
      // are appended to the end of the list.
      list<Triangle> new_triangles;
      bool failed = false;
      try {
	decompose_triangle(tri, edge2points, back_inserter(new_triangles));
      }
      catch (logic_error& e) {
	LOG4CPLUS_ERROR(logger, "Decomposing of triangle failed: " << e.what());
	failed = true;
      }
      if (!failed && new_triangles.size() > 1) {
	for (list<Triangle>::const_iterator t_it = new_triangles.begin(); t_it != new_triangles.end(); ++t_it) {
	  triangles.push_back(Colored_triangle(*t_it, color));
	}
      }
      else {
        // // debug
        // if (abs(tri[0].x() - 5.2767) < 0.0001 || abs(tri[1].x() - 5.2767) < 0.0001) {
        //   LOG4CPLUS_WARN(logger, "looking for this? " << pp(tri[0]) << " " << pp(tri[1]) << " " << pp(tri[2]));
        // }
        // // /debug
	for (int k = 0; k < 3; ++k) {
	  all_points.push_back(Colored_point_3(tri[k], color));
	  points_by_component[component].push_back(Colored_point_3(tri[k], color));
	}
      }
    }
  }

  // Output tiles to file on disk.
  string out_dir(options.output_dir());
  string base(options.base_name());
  if (options.output_raw()) {
    // All in one file
    {
      ofstream out((out_dir + "/" + base + "_tiles.rawc").c_str());
      raw_print_tiles_impl(out, all_points.begin(), all_points.end(), options.z_scale(), true);
      out.close();
    }

    // One file per component
    typedef boost::unordered_map<string, list<Colored_point_3> >::const_iterator PC_iter;
    for (PC_iter it = points_by_component.begin(); it != points_by_component.end(); ++it) {
      const string& component = it->first;
      const list<Colored_point_3>& points = it->second;

      ofstream out((out_dir + "/" + base + component + "_tiles.rawc").c_str());
      raw_print_tiles_impl(out, points.begin(), points.end(), options.z_scale(), true);
      out.close();
    }
  }
}

template
void tile(vector<Slice>::iterator slices_begin, vector<Slice>::iterator slices_end, 
	  const boost::unordered_map<string, Color>& comp2color, const Tiler_options& options);

CONTOURTILER_END_NAMESPACE
