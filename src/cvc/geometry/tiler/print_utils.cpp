#include <ContourTiler/print_utils.h>

#include <sstream>
#include <list>

CONTOURTILER_BEGIN_NAMESPACE

static int DEFAULT_PP_PRECISION = 6;
static int pp_precision = DEFAULT_PP_PRECISION;

void set_default_pp_precision(int precision)
{
  DEFAULT_PP_PRECISION = precision;
  pp_precision = precision;
}

void set_pp_precision(int precision)
{
  pp_precision = precision;
}

void restore_pp_precision()
{
  pp_precision = DEFAULT_PP_PRECISION;
}

// std::string pp(const Point_2& point)
// {
//   std::stringstream out;
//   out << "(" << point.x() << "," << point.y() << ")";
//   return out.str();
// }

std::string pp(const Point_2& point)
{
  std::stringstream out;
  out << std::setprecision(pp_precision);
  if (point.id() != DEFAULT_ID())
    out << "(" << point.x() << "," << point.y() << "," << point.z() << "; " << point.id() << ")";
  else
    out << "(" << point.x() << "," << point.y() << "," << point.z() << "; -1)";

  return out.str();
}

std::string pp(const Point_3& point)
{
  std::stringstream out;
  out << std::setprecision(pp_precision);
  if (point.id() != DEFAULT_ID())
    out << "(" << point.x() << "," << point.y() << "," << point.z() << "; " << point.id() << ")";
  else
    out << "(" << point.x() << "," << point.y() << "," << point.z() << "; -1)";
  return out.str();
}

std::string pp(const Segment_2& segment)
{
  std::stringstream out;
  out << std::setprecision(pp_precision);
  out << "[" << pp(segment.source()) << ", " << pp(segment.target()) << "]";
  return out.str();
}

std::string pp(const Segment_3& segment)
{
  std::stringstream out;
  out << std::setprecision(pp_precision);
  out << "[" << pp(segment.source()) << ", " << pp(segment.target()) << "]";
  return out.str();
}

std::string pp_id(const Segment_2& segment)
{
  std::stringstream out;
  size_t a = segment.source().id();
  size_t b = segment.target().id();
  if (a > b)
    swap(a, b);
  out << a << "--" << b;
  return out.str();
}

std::string pp_id(const Segment_3& segment)
{
  std::stringstream out;
  size_t a = segment.source().id();
  size_t b = segment.target().id();
  if (a > b)
    swap(a, b);
  out << a << "--" << b;
  return out.str();
}

std::string pp(const Point_3& v0, const Point_3& v1)
{
  Segment_3 segment(v0.point_3(), v1.point_3());
  return pp(segment);
}

std::string pp(const Tiling_region& region)
{
  std::stringstream out;
  out << std::setprecision(pp_precision);
  out << region;
  return out.str();
}

// std::string pp(const Polygon& P)
// {
//   Polygon::Vertex_const_iterator vit;
//   std::stringstream out;
//  out << std::setprecision(pp_precision);

//   out << "[ " << P.size() << " vertices:";
//   for (vit = P.vertices_begin(); vit != P.vertices_end(); ++vit)
//     out << " (" << *vit << ')';
//   out << " ]";
//   return out.str();
// }

std::string pp(const Polygon_2& P)
{
  Polygon_2::Vertex_const_iterator vit;
  std::stringstream out;
  out << std::setprecision(pp_precision);

  out << "[ " << P.size() << " vertices:";
  for (vit = P.vertices_begin(); vit != P.vertices_end(); ++vit)
    out << pp(*vit);
  out << " ]";
  return out.str();
}

std::string pp(const Polygon_with_holes_2& pwh)
{
  std::stringstream out;
  out << "[Outer boundary: " << pp(pwh.outer_boundary()) << "]  ";
  for (Polygon_with_holes_2::Hole_const_iterator it = pwh.holes_begin(); it != pwh.holes_end(); ++it) {
    out << "Hole: " << pp(*it) << "  ";
  }
  return out.str();
}

std::string pp(const Polyline_2& polyline)
{
  Polyline_2::const_iterator vit;
  std::stringstream out;
  out << std::setprecision(pp_precision);

  out << "[ " << polyline.size() << " vertices:";
  for (vit = polyline.begin(); vit != polyline.end(); ++vit)
    out << pp(*vit);
  out << " ]";
  return out.str();
}

std::string pp(const Polyline_3& polyline)
{
  Polyline_3::const_iterator vit;
  std::stringstream out;
  out << std::setprecision(pp_precision);

  out << "[ " << polyline.size() << " vertices:";
  for (vit = polyline.begin(); vit != polyline.end(); ++vit)
    out << pp(*vit);
  out << " ]";
  return out.str();
}

std::string pp(const Slice& slice)
{
  std::stringstream out;
  out << std::setprecision(pp_precision);

  list<string> components;
  slice.components(back_inserter(components));
  for (list<string>::const_iterator it = components.begin();
       it != components.end();
       ++it)
  {
    out << *it << ": ";
    for (Slice::Contour_const_iterator cit = slice.begin(*it);
	 cit != slice.end(*it);
	 ++cit)
    {
      out << pp((*cit)->polygon());
    }
  }
  
  return out.str();
}

void gnuplot_print_otvs(std::ostream& out, const Tiler_workspace& w)
{
  using namespace std;
  const OTV_table& otv_table = w.otv_table;
  for (Vertex_iterator it = w.vertices_begin(); it != w.vertices_end(); ++it)
  {
    const Point_3& v0 = *it;
    const Point_3& v1 = w.otv_table[v0];
    if (v1.id() != DEFAULT_ID())
    {
      out << v0 << endl << v1 << endl << endl << endl;
    }
  }
}

void gnuplot_print_otv_pairs(std::ostream& out, const Tiler_workspace& w)
{
  using namespace std;
  const OTV_table& otv_table = w.otv_table;
  for (Vertex_iterator it = w.vertices_begin(); it != w.vertices_end(); ++it)
  {
    const Point_3& v0 = *it;
    const Point_3& v1 = w.otv_table[v0];
    if (v1.is_valid() && w.otv_table[v1] == v0)
    {
      out << v0.point() << " " << w.contour(v0)->slice() << endl;
      out << v1.point() << " " << w.contour(v1)->slice() << endl;
      out << endl << endl;
    }
  }
}

void gnuplot_print_polygons(std::ostream& out, const std::list<Untiled_region>& list)
{
  std::list<Untiled_region>::const_iterator lit = list.begin();
  for (; lit != list.end(); ++lit)
  {
    Untiled_region::const_iterator vit = lit->begin();
    for (; vit != lit->end(); ++vit)
      out << *vit << std::endl;
    out << *(lit->begin()) << std::endl << std::endl << std::endl;
  }
}

void gnuplot_print_polygon(std::ostream& out, const Untiled_region& poly)
{
  int prec = out.precision();
  out << std::setprecision(pp_precision);
  Untiled_region::const_iterator vit = poly.begin();
  for (; vit != poly.end(); ++vit)
    out << *vit << std::endl;
  out << *(poly.begin()) << std::endl << std::endl << std::endl;
  out.precision(prec);
}

void gnuplot_print_polygon(std::ostream& out, const Polygon_2& P)
{
  int prec = out.precision();
  out << std::setprecision(pp_precision);
  Polygon_2::Vertex_const_iterator  vit;
  for (vit = P.vertices_begin(); vit != P.vertices_end(); ++vit)
    vit->insert(out) << std::endl;//" " << vit->z() << std::endl;
  P.vertices_begin()->insert(out) << std::endl;//" " << P.vertices_begin()->z() << std::endl;
  out.precision(prec);
}

void gnuplot_print_polygon(const std::string& filename, const Polygon_2& P)
{
  std::ofstream out(filename.c_str());
  gnuplot_print_polygon(out, P);
  out.close();
}

void gnuplot_print_polygon(std::ostream& out, const Polygon_with_holes_2& P)
{
  gnuplot_print_polygon(out, P.outer_boundary());
  out << std::endl << std::endl << std::endl;
  for (Polygon_with_holes_2::Hole_const_iterator it = P.holes_begin(); it != P.holes_end(); ++it) {
    gnuplot_print_polygon(out, *it);
    out << std::endl << std::endl << std::endl;
  }
}

// void gnuplot_print_polygon(std::ostream& out, const Polygon& P)
// {
//   Polygon::Vertex_const_iterator  vit;
//   for (vit = P.vertices_begin(); vit != P.vertices_end(); ++vit)
//     out << *vit << std::endl;
//   out << *(P.vertices_begin()) << std::endl;
// }

void gnuplot_print_polygon(std::ostream& out, const Polygon_2& P, int z)
{
  int prec = out.precision();
  out << std::setprecision(pp_precision);
  Polygon_2::Vertex_const_iterator  vit;
  for (vit = P.vertices_begin(); vit != P.vertices_end(); ++vit)
    out << *vit << " " << z << std::endl;
  out << *(P.vertices_begin()) << " " << z << std::endl;
  out.precision(prec);
}

void print_contour(Contour_handle contour)
{
  print_polygon(contour->polygon());
}

void gnuplot_print_vertices(std::ostream& out, const Vertices& vertices)
{
  for (Vertex_iterator it = vertices.begin(); it != vertices.end(); ++it)
    out << (*it) << " " << it->unique_id() << endl;
}

// void raw_print_tiles(std::ostream& out, const Tiles& tiles, double z_scale)
// {
// //   out << std::setprecision(12);

//   size_t num_verts = tiles.num_verts();
//   size_t num_tiles = tiles.num_tiles();
//   out << num_verts << " " << num_tiles << endl;
//   list<pair<Point_3, Tiles::Color> > vertices = tiles.vertices();
//   for (list<pair<Point_3, Tiles::Color> >::const_iterator it = vertices.begin(); it != vertices.end(); ++it)
//   {
//     Point_3 p(it->first.x(), it->first.y(), z_scale * CGAL::to_double(it->first.z()));
//     double r = boost::get<0>(it->second);
//     double g = boost::get<1>(it->second);
//     double b = boost::get<2>(it->second);
//     out << p << " " << r << " " << g << " " << b << endl;
//   }
//   list<size_t> indices = tiles.tile_indices();
//   for (list<size_t>::const_iterator it = indices.begin(); it != indices.end(); )
//     out << *it++ << " " << *it++ << " " << *it++ << endl;
// }

template <typename Point>
Color get_color(const Point& point);

template<>
Color get_color(const Point_3& point)
{ return Color(1, 1, 1); }

template<>
Color get_color(const Colored_point_3& point)
{ return point.color(); }

template <typename Point_iter>
void raw_print_tiles_impl(std::ostream& out, Point_iter points_begin, Point_iter points_end, double z_scale, bool color)
{
  static log4cplus::Logger logger = log4cplus::Logger::getInstance("raw_print_tiles_impl");

  // out << std::setprecision(12);

  typedef typename iterator_traits<Point_iter>::value_type Point_type;
  typedef boost::unordered_map<Point_type, size_t> Vertex_map;

  Vertex_map vertices;
  size_t next_idx = 0;
  size_t num_points = 0;
  stringstream ss;
  // ss << std::setprecision(24);
  for (Point_iter it = points_begin; it != points_end; ++it)
  {
    const Point_type& t = *it;
    if (vertices.find(t) == vertices.end())
    {
      // // debug
      // if (abs(it->x() - 5.2767) < 0.0001) {
      //   LOG4CPLUS_WARN(logger, "looking for this? " << it->x() << 
      //                  " " << it->y() << " " << (it->z()+0.5) * z_scale <<
      //                  " " << it->point().id());
      // }
      // // /debug

      Color c = get_color(t);
      vertices[t] = next_idx++;
      ss << t.x() << " " << t.y() << " " << (t.z()+0.5) * z_scale;
      if (color)
	ss << " " << c.r() << " " << c.g() << " " << c.b();
      ss << endl;
    }
    ++num_points;
  }
  size_t num_verts = vertices.size();
  num_points /= 3;

  out << num_verts << " " << num_points << endl;
  out << ss.str();

  size_t n = 0;
  for (Point_iter it = points_begin; it != points_end; ++it)
  {
    out << vertices[*it] << " ";
    if ((++n) % 3 == 0)
      out << endl;
  }
}

template
void raw_print_tiles_impl(std::ostream& out, 
		     vector<Colored_point_3>::iterator points_begin, 
		     vector<Colored_point_3>::iterator points_end, 
		     double z_scale, bool color);
template
void raw_print_tiles_impl(std::ostream& out, 
		     list<Colored_point_3>::iterator points_begin, 
		     list<Colored_point_3>::iterator points_end, 
		     double z_scale, bool color);
template
void raw_print_tiles_impl(std::ostream& out, 
		     list<Colored_point_3>::const_iterator points_begin, 
		     list<Colored_point_3>::const_iterator points_end, 
		     double z_scale, bool color);
template
void raw_print_tiles_impl(std::ostream& out, 
		     Colored_point_3* points_begin, 
		     Colored_point_3* points_end, 
		     double z_scale, bool color);

template
void raw_print_tiles_impl(std::ostream& out, 
		     vector<Point_3>::iterator points_begin, 
		     vector<Point_3>::iterator points_end, 
		     double z_scale, bool color);
template
void raw_print_tiles_impl(std::ostream& out, 
		     list<Point_3>::iterator points_begin, 
		     list<Point_3>::iterator points_end, 
		     double z_scale, bool color);
template
void raw_print_tiles_impl(std::ostream& out, 
		     Point_3* points_begin, 
		     Point_3* points_end, 
		     double z_scale, bool color);


template <typename Tile_iter>
void gnuplot_print_tiles(std::ostream& out, Tile_iter tiles_begin, Tile_iter tiles_end)
{
  list<Point_3> verts;
  for (Tile_iter it = tiles_begin; it != tiles_end; ++it)
    for (int i = 0; i < 3; ++i)
    {
      const Triangle& tri = get_tri(*it);
      verts.push_back(Point_3(vertex(i, tri)));
    }
  
//   std::list<Point_3> verts;
//   tiles.as_single_array(std::back_inserter(verts));

  for (std::list<Point_3>::iterator it = verts.begin(); it != verts.end(); ++it)
  {
    std::list<Point_3>::iterator first = it;
    out << *it << endl;
    ++it;
    out << *it << endl;
    ++it;
    out << *it << endl;
    out << *first << endl;
    out << endl << endl;
  }
}

template
void gnuplot_print_tiles(std::ostream& out, 
			 vector<Triangle>::iterator points_begin, 
			 vector<Triangle>::iterator points_end);

template
void gnuplot_print_tiles(std::ostream& out, 
			 list<Triangle>::iterator points_begin, 
			 list<Triangle>::iterator points_end);

template
void gnuplot_print_tiles(std::ostream& out, 
			 Triangle* points_begin, 
			 Triangle* points_end);

typedef boost::shared_ptr<Triangle> Triangle_handle;

template
void gnuplot_print_tiles(std::ostream& out, 
			 vector<Triangle_handle>::iterator points_begin, 
			 vector<Triangle_handle>::iterator points_end);

template
void gnuplot_print_tiles(std::ostream& out, 
			 list<Triangle_handle>::iterator points_begin, 
			 list<Triangle_handle>::iterator points_end);

template
void gnuplot_print_tiles(std::ostream& out, 
			 list<Triangle_handle>::const_iterator points_begin, 
			 list<Triangle_handle>::const_iterator points_end);

template
void gnuplot_print_tiles(std::ostream& out, 
			 Triangle_handle* points_begin, 
			 Triangle_handle* points_end);

void line_print(std::ostream& out, const std::list<Segment_3>& lines)
{
  size_t num_lines = lines.size();
  size_t num_verts = num_lines * 2;
  out << num_verts << " " << num_lines << endl;
  
  for (list<Segment_3>::const_iterator it = lines.begin(); it != lines.end(); ++it)
  {
    out << it->source() << " 0 0 0" << endl;
    out << it->target() << " 0 0 0" << endl;
  }
  int index = 0;
  for (list<Segment_3>::const_iterator it = lines.begin(); it != lines.end(); ++it)
  {
    out << index++ << " " << index++ << endl;
  }
}

void line_print(std::ostream& out, const std::list<Segment_3_undirected>& lines)
{
  size_t num_lines = lines.size();
  size_t num_verts = num_lines * 2;
  out << num_verts << " " << num_lines << endl;
  
  for (list<Segment_3_undirected>::const_iterator it = lines.begin(); it != lines.end(); ++it)
  {
    out << it->segment().source() << " 0 0 0" << endl;
    out << it->segment().target() << " 0 0 0" << endl;
  }
  int index = 0;
  for (list<Segment_3_undirected>::const_iterator it = lines.begin(); it != lines.end(); ++it)
  {
    out << index++ << " " << index++ << endl;
  }
}

void gnuplot_print_faces_2(std::ostream& out, 
			    CGAL::Straight_skeleton_2<Kernel>::Face_iterator faces_begin, 
			    CGAL::Straight_skeleton_2<Kernel>::Face_iterator faces_end)
{
  typedef CGAL::Straight_skeleton_2<Kernel> Ss;
  typedef Ss::Face_iterator Face_iterator;
  typedef Ss::Halfedge_handle   Halfedge_handle;
  typedef Ss::Vertex_handle     Vertex_handle;

  for (Face_iterator fi = faces_begin; fi != faces_end; ++fi)
  {
    Halfedge_handle halfedge = fi->halfedge();
    Halfedge_handle first = halfedge;
    do
    {
      Vertex_handle s = halfedge->opposite()->vertex();
      Vertex_handle t = halfedge->vertex();
      const Point_2& sp(s->point());
      const Point_2& tp(t->point());
      sp.insert(out) << endl;
      tp.insert(out) << endl;
//       out << sp << endl;
//       out << tp << endl;
      out << endl << endl;

//       // Add polygon vertices to triangulation
//       CDT::Vertex_handle ds = cdt.insert(s->point());
//       CDT::Vertex_handle dt = cdt.insert(t->point());
//       ds->info() = s->is_contour();
//       dt->info() = t->is_contour();
//       cdt.insert_constraint(ds, dt);
      
      halfedge = halfedge->next();
    } while (halfedge != first);
  }
}

void print_ser(std::ostream& out, const Slice& slice, Number_type thickness)
{
  list<string> components;
  slice.components(back_inserter(components));

  out << "<?xml version=\"1.0\"?>" << endl;
  out << "<!DOCTYPE Section SYSTEM \"section.dtd\">" << endl;
  out << endl;
  out << "<Section index=\"87\" thickness=\"" << thickness << "\" alignLocked=\"true\">" << endl;

  for (list<string>::const_iterator it = components.begin(); it != components.end(); ++it) {
    string component(*it);

    out << "<Transform dim=\"0\"" << endl;
    out << "xcoef=\" 0 1 0 0 0 0\"" << endl;
    out << "ycoef=\" 0 0 1 0 0 0\">" << endl;

    typedef Slice::Contour_const_iterator C_iter;
    for (C_iter c_it = slice.begin(component); c_it != slice.end(component); ++c_it) {
      Contour_handle contour = *c_it;
      const Polygon_2& polygon = contour->polygon();

      out << "<Contour name=\"" << component << "\" hidden=\"true\" closed=\"true\" simplified=\"true\" border=\"0 1 0\" fill=\"0 1 0\" mode=\"9\"" << endl;
      out << "points=\"";

      // insert points here
      Polygon_2::Vertex_const_iterator  vit;
      for (vit = polygon.vertices_begin(); vit != polygon.vertices_end(); ++vit) {
	out << "\t" << vit->x() << " " << vit->y() << "," << std::endl;
      }

      out << "\t\"/>" << endl;
    }
    out << "</Transform>" << endl;
    out << endl;
  }

  out << "</Section>" << endl;
  out << endl;
  out << endl;
}

void print_ser(std::ostream& out, const Polygon_2& polygon, const string& component)
{
  out << "<Contour name=\"" << component << "\" hidden=\"true\" closed=\"true\" simplified=\"true\" border=\"0 1 0\" fill=\"0 1 0\" mode=\"9\"" << endl;
  out << "points=\"";

  // insert points here
  Polygon_2::Vertex_const_iterator  vit;
  for (vit = polygon.vertices_begin(); vit != polygon.vertices_end(); ++vit) {
    out << "\t" << vit->x() << " " << vit->y() << "," << std::endl;
  }

  out << "\t\"/>" << endl;
}

// void print_ser(std::ostream& out, const Slice2& slice, Number_type thickness)
// {
//   // list<string> components;
//   // slice.components(back_inserter(components));

//   out << "<?xml version=\"1.0\"?>" << endl;
//   out << "<!DOCTYPE Section SYSTEM \"section.dtd\">" << endl;
//   out << endl;
//   out << "<Section index=\"87\" thickness=\"" << thickness << "\" alignLocked=\"true\">" << endl;

//   for (Slice2::const_iterator it = slice.begin(); it != slice.end(); ++it) {
//   // for (list<string>::const_iterator it = components.begin(); it != components.end(); ++it) {
//     // string component(*it);
//     string component(it->first);
//     const Contour2_handle contour = it->second;

//     out << "<Transform dim=\"0\"" << endl;
//     out << "xcoef=\" 0 1 0 0 0 0\"" << endl;
//     out << "ycoef=\" 0 0 1 0 0 0\">" << endl;

//     for (Contour2::Polygon_iterator pit = contour->begin(); pit != contour->end(); ++pit) {
//       print_ser(out, pit->outer_boundary(), component);
//       for (Polygon_with_holes_2::Hole_iterator pwhit = pit->holes_begin(); pwhit != pit->holes_end(); ++pwhit) {
// 	print_ser(out, *pwhit, component);
//       }
//     }

//     // typedef Slice::Contour_const_iterator C_iter;
//     // for (C_iter c_it = slice.begin(component); c_it != slice.end(component); ++c_it) {
//     //   Contour_handle contour = *c_it;
//     //   const Polygon_2& polygon = contour->polygon();

//     //   out << "<Contour name=\"" << component << "\" hidden=\"true\" closed=\"true\" simplified=\"true\" border=\"0 1 0\" fill=\"0 1 0\" mode=\"9\"" << endl;
//     //   out << "points=\"";

//     //   // insert points here
//     //   Polygon_2::Vertex_const_iterator  vit;
//     //   for (vit = polygon.vertices_begin(); vit != polygon.vertices_end(); ++vit) {
//     // 	out << "\t" << vit->x() << " " << vit->y() << "," << std::endl;
//     //   }

//     //   out << "\t\"/>" << endl;
//     // }
//     out << "</Transform>" << endl;
//     out << endl;
//   }

//   out << "</Section>" << endl;
//   out << endl;
//   out << endl;
// }

CONTOURTILER_END_NAMESPACE
