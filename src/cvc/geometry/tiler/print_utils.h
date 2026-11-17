#ifndef CGAL_PRINT_UTILS_H
#define CGAL_PRINT_UTILS_H

#include <CGAL/Origin.h>
#include <CGAL/Polygon_with_holes_2.h>
#include <CGAL/Straight_skeleton_builder_2.h>

#include <iostream>
#include <ContourTiler/asso_insert_iterator.h>

#include <ContourTiler/Contour.h>
#include <ContourTiler/Contour2.h>
#include <ContourTiler/Tiles.h>
#include <ContourTiler/Tile.h>
#include <ContourTiler/tiler_defines.h>
#include <ContourTiler/Tiler_workspace.h>
#include <ContourTiler/Tiling_region.h>
#include <ContourTiler/Segment_3_undirected.h>
#include <ContourTiler/Untiled_region.h>
#include <ContourTiler/Polyline.h>
#include <ContourTiler/Colored_point.h>
#include <ContourTiler/Slice.h>
#include <ContourTiler/Slice2.h>
#include <ContourTiler/kernel_utils.h>

CONTOURTILER_BEGIN_NAMESPACE

void set_default_pp_precision(int precision);
void set_pp_precision(int precision);
void restore_pp_precision();

std::string pp(const Point_2& point);
inline std::string pp(const Bso_point_2& point) { return pp(to_common<Bso_kernel>(point, 0)); }
std::string pp(const Point_3& point);
std::string pp(const Segment_2& segment);
inline std::string pp(const Bso_segment_2& segment) { return pp(to_common<Bso_kernel>(segment, 0)); }
std::string pp(const Segment_3& segment);
inline std::string pp(const Segment_3_undirected& segment) { return pp(segment.segment()); }
std::string pp_id(const Segment_2& segment);
std::string pp_id(const Segment_3& segment);
inline std::string pp_id(const Segment_3_undirected& segment) { return pp_id(segment.segment()); }
std::string pp(const Point_3& v0, const Point_3& v1);
std::string pp(const Tiling_region& region);
std::string pp(const Polygon_2& polygon);
std::string pp(const Polygon_with_holes_2& pwh);
std::string pp(const Polyline_2& polyline);
std::string pp(const Polyline_3& polyline);
std::string pp(const Slice& slice);

template <typename Tri>
std::string pp_tri(const Tri& triangle)
{
  std::stringstream out;
//   out << std::setprecision(pp_precision);

  out << "[ ";
  for (int i = 0; i < 3; ++i)
    out << pp(triangle[i]);
  out << " ]";
  return out.str();
  
}

template <typename Curve>
std::string pp_curve(const Curve& curve)
{
  typename Curve::const_iterator vit;
  std::stringstream out;
//   out << std::setprecision(pp_precision);

  out << "[ ";
  for (vit = curve.begin(); vit != curve.end(); ++vit)
    out << pp(*vit);
  out << " ]";
  return out.str();
}

void gnuplot_print_otvs(std::ostream& out, const Tiler_workspace& w);

void gnuplot_print_otv_pairs(std::ostream& out, const Tiler_workspace& w);

// void gnuplot_print_tiles(std::ostream& out, const Tiles& tiles);

template <typename Tile_iter>
void gnuplot_print_tiles(std::ostream& out, Tile_iter tiles_begin, Tile_iter tiles_end);

// void gnuplot_print_tile(std::ostream& out, const Tile& t);

// void gnuplot_print_tile_filled(std::ostream& out, const Tile& t);

void gnuplot_print_vertices(std::ostream& out, const Vertices& vertices);

void gnuplot_print_polygon(std::ostream& out, const Polygon_2& P);

void gnuplot_print_polygon(std::ostream& out, const Polygon_with_holes_2& P);

//-----------------------------------------------------------------------------
// gnuplot print a CGAL polygon
//
// void gnuplot_print_polygon(std::ostream& out, const Polygon& P);

//-----------------------------------------------------------------------------
// gnuplot print a CGAL polygon
//
void gnuplot_print_polygon(std::ostream& out, const Polygon_2& P, int z);

void gnuplot_print_polygon(const std::string& filename, const Polygon_2& P);

//-----------------------------------------------------------------------------
// gnuplot print polygons
//
template<typename InputIterator>
void gnuplot_print_polygons(std::ostream& out, InputIterator beg, InputIterator end)
{
  using namespace std;

  for (InputIterator it = beg; it != end; ++it)
  {
    gnuplot_print_polygon(out, (*it));
    out << std::endl << std::endl;
  }
}

//-----------------------------------------------------------------------------
// gnuplot print a slice
//
template<typename InputIterator>
void gnuplot_print_slice(std::ostream& out, InputIterator contoursBegin, InputIterator contoursEnd, int z)
{
  InputIterator it = contoursBegin;
  for (; it != contoursEnd; ++it)
  {
    gnuplot_print_polygon(out, (*it)->polygon(), z);
    out << std::endl << std::endl;
  }

  return;
}

//-----------------------------------------------------------------------------
// gnuplot print contours from list of Contour_vertices
//
void gnuplot_print_polygons(std::ostream& out, const std::list<Untiled_region>& list);

//-----------------------------------------------------------------------------
// gnuplot print contours from list of Contour_vertices
//
void gnuplot_print_polygon(std::ostream& out, const Untiled_region& poly);

//-----------------------------------------------------------------------------
// gnuplot print contours
//
template<typename InputIterator>
void gnuplot_print_contours(std::ostream& out, InputIterator beg, InputIterator end)
{
  using namespace std;

  for (InputIterator it = beg; it != end; ++it)
  {
    int z = (int) CGAL::to_double((*it)->info().slice());
    gnuplot_print_polygon(out, (*it)->polygon(), z);
    out << std::endl << std::endl;
  }
}

//-----------------------------------------------------------------------------
// Pretty-print a set of contours
//
template<typename InputIterator>
void print_contours(InputIterator start, InputIterator end)
{
  using namespace std;
  using namespace CONTOURTILER_NAMESPACE;
  typedef typename iterator_traits<InputIterator>::value_type Contour_handle;
  typedef typename Contour_handle::element_type::Info_type::Slice_type Slice_type;

  set<Slice_type> slices;
  contour_slices(start, end, asso_inserter<set<Slice_type> >(slices));
  cout << "slices: ";
  std::ostream_iterator<Slice_type> slice_output(cout, ", ");
  std::copy(slices.begin(), slices.end(), slice_output);
  cout << endl;

  set<string> objects;
  contour_names(start, end, asso_inserter<set<string> >(objects));
  cout << "objects: ";
  std::ostream_iterator<string> output(cout, ", ");
  std::copy(objects.begin(), objects.end(), output);
  cout << endl;
}

//-----------------------------------------------------------------------------
// Pretty-print a Contour
//
void print_contour(Contour_handle contour);
// {
//   std::cout << contour->info() << std::endl;
//   print_polygon(contour->polygon());
// }

//-----------------------------------------------------------------------------
// Pretty-print a CGAL polygon.
//
template<class Kernel, class Container>
void print_polygon (const CGAL::Polygon_2<Kernel, Container>& P)
{
  typename CGAL::Polygon_2<Kernel, Container>::Vertex_const_iterator  vit;

  std::cout << "[ " << P.size() << " vertices:";
  for (vit = P.vertices_begin(); vit != P.vertices_end(); ++vit)
    std::cout << " (" << *vit << ')';
  std::cout << " ]" << std::endl;

  return;
}

//-----------------------------------------------------------------------------
// Pretty-print a polygon with holes.
//
template<class Kernel, class Container>
void print_polygon_with_holes
    (const CGAL::Polygon_with_holes_2<Kernel, Container>& pwh)
{
  if (! pwh.is_unbounded())
  {
    std::cout << "{ Outer boundary = ";
    print_polygon (pwh.outer_boundary());
  }
  else
    std::cout << "{ Unbounded polygon." << std::endl;

  typename CGAL::Polygon_with_holes_2<Kernel,Container>::
                                             Hole_const_iterator  hit;
  unsigned int                                                     k = 1;

  std::cout << "  " << pwh.number_of_holes() << " holes:" << std::endl;
  for (hit = pwh.holes_begin(); hit != pwh.holes_end(); ++hit, ++k)
  {
    std::cout << "    Hole #" << k << " = ";
    print_polygon (*hit);
  }
  std::cout << " }" << std::endl;

  return;
}

// void raw_print_tiles(std::ostream& out, const Tiles& tiles, double z_scale = 1);

template <typename Point_iter>
void raw_print_tiles_impl(std::ostream& out, Point_iter points_begin, Point_iter points_end, double z_scale, bool color);

template <typename Tri>
const Triangle& get_tri(const Tri& tri)
{ return tri; }

template <>
inline const Triangle& get_tri<::boost::shared_ptr<Triangle> >(const ::boost::shared_ptr<Triangle>& tri)
{ return *tri; }

template <typename Tile_iter>
void raw_print_tiles(std::ostream& out, Tile_iter tiles_begin, Tile_iter tiles_end, 
		     double z_scale = 1)
{
  list<Point_3> p;
  for (Tile_iter it = tiles_begin; it != tiles_end; ++it)
    for (int i = 0; i < 3; ++i)
    {
      const Triangle& tri = get_tri(*it);
      p.push_back(Point_3(vertex(i, tri)));
    }
  
  raw_print_tiles_impl(out, p.begin(), p.end(), z_scale, false);
}

template <typename Tile_iter>
void raw_print_tiles(std::ostream& out, Tile_iter tiles_begin, Tile_iter tiles_end, 
		     double r, double g, double b, double z_scale = 1)
{
  list<Colored_point_3> p;
  for (Tile_iter it = tiles_begin; it != tiles_end; ++it)
    for (int i = 0; i < 3; ++i)
    {
      const Triangle& tri = get_tri(*it);
      p.push_back(Colored_point_3(vertex(i, tri), r, g, b));
    }
  
  raw_print_tiles_impl(out, p.begin(), p.end(), z_scale, true);
}

// template <typename Tile_iter>
// void raw_print_tiles_noh(std::ostream& out, Tile_iter tiles_begin, Tile_iter tiles_end, 
// 		     double r, double g, double b, double z_scale = 1)
// {
//   list<Colored_point_3> p;
//   for (Tile_iter it = tiles_begin; it != tiles_end; ++it)
//     for (int i = 0; i < 3; ++i)
//       p.push_back(Colored_point_3(vertex(i, *it), r, g, b));
  
//   raw_print_tiles(out, p.begin(), p.end(), z_scale, true);
// }

void line_print(std::ostream& out, const std::list<Segment_3>& lines);

void line_print(std::ostream& out, const std::list<Segment_3_undirected>& lines);

void gnuplot_print_faces_2(std::ostream& out, 
			    CGAL::Straight_skeleton_2<Kernel>::Face_iterator faces_begin, 
			    CGAL::Straight_skeleton_2<Kernel>::Face_iterator faces_end);

void print_ser(std::ostream& out, const Slice& slice, Number_type thickness);
// void print_ser(std::ostream& out, const boost::unordered_map<std::string, Contour2_handle>& slice, Number_type thickness);
// void print_ser(std::ostream& out, const Slice2& slice, Number_type thickness);

template <typename Iter>
void gnuplot_points(std::ostream& out, Iter begin, Iter end)
{
  for (Iter it = begin; it != end; ++it) {
    out << it->x() << " " << it->y() << std::endl;
  }
}

CONTOURTILER_END_NAMESPACE

#endif
