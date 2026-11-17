#include <ContourTiler/interp.h>
#include <ContourTiler/print_utils.h>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_2.h>

#include <CGAL/Interpolation_traits_2.h>
#include <CGAL/natural_neighbor_coordinates_2.h>
#include <CGAL/interpolation_functions.h>
#include <CGAL/Interpolation_gradient_fitting_traits_2.h>
#include <CGAL/sibson_gradient_fitting.h>

#include <CGAL/QP_models.h>
#include <CGAL/QP_functions.h>

#include <boost/unordered_map.hpp>

#include <iterator>

using namespace std;

CONTOURTILER_BEGIN_NAMESPACE

// struct K : CGAL::Exact_predicates_inexact_constructions_kernel {};
typedef Kernel K;

typedef CGAL::Delaunay_triangulation_2<K>             Delaunay_triangulation;
typedef CGAL::Interpolation_traits_2<K>               Traits;
typedef K::FT                                         Coord_type;
typedef K::Point_2                                    Point;
typedef std::map<Point, Coord_type, K::Less_xy_2>        Point_value_map ;
typedef std::map<Point, K::Vector_2 , K::Less_xy_2 >     Point_vector_map;

struct Z_access : public std::unary_function<Point_3,
		     std::pair<Number_type, bool> >
{
  typedef Number_type Data_type;
  typedef Point_3 Key_type; 
  std::pair<Data_type, bool> operator()(const Key_type& key) {
    return make_pair(key.z(), true);
  }
};

Number_type interpolate(const Polygon_2& P, const Point_3& point)
{
  return 0;
}

template <typename Poly_iter, typename Iter>
Number_type interpolate(Poly_iter poly_begin, Poly_iter poly_end, Iter points_begin, Iter points_end, Number_type zmid)
{
  log4cplus::Logger logger = log4cplus::Logger::getInstance("tiler.interpolate");

  Polygon_2 P(poly_begin, poly_end);
  LOG4CPLUS_TRACE(logger, "interpolate called with: " << pp(P));
  // P = remove_collinear(P, 0.000000001);
  // LOG4CPLUS_TRACE(logger, "interpolate called with: " << pp(P));
  // for (Iter it = points_begin; it != points_end; ++it) {
  //   LOG4CPLUS_TRACE(logger, "interpolate point: " << pp(*it));
  // }

  typedef CGAL::Data_access< std::map<Point, Coord_type, K::Less_xy_2 > >
                                            Value_access;
//   typedef CGAL::Gmpz ET;
  typedef CGAL::MP_Float ET;
  typedef CGAL::Quadratic_program<ET> Program;
  typedef CGAL::Quadratic_program_solution<ET> Solution;

  const Number_type nan = std::numeric_limits<Number_type>::quiet_NaN();
  typedef typename iterator_traits<Iter>::value_type point_type;

  Delaunay_triangulation known_T;
  Point_value_map function_values;
  Point_vector_map function_gradients;
  using namespace CGAL;

  // Adjust the z values slightly toward the middle to avoid any numerical
  // error causing interpolated points on the slices.
  // for (Poly_iter it = poly_begin; it != poly_end; ++it) {
  //   known_T.push_back(Point_3(it->x(), it->y(), 0.99 * it->z() + 0.01 * zmid));
  // }

  // No, don't adjust z values.
  // LOG4CPLUS_TRACE(logger, "creating triangulation of known points");
  for (Poly_iter it = poly_begin; it != poly_end; ++it) {
    // LOG4CPLUS_TRACE(logger, "triangulating - " << pp(*it));
    known_T.push_back(*it);
  }
  // known_T.insert(poly_begin, poly_end);
  // LOG4CPLUS_TRACE(logger, "  created triangulation of known points");

  boost::unordered_map<Point_2, int> columns;
  boost::unordered_map<Point_2, Number_type> c;

  // Linear program
  Program lp;
  
  int idx = 0;
  list<Point_2> points;
  for (Iter it = points_begin; it != points_end; ++it) {
    point_type& p = *it;
    p.z() = nan;
    points.push_back(Point_3(p.x(), p.y(), nan));
  }

  //coordinate computation
  // for (int j = 0; j < points.size(); ++j) {
  for (int j = 0; j < 1; ++j) {
    LOG4CPLUS_TRACE(logger, "outer loop. j = " << j << " of " << points.size());
    for (Iter it = points_begin; it != points_end; ++it) {
      LOG4CPLUS_TRACE(logger, "interpolating point: " << pp(*it));
      point_type& p = *it;

      Delaunay_triangulation T = known_T;
      // LOG4CPLUS_TRACE(logger, "  creating triangulation");
      for (Iter o_it = points_begin; o_it != points_end; ++o_it) {
	if (*o_it != p && o_it->z() == o_it->z()) {
	  // LOG4CPLUS_TRACE(logger, "    triangulation adding point: " << pp(*o_it));
	  T.push_back(Point_3(o_it->x(), o_it->y(), o_it->z()));
	}
      }

      std::vector< std::pair< Point, Coord_type > > coords;
      Coord_type norm =
	CGAL::natural_neighbor_coordinates_2
	(T, p, std::back_inserter(coords)).second;

      Coord_type res = CGAL::linear_interpolation(coords.begin(), coords.end(),
						   norm,
						   Z_access());
      LOG4CPLUS_TRACE(logger, "  interpolated point: " << pp(*it));
      p.z() = res;
    }
  }
  LOG4CPLUS_TRACE(logger, "done");
  return 0;
}

template
Number_type interpolate(Polygon_2::Vertex_iterator poly_begin, Polygon_2::Vertex_iterator poly_end, 
			list<Point_2>::iterator points_begin, list<Point_2>::iterator points_end, Number_type zmid);

template
Number_type interpolate(list<Point_2>::iterator poly_begin, list<Point_2>::iterator poly_end, 
			list<Point_2>::iterator points_begin, list<Point_2>::iterator points_end, Number_type zmid);

// template
// Number_type interpolate(const Polygon_2& P, list<Point_3>::iterator points_begin, list<Point_3>::iterator points_end);

CONTOURTILER_END_NAMESPACE
