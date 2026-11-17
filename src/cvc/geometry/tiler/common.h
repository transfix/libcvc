#ifndef __COMMON_H__
#define __COMMON_H__

#include <ContourTiler/config.h>

#include <limits>
#define DEFAULT_ID() std::numeric_limits<std::size_t>::max()

#include <iostream>

#include <log4cplus/logger.h>
#include <log4cplus/loggingmacros.h>

// #include <CGAL/Exact_predicates_exact_constructions_kernel_with_sqrt.h>
// #include <CGAL/Gmpz.h>
#include <CGAL/Extended_cartesian.h>
#include <CGAL/Filtered_extended_homogeneous.h>
// #include <CGAL/Lazy_exact_nt.h>
#include <CGAL/Cartesian.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_with_holes_2.h>

#include <ContourTiler/Cartesian_25.h>

CONTOURTILER_BEGIN_NAMESPACE

// #define CONTOUR_EXACT_ARITHMETIC

// class My_nt : public CGAL::Interval_nt<true>
// {
// private:
//   typedef CGAL::Interval_nt<true> Base;
//   static const double _epsilon = 0.000001;
// public:
//   My_nt(int i) : Base(i-_epsilon, i+_epsilon) {}
//   My_nt(double d) : Base(d-_epsilon, d+_epsilon) {}
//   My_nt(double i, double s) : Base(i, s) {}
//   My_nt(std::pair<double, double> p) : Base(p) {}
// };

#ifdef CONTOUR_EXACT_ARITHMETIC

// // Doesn't seem to be working too well
// // typedef CGAL::Interval_nt<true> Number_type;
// // #define TO_NT(d) Number_type(d-0.000001, d+0.000001)

// // Works but slow
// typedef CGAL::Lazy_exact_nt<CGAL::Exact_predicates_exact_constructions_kernel_with_sqrt::FT>   Number_type;

// // Don't work
// // typedef CGAL::Lazy_exact_nt<CGAL::Gmpq>   Number_type;
// // typedef CGAL::Gmpq   Number_type;
// // typedef CGAL::Exact_predicates_exact_constructions_kernel_with_sqrt::FT   Number_type;
// // typedef CGAL::MP_Float   Number_type;
#else
typedef double                            Number_type;
#define TO_NT(d) d
#endif

// typedef CGAL::Cartesian<Number_type>      Kernel;
typedef Cartesian_25<Number_type>         Kernel;
typedef Kernel                            PolygonTraits;

// Many of the algorithms in contour tiler depend on
// the points of the polygon being in a random-access
// container, so don't change the container to a list.
typedef CGAL::Polygon_2<PolygonTraits>  Polygon_2;
typedef CGAL::Polygon_with_holes_2<PolygonTraits>  Polygon_with_holes_2;
typedef Kernel::Point_2                 Point_2;
typedef Kernel::Point_3                 Point_3;
typedef Kernel::Segment_2               Segment_2;
typedef Kernel::Segment_3               Segment_3;
typedef Kernel::Ray_2                   Ray_2;
typedef Kernel::Ray_3                   Ray_3;
typedef Kernel::Line_2                  Line_2;

// BOOST_MPL_ASSERT(( boost::is_same<Point_25_<Kernel>, Kernel_25<Number_type>::Point_2> ))
// BOOST_MPL_ASSERT(( boost::is_same<CGAL::Point_2<Kernel_25>, Cartesian_25<Number_type>::Point_2> ))

// Makes for more robust triangulations -- disabled zig-zag walks.
// See Triangulation_2.h in the CGAL directory.
#define CGAL_LFC_WALK

CONTOURTILER_END_NAMESPACE

#endif
