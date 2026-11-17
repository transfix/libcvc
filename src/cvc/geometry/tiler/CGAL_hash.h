#ifndef __CGAL_UTILS_H__
#define __CGAL_UTILS_H__

#include <boost/functional/hash.hpp>

// #include <CGAL/Gmpz.h>
#include <CGAL/Extended_cartesian.h>
#include <CGAL/Filtered_extended_homogeneous.h>
#include <CGAL/Lazy_exact_nt.h>

#include <ContourTiler/common.h>
// #include "CppUnitLite2/CppUnitLite2.h"

using namespace CONTOURTILER_NAMESPACE;
using namespace std;

// Note: CGAL 5.6+ provides built-in hash functions for Point_2, Point_3, Segment_2, Segment_3
// The custom implementations below conflict with CGAL's standard ones, so they are disabled.
// If needed for custom kernels, use a different namespace or function name.

namespace CGAL {
//   std::size_t hash_value(const CGAL::Point_2<CGAL::Cartesian<double> >& point);

  // Disabled - conflicts with CGAL 5.6+ built-in hash functions
  // template <typename Kernel>
  // std::size_t hash_value(const CGAL::Point_2<Kernel>& point);

  // template <typename Kernel>
  // std::size_t hash_value(const CGAL::Point_3<Kernel>& point);

//   std::size_t hash_value(const CGAL::Point_3<CGAL::Cartesian<double> >& point);
  // template <typename Kernel>
  // std::size_t hash_value(const CGAL::Segment_3<Kernel>& segment);

  // template <typename Kernel>
  // std::size_t hash_value(const CGAL::Segment_2<Kernel>& segment);

//   std::size_t hash_value(const CGAL::Segment_3<CGAL::Cartesian<double> >& segment);
//   std::size_t hash_value(const CGAL::Segment_3<Cartesian_25<double> >& segment);

  
}

// CONTOURTILER_BEGIN_NAMESPACE

// struct CGAL_hash
// {
// //   std::size_t operator()(Contour_handle contour) const
// //   {
// //     return boost::hash_value(contour.get());
// //   }
//   std::size_t operator()(const CGAL::Point_2<CGAL::Cartesian<double> >& point) const
//   {
//     std::size_t seed = 0;
//     boost::hash_combine(seed, point.x());
//     boost::hash_combine(seed, point.y());
//     return seed;
//   }
//   std::size_t operator()(const CGAL::Point_3<CGAL::Cartesian<double> >& point) const
//   {
//     std::size_t seed = 0;
//     boost::hash_combine(seed, point.x());
//     boost::hash_combine(seed, point.y());
//     boost::hash_combine(seed, point.z());
//     return seed;
//   }
//   std::size_t operator()(const CGAL::Segment_3<CGAL::Cartesian<double> >& segment) const
//   {
//     std::size_t seed = 0;
//     boost::hash_combine(seed, segment.source());
//     boost::hash_combine(seed, segment.target());
//     return seed;
//   }
// };


// CONTOURTILER_END_NAMESPACE

#endif
