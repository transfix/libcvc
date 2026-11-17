#include <limits>

#include <boost/functional/hash.hpp>

#include <ContourTiler/common.h>
#include <ContourTiler/Point_25.h>
#include <ContourTiler/Cartesian_25.h>

CONTOURTILER_BEGIN_NAMESPACE

// template<> size_t Point_25_<Cartesian_25<double> >::DEFAULT_ID = std::numeric_limits<std::size_t>::max();

template <typename Kernel>
std::size_t hash_value(const Point_25_<Kernel>& point)
{
  std::size_t seed = 0;
  boost::hash_combine(seed, point.x());
  boost::hash_combine(seed, point.y());
  boost::hash_combine(seed, point.z());
  return seed;
}

template std::size_t hash_value<Cartesian_25<double> >(const Point_25_<Cartesian_25<double> >& point);

CONTOURTILER_END_NAMESPACE
