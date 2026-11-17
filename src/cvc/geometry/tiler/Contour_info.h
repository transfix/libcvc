#ifndef __CONTOUR_INFO_H__
#define __CONTOUR_INFO_H__

// #include <SeriesFileReader.h>
#include <ContourTiler/common.h>

CONTOURTILER_BEGIN_NAMESPACE

//--------------------------------------------------------
// Contour_info class
//--------------------------------------------------------
template <typename Slice_t = Number_type>
class Contour_info
{
public:
  typedef Slice_t Slice_type;

public:
  Contour_info() : _slice(0), _name(""), _object_name("") {}
  Contour_info(Slice_t slice, const std::string& name, const std::string& object_name) 
    : _slice(slice), _name(name), _object_name(object_name) {}
//   Contour_info(SurfRecon::CurvePtr cp) : _slice((int)cp->get<0>()), _name(cp->get<3>()), _object_name(cp->get<3>()) {}
  ~Contour_info() {}

  Slice_t& slice() { return _slice; }
  Slice_t slice() const { return _slice; }
  std::string& name() { return _name; }
  const std::string& name() const { return _name; }

  /// The object name is the name of the object of which this
  /// contour is part.  For example, a dendrite d001 is composed of many
  /// contours.  Each of these contours would have "d001" as their object
  /// name.
  std::string& object_name() { return _object_name; }

  /// The object name is the name of the object of which this
  /// contour is part.  For example, a dendrite d001 is composed of many
  /// contours.  Each of these contours would have "d001" as their object
  /// name.
  const std::string& object_name() const { return _object_name; }

private:
  Slice_t _slice;
  std::string _name;
  std::string _object_name;
};

template <typename Slice_t>
std::ostream& operator<<(std::ostream& out, const Contour_info<Slice_t>& info)
{
  out << "\"" << info.name() << "\"  slice " << info.slice();
  return out;
}

template <typename Slice_t>
std::size_t hash_value(const Contour_info<Slice_t>& info)
{
  std::size_t seed = 0;
  boost::hash_combine(seed, info.name());
  boost::hash_combine(seed, info.slice());
  return seed;
}

template <typename Slice_t>
bool operator==(const Contour_info<Slice_t>& a, const Contour_info<Slice_t>& b)
{
  return a.name() == b.name() && a.slice() == b.slice();
}

CONTOURTILER_END_NAMESPACE

#endif
