#include <ContourTiler/Contour2.h>
#include <ContourTiler/arrange_polygons.h>

#include "boost/lexical_cast.hpp"

CONTOURTILER_BEGIN_NAMESPACE

Contour2::Handle Contour2::create()
{
  return create(Info());
}

Contour2::Handle Contour2::create(const Info& info)
{
  list<Polygon_2> polys;
  return create(polys.begin(), polys.end(), info);;
}

Contour2::Handle Contour2::create(const Polygon_2& polygon)
{
  return create(polygon, Info());
}

Contour2::Handle Contour2::create(const Polygon_2& polygon, const Info& info)
{
  list<Polygon_2> polys;
  polys.push_back(polygon);
  return create(polys.begin(), polys.end(), info);
}

// /// Create a copy of this contour
// Contour2::Handle Contour2::copy()
// {
//   return create(begin(), end(), info());
// }

/// Implementation detail.  Do not call.
void Contour2::set_self(Self_handle self)
{ 
  if (self.lock().get() != this)
    throw std::logic_error("Illegal self handle");
  _self = self; 
}

Contour2::Contour2()
{
}

// template <typename Poly_iter>
// Contour2::Contour2(Poly_iter begin, Poly_iter end, const Info& info) : _info(info)
// {
//   list<Polygon_with_holes_2> all;
//   arrange_polygons(begin, end, back_inserter(all));
// }

// // template Contour2::Contour2(list<Polygon_2>::const_iterator begin, list<Polygon_2>::const_iterator end, const Info& info);
// template Contour2::Contour2(list<Polygon_2>::iterator begin, list<Polygon_2>::iterator end, const Info& info);
// // template Contour2::Contour2(vector<Polygon_2>::const_iterator begin, vector<Polygon_2>::const_iterator end, const Info& info);
// template Contour2::Contour2(vector<Polygon_2>::iterator begin, vector<Polygon_2>::iterator end, const Info& info);

// Contour2::Contour2(const Polygon& polygon, const Info& info) : _polygon(polygon), _info(info)
// {
//   if (!polygon.is_simple())
//     throw Contour_exception("Polygon is not simple in contour " + info.object_name(), info);
// }

template <typename Poly_iter>
Contour2::Contour2(Poly_iter begin, Poly_iter end, const Info& info) : _info(info)
{
  try {
    arrange_polygons(begin, end, back_inserter(_polygons));
  }
  catch(exception& e) {
    throw runtime_error("Error creating contour of component " + info.name() + " at slice " + boost::lexical_cast<string>(info.slice()) + ": " + e.what());
  }
}

template Contour2::Contour2(list<Polygon_2>::iterator begin, list<Polygon_2>::iterator end, const Info& info);
template Contour2::Contour2(list<Polygon_2>::const_iterator begin, list<Polygon_2>::const_iterator end, const Info& info);
template Contour2::Contour2(vector<Polygon_2>::iterator begin, vector<Polygon_2>::iterator end, const Info& info);
template Contour2::Contour2(vector<Polygon_2>::const_iterator begin, vector<Polygon_2>::const_iterator end, const Info& info);

Contour2::~Contour2()
{
}

// std::ostream& operator<<(std::ostream& out, const Contour2& contour)
// {
//   out << contour.polygon();
//   return out;
// }

// std::ostream& operator<<(std::ostream& out, boost::shared_ptr<Contour2> contour)
// {
//   out << *contour;
//   return out;
// }

// std::size_t hash_value(const Contour2_handle& contour)
// {
//   return boost::hash_value(contour.get());
// }

// bool operator==(const Contour2_handle& contour0, const Contour2_handle& contour1)
// {
//   return contour0.get() == contour1.get();
// }

void Contour2::validate() const
{
  static log4cplus::Logger logger = log4cplus::Logger::getInstance("Contour2.validate");

  for (Polygon_const_iterator it = begin(); it != end(); ++it) {
    const Polygon_with_holes_2& pwh = *it;
    if (!pwh.outer_boundary().is_simple()) {
      LOG4CPLUS_ERROR(logger, "Outer boundary is not simple.  Component = " << info().object_name() 
		      << " Polygon = " << pp(pwh.outer_boundary()));
      throw logic_error("Contour is not simple");
    }
    for (Polygon_with_holes_2::Hole_const_iterator hit = pwh.holes_begin(); hit != pwh.holes_end(); ++hit) {
      const Polygon_2& p = *hit;
      if (!p.is_simple()) {
	LOG4CPLUS_ERROR(logger, "Hole is not simple.  Component = " << info().object_name() 
			<< " Polygon = " << pp(p));
	throw logic_error("Contour is not simple");
      }
    }
  }
}

CONTOURTILER_END_NAMESPACE
