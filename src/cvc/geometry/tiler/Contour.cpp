#include <ContourTiler/Contour.h>

CONTOURTILER_BEGIN_NAMESPACE

Contour::Handle Contour::create(const Polygon& polygon)
{
  return create(polygon, Info());
}

Contour::Handle Contour::create(const Polygon& polygon, const Info& info)
{
  Handle instance(new Contour(polygon, info));
  // Why can't I access _self from here?  Even though this function
  // is static it seems like I should still be able to...  Ah, the
  // mysteries of life.
  //     instance->_self = Self_handle(instance);
  instance->set_self(Self_handle(instance));
  return instance;
}

/// Create a copy of this contour
Contour::Handle Contour::copy()
{
  return create(polygon(), info());
}

/// Implementation detail.  Do not call.
void Contour::set_self(Self_handle self)
{ 
  if (self.lock().get() != this)
    throw std::logic_error("Illegal self handle");
  _self = self; 
}

Contour::Contour()
{
}

// Defined in print_utils.cpp
std::string pp(const Polygon_2& polygon);

Contour::Contour(const Polygon& polygon, const Info& info) : _polygon(polygon), _info(info)
{
  static log4cplus::Logger logger = log4cplus::Logger::getInstance("Contour.Contour");
  
  if (!polygon.is_simple()) {
    LOG4CPLUS_ERROR(logger, "Polygon is not simple in contour constructor " << info.object_name());
    LOG4CPLUS_ERROR(logger, "Polygon: " << pp(polygon));
    throw Contour_exception("Polygon is not simple in contour " + info.object_name(), info);
  }
}

Contour::~Contour()
{
}

std::ostream& operator<<(std::ostream& out, const Contour& contour)
{
  out << contour.polygon();
  return out;
}

std::ostream& operator<<(std::ostream& out, boost::shared_ptr<Contour> contour)
{
  out << *contour;
  return out;
}

// std::size_t hash_value(const Contour_handle& contour)
// {
//   return boost::hash_value(contour.get());
// }

// bool operator==(const Contour_handle& contour0, const Contour_handle& contour1)
// {
//   return contour0.get() == contour1.get();
// }

CONTOURTILER_END_NAMESPACE
