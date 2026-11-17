#include <ContourTiler/triangle_utils.h>
#include <ContourTiler/print_utils.h>

CONTOURTILER_BEGIN_NAMESPACE

void Triangle::check_valid() const
{
  static log4cplus::Logger logger = log4cplus::Logger::getInstance("Triangle.check_valid");

  // if (_points[0] == _points[1] || _points[1] == _points[2] || _points[0] == _points[2]) {
  //   // throw logic_error("Triangle is degenerate: " + pp_tri(*this));
  //   LOG4CPLUS_WARN(logger, "Triangle is degenerate: " << pp_tri(*this));
  // }
  // for (int i = 0; i < 3; ++i) {
  //   if (sqrt(CGAL::squared_distance(_points[i], _points[(i+1)%3])) < 0.0000000000001) {
  //     LOG4CPLUS_WARN(logger, "Triangle is nearly degenerate: " << pp_tri(*this));
  //     break;
  //   }
  // }
  for (int i = 0; i < 3; ++i) {
    if (_points[i].id() == 1826) {
      LOG4CPLUS_TRACE(logger, "Triangle with 1826: " << pp_tri(*this));
      break;
    }
  }
  // for (int i = 0; i < 3; ++i) {
  //   if (_points[i].id() == 0) {
  //     int id = _points[i].id();
  //   }
  // }
}

bool Triangle::is_degenerate() const
{
  for (int i = 0; i < 3; ++i) {
    if (xyz_equal(_points[i], _points[(i+1)%3])) {
      return true;
    }
  }
  return false;
}

std::string tri_pp(const Point_2& point)
{
  return pp(point);
}

std::string tri_pp(const Point_3& point)
{
  return pp(point);
}

std::string tri_pp(const Segment_2& segment)
{
  return pp(segment);
}

std::string tri_pp(const Segment_3& segment)
{
  return pp(segment);
}


CONTOURTILER_END_NAMESPACE

