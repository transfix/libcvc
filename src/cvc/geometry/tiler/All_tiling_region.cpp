#include <stdexcept>

#include <ContourTiler/All_tiling_region.h>
#include <ContourTiler/print_utils.h>

CONTOURTILER_BEGIN_NAMESPACE

All_tiling_region::All_tiling_region(Number_type z_home) : Tiling_region(z_home), _empty(false)
{ 
}

All_tiling_region::All_tiling_region(bool empty, Number_type z_home) : Tiling_region(z_home), _empty(empty)
{ 
}

All_tiling_region::~All_tiling_region()
{
}

bool All_tiling_region::contains(const Point_3& point) const
{
  return !_empty;
}

bool All_tiling_region::contains(const Segment_3& segment) const
{
  return !_empty;
}

HTiling_region All_tiling_region::get_complement() const
{
  return HTiling_region(new All_tiling_region(!_empty));
}

std::ostream& All_tiling_region::print(std::ostream& out) const
{
  out << *this;
  return out;
}

std::ostream& operator<<(std::ostream& out, const All_tiling_region& region)
{
  if (region._empty)
    out << "EMPTY";
  else
    out << "ALL";
  return out;
}

CONTOURTILER_END_NAMESPACE
