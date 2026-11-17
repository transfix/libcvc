#include <ContourTiler/Correspondences.h>

CONTOURTILER_BEGIN_NAMESPACE

Correspondences::Correspondences() 
{
}

Correspondences::~Correspondences()
{
}
  
/// Adds a correspondence between the two given contours
void Correspondences::add(Contour_handle c1, Contour_handle c2)
{
  _corr[c1].push_back(c2);
  _corr[c2].push_back(c1);
}

Correspondences::iterator Correspondences::begin(Contour_handle contour)
{
  return get(contour).begin();
}

Correspondences::const_iterator Correspondences::begin(Contour_handle contour) const
{
  return get(contour).begin();
}

Correspondences::iterator Correspondences::end(Contour_handle contour)
{
  return get(contour).end();
}

Correspondences::const_iterator Correspondences::end(Contour_handle contour) const
{
  return get(contour).end();
}

size_t Correspondences::count(Contour_handle contour) const
{ return get(contour).size(); }

const Correspondences::Container& Correspondences::get(Contour_handle contour) const
{
  Map::const_iterator it = _corr.find(contour);
  if (it == _corr.end())
    return _empty;
  return it->second;
}

Correspondences::Container& Correspondences::get(Contour_handle contour)
{
  Map::iterator it = _corr.find(contour);
  if (it == _corr.end())
    return _empty;
  return it->second;
}

CONTOURTILER_END_NAMESPACE
