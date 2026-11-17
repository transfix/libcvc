#ifndef __CORRESPONDENCES_H__
#define __CORRESPONDENCES_H__

#include <list>
#include <boost/unordered_map.hpp>
#include <ContourTiler/Contour.h>

CONTOURTILER_BEGIN_NAMESPACE

class Correspondences
{
private:
  typedef std::list<Contour_handle> Container;
  typedef boost::unordered_map<Contour_handle, Container> Map;

public:
  typedef Container::iterator iterator;
  typedef Container::const_iterator const_iterator;

public:
  Correspondences();
  ~Correspondences();
  
  /// Adds a correspondence between the two given contours
  void add(Contour_handle c1, Contour_handle c2);

  iterator begin(Contour_handle contour);
  const_iterator begin(Contour_handle contour) const;
  iterator end(Contour_handle contour);
  const_iterator end(Contour_handle contour) const;

  size_t count(Contour_handle contour) const;

private:
  Container& get(Contour_handle contour);
  const Container& get(Contour_handle contour) const;

private:
  Map _corr;
  Container _empty;
};


CONTOURTILER_END_NAMESPACE

#endif
