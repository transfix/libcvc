#ifndef __SET_UTILS_H__
#define __SET_UTILS_H__

#include <boost/unordered_set.hpp>

#include <ContourTiler/common.h>

CONTOURTILER_BEGIN_NAMESPACE

template <typename T, typename Out_iter>
void set_intersection(const boost::unordered_set<T>& a,
		      const boost::unordered_set<T>& b,
		      Out_iter results)
{
  typename boost::unordered_set<T>::const_iterator it = a.begin();
  for (; it != a.end(); ++it) {
    if (b.find(*it) != b.end())
      *results++ = *it;
  }
}

template <typename T>
boost::unordered_set<T> set_intersection(const boost::unordered_set<T>& a,
				     const boost::unordered_set<T>& b)
{
  boost::unordered_set<T> c;
  set_intersection(a, b, inserter(c, c.end()));
  return c;
}

template <typename T>
boost::unordered_set<T> set_union(const boost::unordered_set<T>& a,
				  const boost::unordered_set<T>& b)
{
  boost::unordered_set<T> c;
  c.insert(a.begin(), a.end());
  c.insert(b.begin(), b.end());
  return c;
}

template <typename T, typename Out_iter>
void set_union(const boost::unordered_set<T>& a,
		      const boost::unordered_set<T>& b,
		      Out_iter results)
{
  boost::unordered_set<T> u = set_union(a, b);
  typename boost::unordered_set<T>::const_iterator it = u.begin();
  for (; it != u.end(); ++it) {
    *results++ = *it;
  }
}

CONTOURTILER_END_NAMESPACE

#endif
