#ifndef __OTV_PAIR_H__
#define __OTV_PAIR_H__

#include <ContourTiler/common.h>

CONTOURTILER_BEGIN_NAMESPACE

class OTV_pair
{
public:
  OTV_pair() {}

  OTV_pair(const Point_3& u, const Point_3& v)
  {
    _top = u;
    _bottom = v;
    if (u.z() < v.z())
      std::swap(_top, _bottom);
  }

  ~OTV_pair() {}

  const Point_3& top() const { return _top; }
  Point_3& top() { return _top; }
  const Point_3& bottom() const { return _bottom; }
  Point_3& bottom() { return _bottom; }

private:
  Point_3 _top;
  Point_3 _bottom;
};

CONTOURTILER_END_NAMESPACE

#endif
