#ifndef __XY_PRED_H__
#define __XY_PRED_H__

#include <ContourTiler/common.h>

CONTOURTILER_BEGIN_NAMESPACE

class xy_pred
{
public:
  xy_pred(const Point_2& p) : _p(p) {}
  bool operator()(const Point_2& q) { return xy_equal(_p, q); }
  Point_2 _p;
};

CONTOURTILER_END_NAMESPACE

#endif
