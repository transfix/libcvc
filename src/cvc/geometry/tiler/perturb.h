#ifndef __PERTURB_H__
#define __PERTURB_H__

#include <ContourTiler/common.h>

CONTOURTILER_BEGIN_NAMESPACE

class Slice;

extern const Number_type DEFAULT_PERTURB_EPSILON;

Number_type perturb(Number_type d, Number_type epsilon);
void perturb(Polygon_2& p, Number_type epsilon);
void perturb(Polygon_with_holes_2& p, Number_type epsilon);
void perturb(Slice& slice, Number_type epsilon);


CONTOURTILER_END_NAMESPACE

#endif
