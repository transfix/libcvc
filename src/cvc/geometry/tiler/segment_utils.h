#ifndef __SEGMENT_UTILS_H__
#define __SEGMENT_UTILS_H__

#include <ContourTiler/common.h>

CONTOURTILER_BEGIN_NAMESPACE

Segment_2 lexicographically_ordered(const Segment_2& s);
Segment_3 lexicographically_ordered(const Segment_3& s);
bool equal_ignore_order(const Segment_3& a, const Segment_3& b);
bool equal_ignore_order(const Segment_2& a, const Segment_3& b);

Segment_3 project_3(const Segment_2& s);
Segment_2 project_2(const Segment_3& s);

CONTOURTILER_END_NAMESPACE

#endif
