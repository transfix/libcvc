#ifndef __PROJECTION_H__
#define __PROJECTION_H__

#include <ContourTiler/common.h>

CONTOURTILER_BEGIN_NAMESPACE

/// Projects the 3D segment onto the plane with the z axis as the normal.
Segment_2 projection_z(const Segment_3& chord);

/// Projects the 3D segment onto the plane with the x axis as the normal.
Segment_2 projection_x(const Segment_3& chord);

/// Projects the 3D segment onto the plane with the y axis as the normal.
Segment_2 projection_y(const Segment_3& chord);

CONTOURTILER_END_NAMESPACE

#endif
