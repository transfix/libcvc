#include <ContourTiler/projection.h>

CONTOURTILER_BEGIN_NAMESPACE

Segment_2 projection_z(const Segment_3& chord)
{
  return Segment_2(Point_2(chord.source().x(), chord.source().y()), 
		   Point_2(chord.target().x(), chord.target().y()));
}

Segment_2 projection_x(const Segment_3& chord)
{
  return Segment_2(Point_2(chord.source().y(), chord.source().z()), 
		   Point_2(chord.target().y(), chord.target().z()));
}

Segment_2 projection_y(const Segment_3& chord)
{
  return Segment_2(Point_2(chord.source().x(), chord.source().z()), 
		   Point_2(chord.target().x(), chord.target().z()));
}

CONTOURTILER_END_NAMESPACE
