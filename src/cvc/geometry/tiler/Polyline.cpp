#include <ContourTiler/Polyline.h>
#include <ContourTiler/print_utils.h>

CONTOURTILER_BEGIN_NAMESPACE

template <typename Point>
void Polyline<Point>::check_valid() const
{
//   for (const_iterator it = begin(); it != end(); ++it)
//   {
//     if (find(it+1, end(), *it) != end())
//       throw logic_error("Duplicate point found in polyline: " + pp(*this));
//   }
}

template void Polyline<Point_2>::check_valid() const;
template void Polyline<Point_3>::check_valid() const;


CONTOURTILER_END_NAMESPACE

