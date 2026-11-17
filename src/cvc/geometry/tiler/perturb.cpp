#include <ContourTiler/perturb.h>
#include <ContourTiler/Slice.h>

CONTOURTILER_BEGIN_NAMESPACE

const Number_type DEFAULT_PERTURB_EPSILON = 0.000000000001;

Number_type perturb(Number_type d, Number_type epsilon) {
  return d + epsilon * (rand()/(double)RAND_MAX);
}

void perturb(Polygon_2& p, Number_type epsilon) {
  Polygon_2::Vertex_iterator vit;
  for (vit = p.vertices_begin(); vit != p.vertices_end(); ++vit) {
    Point_2 pnt = *vit;
    // Make sure we get a consistent perturbation with each point across runs
    const static Number_type prime = 827;
    srand((int)(pnt.x() + prime*(pnt.y() + prime*pnt.z())));
    pnt = Point_25_<Kernel>(perturb(pnt.x(), epsilon), perturb(pnt.y(), epsilon), pnt.z(), pnt.id());
    p.set(vit, pnt);
  }

  p = Polygon_2(p.vertices_begin(), unique(p.vertices_begin(), p.vertices_end()));
}

void perturb(Polygon_with_holes_2& p, Number_type epsilon) {
  perturb(p.outer_boundary(), epsilon);
  // Polygon_with_holes_2 q(p.outer_boundary());
  for (Polygon_with_holes_2::Hole_iterator it = p.holes_begin(); it != p.holes_end(); ++it) {
    perturb(*it, epsilon);
    // q.add_hole();
  }
  // p = q;
}

void perturb(Slice& slice, Number_type epsilon) {
  list<string> components;
  slice.components(back_inserter(components));
  for (list<string>::const_iterator it = components.begin();
       it != components.end();
       ++it)
  {
    for (Slice::Contour_iterator cit = slice.begin(*it);
	 cit != slice.end(*it);
	 ++cit)
    {
      perturb((*cit)->polygon(), epsilon);
    }
  }
}

CONTOURTILER_END_NAMESPACE
