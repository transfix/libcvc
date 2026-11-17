#include <stdexcept>

#include <ContourTiler/Composite_tiling_region.h>
#include <ContourTiler/print_utils.h>

CONTOURTILER_BEGIN_NAMESPACE

// Tiling_region Tiling_region::overlapping_vertex(const Point_2& p, const Point_2& q, const Point_2& r,
// 				   const Point_2& a, const Point_2& b, const Point_2& c)
// {
//   if (!xy_equal(q, b))
//     throw std::logic_error("overlapping_vertex requires that q == b");

//   Wedge LSV = Wedge::LS(p, q, r);
//   Wedge RSV = Wedge::RS(p, q, r);
//   Wedge LSVp = Wedge::LS(a, b, c);
//   Wedge RSVp = Wedge::RS(a, b, c);

//   Tiling_region L(LSV);
//   Tiling_region R(RSV);
//   Tiling_region Lp(LSVp);
//   Tiling_region Rp(RSVp);

// //   Tiling_region ret = !(Tiling_region(LSV & LSVp) | Tiling_region(RSV & RSVp));
//   if (LSV.is_empty_intersection(LSVp) && RSV.is_empty_intersection(RSVp)) {
// //     return Tiling_region(Wedge::EMPTY());
//     return Tiling_region::EMPTY();
//   }

//   if (LSV.is_empty_intersection(LSVp)) {
// //     return Tiling_region::EMPTY();
// //     return Tiling_region(Wedge::EMPTY());
// //     return Tiling_region(!Lp, Tiling_region(b), true);
// //     return Tiling_region(!R, Tiling_region(b), true);
// //     return !R;
// //     return !Rp;
//     return !(R & Rp);
//   }

//   if (RSV.is_empty_intersection(RSVp)) {
// //     return Tiling_region::EMPTY();
// //     return Tiling_region(Wedge::EMPTY());
// //     return Tiling_region(!Rp, Tiling_region(b), true);
// //     return Tiling_region(!L, Tiling_region(b), true);
// //     return !L;
// //     return !Lp;
//     return !(L & Lp);
//   }

//   return !(Tiling_region(L & Lp) | Tiling_region(R & Rp));
// }

// Tiling_region Tiling_region::backwards(const Point_2& p, const Point_2& q, const Point_2& r)
// {
//   Tiling_region LSV = Tiling_region::LS(p, q, r);
//   Tiling_region RSV = Tiling_region::RS(p, q, r);
//   return !RSV;
// }

Composite_tiling_region::Composite_tiling_region(Number_type z_home) : Tiling_region(z_home)
{ 
}

Composite_tiling_region::Composite_tiling_region(HTiling_region a, HTiling_region b, bool and_, Number_type z_home)
  : Tiling_region(z_home), _A(a), _B(b), _and(and_)
{ 
}

Composite_tiling_region::~Composite_tiling_region()
{
}

bool Composite_tiling_region::contains(const Point_3& point) const
{
  if (_and)
    return _A->contains(point) && _B->contains(point);
  return _A->contains(point) || _B->contains(point);
}

bool Composite_tiling_region::contains(const Segment_3& segment) const
{
  if (_and)
    return _A->contains(segment) && _B->contains(segment);
  return _A->contains(segment) || _B->contains(segment);
}

HTiling_region Composite_tiling_region::get_complement() const
{
  return HTiling_region(new Composite_tiling_region(_A->get_complement(), _B->get_complement(), !_and, z_home_nothrow()));
}

std::ostream& Composite_tiling_region::print(std::ostream& out) const
{
  out << *this;
  return out;
}

std::ostream& operator<<(std::ostream& out, const Composite_tiling_region& region)
{
  if (region._and) {
    out << "[";
    region._A->print(out);
    out << "] AND [";
    region._B->print(out);
    out << "]";
  } else {
    out << "[";
    region._A->print(out);
    out << "] OR [";
    region._B->print(out);
    out << "]";
  }
  return out;
}

// Tiling_region Tiling_region::get_intersection(const Tiling_region& region) const
// {
//   if (_empty)
//     throw std::logic_error("Don't know how to take the complement of empty");
//   return Tiling_region(*this, region, true);
// }

// Tiling_region Tiling_region::get_union(const Tiling_region& region) const
// {
//   if (_empty)
//     throw std::logic_error("Don't know how to take the complement of empty");
//   return Tiling_region(*this, region, false);
// }

// bool operator==(const Tiling_region& a, const Tiling_region& b)
// {
//   if (a._empty)
//     return b._empty;
//   if (a._wedge)
//   {
//     if (!b._wedge) return false;
//     return (*a._wedge) == (*b._wedge);
//   }
//   if (b._wedge) return false;

//   if (a._null_point)
//   {
//     if (!b._null_point) return false;
//     return (*a._null_point) == (*b._null_point);
//   }
//   if (b._null_point) return false;

//   return *(a._A) == *(b._A) && *(a._B) == *(b._B) && a._and == b._and;
// }

// std::ostream& operator<<(std::ostream& out, const Wedge& wedge)
// {
//   out <<    
//     "p = " << pp(wedge._ray2.point(1)) << 
//     " q = " << pp(wedge.vertex()) << 
//     " r = " << pp(wedge._ray1.point(1)) <<
//     " closed = " << wedge._closed;
//   return out;
// }

CONTOURTILER_END_NAMESPACE
