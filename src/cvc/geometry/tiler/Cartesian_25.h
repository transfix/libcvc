#ifndef MYKERNEL_H
#define MYKERNEL_H

#include <CGAL/Cartesian.h>
#include <CGAL/Kernel/function_objects.h>

#include <ContourTiler/Point_25.h>
#include <ContourTiler/Segment_25.h>
#include <ContourTiler/Point_3_id.h>
#include <ContourTiler/Segment_3_id.h>

CONTOURTILER_BEGIN_NAMESPACE

// template <typename K>
// class Cartesian_25_Equal_2 : public CGAL::CommonKernelFunctors::Equal_2<K>
// {
//   typedef typename K::Point_2       Point_2;
//   typedef typename K::Vector_2      Vector_2;
//   typedef typename K::Direction_2   Direction_2;
//   typedef typename K::Segment_2     Segment_2;
//   typedef typename K::Ray_2         Ray_2;
//   typedef typename K::Line_2        Line_2;
//   typedef typename K::Triangle_2    Triangle_2;
//   typedef typename K::Iso_rectangle_2 Iso_rectangle_2;
//   typedef typename K::Circle_2      Circle_2;
// public:
//   typedef typename K::Bool_type     result_type;

// public:
//   result_type operator()(const Point_2 &p, const Point_2 &q) const
//   {
//     return xy_equal(p.rep(), q.rep());
// //     return p.rep() == q.rep();
//   }
//   result_type
//   operator()(const Vector_2 &v1, const Vector_2 &v2) const
//   {
//     return v1.rep() == v2.rep();
//   }

//   result_type
//   operator()(const Vector_2 &v, const CGAL::Null_vector &n) const
//   {
//     return v.rep() == n;
//   }

//   result_type
//   operator()(const Direction_2 &d1, const Direction_2 &d2) const
//   {
//     return d1.rep() == d2.rep();
//   }

//   result_type
//   operator()(const Segment_2 &s1, const Segment_2 &s2) const
//   {
//     return xy_equal(s1.source(), s2.source()) && xy_equal(s1.target(), s2.target());
//   }

//   result_type
//   operator()(const Line_2 &l1, const Line_2 &l2) const
//   {
//     return l1.rep() == l2.rep();
//   }

//   result_type
//   operator()(const Ray_2& r1, const Ray_2& r2) const
//   {
// //     return r1.source() == r2.source() && r1.direction() == r2.direction();
//     return xy_equal(r1.source(), r2.source()) && r1.direction() == r2.direction();
//   }

//   result_type
//   operator()(const Circle_2& c1, const Circle_2& c2) const
//   {
//     return xy_equal(c1.center(), c2.center()) &&
//       c1.squared_radius() == c2.squared_radius() &&
//       c1.orientation() == c2.orientation();
//   }

//   result_type
//   operator()(const Triangle_2& t1, const Triangle_2& t2) const
//   {
//     int i;
//     for(i=0; i<3; i++)
//       if ( xy_equal(t1.vertex(0), t2.vertex(i)) )
// 	break;

//     return (i<3) && xy_equal(t1.vertex(1), t2.vertex(i+1))
//       && xy_equal(t1.vertex(2), t2.vertex(i+2));
//   }

//   result_type
//   operator()(const Iso_rectangle_2& i1, const Iso_rectangle_2& i2) const
//   {
//     return (xy_equal((i1.min)(), (i2.min)())) && xy_equal(((i1.max)(), (i2.max)()));
//   }
// };

// K_ is the new kernel, and K_Base is the old kernel
template < typename K_, typename K_Base >
class MyCartesian_base
  : public K_Base::template Base<K_>::Type
{
public:
  typedef typename K_Base::template Base<K_>::Type   OldK;
public:
  typedef K_                                Kernel;

  typedef Point_25_<Kernel>                         Point_2;
  typedef Segment_25_<Kernel>               Segment_2;
  typedef MyConstruct_point_2<Kernel, OldK>       Construct_point_2;
  typedef const double*                     Cartesian_const_iterator_2;
  typedef MyConstruct_coord_iterator<Kernel>        Construct_cartesian_const_iterator_2;
//   typedef MyConstruct_bbox_2<typename OldK::Construct_bbox_2>
  typedef MyConstruct_bbox_2<Kernel>
                                            Construct_bbox_2;
//   typedef Cartesian_25_Equal_2<Kernel>      Equal_2;

  typedef Point_3_id<Kernel>                         Point_3;
  typedef Segment_3_id<Kernel>               Segment_3;
  typedef MyConstruct_point_3_id<Kernel, OldK>       Construct_point_3;
  typedef const double*                     Cartesian_const_iterator_3;
  typedef MyConstruct_coord_iterator_3<Kernel>        Construct_cartesian_const_iterator_3;
  typedef MyConstruct_bbox_3<Kernel>  Construct_bbox_3;

  Construct_point_2
  construct_point_2_object() const
  { return Construct_point_2(); }

  Construct_bbox_2 construct_bbox_2_object() const
  { return Construct_bbox_2(); }

//   Equal_2 equal_2_object() const
//   { return Equal_2(); }

  Construct_point_3
  construct_point_3_object() const
  { return Construct_point_3(); }

  Construct_bbox_3 construct_bbox_3_object() const
  { return Construct_bbox_3(); }

  template < typename Kernel2 >
  struct Base { typedef MyCartesian_base<Kernel2, K_Base>  Type; };
};


template < typename FT_ >
struct Cartesian_25
  : public CGAL::Type_equality_wrapper<
  MyCartesian_base<Cartesian_25<FT_>, CGAL::Cartesian<FT_> >,
  Cartesian_25<FT_> >
{};

CONTOURTILER_END_NAMESPACE

#endif // MYKERNEL_H
