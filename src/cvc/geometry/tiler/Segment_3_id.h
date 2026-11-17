#ifndef __SEGMENT_3_ID_H__
#define __SEGMENT_3_ID_H__

#include <boost/config.hpp>

CONTOURTILER_BEGIN_NAMESPACE

template < class R_ >
class Segment_3_id
{
  typedef typename R_::FT                   FT;
  typedef typename R_::Point_3              Point_3;
  typedef typename R_::Vector_3             Vector_3;
  typedef typename R_::Direction_3          Direction_3;
  typedef typename R_::Line_3               Line_3;
  typedef typename R_::Segment_3            Segment_3;
  typedef typename R_::Aff_transformation_3 Aff_transformation_3;

  Point_3 sp_, tp_;
public:
  typedef R_                                     R;

  Segment_3_id() {}

  Segment_3_id(const Point_3 &sp, const Point_3 &tp)
    : sp_(sp), tp_(tp) {}

  bool        is_horizontal() const;
  bool        is_vertical() const;
  bool        has_on(const Point_3 &p) const;
  bool        collinear_has_on(const Point_3 &p) const;

  bool        operator==(const Segment_3_id &s) const;
  bool        operator!=(const Segment_3_id &s) const;

  const Point_3 &   source() const
  {
      return sp_;
  }

  const Point_3 &   target() const
  {
      return tp_;
  }
  const Point_3 &    start() const;
  const Point_3 &    end() const;

  const Point_3 &   min BOOST_PREVENT_MACRO_SUBSTITUTION () const;
  const Point_3 &   max BOOST_PREVENT_MACRO_SUBSTITUTION () const;
  const Point_3 &   vertex(int i) const;
  const Point_3 &   point(int i) const;
  const Point_3 &   operator[](int i) const;

  FT          squared_length() const;

  Direction_3 direction() const;
  Vector_3    to_vector() const;
  Line_3      supporting_line() const;
  Segment_3   opposite() const;

  Segment_3   transform(const Aff_transformation_3 &t) const
  {
    return Segment_3(t.transform(source()), t.transform(target()));
  }

  bool        is_degenerate() const;
  CGAL::Bbox_3      bbox() const;
};

template < class R >
inline
bool
Segment_3_id<R>::operator==(const Segment_3_id<R> &s) const
{
  return source() == s.source() && target() == s.target();
}

template < class R >
inline
bool
Segment_3_id<R>::operator!=(const Segment_3_id<R> &s) const
{
  return !(*this == s);
}


template < class R >
CGAL_KERNEL_INLINE
const typename Segment_3_id<R>::Point_3 &
Segment_3_id<R>::min BOOST_PREVENT_MACRO_SUBSTITUTION () const
{
  typename R::Less_xy_3 less_xy;
  return less_xy(source(),target()) ? source() : target();
}

template < class R >
CGAL_KERNEL_INLINE
const typename Segment_3_id<R>::Point_3 &
Segment_3_id<R>::max BOOST_PREVENT_MACRO_SUBSTITUTION() const
{
  typename R::Less_xy_3 less_xy;
  return less_xy(source(),target()) ? target() : source();
}

template < class R >
CGAL_KERNEL_INLINE
const typename Segment_3_id<R>::Point_3 &
Segment_3_id<R>::vertex(int i) const
{
  return (i%2 == 0) ? source() : target();
}

template < class R >
CGAL_KERNEL_INLINE
const typename Segment_3_id<R>::Point_3 &
Segment_3_id<R>::point(int i) const
{
  return (i%2 == 0) ? source() : target();
}

template < class R >
inline
const typename Segment_3_id<R>::Point_3 &
Segment_3_id<R>::operator[](int i) const
{
  return vertex(i);
}

template < class R >
CGAL_KERNEL_INLINE
typename Segment_3_id<R>::FT
Segment_3_id<R>::squared_length() const
{
  typename R::Compute_squared_distance_3 squared_distance;
  return squared_distance(source(), target());
}

template < class R >
CGAL_KERNEL_INLINE
typename Segment_3_id<R>::Direction_3
Segment_3_id<R>::direction() const
{
  typename R::Construct_vector_3 construct_vector;
  return Direction_3( construct_vector( source(), target()));
}

template < class R >
CGAL_KERNEL_INLINE
typename Segment_3_id<R>::Vector_3
Segment_3_id<R>::to_vector() const
{
  typename R::Construct_vector_3 construct_vector;
  return construct_vector( source(), target());
}

template < class R >
inline
typename Segment_3_id<R>::Line_3
Segment_3_id<R>::supporting_line() const
{
  typename R::Construct_line_3 construct_line;

  return construct_line(*this);
}

template < class R >
inline
typename Segment_3_id<R>::Segment_3
Segment_3_id<R>::opposite() const
{
  return Segment_3_id<R>(target(), source());
}

template < class R >
CGAL_KERNEL_INLINE
CGAL::Bbox_3
Segment_3_id<R>::bbox() const
{
  return source().bbox() + target().bbox();
}

template < class R >
inline
bool
Segment_3_id<R>::is_degenerate() const
{
  return R().equal_3_object()(source(), target());
}

template < class R >
CGAL_KERNEL_INLINE
bool
Segment_3_id<R>::is_horizontal() const
{
  return R().equal_y_3_object()(source(), target());
}

template < class R >
CGAL_KERNEL_INLINE
bool
Segment_3_id<R>::is_vertical() const
{
  return R().equal_x_3_object()(source(), target());
}

template < class R >
CGAL_KERNEL_INLINE
bool
Segment_3_id<R>::
has_on(const typename Segment_3_id<R>::Point_3 &p) const
{
  return R().collinear_are_ordered_along_line_3_object()(source(), p, target());
}

template < class R >
inline
bool
Segment_3_id<R>::
collinear_has_on(const typename Segment_3_id<R>::Point_3 &p) const
{
  return R().collinear_has_on_3_object()(*this, p);
}

template < class R >
std::ostream &
operator<<(std::ostream& os, const Segment_3_id<R>& s)
{
    switch(CGAL::IO::get_mode(os)) {
    case CGAL::IO::ASCII :
        return os << s.source() << ' ' << s.target();
    case CGAL::IO::BINARY :
        return os << s.source() << s.target();
    default:
        return os << "Segment_3_id(" << s.source() <<  ", " << s.target() << ")";
    }
}



template < class R >
std::istream &
operator>>(std::istream &is, Segment_3_id<R> &s)
{
    typename R::Point_3 p, q;

    is >> p >> q;

    if (is)
	s = Segment_3_id<R>(p, q);
    return is;
}

CONTOURTILER_END_NAMESPACE

#endif
