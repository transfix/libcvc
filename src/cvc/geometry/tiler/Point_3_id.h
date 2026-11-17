#ifndef __POINT_3_ID_H__
#define __POINT_3_ID_H__

#include <CGAL/Cartesian.h>
#include <CGAL/Origin.h>
#include <CGAL/Bbox_3.h>
#include <CGAL/Vector_3.h>

#include <ContourTiler/common.h>
#include <ContourTiler/Point_25.h>

CONTOURTILER_BEGIN_NAMESPACE

/// A CGAL Point_3 that also contains a unique id data member.  The client
/// is responsible for setting this id (it is not managed internal to
/// Point_3_id).
template <typename Kernel>
class Point_3_id : public Point_25_<Kernel> {
private:
  typedef typename Kernel::FT Number_type;
  typedef Point_25_<Kernel> base;

private:

public:

  Point_3_id() : base() {}
  Point_3_id(CGAL::Origin o) : base(o) {}
  Point_3_id(const Number_type x, const Number_type y, Number_type z = 0) : base(x, y, z) {}
  Point_3_id(const Number_type& x, const Number_type& y, const Number_type& z, const size_t& id) : base(x, y, z, id) {}

#ifdef CONTOUR_EXACT_ARITHMETIC
  Point_3_id(const double x, const double y, double z = 0) : base(x, y, z) {}
  Point_3_id(const double& x, const double& y, const double& z, const size_t& id) : base(x, y, z, id) {}
  Point_3_id(const float x, const float y, float z = 0) : base(x, y, z) {}
  Point_3_id(const float& x, const float& y, const float& z, const size_t& id) : base(x, y, z, id) {}
  Point_3_id(const int x, const int y, int z = 0) : base(x, y, z) {}
  Point_3_id(const int& x, const int& y, const int& z, const size_t& id) : base(x, y, z, id) {}
#endif

  Point_3_id<Kernel>& point()
  { return *this; }

  const Point_3_id<Kernel>& point() const
  { return *this; }

  Point_25_<Kernel> point_2() const
  { return Point_25_<Kernel>(this->x(), this->y(), this->z(), this->id()); }

  Point_3_id<Kernel> point_3() const
  {
    return *this;
  }

  bool operator==(const Point_3_id<Kernel> &p) const
  {
    return this->x() == p.x() && this->y() == p.y() && this->z() == p.z();
  }

  bool operator!=(const Point_3_id<Kernel> &p) const
  {
      return !(*this == p);
  }

  CGAL::Vector_3<Kernel> operator-(const Point_3_id<Kernel>& q)
  {
    const Point_3_id<Kernel>& p = *this;
    return CGAL::Vector_3<Kernel>(p.x() - q.x(), p.y() - q.y(), p.z() - q.z());
  }

//   friend bool xy_equal(const Point_3_id<Kernel>& a, const Point_3_id<Kernel>& b)
//   {
//     return a.x() == b.x() && a.y() == b.y();
//   }

//   friend bool xyz_equal(const Point_3_id<Kernel>& a, const Point_3_id<Kernel>& b)
//   {
//     return a.x() == b.x() && a.y() == b.y() && a.z() == b.z();
//   }

  std::ostream& insert(std::ostream &os) const
  {
    const Point_3_id &p(*this);
    switch(CGAL::IO::get_mode(os)) {
    case CGAL::IO::ASCII :
      return os << p.x() << ' ' << p.y() << ' ' << p.z();// << ' ' << p.color();
    case CGAL::IO::BINARY :
      CGAL::write(os, p.x());
      CGAL::write(os, p.y());
      CGAL::write(os, p.z());
      return os;
    default:
      return os << "Point_3_id(" << p.x() << ", " << p.y() /*<< ", " << p.color()*/ << ')';
    }
  }

  friend std::ostream &
  operator<<(std::ostream &os, const Point_3_id &p)
  {
    switch(CGAL::IO::get_mode(os)) {
    case CGAL::IO::ASCII :
      return os << p.x() << ' ' << p.y() << ' ' << p.z();
    case CGAL::IO::BINARY :
      CGAL::write(os, p.x());
      CGAL::write(os, p.y());
      CGAL::write(os, p.z());
      return os;
    default:
      return os << "Point_3_id(" << p.x() << ", " << p.y() << ", " << p.z() << ')';
    }
  }

  friend std::istream &
  operator>>(std::istream &is, Point_3_id &p)
  {
    double x, y, z;
    switch(CGAL::IO::get_mode(is)) {
    case CGAL::IO::ASCII :
      is >> x >> y >> z;
      break;
    case CGAL::IO::BINARY :
      CGAL::read(is, x);
      CGAL::read(is, y);
      CGAL::read(is, z);
      break;
    default:
      std::cerr << "" << std::endl;
      std::cerr << "Stream must be in ascii or binary mode" << std::endl;
      break;
    }
    if (is) {
      p = Point_3_id(x, y, z);
    }
    return is;
  }

};

// template <class ConstructBbox_3>
template <class Kernel>
// class MyConstruct_bbox_3 : public ConstructBbox_3 {
class MyConstruct_bbox_3 : public Kernel::OldK::Construct_bbox_3 {
public:
  CGAL::Bbox_3 operator()(const Point_3_id<Kernel>& p) const {
    return CGAL::Bbox_3(p.x(), p.y(), p.x(), p.y());
  }
//   CGAL::Bbox_3 operator()(const CGAL::Point_3<Kernel>& p) const {
//     return CGAL::Bbox_3(p.x(), p.y(), p.x(), p.y());
//   }
};

template <typename Kernel>
class MyConstruct_coord_iterator_3 {
private:
  typedef typename Kernel::FT Number_type;
public:
  const Number_type* operator()(const Point_3_id<Kernel>& p)
  {
    return &p.x();
  }

  const Number_type* operator()(const Point_3_id<Kernel>& p, int)
  {
    const Number_type* pyptr = &p.y();
    pyptr++;
    return pyptr;
  }
};

template <typename K, typename OldK>
class MyConstruct_point_3_id
{
  typedef typename K::RT         RT;
  typedef typename K::Point_3    Point_3;
  typedef typename K::Line_3     Line_3;
  typedef typename Point_3::Rep  Rep;
public:
  typedef Point_3                result_type;

  // Note : the CGAL::Return_base_tag is really internal CGAL stuff.
  // Unfortunately it is needed for optimizing away copy-constructions,
  // due to current lack of delegating constructors in the C++ standard.
  Rep // Point_3
  operator()(CGAL::Return_base_tag, CGAL::Origin o) const
  { return Rep(o); }

  Rep // Point_3
  operator()(CGAL::Return_base_tag, const RT& x, const RT& y) const
  { return Rep(x, y); }

  Rep // Point_3
  operator()(CGAL::Return_base_tag, const RT& x, const RT& y, const RT& w) const
  { return Rep(x, y, w); }

  Rep // Point_3
  operator()(CGAL::Return_base_tag, const RT& x, const RT& y, const RT& z, const size_t& id) const
  { return Rep(x, y, z, id); }

  Point_3
  operator()(CGAL::Origin o) const
  { return Point_3_id<K>(0, 0, 0); }

  Point_3
  operator()(const RT& x, const RT& y) const
  {
    return Point_3_id<K>(x, y, 0);
  }

  Point_3
  operator()(const Line_3& l) const
  {
    typename OldK::Construct_point_3 base_operator;
    Point_3 p = base_operator(l);
    return p;
  }

  Point_3
  operator()(const Line_3& l, int i) const
  {
    typename OldK::Construct_point_3 base_operator;
    return base_operator(l, i);
  }

  // We need this one, as such a functor is in the Filtered_kernel
  Point_3
  operator()(const RT& x, const RT& y, const RT& w) const
  {
    if(w != 1){
      return Point_3_id<K>(x/w, y/w, 0);
    } else {
      return Point_3_id<K>(x,y, 0);
    }
  }
};

template <typename Kernel>
std::size_t hash_value(const Point_3_id<Kernel>& point);

template <typename Kernel>
bool xy_equal(const Point_3_id<Kernel>& a, const Point_3_id<Kernel>& b)
{
  return a.x() == b.x() && a.y() == b.y();
}

template <typename Kernel>
bool xyz_equal(const Point_3_id<Kernel>& a, const Point_3_id<Kernel>& b)
{
  return a.x() == b.x() && a.y() == b.y() && a.z() == b.z();
}

CONTOURTILER_END_NAMESPACE

#endif
