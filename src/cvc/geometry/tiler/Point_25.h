#ifndef MY_POINTC2_H
#define MY_POINTC2_H

#include <CGAL/Cartesian.h>
#include <CGAL/Origin.h>
#include <CGAL/Bbox_2.h>

#include <ContourTiler/common.h>

CONTOURTILER_BEGIN_NAMESPACE

template <typename Kernel>
class Point_25_ {
private:
//   static size_t DEFAULT_ID;

  typedef typename Kernel::FT Number_type;

private:
  Number_type vec[3];
  size_t _id;

public:

  Point_25_() : _id(DEFAULT_ID())
  {
    *vec = 0;
    *(vec+1) = 0;
    *(vec+2) = 0;
  }

  Point_25_(CGAL::Origin o) : _id(DEFAULT_ID())
  {
    *vec = 0;
    *(vec+1) = 0;
    *(vec+2) = 0;
  }

  Point_25_(const Number_type x, const Number_type y, Number_type z = 0) : _id(DEFAULT_ID())
  {
    *vec = x;
    *(vec+1) = y;
    *(vec+2) = z;
  }

  Point_25_(const Number_type& x, const Number_type& y, const Number_type& z, const size_t& id) : _id(id)
  {
    *vec = x;
    *(vec+1) = y;
    *(vec+2) = z;
  }

#ifdef CONTOUR_EXACT_ARITHMETIC
  Point_25_(const double x, const double y, double z = 0) 
    : Point_25_(TO_NT(x), TO_NT(y), TO_NT(z)) {}
  Point_25_(const double& x, const double& y, const double& z, const size_t& id)
    : Point_25_(TO_NT(x), TO_NT(y), TO_NT(z), id) {}
  Point_25_(const float x, const float y, float z = 0) 
    : Point_25_(TO_NT(x), TO_NT(y), TO_NT(z)) {}
  Point_25_(const float& x, const float& y, const float& z, const size_t& id)
    : Point_25_(TO_NT(x), TO_NT(y), TO_NT(z), id) {}
  Point_25_(const int x, const int y, int z = 0) 
    : Point_25_(TO_NT(x), TO_NT(y), TO_NT(z)) {}
  Point_25_(const int& x, const int& y, const int& z, const size_t& id)
    : Point_25_(TO_NT(x), TO_NT(y), TO_NT(z), id) {}
#endif

  const Number_type& x() const  { return *vec; }
  const Number_type& y() const { return *(vec+1); }
  const Number_type& z() const { return *(vec+2); }
  const size_t& id() const { return _id; }
  const size_t& unique_id() const { return _id; }

  Number_type& x() { return *vec; }
  Number_type& y() { return *(vec+1); }
  Number_type& z() { return *(vec+2); }
  size_t& id() { return _id; }

  const Number_type& hx() const { return x(); }
  const Number_type& hy() const { return y(); }
  const Number_type& hz() const { return z(); }
  Number_type& hx() { return x(); }
  Number_type& hy() { return y(); }
  Number_type& hz() { return z(); }

  static size_t default_id() { return DEFAULT_ID(); }

  Point_25_<Kernel>& point()
  { return *this; }

  const Point_25_<Kernel>& point() const
  { return *this; }

//   Point_25_<Kernel>& point_2()
//   { return *this; }

//   const Point_25_<Kernel>& point_2() const
//   { return *this; }

  CGAL::Point_2<CGAL::Cartesian<typename Kernel::FT> > point_2() const
  {
    return CGAL::Point_2<CGAL::Cartesian<typename Kernel::FT> >(x(), y());
  }

  CGAL::Point_3<Kernel> point_3() const
  {
    // return CGAL::Point_3<Kernel>(x(), y(), z(), id());
    CGAL::Point_3<Kernel> p(x(), y(), z());
    p.id() = id();
    return p;
  }

  Point_25_<Kernel>& operator=(const CGAL::Point_3<Kernel>& p)
  {
    x() = p.x();
    y() = p.y();
    z() = p.z();
    id() = p.id();
    return *this;
  }

  bool operator==(const Point_25_<Kernel> &p) const
  {
    return x() == p.x() && y() == p.y();
  }

  bool operator!=(const Point_25_<Kernel> &p) const
  {
      return !(*this == p);
  }

  CGAL::Vector_2<Kernel> operator-(const Point_25_<Kernel>& q)
  {
    const Point_25_<Kernel>& p = *this;
    return CGAL::Vector_2<Kernel>(p.x() - q.x(), p.y() - q.y());
  }

  operator CGAL::Point_2<Kernel>() const
  {
//     return CGAL::Point_2<Kernel>(x(), y(), z(), id());
    return Point_25_<Kernel>(x(), y(), z(), id());
//     return point_2();
  }

  operator CGAL::Point_3<Kernel>() const
  {
    return point_3();
  }

  bool is_valid() const { return id() != DEFAULT_ID(); }

//   operator size_t() const { return _id; }

  std::ostream& insert(std::ostream &os) const
  {
    const Point_25_ &p(*this);
    switch(CGAL::IO::get_mode(os)) {
    case CGAL::IO::ASCII :
      return os << p.x() << ' ' << p.y() << ' ' << p.z();// << ' ' << p.color();
    case CGAL::IO::BINARY :
      CGAL::write(os, p.x());
      CGAL::write(os, p.y());
      CGAL::write(os, p.z());
      return os;
    default:
      return os << "Point_25_(" << p.x() << ", " << p.y() /*<< ", " << p.color()*/ << ')';
    }
  }

  friend std::ostream &
  operator<<(std::ostream &os, const Point_25_ &p)
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
      return os << "Point_25_(" << p.x() << ", " << p.y() << ", " << p.z() << ')';
    }
  }

  friend std::istream &
  operator>>(std::istream &is, Point_25_ &p)
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
      p = Point_25_(x, y, z);
    }
    return is;
  }

};

// template <class ConstructBbox_2>
template <class Kernel>
// class MyConstruct_bbox_2 : public ConstructBbox_2 {
class MyConstruct_bbox_2 : public Kernel::OldK::Construct_bbox_2 {
public:
  CGAL::Bbox_2 operator()(const Point_25_<Kernel>& p) const {
    return CGAL::Bbox_2(CGAL::to_interval(p.x()).first, 
			CGAL::to_interval(p.y()).first, 
			CGAL::to_interval(p.x()).second, 
			CGAL::to_interval(p.y()).second);
  }
//   CGAL::Bbox_2 operator()(const CGAL::Point_2<Kernel>& p) const {
//     return CGAL::Bbox_2(p.x(), p.y(), p.x(), p.y());
//   }
};

template <typename Kernel>
class MyConstruct_coord_iterator {
private:
  typedef typename Kernel::FT Number_type;
public:
  const Number_type* operator()(const Point_25_<Kernel>& p)
  {
    return &p.x();
  }

  const Number_type* operator()(const Point_25_<Kernel>& p, int)
  {
    const Number_type* pyptr = &p.y();
    pyptr++;
    return pyptr;
  }
};

template <typename K, typename OldK>
class MyConstruct_point_2
{
  typedef typename K::RT         RT;
  typedef typename K::Point_2    Point_2;
  typedef typename K::Line_2     Line_2;
  typedef typename Point_2::Rep  Rep;
public:
  typedef Point_2                result_type;

  // Note : the CGAL::Return_base_tag is really internal CGAL stuff.
  // Unfortunately it is needed for optimizing away copy-constructions,
  // due to current lack of delegating constructors in the C++ standard.
  Rep // Point_2
  operator()(CGAL::Return_base_tag, CGAL::Origin o) const
  { return Rep(o); }

  Rep // Point_2
  operator()(CGAL::Return_base_tag, const RT& x, const RT& y) const
  { return Rep(x, y); }

  Rep // Point_2
  operator()(CGAL::Return_base_tag, const RT& x, const RT& y, const RT& w) const
  { return Rep(x, y, w); }

  Point_2
  operator()(CGAL::Origin o) const
  { return Point_25_<K>(0, 0, 0); }

  Point_2
  operator()(const Point_2& p) const
  { return p; }

  Point_2
  operator()(const RT& x, const RT& y) const
  {
    return Point_25_<K>(x, y, 0);
  }

  Point_2
  operator()(const Line_2& l) const
  {
    typename OldK::Construct_point_2 base_operator;
    Point_2 p = base_operator(l);
    return p;
  }

  Point_2
  operator()(const Line_2& l, int i) const
  {
    typename OldK::Construct_point_2 base_operator;
    return base_operator(l, i);
  }

  // We need this one, as such a functor is in the Filtered_kernel
  Point_2
  operator()(const RT& x, const RT& y, const RT& w) const
  {
    if(w != 1){
      return Point_25_<K>(x/w, y/w, 0);
    } else {
      return Point_25_<K>(x,y, 0);
    }
  }
};

CONTOURTILER_END_NAMESPACE

// boost::result_of specialization for C++17 compatibility
// MUST be outside ContourTiler namespace to avoid namespace collision
namespace boost {
  template <typename K, typename OldK>
  struct result_of<const ContourTiler::MyConstruct_point_2<K, OldK>(const typename K::Point_2&)> {
    typedef typename K::Point_2 type;
  };
}

CONTOURTILER_BEGIN_NAMESPACE

template <typename Kernel>
std::size_t hash_value(const Point_25_<Kernel>& point);

template <typename Kernel>
bool xy_equal(const Point_25_<Kernel>& a, const Point_25_<Kernel>& b)
{
  return a.x() == b.x() && a.y() == b.y();
}

template <typename Kernel>
bool xyz_equal(const Point_25_<Kernel>& a, const Point_25_<Kernel>& b)
{
  return a.x() == b.x() && a.y() == b.y() && a.z() == b.z();
}

CONTOURTILER_END_NAMESPACE

#endif // MY_POINTC2_H
