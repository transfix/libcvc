#ifndef __SEGMENT_3_UNDIRECTED__
#define __SEGMENT_3_UNDIRECTED__

#include <ContourTiler/common.h>
#include <ContourTiler/segment_utils.h>
#include <ContourTiler/CGAL_hash.h>

CONTOURTILER_BEGIN_NAMESPACE

/// There is no concept of a source or target in this segment.  There
/// is no direction.
///
/// (0,0,0)-(1,1,1) == (1,1,1)-(0,0,0)
template <typename Segment, typename Point>
class Segment_undirected
{
public:
  Segment_undirected() {}
  Segment_undirected(const Point& a, const Point& b) 
  { *this = Segment(a, b); }
  Segment_undirected(const Segment& s) 
  { *this = s; }
  ~Segment_undirected() {}

  Segment_undirected& operator=(const Segment& s)
  {
    _s = lexicographically_ordered(s);
    _orig = s;
    return *this;
  }

  bool operator==(const Segment_undirected& s) const
  {
    return _s == s._s;
  }

  bool operator<(const Segment_undirected& s) const
  {
    return lesser() < s.lesser();
  }

  // Returns the endpoint that is lexicographically
  // less than the other
  Point lesser() const
  { return _s.source(); }

  // Returns the endpoint that is lexicographically
  // greater than the other
  Point greater() const
  { return _s.target(); }

  Point operator[](size_t index)
  { return _s[index]; }

  /// Returns the underlying segment
  const Segment& segment() const
  { return _s; }

  const Segment& orig_segment() const
  { return _orig; }

private:
  Segment _s;
  Segment _orig;
};

typedef Segment_undirected<Segment_2, Point_2> Segment_2_undirected;
typedef Segment_undirected<Segment_3, Point_3> Segment_3_undirected;

inline std::size_t hash_value(const Segment_2_undirected& s)
{
  return CGAL::hash_value(s.segment());
}

inline std::size_t hash_value(const Segment_3_undirected& s)
{
  return CGAL::hash_value(s.segment());
}

CONTOURTILER_END_NAMESPACE

#endif
