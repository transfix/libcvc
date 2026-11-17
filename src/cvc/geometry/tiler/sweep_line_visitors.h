#ifndef __SWEEP_LINE_VISITORS_H__
#define __SWEEP_LINE_VISITORS_H__

#include <ContourTiler/common.h>
#include <ContourTiler/sweep_line.h>

CONTOURTILER_BEGIN_NAMESPACE

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
class Debug_visitor
{
public:
  Debug_visitor() {}
  ~Debug_visitor() {}

  bool end_end(const Point_2& p, const Segment_2& a, size_t a_id, const Segment_2& b, size_t b_id)
  {
    log4cplus::Logger logger = log4cplus::Logger::getInstance("sweep_line");
    LOG4CPLUS_INFO(logger, "end-end intersection: " << pp(p) << " " << pp(a) << " " << pp(b));
    return true;
  }
  bool end_int(const Point_2& p, const Segment_2& a, size_t a_id, const Segment_2& b, size_t b_id)
  {
    log4cplus::Logger logger = log4cplus::Logger::getInstance("sweep_line");
    LOG4CPLUS_INFO(logger, "end-int intersection: " << pp(p) << " " << pp(a) << " " << pp(b));
    return true;
  }
  bool int_int(const Point_2& p, const Segment_2& a, size_t a_id, const Segment_2& b, size_t b_id)
  {
    log4cplus::Logger logger = log4cplus::Logger::getInstance("sweep_line");
    LOG4CPLUS_INFO(logger, "int-int intersection: " << pp(p) << " " << pp(a) << " " << pp(b));
    return true;
  }
};

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
template <typename Visitor>
class Filter_visitor
{
public:
  Filter_visitor(Visitor v, bool end_int, bool end_end) 
    : _v(v), _ei(end_int), _ee(end_end) {}
  ~Filter_visitor() {}

  bool end_end(const Point_2& p, const Segment_2& a, size_t a_id, const Segment_2& b, size_t b_id)
  {
    if (_ee)
      return _v.end_end(p, a, a_id, b, b_id);
    return true;
  }
  bool end_int(const Point_2& p, const Segment_2& a, size_t a_id, const Segment_2& b, size_t b_id)
  {
    if (_ei)
      return _v.end_int(p, a, a_id, b, b_id);
    return true;
  }
  bool int_int(const Point_2& p, const Segment_2& a, size_t a_id, const Segment_2& b, size_t b_id)
  {
    return _v.int_int(p, a, a_id, b, b_id);
  }
  
  Visitor& visitor()
  { return _v; }

  const Visitor& visitor() const
  { return _v; }

private:
  Visitor _v;
  bool _ei, _ee;
};

template <typename Visitor>
Filter_visitor<Visitor> filter_visitor(Visitor v, bool end_int, bool end_end)
{ return Filter_visitor<Visitor>(v, end_int, end_end); }

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
class SL_intersection
{
public:
  SL_intersection(const Point_2& p)//, const Segment_2& a, const Segment_2& b)
    : _p(p) {}//, _a(a), _b(b) {}
  ~SL_intersection() {}

  const Point_2& point() const
  { return _p; }

//   const Segment_2& a() const
//   { return _a; }

//   const Segment_2& b() const
//   { return _b; }

  Point_2& point()
  { return _p; }

  void set(const Segment_2& a, size_t a_id, const Segment_2& b, size_t b_id)
  {
    _id2seg[a_id] = a;
    _id2seg[b_id] = b;
  }

  void push_back_end(const Segment_2& s)
  { _e.push_back(s); }

  void push_back_int(const Segment_2& s)
  { _i.push_back(s); }

  const list<Segment_2>& ends() const
  { return _e; }

  const list<Segment_2>& interiors() const
  { return _i; }

  const Segment_2& segment(size_t component_id) const
  { return _id2seg.find(component_id)->second; }

  std::pair<size_t, size_t> components() const
  {
    boost::unordered_map<size_t, Segment_2>::const_iterator it = _id2seg.begin();
    size_t a = it->first;
    size_t b = a;
    if (++it != _id2seg.end())
      b = it->first;
    return std::make_pair(a, b);
  }

private:
  Point_2 _p;
  list<Segment_2> _e, _i;
  boost::unordered_map<size_t, Segment_2> _id2seg;
};

template <typename Intersection_iter>
void get_interior_only(Intersection_iter begin, Intersection_iter end, 
		       boost::unordered_map<Segment_2, list<Point_2> >& map)
{
  static log4cplus::Logger logger = log4cplus::Logger::getInstance("sweep_line");
  typedef boost::unordered_map<Segment_2, list<Point_2> > Map;

  for (Intersection_iter it = begin; it != end; ++it)
  {
    const SL_intersection& i = *it;
    const list<Segment_2>& interior = i.interiors();
    const list<Segment_2>& ends = i.ends();
    typedef list<Segment_2>::const_iterator l_iter;
//     LOG4CPLUS_TRACE(logger, pp(i.point()) << " Interiors: " << interior.size() << " Ends: " << ends.size());
//     for (l_iter sit = interior.begin(); sit != interior.end(); ++sit)
//       LOG4CPLUS_TRACE(logger, "  " << pp(*sit));
//     for (l_iter sit = ends.begin(); sit != ends.end(); ++sit)
//       LOG4CPLUS_TRACE(logger, "  " << pp(*sit));
    for (l_iter ii = interior.begin(); ii != interior.end(); ++ii)
      map[*ii].push_back(i.point());
  }

  for (Map::iterator it = map.begin(); it != map.end(); ++it)
  {
    const Segment_2& segment = it->first;
    list<Point_2>& points = it->second;
    points.sort(dist_functor(segment.source()));
    points.unique();
  }

}

//------------------------------------------------------------------------------
// Intersection_visitor
//------------------------------------------------------------------------------
template <typename Output_iter>
class Intersection_visitor
{
public:
  Intersection_visitor(Output_iter iter) : _iter(iter) {}
  ~Intersection_visitor() {}

  bool end_end(const Point_2& p, const Segment_2& a, size_t a_id, const Segment_2& b, size_t b_id)
  {
    SL_intersection i(p);
    i.set(a, a_id, b, b_id);
    i.push_back_end(a);
    i.push_back_end(b);
    *_iter++ = i;//, a, b);
    return true;
  }
  bool end_int(const Point_2& p, const Segment_2& a, size_t a_id, const Segment_2& b, size_t b_id)
  {
    SL_intersection i(p);
    i.set(a, a_id, b, b_id);
    i.push_back_end(a);
    i.push_back_int(b);
    *_iter++ = i;//, a, b);
//     *_iter++ = SL_intersection(p, a, b);
    return true;
  }
  bool int_int(const Point_2& p, const Segment_2& a, size_t a_id, const Segment_2& b, size_t b_id)
  {
    SL_intersection i(p);
    i.set(a, a_id, b, b_id);
    i.push_back_int(a);
    i.push_back_int(b);
    *_iter++ = i;
//     *_iter++ = SL_intersection(p, a, b);
    return true;
  }

private:
  Output_iter _iter;
};

template <typename Out_iter>
Intersection_visitor<Out_iter> intersection_visitor(Out_iter out)
{ return Intersection_visitor<Out_iter>(out); }

//------------------------------------------------------------------------------
// Has_intersection_visitor
//------------------------------------------------------------------------------
class Has_intersection_visitor
{
public:
  Has_intersection_visitor() : _b(false) {}
  ~Has_intersection_visitor() {}

  bool end_end(const Point_2& p, const Segment_2& a, size_t a_id, const Segment_2& b, size_t b_id)
  {
    _b = true;
    return false;
  }
  bool end_int(const Point_2& p, const Segment_2& a, size_t a_id, const Segment_2& b, size_t b_id)
  {
    _b = true;
    return false;
  }
  bool int_int(const Point_2& p, const Segment_2& a, size_t a_id, const Segment_2& b, size_t b_id)
  {
    _b = true;
    return false;
  }
  
  bool has_intersection() const { return _b; }

private:
  bool _b;
};


//------------------------------------------------------------------------------
// Functions
//------------------------------------------------------------------------------

template <typename Segment_iter, typename Output_iter>
void get_intersections(Segment_iter segments_begin, Segment_iter segments_end, Output_iter intersections, 
		       bool end_internal, bool end_end)
{
  typedef Intersection_visitor<Output_iter> Visitor;
  sweep_line(segments_begin, segments_end, filter_visitor(Visitor(intersections), end_internal, end_end));
}

template <typename Segment_iter, typename Output_iter>
void get_intersections(Segment_iter segments_begin0, Segment_iter segments_end0, 
		       Segment_iter segments_begin1, Segment_iter segments_end1, 
		       Output_iter intersections, 
		       bool end_internal, bool end_end)
{
  typedef Intersection_visitor<Output_iter> Visitor;
  sweep_line(segments_begin0, segments_end0, segments_begin1, segments_end1, 
	     filter_visitor(Visitor(intersections), end_internal, end_end));
}

template <typename Segment_iter, typename Output_iter>
void get_intersection_points(Segment_iter segments_begin, Segment_iter segments_end, Output_iter points, 
		       bool end_internal, bool end_end)
{
  typedef list<SL_intersection> List;
  typedef Intersection_visitor<back_insert_iterator<List> > Visitor;
  List intersections;
  sweep_line(segments_begin, segments_end, filter_visitor(Visitor(back_inserter(intersections)), end_internal, end_end));
  for (list<SL_intersection>::iterator it = intersections.begin(); it != intersections.end(); ++it)
    *points++ = it->point();
}

template <typename Segment_iter>
bool has_intersection(Segment_iter segments_begin, Segment_iter segments_end,
		      bool end_internal, bool end_end)
{
  typedef Has_intersection_visitor Visitor;
  typedef Filter_visitor<Visitor> FV;
  FV fv(Visitor(), end_internal, end_end);
  fv = sweep_line(segments_begin, segments_end, fv);
  return fv.visitor().has_intersection();
}

template <typename Segment_iter>
bool has_intersection(Segment_iter segments_begin0, Segment_iter segments_end0, 
		       Segment_iter segments_begin1, Segment_iter segments_end1,
		      bool end_internal, bool end_end)
{
  typedef Has_intersection_visitor Visitor;
  typedef Filter_visitor<Visitor> FV;
  FV fv(Visitor(), end_internal, end_end);
  fv = sweep_line(segments_begin0, segments_end0, segments_begin1, segments_end1, fv);
  return fv.visitor().has_intersection();
}

CONTOURTILER_END_NAMESPACE

#endif
