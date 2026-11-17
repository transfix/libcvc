#ifndef CEP_INTERSECTION_SEGMENT_3_POINT_3_H
#define CEP_INTERSECTION_SEGMENT_3_POINT_3_H

#include <CGAL/Object.h>
#include <CGAL/Segment_3.h>
#include <CGAL/Point_3.h>
#include <CGAL/predicates_on_points_3.h>


namespace CEP {
  namespace intersection {


    //     using CGAL::Segment_3;
    //     using CGAL::Point_3;



    template <class R>
    inline
    bool
    do_intersect( const CGAL::Segment_3<R>& s, 
		  const CGAL::Point_3<R>& p )
    {
      return s.has_on(p);
    }



    template <class R>
    inline
    CGAL::Object
    intersection( const CGAL::Segment_3<R>& s, 
		  const CGAL::Point_3<R>& p )
    {
      if ( CEP::intersection::do_intersect(s,p) )
        return CGAL::make_object(p);
      return CGAL::Object();
    }


    template <class R>
    inline 
    bool
    collinear( const CGAL::Segment_3<R>& s, 
	       const CGAL::Point_3<R>& p )
    {
      //     return CGAL::collinear( s.source(), s.target(), p );
      typedef typename R::Collinear_3 fo;
      return fo()( s.source(), s.target(), p );
    }

  }
}

#endif
