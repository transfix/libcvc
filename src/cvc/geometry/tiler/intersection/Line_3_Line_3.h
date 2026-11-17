#ifndef CEP_INTERSECTION_LINE_3_LINE_3_H    // -*- C++ -*-
#define CEP_INTERSECTION_LINE_3_LINE_3_H

#include <CGAL/Object.h>
#include <CGAL/Point_3.h>


#include <CGAL/Object.h>
#include <CGAL/Point_3.h>
#include <CGAL/Vector_3.h>
#include <CGAL/predicates_on_points_3.h>


namespace CEP {  
namespace intersection {

    namespace Line_3_Line_3 {

        template <class R>
        CGAL::Object
        coplanar_nonparallel_intersection( const CGAL::Point_3<R>& p,
                                           const CGAL::Point_3<R>& q,
                                           const CGAL::Point_3<R>& r,
                                           const CGAL::Point_3<R>& s )
        {
            CGAL_exactness_precondition( CGAL::coplanar(p,q,r,s) );

            typedef typename R::FT FT;

            CGAL::Vector_3<R> d1 = q - p;
            CGAL::Vector_3<R> d2 = s - r;
            CGAL::Vector_3<R> b = r - p;

            // Can we assume compilers are smart enough to pull
            // out the common subexpressions here ... ?
            FT num = (d2*d2) * (d1*b) - (d1*d2) * (d2*b);
            FT den = (d1*d1) * (d2*d2) - (d1*d2) * (d1*d2);

            FT zero(0);
            CGAL_exactness_assertion( den != zero );

            CGAL::Point_3<R> i_point = p + d1 * num / den;
            return CGAL::make_object(i_point);
        }


        template <class R>
        CGAL::Object
        coplanar_intersection( const CGAL::Point_3<R>& p,
                               const CGAL::Point_3<R>& q,
                               const CGAL::Point_3<R>& r,
                               const CGAL::Point_3<R>& s )
        {
            CGAL_exactness_precondition( CGAL::coplanar(p,q,r,s) );
    
            CGAL::Vector_3<R> d1 = q-p;
            CGAL::Vector_3<R> d2 = s-r;

            if ( d1.direction() ==  d2.direction() ||
                 d1.direction() == -d2.direction() )
                return CGAL::Object();

            return coplanar_nonparallel_intersection(p,q,r,s);
        }
        
        
    }
}
}
// namespace CEP {  
// namespace intersection {

//     using CGAL::Point_3;

//     namespace Line_3_Line_3 {
        
//         template <class R>
//         CGAL::Object
//         coplanar_intersection( const Point_3<R>& p,
//                                const Point_3<R>& q,
//                                const Point_3<R>& r,
//                                const Point_3<R>& s );


//         template <class R>
//         CGAL::Object
//         coplanar_nonparallel_intersection( const Point_3<R>& p,
//                                            const Point_3<R>& q,
//                                            const Point_3<R>& r,
//                                            const Point_3<R>& s );
        
//     }
// }
// }


// #ifdef CGAL_CFG_NO_AUTOMATIC_TEMPLATE_INCLUSION
// #  include <CEP/intersection/Line_3_Line_3.C>
// #endif

#endif
