//-----------------------
// Multi-tiler operations
//-----------------------

#include <vector>
#include <list>
#include <map>

#include <CGAL/Direction_2.h>
#include <CGAL/Kernel/global_functions_2.h>

#include <ContourTiler/mtiler_operations.h>
#include <ContourTiler/tiler_operations.h>
#include <ContourTiler/print_utils.h>
#include <ContourTiler/segment_utils.h>
#include <ContourTiler/Intersections.h>
#include <ContourTiler/intersection.h>
#include <ContourTiler/cut.h>
#include <ContourTiler/Z_adjustments.h>

CONTOURTILER_BEGIN_NAMESPACE

using namespace std;

typedef boost::shared_ptr<Triangle> Triangle_handle;

// void mtest_output(vector<Triangle_handle>& new_yellow, vector<Triangle_handle>& new_green,
// 		  list<Triangle_handle>& yellow, list<Triangle_handle>& green)
// {
//   list<Colored_point_3> all;
//   for (vector<Triangle_handle>::iterator it = new_yellow.begin(); it != new_yellow.end(); ++it)
//     for (int i = 0; i < 3; ++i)
//       all.push_back(Colored_point_3(vertex(i, **it), 1, 1, 0));
//   for (vector<Triangle_handle>::iterator it = new_green.begin(); it != new_green.end(); ++it)
//     for (int i = 0; i < 3; ++i)
//       all.push_back(Colored_point_3(vertex(i, **it), 0, 1, 0));

//   ofstream out("output/aa.rawc");
//   raw_print_tiles_impl(out, all.begin(), all.end(), 1, true);
//   out.close();

//   ofstream yr("output/aayellow.rawc");
//   raw_print_tiles(yr, new_yellow.begin(), new_yellow.end(), 1, 1, 0);
//   yr.close();

//   ofstream gr("output/aagreen.rawc");
//   raw_print_tiles(gr, new_green.begin(), new_green.end(), 0, 1, 0);
//   gr.close();

//   //
//   // Output original
//   // 
//   list<Colored_point_3> orig;
//   for (list<Triangle_handle>::iterator it = yellow.begin(); it != yellow.end(); ++it)
//     for (int i = 0; i < 3; ++i)
//       orig.push_back(Colored_point_3(vertex(i, **it), 1, 1, 0));
//   for (list<Triangle_handle>::iterator it = green.begin(); it != green.end(); ++it)
//     for (int i = 0; i < 3; ++i)
//       orig.push_back(Colored_point_3(vertex(i, **it), 0, 1, 0));

//   ofstream orig_out("output/zz.rawc");
//   raw_print_tiles_impl(orig_out, orig.begin(), orig.end(), 1, true);
//   orig_out.close();

//   ofstream orig_yg("output/zzyellow.rawc");
//   raw_print_tiles(orig_yg, yellow.begin(), yellow.end(), 1, 1, 0);
//   orig_yg.close();

//   ofstream orig_gg("output/zzgreen.rawc");
//   raw_print_tiles(orig_gg, green.begin(), green.end(), 0, 1, 0);
//   orig_gg.close();
// }

// template <typename TW_iterator>
// void multi_test(TW_iterator begin, TW_iterator end, Number_type epsilon)
// {
//   typedef Tiler_workspace::Contours::const_iterator C_iter;
//   typedef Correspondences::iterator Corr_iter;
// //   typedef Tile::Skeletons Skeletons;
//   typedef Chord_list::iterator Chords_iter;
// //   typedef Tile::Skeletons::iterator Skel_iter;

//   static log4cplus::Logger logger = log4cplus::Logger::getInstance("multitiler");
//   LOG4CPLUS_INFO(logger, "Multi-tiling");

//   Tiler_workspace::Contours all1;
//   Tiler_workspace::Contours all2;
//   // maps contours to their tiler workspace
//   map<Contour_handle, TW_handle> contour2tw;

//   for (TW_iterator it = begin; it != end; ++it)
//   {
//     // Get all contours
//     all1.insert(all1.end(), (*it)->bottom.begin(), (*it)->bottom.end());
//     all2.insert(all2.end(), (*it)->top.begin(), (*it)->top.end());

//     // Assign the tiler workspaces
//     for (C_iter c = (*it)->bottom.begin(); c != (*it)->bottom.end(); ++c)
//       contour2tw[*c] = *it;
//     for (C_iter c = (*it)->top.begin(); c != (*it)->top.end(); ++c)
//       contour2tw[*c] = *it;
//   }

//   Hierarchy h1(all1.begin(), all1.end(), Hierarchy_policy::FORCE_CCW);
//   Hierarchy h2(all2.begin(), all2.end(), Hierarchy_policy::FORCE_CCW);
  

//   Correspondences correspondences = find_correspondences(all1.begin(), all1.end(), h1,
// 							 all2.begin(), all2.end(), h2);

//   // For every contour c1 in slice 1, find all corresponding contours in slice 2.
//   // For every corresponding contour c2 that doesn't belong to the same
//   // component as c1, compare all of c1's chords to c2's chords
//   // to find intersections.
//   //
//   // c1                     contour in slice 1
//   // c2                     corresponding contour in slice 2
//   // contour2tw             maps contours to their tiler workspaces; contours with the same
//   //                        tiler workspace belong to the same component and need not be
//   //                        checked
//   for (C_iter it = all1.begin(); it != all1.end(); ++it)
//   {
//     Contour_handle c1 = *it;

//     for (Corr_iter cit = correspondences.begin(c1); cit != correspondences.end(c1); ++cit)
//     {
//       Contour_handle c2 = *cit;
//       if (contour2tw[c1] != contour2tw[c2])
//       {

// 	// Get all tiles for each contour, compare and add
// 	TW_handle ytw = contour2tw[c1];
// 	TW_handle gtw = contour2tw[c2];
// 	const Tile_list& yellow = ytw->tiles(c1);
// 	const Tile_list& green = gtw->tiles(c2);
// 	list<Triangle_handle> ytriangles, gtriangles;

// 	for (Tile_list::const_iterator yit = yellow.begin(); yit != yellow.end(); ++yit)
// 	  ytriangles.push_back(Triangle_handle(new Triangle((**yit)[0], (**yit)[1], (**yit)[2])));
// 	for (Tile_list::const_iterator git = green.begin(); git != green.end(); ++git)
// 	  gtriangles.push_back(Triangle_handle(new Triangle((**git)[0], (**git)[1], (**git)[2])));

// 	vector<Triangle_handle> new_yellow, new_green;
// 	remove_intersections(ytriangles.begin(), ytriangles.end(),
// 			     gtriangles.begin(), gtriangles.end(),
// 			     c1->slice(), c2->slice(),
// 			     back_inserter(new_yellow), back_inserter(new_green),
// 			     0.04);
// 	mtest_output(new_yellow, new_green, ytriangles, gtriangles);
//       }
//     }
//   }
// }

// template void multi_test(std::vector<boost::shared_ptr<Tiler_workspace> >::iterator begin, 
// 			 std::vector<boost::shared_ptr<Tiler_workspace> >::iterator end, 
// 			 Number_type epsilon);
// template void multi_test(std::list<boost::shared_ptr<Tiler_workspace> >::iterator begin, 
// 			 std::list<boost::shared_ptr<Tiler_workspace> >::iterator end, 
// 			 Number_type epsilon);


CONTOURTILER_END_NAMESPACE
