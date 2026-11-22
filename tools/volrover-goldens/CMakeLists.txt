# The legacy VolumeRover libraries, built from a transfix/volrover checkout at the pinned sha
# (volrover.sha), to make the ports' T1 goldens (README.md). Never part of libcvc's build: make_goldens.py
# configures this directory on its own. GPL code (the legacy tree's own CGAL, for the libraries
# that need it) may be built here and only here; nothing of it is linked into libcvc.
cmake_minimum_required(VERSION 3.16)
project(volrover_goldens CXX)

set(VOLROVER_DIR "" CACHE PATH "A transfix/volrover checkout at the sha in volrover.sha")
if(NOT EXISTS "${VOLROVER_DIR}/src/Segmentation/GenSeg/segment.cpp" AND
   NOT EXISTS "${VOLROVER_DIR}/src/Segmentation/SegMed/GVF.cpp" AND
   NOT EXISTS "${VOLROVER_DIR}/src/Skeletonization/Skeletonization.cpp" AND
   NOT EXISTS "${VOLROVER_DIR}/src/SecondaryStructures/skel.cpp" AND
   NOT EXISTS "${VOLROVER_DIR}/src/ContourTiler/tiler.cpp" AND
   NOT EXISTS "${VOLROVER_DIR}/src/PocketTunnel/pocket_tunnel.cpp")
  message(FATAL_ERROR "VOLROVER_DIR (${VOLROVER_DIR}) is not a volrover checkout with a library "
                      "this directory builds")
endif()

# Legacy code as it is: built with its warnings off, and never with -Werror.
function(volrover_legacy target)
  # The shim comes first, so <VolMagick/...> and <XmlRPC/...> resolve to it (shim/*/*.h say why).
  target_include_directories(${target} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/shim ${VOLROVER_DIR}/inc)
  if(WIN32)
    target_compile_definitions(${target} PRIVATE __WINDOWS__ _CRT_SECURE_NO_WARNINGS)
  endif()
  if(MSVC)
    target_compile_options(${target} PRIVATE /w)
  else()
    target_compile_options(${target} PRIVATE -w)
  endif()
endfunction()

# GenSeg (src/Segmentation/GenSeg): the fast marching and its rawiv reader and writer, as they are.
# general_segmentation.cpp is its XML-RPC entry, which genseg_driver.cpp stands in for.
if(EXISTS "${VOLROVER_DIR}/src/Segmentation/GenSeg/segment.cpp")
  add_executable(genseg_legacy genseg_driver.cpp
    ${VOLROVER_DIR}/src/Segmentation/GenSeg/segment.cpp
    ${VOLROVER_DIR}/src/Segmentation/GenSeg/inout.cpp)
  volrover_legacy(genseg_legacy)
endif()

# SegMed (src/Segmentation/SegMed): GVF.cpp (the gradient vector diffusion and its critical points)
# and inout.cpp (the rawiv reader), from copies in the build tree with two patches, each of which
# must apply:
#   - GVF_Compute's `return;` right after InitGVF (GVF.cpp:97) goes: it skipped the diffusion, the
#     critical points and their clustering, the code the port (src/cvc/volume/segmentation/
#     critical_points.cpp) and the goldens are about;
#   - read_data opens the rawiv file "rb", not "r" (the same on POSIX; text mode on Windows mangles
#     the floats).
# segmed_driver.cpp stands in for medical_segmentation.cpp, the XML-RPC entry, and includes the
# patched GVF.cpp to read the diffused field from its globals. segment.cpp (a march from 13 seeds
# hard-coded for one heart dataset) and diffuse.cpp (uncalled) are left out.
if(EXISTS "${VOLROVER_DIR}/src/Segmentation/SegMed/GVF.cpp")
  set(_segmed_out "${CMAKE_CURRENT_BINARY_DIR}/segmed_src")
  file(MAKE_DIRECTORY "${_segmed_out}")
  set(_gvf_in "${VOLROVER_DIR}/src/Segmentation/SegMed/GVF.cpp")
  set(_inout_in "${VOLROVER_DIR}/src/Segmentation/SegMed/inout.cpp")
  set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${_gvf_in}" "${_inout_in}")
  file(READ "${_gvf_in}" _text)
  string(FIND "${_text}" "InitGVF(edge_mag);" _first)
  string(FIND "${_text}" "InitGVF(edge_mag);" _last REVERSE)
  string(REGEX REPLACE "InitGVF\\(edge_mag\\);[ \t\r\n]*return;" "InitGVF(edge_mag);" _patched
                       "${_text}")
  if(_first EQUAL -1 OR NOT _first EQUAL _last OR _patched STREQUAL _text)
    message(FATAL_ERROR "volrover-goldens: the patch for SegMed's GVF.cpp does not apply")
  endif()
  file(WRITE "${_segmed_out}/GVF.cpp" "${_patched}")
  file(READ "${_inout_in}" _text)
  string(FIND "${_text}" "fopen(input_name, \"r\")" _at)
  if(_at EQUAL -1)
    message(FATAL_ERROR "volrover-goldens: the patch for SegMed's inout.cpp does not apply")
  endif()
  string(REPLACE "fopen(input_name, \"r\")" "fopen(input_name, \"rb\")" _text "${_text}")
  file(WRITE "${_segmed_out}/inout.cpp" "${_text}")
  add_executable(segmed_legacy segmed_driver.cpp "${_segmed_out}/inout.cpp")
  volrover_legacy(segmed_legacy)
  target_include_directories(segmed_legacy PRIVATE "${_segmed_out}")
endif()

# Skeletonization (src/Skeletonization), when the checkout has it and CGAL is found (CGAL, GMP,
# MPFR and Boost, e.g. from cvcpkg on CMAKE_PREFIX_PATH). The sources are compiled from patched
# copies in the build tree; each patch must apply, or configuring fails:
#   - five CGAL_assertion()s with side effects become plain calls: has_vertex() in medax.cpp and
#     tcocone.cpp and find_acceptor() in u1.cpp and u2.cpp fill the indices the code then uses,
#     and a release build compiles the assertions out, leaving those indices -1 (or
#     uninitialised), which reads and writes out of bounds. The port (src/cvc/geometry/skeleton)
#     makes the same calls;
#   - is_cospherical_pair() returns false when a cell is infinite instead of testing the
#     infinite vertex's default-constructed (uninitialised) point, as the port does;
#   - the helix strips build_simple_skel() writes get alpha 0.75 instead of 1, so the driver can
#     tell them from the polyline segments, which have the same colours.
# PolyTess (not called, and not for commercial use) is left out: shim/Skeletonization/PolyTess.h.
if(EXISTS "${VOLROVER_DIR}/src/Skeletonization/Skeletonization.cpp")
  find_package(CGAL CONFIG QUIET)
  find_package(Boost QUIET)
  if(CGAL_FOUND AND Boost_FOUND)
    set(_skel_src_dir "${VOLROVER_DIR}/src/Skeletonization")
    set(_skel_out "${CMAKE_CURRENT_BINARY_DIR}/skeleton_src")
    file(MAKE_DIRECTORY "${_skel_out}")
    set(_skel_files Skeletonization degen graph hfn_util init intersect medax op rcocone robust_cc
      skel tcocone u1 u2 util)
    # file|old text|new text, with %SEMI% for ';' (a CMake list cannot hold one)
    set(_skel_patches
      "medax|CGAL_assertion(triang.has_vertex(ch, fid, u, u_id))%SEMI%|triang.has_vertex(ch, fid, u, u_id)%SEMI%"
      "medax|CGAL_assertion(triang.has_vertex(ch, fid, v, v_id))%SEMI%|triang.has_vertex(ch, fid, v, v_id)%SEMI%"
      "tcocone|CGAL_assertion(ch->has_vertex(v, iv) && (iv != -1))%SEMI%|ch->has_vertex(v, iv)%SEMI%"
      "tcocone|CGAL_assertion(ch->has_vertex(w, iw) && (iw != -1))%SEMI%|ch->has_vertex(w, iw)%SEMI%"
      "u1|CGAL_assertion(find_acceptor(c, fid, uid, vid, wid))%SEMI%|find_acceptor(c, fid, uid, vid, wid)%SEMI%"
      "u2|CGAL_assertion(find_acceptor(c0, id, uid, vid, wid))%SEMI%|find_acceptor(c0, id, uid, vid, wid)%SEMI%"
      "u2|CGAL_assertion(find_acceptor(c, ofid[0], uid, vid, wid))%SEMI%|find_acceptor(c, ofid[0], uid, vid, wid)%SEMI%"
      "util|    if(triang.side_of_sphere(c[0], c[1]->vertex(id[1])->point())|    if(!triang.is_infinite(c[0]) && !triang.is_infinite(c[1]) && triang.side_of_sphere(c[0], c[1]->vertex(id[1])->point())"
      "Skeletonization|Simple_color(1.0,0.0,0.0,1.0)|Simple_color(1.0,0.0,0.0,0.75)"
      "Skeletonization|Simple_color(0.0,1.0,0.0,1.0)|Simple_color(0.0,1.0,0.0,0.75)")
    set(_skel_sources "")
    foreach(_f ${_skel_files})
      set(_in "${_skel_src_dir}/${_f}.cpp")
      set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${_in}")
      file(READ "${_in}" _text)
      foreach(_p ${_skel_patches})
        string(REPLACE "|" ";" _parts "${_p}")
        list(GET _parts 0 _pf)
        list(GET _parts 1 _old)
        list(GET _parts 2 _new)
        if(_pf STREQUAL _f)
          string(REPLACE "%SEMI%" ";" _old "${_old}")
          string(REPLACE "%SEMI%" ";" _new "${_new}")
          string(FIND "${_text}" "${_old}" _at)
          if(_at EQUAL -1)
            message(FATAL_ERROR "volrover-goldens: the patch for ${_f}.cpp does not apply: ${_old}")
          endif()
          string(REPLACE "${_old}" "${_new}" _text "${_text}")
        endif()
      endforeach()
      file(WRITE "${_skel_out}/${_f}.cpp" "${_text}")
      list(APPEND _skel_sources "${_skel_out}/${_f}.cpp")
    endforeach()
    add_executable(skeleton_legacy skeleton_driver.cpp ${_skel_sources})
    volrover_legacy(skeleton_legacy)
    target_link_libraries(skeleton_legacy PRIVATE CGAL::CGAL)
    target_include_directories(skeleton_legacy PRIVATE ${Boost_INCLUDE_DIRS})
    set_target_properties(skeleton_legacy PROPERTIES CXX_STANDARD 17)
    target_compile_definitions(skeleton_legacy PRIVATE _USE_MATH_DEFINES) # M_PI on MSVC
    if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
      target_compile_options(skeleton_legacy PRIVATE -frounding-math)
    endif()
    message(STATUS "volrover-goldens: Skeletonization with CGAL ${CGAL_VERSION}")
  else()
    message(STATUS "volrover-goldens: no CGAL or Boost, so no Skeletonization")
  endif()
endif()

# SecondaryStructures (src/SecondaryStructures), when the checkout has it and CGAL is found, as for
# Skeletonization. Every legacy source is compiled (the .back and .junk files are not sources), from
# copies in the build tree with these patches, each of which must apply, or configuring fails:
#   - eight CGAL_assertion()s with side effects become plain calls: has_vertex() in tcocone.cpp,
#     find_acceptor() in u1.cpp and u2.cpp (as for Skeletonization), and get_eid() in skel.cpp's
#     add_u1_to_skel and add_u2_to_skel, which finds the edge whose polygons and flags the code then
#     updates: compiled out, it left the index -1, writing before the edge array;
#   - is_cospherical_pair() returns false when a cell is infinite, as for Skeletonization;
#   - a chain step between cells with no common facet gets the smaller cell's circumradius as its
#     width, not its squared circumradius (every other width is a length), as the port does;
#   - build_cylindrical_helices skips an empty run (a candidate point with no other within the join
#     distance), which it divided by zero for and read past an empty array, as the port does;
#   - its variable-length array (a GCC extension MSVC lacks) becomes a std::vector;
#   - init.cpp's triang.dual(cit) becomes CGAL::circumcenter of the cell's points, which is what
#     dual() computed in the CGAL it was written for (CGAL 6's asks the cell base for it).
# helix.cpp calls LAPACK's dsyev_; secondary_dsyev.cpp stands in for it. shim/cvcraw_geometry,
# shim/Histogram and shim/Contour stand in for the volrover libraries its headers include. The
# legacy CMakeLists defined HUGE as 1000000, which math.h redefines or rejects; shim/secondary_huge.h,
# force-included, makes it 1000000 everywhere. The legacy code writes debug files to the current
# directory, so make_goldens.py runs it in a scratch one.
if(EXISTS "${VOLROVER_DIR}/src/SecondaryStructures/skel.cpp")
  find_package(CGAL CONFIG QUIET)
  find_package(Boost QUIET)
  if(CGAL_FOUND AND Boost_FOUND)
    set(_ss_src_dir "${VOLROVER_DIR}/src/SecondaryStructures")
    set(_ss_out "${CMAKE_CURRENT_BINARY_DIR}/secondary_src")
    file(MAKE_DIRECTORY "${_ss_out}")
    set(_ss_files degen helix hfn_util init intersect medax op rcocone robust_cc skel tcocone u1 u2
      util)
    # file|old text|new text, with %SEMI% for ';' (a CMake list cannot hold one)
    set(_ss_patches
      "tcocone|CGAL_assertion(ch->has_vertex(v, iv) && (iv != -1))%SEMI%|ch->has_vertex(v, iv)%SEMI%"
      "tcocone|CGAL_assertion(ch->has_vertex(w, iw) && (iw != -1))%SEMI%|ch->has_vertex(w, iw)%SEMI%"
      "u1|CGAL_assertion(find_acceptor(c, fid, uid, vid, wid))%SEMI%|find_acceptor(c, fid, uid, vid, wid)%SEMI%"
      "u2|CGAL_assertion(find_acceptor(c0, id, uid, vid, wid))%SEMI%|find_acceptor(c0, id, uid, vid, wid)%SEMI%"
      "u2|CGAL_assertion(find_acceptor(c, ofid[0], uid, vid, wid))%SEMI%|find_acceptor(c, ofid[0], uid, vid, wid)%SEMI%"
      "skel|CGAL_assertion(sv.get_eid(uid, eid))%SEMI%|sv.get_eid(uid, eid)%SEMI%"
      "util|if (triang.side_of_sphere(c[0], c[1]->vertex(id[1])->point())|if (!triang.is_infinite(c[0]) && !triang.is_infinite(c[1]) && triang.side_of_sphere(c[0], c[1]->vertex(id[1])->point())"
      "skel|chain[j]->cell_radius() : chain[j+1]->cell_radius()%SEMI%|std::sqrt(chain[j]->cell_radius()) : std::sqrt(chain[j+1]->cell_radius())%SEMI%"
      "init|cit->set_voronoi(triang.dual(cit))%SEMI%|cit->set_voronoi(CGAL::circumcenter(cit->vertex(0)->point(), cit->vertex(1)->point(), cit->vertex(2)->point(), cit->vertex(3)->point()))%SEMI%"
      "helix|Vector centold(0.0, 0.0, 0.0)%SEMI%|if (curve_comps[i].empty()) continue%SEMI% Vector centold(0.0, 0.0, 0.0)%SEMI%"
      "helix|double lambda[curve_comps[i].size()]%SEMI%|std::vector<double> lambda(curve_comps[i].size())%SEMI%")
    set(_ss_sources "")
    foreach(_f ${_ss_files})
      set(_in "${_ss_src_dir}/${_f}.cpp")
      set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${_in}")
      file(READ "${_in}" _text)
      foreach(_p ${_ss_patches})
        string(REPLACE "|" ";" _parts "${_p}")
        list(GET _parts 0 _pf)
        list(GET _parts 1 _old)
        list(GET _parts 2 _new)
        if(_pf STREQUAL _f)
          string(REPLACE "%SEMI%" ";" _old "${_old}")
          string(REPLACE "%SEMI%" ";" _new "${_new}")
          string(FIND "${_text}" "${_old}" _at)
          if(_at EQUAL -1)
            message(FATAL_ERROR "volrover-goldens: the patch for SecondaryStructures' ${_f}.cpp "
                                "does not apply: ${_old}")
          endif()
          string(REPLACE "${_old}" "${_new}" _text "${_text}")
        endif()
      endforeach()
      file(WRITE "${_ss_out}/${_f}.cpp" "${_text}")
      list(APPEND _ss_sources "${_ss_out}/${_f}.cpp")
    endforeach()
    add_executable(secondary_legacy secondary_driver.cpp secondary_dsyev.cpp ${_ss_sources})
    volrover_legacy(secondary_legacy)
    target_link_libraries(secondary_legacy PRIVATE CGAL::CGAL)
    target_include_directories(secondary_legacy PRIVATE ${Boost_INCLUDE_DIRS})
    set_target_properties(secondary_legacy PROPERTIES CXX_STANDARD 17)
    target_compile_definitions(secondary_legacy PRIVATE _USE_MATH_DEFINES)
    if(MSVC)
      target_compile_options(secondary_legacy PRIVATE "/FI${CMAKE_CURRENT_SOURCE_DIR}/shim/secondary_huge.h")
    else()
      target_compile_options(secondary_legacy PRIVATE -include "${CMAKE_CURRENT_SOURCE_DIR}/shim/secondary_huge.h")
    endif()
    if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
      target_compile_options(secondary_legacy PRIVATE -frounding-math)
    endif()
    message(STATUS "volrover-goldens: SecondaryStructures with CGAL ${CGAL_VERSION}")
  else()
    message(STATUS "volrover-goldens: no CGAL or Boost, so no SecondaryStructures")
  endif()
endif()

# ContourTiler (src/ContourTiler), when the checkout has it and CGAL is found, as for
# Skeletonization. Its library sources are compiled as they are, but for its tools (ecs, smfconvert,
# its command line), contour_graph.cpp (unused; its reader needs ContourFilter) and minkowski.cpp
# (unused), with tiler_driver.cpp in place of main.cpp. shim/log4cplus stands in for log4cplus and
# logs nothing. Four files are compiled from patched copies in the build tree, and each patch must
# apply, or configuring fails:
#   - print_utils.cpp: raw_print_tiles_impl writes coordinates with 17 digits (its commented-out
#     setprecision, back in), instead of the stream's default 6, and a vertex's z as the tiler's own
#     (a contour vertex's is its section index), instead of (z + 0.5) * z_scale;
#   - offset_polygon.cpp and skeleton.cpp: CGAL 6's straight-skeleton functions return
#     std::shared_ptr where CGAL 5 returned boost::shared_ptr;
#   - tiler_output.cpp: Callback::untiled_region (the debug output hook, called for every untiled
#     region) falls off its end without returning its bool, undefined behaviour that clang's code
#     aborts on; it returns true, as Callback::tile_added does.
# MSVC's standard library leaves out std::ptr_fun and std::unary_function under C++17, which
# xml/Ser_reader.cpp and interp.cpp use; _HAS_AUTO_PTR_ETC brings them back (libstdc++ keeps them). MSVC itself cannot compile the legacy kernel (Cartesian_25's conversions); clang-cl can.
if(EXISTS "${VOLROVER_DIR}/src/ContourTiler/tiler.cpp")
  find_package(CGAL CONFIG QUIET)
  find_package(Boost QUIET)
  if(CGAL_FOUND AND Boost_FOUND)
    set(_ct_dir "${VOLROVER_DIR}/src/ContourTiler")
    set(_ct_out "${CMAKE_CURRENT_BINARY_DIR}/tiler_src")
    file(MAKE_DIRECTORY "${_ct_out}")
    set(_ct_files CGAL_hash Contour Contour2 Hierarchy tiler_operations contour_utils polygon_utils
      Correspondences Tiling_region Composite_tiling_region All_tiling_region Point_tiling_region
      Wedge skeleton Point_25 Boundary_slice_chords theorems projection Tiler_workspace tiler
      tiler_output Tiles mtiler_operations segment_utils intersection cut augment Polyline
      triangle_utils remove_contour_intersections offset_polygon polygon_difference
      polygon_intersection Slice Slice2 Z_adjustments interp arrange_polygons perturb print_utils
      intersection/Segment_3_Segment_3 intersection/Line_3_Line_3 xml/Ser_reader xml/tinyxml
      xml/tinystr xml/tinyxmlerror xml/tinyxmlparser)
    # file|old text|new text, with %SEMI% for ';' and %Q% for '"'; every occurrence is replaced
    set(_ct_patches
      "print_utils|  // ss << std::setprecision(24)%SEMI%|  ss << std::setprecision(17)%SEMI%"
      "print_utils|ss << t.x() << %Q% %Q% << t.y() << %Q% %Q% << (t.z()+0.5) * z_scale%SEMI%|ss << t.x() << %Q% %Q% << t.y() << %Q% %Q% << t.z()%SEMI%"
      "offset_polygon|boost::shared_ptr|std::shared_ptr"
      "skeleton|boost::shared_ptr|std::shared_ptr"
      "tiler_output|    out.close()%SEMI%\n  }\n}|    out.close()%SEMI%\n  }\n  return true%SEMI%\n}")
    set(_ct_sources "")
    foreach(_f ${_ct_files})
      set(_in "${_ct_dir}/${_f}.cpp")
      set(_patched OFF)
      foreach(_p ${_ct_patches})
        string(REPLACE "|" ";" _parts "${_p}")
        list(GET _parts 0 _pf)
        if(_pf STREQUAL _f)
          if(NOT _patched)
            set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${_in}")
            file(READ "${_in}" _text)
            set(_patched ON)
          endif()
          list(GET _parts 1 _old)
          list(GET _parts 2 _new)
          foreach(_v _old _new)
            string(REPLACE "%SEMI%" ";" ${_v} "${${_v}}")
            string(REPLACE "%Q%" "\"" ${_v} "${${_v}}")
          endforeach()
          string(FIND "${_text}" "${_old}" _at)
          if(_at EQUAL -1)
            message(FATAL_ERROR "volrover-goldens: the patch for ${_f}.cpp does not apply: ${_old}")
          endif()
          string(REPLACE "${_old}" "${_new}" _text "${_text}")
        endif()
      endforeach()
      if(_patched)
        file(WRITE "${_ct_out}/${_f}.cpp" "${_text}")
        list(APPEND _ct_sources "${_ct_out}/${_f}.cpp")
      else()
        list(APPEND _ct_sources "${_in}")
      endif()
    endforeach()
    add_executable(tiler_legacy tiler_driver.cpp ${_ct_sources})
    volrover_legacy(tiler_legacy)
    target_link_libraries(tiler_legacy PRIVATE CGAL::CGAL)
    target_include_directories(tiler_legacy PRIVATE ${Boost_INCLUDE_DIRS})
    target_compile_definitions(tiler_legacy PRIVATE TIXML_USE_STL _USE_MATH_DEFINES
      _HAS_AUTO_PTR_ETC=1)
    set_target_properties(tiler_legacy PROPERTIES CXX_STANDARD 17)
    if(MSVC)
      # xml/Ser_reader.cpp uses std::ptr_fun without <functional>, which libstdc++ brings in anyway.
      set_source_files_properties("${_ct_dir}/xml/Ser_reader.cpp" PROPERTIES COMPILE_OPTIONS
        "/FIfunctional")
    endif()
    if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
      target_compile_options(tiler_legacy PRIVATE -frounding-math)
    endif()
    message(STATUS "volrover-goldens: ContourTiler with CGAL ${CGAL_VERSION}")
  else()
    message(STATUS "volrover-goldens: no CGAL or Boost, so no ContourTiler")
  endif()
endif()

# PocketTunnel (src/PocketTunnel), when the checkout has it and CGAL is found, as for
# Skeletonization. Every source but main.cpp.unused is compiled, two of them from patched copies in
# the build tree; each patch must apply, or configuring fails:
#   - tcocone.cpp is Skeletonization's, with the same two CGAL_assertion()s whose has_vertex()
#     calls fill the indices the code then uses (a release build compiles them out): plain calls;
#   - pocket_tunnel.cpp calls pocket_tunnel_driver_report() (pockettunnel_driver.cpp) after
#     detect_handle(), so the driver can read the triangulation and clusters the function keeps in
#     locals. The hook only reads.
# shim/cvcraw_geometry/cvcgeom.h stands in for the legacy cvcgeom_t, and shim/PEDetection/Timer.h
# for a header datastruct.h includes and does not use. The legacy build's -DHUGE=1000000 is left
# out: no source uses it, and it breaks the C runtime's own HUGE (MSVC's math.h).
if(EXISTS "${VOLROVER_DIR}/src/PocketTunnel/pocket_tunnel.cpp")
  find_package(CGAL CONFIG QUIET)
  find_package(Boost QUIET)
  if(CGAL_FOUND AND Boost_FOUND)
    set(_pt_dir "${VOLROVER_DIR}/src/PocketTunnel")
    set(_pt_out "${CMAKE_CURRENT_BINARY_DIR}/pockettunnel_src")
    file(MAKE_DIRECTORY "${_pt_out}")
    set(_pt_sources "")
    set(_pt_in "${_pt_dir}/tcocone.cpp")
    set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${_pt_in}")
    file(READ "${_pt_in}" _text)
    foreach(_v v w)
      set(_old "CGAL_assertion(ch->has_vertex(${_v}, i${_v}) && (i${_v} != -1));")
      string(FIND "${_text}" "${_old}" _at)
      if(_at EQUAL -1)
        message(FATAL_ERROR "volrover-goldens: the patch for PocketTunnel's tcocone.cpp does not "
                            "apply: ${_old}")
      endif()
      string(REPLACE "${_old}" "ch->has_vertex(${_v}, i${_v});" _text "${_text}")
    endforeach()
    file(WRITE "${_pt_out}/tcocone.cpp" "${_text}")
    set(_pt_in "${_pt_dir}/pocket_tunnel.cpp")
    set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${_pt_in}")
    file(READ "${_pt_in}" _text)
    foreach(_old "#include <PocketTunnel/pocket_tunnel.h>" "detect_handle(triang, cluster_set);")
      string(FIND "${_text}" "${_old}" _at)
      if(_at EQUAL -1)
        message(FATAL_ERROR "volrover-goldens: the patch for PocketTunnel's pocket_tunnel.cpp does "
                            "not apply: ${_old}")
      endif()
    endforeach()
    string(REPLACE "#include <PocketTunnel/pocket_tunnel.h>"
      "#include <PocketTunnel/pocket_tunnel.h>\nnamespace PocketTunnel { void pocket_tunnel_driver_report(Triangulation &, map<int, cell_cluster> &, const vector<int> &); }"
      _text "${_text}")
    string(REPLACE "detect_handle(triang, cluster_set);"
      "detect_handle(triang, cluster_set); pocket_tunnel_driver_report(triang, cluster_set, sorted_smax_index_vector);"
      _text "${_text}")
    file(WRITE "${_pt_out}/pocket_tunnel.cpp" "${_text}")
    list(APPEND _pt_sources "${_pt_out}/tcocone.cpp" "${_pt_out}/pocket_tunnel.cpp")
    foreach(_f handle hfn_util init intersect op rcocone robust_cc smax util)
      list(APPEND _pt_sources "${_pt_dir}/${_f}.cpp")
    endforeach()
    add_executable(pockettunnel_legacy pockettunnel_driver.cpp ${_pt_sources})
    volrover_legacy(pockettunnel_legacy)
    target_link_libraries(pockettunnel_legacy PRIVATE CGAL::CGAL)
    target_include_directories(pockettunnel_legacy PRIVATE ${Boost_INCLUDE_DIRS})
    set_target_properties(pockettunnel_legacy PROPERTIES CXX_STANDARD 17)
    target_compile_definitions(pockettunnel_legacy PRIVATE _USE_MATH_DEFINES)
    if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
      target_compile_options(pockettunnel_legacy PRIVATE -frounding-math)
    endif()
    message(STATUS "volrover-goldens: PocketTunnel with CGAL ${CGAL_VERSION}")
  else()
    message(STATUS "volrover-goldens: no CGAL or Boost, so no PocketTunnel")
  endif()
endif()
