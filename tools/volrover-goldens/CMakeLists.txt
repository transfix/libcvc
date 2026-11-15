# The legacy VolumeRover libraries, built from a transfix/volrover checkout at the pinned sha
# (volrover.sha), to make the ports' T1 goldens (README.md). Never part of libcvc's build: make_goldens.py
# configures this directory on its own. GPL code (the legacy tree's own CGAL, for the libraries
# that need it) may be built here and only here; nothing of it is linked into libcvc.
cmake_minimum_required(VERSION 3.16)
project(volrover_goldens CXX)

set(VOLROVER_DIR "" CACHE PATH "A transfix/volrover checkout at the sha in volrover.sha")
if(NOT EXISTS "${VOLROVER_DIR}/src/Segmentation/GenSeg/segment.cpp" AND
   NOT EXISTS "${VOLROVER_DIR}/src/Segmentation/SegMed/GVF.cpp" AND
   NOT EXISTS "${VOLROVER_DIR}/src/Skeletonization/Skeletonization.cpp")
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
