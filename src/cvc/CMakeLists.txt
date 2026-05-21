cmake_minimum_required(VERSION 3.15)

# Platform-specific definitions
if(WIN32)
  add_compile_definitions(__WINDOWS__)
endif()

# Header files
set(INCLUDE_FILES
  ../../inc/cvc/composite_function.h
  ../../inc/cvc/types.h
  ../../inc/cvc/project_verts.h
  ../../inc/cvc/hdf5_utils.h
  ../../inc/cvc/volume_file_info.h
  ../../inc/cvc/exception.h
  ../../inc/cvc/bounding_box.h
  ../../inc/cvc/namespace.h
  ../../inc/cvc/volume_file_io.h
  ../../inc/cvc/volume.h
  ../../inc/cvc/algorithm.h
  ../../inc/cvc/dimension.h
  ../../inc/cvc/voxels.h
  ../../inc/cvc/geometry.h
  ../../inc/cvc/state.h
  ../../inc/cvc/state_change_journal.h
  ../../inc/cvc/state_subscription_router.h
  ../../inc/cvc/state_sync_adapter.h
  ../../inc/cvc/state_blob_store.h
  ../../inc/cvc/state_codec_registry.h
  ../../inc/cvc/state_replica.h
  ../../inc/cvc/state_authority_map.h
  ../../inc/cvc/state_cluster_shard.h
  ../../inc/cvc/state_message.h
  ../../inc/cvc/state_message_bus.h
  ../../inc/cvc/state_transport.h
  ../../inc/cvc/state_transport_inproc.h
  ../../inc/cvc/state_transport_ipc.h
  ../../inc/cvc/state_transport_grpc.h
  ../../inc/cvc/bunny.h
  ../../inc/cvc/endians.h
  ../../inc/cvc/utility.h
  ../../inc/cvc/state_object.h
  ../../inc/cvc/app.h
  ../../inc/cvc/volmagick.h
  ../../inc/cvc/geometry_file_io.h
  ../../inc/cvc/volume_ops.h
)

# Source files
set(SOURCE_FILES
  algorithm.cpp
  anisotropic_diffusion.cpp
  app.cpp
  bilateral_filter.cpp
  contrast_enhancement.cpp
  cvcraw_io.cpp
  gdtv_filter.cpp
  geometry.cpp
  geometry_file_io.cpp
  mrc_io.cpp
  null_io.cpp
  off_io.cpp
  rawiv_io.cpp
  rawv_io.cpp
  smoothing.cpp
  spider_io.cpp
  state.cpp
  state_change_journal.cpp
  state_subscription_router.cpp
  state_sync_adapter.cpp
  state_blob_store.cpp
  state_codec_registry.cpp
  state_replica.cpp
  state_authority_map.cpp
  state_cluster_shard.cpp
  state_transport_inproc.cpp
  state_transport_ipc.cpp
  state_message_bus.cpp
  utility.cpp
  cuda_utils.cpp
  voxels.cpp
  volume.cpp
  volume_file_io.cpp
  volume_file_info.cpp
  volume_ops.cpp
  vtk_io.cpp
)

# Configuration
set(CVC_NAMESPACE cvc CACHE STRING "C++ namespace for CVC library")
mark_as_advanced(CVC_NAMESPACE)

# The libcvc version number
set(CVC_VERSION_MAJOR 3)
set(CVC_VERSION_MINOR 0)
set(CVC_VERSION ${CVC_VERSION_MAJOR}.${CVC_VERSION_MINOR})
message(STATUS "libcvc version: ${CVC_VERSION}")

set(CVC_LINK_LIBS "")

# Build options
option(CVC_USING_XMLRPC "Enable cvc::state network sharing support" OFF)
option(CVC_USING_HDF5 "Enable HDF5 support for *.cvc file format" ON)
option(CVC_USING_IMOD_MRC "Use IMOD's MRC loading routines" ON)
option(CVC_LOG4CPLUS_DEFAULT "Use log4cplus output by default" OFF)
option(LOG4CPLUS_DISABLE_TRACE "Disable logging trace" OFF)
option(CVC_APP_XML_PROPERTY_TREE "Use XML for property map (otherwise Boost INFO format)" OFF)
option(CVC_GEOMETRY_ENABLE_BUNNY "Enable built-in bunny geometry test tri mesh" ON)
option(CVC_GEOMETRY_CORRECT_INDEX_START "Adjust indices if they start from 1 rather than 0" OFF)
option(CVC_ENABLE_MESHER "Enable isosurfacing/meshing capability" ON)
option(CVC_ENABLE_SDF "Enable signed distance function calculation capability" ON)
option(CVC_ENABLE_GRPC "Enable gRPC-based state_transport (Phase 3e)" OFF)

# XMLRPC support
if(CVC_USING_XMLRPC)
  list(APPEND CVC_LINK_LIBS xmlrpc)
  list(APPEND SOURCE_FILES
    xmlrpc_client.cpp
    xmlrpc_server.cpp
  )
  add_compile_definitions(USING_XMLRPC)
endif()

# HDF5 support
if(CVC_USING_HDF5)
  # Prefer CMake CONFIG mode first: vcpkg ships a working
  # hdf5-config.cmake (lowercase package name 'hdf5') that exposes
  # imported targets like hdf5::hdf5-shared / hdf5::hdf5_cpp-shared.
  # The bundled FindHDF5.cmake module-mode probe does not understand
  # vcpkg's layout on Windows: it sniffs a stray header for the
  # version, returns "found version 2.1.1" but never sets
  # HDF5_INCLUDE_DIRS, so the build silently disables *.cvc support.
  # Fall back to module mode for system installs (apt libhdf5-dev,
  # brew hdf5) which provide a working FindHDF5 path but no config.
  set(_hdf5_config_targets "")
  find_package(hdf5 CONFIG QUIET)
  if(hdf5_FOUND)
    # vcpkg names imported targets hdf5::hdf5-shared / -static (and
    # _cpp variants); brew + upstream HDF5 ship hdf5::hdf5 /
    # hdf5::hdf5_cpp with no shared/static suffix. Probe both.
    foreach(_cand
        hdf5::hdf5-shared hdf5::hdf5-static hdf5::hdf5
        hdf5::hdf5_cpp-shared hdf5::hdf5_cpp-static hdf5::hdf5_cpp)
      if(TARGET ${_cand})
        list(APPEND _hdf5_config_targets ${_cand})
      endif()
    endforeach()
  endif()
  if(_hdf5_config_targets)
    set(HDF5_FOUND TRUE)
    set(HDF5_LIBRARIES ${_hdf5_config_targets})
    set(HDF5_INCLUDE_DIRS "")  # imported targets carry includes
    set(HDF5_DEFINITIONS "")
  else()
    # Either no hdf5-config.cmake on prefix path, or its targets
    # don't match anything we recognize. Fall back to module-mode
    # FindHDF5, which works for apt libhdf5-dev and most system
    # installs that ship a usable FindHDF5.cmake path.
    find_package(HDF5 COMPONENTS C CXX)
  endif()
  if(HDF5_FOUND)
    message(STATUS "HDF5 found - Enabling *.cvc file support")

    add_compile_definitions(CVC_USING_HDF5 ${HDF5_DEFINITIONS})
    list(APPEND INCLUDE_FILES ../../inc/cvc/hdf5_utils.h)
    list(APPEND SOURCE_FILES hdf5_io.cpp hdf5_utils.cpp)
    list(APPEND CVC_INCLUDE_DIRS ${HDF5_INCLUDE_DIRS})
    list(APPEND CVC_LINK_LIBS ${HDF5_LIBRARIES})
  else()
    message(WARNING "HDF5 not found - *.cvc file support disabled")
    set(CVC_HDF5_DISABLED TRUE)
  endif()
else()
  set(CVC_HDF5_DISABLED TRUE)
endif()

# IMOD MRC support
#
# libiimod is no longer vendored in this tree.  It is provided by the
# libcvc-deps distribution (built from the upstream LabShare-Archive/IMOD
# repository at a pinned SHA) as a static `libiimod::iimod` CMake target
# that already carries its include directory and the `NOTIFFLIBS`
# compile definition.  We only need to depend on it.
if(CVC_USING_IMOD_MRC)
  find_package(libiimod CONFIG REQUIRED)
  add_compile_definitions(CVC_USING_IMOD_MRC)
  list(APPEND CVC_LINK_LIBS libiimod::iimod)
endif()

# Log4cplus support
if(CVC_LOG4CPLUS_DEFAULT)
  find_package(Log4cplus)
  if(LOG4CPLUS_FOUND)
    add_compile_definitions(USING_LOG4CPLUS_DEFAULT)
    list(APPEND CVC_INCLUDE_DIRS ${LOG4CPLUS_INCLUDE_DIR})
    list(APPEND CVC_LINK_LIBS ${LOG4CPLUS_LIBRARIES})
  else()
    message(WARNING "Log4cplus not found - Log4cplus based logging disabled")
  endif()
endif()

# Property tree format
if(CVC_APP_XML_PROPERTY_TREE)
  add_compile_definitions(CVC_APP_XML_PROPERTY_TREE)
endif()

# Bunny geometry
if(CVC_GEOMETRY_ENABLE_BUNNY)
  add_compile_definitions(CVC_GEOMETRY_ENABLE_BUNNY)
  list(APPEND SOURCE_FILES bunny_io.cpp)
endif()

# CGAL support
if(NOT DISABLE_CGAL)
  find_package(CGAL CONFIG)
  if(CGAL_FOUND)
    add_compile_definitions(CVC_GEOMETRY_ENABLE_PROJECT)
    list(APPEND INCLUDE_FILES ../../inc/cvc/project_verts.h)
  endif()
endif()

# IDE source organization
source_group("Source Files" FILES ${SOURCE_FILES})
source_group("Header Files" FILES ${INCLUDE_FILES})

# Mesher support
if(CVC_ENABLE_MESHER)
  # TODO: Address duplicate volmagick code (noted 1/9/2014)
  add_compile_definitions(
    CVC_ENABLE_MESHER
    _BOOL
    SP2
    USEDICT
    NDEBUG
  )
  list(APPEND CVC_INCLUDE_DIRS
    cvc-mesher
    cvc-mesher/contour
    cvc-mesher/FastContouring
    #cvc-mesher/VolMagick
    cvc-mesher/LBIE
    cvc-mesher/Mesher
  )
  set(INCLUDE_FILES
      ${INCLUDE_FILES}
      cvc-mesher/contour/edgehash.h
      cvc-mesher/contour/respprop2.h
      cvc-mesher/contour/hash.h
      cvc-mesher/contour/conplot2d.h
      cvc-mesher/contour/cellsearch.h
      cvc-mesher/contour/data.h
      cvc-mesher/contour/seedchkr2.h
      cvc-mesher/contour/dataset.h
      cvc-mesher/contour/cellqueue.h
      cvc-mesher/contour/cubes.h
      cvc-mesher/contour/datasetreg3.h
      cvc-mesher/contour/dirseeds.h
      cvc-mesher/contour/seeddirreg3.h
      cvc-mesher/contour/basic.h
      cvc-mesher/contour/dataslc.h
      cvc-mesher/contour/contour.h
      cvc-mesher/contour/dict.h
      cvc-mesher/contour/contour2d.h
      cvc-mesher/contour/seedchkr3.h
      cvc-mesher/contour/iqueue.h
      cvc-mesher/contour/segtree.h
      cvc-mesher/contour/commdata.h
      cvc-mesher/contour/bin.h
      cvc-mesher/contour/datasetslc.h
      cvc-mesher/contour/inttree.h
      cvc-mesher/contour/datasetvol.h
      cvc-mesher/contour/ipqueue.h
      cvc-mesher/contour/seedall.h
      cvc-mesher/contour/contour3d.h
      cvc-mesher/contour/regprop.h
      cvc-mesher/contour/compute.h
      cvc-mesher/contour/datavol.h
      cvc-mesher/contour/vtkmarchingcubescases.h
      cvc-mesher/contour/shelf.h
      cvc-mesher/contour/range.h
      cvc-mesher/contour/rangesweep.h
      cvc-mesher/contour/endian_io.h
      cvc-mesher/contour/bucketsearch.h
      cvc-mesher/contour/seedcells.h
      cvc-mesher/contour/conplotreg2.h
      cvc-mesher/contour/datareg3.h
      cvc-mesher/contour/conplot3d.h
      cvc-mesher/contour/conplot.h
      cvc-mesher/contour/utilities.h
      cvc-mesher/contour/datasetreg2.h
      cvc-mesher/contour/dirseedsreg2.h
      cvc-mesher/contour/regprop2.h
      cvc-mesher/contour/queue.h
      cvc-mesher/contour/conplotreg3.h
      cvc-mesher/contour/datareg2.h
      cvc-mesher/contour/squeue.h
      cvc-mesher/contour/rangeprop.h
      cvc-mesher/FastContouring/ContourGeometry.h
      cvc-mesher/FastContouring/cubes.h
      cvc-mesher/FastContouring/Matrix.h
      cvc-mesher/FastContouring/FastContouring.h
      cvc-mesher/FastContouring/Vector.h
      cvc-mesher/FastContouring/MarchingCubesBuffers.h
      cvc-mesher/FastContouring/Ray.h
      cvc-mesher/FastContouring/Quaternion.h
      cvc-mesher/FastContouring/Tuple.h
      cvc-mesher/LBIE/cubes.h
      cvc-mesher/LBIE/pcio.h
      cvc-mesher/LBIE/LBIE_Mesher.h
      cvc-mesher/LBIE/normalspline.h
      cvc-mesher/LBIE/vtkMarchingCubesCases.h
      cvc-mesher/LBIE/LBIE_geoframe.h
      cvc-mesher/LBIE/geoframe_adapter.h
      cvc-mesher/LBIE/e_face.h
      cvc-mesher/LBIE/endians.h
      cvc-mesher/LBIE/octree.h
      cvc-mesher/LBIE/LBIEMesherRenderable.h
      cvc-mesher/Mesher/mesher.h
  )
  set(SOURCE_FILES
      ${SOURCE_FILES}
      cvc-mesher/contour/squeue.cpp
      cvc-mesher/contour/rangesweep.cpp
      cvc-mesher/contour/dirseeds.cpp
      cvc-mesher/contour/contour.cpp
      cvc-mesher/contour/seeddirreg3.cpp
      cvc-mesher/contour/contour3d.cpp
      cvc-mesher/contour/conplotreg3.cpp
      cvc-mesher/contour/dict.c
      cvc-mesher/contour/edgehash.cpp
      cvc-mesher/contour/regprop2.cpp
      cvc-mesher/contour/spectrumtest.cpp
      cvc-mesher/contour/ipqueue.cpp
      cvc-mesher/contour/seedcells.cpp
      cvc-mesher/contour/conplot3d.cpp
      cvc-mesher/contour/dataslc.cpp
      cvc-mesher/contour/cubes.cpp
      cvc-mesher/contour/conplot2d.cpp
      cvc-mesher/contour/shelf.cpp
      cvc-mesher/contour/iqueue.cpp
      cvc-mesher/contour/datavol.cpp
      cvc-mesher/contour/cellqueue.cpp
      cvc-mesher/contour/bin.cpp
      cvc-mesher/contour/datareg2.cpp
      cvc-mesher/contour/bucketsearch.cpp
      cvc-mesher/contour/seedall.cpp
      cvc-mesher/contour/range.cpp
      cvc-mesher/contour/hash.cpp
      cvc-mesher/contour/conplotreg2.cpp
      cvc-mesher/contour/data.cpp
      cvc-mesher/contour/inttree.cpp
      cvc-mesher/contour/seedchkr3.cpp
      cvc-mesher/contour/conplot.cpp
      cvc-mesher/contour/queue.cpp
      #cvc-mesher/contour/main.cpp
      cvc-mesher/contour/regprop.cpp
      cvc-mesher/contour/respprop2.cpp
      cvc-mesher/contour/rangeprop.cpp
      cvc-mesher/contour/segtree.cpp
      cvc-mesher/contour/conplot_p.cpp
      cvc-mesher/contour/contour2d.cpp
      cvc-mesher/contour/datareg3.cpp
      cvc-mesher/contour/dirseedsreg2.cpp
      cvc-mesher/contour/seedchkr2.cpp
      cvc-mesher/FastContouring/Ray.cpp
      cvc-mesher/FastContouring/FastContouring.cpp
      cvc-mesher/FastContouring/Quaternion.cpp
      cvc-mesher/FastContouring/Tuple.cpp
      cvc-mesher/FastContouring/Vector.cpp
      cvc-mesher/FastContouring/ContourGeometry.cpp
      cvc-mesher/FastContouring/Matrix.cpp
      cvc-mesher/FastContouring/MarchingCubesBuffers.cpp
      cvc-mesher/LBIE/normalspline.cpp
      cvc-mesher/LBIE/e_face.cpp
      cvc-mesher/LBIE/pcio.cpp
      cvc-mesher/LBIE/LBIE_geoframe.cpp
      cvc-mesher/LBIE/geoframe_adapter.cpp
      cvc-mesher/LBIE/tetra.cpp
      cvc-mesher/LBIE/hexa.cpp
      cvc-mesher/LBIE/LBIE_Mesher.cpp
      #cvc-mesher/LBIE/test.cpp
      cvc-mesher/LBIE/octree.cpp
      #cvc-mesher/Mesher/main.cpp
      cvc-mesher/Mesher/mesher.cpp
  )
endif(CVC_ENABLE_MESHER)

# Signed distance function support
if(CVC_ENABLE_SDF)
  list(APPEND CVC_INCLUDE_DIRS
    SDF/UsefulMath
    SDF/GeometryFileTypes
    SDF/ByteOrder
    SDF/SignDistanceFunction
    SDF/Geometry
    SDF/VolumeFileTypes
    SDF/SDF
    SDF/SignDistanceFunction_v2
  )
  set(INCLUDE_FILES
      ${INCLUDE_FILES}
      # SDF/UsefulMath/AreaVolumes.h
      # SDF/UsefulMath/Matrix.h
      # SDF/UsefulMath/TrilinearGrid.h
      # SDF/UsefulMath/LinearAlgebra.h
      # SDF/UsefulMath/Vector.h
      # SDF/UsefulMath/Ray.h
      # SDF/UsefulMath/Quaternion.h
      # SDF/UsefulMath/Tuple.h
      # SDF/GeometryFileTypes/RawnFile.h
      # SDF/GeometryFileTypes/RawncFile.h
      # SDF/GeometryFileTypes/MayaOBJFile.h
      # SDF/GeometryFileTypes/RawcFile.h
      # SDF/GeometryFileTypes/RawFile.h
      # SDF/GeometryFileTypes/GeometryFileType.h
      # SDF/GeometryFileTypes/GeometryLoader.h
      #SDF/ByteOrder/ByteSwapping.h
      SDF/SignDistanceFunction/common.h
      SDF/SignDistanceFunction/sdfLib.h
      SDF/SignDistanceFunction/head.h
      SDF/SignDistanceFunction/SDFContext.h
      #SDF/Geometry/IntQueue.h
      #SDF/Geometry/GeometrySceneArray.h
      #SDF/Geometry/Geometry.h
      #SDF/Geometry/SceneArrayNode.h
      #SDF/Geometry/MeshDerivatives.h
      #SDF/Geometry/GeometryScene.h
      #SDF/VolumeFileTypes/RawVFile.h
      #SDF/VolumeFileTypes/RawIVFile.h
      #SDF/VolumeFileTypes/MRCFile.h
      #SDF/VolumeFileTypes/SimpleVolumeData.h
      #SDF/VolumeFileTypes/DXFile.h
      #SDF/VolumeFileTypes/VolumeLoader.h
      #SDF/VolumeFileTypes/VolumeFileType.h
      # SDF/SignDistanceFunction_v2/Geom3DParser.h
      # SDF/SignDistanceFunction_v2/reg3data.h
      # SDF/SignDistanceFunction_v2/mtxlib.h
      # SDF/SignDistanceFunction_v2/bufferedio.h
      # SDF/SignDistanceFunction_v2/geom.h
      # SDF/SignDistanceFunction_v2/cubes.h
      # SDF/SignDistanceFunction_v2/diskio.h
      # SDF/SignDistanceFunction_v2/FaceVertSet3D.h
      # SDF/SignDistanceFunction_v2/dynarray.h
      # SDF/SignDistanceFunction_v2/Reg3Parser.h
      # SDF/SignDistanceFunction_v2/DistanceTransform.h
      # SDF/SignDistanceFunction_v2/bio.h
      # SDF/SignDistanceFunction_v2/RawivParser.h
  )
  set(SOURCE_FILES
      ${SOURCE_FILES}
      # SDF/UsefulMath/TrilinearGrid.cpp
      # SDF/UsefulMath/Ray.cpp
      # SDF/UsefulMath/Quaternion.cpp
      # SDF/UsefulMath/Tuple.cpp
      # SDF/UsefulMath/Vector.cpp
      # SDF/UsefulMath/Matrix.cpp
      # SDF/UsefulMath/AreaVolumes.cpp
      # SDF/UsefulMath/LinearAlgebra.cpp
      # SDF/GeometryFileTypes/GeometryFileType.cpp
      # SDF/GeometryFileTypes/RawnFile.cpp
      # SDF/GeometryFileTypes/MayaOBJFile.cpp
      # SDF/GeometryFileTypes/RawcFile.cpp
      # SDF/GeometryFileTypes/GeometryLoader.cpp
      # SDF/GeometryFileTypes/RawFile.cpp
      # SDF/GeometryFileTypes/RawncFile.cpp
      # SDF/ByteOrder/ByteSwapping.cpp
      # SDF/SignDistanceFunction/init.cpp  # OLD - uses global sdf, all functions moved to SDFContext
      SDF/SignDistanceFunction/compute.cpp
      SDF/SignDistanceFunction/propagate.cpp
      SDF/SignDistanceFunction/head.cpp
      SDF/SignDistanceFunction/new_adjust.cpp
      SDF/SignDistanceFunction/main.cpp
      SDF/SignDistanceFunction/common.cpp
      SDF/SignDistanceFunction/octree.cpp
      SDF/SignDistanceFunction/SDFContext.cpp
      #SDF/Geometry/GeometryScene.cpp
      #SDF/Geometry/GeometrySceneArray.cpp
      #SDF/Geometry/Geometry.cpp
      #SDF/Geometry/SceneArrayNode.cpp
      #SDF/Geometry/IntQueue.cpp
      #SDF/Geometry/MeshDerivatives.cpp
      #SDF/VolumeFileTypes/VolumeLoader.cpp
      #SDF/VolumeFileTypes/DXFile.cpp
      #SDF/VolumeFileTypes/RawIVFile.cpp
      #SDF/VolumeFileTypes/RawVFile.cpp
      #SDF/VolumeFileTypes/VolumeFileType.cpp
      #SDF/VolumeFileTypes/MRCFile.cpp
      #SDF/VolumeFileTypes/SimpleVolumeData.cpp
      #SDF/Server.cpp
      #SDF/SDF/main.cpp
      SDF/SignDistanceFunction_v2/Reg3Parser.cpp
      SDF/SignDistanceFunction_v2/RawivParser.cpp
      SDF/SignDistanceFunction_v2/Geom3DParser.cpp
      SDF/SignDistanceFunction_v2/geom.cpp
      SDF/SignDistanceFunction_v2/FaceVertSet3D.cpp
      SDF/SignDistanceFunction_v2/DistanceTransform.cpp
      SDF/SignDistanceFunction_v2/mtxlib.cpp
      SDF/SignDistanceFunction_v2/bufferedio.cpp
  )
endif(CVC_ENABLE_SDF)

# Optional: Poco HTTP server support (currently disabled)
# find_package(Poco)
# if(Poco_FOUND)
#   add_compile_definitions(CVC_USING_POCO)
#   list(APPEND CVC_INCLUDE_DIRS ${POCO_INCLUDE_DIRS})
#   list(APPEND CVC_LINK_LIBS ${POCO_LIBRARIES})
#   message(STATUS "Poco libs: ${POCO_LIBRARIES}")
# endif()

# Add CUDA source files if enabled
if(CVC_ENABLE_CUDA)
  list(APPEND SOURCE_FILES voxels_kernels.cu)
  message(STATUS "CUDA enabled for libcvc - including voxels_kernels.cu")
endif()

# ImageMagick support (for volume_ops image I/O)
#
# Quantum-depth and HDRI flags must match the installed library's build:
# Q8 vs Q16, HDRI on/off. On Linux/macOS we ask pkg-config for the
# canonical cflags, which include the right `MAGICKCORE_QUANTUM_DEPTH`
# and `MAGICKCORE_HDRI_ENABLE` defines. On Windows our overlay vcpkg
# port ships Q16-HDRI x64.
option(CVC_ENABLE_IMAGEMAGICK "Enable ImageMagick support for image I/O in volume_ops" ON)
if(CVC_ENABLE_IMAGEMAGICK)
  set(CVC_IMAGEMAGICK_QUANTUM_DEPTH 16)
  set(CVC_IMAGEMAGICK_HDRI_ENABLE 0)
  if(NOT WIN32)
    find_package(PkgConfig QUIET)
    if(PkgConfig_FOUND)
      pkg_check_modules(IMAGEMAGICK_PC QUIET Magick++)
      if(IMAGEMAGICK_PC_FOUND)
        # Extract Q-depth / HDRI from -D flags. Drop them from the cflag
        # list afterwards so we add them once via target_compile_definitions
        # below (avoids per-source duplication).
        foreach(_def IN LISTS IMAGEMAGICK_PC_CFLAGS_OTHER)
          if(_def MATCHES "^-DMAGICKCORE_QUANTUM_DEPTH=([0-9]+)$")
            set(CVC_IMAGEMAGICK_QUANTUM_DEPTH "${CMAKE_MATCH_1}")
          elseif(_def MATCHES "^-DMAGICKCORE_HDRI_ENABLE=([0-9]+)$")
            set(CVC_IMAGEMAGICK_HDRI_ENABLE "${CMAKE_MATCH_1}")
          endif()
        endforeach()
      endif()
    endif()
  else()
    set(CVC_IMAGEMAGICK_HDRI_ENABLE 1)  # Q16-HDRI x64 from vcpkg overlay
  endif()
  find_package(ImageMagick COMPONENTS Magick++ MagickCore)
  if(ImageMagick_FOUND)
    message(STATUS "ImageMagick found: ${ImageMagick_INCLUDE_DIRS} "
                   "(Q${CVC_IMAGEMAGICK_QUANTUM_DEPTH}, "
                   "HDRI=${CVC_IMAGEMAGICK_HDRI_ENABLE})")
  else()
    message(STATUS "ImageMagick not found - disabling image I/O in volume_ops")
    set(CVC_ENABLE_IMAGEMAGICK OFF)
  endif()
endif()

# FFTW support (for volume_ops filtered back-projection)
option(CVC_ENABLE_FFTW "Enable FFTW support for filtered back-projection in volume_ops" ON)
if(CVC_ENABLE_FFTW)
  find_package(PkgConfig QUIET)
  if(PkgConfig_FOUND)
    pkg_check_modules(FFTW3 QUIET fftw3)
    # pkg_check_modules sets <prefix>_LIBRARIES to bare `-l` names and puts
    # full paths in <prefix>_LINK_LIBRARIES. Prefer the full-path form so
    # the linker doesn't need a separate -L for Homebrew etc.
    if(FFTW3_FOUND AND FFTW3_LINK_LIBRARIES)
      set(FFTW3_LIBRARIES ${FFTW3_LINK_LIBRARIES})
    endif()
  endif()
  if(NOT FFTW3_FOUND)
    find_path(FFTW3_INCLUDE_DIRS fftw3.h)
    find_library(FFTW3_LIBRARIES NAMES fftw3)
    if(FFTW3_INCLUDE_DIRS AND FFTW3_LIBRARIES)
      set(FFTW3_FOUND TRUE)
    endif()
  endif()
  if(FFTW3_FOUND)
    message(STATUS "FFTW3 found: ${FFTW3_LIBRARIES}")
  else()
    message(STATUS "FFTW3 not found - disabling filtered back-projection in volume_ops")
    set(CVC_ENABLE_FFTW OFF)
  endif()
endif()

# gRPC-backed state transport (Phase 3e). Generates protobuf and
# gRPC C++ stubs from libcvc/proto/state_transport.proto and links
# the cvc target against gRPC::grpc++ + protobuf::libprotobuf.
if(CVC_ENABLE_GRPC)
  find_package(Protobuf CONFIG QUIET)
  if(NOT Protobuf_FOUND)
    find_package(Protobuf REQUIRED)
  endif()
  find_package(gRPC CONFIG REQUIRED)

  # Locate grpc_cpp_plugin executable.
  if(TARGET gRPC::grpc_cpp_plugin)
    set(_grpc_cpp_plugin_target gRPC::grpc_cpp_plugin)
    get_target_property(_grpc_cpp_plugin_path gRPC::grpc_cpp_plugin
      IMPORTED_LOCATION_RELEASE)
    if(NOT _grpc_cpp_plugin_path)
      get_target_property(_grpc_cpp_plugin_path gRPC::grpc_cpp_plugin
        IMPORTED_LOCATION)
    endif()
  endif()
  if(NOT _grpc_cpp_plugin_path)
    find_program(_grpc_cpp_plugin_path grpc_cpp_plugin)
    if(NOT _grpc_cpp_plugin_path)
      message(FATAL_ERROR "CVC_ENABLE_GRPC: grpc_cpp_plugin not found")
    endif()
  endif()

  # Locate protoc.
  if(TARGET protobuf::protoc)
    set(_protoc_target protobuf::protoc)
  else()
    find_program(_protoc_path protoc REQUIRED)
  endif()

  set(_proto_src "${CMAKE_SOURCE_DIR}/proto/state_transport.proto")
  set(_proto_out_dir "${CMAKE_CURRENT_BINARY_DIR}/proto")
  file(MAKE_DIRECTORY "${_proto_out_dir}")
  set(_proto_pb_cc   "${_proto_out_dir}/state_transport.pb.cc")
  set(_proto_pb_h    "${_proto_out_dir}/state_transport.pb.h")
  set(_proto_grpc_cc "${_proto_out_dir}/state_transport.grpc.pb.cc")
  set(_proto_grpc_h  "${_proto_out_dir}/state_transport.grpc.pb.h")

  if(_protoc_target)
    set(_protoc_cmd $<TARGET_FILE:${_protoc_target}>)
  else()
    set(_protoc_cmd "${_protoc_path}")
  endif()

  add_custom_command(
    OUTPUT ${_proto_pb_cc} ${_proto_pb_h} ${_proto_grpc_cc} ${_proto_grpc_h}
    COMMAND ${_protoc_cmd}
      --proto_path=${CMAKE_SOURCE_DIR}/proto
      --cpp_out=${_proto_out_dir}
      --grpc_out=${_proto_out_dir}
      --plugin=protoc-gen-grpc=${_grpc_cpp_plugin_path}
      ${_proto_src}
    DEPENDS ${_proto_src}
    COMMENT "Generating protobuf/gRPC C++ stubs for state_transport.proto"
    VERBATIM
  )

  list(APPEND SOURCE_FILES
    state_transport_grpc.cpp
    ${_proto_pb_cc}
    ${_proto_grpc_cc}
  )
endif()

# Create the main library
add_library(cvc ${SOURCE_FILES} ${INCLUDE_FILES})
add_library(cvc::cvc ALIAS cvc)

# Set library version
set_target_properties(cvc PROPERTIES
  VERSION ${CVC_VERSION}
  SOVERSION ${CVC_VERSION_MAJOR}
  OUTPUT_NAME cvc
)

# On Windows, the sources have no __declspec(dllexport) annotations, so
# a SHARED build would produce a .dll with an empty export table and no
# import .lib — breaking every consumer (including our own tests).
# Ask CMake to auto-generate the export table from all public symbols.
if(WIN32)
  set_target_properties(cvc PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS ON)
endif()

# CUDA-specific configuration
if(CVC_ENABLE_CUDA)
  # Set CUDA properties for the target
  set_target_properties(cvc PROPERTIES
    CUDA_SEPARABLE_COMPILATION ON
    CUDA_RESOLVE_DEVICE_SYMBOLS ON
  )
  
  # Add CUDA compile definitions
  target_compile_definitions(cvc PUBLIC CVC_USING_CUDA)
  
  # CUDA-specific compile options can be added here
  # target_compile_options(cvc PRIVATE
  #   $<$<COMPILE_LANGUAGE:CUDA>:--expt-relaxed-constexpr>
  # )
endif()

# Modern include directory handling
# Split CVC_INCLUDE_DIRS: absolute paths (external deps like HDF5, FFTW,
# Log4cplus) go PUBLIC so downstream consumers that include transitive
# headers (e.g. cvc/hdf5_utils.h) can find them. Relative paths (e.g.
# libiimod, SDF internals) stay PRIVATE.
set(_CVC_PUBLIC_INCLUDE_DIRS "")
set(_CVC_PRIVATE_INCLUDE_DIRS "")
foreach(_inc IN LISTS CVC_INCLUDE_DIRS)
  if(IS_ABSOLUTE "${_inc}")
    list(APPEND _CVC_PUBLIC_INCLUDE_DIRS "${_inc}")
  else()
    list(APPEND _CVC_PRIVATE_INCLUDE_DIRS "${_inc}")
  endif()
endforeach()

target_include_directories(cvc
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/inc>
    $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}/inc>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${_CVC_PRIVATE_INCLUDE_DIRS}
)

# Public absolute include dirs (external deps): wrap each in its own
# BUILD_INTERFACE genex so CMake's list separator (`;`) does not collide
# with the genex's argument boundary. Embedding a list directly in
# "$<BUILD_INTERFACE:${list}>" would either be syntactically invalid or
# silently drop all but the first entry.
foreach(_inc IN LISTS _CVC_PUBLIC_INCLUDE_DIRS)
  target_include_directories(cvc PUBLIC "$<BUILD_INTERFACE:${_inc}>")
endforeach()

# Compile definitions
target_compile_definitions(cvc
  PUBLIC
    CVC_NAMESPACE=${CVC_NAMESPACE}
  PRIVATE
    CVC_VERSION_STRING="${CVC_VERSION}"
)

# Public feature flags so consumers can check what libcvc was built with
if(CVC_ENABLE_MESHER)
  target_compile_definitions(cvc PUBLIC CVC_ENABLE_MESHER)
endif()
if(CVC_ENABLE_SDF)
  target_compile_definitions(cvc PUBLIC CVC_ENABLE_SDF)
endif()
if(CVC_ENABLE_GRPC)
  target_compile_definitions(cvc PUBLIC CVC_ENABLE_GRPC)
  target_include_directories(cvc PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
  target_link_libraries(cvc PRIVATE gRPC::grpc++ protobuf::libprotobuf)
endif()

# Pass DISABLE_CGAL to the preprocessor so source files can guard CGAL includes
if(DISABLE_CGAL)
  target_compile_definitions(cvc PUBLIC DISABLE_CGAL)
endif()

# Setup dependencies
SetupBoost(cvc)

if(NOT DISABLE_CGAL)
  SetupCGAL(cvc)
endif()

if(CVC_ENABLE_CUDA)
  SetupCUDA(cvc)
endif()

if(CVC_ENABLE_OPENMP AND OpenMP_CXX_FOUND)
  target_link_libraries(cvc PUBLIC OpenMP::OpenMP_CXX)
  target_compile_definitions(cvc PUBLIC CVC_USING_OPENMP)
  # voxels.cpp uses unsigned loop indices and OpenMP `min`/`max` reductions
  # that the classic MSVC OpenMP runtime (spec 2.0) rejects. Opt into the
  # LLVM OpenMP runtime on MSVC so those constructs compile as-is. Gate on
  # COMPILE_LANGUAGE:CXX so nvcc (which rejects the unknown flag with
  # "nvcc fatal: Unknown option '-openmp:llvm'") does not see it when it
  # walks cvc's INTERFACE_COMPILE_OPTIONS for .cu sources.
  if(MSVC)
    target_compile_options(cvc PRIVATE $<$<COMPILE_LANGUAGE:CXX>:-openmp:llvm>)
  endif()
endif()

# ImageMagick linking
if(CVC_ENABLE_IMAGEMAGICK)
  target_compile_definitions(cvc PUBLIC CVC_ENABLE_IMAGEMAGICK
    MAGICKCORE_QUANTUM_DEPTH=${CVC_IMAGEMAGICK_QUANTUM_DEPTH}
    MAGICKCORE_HDRI_ENABLE=${CVC_IMAGEMAGICK_HDRI_ENABLE})
  target_include_directories(cvc PUBLIC ${ImageMagick_INCLUDE_DIRS})
  target_link_libraries(cvc PUBLIC ${ImageMagick_LIBRARIES})
endif()

# FFTW linking
if(CVC_ENABLE_FFTW)
  target_compile_definitions(cvc PUBLIC CVC_ENABLE_FFTW)
  target_include_directories(cvc PUBLIC ${FFTW3_INCLUDE_DIRS})
  target_link_libraries(cvc PUBLIC ${FFTW3_LIBRARIES})
endif()

# Link libraries
target_link_libraries(cvc PUBLIC ${CVC_LINK_LIBS})

# Install targets
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

install(TARGETS cvc
  EXPORT cvcTargets
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT libcvc
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT libcvc
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT libcvc
)

install(EXPORT cvcTargets
  FILE cvcTargets.cmake
  NAMESPACE cvc::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/cvc
  COMPONENT libcvc
)

configure_package_config_file(
  "${PROJECT_SOURCE_DIR}/CMake/cvcConfig.cmake.in"
  "${CMAKE_BINARY_DIR}/cvcConfig.cmake"
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/cvc
)

write_basic_package_version_file(
  "${CMAKE_BINARY_DIR}/cvcConfigVersion.cmake"
  VERSION ${CVC_VERSION}
  COMPATIBILITY SameMajorVersion
)

install(FILES
  "${CMAKE_BINARY_DIR}/cvcConfig.cmake"
  "${CMAKE_BINARY_DIR}/cvcConfigVersion.cmake"
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/cvc
  COMPONENT libcvc
)

install(DIRECTORY ${PROJECT_SOURCE_DIR}/inc/cvc/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/cvc
  COMPONENT libcvc
  FILES_MATCHING PATTERN "*.h"
)

install(FILES ${PROJECT_BINARY_DIR}/inc/cvc/config.h
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/cvc
  COMPONENT libcvc
)

# Generate config.h
# Use PROJECT_BINARY_DIR (not CMAKE_BINARY_DIR) so the generated header lands
# under libcvc's own build tree when consumed via FetchContent/add_subdirectory.
# Downstream projects can then add ${libcvc_BINARY_DIR}/inc to their include path.
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/../../inc/cvc/config.h.cmake
  ${PROJECT_BINARY_DIR}/inc/cvc/config.h
)

# Windows-specific batch file
if(WIN32)
  file(TO_NATIVE_PATH "${Boost_LIBRARY_DIRS}" BOOST_LOCATION)
  file(TO_NATIVE_PATH "${CMAKE_BINARY_DIR}" BINARIES_LOCATION)
  configure_file(
    libcvc.bat.cmake
    ${CMAKE_BINARY_DIR}/libcvc.bat
  )
endif()

# Add unit tests if enabled
if(CVC_BUILD_TESTS)
  add_subdirectory(tests)
endif()
