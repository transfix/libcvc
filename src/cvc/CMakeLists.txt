cmake_minimum_required(VERSION 3.15)

# Platform-specific definitions
if(WIN32)
  add_compile_definitions(__WINDOWS__)
endif()

# Header files
set(INCLUDE_FILES
  ../../inc/cvc/utility/composite_function.h
  ../../inc/cvc/core/types.h
  ../../inc/cvc/geometry/project_verts.h
  ../../inc/cvc/volume/hdf5_utils.h
  ../../inc/cvc/volume/volume_file_info.h
  ../../inc/cvc/core/exception.h
  ../../inc/cvc/volume/bounding_box.h
  ../../inc/cvc/core/namespace.h
  ../../inc/cvc/volume/volume_file_io.h
  ../../inc/cvc/volume/volume.h
  ../../inc/cvc/utility/algorithm.h
  ../../inc/cvc/volume/dimension.h
  ../../inc/cvc/volume/voxels.h
  ../../inc/cvc/geometry/geometry.h
  ../../inc/cvc/core/state.h
  ../../inc/cvc/core/state_change_journal.h
  ../../inc/cvc/core/state_subscription_router.h
  ../../inc/cvc/core/state_sync_adapter.h
  ../../inc/cvc/core/state_blob_store.h
  ../../inc/cvc/core/state_chunked_blob.h
  ../../inc/cvc/core/state_compression_registry.h
  ../../inc/cvc/core/state_delta_codec.h
  ../../inc/cvc/core/state_codec_registry.h
  ../../inc/cvc/core/state_replica.h
  ../../inc/cvc/core/state_authority_map.h
  ../../inc/cvc/core/state_cluster_shard.h
  ../../inc/cvc/core/state_bounded_queue.h
  ../../inc/cvc/core/state_message.h
  ../../inc/cvc/core/state_message_bus.h
  ../../inc/cvc/core/state_peer_registry.h
  ../../inc/cvc/core/state_write_policy.h
  ../../inc/cvc/core/state_distributed_metrics.h
  ../../inc/cvc/core/state_delegation_manager.h
  ../../inc/cvc/core/state_distributed_admin.h
  ../../inc/cvc/core/state_volume_codec.h
  ../../inc/cvc/core/state_brick_manifest.h
  ../../inc/cvc/core/state_data_hydrator.h
  ../../inc/cvc/core/state_node_telemetry.h
  ../../inc/cvc/core/state_telemetry_aggregator.h
  ../../inc/cvc/core/state_cluster_membership.h
  ../../inc/cvc/core/state_hybrid_time.h
  ../../inc/cvc/core/world_clock.h
  ../../inc/cvc/core/state_hash_partition.h
  ../../inc/cvc/core/distributed_state_session.h
  ../../inc/cvc/core/state_transport.h
  ../../inc/cvc/core/state_transport_inproc.h
  ../../inc/cvc/core/state_transport_ipc.h
  ../../inc/cvc/core/state_transport_grpc.h
  ../../inc/cvc/geometry/bunny.h
  ../../inc/cvc/volume/endians.h
  ../../inc/cvc/utility/utility.h
  ../../inc/cvc/core/state_object.h
  ../../inc/cvc/core/state_list.h
  ../../inc/cvc/core/state_eviction_store.h
  ../../inc/cvc/core/state_memory_manager.h
  ../../inc/cvc/core/state_exec/types.h
  ../../inc/cvc/core/state_exec/parser.h
  ../../inc/cvc/core/state_exec/builtins.h
  ../../inc/cvc/core/state_exec/evaluator.h
  ../../inc/cvc/core/state_exec/stackless_evaluator.h
  ../../inc/cvc/core/state_exec/task.h
  ../../inc/cvc/core/state_exec/async_evaluator.h
  ../../inc/cvc/core/state_exec/async_stackless_evaluator.h
  ../../inc/cvc/core/state_exec/state_value_codec.h
  ../../inc/cvc/core/state_exec/process.h
  ../../inc/cvc/core/state_exec/memory_tracker.h
  ../../inc/cvc/core/state_exec/scheduler.h
  ../../inc/cvc/core/state_exec/async_scheduler.h
  ../../inc/cvc/core/state_exec/intrinsics.h
  ../../inc/cvc/core/state_exec/resource_policy.h
  ../../inc/cvc/core/state_exec/stdlib.h
  ../../inc/cvc/core/state_exec/exec_coordinator.h
  ../../inc/cvc/core/app.h
  ../../inc/cvc/volume/volmagick.h
  ../../inc/cvc/geometry/geometry_file_io.h
  ../../inc/cvc/model/model.h
  ../../inc/cvc/model/model_file_io.h
  ../../inc/cvc/volume/volume_ops.h
  ../../inc/cvc/nav/grid_nav.h
)

# Source files
set(SOURCE_FILES
  utility/algorithm.cpp
  processing/anisotropic_diffusion.cpp
  core/app.cpp
  processing/bilateral_filter.cpp
  processing/contrast_enhancement.cpp
  volume/cvcraw_io.cpp
  processing/gdtv_filter.cpp
  geometry/geometry.cpp
  geometry/geometry_file_io.cpp
  image/image.cpp
  image/magick_io.cpp
  model/model.cpp
  model/model_file_io.cpp
  model/assimp_io.cpp
  volume/mrc_io.cpp
  volume/null_io.cpp
  geometry/off_io.cpp
  volume/rawiv_io.cpp
  volume/rawv_io.cpp
  processing/smoothing.cpp
  volume/spider_io.cpp
  core/state.cpp
  core/state_list.cpp
  core/state_eviction_store.cpp
  core/state_memory_manager.cpp
  core/state_exec/types.cpp
  core/state_exec/parser.cpp
  core/state_exec/builtins.cpp
  core/state_exec/evaluator.cpp
  core/state_exec/stackless_evaluator.cpp
  core/state_exec/async_evaluator.cpp
  core/state_exec/async_stackless_evaluator.cpp
  core/state_exec/state_value_codec.cpp
  core/state_exec/process.cpp
  core/state_exec/memory_tracker.cpp
  core/state_exec/scheduler.cpp
  core/state_exec/async_scheduler.cpp
  core/state_exec/intrinsics.cpp
  core/state_exec/resource_policy.cpp
  core/state_exec/stdlib.cpp
  core/state_exec/exec_coordinator.cpp
  core/world_clock.cpp
  core/state_change_journal.cpp
  core/state_subscription_router.cpp
  core/state_sync_adapter.cpp
  core/state_blob_store.cpp
  core/state_chunked_blob.cpp
  core/state_compression_registry.cpp
  core/state_delta_codec.cpp
  core/state_codec_registry.cpp
  core/state_replica.cpp
  core/state_authority_map.cpp
  core/state_cluster_shard.cpp
  core/state_transport_inproc.cpp
  core/state_transport_ipc.cpp
  core/state_message_bus.cpp
  core/state_peer_registry.cpp
  core/state_write_policy.cpp
  core/state_distributed_metrics.cpp
  core/state_delegation_manager.cpp
  core/state_distributed_admin.cpp
  core/state_volume_codec.cpp
  core/state_brick_manifest.cpp
  core/state_data_hydrator.cpp
  core/state_node_telemetry.cpp
  core/state_telemetry_aggregator.cpp
  core/state_cluster_membership.cpp
  core/distributed_state_session.cpp
  core/state_message.cpp
  utility/utility.cpp
  utility/cuda_utils.cpp
  volume/voxels.cpp
  volume/volume.cpp
  volume/volume_file_io.cpp
  volume/volume_file_info.cpp
  volume/volume_ops.cpp
  volume/vtk_io.cpp
  nav/grid_nav.cpp
  nav/spatial.cpp
  nav/drive.cpp
  nav/material.cpp
  nav/coef_mlp.cpp
  nav/coef_train.cpp
  nav/belief_occupancy.cpp
  nav/sim_world.cpp
  nav/sim_thread.cpp
)

# material.cpp is a BIT-identical twin of a Python reference whose gaussian
# blur accumulates float64 with exp-derived weights: the compiler default
# ffp-contract (fast on GCC, on for Clang) would FMA-contract it on targets
# with FMA ISA (aarch64 macOS CI) and flip the f32-stored planes against the
# cross-platform goldens. The other nav TUs rely on the no-flags convention;
# this one gets the explicit flag (the CPU analog of the nav .cu precise-math
# properties below).
if(NOT MSVC)
  set_source_files_properties(nav/material.cpp PROPERTIES COMPILE_OPTIONS "-ffp-contract=off")
else()
  set_source_files_properties(nav/material.cpp PROPERTIES COMPILE_OPTIONS "/fp:precise")
endif()

# state_transport_ipc is POSIX-only (Unix domain sockets, poll()); the IPC
# header itself is portable but the implementation is not. Drop it on Windows.
if(WIN32)
  list(REMOVE_ITEM SOURCE_FILES core/state_transport_ipc.cpp)
endif()

# state_exec / state_memory_manager feature flags
option(CVC_STATE_EXEC "Enable state_exec DSL evaluator and process scheduler" ON)
option(CVC_STATE_MEMORY_MANAGER "Enable state memory manager with LRU/LFU eviction" ON)

if(NOT CVC_STATE_EXEC)
  list(FILTER SOURCE_FILES EXCLUDE REGEX "core/state_exec/")
endif()

# state_memory_manager feature flag
if(NOT CVC_STATE_MEMORY_MANAGER)
  list(REMOVE_ITEM SOURCE_FILES core/state_memory_manager.cpp core/state_eviction_store.cpp)
endif()

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
    xmlrpc/xmlrpc_client.cpp
    xmlrpc/xmlrpc_server.cpp
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
    list(APPEND INCLUDE_FILES ../../inc/cvc/volume/hdf5_utils.h)
    list(APPEND SOURCE_FILES volume/hdf5_io.cpp volume/hdf5_utils.cpp)
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
  list(APPEND SOURCE_FILES volume/bunny_io.cpp)
endif()

# CGAL support
if(NOT DISABLE_CGAL)
  find_package(CGAL CONFIG)
  if(CGAL_FOUND)
    add_compile_definitions(CVC_GEOMETRY_ENABLE_PROJECT)
    list(APPEND INCLUDE_FILES ../../inc/cvc/geometry/project_verts.h)
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
    geometry/cvc-mesher
    geometry/cvc-mesher/contour
    geometry/cvc-mesher/FastContouring
    #geometry/cvc-mesher/VolMagick
    geometry/cvc-mesher/LBIE
    geometry/cvc-mesher/Mesher
  )
  set(INCLUDE_FILES
      ${INCLUDE_FILES}
      geometry/cvc-mesher/contour/edgehash.h
      geometry/cvc-mesher/contour/respprop2.h
      geometry/cvc-mesher/contour/hash.h
      geometry/cvc-mesher/contour/conplot2d.h
      geometry/cvc-mesher/contour/cellsearch.h
      geometry/cvc-mesher/contour/data.h
      geometry/cvc-mesher/contour/seedchkr2.h
      geometry/cvc-mesher/contour/dataset.h
      geometry/cvc-mesher/contour/cellqueue.h
      geometry/cvc-mesher/contour/cubes.h
      geometry/cvc-mesher/contour/datasetreg3.h
      geometry/cvc-mesher/contour/dirseeds.h
      geometry/cvc-mesher/contour/seeddirreg3.h
      geometry/cvc-mesher/contour/basic.h
      geometry/cvc-mesher/contour/dataslc.h
      geometry/cvc-mesher/contour/contour.h
      geometry/cvc-mesher/contour/dict.h
      geometry/cvc-mesher/contour/contour2d.h
      geometry/cvc-mesher/contour/seedchkr3.h
      geometry/cvc-mesher/contour/iqueue.h
      geometry/cvc-mesher/contour/segtree.h
      geometry/cvc-mesher/contour/commdata.h
      geometry/cvc-mesher/contour/bin.h
      geometry/cvc-mesher/contour/datasetslc.h
      geometry/cvc-mesher/contour/inttree.h
      geometry/cvc-mesher/contour/datasetvol.h
      geometry/cvc-mesher/contour/ipqueue.h
      geometry/cvc-mesher/contour/seedall.h
      geometry/cvc-mesher/contour/contour3d.h
      geometry/cvc-mesher/contour/regprop.h
      geometry/cvc-mesher/contour/compute.h
      geometry/cvc-mesher/contour/datavol.h
      geometry/cvc-mesher/contour/vtkmarchingcubescases.h
      geometry/cvc-mesher/contour/shelf.h
      geometry/cvc-mesher/contour/range.h
      geometry/cvc-mesher/contour/rangesweep.h
      geometry/cvc-mesher/contour/endian_io.h
      geometry/cvc-mesher/contour/bucketsearch.h
      geometry/cvc-mesher/contour/seedcells.h
      geometry/cvc-mesher/contour/conplotreg2.h
      geometry/cvc-mesher/contour/datareg3.h
      geometry/cvc-mesher/contour/conplot3d.h
      geometry/cvc-mesher/contour/conplot.h
      geometry/cvc-mesher/contour/utilities.h
      geometry/cvc-mesher/contour/datasetreg2.h
      geometry/cvc-mesher/contour/dirseedsreg2.h
      geometry/cvc-mesher/contour/regprop2.h
      geometry/cvc-mesher/contour/queue.h
      geometry/cvc-mesher/contour/conplotreg3.h
      geometry/cvc-mesher/contour/datareg2.h
      geometry/cvc-mesher/contour/squeue.h
      geometry/cvc-mesher/contour/rangeprop.h
      geometry/cvc-mesher/FastContouring/ContourGeometry.h
      geometry/cvc-mesher/FastContouring/cubes.h
      geometry/cvc-mesher/FastContouring/Matrix.h
      geometry/cvc-mesher/FastContouring/FastContouring.h
      geometry/cvc-mesher/FastContouring/Vector.h
      geometry/cvc-mesher/FastContouring/MarchingCubesBuffers.h
      geometry/cvc-mesher/FastContouring/Ray.h
      geometry/cvc-mesher/FastContouring/Quaternion.h
      geometry/cvc-mesher/FastContouring/Tuple.h
      geometry/cvc-mesher/LBIE/cubes.h
      geometry/cvc-mesher/LBIE/pcio.h
      geometry/cvc-mesher/LBIE/LBIE_Mesher.h
      geometry/cvc-mesher/LBIE/normalspline.h
      geometry/cvc-mesher/LBIE/vtkMarchingCubesCases.h
      geometry/cvc-mesher/LBIE/LBIE_geoframe.h
      geometry/cvc-mesher/LBIE/geoframe_adapter.h
      geometry/cvc-mesher/LBIE/e_face.h
      geometry/cvc-mesher/LBIE/endians.h
      geometry/cvc-mesher/LBIE/octree.h
      geometry/cvc-mesher/LBIE/LBIEMesherRenderable.h
      geometry/cvc-mesher/Mesher/mesher.h
  )
  set(SOURCE_FILES
      ${SOURCE_FILES}
      geometry/cvc-mesher/contour/squeue.cpp
      geometry/cvc-mesher/contour/rangesweep.cpp
      geometry/cvc-mesher/contour/dirseeds.cpp
      geometry/cvc-mesher/contour/contour.cpp
      geometry/cvc-mesher/contour/seeddirreg3.cpp
      geometry/cvc-mesher/contour/contour3d.cpp
      geometry/cvc-mesher/contour/conplotreg3.cpp
      geometry/cvc-mesher/contour/dict.c
      geometry/cvc-mesher/contour/edgehash.cpp
      geometry/cvc-mesher/contour/regprop2.cpp
      geometry/cvc-mesher/contour/spectrumtest.cpp
      geometry/cvc-mesher/contour/ipqueue.cpp
      geometry/cvc-mesher/contour/seedcells.cpp
      geometry/cvc-mesher/contour/conplot3d.cpp
      geometry/cvc-mesher/contour/dataslc.cpp
      geometry/cvc-mesher/contour/cubes.cpp
      geometry/cvc-mesher/contour/conplot2d.cpp
      geometry/cvc-mesher/contour/shelf.cpp
      geometry/cvc-mesher/contour/iqueue.cpp
      geometry/cvc-mesher/contour/datavol.cpp
      geometry/cvc-mesher/contour/cellqueue.cpp
      geometry/cvc-mesher/contour/bin.cpp
      geometry/cvc-mesher/contour/datareg2.cpp
      geometry/cvc-mesher/contour/bucketsearch.cpp
      geometry/cvc-mesher/contour/seedall.cpp
      geometry/cvc-mesher/contour/range.cpp
      geometry/cvc-mesher/contour/hash.cpp
      geometry/cvc-mesher/contour/conplotreg2.cpp
      geometry/cvc-mesher/contour/data.cpp
      geometry/cvc-mesher/contour/inttree.cpp
      geometry/cvc-mesher/contour/seedchkr3.cpp
      geometry/cvc-mesher/contour/conplot.cpp
      geometry/cvc-mesher/contour/queue.cpp
      #geometry/cvc-mesher/contour/main.cpp
      geometry/cvc-mesher/contour/regprop.cpp
      geometry/cvc-mesher/contour/respprop2.cpp
      geometry/cvc-mesher/contour/rangeprop.cpp
      geometry/cvc-mesher/contour/segtree.cpp
      geometry/cvc-mesher/contour/conplot_p.cpp
      geometry/cvc-mesher/contour/contour2d.cpp
      geometry/cvc-mesher/contour/datareg3.cpp
      geometry/cvc-mesher/contour/dirseedsreg2.cpp
      geometry/cvc-mesher/contour/seedchkr2.cpp
      geometry/cvc-mesher/FastContouring/Ray.cpp
      geometry/cvc-mesher/FastContouring/FastContouring.cpp
      geometry/cvc-mesher/FastContouring/Quaternion.cpp
      geometry/cvc-mesher/FastContouring/Tuple.cpp
      geometry/cvc-mesher/FastContouring/Vector.cpp
      geometry/cvc-mesher/FastContouring/ContourGeometry.cpp
      geometry/cvc-mesher/FastContouring/Matrix.cpp
      geometry/cvc-mesher/FastContouring/MarchingCubesBuffers.cpp
      geometry/cvc-mesher/LBIE/normalspline.cpp
      geometry/cvc-mesher/LBIE/e_face.cpp
      geometry/cvc-mesher/LBIE/pcio.cpp
      geometry/cvc-mesher/LBIE/LBIE_geoframe.cpp
      geometry/cvc-mesher/LBIE/geoframe_adapter.cpp
      geometry/cvc-mesher/LBIE/tetra.cpp
      geometry/cvc-mesher/LBIE/hexa.cpp
      geometry/cvc-mesher/LBIE/LBIE_Mesher.cpp
      #geometry/cvc-mesher/LBIE/test.cpp
      geometry/cvc-mesher/LBIE/octree.cpp
      #geometry/cvc-mesher/Mesher/main.cpp
      geometry/cvc-mesher/Mesher/mesher.cpp
  )
endif(CVC_ENABLE_MESHER)

# Signed distance function support
if(CVC_ENABLE_SDF)
  list(APPEND CVC_INCLUDE_DIRS
    geometry/SDF/UsefulMath
    geometry/SDF/GeometryFileTypes
    geometry/SDF/ByteOrder
    geometry/SDF/SignDistanceFunction
    geometry/SDF/Geometry
    geometry/SDF/VolumeFileTypes
    geometry/SDF/SDF
    geometry/SDF/SignDistanceFunction_v2
  )
  set(INCLUDE_FILES
      ${INCLUDE_FILES}
      # geometry/SDF/UsefulMath/AreaVolumes.h
      # geometry/SDF/UsefulMath/Matrix.h
      # geometry/SDF/UsefulMath/TrilinearGrid.h
      # geometry/SDF/UsefulMath/LinearAlgebra.h
      # geometry/SDF/UsefulMath/Vector.h
      # geometry/SDF/UsefulMath/Ray.h
      # geometry/SDF/UsefulMath/Quaternion.h
      # geometry/SDF/UsefulMath/Tuple.h
      # geometry/SDF/GeometryFileTypes/RawnFile.h
      # geometry/SDF/GeometryFileTypes/RawncFile.h
      # geometry/SDF/GeometryFileTypes/MayaOBJFile.h
      # geometry/SDF/GeometryFileTypes/RawcFile.h
      # geometry/SDF/GeometryFileTypes/RawFile.h
      # geometry/SDF/GeometryFileTypes/GeometryFileType.h
      # geometry/SDF/GeometryFileTypes/GeometryLoader.h
      #geometry/SDF/ByteOrder/ByteSwapping.h
      geometry/SDF/SignDistanceFunction/common.h
      geometry/SDF/SignDistanceFunction/sdfLib.h
      geometry/SDF/SignDistanceFunction/head.h
      geometry/SDF/SignDistanceFunction/SDFContext.h
      #geometry/SDF/Geometry/IntQueue.h
      #geometry/SDF/Geometry/GeometrySceneArray.h
      #geometry/SDF/Geometry/Geometry.h
      #geometry/SDF/Geometry/SceneArrayNode.h
      #geometry/SDF/Geometry/MeshDerivatives.h
      #geometry/SDF/Geometry/GeometryScene.h
      #geometry/SDF/VolumeFileTypes/RawVFile.h
      #geometry/SDF/VolumeFileTypes/RawIVFile.h
      #geometry/SDF/VolumeFileTypes/MRCFile.h
      #geometry/SDF/VolumeFileTypes/SimpleVolumeData.h
      #geometry/SDF/VolumeFileTypes/DXFile.h
      #geometry/SDF/VolumeFileTypes/VolumeLoader.h
      #geometry/SDF/VolumeFileTypes/VolumeFileType.h
      # geometry/SDF/SignDistanceFunction_v2/Geom3DParser.h
      # geometry/SDF/SignDistanceFunction_v2/reg3data.h
      # geometry/SDF/SignDistanceFunction_v2/mtxlib.h
      # geometry/SDF/SignDistanceFunction_v2/bufferedio.h
      # geometry/SDF/SignDistanceFunction_v2/geom.h
      # geometry/SDF/SignDistanceFunction_v2/cubes.h
      # geometry/SDF/SignDistanceFunction_v2/diskio.h
      # geometry/SDF/SignDistanceFunction_v2/FaceVertSet3D.h
      # geometry/SDF/SignDistanceFunction_v2/dynarray.h
      # geometry/SDF/SignDistanceFunction_v2/Reg3Parser.h
      # geometry/SDF/SignDistanceFunction_v2/DistanceTransform.h
      # geometry/SDF/SignDistanceFunction_v2/bio.h
      # geometry/SDF/SignDistanceFunction_v2/RawivParser.h
  )
  set(SOURCE_FILES
      ${SOURCE_FILES}
      # geometry/SDF/UsefulMath/TrilinearGrid.cpp
      # geometry/SDF/UsefulMath/Ray.cpp
      # geometry/SDF/UsefulMath/Quaternion.cpp
      # geometry/SDF/UsefulMath/Tuple.cpp
      # geometry/SDF/UsefulMath/Vector.cpp
      # geometry/SDF/UsefulMath/Matrix.cpp
      # geometry/SDF/UsefulMath/AreaVolumes.cpp
      # geometry/SDF/UsefulMath/LinearAlgebra.cpp
      # geometry/SDF/GeometryFileTypes/GeometryFileType.cpp
      # geometry/SDF/GeometryFileTypes/RawnFile.cpp
      # geometry/SDF/GeometryFileTypes/MayaOBJFile.cpp
      # geometry/SDF/GeometryFileTypes/RawcFile.cpp
      # geometry/SDF/GeometryFileTypes/GeometryLoader.cpp
      # geometry/SDF/GeometryFileTypes/RawFile.cpp
      # geometry/SDF/GeometryFileTypes/RawncFile.cpp
      # geometry/SDF/ByteOrder/ByteSwapping.cpp
      # geometry/SDF/SignDistanceFunction/init.cpp  # OLD - uses global sdf, all functions moved to SDFContext
      geometry/SDF/SignDistanceFunction/compute.cpp
      geometry/SDF/SignDistanceFunction/propagate.cpp
      geometry/SDF/SignDistanceFunction/head.cpp
      geometry/SDF/SignDistanceFunction/new_adjust.cpp
      geometry/SDF/SignDistanceFunction/main.cpp
      geometry/SDF/SignDistanceFunction/common.cpp
      geometry/SDF/SignDistanceFunction/octree.cpp
      geometry/SDF/SignDistanceFunction/SDFContext.cpp
      #geometry/SDF/Geometry/GeometryScene.cpp
      #geometry/SDF/Geometry/GeometrySceneArray.cpp
      #geometry/SDF/Geometry/Geometry.cpp
      #geometry/SDF/Geometry/SceneArrayNode.cpp
      #geometry/SDF/Geometry/IntQueue.cpp
      #geometry/SDF/Geometry/MeshDerivatives.cpp
      #geometry/SDF/VolumeFileTypes/VolumeLoader.cpp
      #geometry/SDF/VolumeFileTypes/DXFile.cpp
      #geometry/SDF/VolumeFileTypes/RawIVFile.cpp
      #geometry/SDF/VolumeFileTypes/RawVFile.cpp
      #geometry/SDF/VolumeFileTypes/VolumeFileType.cpp
      #geometry/SDF/VolumeFileTypes/MRCFile.cpp
      #geometry/SDF/VolumeFileTypes/SimpleVolumeData.cpp
      #geometry/SDF/Server.cpp
      #geometry/SDF/geometry/SDF/main.cpp
      geometry/SDF/SignDistanceFunction_v2/Reg3Parser.cpp
      geometry/SDF/SignDistanceFunction_v2/RawivParser.cpp
      geometry/SDF/SignDistanceFunction_v2/Geom3DParser.cpp
      geometry/SDF/SignDistanceFunction_v2/geom.cpp
      geometry/SDF/SignDistanceFunction_v2/FaceVertSet3D.cpp
      geometry/SDF/SignDistanceFunction_v2/DistanceTransform.cpp
      geometry/SDF/SignDistanceFunction_v2/mtxlib.cpp
      geometry/SDF/SignDistanceFunction_v2/bufferedio.cpp
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
  list(APPEND SOURCE_FILES volume/voxels_kernels.cu nav/drive.cu nav/coef_train.cu)
  message(STATUS "CUDA enabled for libcvc - including voxels_kernels.cu, nav/drive.cu, nav/coef_train.cu")
  # Volume rendering opts into fast math per source (it has no bit/float-equiv
  # contract); the nav drive + trainer kernels MUST stay precise (float-equivalent
  # to the CPU/torch reference) — no fast-math, no FMA contraction, IEEE div/sqrt.
  set_source_files_properties(volume/voxels_kernels.cu PROPERTIES
    COMPILE_OPTIONS "$<$<CONFIG:Release>:--use_fast_math>")
  set_source_files_properties(nav/drive.cu nav/coef_train.cu PROPERTIES
    COMPILE_OPTIONS "-fmad=false;--prec-div=true;--prec-sqrt=true;--ftz=false")
endif()

# ImageMagick support (for volume_ops image I/O)
#
# Quantum-depth and HDRI flags must match the installed library's build:
# Q8 vs Q16, HDRI on/off. On Linux/macOS we ask pkg-config for the
# canonical cflags, which include the right `MAGICKCORE_QUANTUM_DEPTH`
# and `MAGICKCORE_HDRI_ENABLE` defines. On Windows the cvcpkg imagemagick
# bundle ships the official Q16-HDRI x64 build.
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
    set(CVC_IMAGEMAGICK_HDRI_ENABLE 1)  # official Q16-HDRI x64 build
  endif()
  find_package(ImageMagick COMPONENTS Magick++ MagickCore)
  # Windows fallback: the stock FindImageMagick module searches only
  # pkg-config hints and the ImageMagick registry key, both with
  # NO_DEFAULT_PATH, so it never looks in CMAKE_PREFIX_PATH. The cvcpkg
  # imagemagick bundle is innoextracted straight into the prefix (no
  # registry entry, no .pc), so the module misses it and image I/O gets
  # silently disabled on Windows. find_path/find_library DO honor
  # CMAKE_PREFIX_PATH, so search the prefix directly for the staged
  # headers and the canonical import-lib aliases the bundle provides.
  if(WIN32 AND NOT ImageMagick_FOUND)
    find_path(ImageMagick_INCLUDE_DIR NAMES Magick++.h
              PATH_SUFFIXES ImageMagick ImageMagick-7)
    find_library(ImageMagick_Magickxx_LIBRARY NAMES Magick++ CORE_RL_Magick++_)
    find_library(ImageMagick_MagickCore_LIBRARY NAMES MagickCore CORE_RL_MagickCore_)
    if(ImageMagick_INCLUDE_DIR AND ImageMagick_Magickxx_LIBRARY
       AND ImageMagick_MagickCore_LIBRARY)
      set(ImageMagick_FOUND TRUE)
      set(ImageMagick_INCLUDE_DIRS "${ImageMagick_INCLUDE_DIR}")
      set(ImageMagick_LIBRARIES
          "${ImageMagick_Magickxx_LIBRARY};${ImageMagick_MagickCore_LIBRARY}")
      message(STATUS "ImageMagick found via cvcpkg-prefix fallback: "
                     "${ImageMagick_INCLUDE_DIR}")
    endif()
  endif()
  if(ImageMagick_FOUND)
    message(STATUS "ImageMagick found: ${ImageMagick_INCLUDE_DIRS} "
                   "(Q${CVC_IMAGEMAGICK_QUANTUM_DEPTH}, "
                   "HDRI=${CVC_IMAGEMAGICK_HDRI_ENABLE})")
  else()
    message(STATUS "ImageMagick not found - disabling image I/O in volume_ops")
    set(CVC_ENABLE_IMAGEMAGICK OFF)
  endif()
endif()

# Assimp support (for native mesh/model import - cvc::model / read_geometry)
option(CVC_ENABLE_ASSIMP "Enable Assimp for native mesh/model import" ON)
if(CVC_ENABLE_ASSIMP)
  find_package(assimp CONFIG)
  if(assimp_FOUND)
    message(STATUS "Assimp found: ${assimp_VERSION}")
    # DEFENSIVE: the cvcpkg assimp bundle's exported target hardcodes an ABSOLUTE
    # build-sandbox zlib path in INTERFACE_LINK_LIBRARIES. The recipe builds
    # assimp with -DASSIMP_BUILD_ZLIB=OFF, and assimp's CMake bakes
    # ${ZLIB_LIBRARIES} (an absolute path from FindZLIB) into the export instead
    # of the relocatable ZLIB::ZLIB target — so the published bundle carries a
    # path like /tmp/cvcpkg-builder/.../lib/libz.so that exists on no consumer,
    # and linking assimp::assimp fails everywhere (locally and in CI). libcvc
    # links zlib itself, so rewrite that stale entry to the relocatable
    # ZLIB::ZLIB target (falling back to a bare -lz). Remove once the assimp
    # recipe exports zlib via an imported target (tracked libcvc-deps follow-up).
    find_package(ZLIB QUIET)
    get_target_property(_cvc_assimp_ill assimp::assimp INTERFACE_LINK_LIBRARIES)
    if(_cvc_assimp_ill AND "${_cvc_assimp_ill}" MATCHES "libz\\.|zlib\\.|cvcpkg-builder")
      # Match the stale zlib entry in BOTH forms: unix libz.so/.a AND windows
      # zlib.lib (the earlier "libz\\." pattern missed the windows filename, so the
      # absolute cvcpkg-builder path survived and cvc.dll failed to link).
      if(TARGET ZLIB::ZLIB)
        string(REGEX REPLACE "[^;]*(libz|zlib)\\.[^;]*" "ZLIB::ZLIB" _cvc_assimp_ill "${_cvc_assimp_ill}")
      else()
        string(REGEX REPLACE "[^;]*(libz|zlib)\\.[^;]*" "z" _cvc_assimp_ill "${_cvc_assimp_ill}")
      endif()
      set_target_properties(assimp::assimp PROPERTIES INTERFACE_LINK_LIBRARIES "${_cvc_assimp_ill}")
      message(STATUS "  (rewrote assimp's stale absolute zlib link to a relocatable target)")
    endif()
  else()
    message(STATUS "Assimp not found - disabling native mesh/model import")
    set(CVC_ENABLE_ASSIMP OFF)
  endif()
endif()

# FFTW support (for volume_ops filtered back-projection)
option(CVC_ENABLE_FFTW "Enable FFTW support for filtered back-projection in volume_ops" ON)
if(CVC_ENABLE_FFTW)
  find_package(PkgConfig QUIET)
  if(PkgConfig_FOUND)
    pkg_check_modules(FFTW3 QUIET IMPORTED_TARGET fftw3)
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
    core/state_transport_grpc.cpp
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
    ${CMAKE_CURRENT_SOURCE_DIR}/geometry
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
if(CVC_STATE_EXEC)
  target_compile_definitions(cvc PUBLIC CVC_STATE_EXEC)
endif()
if(CVC_STATE_MEMORY_MANAGER)
  target_compile_definitions(cvc PUBLIC CVC_STATE_MEMORY_MANAGER)
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
  # Wrap each external include dir in its own BUILD_INTERFACE genex so a
  # bundled toolchain (e.g. libcvc-deps living under a parent project's
  # build tree) does not leak a build-dir path into the installed cvc
  # target's INTERFACE_INCLUDE_DIRECTORIES (CMake rejects that with
  # "path is prefixed in the build directory").
  foreach(_im_inc IN LISTS ImageMagick_INCLUDE_DIRS)
    target_include_directories(cvc PUBLIC "$<BUILD_INTERFACE:${_im_inc}>")
  endforeach()
  # Link via imported targets when available (config-mode from libcvc-deps),
  # otherwise fall back to the raw library paths from FindImageMagick.
  if(TARGET ImageMagick::Magick++ AND TARGET ImageMagick::MagickCore)
    target_link_libraries(cvc PUBLIC ImageMagick::Magick++ ImageMagick::MagickCore)
  else()
    target_link_libraries(cvc PUBLIC ${ImageMagick_LIBRARIES})
  endif()
endif()

# Assimp linking. PUBLIC so tests inherit the CVC_ENABLE_ASSIMP define (as with
# ImageMagick). The assimp::assimp imported target carries its own include dirs.
if(CVC_ENABLE_ASSIMP)
  target_compile_definitions(cvc PUBLIC CVC_ENABLE_ASSIMP)
  target_link_libraries(cvc PUBLIC assimp::assimp)
endif()

# FFTW linking
if(CVC_ENABLE_FFTW)
  target_compile_definitions(cvc PUBLIC CVC_ENABLE_FFTW)
  # Same BUILD_INTERFACE wrapping rationale as the ImageMagick block above.
  if(TARGET PkgConfig::FFTW3)
    target_link_libraries(cvc PUBLIC PkgConfig::FFTW3)
  else()
    foreach(_fftw_inc IN LISTS FFTW3_INCLUDE_DIRS)
      target_include_directories(cvc PUBLIC "$<BUILD_INTERFACE:${_fftw_inc}>")
    endforeach()
    # The LIBRARIES need the same treatment the INCLUDE_DIRS just got, and for
    # the same reason: find_library() yields an ABSOLUTE path, and linking it
    # PUBLIC bakes that build-machine path into the exported cvcTargets.cmake
    # INTERFACE_LINK_LIBRARIES. The published windows libcvc bundle carries
    # "D:/libcvc-deps/lib/fftw3.lib" from exactly this line, so consuming it
    # anywhere without a D:\libcvc-deps fails at link with
    #   ninja: error: 'D:/libcvc-deps/lib/fftw3.lib', needed by ..., missing
    # cvc is shared here, so it already resolves fftw3 itself — consumers do
    # not need it on their link line.
    foreach(_fftw_lib IN LISTS FFTW3_LIBRARIES)
      target_link_libraries(cvc PUBLIC "$<BUILD_INTERFACE:${_fftw_lib}>")
    endforeach()
  endif()
endif()

# zstd compression (production compressor for distributed state).
find_package(PkgConfig QUIET)
if(PkgConfig_FOUND)
  pkg_check_modules(ZSTD QUIET IMPORTED_TARGET libzstd)
endif()
if(NOT ZSTD_FOUND)
  find_library(ZSTD_LIBRARIES NAMES zstd)
  find_path(ZSTD_INCLUDE_DIRS NAMES zstd.h)
  if(ZSTD_LIBRARIES AND ZSTD_INCLUDE_DIRS)
    set(ZSTD_FOUND TRUE)
  endif()
endif()
if(ZSTD_FOUND)
  if(TARGET PkgConfig::ZSTD)
    target_link_libraries(cvc PRIVATE PkgConfig::ZSTD)
  else()
    target_include_directories(cvc PRIVATE ${ZSTD_INCLUDE_DIRS})
    target_link_libraries(cvc PRIVATE ${ZSTD_LIBRARIES})
  endif()
else()
  message(WARNING "libzstd not found; zstd compression codec will not link")
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

# The vendored XmlRpc++ headers must always ship with the SDK:
# cvc/utility/utility.h unconditionally does #include <xmlrpc/XmlRpc.h>,
# so external find_package(cvc) consumers need these headers on the
# include path even when the xmlrpc network library itself was not
# built (CVC_USING_XMLRPC=OFF). When CVC_USING_XMLRPC=ON the xmlrpc
# subproject installs the same files; the duplicate rule is idempotent.
install(DIRECTORY ${PROJECT_SOURCE_DIR}/inc/xmlrpc
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  COMPONENT libcvc
  FILES_MATCHING PATTERN "*.h"
)

install(FILES ${PROJECT_BINARY_DIR}/inc/cvc/core/config.h
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/cvc/core
  COMPONENT libcvc
)

# Generate config.h
# cvc::nav policy data dir, baked into config.h so a default-installed build finds
# $PREFIX/share/cvc/nav/coef_mlp.cvcnav (a relocated install falls back to a path
# relative to the loaded library — see coef_mlp::default_weights_path).
if(IS_ABSOLUTE "${CMAKE_INSTALL_DATADIR}")
  set(CVC_NAV_DATADIR "${CMAKE_INSTALL_DATADIR}/cvc/nav")
else()
  set(CVC_NAV_DATADIR "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_DATADIR}/cvc/nav")
endif()

# Use PROJECT_BINARY_DIR (not CMAKE_BINARY_DIR) so the generated header lands
# under libcvc's own build tree when consumed via FetchContent/add_subdirectory.
# Downstream projects can then add ${libcvc_BINARY_DIR}/inc to their include path.
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/../../inc/cvc/core/config.h.cmake
  ${PROJECT_BINARY_DIR}/inc/cvc/core/config.h
)

# Install a navigation policy to the canonical location when one is supplied
# (trained + exported from GRL-SNAM via coef_export.py). Optional: an app can
# always pass an explicit path to coef_mlp::load / sim_world.
# Default to the reference policy shipped in the tree (exported from GRL-SNAM's
# CoefMLP; retrain + re-export via grl_snam/tools/coef_train.py). A build can
# override -DCVC_NAV_WEIGHTS_FILE=<path> to install its own trained policy.
if(NOT CVC_NAV_WEIGHTS_FILE AND EXISTS "${PROJECT_SOURCE_DIR}/share/cvc/nav/coef_mlp.cvcnav")
  set(CVC_NAV_WEIGHTS_FILE "${PROJECT_SOURCE_DIR}/share/cvc/nav/coef_mlp.cvcnav")
endif()
if(CVC_NAV_WEIGHTS_FILE)
  install(FILES "${CVC_NAV_WEIGHTS_FILE}"
    DESTINATION ${CMAKE_INSTALL_DATADIR}/cvc/nav RENAME coef_mlp.cvcnav COMPONENT libcvc)
  message(STATUS "cvc::nav: installing policy ${CVC_NAV_WEIGHTS_FILE} -> share/cvc/nav/coef_mlp.cvcnav")
endif()

# The pure-C++ nav swarm demo (the template a cvcGL scene copies to add agents).
option(CVC_BUILD_NAV_EXAMPLE "Build the pure-C++ nav swarm + trainer demos" OFF)
if(CVC_BUILD_NAV_EXAMPLE)
  add_executable(nav_swarm_demo ${PROJECT_SOURCE_DIR}/examples/nav_swarm_demo.cpp)
  target_link_libraries(nav_swarm_demo PRIVATE cvc)
  add_executable(nav_train_demo ${PROJECT_SOURCE_DIR}/examples/nav_train_demo.cpp)
  target_link_libraries(nav_train_demo PRIVATE cvc)
  if(CVC_ENABLE_CUDA)
    target_compile_definitions(nav_train_demo PRIVATE CVC_ENABLE_CUDA)
  endif()
endif()

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
