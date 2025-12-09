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
  ../../inc/cvc/bunny.h
  ../../inc/cvc/endians.h
  ../../inc/cvc/utility.h
  ../../inc/cvc/state_object.h
  ../../inc/cvc/app.h
  ../../inc/cvc/volmagick.h
  ../../inc/cvc/geometry_file_io.h
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
  utility.cpp
  voxels.cpp
  volume.cpp
  volume_file_io.cpp
  volume_file_info.cpp
  vtk_io.cpp
)

# Configuration
set(CVC_NAMESPACE cvc CACHE STRING "C++ namespace for CVC library")
mark_as_advanced(CVC_NAMESPACE)

# The libcvc version number
set(CVC_VERSION_MAJOR 2)
set(CVC_VERSION_MINOR 0)
set(CVC_VERSION ${CVC_VERSION_MAJOR}.${CVC_VERSION_MINOR})
message(STATUS "libcvc version: ${CVC_VERSION}")

set(CVC_LINK_LIBS "")

# Build options
option(CVC_USING_XMLRPC "Enable cvc::state network sharing support" OFF)
option(CVC_USING_HDF5 "Enable HDF5 support for *.cvc file format" OFF)
option(CVC_USING_IMOD_MRC "Use IMOD's MRC loading routines" ON)
option(CVC_LOG4CPLUS_DEFAULT "Use log4cplus output by default" OFF)
option(LOG4CPLUS_DISABLE_TRACE "Disable logging trace" OFF)
option(CVC_APP_XML_PROPERTY_TREE "Use XML for property map (otherwise Boost INFO format)" OFF)
option(CVC_GEOMETRY_ENABLE_BUNNY "Enable built-in bunny geometry test tri mesh" ON)
option(CVC_GEOMETRY_CORRECT_INDEX_START "Adjust indices if they start from 1 rather than 0" OFF)
option(CVC_ENABLE_MESHER "Enable isosurfacing/meshing capability" ON)
option(CVC_ENABLE_SDF "Enable signed distance function calculation capability" ON)

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
  find_package(HDF5 COMPONENTS C CXX)
  if(HDF5_FOUND)
    message(STATUS "HDF5 found - Enabling *.cvc file support")
    
    add_compile_definitions(CVC_USING_HDF5 ${HDF5_DEFINITIONS})
    list(APPEND INCLUDE_FILES ../../inc/cvc/hdf5_utils.h)
    list(APPEND SOURCE_FILES
      hdf5_io.cpp
      hdf5_utils.cpp
    )
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
if(CVC_USING_IMOD_MRC)
  add_compile_definitions(NOTIFFLIBS CVC_USING_IMOD_MRC)
  list(APPEND CVC_INCLUDE_DIRS libiimod)
  list(APPEND INCLUDE_FILES
    libiimod/ilist.h
    libiimod/imodconfig.h
    libiimod/notiff.h
    libiimod/iimage.h
    libiimod/mrcfiles.h
    libiimod/hvemtypes.h
    libiimod/sliceproc.h
    libiimod/mrcc.h
    libiimod/b3dutil.h
    libiimod/mrcslice.h
    libiimod/cfsemshare.h
    libiimod/notiffio.h
  )
  list(APPEND SOURCE_FILES
    libiimod/b3dutil.c
    libiimod/diffusion.c
    libiimod/iilikemrc.c
    libiimod/iimage.c
    libiimod/iimrc.c
    libiimod/iitif.c
    libiimod/ilist.c
    libiimod/mrcfiles.c
    libiimod/mrcsec.c
    libiimod/mrcslice.c
    libiimod/plist.c
    libiimod/tiffstub.c
    libiimod/islice.c
    libiimod/parallelwrite.c
  )
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
  find_package(CGAL)
  if(CGAL_FOUND)
    include(${CGAL_USE_FILE})
    add_compile_definitions(CVC_GEOMETRY_ENABLE_PROJECT)
    list(APPEND INCLUDE_FILES ../../inc/cvc/project_verts.h)
    # CGAL may have special compiler requirements
    string(APPEND CMAKE_CXX_FLAGS " ${CGAL_CXX_FLAGS_INIT}")
    list(APPEND CVC_LINK_LIBS ${CGAL_LIBRARIES} ${GMP_LIBRARIES} ${GMPXX_LIBRARIES})
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
    cvc-mesher/contour
    cvc-mesher/FastContouring
    cvc-mesher/VolMagick
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
      cvc-mesher/VolMagick/Exceptions.h
      cvc-mesher/VolMagick/Dimension.h
      cvc-mesher/VolMagick/BoundingBox.h
      cvc-mesher/VolMagick/endians.h
      cvc-mesher/VolMagick/VolMagick.h
      cvc-mesher/VolMagick/VolumeCache.h
      cvc-mesher/LBIE/cubes.h
      cvc-mesher/LBIE/pcio.h
      cvc-mesher/LBIE/LBIE_Mesher.h
      cvc-mesher/LBIE/normalspline.h
      cvc-mesher/LBIE/vtkMarchingCubesCases.h
      cvc-mesher/LBIE/LBIE_geoframe.h
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
      cvc-mesher/VolMagick/RawV.cpp
      cvc-mesher/VolMagick/MRC.cpp
      cvc-mesher/VolMagick/VolumeCache.cpp
      cvc-mesher/VolMagick/ContrastEnhancement.cpp
      cvc-mesher/VolMagick/AnisotropicDiffusion.cpp
      cvc-mesher/VolMagick/BilateralFilter.cpp
      cvc-mesher/VolMagick/INR.cpp
      cvc-mesher/VolMagick/RawIV.cpp
      cvc-mesher/VolMagick/VolMagick.cpp
      cvc-mesher/VolMagick/Spider.cpp
      #cvc-mesher/VolMagick/main.cpp
      cvc-mesher/LBIE/normalspline.cpp
      cvc-mesher/LBIE/e_face.cpp
      cvc-mesher/LBIE/pcio.cpp
      cvc-mesher/LBIE/LBIE_geoframe.cpp
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
      SDF/UsefulMath/AreaVolumes.h
      SDF/UsefulMath/Matrix.h
      SDF/UsefulMath/TrilinearGrid.h
      SDF/UsefulMath/LinearAlgebra.h
      SDF/UsefulMath/Vector.h
      SDF/UsefulMath/Ray.h
      SDF/UsefulMath/Quaternion.h
      SDF/UsefulMath/Tuple.h
      SDF/GeometryFileTypes/RawnFile.h
      SDF/GeometryFileTypes/RawncFile.h
      SDF/GeometryFileTypes/MayaOBJFile.h
      SDF/GeometryFileTypes/RawcFile.h
      SDF/GeometryFileTypes/RawFile.h
      SDF/GeometryFileTypes/GeometryFileType.h
      SDF/GeometryFileTypes/GeometryLoader.h
      SDF/ByteOrder/ByteSwapping.h
      SDF/SignDistanceFunction/common.h
      SDF/SignDistanceFunction/sdfLib.h
      SDF/SignDistanceFunction/head.h
      SDF/Geometry/IntQueue.h
      SDF/Geometry/GeometrySceneArray.h
      SDF/Geometry/Geometry.h
      SDF/Geometry/SceneArrayNode.h
      SDF/Geometry/MeshDerivatives.h
      SDF/Geometry/GeometryScene.h
      SDF/VolumeFileTypes/RawVFile.h
      SDF/VolumeFileTypes/RawIVFile.h
      SDF/VolumeFileTypes/MRCFile.h
      SDF/VolumeFileTypes/SimpleVolumeData.h
      SDF/VolumeFileTypes/DXFile.h
      SDF/VolumeFileTypes/VolumeLoader.h
      SDF/VolumeFileTypes/VolumeFileType.h
      SDF/SignDistanceFunction_v2/Geom3DParser.h
      SDF/SignDistanceFunction_v2/reg3data.h
      SDF/SignDistanceFunction_v2/mtxlib.h
      SDF/SignDistanceFunction_v2/bufferedio.h
      SDF/SignDistanceFunction_v2/geom.h
      SDF/SignDistanceFunction_v2/cubes.h
      SDF/SignDistanceFunction_v2/diskio.h
      SDF/SignDistanceFunction_v2/FaceVertSet3D.h
      SDF/SignDistanceFunction_v2/dynarray.h
      SDF/SignDistanceFunction_v2/Reg3Parser.h
      SDF/SignDistanceFunction_v2/DistanceTransform.h
      SDF/SignDistanceFunction_v2/bio.h
      SDF/SignDistanceFunction_v2/RawivParser.h
  )
  set(SOURCE_FILES
      ${SOURCE_FILES}
      SDF/UsefulMath/TrilinearGrid.cpp
      SDF/UsefulMath/Ray.cpp
      SDF/UsefulMath/Quaternion.cpp
      SDF/UsefulMath/Tuple.cpp
      SDF/UsefulMath/Vector.cpp
      SDF/UsefulMath/Matrix.cpp
      SDF/UsefulMath/AreaVolumes.cpp
      SDF/UsefulMath/LinearAlgebra.cpp
      SDF/GeometryFileTypes/GeometryFileType.cpp
      SDF/GeometryFileTypes/RawnFile.cpp
      SDF/GeometryFileTypes/MayaOBJFile.cpp
      SDF/GeometryFileTypes/RawcFile.cpp
      SDF/GeometryFileTypes/GeometryLoader.cpp
      SDF/GeometryFileTypes/RawFile.cpp
      SDF/GeometryFileTypes/RawncFile.cpp
      SDF/ByteOrder/ByteSwapping.cpp
      SDF/SignDistanceFunction/init.cpp
      SDF/SignDistanceFunction/compute.cpp
      SDF/SignDistanceFunction/propagate.cpp
      SDF/SignDistanceFunction/head.cpp
      SDF/SignDistanceFunction/new_adjust.cpp
      SDF/SignDistanceFunction/main.cpp
      SDF/SignDistanceFunction/common.cpp
      SDF/SignDistanceFunction/octree.cpp
      SDF/Geometry/GeometryScene.cpp
      SDF/Geometry/GeometrySceneArray.cpp
      SDF/Geometry/Geometry.cpp
      SDF/Geometry/SceneArrayNode.cpp
      SDF/Geometry/IntQueue.cpp
      SDF/Geometry/MeshDerivatives.cpp
      SDF/VolumeFileTypes/VolumeLoader.cpp
      SDF/VolumeFileTypes/DXFile.cpp
      SDF/VolumeFileTypes/RawIVFile.cpp
      SDF/VolumeFileTypes/RawVFile.cpp
      SDF/VolumeFileTypes/VolumeFileType.cpp
      SDF/VolumeFileTypes/MRCFile.cpp
      SDF/VolumeFileTypes/SimpleVolumeData.cpp
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
  # Example: Add .cu files here when they exist
  # file(GLOB CUDA_SOURCE_FILES *.cu)
  # list(APPEND SOURCE_FILES ${CUDA_SOURCE_FILES})
  message(STATUS "CUDA enabled for libcvc - add .cu files as needed")
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
target_include_directories(cvc
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/inc>
    $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/inc>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CVC_INCLUDE_DIRS}
)

# Compile definitions
target_compile_definitions(cvc
  PRIVATE
    CVC_NAMESPACE=${CVC_NAMESPACE}
    CVC_VERSION_STRING="${CVC_VERSION}"
)

# Setup dependencies
SetupBoost(cvc)

if(NOT DISABLE_CGAL)
  SetupCGAL(cvc)
endif()

if(CVC_ENABLE_CUDA)
  SetupCUDA(cvc)
endif()

# Link libraries
target_link_libraries(cvc PUBLIC ${CVC_LINK_LIBS})

# Create executable
add_executable(trans-cvc trans-cvc.cpp)
target_link_libraries(trans-cvc PRIVATE cvc::cvc)

# Generate config.h
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/../../inc/cvc/config.h.cmake
  ${CMAKE_BINARY_DIR}/inc/cvc/config.h
)

# Windows-specific batch file
if(WIN32)
  file(TO_NATIVE_PATH "${Boost_LIBRARY_DIRS}" BOOST_LOCATION)
  file(TO_NATIVE_PATH "${CMAKE_BINARY_DIR}" BINARIES_LOCATION)
  configure_file(
    trans-cvc.bat.cmake
    ${CMAKE_BINARY_DIR}/trans-cvc.bat
  )
endif()

# Add unit tests if enabled
if(CVC_BUILD_TESTS)
  add_subdirectory(tests)
endif()
