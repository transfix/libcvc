cmake_minimum_required(VERSION 3.16)

#project(volrover3 VERSION 3.0.0)

# Set C++ standard
#set(CMAKE_CXX_STANDARD 17)
#set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Enable automoc, autouic, autorcc for Qt
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# Find required packages
# Try Qt6 first, fall back to Qt5 if VTK requires it
find_package(Qt6 QUIET COMPONENTS Core Widgets OpenGL OpenGLWidgets Test)
if(NOT Qt6_FOUND)
  find_package(Qt5 REQUIRED COMPONENTS Core Widgets OpenGL Test)
  set(QT_VERSION_MAJOR 5)
else()
  set(QT_VERSION_MAJOR 6)
endif()

find_package(VTK REQUIRED)
find_package(OpenGL REQUIRED)

# Include VTK use file
include(${VTK_USE_FILE})

# Include directories
include_directories(
  ${CMAKE_SOURCE_DIR}/inc
  ${VTK_INCLUDE_DIRS}
)

# Source files
set(VOLROVER3_SOURCES
  main.cpp
  MainWindow.cpp
  VTKRenderWidget.cpp
  SceneGraph.cpp
  SceneNode.cpp
  GeometryNode.cpp
  GraphicsNode.cpp
  VolumeNode.cpp
  NullGraphicNode.cpp
  GridNode.cpp
  AxisNode.cpp
  BBoxNode.cpp
  CameraController.cpp
  TransferFunctionWidget.cpp
  ThreadMonitorWidget.cpp
  StateTreeWidget.cpp
  AppState.cpp
  BoundingBoxDialog.cpp
  CameraSettingsDialog.cpp
  GridOptionsDialog.cpp
  SDFDialog.cpp
  IsosurfaceDialog.cpp
  GeometryDialog.cpp
  VolumeDialog.cpp
  GraphicsParentDialog.cpp
)

# Header files
set(VOLROVER3_HEADERS
  ${CMAKE_SOURCE_DIR}/inc/volrover3/MainWindow.h
  ${CMAKE_SOURCE_DIR}/inc/volrover3/VTKRenderWidget.h
  ${CMAKE_SOURCE_DIR}/inc/volrover3/SceneGraph.h
  ${CMAKE_SOURCE_DIR}/inc/volrover3/SceneNode.h
  ${CMAKE_SOURCE_DIR}/inc/volrover3/GeometryNode.h
  ${CMAKE_SOURCE_DIR}/inc/volrover3/GraphicsNode.h
  ${CMAKE_SOURCE_DIR}/inc/volrover3/VolumeNode.h
  ${CMAKE_SOURCE_DIR}/inc/volrover3/GridNode.h
  ${CMAKE_SOURCE_DIR}/inc/volrover3/AxisNode.h
  ${CMAKE_SOURCE_DIR}/inc/volrover3/BBoxNode.h
  ${CMAKE_SOURCE_DIR}/inc/volrover3/CameraController.h
  ${CMAKE_SOURCE_DIR}/inc/volrover3/TransferFunctionWidget.h
  ${CMAKE_SOURCE_DIR}/inc/volrover3/ThreadMonitorWidget.h
  ${CMAKE_SOURCE_DIR}/inc/volrover3/StateTreeWidget.h
  ${CMAKE_SOURCE_DIR}/inc/volrover3/AppState.h
  ${CMAKE_SOURCE_DIR}/inc/volrover3/BoundingBoxDialog.h
  ${CMAKE_SOURCE_DIR}/inc/volrover3/CameraSettingsDialog.h
  ${CMAKE_SOURCE_DIR}/inc/volrover3/GridOptionsDialog.h
  ${CMAKE_SOURCE_DIR}/inc/volrover3/SDFDialog.h
  ${CMAKE_SOURCE_DIR}/inc/volrover3/IsosurfaceDialog.h
  ${CMAKE_SOURCE_DIR}/inc/volrover3/GeometryDialog.h
  ${CMAKE_SOURCE_DIR}/inc/volrover3/VolumeDialog.h
  ${CMAKE_SOURCE_DIR}/inc/volrover3/GraphicsParentDialog.h
)

# Create executable
add_executable(volrover3
  ${VOLROVER3_SOURCES}
  ${VOLROVER3_HEADERS}
)

# Link libraries
if(QT_VERSION_MAJOR EQUAL 6)
  target_link_libraries(volrover3
    cvc
    Qt6::Core
    Qt6::Widgets
    Qt6::OpenGL
    Qt6::OpenGLWidgets
    ${VTK_LIBRARIES}
    ${OPENGL_LIBRARIES}
  )
else()
  target_link_libraries(volrover3
    cvc
    Qt5::Core
    Qt5::Widgets
    Qt5::OpenGL
    ${VTK_LIBRARIES}
    ${OPENGL_LIBRARIES}
  )
endif()

# Auto-initialize VTK modules
vtk_module_autoinit(
  TARGETS volrover3
  MODULES ${VTK_LIBRARIES}
)

# Install target
install(TARGETS volrover3
  RUNTIME DESTINATION bin
)

# Tests
if(CVC_BUILD_TESTS)  
  # Create a library with all volrover3 components except main.cpp for testing
  set(VOLROVER3_LIB_SOURCES ${VOLROVER3_SOURCES})
  list(REMOVE_ITEM VOLROVER3_LIB_SOURCES main.cpp)
  
  add_library(volrover3_lib STATIC ${VOLROVER3_LIB_SOURCES} ${VOLROVER3_HEADERS})
  
  if(QT_VERSION_MAJOR EQUAL 6)
    target_link_libraries(volrover3_lib
      cvc
      Qt6::Core
      Qt6::Widgets
      Qt6::OpenGL
      Qt6::OpenGLWidgets
      ${VTK_LIBRARIES}
      ${OPENGL_LIBRARIES}
    )
  else()
    target_link_libraries(volrover3_lib
      cvc
      Qt5::Core
      Qt5::Widgets
      Qt5::OpenGL
      ${VTK_LIBRARIES}
      ${OPENGL_LIBRARIES}
    )
  endif()
  
  vtk_module_autoinit(
    TARGETS volrover3_lib
    MODULES ${VTK_LIBRARIES}
  )
  
  # Find GTest
  find_package(GTest REQUIRED)
  include_directories(${GTEST_INCLUDE_DIRS})
  
  # AppState tests
  add_executable(volrover3_appstate_test
    tests/AppStateTest.cpp)
  target_link_libraries(volrover3_appstate_test
    volrover3_lib
    GTest::gtest
    GTest::gtest_main)
  add_test(NAME AppStateTest COMMAND volrover3_appstate_test)
  
  # SceneGraph tests
  add_executable(volrover3_scenegraph_test
    tests/SceneGraphTest.cpp)
  target_link_libraries(volrover3_scenegraph_test
    volrover3_lib
    GTest::gtest
    GTest::gtest_main)
  add_test(NAME SceneGraphTest COMMAND volrover3_scenegraph_test)
  
  # TransferFunction tests
  add_executable(volrover3_transferfunction_test
    tests/TransferFunctionTest.cpp)
  target_link_libraries(volrover3_transferfunction_test
    volrover3_lib
    GTest::gtest
    GTest::gtest_main)
  add_test(NAME TransferFunctionTest COMMAND volrover3_transferfunction_test)
  
  # CameraController tests
  add_executable(volrover3_camera_test
    tests/CameraControllerTest.cpp)
  target_link_libraries(volrover3_camera_test
    volrover3_lib
    GTest::gtest
    GTest::gtest_main)
  add_test(NAME CameraControllerTest COMMAND volrover3_camera_test)
  
  # GridNode tests
  add_executable(volrover3_gridnode_test
    tests/GridNodeTest.cpp)
  target_link_libraries(volrover3_gridnode_test
    volrover3_lib
    GTest::gtest
    GTest::gtest_main)
  add_test(NAME GridNodeTest COMMAND volrover3_gridnode_test)
  
  # StateTreeWidget tests
  add_executable(volrover3_statetree_test
    tests/StateTreeWidgetTest.cpp)
  target_link_libraries(volrover3_statetree_test
    volrover3_lib
    GTest::gtest)
  add_test(NAME StateTreeWidgetTest COMMAND volrover3_statetree_test)
  
  # GraphicsNode tests
  add_executable(volrover3_graphicsnode_test
    tests/GraphicsNodeTest.cpp)
  target_link_libraries(volrover3_graphicsnode_test
    volrover3_lib
    GTest::gtest
    GTest::gtest_main)
  add_test(NAME GraphicsNodeTest COMMAND volrover3_graphicsnode_test)
  
  # VolumeNode tests
  add_executable(volrover3_volumenode_test
    tests/VolumeNodeTest.cpp)
  target_link_libraries(volrover3_volumenode_test
    volrover3_lib
    GTest::gtest
    GTest::gtest_main)
  add_test(NAME VolumeNodeTest COMMAND volrover3_volumenode_test)
  
  # BBoxNode tests
  add_executable(volrover3_bboxnode_test
    tests/BBoxNodeTest.cpp)
  target_link_libraries(volrover3_bboxnode_test
    volrover3_lib
    GTest::gtest
    GTest::gtest_main)
  add_test(NAME BBoxNodeTest COMMAND volrover3_bboxnode_test)
  
  # NullGraphicNode tests
  add_executable(volrover3_nullgraphicnode_test
    tests/NullGraphicNodeTest.cpp)
  target_link_libraries(volrover3_nullgraphicnode_test
    volrover3_lib
    GTest::gtest
    GTest::gtest_main)
  add_test(NAME NullGraphicNodeTest COMMAND volrover3_nullgraphicnode_test)
  
  # BoundingBox Semantics tests
  add_executable(volrover3_bbox_semantics_test
    tests/BoundingBoxSemanticsTest.cpp)
  target_link_libraries(volrover3_bbox_semantics_test
    volrover3_lib
    GTest::gtest
    GTest::gtest_main)
  add_test(NAME BoundingBoxSemanticsTest COMMAND volrover3_bbox_semantics_test)
  
  # GeometryDialog tests
  add_executable(volrover3_geometrydialog_test
    tests/GeometryDialogTest.cpp)
  target_link_libraries(volrover3_geometrydialog_test
    volrover3_lib
    Qt${QT_VERSION_MAJOR}::Test
    GTest::gtest
    GTest::gtest_main)
  add_test(NAME GeometryDialogTest COMMAND volrover3_geometrydialog_test)
  
  # VolumeDialog tests
  add_executable(volrover3_volumedialog_test
    tests/VolumeDialogTest.cpp)
  target_link_libraries(volrover3_volumedialog_test
    volrover3_lib
    Qt${QT_VERSION_MAJOR}::Test
    GTest::gtest
    GTest::gtest_main)
  add_test(NAME VolumeDialogTest COMMAND volrover3_volumedialog_test)
endif()

