cmake_minimum_required(VERSION 3.16)

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

# Source files
set(VOLROVER3_SOURCES
  main.cpp
  volrover3_app.cpp
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
  ViewerOptionsDialog.cpp
  SDFDialog.cpp
  IsosurfaceDialog.cpp
  GeometryDialog.cpp
  VolumeDialog.cpp
  GraphicsParentDialog.cpp
  ProceduralGeometryDialog.cpp
)

# Header files
set(VOLROVER3_HEADERS
  ${PROJECT_SOURCE_DIR}/inc/volrover3/MainWindow.h
  ${PROJECT_SOURCE_DIR}/inc/volrover3/VTKRenderWidget.h
  ${PROJECT_SOURCE_DIR}/inc/volrover3/SceneGraph.h
  ${PROJECT_SOURCE_DIR}/inc/volrover3/SceneNode.h
  ${PROJECT_SOURCE_DIR}/inc/volrover3/GeometryNode.h
  ${PROJECT_SOURCE_DIR}/inc/volrover3/GraphicsNode.h
  ${PROJECT_SOURCE_DIR}/inc/volrover3/VolumeNode.h
  ${PROJECT_SOURCE_DIR}/inc/volrover3/GridNode.h
  ${PROJECT_SOURCE_DIR}/inc/volrover3/AxisNode.h
  ${PROJECT_SOURCE_DIR}/inc/volrover3/BBoxNode.h
  ${PROJECT_SOURCE_DIR}/inc/volrover3/CameraController.h
  ${PROJECT_SOURCE_DIR}/inc/volrover3/TransferFunctionWidget.h
  ${PROJECT_SOURCE_DIR}/inc/volrover3/ThreadMonitorWidget.h
  ${PROJECT_SOURCE_DIR}/inc/volrover3/StateTreeWidget.h
  ${PROJECT_SOURCE_DIR}/inc/volrover3/AppState.h
  ${PROJECT_SOURCE_DIR}/inc/volrover3/BoundingBoxDialog.h
  ${PROJECT_SOURCE_DIR}/inc/volrover3/CameraSettingsDialog.h
  ${PROJECT_SOURCE_DIR}/inc/volrover3/GridOptionsDialog.h
  ${PROJECT_SOURCE_DIR}/inc/volrover3/ViewerOptionsDialog.h
  ${PROJECT_SOURCE_DIR}/inc/volrover3/SDFDialog.h
  ${PROJECT_SOURCE_DIR}/inc/volrover3/IsosurfaceDialog.h
  ${PROJECT_SOURCE_DIR}/inc/volrover3/GeometryDialog.h
  ${PROJECT_SOURCE_DIR}/inc/volrover3/VolumeDialog.h
  ${PROJECT_SOURCE_DIR}/inc/volrover3/GraphicsParentDialog.h
  ${PROJECT_SOURCE_DIR}/inc/volrover3/ProceduralGeometryDialog.h
)

# Platform-specific bundle / icon resources
# ─────────────────────────────────────────
# Linux: a simple PNG icon installed alongside a .desktop file.
# macOS: the .icns is copied into the .app bundle's Resources/ directory.
# Windows: the .ico is compiled into the EXE via a generated .rc file.
set(_VOLROVER3_ICON_DIR "${PROJECT_SOURCE_DIR}/share/icons")
set(_VOLROVER3_ICON_PNG  "${_VOLROVER3_ICON_DIR}/volrover_logo.png")
set(_VOLROVER3_ICON_ICNS "${_VOLROVER3_ICON_DIR}/volrover_logo.icns")
set(_VOLROVER3_ICON_ICO  "${_VOLROVER3_ICON_DIR}/volrover_logo.ico")

set(_VOLROVER3_PLATFORM_SOURCES "")

if(WIN32 AND EXISTS "${_VOLROVER3_ICON_ICO}")
  # Generate a Windows resource script that embeds the icon and basic
  # version info into the EXE so Explorer + Task-Manager show the logo.
  set(_VOLROVER3_RC "${CMAKE_CURRENT_BINARY_DIR}/volrover3.rc")
  file(WRITE "${_VOLROVER3_RC}"
"#include <windows.h>\n"
"IDI_ICON1 ICON DISCARDABLE \"${_VOLROVER3_ICON_ICO}\"\n"
)
  list(APPEND _VOLROVER3_PLATFORM_SOURCES "${_VOLROVER3_RC}")
endif()

if(APPLE AND EXISTS "${_VOLROVER3_ICON_ICNS}")
  # Mark the .icns as a bundle resource so CMake copies it into
  # Contents/Resources/ during install.
  set_source_files_properties("${_VOLROVER3_ICON_ICNS}" PROPERTIES
    MACOSX_PACKAGE_LOCATION "Resources")
  list(APPEND _VOLROVER3_PLATFORM_SOURCES "${_VOLROVER3_ICON_ICNS}")
endif()

# Create executable
add_executable(volrover3
  ${VOLROVER3_SOURCES}
  ${VOLROVER3_HEADERS}
  ${_VOLROVER3_PLATFORM_SOURCES}
)

# Apple bundle / Windows GUI subsystem properties
if(APPLE)
  set_target_properties(volrover3 PROPERTIES
    MACOSX_BUNDLE TRUE
    MACOSX_BUNDLE_BUNDLE_NAME           "VolumeRover3"
    MACOSX_BUNDLE_BUNDLE_VERSION        "${PROJECT_VERSION}"
    MACOSX_BUNDLE_SHORT_VERSION_STRING  "${PROJECT_VERSION}"
    MACOSX_BUNDLE_LONG_VERSION_STRING   "${PROJECT_VERSION}"
    MACOSX_BUNDLE_GUI_IDENTIFIER        "edu.utexas.cvc.volrover3"
    MACOSX_BUNDLE_INFO_STRING           "VolumeRover3 ${PROJECT_VERSION}"
    MACOSX_BUNDLE_COPYRIGHT             "Copyright (c) The University of Texas at Austin / CVC"
    MACOSX_BUNDLE_ICON_FILE             "volrover_logo.icns"
    MACOSX_BUNDLE_INFO_PLIST            "${PROJECT_SOURCE_DIR}/CMake/Info.plist.in"
    OUTPUT_NAME                         "VolumeRover3"
  )
elseif(WIN32)
  # WIN32_EXECUTABLE TRUE => /SUBSYSTEM:WINDOWS so no console window pops
  # up when launching from Explorer.
  set_target_properties(volrover3 PROPERTIES WIN32_EXECUTABLE TRUE)
endif()

target_include_directories(volrover3 PRIVATE ${PROJECT_SOURCE_DIR}/inc)

# Link libraries
if(QT_VERSION_MAJOR EQUAL 6)
  target_link_libraries(volrover3
    cvc::cvc
    Qt6::Core
    Qt6::Widgets
    Qt6::OpenGL
    Qt6::OpenGLWidgets
    ${VTK_LIBRARIES}
    ${OPENGL_LIBRARIES}
  )
else()
  target_link_libraries(volrover3
    cvc::cvc
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
if(APPLE)
  # Drop the .app bundle directly under the install prefix so a user can
  # drag VolumeRover3.app into /Applications.
  install(TARGETS volrover3
    BUNDLE      DESTINATION .            COMPONENT volrover3
    RUNTIME     DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT volrover3
  )
else()
  install(TARGETS volrover3
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    COMPONENT volrover3
  )
endif()

# Linux: install a Freedesktop .desktop file and PNG icon for the menu /
# launcher. The hicolor PNG installation is owned by the root CMakeLists.
if(UNIX AND NOT APPLE)
  if(EXISTS "${PROJECT_SOURCE_DIR}/share/applications/volrover3.desktop.in")
    configure_file(
      "${PROJECT_SOURCE_DIR}/share/applications/volrover3.desktop.in"
      "${CMAKE_CURRENT_BINARY_DIR}/volrover3.desktop"
      @ONLY)
    install(FILES "${CMAKE_CURRENT_BINARY_DIR}/volrover3.desktop"
      DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/applications
      COMPONENT volrover3)
  endif()
endif()

# Bundle the README + LICENSE with the volrover3 component for the
# end-user-facing distributions.
install(FILES
  "${CMAKE_CURRENT_SOURCE_DIR}/README.md"
  "${PROJECT_SOURCE_DIR}/LICENSE"
  DESTINATION ${CMAKE_INSTALL_DOCDIR}/volrover3
  COMPONENT volrover3)

# Windows: bundle runtime DLLs alongside the executable AND run windeployqt
# to copy Qt platform plugins (qwindows.dll), styles, and translations.
# Without windeployqt the resulting EXE will not start on a vanilla
# Windows machine because Qt cannot find a platform plugin.
if(WIN32)
  install(FILES $<TARGET_RUNTIME_DLLS:volrover3>
    DESTINATION ${CMAKE_INSTALL_BINDIR}
    COMPONENT volrover3)

  # ── Belt-and-suspenders runtime-dependency walker ──
  #
  # $<TARGET_RUNTIME_DLLS:volrover3> only resolves DLLs that arrived
  # via CMake imported targets with IMPORTED_LOCATION set (e.g. modern
  # Qt6, vcpkg-built cgal, vtk). It silently misses:
  #   - fftw3.dll  : found via legacy CMake/FindFFTW.cmake which only
  #                  sets variables, no imported target.
  #   - zlib1.dll, libpng16.dll, etc. : transitive deps pulled in via
  #                                     hdf5/imagemagick that don't
  #                                     propagate through cvc's
  #                                     interface link libraries.
  #   - cudart64_*.dll : if a future change accidentally re-introduces
  #                      a dynamic CUDA::cudart link.
  #
  # file(GET_RUNTIME_DEPENDENCIES) is CMake's PE-import walker. We run
  # it at install time over the staged volrover3.exe and cvc.dll, then
  # copy every non-system DLL it finds into bin/. Search dirs include
  # the build tree, CMAKE_PREFIX_PATH (vcpkg + Qt + VTK) and
  # CUDAToolkit_BIN_DIR (in case dynamic cudart slips in).
  #
  # We pre-compute the search-dir list at configure time and store it
  # as a quoted CMake list so paths with spaces survive the install
  # script generation intact.
  set(_cvc_runtime_search_dirs
    "${CMAKE_BINARY_DIR}/bin"
    "${CMAKE_BINARY_DIR}/lib"
  )
  foreach(_p IN LISTS CMAKE_PREFIX_PATH)
    list(APPEND _cvc_runtime_search_dirs "${_p}/bin")
  endforeach()
  if(DEFINED CUDAToolkit_BIN_DIR)
    list(APPEND _cvc_runtime_search_dirs "${CUDAToolkit_BIN_DIR}")
  endif()

  install(CODE "
    set(_search_dirs \"${_cvc_runtime_search_dirs}\")
    list(PREPEND _search_dirs \"\${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_BINDIR}\")
    if(DEFINED ENV{QT_ROOT_DIR})
      list(APPEND _search_dirs \"\$ENV{QT_ROOT_DIR}/bin\")
    endif()
    if(DEFINED ENV{VCPKG_INSTALLATION_ROOT})
      list(APPEND _search_dirs \"\$ENV{VCPKG_INSTALLATION_ROOT}/installed/x64-windows/bin\")
    endif()

    file(GET_RUNTIME_DEPENDENCIES
      EXECUTABLES \"\${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_BINDIR}/volrover3.exe\"
      LIBRARIES   \"\${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_BINDIR}/cvc.dll\"
      RESOLVED_DEPENDENCIES_VAR _resolved
      UNRESOLVED_DEPENDENCIES_VAR _unresolved
      CONFLICTING_DEPENDENCIES_PREFIX _conflicting
      DIRECTORIES \${_search_dirs}
      PRE_EXCLUDE_REGEXES
        [[api-ms-.*]]
        [[ext-ms-.*]]
      POST_EXCLUDE_REGEXES
        [[.*/system32/.*\\.dll]]
        [[.*/SysWOW64/.*\\.dll]]
        [[.*/[Ww]indows/.*\\.dll]]
    )

    foreach(_dll IN LISTS _resolved)
      message(STATUS \"Bundling runtime dep: \${_dll}\")
      file(INSTALL \"\${_dll}\"
        DESTINATION \"\${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_BINDIR}\"
        FOLLOW_SYMLINK_CHAIN)
    endforeach()

    if(_unresolved)
      message(STATUS \"Unresolved runtime deps (assumed system): \${_unresolved}\")
    endif()
  " COMPONENT volrover3)

  if(QT_VERSION_MAJOR EQUAL 6)
    set(_qt_bin_hint "${Qt6_DIR}/../../../bin")
  else()
    set(_qt_bin_hint "${Qt5_DIR}/../../../bin")
  endif()
  # Prefer aqtinstall's windeployqt (exported via $ENV{QT_ROOT_DIR}) over
  # any vcpkg-shipped one. vcpkg's qt6-base port lacks translations/
  # catalogs.json and (for debug builds) Qt6OpenGLWidgetsd.dll, so its
  # windeployqt fails the dependency walk and emits warnings about
  # missing translation catalogs. Search aqtinstall's bin first, then
  # the Qt cmake-config-derived hint, then fall back to PATH.
  find_program(WINDEPLOYQT_EXECUTABLE
    NAMES windeployqt
    HINTS "$ENV{QT_ROOT_DIR}/bin" "${_qt_bin_hint}"
    NO_CMAKE_SYSTEM_PATH)
  if(WINDEPLOYQT_EXECUTABLE)
    # CPack runs `cmake --install` per-component before invoking the
    # archive / NSIS generator, so this install(CODE) script is the right
    # hook to materialize Qt's platform plugins into the staged tree.
    install(CODE "
      execute_process(COMMAND \"${WINDEPLOYQT_EXECUTABLE}\"
        --release
        --no-translations
        --no-system-d3d-compiler
        --no-opengl-sw
        --no-compiler-runtime
        \"\${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_BINDIR}/volrover3.exe\")
    " COMPONENT volrover3)
  else()
    message(WARNING "windeployqt not found — Windows volrover3 packages "
      "will be missing Qt platform plugins and will fail to launch on "
      "machines without Qt installed.")
  endif()
endif()

# macOS: run macdeployqt during install so the .app bundle is fully
# self-contained (Qt frameworks, plugins, VTK & libcvc dylibs all
# rewritten with @executable_path/.. references).
if(APPLE)
  if(QT_VERSION_MAJOR EQUAL 6)
    set(_qt_bin_hint "${Qt6_DIR}/../../../bin")
  else()
    set(_qt_bin_hint "${Qt5_DIR}/../../../bin")
  endif()
  find_program(MACDEPLOYQT_EXECUTABLE
    NAMES macdeployqt
    HINTS "${_qt_bin_hint}")
  if(MACDEPLOYQT_EXECUTABLE)
    install(CODE "
      execute_process(COMMAND \"${MACDEPLOYQT_EXECUTABLE}\"
        \"\${CMAKE_INSTALL_PREFIX}/VolumeRover3.app\"
        -always-overwrite)
    " COMPONENT volrover3)
  else()
    message(WARNING "macdeployqt not found — macOS volrover3 .app will "
      "not be self-contained and will fail to launch without a Qt "
      "installation on the target machine.")
  endif()
endif()

# Tests
if(CVC_BUILD_TESTS)
  # Create a library with all volrover3 components except main.cpp for testing
  set(VOLROVER3_LIB_SOURCES ${VOLROVER3_SOURCES})
  list(REMOVE_ITEM VOLROVER3_LIB_SOURCES main.cpp)
  
  add_library(volrover3_lib STATIC ${VOLROVER3_LIB_SOURCES} ${VOLROVER3_HEADERS})
  target_include_directories(volrover3_lib PUBLIC ${PROJECT_SOURCE_DIR}/inc)
  
  if(QT_VERSION_MAJOR EQUAL 6)
    target_link_libraries(volrover3_lib
      cvc::cvc
      Qt6::Core
      Qt6::Widgets
      Qt6::OpenGL
      Qt6::OpenGLWidgets
      ${VTK_LIBRARIES}
      ${OPENGL_LIBRARIES}
    )
  else()
    target_link_libraries(volrover3_lib
      cvc::cvc
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

