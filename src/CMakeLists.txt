cmake_minimum_required(VERSION 3.15)

# Build options
option(CVC_USING_XMLRPC "Enable cvc::state network sharing with XMLRPC" OFF)

add_subdirectory(cvc)
add_subdirectory(cvc-cli)

if(CVC_USING_XMLRPC)
  add_subdirectory(xmlrpc)
endif()

# Note: CUDA is now integrated via native CMake support (3.17+)
# No special clean targets needed - handled automatically

# Build volrover3 application if Qt (5 or 6) and VTK are available
option(CVC_BUILD_VOLROVER3 "Build VolRover3 application" ON)
if(CVC_BUILD_VOLROVER3)
  find_package(Qt6 QUIET COMPONENTS Core Widgets OpenGL OpenGLWidgets)
  if(NOT Qt6_FOUND)
    find_package(Qt5 QUIET COMPONENTS Core Widgets OpenGL)
  endif()
  find_package(VTK QUIET)
  
  if((Qt6_FOUND OR Qt5_FOUND) AND VTK_FOUND)
    if(Qt6_FOUND)
      message(STATUS "Building VolRover3 application (Qt6 and VTK found)")
    else()
      message(STATUS "Building VolRover3 application (Qt5 and VTK found)")
    endif()
    add_subdirectory(volrover3)
  else()
    if(NOT Qt6_FOUND AND NOT Qt5_FOUND)
      message(STATUS "Neither Qt6 nor Qt5 found - skipping VolRover3 build")
    endif()
    if(NOT VTK_FOUND)
      message(STATUS "VTK not found - skipping VolRover3 build")
    endif()
  endif()
endif()
