cmake_minimum_required(VERSION 3.15)

# Collect source and header files
file(GLOB SOURCE_FILES *.cpp)
file(GLOB INCLUDE_FILES ../../inc/xmlrpc/*.[Hh])

# IDE source organization
source_group("Source Files" FILES ${SOURCE_FILES})
source_group("Header Files" FILES ${INCLUDE_FILES})

# Create library
# Force STATIC to side-step Windows DLL symbol-export issues (the sources
# don't carry __declspec(dllexport) annotations, so a SHARED build on MSVC
# produces a DLL with no import library and downstream links fail).
add_library(xmlrpc STATIC ${SOURCE_FILES} ${INCLUDE_FILES})
add_library(cvc::xmlrpc ALIAS xmlrpc)

# Position-independent code so this STATIC library can be linked into a
# SHARED libcvc (consumers build libcvc as .so / .dylib; without PIC the
# linker rejects with "final link failed: bad value").
set_target_properties(xmlrpc PROPERTIES POSITION_INDEPENDENT_CODE ON)

# Modern include directory handling
target_include_directories(xmlrpc
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/inc>
    $<INSTALL_INTERFACE:include>
)

# Platform-specific definitions
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
  target_compile_definitions(xmlrpc PRIVATE __LINUX__)
elseif(CMAKE_SYSTEM_NAME STREQUAL "Windows")
  target_compile_definitions(xmlrpc PRIVATE __WIN32__ __WINDOWS__ _WINDOWS)
elseif(CMAKE_SYSTEM_NAME MATCHES ".*BSD")
  target_compile_definitions(xmlrpc PRIVATE __BSD__)
endif()

# Platform-specific libraries
if(WIN32)
  target_link_libraries(xmlrpc PRIVATE wsock32 ws2_32)
endif()

# Install into the same export set as the main cvc target so downstream
# consumers of libcvc (FetchContent or find_package) get cvc::xmlrpc.
include(GNUInstallDirs)
install(TARGETS xmlrpc
  EXPORT cvcTargets
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT libcvc
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT libcvc
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT libcvc
)
install(DIRECTORY ${PROJECT_SOURCE_DIR}/inc/xmlrpc
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  COMPONENT libcvc
  FILES_MATCHING PATTERN "*.h" PATTERN "*.H"
)
