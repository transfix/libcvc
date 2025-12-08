cmake_minimum_required(VERSION 3.15)

# Collect source and header files
file(GLOB SOURCE_FILES *.cpp)
file(GLOB INCLUDE_FILES ../../inc/xmlrpc/*.[Hh])

# IDE source organization
source_group("Source Files" FILES ${SOURCE_FILES})
source_group("Header Files" FILES ${INCLUDE_FILES})

# Create library
add_library(xmlrpc ${SOURCE_FILES} ${INCLUDE_FILES})
add_library(cvc::xmlrpc ALIAS xmlrpc)

# Modern include directory handling
target_include_directories(xmlrpc
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/inc>
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
