#
# Modern macro for setting up Boost for a target
# Requires CMake 3.15+ for better Boost support
#

function(SetupBoost TargetName)
  set(Boost_USE_STATIC_LIBS OFF)
  set(Boost_USE_MULTITHREADED ON)
  
  # Find required Boost components
  # system and chrono are header-only on newer Boost (>=1.69/1.72) but
  # still compiled on older versions — use OPTIONAL_COMPONENTS so the
  # lookup succeeds either way.
  find_package(Boost 1.58 REQUIRED COMPONENTS 
    thread 
    date_time 
    regex 
    filesystem
    OPTIONAL_COMPONENTS
    system
    chrono
  )
  
  if(Boost_FOUND)
    message(STATUS "Boost version: ${Boost_VERSION}")
    message(STATUS "Boost include dirs: ${Boost_INCLUDE_DIRS}")
    message(STATUS "Boost libraries: ${Boost_LIBRARIES}")
    
    # Use modern target-based approach
    target_include_directories(${TargetName} PUBLIC ${Boost_INCLUDE_DIRS})
    target_link_libraries(${TargetName} PUBLIC ${Boost_LIBRARIES})
    
    # Platform-specific definitions
    target_compile_definitions(${TargetName} PUBLIC BOOST_ALL_DYN_LINK)
    
    if(MSVC)
      target_compile_definitions(${TargetName} PRIVATE 
        _VARIADIC_MAX=10
        BOOST_ALL_NO_LIB
      )
    endif()
  else()
    message(FATAL_ERROR 
      "Boost not found! Please set BOOST_ROOT, BOOST_INCLUDEDIR, or BOOST_LIBRARYDIR environment variables.")
  endif()
endfunction(SetupBoost)
