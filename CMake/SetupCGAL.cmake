#
# Modern function for setting up CGAL
# CGAL provides computational geometry algorithms
#

function(SetupCGAL TargetName)
  if(NOT DISABLE_CGAL)
    find_package(CGAL CONFIG QUIET)
    
    if(CGAL_FOUND)
      message(STATUS "CGAL version: ${CGAL_VERSION}")
      
      # Modern CGAL provides imported targets
      if(TARGET CGAL::CGAL)
        target_link_libraries(${TargetName} PUBLIC CGAL::CGAL)
      else()
        # Fallback for older CGAL versions
        include(${CGAL_USE_FILE})
        target_link_libraries(${TargetName} PUBLIC ${CGAL_LIBRARIES})
        
        # CGAL may have special compiler requirements
        if(CGAL_CXX_FLAGS_INIT)
          target_compile_options(${TargetName} PUBLIC ${CGAL_CXX_FLAGS_INIT})
        endif()
      endif()
      
      target_compile_definitions(${TargetName} PUBLIC USING_CGAL)
      
      # GCC requires -frounding-math for CGAL
      if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
        message(STATUS "SetupCGAL: GCC detected, adding -frounding-math")
        target_compile_options(${TargetName} PUBLIC -frounding-math)
      endif()
      
      # Link GMP libraries if needed
      if(GMP_LIBRARIES)
        target_link_libraries(${TargetName} PUBLIC ${GMP_LIBRARIES})
      endif()
      if(GMPXX_LIBRARIES)
        target_link_libraries(${TargetName} PUBLIC ${GMPXX_LIBRARIES})
      endif()
    else()
      message(STATUS "CGAL not found - CGAL-dependent features disabled")
    endif()
  endif()
endfunction(SetupCGAL)
