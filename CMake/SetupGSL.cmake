#
# Modern function for setting up the GNU Scientific Library (GSL)
#

function(SetupGSL TargetName)
  find_package(GSL)
  
  if(GSL_FOUND)
    message(STATUS "GSL found: ${GSL_VERSION}")
    message(STATUS "GSL include dirs: ${GSL_INCLUDE_DIRS}")
    message(STATUS "GSL libraries: ${GSL_LIBRARIES}")
    
    # Use modern target-based approach
    target_include_directories(${TargetName} PUBLIC ${GSL_INCLUDE_DIRS})
    target_link_libraries(${TargetName} PUBLIC ${GSL_LIBRARIES})
    
    if(CMAKE_GSL_CXX_FLAGS)
      target_compile_options(${TargetName} PUBLIC ${CMAKE_GSL_CXX_FLAGS})
    endif()
  else()
    message(WARNING "GSL not found - GSL-dependent features disabled")
  endif()
endfunction(SetupGSL)
