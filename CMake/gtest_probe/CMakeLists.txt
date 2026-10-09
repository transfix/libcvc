# Added by _cvc_gtest_msvc_check() in ../SetupGTest.cmake (MSVC only). Loads the
# installed GoogleTest in this throwaway directory scope, so its IMPORTED
# GTest::* targets stay here, and hands back where it is, its library type and
# its imported configurations. Builds nothing.
find_package(GTest ${CVC_GTEST_VERSION} CONFIG QUIET)
if(GTest_FOUND)
  get_target_property(_type GTest::gtest TYPE)
  get_target_property(_cfgs GTest::gtest IMPORTED_CONFIGURATIONS)
  if(NOT _cfgs)
    set(_cfgs "")
  endif()
  set(_cvc_gtest_probe "${GTest_DIR};${_type};${_cfgs}" PARENT_SCOPE)
endif()
