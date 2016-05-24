#--------------------------------------------------------
# The CMakeLists.txt for:                       pCoverageMap
# Author(s):                              
#--------------------------------------------------------

SET(SRC
  CoverageMap.cpp
  CoverageMap_Info.cpp
  main.cpp
)

file(GLOB h_files *.h)

ADD_EXECUTABLE(pCoverageMap ${SRC})

TARGET_LINK_LIBRARIES(pCoverageMap
   ${MOOS_LIBRARIES}
   apputil
   geometry
   mbutil
   m
   pthread
   lcm)

pods_use_pkg_config_packages(pCoverageMap probabilistic_coverage)
pods_install_headers(${h_files} DESTINATION ${POD_NAME})
pods_install_executables(pCoverageMap)