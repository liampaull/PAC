#--------------------------------------------------------
# The CMakeLists.txt for:                       pAdaptiveSurveyPlanner
# Author(s):                              
#--------------------------------------------------------

SET(SRC
  AdaptiveSurveyPlanner.cpp
  AdaptiveSurveyPlanner_Info.cpp
  main.cpp
)

file(GLOB h_files *.h)

ADD_EXECUTABLE(pAdaptiveSurveyPlanner ${SRC})

TARGET_LINK_LIBRARIES(pAdaptiveSurveyPlanner
   ${MOOS_LIBRARIES}
   apputil
   geometry
   mbutil
   m
   pthread)

pods_install_headers(${h_files} DESTINATION ${POD_NAME})
pods_install_executables(pAdaptiveSurveyPlanner)