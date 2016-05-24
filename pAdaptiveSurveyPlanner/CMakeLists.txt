cmake_minimum_required(VERSION 2.6.0)
set(MAKE_BUILD_TYPE DEBUG)
#add_definitions("-std=c++11")
add_definitions("-g")

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME pAdaptiveSurveyPlanner)
include(cmake/pods.cmake)


SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake")
find_package(PkgConfig REQUIRED)
find_package(MOOSIvP REQUIRED)


include_directories("${PROJECT_SOURCE_DIR}/src")

add_subdirectory(src)
