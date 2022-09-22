cmake_minimum_required(VERSION 2.8.3)
project(dvp_validation)

## c++11 feature
set(CMAKE_CXX_FLAGS "-std=c++14 -Wall -Wextra ${CMAKE_CXX_FLAGS}")

set(catkin_deps
    roscpp
    roslib
    roslint
    nodelet
    network_interface
    sensor_msgs
)

find_package(catkin REQUIRED ${catkin_deps} dvp_waypoint_replanner)
find_package(Boost REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS ${catkin_deps}
    LIBRARIES ${PROJECT_NAME}_nodelet
)

roslint_cpp()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

# nodelet
add_library(${PROJECT_NAME}_nodelet
    src/${PROJECT_NAME}_nodelet.cpp
)

target_link_libraries(${PROJECT_NAME}_nodelet
    ${catkin_LIBRARIES}
)

# node
add_executable(${PROJECT_NAME}_node
    src/dvp_validation_node.cpp
)

target_link_libraries(${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS
    ${PROJECT_NAME}_node
    ${PROJECT_NAME}_nodelet
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES nodelets.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
