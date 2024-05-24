cmake_minimum_required(VERSION 3.0.2)
project(planning)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  nav_msgs
)
add_subdirectory(third_party/GeographicLib)
find_package(Eigen3 REQUIRED)
find_package(rosbag REQUIRED)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES 
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs nav_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${rosbag_INCLUDE_DIRS}
  third_party/GeographicLib/include
)
include_directories("${PROJECT_SOURCE_DIR}/include")
## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/my_image_transport.cpp
# )

add_executable(RecPath src/RecPath.cpp  
                    )
target_link_libraries(RecPath ${catkin_LIBRARIES} ${rosbag_LIBRARIES} libGeographiccc)
add_executable(GpsPath src/GpsPath.cpp  
                    )
target_link_libraries(GpsPath ${catkin_LIBRARIES} ${rosbag_LIBRARIES} libGeographiccc)
add_executable(pure_persuit src/pure_persuit.cpp  
                    )
target_link_libraries(pure_persuit ${catkin_LIBRARIES} ${rosbag_LIBRARIES})
# add_executable(rotate src/rotate.cpp)
# target_link_libraries(rotate ${catkin_LIBRARIES})
# target_link_libraries(rotate ${OpenCV_LIBS})
# add_executable(search src/search.cpp)
# target_link_libraries(search ${catkin_LIBRARIES})
# target_link_libraries(search ${OpenCV_LIBRARIES})
