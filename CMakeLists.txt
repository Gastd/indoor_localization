cmake_minimum_required(VERSION 2.8.3)
project(indoor_localization)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  roslib
  cmake_modules
  tf
)

find_package(Eigen3)
if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  # Not strictly necessary as Eigen is head only
  # Possibly map additional variables to the EIGEN3_ prefix.
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()
# Use ${EIGEN3_...} variables in either case below
include_directories(${EIGEN_INCLUDE_DIR})

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES indoor_localization
 CATKIN_DEPENDS geometry_msgs nav_msgs roscpp tf
#  DEPENDS system_lib
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(ar_pose_interface_node src/ar_pose_interface.cpp src/ar_map.cpp)

target_link_libraries(ar_pose_interface_node
  ${catkin_LIBRARIES}
)
