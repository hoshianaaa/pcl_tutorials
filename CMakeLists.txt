cmake_minimum_required(VERSION 3.0.2)
project(pcl_tutorial)

find_package(catkin REQUIRED)
find_package(PCL 1.8 REQUIRED)

catkin_package(
)

include_directories(
 include
 ${PCL_INCLUDE_DIRS}
 #${catkin_INCLUDE_DIRS}
)

add_executable(lccp src/lccp.cpp)

target_link_libraries(lccp
  ${PCL_LIBRARIES}
  #${catkin_LIBRARIES}
)

add_executable(icp src/icp.cpp)

target_link_libraries(icp
  ${PCL_LIBRARIES}
  #${catkin_LIBRARIES}
)

add_executable(tf src/tf.cpp)

target_link_libraries(tf
  ${PCL_LIBRARIES}
  #${catkin_LIBRARIES}
)
