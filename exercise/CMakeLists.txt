cmake_minimum_required(VERSION 2.8.3)
project(exercise)

find_package(OpenCV 3.2 REQUIRED)
find_package(OpenMP REQUIRED)
find_package(Boost COMPONENTS graph system REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  nav_msgs
  message_filters
  sensor_mockup
  amiro_msgs
  )

if (OPENMP_FOUND)
  message("OpenMP FOUND!")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
else (OPENMP_FOUND)
  message("NO OpenMP FOUND!")
endif (OPENMP_FOUND)

set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -Wall -Wno-deprecated-declarations -Ofast -Wno-write-strings -std=c++11 -march=sandybridge -mtune=haswell")

catkin_package(
  CATKIN_DEPENDS
  roscpp
  LIBRARIES ${PROJECT_NAME}
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  includes
)

add_executable(mapping_with_known_poses
  src/mapping_with_known_poses.cpp
  )
target_link_libraries(mapping_with_known_poses
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  )

add_executable(beacon_navigation
  src/beacon_navigation.cpp
  )
target_link_libraries(beacon_navigation
  ${catkin_LIBRARIES}
  ${Boost_LIBRARY_DIR}
  )

add_executable(line_following_node
  src/line_following_node.cpp
  )
add_dependencies(line_following_node amiro_msgs_generate_messages_cpp)
target_link_libraries(line_following_node
  ${catkin_LIBRARIES}
  )

add_executable(wall_following
  src/wall_following.cpp
  )
target_link_libraries(wall_following
  ${catkin_LIBRARIES}
  )
