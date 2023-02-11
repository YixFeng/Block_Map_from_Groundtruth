cmake_minimum_required(VERSION 3.0.2)
project(bm_generator)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  # pcl library
  pcl_conversions
  pcl_ros
  # msgs
  std_msgs
  sensor_msgs
  geometry_msgs
)

include_directories("/usr/include/eigen3")
find_package(OpenMP REQUIRED)
find_package(PCL REQUIRED QUIET)
find_package(Boost REQUIRED)

catkin_package(
  INCLUDE_DIRS
  DEPENDS PCL

  CATKIN_DEPENDS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
)

# include directories
include_directories(
	${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
)

# link directories
link_directories(
	${PCL_LIBRARY_DIRS}
)

###########
## Build ##
###########
add_executable(bm_generator src/bm_generator.cpp)
add_dependencies(bm_generator ${catkin_EXPORTED_TARGETS})
target_compile_options(bm_generator PRIVATE ${OpenMP_CXX_FLAGS})
target_link_libraries(bm_generator ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenMP_CXX_FLAGS})