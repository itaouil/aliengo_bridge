cmake_minimum_required(VERSION 2.8.3)
project(aliengo_bridge)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  tf
  nav_msgs
  sensor_msgs
  geometry_msgs
  unitree_legged_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
 INCLUDE_DIRS include
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

link_directories(lib)

add_compile_options(-std=c++11)

set(EXTRA_LIBS -pthread libunitree_legged_sdk.so lcm)

set(CMAKE_CXX_FLAGS "-O3 -fPIC")

## Declare a C++ executable
add_executable(aliengo_bridge src/aliengo_bridge.cpp src/node.cpp)
target_link_libraries(aliengo_bridge ${catkin_LIBRARIES} ${EXTRA_LIBS})
