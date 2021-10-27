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
  config_server
  unitree_legged_msgs
)

set(CMAKE_CXX_STANDARD 11)

###################################
## catkin specific configuration ##
###################################
catkin_package(
 INCLUDE_DIRS include
 #LIBRARIES conversion 
#  CATKIN_DEPENDS geometry_msgs mod_planning nav_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

link_directories(lib)

# On-board PC (AMD64)
if (CMAKE_SYSTEM_PROCESSOR MATCHES "(amd64)|(AMD64)")
  set(EXTRA_LIBS -pthread libunitree_legged_sdk_amd64.so lcm)
# Nvidia TX2 (ARM64)
else ()
  set(EXTRA_LIBS -pthread libunitree_legged_sdk_arm64.so lcm)

# set(CMAKE_CXX_FLAGS "-O3")

## Declare a C++ executable
add_executable(aliengo_bridge src/aliengo_bridge.cpp src/node.cpp)
target_link_libraries(aliengo_bridge ${catkin_LIBRARIES} ${EXTRA_LIBS})
