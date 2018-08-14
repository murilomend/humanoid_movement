cmake_minimum_required(VERSION 2.8.3)
project(humanoid_model)


LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/.. )

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

add_definitions(-std=c++11 -Wall )

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
FIND_PACKAGE (RBDL COMPONENTS URDFReader REQUIRED)
FIND_PACKAGE (Eigen3 REQUIRED)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  std_msgs
  humanoid_msgs
  humanoid_general
  humanoid_loadmap
  dynamic_reconfigure
  message_filters
  eigen_conversions
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS roscpp std_msgs humanoid_general humanoid_loadmap humanoid_msgs   dynamic_reconfigure  message_filters eigen_conversions
)
   

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
INCLUDE_DIRECTORIES ( ${RBDL_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIR} ${LIBXML2_INCLUDE_DIR})
include_directories(include ${catkin_INCLUDE_DIRS})


add_library(invkin src/InvKin.cpp)
add_library(robotmodel src/RobotModel.cpp)
add_library(humanoidmodel src/HumanoidModel.cpp)
add_library(humanoidmodelnode src/HumanoidModelNode.cpp)
ADD_EXECUTABLE (humanoidmodel_node src/humanoidmodel_node.cpp)



target_link_libraries(invkin ${EIGEN3_LIBRARY_DIR})
target_link_libraries(robotmodel map ${RBDL_LIBRARY} ${RBDL_URDFReader_LIBRARY}  ${EIGEN3_LIBRARY_DIR})
target_link_libraries(humanoidmodel map invkin robotmodel ${EIGEN3_LIBRARY_DIR})
target_link_libraries(humanoidmodelnode humanoidmodel ${EIGEN3_LIBRARY_DIR}  ${catkin_LIBRARIES})
target_link_libraries(humanoidmodel_node humanoidmodelnode  ${catkin_LIBRARIES})




