cmake_minimum_required(VERSION 2.8.3)
project(humanoid_learning)


LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/.. )

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

add_definitions(`xml2-config --cflags` -std=c++11 -Wall )

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
FIND_PACKAGE (Eigen3 REQUIRED)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  std_msgs
  humanoid_msgs
  humanoid_general
  humanoid_loadmap
  message_filters
  eigen_conversions
  dynamic_reconfigure
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES  humanoid_learning
  CATKIN_DEPENDS roscpp std_msgs humanoid_msgs  humanoid_general humanoid_loadmap message_filters  eigen_conversions   dynamic_reconfigure   humanoid_loadmap
)
   

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
INCLUDE_DIRECTORIES (${EIGEN3_INCLUDE_DIR})
include_directories(include ${catkin_INCLUDE_DIRS})



add_library(humanoid_learning   src/HumanoidLearningNode.cpp)

ADD_EXECUTABLE (humanoidlearning_node src/humanoid_learning_node.cpp)

#add_dependencies(humanoid_learning ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(humanoid_learning ${EIGEN3_LIBRARY_DIR} ${catkin_LIBRARIES} map humanoid_general)
target_link_libraries(humanoidlearning_node ${EIGEN3_LIBRARY_DIR} ${catkin_LIBRARIES} humanoid_learning)




## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

