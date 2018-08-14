cmake_minimum_required(VERSION 2.8.3)
project(humanoid_walking)


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
  message_filters
  eigen_conversions
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES  humanoid_walking
  CATKIN_DEPENDS roscpp std_msgs humanoid_msgs  humanoid_general message_filters  eigen_conversions
)
   

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
INCLUDE_DIRECTORIES (${EIGEN3_INCLUDE_DIR})
include_directories(include ${catkin_INCLUDE_DIRS})



add_library(humanoid_walking
    src/LipWalk.cpp
    src/LipWalkNode.cpp
)

ADD_EXECUTABLE (lipwalk_node src/lipwalk_node.cpp)

add_dependencies(humanoid_walking ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(humanoid_walking ${EIGEN3_LIBRARY_DIR} ${catkin_LIBRARIES})
target_link_libraries(lipwalk_node ${EIGEN3_LIBRARY_DIR} ${catkin_LIBRARIES} humanoid_walking)



install(TARGETS humanoid_walking
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS lipwalk_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

