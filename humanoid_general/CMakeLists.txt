################################################################################
# CMake
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(humanoid_general)


LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/.. )

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

add_definitions(`xml2-config --cflags` -std=c++11 -Wall )

################################################################################
# Packages
################################################################################
FIND_PACKAGE (Eigen3 REQUIRED)
find_package(LibXml2 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)





################################################################################
# Declare ROS messages, services and actions
################################################################################

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Catkin specific configuration
################################################################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES    humanoid_general
  CATKIN_DEPENDS roscpp
)

################################################################################
# Build
################################################################################
INCLUDE_DIRECTORIES (${EIGEN3_INCLUDE_DIR} ${LIBXML2_INCLUDE_DIR})
include_directories (include ${catkin_INCLUDE_DIRS})

add_library(humanoid_general
    src/Enums.cpp
    src/basicxml.cpp
    src/MathUtils.cpp
    src/State.cpp
    src/spline.cpp
)
add_dependencies(humanoid_general ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(humanoid_general  ${EIGEN3_LIBRARY_DIR} ${LIBXML2_LIBRARIES} ${catkin_LIBRARIES})


################################################################################
# Install
################################################################################

install(TARGETS humanoid_general
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h  *.hpp"
  PATTERN ".svn" EXCLUDE
)

