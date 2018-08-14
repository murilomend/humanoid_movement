cmake_minimum_required(VERSION 2.8.3)
project(humanoid_loadmap)


LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/.. )

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

add_definitions(`xml2-config --cflags` -std=c++11 -Wall )

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
FIND_PACKAGE (RBDL COMPONENTS URDFReader REQUIRED)
FIND_PACKAGE (Eigen3 REQUIRED)
find_package(LibXml2 REQUIRED)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  humanoid_general
  humanoid_msgs 
  eigen_conversions
  roslib 

)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES 
  CATKIN_DEPENDS roscpp humanoid_general humanoid_msgs  eigen_conversions   roslib 
)
   

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
INCLUDE_DIRECTORIES ( ${RBDL_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIR} ${LIBXML2_INCLUDE_DIR})
include_directories(include ${catkin_INCLUDE_DIRS} ${humanoid_general_INCLUDE_DIRS} ${humanoid_msgs_INCLUDE_DIRS} ${eigen_conversions_INCLUDE_DIRS})



add_library(loader src/Loader.cpp)
add_library(map src/Mapping.cpp)
add_library(loadmap src/LoadMapNode.cpp)
ADD_EXECUTABLE (loadmap_node src/loadmap_node.cpp)


target_link_libraries(map ${EIGEN3_LIBRARY_DIR})
target_link_libraries(loader  map ${RBDL_LIBRARY} ${RBDL_URDFReader_LIBRARY} ${EIGEN3_LIBRARY_DIR} ${LIBXML2_LIBRARIES} )
target_link_libraries(loadmap  map loader ${catkin_LIBRARIES})
target_link_libraries(loadmap_node map loader loadmap ${catkin_LIBRARIES} )




