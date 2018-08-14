cmake_minimum_required(VERSION 2.8.0)
project(main_movement)

LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/.. )

add_definitions(`xml2-config --cflags` -std=c++11 -Wall)

find_package(catkin REQUIRED COMPONENTS
    roscpp 
    message_filters
    humanoid_general
    humanoid_interface
    humanoid_msgs
    dynamic_reconfigure
    humanoid_loadmap
    motor_set_control
    )
find_package( LibXml2 REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS} ${LIBXML2_INCLUDE_DIR})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES  main_movement
  CATKIN_DEPENDS roscpp std_msgs humanoid_msgs  humanoid_general message_filters dynamic_reconfigure humanoid_loadmap motor_set_control)

add_library(main_movement 
	src/movement.cpp 
)

target_link_libraries(main_movement ${catkin_LIBRARIES} ${LIBXML2_LIBRARIES})
add_executable(main_movement_node src/movement_node.cpp)
target_link_libraries(main_movement_node main_movement ${catkin_LIBRARIES} ${LIBXML2_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

