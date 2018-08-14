cmake_minimum_required(VERSION 2.8.0)
project(motor_set_control)

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
    )
find_package( LibXml2 REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS} ${LIBXML2_INCLUDE_DIR})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES  motor_set_control
  CATKIN_DEPENDS roscpp std_msgs humanoid_msgs  humanoid_general message_filters dynamic_reconfigure humanoid_loadmap)

add_library(motor_set_control 
	src/motorset.cpp 
	src/jointcommand.cpp 
	src/page.cpp 
	src/pose.cpp
)

target_link_libraries(motor_set_control ${catkin_LIBRARIES} ${LIBXML2_LIBRARIES} humanoid_general)
add_executable(motorset_node src/motorset_node.cpp)
target_link_libraries(motorset_node motor_set_control ${catkin_LIBRARIES} ${LIBXML2_LIBRARIES})

#target_link_libraries(humanoid_interface_node  humanoid_interface ${catkin_LIBRARIES} )
#install(TARGETS motor_set_control RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

