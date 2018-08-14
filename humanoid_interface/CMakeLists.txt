cmake_minimum_required(VERSION 2.8.3)

project(humanoid_interface CXX)

find_package(catkin REQUIRED COMPONENTS
    hardware_interface
    controller_manager
    realtime_tools
    dynamic_reconfigure
    humanoid_general
    humanoid_msgs
    humanoid_loadmap
    actionlib
    actionlib_msgs
    message_filters
    dynamixel_sdk
    dynamixel_workbench_toolbox
    )

set(CMAKE_BUILD_TYPE Debug)
add_definitions(-std=c++11 -Wall)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS hardware_interface realtime_tools dynamic_reconfigure humanoid_general humanoid_msgs humanoid_loadmap actionlib actionlib_msgs  message_filters    dynamixel_sdk dynamixel_workbench_toolbox
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(humanoid_interface        src/HumanoidInterface.cpp)
add_executable(humanoid_interface_node src/humanoid_interface_node.cpp)

target_link_libraries(humanoid_interface ${catkin_LIBRARIES} map)
target_link_libraries(humanoid_interface_node  humanoid_interface ${catkin_LIBRARIES} )

#Somente para exibir os headers na arvore do Qt Creator
FILE(GLOB_RECURSE headers "*.h")
add_custom_target("${PROJECT_NAME}_headers" SOURCES ${headers})
install(PROGRAMS scripts/setserial.sh DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
