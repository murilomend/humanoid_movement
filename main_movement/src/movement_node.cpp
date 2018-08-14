#include "ros/ros.h"
#include "../include/main_movement/movement.h"


int main(int argc, char **argv)
{
    ros::init (argc, argv, "MovementNode");
    ros::NodeHandle nh;
    move::Movement movement_node(nh);
    ros::spin();
    return 0;
}

