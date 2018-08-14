#include "ros/ros.h"
#include "humanoid_interface/HumanoidInterface.h"


int main(int argc, char **argv)
{
    ros::init (argc, argv, "HumanoidInterfaceNode");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    HumanoidInterface humanoidinterface_node(nh, nh_private);
    ros::spin();

    return 0;
}



