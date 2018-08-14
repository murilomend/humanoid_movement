
#include "ros/ros.h"
#include "humanoid_control/HumanoidCtrlNode.h"


int main(int argc, char **argv)
{
    ros::init (argc, argv, "HumanoidCtrlNode");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    HumanoidCtrlNode walking_ctrl_node(nh, nh_private);
    ros::spin();
    return 0;
}



