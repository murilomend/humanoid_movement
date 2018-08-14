#include "humanoid_walking/LipWalkNode.h"
#include "ros/ros.h"


int main(int argc, char **argv)
{
    ros::init (argc, argv, "HumanoidWalkingNode");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    ros::NodeHandle nh_private_params("~/params");
    ros::NodeHandle nh_private_cmd("~/cmd");
    ros::NodeHandle nh_private_ctrl("~/ctrl");
    LipWalkNode lipwalk_node(nh, nh_private,nh_private_params,nh_private_cmd,nh_private_ctrl);
    ros::spin();
    return 0;
}



