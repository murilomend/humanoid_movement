
#include "ros/ros.h"
#include "humanoid_model/HumanoidModelNode.h"


int main(int argc, char **argv)
{
    ros::init (argc, argv, "HumanoidModelNode");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    ros::NodeHandle nh_private_walk("~/walk");
    ros::NodeHandle nh_private_movcreator("~/movcreator");
    HumanoidModelNode humanoidmodel_node(nh, nh_private,nh_private_walk,nh_private_movcreator);
    ros::spin();
    return 0;
}



