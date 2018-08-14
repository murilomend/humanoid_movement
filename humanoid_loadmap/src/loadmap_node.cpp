
#include "ros/ros.h"
#include "humanoid_loadmap/LoadMapNode.h"


int main(int argc, char **argv)
{
    ros::init (argc, argv, "LoadMapNode");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    LoadMapNode loadmap_node(nh, nh_private);
    ros::spin();
    return 0;
}



