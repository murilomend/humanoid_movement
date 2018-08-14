
#include "ros/ros.h"
#include "humanoid_learning/HumanoidLearningNode.h"


int main(int argc, char **argv)
{
    ros::init (argc, argv, "HumanoidLearningNode");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    HumanoidLearningNode walking_learning_node(nh, nh_private);
    ros::spin();
    return 0;
}



