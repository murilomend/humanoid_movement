#include "ros/ros.h"
#include "motor_set_control/motorset.h"

int main(int argc, char **argv)
{
    ros::init (argc, argv, "MotorSetNode");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private;
    MotorSet motorset_node(nh, nh_private);
    ros::spin();
    return 0;
}
