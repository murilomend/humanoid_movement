#ifndef MOTORSET_H
#define MOTORSET_H

#include "jointcommand.h"
#include "pose.h"
#include "page.h"
#include <iostream>
#include <vector>
#include <string>

#include "ros/ros.h"
#include <message_filters/subscriber.h>
#include "humanoid_general/State.h"
#include "humanoid_general/spline.h"

#include "humanoid_msgs/MotorSetSrv.h"
#include "humanoid_msgs/InterfaceSrv.h"
#include "humanoid_msgs/JointStateMsg.h"
class MotorSet
{
public:
    MotorSet(ros::NodeHandle nh_, ros::NodeHandle nh_private_);
private:
    typedef humanoid_msgs::JointStateMsg                JointStateMsg;
    bool toRobot;
    bool toGazebo;

    double dt;

    //ROS Node
    ros::NodeHandle    nh;
    ros::NodeHandle    nh_private;

    //ROS Publishers
    ros::Publisher                   jointStateTopic;

    //ROS  Services
    ros::ServiceServer motorSetSrv;

    //ROS  Service client
    ros::ServiceClient            interfaceCli;
    humanoid_msgs::InterfaceSrv   interfaceSrv;

    //Methods
    bool motorSetService(humanoid_msgs::MotorSetSrv::Request  &msg,humanoid_msgs::MotorSetSrv::Response &res);
    void playPage(Page page, bool robot);
    void linearInterpol(Page page, bool robot);
    std::vector<double> getCurrentPose();

};

#endif // MOTORSET_H
