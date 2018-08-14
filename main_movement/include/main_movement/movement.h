#ifndef MOVEMENT_H
#define MOVEMENT_H

#include "ros/ros.h"
#include <message_filters/subscriber.h>
#include <cmath>

#include "humanoid_general/Enums.h"

#include "humanoid_msgs/MovementMsg.h"
#include "humanoid_msgs/MovementSrv.h"
#include "humanoid_msgs/MotorSetSrv.h"
#include "humanoid_msgs/InterfaceSrv.h"
#include "humanoid_msgs/JointStateMsg.h"
#include "humanoid_msgs/WalkingMsg.h"
#include "humanoid_msgs/HeadMoveMsg.h"
#include <humanoid_msgs/WalkingMsg.h>
#include <humanoid_msgs/InterfaceSrv.h>
#include <sensor_msgs/Imu.h>

namespace move {
    enum PAGE_MOVEMENT   {NONE, RIGHT_LEG_KICK,LEFT_LEG_KICK, FRONT_STAND_UP,BACK_STAND_UP};
    enum MOVE_STATE      {STOP,DO_NOTHING,GO};

    class Movement
    {
    public:
        Movement(ros::NodeHandle nh_);
    private:
        typedef humanoid_msgs::MovementMsg                  MovementMsg;
        typedef humanoid_msgs::JointStateMsg                JointStateMsg;
        typedef humanoid_msgs::WalkingMsg                   WalkingMsg;
        typedef humanoid_msgs::HeadMoveMsg                  HeadMoveMsg;

        //ROS Node
        ros::NodeHandle    nh;

        //ROS Publishers
        ros::Publisher                   movementTopic;
        ros::Publisher                   jointStateTopic;
        ros::Publisher                   headJointStateTopic;
        ros::Publisher                   walkingTopic;

        //ROS  Services
        ros::ServiceServer movementSrv;

        //ROS  Service client
        ros::ServiceClient            interfaceCli;
        humanoid_msgs::InterfaceSrv   interfaceSrv;

        ros::ServiceClient           motorSetCli;
        humanoid_msgs::MotorSetSrv   motorSetSrv;

        //ROS Subsriber
        typedef sensor_msgs::Imu                        ImuMsg;
        typedef message_filters::Subscriber<ImuMsg>     ImuSub;
        boost::shared_ptr<ImuSub>                       imuSubPtr;

        //ROS Timers
        ros::Timer runTimer;

        //Methods
        bool movementService(humanoid_msgs::MovementSrv::Request &msg, humanoid_msgs::MovementSrv::Response &res);
        void runCallBack(const ros::TimerEvent&);
        void publish();
        void pageMovements(int pageEnum);
        void updateHeadCommand(HeadMoveMsg msg);
        void sendHeadMessage(double x,double y,double vel);
        void sendWalkingParam(int go,double vx,double vy, double vz);
        void fallSrv();
        void imuCallback(const sensor_msgs::ImuPtr &msg);

        //State Variables
        //Verificar se precisa do falling
        bool walking;
        bool xHeadMove;
        bool yHeadMove;

        //Variables
        double dt;
        double xPos;
        double yPos;
        double desiredXPos;
        double desiredYPos;
        double roll;
        double pitch;
        double yaw;

        //Constants
        static const int HEAD_VEL = 10;  // Em graus por segundo
    };

}

#endif // MOTORSET_H
