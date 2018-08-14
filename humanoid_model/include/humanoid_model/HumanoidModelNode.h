#ifndef HUMANOIDMODELNODE_H
#define HUMANOIDMODELNODE_H


#include <iostream>
#include <fstream>
#include <string>
#include <unistd.h>
#include "humanoid_model/HumanoidModel.h"
#include "humanoid_general/State.h"
#include "humanoid_general/MathUtils.h"
#include "humanoid_general/Enums.h"
#include "humanoid_loadmap/Mapping.h"
#include "eigen_conversions/eigen_msg.h"

#include "ros/ros.h"

#include <dynamic_reconfigure/server.h>
#include <humanoid_msgs/HumanoidModelConfig.h>
#include <humanoid_msgs/HumanoidModelMovCreatorConfig.h>

#include <message_filters/subscriber.h>

#include "humanoid_msgs/JointStateMsg.h"
#include "humanoid_msgs/EndEffStateMsg.h"
#include "humanoid_msgs/HumanoidControlMsg.h"
#include "humanoid_msgs/LoadMapConfigsSrv.h"
#include "humanoid_msgs/LoadHumanoidPropertiesSrv.h"
#include "humanoid_msgs/HumanoidPropertiesMsg.h"




class HumanoidModelNode
{
public:
    HumanoidModelNode(ros::NodeHandle nh_,ros::NodeHandle nh_private_,ros::NodeHandle nh_private_walk_,ros::NodeHandle nh_private_movcreator_);
    ~HumanoidModelNode();
private:
    Mapping map;
    int     robotDOF;
    int     urdfDOF;
    int     ikDOF;
    double  dt;
    //ROS Node
    ros::NodeHandle    nh;
    ros::NodeHandle    nh_private;
    ros::NodeHandle    nh_private_walk;
    ros::NodeHandle    nh_private_movcreator;

    //ROS Subscribers
    typedef humanoid_msgs::JointStateMsg                JointStateMsg;
    typedef message_filters::Subscriber<JointStateMsg>  JointStateSub;
    boost::shared_ptr<JointStateSub>                    jointStateSubPtr;

    typedef humanoid_msgs::EndEffStateMsg               EndEffStateMsg;
    typedef message_filters::Subscriber<EndEffStateMsg> EndEffStateSub;
    boost::shared_ptr<EndEffStateSub>                   endEffStateSubPtr;

    typedef humanoid_msgs::HumanoidControlMsg               HumanoidControlMsg;
    typedef message_filters::Subscriber<HumanoidControlMsg> HumanoidControlSub;
    boost::shared_ptr<HumanoidControlSub>                   humanoidControlSubPtr;

    HumanoidControlMsg  humanoidControlMsg;


    //ROS Publishers
    ros::Publisher                   jointStateTopic;
    JointStateMsg                    jointMsg;
    ros::Publisher                   endEffStateTopic;
    EndEffStateMsg                   endEffMsg;

    ros::Publisher                             humanoidPropsTopic;
    humanoid_msgs::HumanoidPropertiesMsg       humanoidPropsMsg;


    //ROS Timers
    ros::Timer runTimer;


    //ROS  Services
    ros::ServiceClient                 mapCli;
    humanoid_msgs::LoadMapConfigsSrv   mapMsg;
    ros::ServiceServer                 humanoidPropsSrv;

    //ROS Dynamic Reconfigure Server
    typedef humanoid_msgs::HumanoidModelConfig                 HumanoidModelConfig;
    typedef dynamic_reconfigure::Server<HumanoidModelConfig>   HumanoidModelConfigServer;
    boost::shared_ptr<HumanoidModelConfigServer>               config_server;

    typedef humanoid_msgs::HumanoidModelMovCreatorConfig                 HumanoidModelMovCreatorConfig;
    typedef dynamic_reconfigure::Server<HumanoidModelMovCreatorConfig>   HumanoidModelMovCreatorConfigServer;
    boost::shared_ptr<HumanoidModelMovCreatorConfigServer>               config_server_mov;

    HumanoidModelMovCreatorConfig  modelMovCreatorMsg;


    //Config Params
    boost::mutex mutex;
    double squat;
    double open;
    double incl;
    double sideIncl;
    double footIncl;
    double comX;
    double comY;
    double comZ;
    double arm0;
    double arm1;
    double arm2;
    bool   override_com;
    bool   calcInvDyn;
    bool   calcZMP;
    bool   calcCOM;
    bool   calcIK;
    bool   calcFK;
    bool   setRobotFlag;

    //RoboModel
    HumanoidModel  robot;
    std::string    fUrdf;
    BodyPointState trunk;
    BodyPointState lLegBP;
    BodyPointState rLegBP;
    std::vector<BodyPointState> vecBody;
    std::vector<BodyPointState> vecBodyBuff;

    Eigen::Vector3d  zmpPoint;
    Eigen::Vector3d  comPoint;
    Eigen::Vector3d  footComPoint;
    Eigen::VectorXd  torq;





    //Methods

    bool humanoidPropertiesSrv(humanoid_msgs::LoadHumanoidPropertiesSrv::Request  &msg,
                               humanoid_msgs::LoadHumanoidPropertiesSrv::Response &res);
    void sendJointState(const JointState &jointState, const double &dt);
    void jointStateCallback(const humanoid_msgs::JointStateMsgPtr &jointState);
    void humanoidControlCallback(const humanoid_msgs::HumanoidControlMsgPtr &humanoidControl);
    void endEffStateCallback(const humanoid_msgs::EndEffStateMsgPtr &endEffState);
    void loadRobotParams();
    void setRobot(double squat, double open, double incl, double sideIncl, double footIncl);
    void loadMap();
    void reconfigCallback(humanoid_msgs::HumanoidModelConfig& config, uint32_t level);
    void reconfigMovCreatorCallback(humanoid_msgs::HumanoidModelMovCreatorConfig& config, uint32_t level);
    void runCallBack(const ros::TimerEvent&);
    void sendHumanoidProps(const humanoid_msgs::HumanoidPropertiesMsg &msg);


};

#endif // HUMANOIDMODELNODE_H
