#ifndef LIPWALKNODE_H
#define LIPWALKNODE_H


#include <iostream>
#include <fstream>
#include <string>
#include <unistd.h>
#include "humanoid_general/State.h"
#include "humanoid_general/Enums.h"
#include "humanoid_general/MathUtils.h"
#include "humanoid_walking/LipWalk.h"
#include "eigen_conversions/eigen_msg.h"

#include "ros/ros.h"


#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

#include <dynamic_reconfigure/server.h>
#include <humanoid_msgs/HumanoidLipWalkingParamsConfig.h>
#include <humanoid_msgs/HumanoidLipWalkingCmdConfig.h>
#include <humanoid_msgs/HumanoidLipWalkingCtrlConfig.h>


#include <message_filters/subscriber.h>

#include <humanoid_msgs/LipParamsSrv.h>
#include <humanoid_msgs/LipCmdSrv.h>
#include <humanoid_msgs/LipCtrlSrv.h>
#include <humanoid_msgs/HumanoidControlMsg.h>
#include <humanoid_msgs/LipParamsMsg.h>
#include <humanoid_msgs/WalkingMsg.h>
#include <humanoid_msgs/LipFeedBack.h>
#include <humanoid_msgs/EndEffStateMsg.h>
#include <humanoid_msgs/LoadHumanoidPropertiesSrv.h>
#include <humanoid_msgs/HumanoidPropertiesMsg.h>



class LipWalkNode
{
public:


    //ROS Node
    ros::NodeHandle    nh;
    ros::NodeHandle    nh_private;
    ros::NodeHandle    nh_private_params;
    ros::NodeHandle    nh_private_cmd;
    ros::NodeHandle    nh_private_ctrl;

    //ROS Publishers
    ros::Publisher                   lipTopic;
    humanoid_msgs::LipFeedBack       lipMsg;
    ros::Publisher                   paramsTopic;
    humanoid_msgs::LipParamsMsg      paramsMsg;
    ros::Publisher                   endEffTopic;
    humanoid_msgs::EndEffStateMsg    endEffMsg;

    //ROS Subscribers
    typedef humanoid_msgs::HumanoidControlMsg    ctrlMsg;
    typedef message_filters::Subscriber<ctrlMsg> ctrlSub;
    boost::shared_ptr<ctrlSub>                   ctrlSubPtr;
    humanoid_msgs::HumanoidControlMsg            humanoidCtrlMsg;

    typedef humanoid_msgs::HumanoidPropertiesMsg          HumanoidPropsMsg;
    typedef message_filters::Subscriber<HumanoidPropsMsg> HumanoidPropsSub;
    boost::shared_ptr<HumanoidPropsSub>                   humanoidPropsSubPtr;

    typedef humanoid_msgs::WalkingMsg                     WalkingMsg;
    typedef message_filters::Subscriber<WalkingMsg>       WalkingSub;
    boost::shared_ptr<WalkingSub>                         walkingSubPtr;

    //ROS  Services
    ros::ServiceServer paramsSrv;
    ros::ServiceServer cmdSrv;
    ros::ServiceServer ctrlSrv;
    ros::ServiceClient                        humanoidPropsCli;
    humanoid_msgs::LoadHumanoidPropertiesSrv  humanoidPropsMsg;

    //ROS Dynamic Reconfigure
    dynamic_reconfigure::ReconfigureRequest  srv_req;
    dynamic_reconfigure::ReconfigureResponse srv_resp;
    dynamic_reconfigure::BoolParameter       bool_param;
    dynamic_reconfigure::Config              conf;

    //ROS Dynamic Reconfigure Server
    boost::mutex mutex_params;
    typedef humanoid_msgs::HumanoidLipWalkingParamsConfig                 HumanoidLipWalkingParamsConfig;
    typedef dynamic_reconfigure::Server<HumanoidLipWalkingParamsConfig>   HumanoidLipWalkingParamsConfigServer;
    boost::shared_ptr<HumanoidLipWalkingParamsConfigServer>               config_server_params;

    boost::mutex mutex_cmd;
    typedef humanoid_msgs::HumanoidLipWalkingCmdConfig                 HumanoidLipWalkingCmdConfig;
    typedef dynamic_reconfigure::Server<HumanoidLipWalkingCmdConfig>   HumanoidLipWalkingCmdConfigServer;
    boost::shared_ptr<HumanoidLipWalkingCmdConfigServer>               config_server_cmd;

    boost::mutex mutex_ctrl;
    typedef humanoid_msgs::HumanoidLipWalkingCtrlConfig                 HumanoidLipWalkingCtrlConfig;
    typedef dynamic_reconfigure::Server<HumanoidLipWalkingCtrlConfig>   HumanoidLipWalkingCtrlConfigServer;
    boost::shared_ptr<HumanoidLipWalkingCtrlConfigServer>               config_server_ctrl;

    //ROS Timers
    ros::Timer timerLip;
    ros::Timer timerSend;

    //ROS Params

    Eigen::Vector3d  footCom;
    Eigen::Vector3d  refCom;

    double tS;
    double tD;
    double stepH;
    double zCCorr;
    double slope;

    double vx;
    double vy;
    double vz;
    bool   walking_flag;
    bool   walk_flag;
    bool   reset_walk;
    bool   lipUpdate;

    double delayL;
    double delayR;
    double delayAll;
    bool   delayAllChanged;
    bool   delayRLChanged;
    double timeFac;
    double sinFreq;
    double sinAmp;
    double sinPhase;
    double tSin;
    double tNormSin;
    double nSin;
    bool   startedLip;

    double dt;
    double lip_dt;
    int curve;


    //RoboModel
    BodyPointState rFoot;
    BodyPointState lFoot;
    BodyPointState trunk;
    std::vector<BodyPointState> vecBody;
    //LipWalk
    LipWalk        robotLip;


    LipWalkNode(ros::NodeHandle nh_,ros::NodeHandle nh_private_,ros::NodeHandle nh_private_params_,ros::NodeHandle nh_private_cmd_,ros::NodeHandle nh_private_ctrl_);
    ~LipWalkNode();

    //Methods
    void initWalkingSettings();
    void configHumanoidNode(bool lipBool);
    void sendGoal(const std::vector<BodyPointState> &vecBody, const double &dt);
    void send2Topic();
    void runCallBackLip(const ros::TimerEvent&);
    void runCallBackSend(const ros::TimerEvent&);
    void reconfigParamsCallback(humanoid_msgs::HumanoidLipWalkingParamsConfig& config, uint32_t level);
    void reconfigCmdCallback(humanoid_msgs::HumanoidLipWalkingCmdConfig& config, uint32_t level);
    void reconfigCtrlCallback(humanoid_msgs::HumanoidLipWalkingCtrlConfig& config, uint32_t level);
    bool updateWalkingParams(humanoid_msgs::LipParamsSrv::Request  &params,
                             humanoid_msgs::LipParamsSrv::Response &res);

    bool walkingCmd(humanoid_msgs::LipCmdSrv::Request  &cmd,
                    humanoid_msgs::LipCmdSrv::Response &res);

    void ctrlCallback(const humanoid_msgs::HumanoidControlMsgPtr &ctrl);
    void humanoidPropsCallback(const humanoid_msgs::HumanoidPropertiesMsgPtr &msg);
    void walkingCallback(const humanoid_msgs::WalkingMsgPtr &msg);
    void initPosition();






};



#endif // LIPWALKNODE_H
