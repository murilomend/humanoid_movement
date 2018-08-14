#ifndef HUMANOIDINTERFACE_H
#define HUMANOIDINTERFACE_H

#include <iostream>
#include <fstream>
#include <string>
#include <unistd.h>
#include <stdint.h>
#include "humanoid_general/State.h"
#include "humanoid_general/MathUtils.h"
#include "humanoid_general/Enums.h"
#include "humanoid_loadmap/Mapping.h"

#include "ros/ros.h"
#include "humanoid_msgs/JointStateMsg.h"
#include "humanoid_msgs/MotorStateMsg.h"
#include "humanoid_msgs/MotorPIDMsg.h"
#include "humanoid_msgs/InterfaceSrv.h"
#include "humanoid_msgs/LoadMapConfigsSrv.h"
#include "control_msgs/JointTrajectoryControllerState.h"


#include <message_filters/subscriber.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

#include <dynamic_reconfigure/server.h>
#include <humanoid_msgs/HumanoidInterfaceConfig.h>


#include <dynamixel_workbench_toolbox/dynamixel_multi_driver.h>
#include <dynamixel_workbench_toolbox/dynamixel_driver.h>

#define  RAD_2_MOTOR_MX   (4095./(2*3.14159))
#define  MOTOR_MX_2_RAD   ((2*3.14159)/(4095.))
#define  RPM_2_MOTOR_MX   0.229//TODOOOO


typedef struct
{
  std::vector<uint8_t>  torque;
  std::vector<uint32_t> pos;
  std::vector<uint32_t> prof_vel;
  std::vector<uint32_t> prof_acc;
  std::vector<uint32_t> max_pos;
  std::vector<uint32_t> min_pos;
  std::vector<uint16_t> kp;
  std::vector<uint8_t>  returnLevel;
  std::vector<uint8_t>  operMode;
  std::vector<uint8_t>  shutdown;

}WriteValue;

typedef struct
{
  std::vector<uint8_t>  torque;
  std::vector<uint32_t> curr;
  std::vector<uint32_t> pos;
  std::vector<uint32_t> vel;
}ReadValue;

class HumanoidInterface
{
public:
    HumanoidInterface(ros::NodeHandle nh_,ros::NodeHandle nh_private_);
    ~HumanoidInterface();
private:
    Mapping map;
    int     robotDOF;
    int     urdfDOF;
    int     ikDOF;
    double  dt;
    bool    toGazebo;
    bool    toRobot;
    bool    send2Motor;
    bool    readMotor;
    int     readParam;
    bool    pingAll;
    bool    pingScan;
    bool    testDT;
    bool    firstCmd;

    std::vector<int> ids;
    std::vector<int> idsInverse1;
    std::vector<int> idsInverse2;
    std::vector<int> idsProtocol;


    std::string device_name;
    double      protocol_version;
    int         baud_rate;
    bool        inversePorts;

    //ROS Node
    ros::NodeHandle    nh;
    ros::NodeHandle    nh_private;
    ros::Time lasttime;
    ros::Time currtime;
    ros::Time dtNow;
    ros::Time dtOld;




    //ROS Subscribers
    typedef humanoid_msgs::JointStateMsg                JointStateMsg;
    typedef message_filters::Subscriber<JointStateMsg>  JointStateSub;
    boost::shared_ptr<JointStateSub>                    jointStateSubPtr;
    boost::shared_ptr<JointStateSub>                    jointHeadStateSubPtr;

    typedef humanoid_msgs::MotorPIDMsg                  MotorPIDMsg;
    typedef message_filters::Subscriber<MotorPIDMsg>    MotorPIDSub;
    boost::shared_ptr<MotorPIDSub>                      motorPIDSubPtr;

    typedef control_msgs::JointTrajectoryControllerState        JointStateGazeboMsg;
    typedef message_filters::Subscriber<JointStateGazeboMsg>    JointStateGazeboSub;
    boost::shared_ptr<JointStateGazeboSub>                      jointStateGazeboSubPtr;
    JointStateGazeboMsg  jointStateGazeboMsg;

    MotorPIDMsg   motorPID;
    double        robotPID;
    double        gazeboPID;
    JointStateMsg jointMotor;
    JointStateMsg jointState;
    JointStateMsg jointHeadState;
    JointStateMsg jointStateOld;


    //ROS Publishers
    ros::Publisher                   motorStateTopic;
    humanoid_msgs::MotorStateMsg     motorStateMsg;


    ros::Publisher                   jointStateTopic;
    JointStateMsg                    jointReadState;


    ros::Publisher                   motorPIDTopic;
    MotorPIDMsg                      motorPIDMsg;
    bool   updatePID;

    //ROS  Services
    ros::ServiceServer interfaceSrv;

    //ROS Timers
    ros::Timer runTimer;


    //ROS  Services
    ros::ServiceClient                 mapCli;
    humanoid_msgs::LoadMapConfigsSrv   mapMsg;

    //ROS Actions    
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> *trj_client;
    control_msgs::FollowJointTrajectoryGoal                                  goal;

    //ROS Dynamic Reconfigure Server
    boost::mutex mutex;
    typedef humanoid_msgs::HumanoidInterfaceConfig                 HumanoidInterfaceConfig;
    typedef dynamic_reconfigure::Server<HumanoidInterfaceConfig>   HumanoidInterfaceConfigServer;
    boost::shared_ptr<HumanoidInterfaceConfigServer>               config_server;

    //ROS Dynamic Reconfigure Client
    dynamic_reconfigure::ReconfigureRequest srv_req;
    dynamic_reconfigure::ReconfigureResponse srv_resp;
    dynamic_reconfigure::DoubleParameter double_param;
    dynamic_reconfigure::Config conf;

    // Dynamixel Workbench Parameters
    std::vector<dynamixel_driver::DynamixelInfo*> dynamixel_info2_;
    std::vector<dynamixel_driver::DynamixelInfo*> dynamixel_info1_;
    dynamixel_driver::DynamixelInfo*              dynamixel_info_scan1_;
    dynamixel_driver::DynamixelInfo*              dynamixel_info_scan2_;
    dynamixel_multi_driver::DynamixelMultiDriver  *multi_driver2_;
    dynamixel_multi_driver::DynamixelMultiDriver  *multi_driver1_;
    dynamixel_multi_driver::DynamixelMultiDriver  *multi_driver_read2_;
    std::vector<dynamixel_driver::DynamixelInfo*> dynamixel_info_read2_;
    dynamixel_multi_driver::DynamixelMultiDriver  *multi_driver_read1_;
    std::vector<dynamixel_driver::DynamixelInfo*> dynamixel_info_read1_;
    dynamixel_driver::DynamixelDriver             *dynamixel_driver10_;
    dynamixel_driver::DynamixelDriver             *dynamixel_driver21_;
    dynamixel_driver::DynamixelDriver             *dynamixel_driver20_;
    dynamixel_driver::DynamixelDriver             *dynamixel_driver11_;
    WriteValue *writeValue1_;
    WriteValue *writeValue2_;

    int count;


    //RoboModel
    JointState  qState;

    //Methods
    void loadMap();
    void reconfigCallback(humanoid_msgs::HumanoidInterfaceConfig& config, uint32_t level);
    void jointStateCallback(const humanoid_msgs::JointStateMsgPtr &jointState);
    void jointHeadStateCallback(const humanoid_msgs::JointStateMsgPtr &jointHeadState);
    void jointStateGazeboCallback(const control_msgs::JointTrajectoryControllerStatePtr &msg);


    void motorPIDCallback(const humanoid_msgs::MotorPIDMsgPtr &motorPID);
    bool interfaceService(humanoid_msgs::InterfaceSrv::Request  &msg,
                          humanoid_msgs::InterfaceSrv::Response &res);
    void getRad2MotorConv(int index,double &rad2Motor,double &radVel2Motor);
    void send2Gazebo(const JointStateMsg &qState, const double &dt);
    void updateGazeboPID(const MotorPIDMsg &motorPID);
    bool send2Robot(const JointStateMsg& qState, const double &dt);
    void updateRobotPID(const MotorPIDMsg &motorPID);
    void updateRobotP(const double kp);
    void runCallBack(const ros::TimerEvent&);

    void initActionSettings();

    humanoid_msgs::JointStateMsg jointStateMsg2Motor(const Eigen::MatrixXi &map,const JointStateMsg& q0,const JointStateMsg& q1,double dt);
    humanoid_msgs::JointStateMsg motor2JointState(const Eigen::MatrixXi &map,const JointStateMsg& qMotor);
    std::vector<int> scanDynamixel();
    void clearDynamixelDriver();
    bool loadDynamixel();
    bool setTorque(bool onoff);
    bool setTorque(std::vector<bool> onoff);
    bool setReturnLevel(unsigned int level);
    bool setOperatingMode(unsigned int mode);
    bool setShutdown(unsigned int mode);
    void readFromGazebo(int option = READ_ALL, bool publish = true);
    void readFromRobot(int option = READ_ALL, bool publish = true);






};



#endif // HUMANOIDINTERFACE_H


