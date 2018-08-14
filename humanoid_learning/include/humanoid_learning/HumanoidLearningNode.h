#ifndef HUMANOIDLEARNINGNODE_H
#define HUMANOIDLEARNINGNODE_H


#include <iostream>
#include <fstream>
#include <string>
#include <unistd.h>
#include "humanoid_general/State.h"
#include "humanoid_general/Enums.h"
#include "humanoid_general/MathUtils.h"
#include "humanoid_general/spline.h"
#include "humanoid_loadmap/Mapping.h"
#include "eigen_conversions/eigen_msg.h"

#include "ros/ros.h"

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <humanoid_msgs/HumanoidControlMsg.h>
#include <humanoid_msgs/LearningMsg.h>
#include <humanoid_msgs/PerformanceMsg.h>
#include <humanoid_msgs/ImpactMsg.h>
#include <humanoid_msgs/HumanoidStateMsg.h>
#include <humanoid_msgs/HumanoidPropertiesMsg.h>
#include <sensor_msgs/Imu.h>
#include <humanoid_msgs/LipParamsMsg.h>
#include <humanoid_msgs/LipParamsSrv.h>
#include <humanoid_msgs/JointStateMsg.h>
#include <humanoid_msgs/LipFeedBack.h>
#include <humanoid_msgs/LoadMapConfigsSrv.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/WrenchStamped.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>


#include <dynamic_reconfigure/server.h>
#include <humanoid_msgs/HumanoidLearningConfig.h>



class HumanoidLearningNode
{
public:

    //ROS Node
    ros::NodeHandle    nh;
    ros::NodeHandle    nh_private;

    //ROS Subcriber
    typedef humanoid_msgs::JointStateMsg                JointStateMsg;
    typedef message_filters::Subscriber<JointStateMsg>  JointStateSub;
    boost::shared_ptr<JointStateSub>                    jointStateSubPtr;
    JointStateMsg  jointStateMsg;

    typedef sensor_msgs::Imu                            ImuMsg;
    typedef std::valarray<ImuMsg>                       ImuArrayMsg;
    typedef message_filters::Subscriber<ImuMsg>         ImuSub;
    boost::shared_ptr<ImuSub>                           imuEulerSubPtr;
    sensor_msgs::Imu                                    imuMsg;
    ImuArrayMsg                                         imuArrayMsg;
    ImuArrayMsg                                         imuArrayBatchMsg;

    CArray   imuEulerMsgXData;
    CArray   imuEulerMsgYData;
    CArray   imuEulerMsgZData;

    CArray   imuEulerMsgXFreq;
    CArray   imuEulerMsgYFreq;
    CArray   imuEulerMsgZFreq;

    CArray   rForceData;
    CArray   lForceData;

    CArray   rForceFreq;
    CArray   lForceFreq;

    CArray   r3TorqueData;
    CArray   l3TorqueData;
    CArray   r3TorqueFreq;
    CArray   l3TorqueFreq;

    CArray   r4TorqueData;
    CArray   l4TorqueData;
    CArray   r4TorqueFreq;
    CArray   l4TorqueFreq;

    CArray   rfTorqueData;
    CArray   lfTorqueData;
    CArray   rfTorqueFreq;
    CArray   lfTorqueFreq;

    typedef humanoid_msgs::LipParamsMsg                       LipParamsMsg;
    typedef message_filters::Subscriber<LipParamsMsg>         LipParamsSub;
    boost::shared_ptr<LipParamsSub>                           lipParamsSubPtr;
    LipParamsMsg  lipParamsMsg;


    typedef humanoid_msgs::LipFeedBack                        LipFeedBackMsg;
    typedef message_filters::Subscriber<LipFeedBackMsg>       LipFeedBackSub;
    boost::shared_ptr<LipFeedBackSub>                         lipFeedBackSubPtr;
    LipFeedBackMsg  lipFeedbackMsg;
    int             lastFootGround;
    int             footGroundChanged;
    bool            rFootOnGround;
    bool            lFootOnGround;

    typedef humanoid_msgs::HumanoidPropertiesMsg              HumanoidPropsMsg;
    typedef message_filters::Subscriber<HumanoidPropsMsg>     HumanoidPropsSub;
    boost::shared_ptr<HumanoidPropsSub>                       humanoidPropsSubPtr;
    HumanoidPropsMsg humanoidPropsMsg;

    typedef geometry_msgs::WrenchStamped                      ForceMsg;
    typedef message_filters::Subscriber<ForceMsg>             ForceSub;
    boost::shared_ptr<ForceSub>                               forceRSubPtr;
    boost::shared_ptr<ForceSub>                               forceLSubPtr;
    boost::shared_ptr<ForceSub>                               torqueR3SubPtr;
    boost::shared_ptr<ForceSub>                               torqueR4SubPtr;
    boost::shared_ptr<ForceSub>                               torqueRFSubPtr;
    boost::shared_ptr<ForceSub>                               torqueL3SubPtr;
    boost::shared_ptr<ForceSub>                               torqueL4SubPtr;
    boost::shared_ptr<ForceSub>                               torqueLFSubPtr;


    typedef message_filters::sync_policies::ApproximateTime<ForceMsg, ForceMsg,ForceMsg,ForceMsg,ForceMsg,ForceMsg> TorqueSyncPolicy;
    typedef message_filters::Synchronizer<TorqueSyncPolicy> TorqueSync;
    boost::shared_ptr<TorqueSync> torqueSync;

    //ROS Publishers
    ros::Publisher                     controlTopic;
    humanoid_msgs::HumanoidControlMsg  controlMsg;
    ros::Publisher                     learningTopic;
    humanoid_msgs::LearningMsg         learningMsg;
    std::vector<humanoid_msgs::LearningMsg>         learningVec;
    ros::Publisher                     impactTopic;
    humanoid_msgs::ImpactMsg           impactMsg;


    spline    rFootFacPosX;
    spline    rFootFacPosY;
    spline    rFootFacPosZ;

    spline    lFootFacPosX;
    spline    lFootFacPosY;
    spline    lFootFacPosZ;

    spline    rFootFacRotX;
    spline    rFootFacRotY;
    spline    rFootFacRotZ;

    spline    lFootFacRotX;
    spline    lFootFacRotY;
    spline    lFootFacRotZ;

    bool controlUpdateFlag;

    //std::vector<humanoid_msgs::EndEffMsg>           rFootFac;
    //std::vector<humanoid_msgs::EndEffMsg>           lFootFac;


    //ROS Dynamic Reconfigure Server
    typedef humanoid_msgs::HumanoidLearningConfig                 HumanoidLearningConfig;
    typedef dynamic_reconfigure::Server<HumanoidLearningConfig>   HumanoidLearningConfigServer;
    boost::shared_ptr<HumanoidLearningConfigServer>               config_server;
    boost::mutex mutex;



    //ROS  Services
    ros::ServiceClient                 mapCli;
    humanoid_msgs::LoadMapConfigsSrv   mapMsg;

    ros::ServiceClient                 lipCli;
    humanoid_msgs::LipParamsSrv        lipMsg;

    ros::ServiceServer resetSrv;
    //Timers
    ros::Timer runTimer;

    //Members
    Mapping map;
    int     robotDOF;
    int     urdfDOF;
    int     ikDOF;
    double  dt;
    double  df;
    double  testTime;
    int     wPoints;
    int     wUpdate;
    int     fUpdate;
    int     fPoints;
    int     tUpdate;
    int     tPoints;
    int     testPoint;
    int     imuCount;
    bool    imuInit;
    int     forceCount;
    bool    forceInit;
    int     torqueCount;
    bool    torqueInit;
    bool    lipParamsUpdate;
    bool    test;
    int     testNum;

    ros::Time  testBegin;
    ros::Time  testFinal;
    int        countControl;
    bool       testBeginFlag;

    double maxXAmp;
    double maxYAmp;
    double maxZAmp;
    double maxXFreq;
    double maxYFreq;
    double maxZFreq;

    double meanX;
    double meanY;
    double meanZ;


    HumanoidLearningNode(ros::NodeHandle nh_,ros::NodeHandle nh_private_);
    ~HumanoidLearningNode();

    //Methods
    void loadMap();
    void calcFeatures(CArray &sample,double &amp,double &freq,double &mean,double minFreq = 0.5,double maxFreq = 8.);
    int  calcWindowPoints(double totalDt,double dt);
    void calcFreqRefs(double tS,double tD,double delayAll);
    void resizeData(int nIMUSamples, int nForceSamples, int nTorqueSamples);
    void getBiggerFreq(CArray &fft, double &max, double &maxIndex, double freqLower, double freqUpper);
    void calcPerformance();
    void sendControl();
    void calcControl();

    void buildLearningMsg();
    //Callbacks
    void runCallBack(const ros::TimerEvent&);
    void imuEulerCallback(const sensor_msgs::ImuPtr &msg);
    void lipParamsCallback(const humanoid_msgs::LipParamsMsgPtr &msg);
    void humanoidPropsCallback(const humanoid_msgs::HumanoidPropertiesMsgPtr &msg);
    void jointStateCallback(const humanoid_msgs::JointStateMsgPtr &msg);
    void lipFeedBackCallback(const humanoid_msgs::LipFeedBackPtr &msg);
    void forceRCallback(const geometry_msgs::WrenchStampedPtr &msg);
    void forceLCallback(const geometry_msgs::WrenchStampedPtr &msg);

    void torqueCallback(const ForceMsg::ConstPtr &r3, const ForceMsg::ConstPtr &r4, const ForceMsg::ConstPtr &rf,
                        const ForceMsg::ConstPtr &l3, const ForceMsg::ConstPtr &l4, const ForceMsg::ConstPtr &lf);

    void reconfigCallback(humanoid_msgs::HumanoidLearningConfig& config, uint32_t level);
    bool reset(std_srvs::Empty::Request  &cmd,
               std_srvs::Empty::Response &res);

    void send2Topic();

};

#endif // HUMANOIDLEARNINGNODE_H

