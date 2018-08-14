#include "humanoid_control/HumanoidCtrlNode.h"


HumanoidCtrlNode::HumanoidCtrlNode(ros::NodeHandle nh_, ros::NodeHandle nh_private_) : nh(nh_) , nh_private(nh_private_)
{
    //Parameter Server
    if (!nh_private.getParam ("dt", dt))
        dt =  0.07; //s


    //Publisher Stuff
    fftTopic     = nh.advertise<humanoid_msgs::FFTMsg>("humanoid_control/fftIMU", 1000);
    motorPIDTopic = nh.advertise<humanoid_msgs::MotorPIDMsg>("humanoid_interface/set_motorPID", 1000);
    imuTopic      = nh.advertise<sensor_msgs::Imu >("humanoid_control/imu_euler", 1000);
    controlTopic  = nh.advertise<humanoid_msgs::HumanoidControlMsg>("humanoid_control/cmd", 1000);

    //Subscriber Stuffs

    imuSubPtr.reset(new ImuSub(nh,"imu/data",1));
    imuSubPtr->registerCallback(&HumanoidCtrlNode::imuCallback, this);

    torqueSubPtr.reset(new TorqueSub(nh,"ft_sensor_rleg3",1));
    torqueSubPtr->registerCallback(&HumanoidCtrlNode::torqueCallback, this);

    lipSubPtr.reset(new LipSub(nh,"humanoid_walking/lipFeedback",1));
    lipSubPtr->registerCallback(&HumanoidCtrlNode::lipCallback, this);

    humanoidPropsSubPtr.reset(new HumanoidPropsSub(nh,"humanoid_model/humanoid_properties",1));
    humanoidPropsSubPtr->registerCallback(&HumanoidCtrlNode::humanoidPropsCallback, this);

    motorPIDSubPtr.reset(new MotorPIDSub(nh,"humanoid_interface/get_motorPID",1));
    motorPIDSubPtr->registerCallback(&HumanoidCtrlNode::motorPIDCallback, this);


    updateParams = false;

    runTimer = nh.createTimer(ros::Duration(dt), &HumanoidCtrlNode::runCallBack,this);

    //Dynamic Reconfigure Server
    config_server.reset(new HumanoidCtrlConfigServer(nh_private));
    HumanoidCtrlConfigServer::CallbackType f = boost::bind(&HumanoidCtrlNode::reconfigCallback, this, _1, _2);
    config_server->setCallback(f);/**/


    //Service Stuff
    mapCli       = nh.serviceClient<humanoid_msgs::LoadMapConfigsSrv>("humanoid_loadmap/load");
    interfaceCli = nh.serviceClient<humanoid_msgs::InterfaceSrv>("humanoid_interface/cmd");
    loadMap();
    kpUpperBodyUpdate = false;
    kpHipRollUpdate   = false;
    kpHipPitchUpdate  = false;
    kpKneePitchUpdate = false;
    kpFootPitchUpdate = false;
    kpFootRollUpdate  = false;
    ctrl_flag         = false;
    arm_ctrl_flag     = false;
    torso_ctrl_flag   = false;
    phase_ctrl_flag   = false;
    kpUpperBody  = 0;
    kpHipRoll    = 0;
    kpHipPitch   = 0;
    kpKneePitch  = 0;
    kpFootPitch  = 0;
    kpFootRoll   = 0;
    roll         = 0;
    pitch        = 0;
    yaw          = 0;
    inclCtrl     = 0;
    sideIncl     = 0;
    torsoKp      = 0;
    torsoKi      = 0;
    torsoKd      = 0;
    armKp        = 0;
    armKi        = 0;
    armKd        = 0;
    phaseKp      = 0;
    phaseKi      = 0;
    phaseKd      = 0;
    phaseRef     = 0;

    rollDt  = 0;
    rollInt = 0;
    rollOld = 0;
    timeDt  = 0;
    timeOld = 0;
    loadPIDs();



}


HumanoidCtrlNode::~HumanoidCtrlNode()
{

}

void HumanoidCtrlNode::loadMap()
{
    mapSrv.request.update = false;
    if(mapCli.call(mapSrv))
    {
        int h = mapSrv.response.idMap.map.layout.dim[0].size;
        int w = mapSrv.response.idMap.map.layout.dim[1].size;
        std::vector<int> data =  mapSrv.response.idMap.map.data;
        Eigen::Map<Eigen::MatrixXi> mat(data.data(), h, w);
        map.map = mat;
        for(int i = 0; i < int(mapSrv.response.idMap.jNames.size());i++)
        {
            map.enumMap.insert(std::make_pair(mapSrv.response.idMap.jNames[i],i));
        }
        robotDOF = mapSrv.response.idMap.robotDOF;
        urdfDOF  = mapSrv.response.idMap.urdfDOF;
        ikDOF    = mapSrv.response.idMap.ikDOF;
        //map.print();
        ROS_INFO("[HUMANOID_CONTROL] ROBOT_DOF: %d  URDF_DOF: %d  IK_DOF: %d",robotDOF,urdfDOF,ikDOF);
    }

}

void HumanoidCtrlNode::loadPIDs()
{
    /*interfaceSrv.request.getPID = true;
    if(interfaceCli.call(interfaceSrv))
    {
        currentMotorPID = interfaceSrv.response.motorPID;
    }*/
    for(int i = 0; i < robotDOF;i++)
    {
        currentMotorPID.kp.push_back(850);
        currentMotorPID.kd.push_back(0);
        currentMotorPID.ki.push_back(0);
    }
    //if(checkPIDConfig()) updatePIDconfigs();
}


bool HumanoidCtrlNode::checkPIDConfig()
{
    /*if(currentMotorPID.kp.size() == robotDOF) ///<------------ CONSERTAR
    {
        kpAll      = currentMotorPID.kp[map.map(RARM0,ROBOT_IDS)];
        kpLegAll   = currentMotorPID.kp[map.map(RLEG0,ROBOT_IDS)];
        kpLegRoll  = currentMotorPID.kp[map.map(RLEG1,ROBOT_IDS)];
        kpLegPitch = currentMotorPID.kp[map.map(RLEG3,ROBOT_IDS)];
        kpFoot     = currentMotorPID.kp[map.map(RFOOT,ROBOT_IDS)];
        ROS_INFO("CURRENT_PID_CONFIG: ");
        ROS_INFO("KP_ALL: %4.3f  KP_LEG_ALL: %4.3f  KP_LEG_ROLL: %4.3f  KP_LEG_PITCH: %4.3f  KP_FOOT: %4.3f",kpAll,kpLegAll,kpLegRoll,kpLegPitch,kpFoot  );
        return true;
    }
    else
    {
        ROS_WARN("It is not possible to determine the PID parameters");
        return false;
    }*/
}

void HumanoidCtrlNode::updatePIDconfigs()
{
    /*conf.doubles.clear();
    double_param.name = "kpAll";
    double_param.value = kpAll;
    conf.doubles.push_back(double_param);
    double_param.name = "kpLegAll";
    double_param.value = kpLegAll;
    conf.doubles.push_back(double_param);
    double_param.name = "kpLegRoll";
    double_param.value = kpLegRoll;
    conf.doubles.push_back(double_param);
    double_param.name = "kpLegPitch";
    double_param.value = kpLegPitch;
    conf.doubles.push_back(double_param);
    srv_req.config = conf;
    std::cout << "ANTES" << std::endl;
    //ros::service::call("set_parameters", srv_req, srv_resp);
    std::cout << "DEPOIS" << std::endl;*/
}


void HumanoidCtrlNode::reconfigCallback(humanoid_msgs::HumanoidCtrlConfig& config, uint32_t level)
{
    boost::mutex::scoped_lock lock(mutex);
    dt           =  config.dt;
    runTimer.setPeriod(ros::Duration(dt));
    if(kpUpperBody !=  config.kpUpperBody)
    {
        kpUpperBody =  config.kpUpperBody;
        kpUpperBodyUpdate = true;
    }
    if(kpHipRoll !=  config.kpHipRoll)
    {
        kpHipRoll =  config.kpHipRoll;
        kpHipRollUpdate = true;
    }
    if(kpHipPitch != config.kpHipPitch)
    {
        kpHipPitch =  config.kpHipPitch;
        kpHipPitchUpdate = true;
    }

    if(kpKneePitch != config.kpKneePitch)
    {
        kpKneePitch =  config.kpKneePitch;
        kpKneePitchUpdate = true;
    }

    if(kpFootPitch     !=  config.kpFootPitch)
    {
        kpFootPitch =  config.kpFootPitch;
        kpFootPitchUpdate = true;
    }
    if(kpFootRoll     !=  config.kpFootRoll)
    {
        kpFootRoll =  config.kpFootRoll;
        kpFootRollUpdate = true;
    }

    torsoKp  = config.torsoKp;
    torsoKi  = config.torsoKi;
    torsoKd  = config.torsoKd;
    phaseKp  = config.phaseKp;
    phaseKi  = config.phaseKi;
    phaseKd  = config.phaseKd;
    phaseRef = config.phaseRef;

    armKp  = config.armKp;
    armKi  = config.armKi;
    armKd  = config.armKd;
    ctrl_flag        =  config.ctrl_flag;
    arm_ctrl_flag    =  config.arm_ctrl_flag;
    torso_ctrl_flag  =  config.torso_ctrl_flag;
    phase_ctrl_flag  =  config.phase_ctrl_flag;
    slope_ctrl_flag  =  config.slope_ctrl_flag;
    foot_ctrl_flag   =  config.foot_ctrl_flag;

    controlMsg.ctrl_flag = ctrl_flag;
    controlMsg.torso_ctrl_flag = torso_ctrl_flag;
    controlMsg.arm_ctrl_flag = arm_ctrl_flag;
    controlMsg.phase_ctrl_flag = phase_ctrl_flag;
    controlMsg.foot_ctrl_flag = foot_ctrl_flag;

    ROS_INFO("CONFIG_PID_CONFIG: ");
    ROS_INFO("KP_UPPER_BODY: %4.3f  KP_HIP_ROLL: %4.3f  KP_HIP_PITCH: %4.3f  KP_KNEE_PITCH: %4.3f  KP_FOOT_PITCH: %4.3f  KP_FOOT_ROLL: %4.3f",kpUpperBody,kpHipRoll,kpHipPitch,kpKneePitch,kpFootPitch,kpFootRoll  );
    if(torso_ctrl_flag) ROS_INFO("TORSO_KP: %4.3f  TORSO_KI: %4.3f  TORSO_KD: %4.3f",torsoKp,torsoKi,torsoKd);
    if(arm_ctrl_flag)   ROS_INFO("ARM_KP: %4.3f  ARM_KI: %4.3f  ARM_KD: %4.3f",armKp,armKi,armKd);
    if(phase_ctrl_flag)   ROS_INFO("PHASE_KP: %4.3f  PHASE_KI: %4.3f  PHASE_KD: %4.3f",phaseKp,phaseKi,phaseKd);
    if(slope_ctrl_flag) ROS_INFO("SLOPE_KP: %4.3f  SLOPE_KI: %4.3f  SLOPE_KD: %4.3f",torsoKp,torsoKi,torsoKd);
    if(foot_ctrl_flag) ROS_INFO("FOOT_KP: %4.3f  FOOT_KI: %4.3f  FOOT_KD: %4.3f",torsoKp,torsoKi,torsoKd);

    updateParams = true;
}


void HumanoidCtrlNode::sendPID()
{
    int nSize = int(currentMotorPID.kp.size());
    if(nSize > 0)
    {
        motorPIDMsg = currentMotorPID;
        if(kpUpperBodyUpdate)
        {
            motorPIDMsg.kp[map.map(RARM0,ROBOT_IDS)] = kpUpperBody;
            motorPIDMsg.kp[map.map(RARM1,ROBOT_IDS)] = kpUpperBody;
            motorPIDMsg.kp[map.map(RARM2,ROBOT_IDS)] = kpUpperBody;

            motorPIDMsg.kp[map.map(LARM0,ROBOT_IDS)] = kpUpperBody;
            motorPIDMsg.kp[map.map(LARM1,ROBOT_IDS)] = kpUpperBody;
            motorPIDMsg.kp[map.map(LARM2,ROBOT_IDS)] = kpUpperBody;

            motorPIDMsg.kp[map.map(RLEG0,ROBOT_IDS)] = kpUpperBody;
            motorPIDMsg.kp[map.map(LLEG0,ROBOT_IDS)] = kpUpperBody;
            kpUpperBodyUpdate = false;
        }
        if(kpHipRollUpdate)
        {
            motorPIDMsg.kp[map.map(LLEG1,ROBOT_IDS)] = kpHipRoll;
            motorPIDMsg.kp[map.map(RLEG1,ROBOT_IDS)] = kpHipRoll;
            kpHipRollUpdate = false;
        }
        if(kpHipPitchUpdate)
        {
            motorPIDMsg.kp[map.map(LLEG2,ROBOT_IDS)] = kpHipPitch;
            motorPIDMsg.kp[map.map(RLEG2,ROBOT_IDS)] = kpHipPitch;
            kpHipPitchUpdate = false;
        }
        if(kpKneePitchUpdate)
        {
            motorPIDMsg.kp[map.map(LLEG3,ROBOT_IDS)] = kpKneePitch;
            motorPIDMsg.kp[map.map(RLEG3,ROBOT_IDS)] = kpKneePitch;
            kpKneePitchUpdate = false;
        }
        if(kpFootPitchUpdate)
        {
            motorPIDMsg.kp[map.map(LLEG4,ROBOT_IDS)] = kpFootPitch;
            motorPIDMsg.kp[map.map(RLEG4,ROBOT_IDS)] = kpFootPitch;
            kpFootPitchUpdate = false;
        }
        if(kpFootRollUpdate)
        {
            motorPIDMsg.kp[map.map(LFOOT,ROBOT_IDS)] = kpFootRoll;
            motorPIDMsg.kp[map.map(RFOOT,ROBOT_IDS)] = kpFootRoll;
            kpFootRollUpdate = false;
        }

        ROS_INFO("SET_PID_CONFIG: ");
        ROS_INFO("KP_UPPER_BODY: %4.3f  KP_HIP_ROLL: %4.3f  KP_HIP_PITCH: %4.3f  KP_KNEE_PITCH: %4.3f  KP_FOOT_PITCH: %4.3f  KP_FOOT_ROLL: %4.3f",kpUpperBody,kpHipRoll,kpHipPitch,kpKneePitch,kpFootPitch,kpFootRoll  );
        motorPIDTopic.publish(motorPIDMsg);
    }
    else
    {
        ROS_WARN("CURRENT PID VALUES WERE NOT UPDATED");
    }
}

void HumanoidCtrlNode::runCallBack(const ros::TimerEvent&)
{
    if(updateParams)
    {
        sendPID();
        updateParams = false;
    }
    if(ctrl_flag)
    {
        controlMsg.header.stamp = ros::Time::now();
        /*if(torso_ctrl_flag)
        {
            controlMsg.torsoAddY = -torsoKp*(roll - (incl-14.)) - torsoKi*rollDt - torsoKi*rollInt;
        }
        else
        {
            rollInt = 0;
        }
        if(arm_ctrl_flag)
        {
            controlMsg.armAdd    = +armKp*(roll - (incl-14.)) + armKi*rollDt + armKi*rollInt;
        }
        else
        {
            rollInt = 0;
        }
        if(phase_ctrl_flag)
        {
            double phaseDiff = MathUtils::absf(pitch - phaseRef);
            if(phaseDiff > 0)  controlMsg.phaseFac    = 1 + phaseKp*(phaseDiff/phaseRef);
            else               controlMsg.phaseFac    = 1;
        }*/


        if(torso_ctrl_flag)
        {
            controlMsg.torsoAddY = torsoKp*(roll + (incl))  - torsoKi*rollDt - torsoKi*rollInt;;
        }
        if(slope_ctrl_flag || foot_ctrl_flag)
        {
            controlMsg.slope = -torsoKp*(roll+ incl) - torsoKi*rollDt - torsoKi*rollInt;
        }
        else
        {
            controlMsg.slope = 0;
        }
        if(arm_ctrl_flag)
        {
            controlMsg.armAdd    = armKp*(roll + (incl))  - armKi*rollDt - armKi*rollInt;

        }
        controlTopic.publish(controlMsg);
    }
    else
    {
        rollInt = 0;
    }
}



void HumanoidCtrlNode::send2Topic()
{

}

void HumanoidCtrlNode::imuCallback(const sensor_msgs::ImuPtr &msg)
{

    double time = (double)msg->header.stamp.sec + ((double)msg->header.stamp.nsec)/1000000000;
    timeDt  = time - timeOld;
    timeOld = time;
    double q0 = (double)msg->orientation.w;
    double q1 = (double)msg->orientation.x;
    double q2 = (double)msg->orientation.y;
    double q3 = (double)msg->orientation.z;
    MathUtils::quatern2euler(q0,-q1,-q2,-q3,roll,pitch,yaw);


    rollDt  = (roll - rollOld)/timeDt;
    rollOld = roll;
    rollInt += (roll - rollOld)/timeDt;
    imuMsg.header = msg->header;
    imuMsg.orientation.x = roll;
    imuMsg.orientation.y = pitch;
    imuMsg.orientation.z = yaw;

    imuMsg.linear_acceleration = msg->linear_acceleration;
    imuMsg.angular_velocity    = msg->angular_velocity;

    imuTopic.publish(imuMsg);




}


void HumanoidCtrlNode::motorPIDCallback(const humanoid_msgs::MotorPIDMsgPtr &msg)
{
    this->currentMotorPID    = *msg;
}
void HumanoidCtrlNode::torqueCallback(const geometry_msgs::WrenchStampedPtr &msg)
{


}
void HumanoidCtrlNode::lipCallback(const humanoid_msgs::LipFeedBackPtr &msg)
{

}
void HumanoidCtrlNode::humanoidPropsCallback(const humanoid_msgs::HumanoidPropertiesMsgPtr &humanoidPropsMsg)
{
    incl     = humanoidPropsMsg->incl;
    sideIncl = humanoidPropsMsg->sideIncl;


}


void HumanoidCtrlNode::callback(const ImuMsg::ConstPtr&    imu_msg,
                                const TorqueMsg::ConstPtr& torque_msg,
                                const LipMsg::ConstPtr&    lip_msg)
{

}
