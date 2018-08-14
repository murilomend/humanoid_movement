#include "humanoid_walking/LipWalkNode.h"




LipWalkNode::LipWalkNode(ros::NodeHandle nh_,ros::NodeHandle nh_private_,ros::NodeHandle nh_private_params_,ros::NodeHandle nh_private_cmd_,ros::NodeHandle nh_private_ctrl_) : nh(nh_) , nh_private(nh_private_), nh_private_params(nh_private_params_), nh_private_cmd(nh_private_cmd_), nh_private_ctrl(nh_private_ctrl_)
{
    //Parameter Server
    if (!nh_private_params.getParam("tS", tS))
      tS = 0.3; // 0.8 s
    if (!nh_private_params.getParam ("tD", tD))
      tD = 0.1; // 0.0 s
    if (!nh_private_params.getParam ("zCCorr", zCCorr))
      zCCorr =  1;
    if (!nh_private_params.getParam ("stepH", stepH))
      stepH =  0.05; //0.05 m
    if (!nh_private_ctrl.getParam ("delayR", delayR))
      delayR =  1; // 1
    if (!nh_private_ctrl.getParam ("delayL", delayL))
      delayL =  1; // 1
    if (!nh_private_ctrl.getParam ("delayAll", delayAll))
      delayAll =  1; // 1
    if (!nh_private_ctrl.getParam ("timeFac", timeFac))
      timeFac =  1; // 1
    if (!nh_private_params.getParam ("dt", dt))
      dt =  0.07; // 0.07 s
    if (!nh_private_params.getParam ("lip_dt", lip_dt))
      lip_dt =  0.005; // 0.005 s
    if (!nh_private_cmd.getParam ("vx", vx))
      vx =  0; // 0 m/s
    if (!nh_private_cmd.getParam ("vy", vy))
      vy =  0; // 0 m/s
    if (!nh_private_cmd.getParam ("vz", vz))
      vz =  0; // 0 deg/s
    if (!nh_private_cmd.getParam ("walk_flag", walk_flag))
        walk_flag =  false; // false
    if (!nh_private_params.getParam ("curve", curve))
      curve =  SINE_2;

    walking_flag = walk_flag;
    delayAllChanged = false;
    delayRLChanged  = false;


    //Publisher Stuff
    lipTopic       = nh.advertise<humanoid_msgs::LipFeedBack>("humanoid_walking/lipFeedback", 1000);
    paramsTopic    = nh.advertise<humanoid_msgs::LipParamsMsg>("humanoid_walking/walking_params_state", 1000);
    endEffTopic    = nh.advertise<humanoid_msgs::EndEffStateMsg>("humanoid_model/endEffState", 1000);

    //Subscriber Stuff
    ctrlSubPtr.reset(new ctrlSub(nh,"humanoid_control/cmd",1));
    ctrlSubPtr->registerCallback(&LipWalkNode::ctrlCallback, this);

    humanoidPropsSubPtr.reset(new HumanoidPropsSub(nh,"humanoid_model/humanoid_properties",1));
    humanoidPropsSubPtr->registerCallback(&LipWalkNode::humanoidPropsCallback, this);

    walkingSubPtr.reset(new WalkingSub(nh,"humanoid_walking/cmd",1));
    walkingSubPtr->registerCallback(&LipWalkNode::walkingCallback, this);

    //Service Stuff
    paramsSrv = nh.advertiseService("humanoid_walking/walking_params",&LipWalkNode::updateWalkingParams,this);
    cmdSrv    = nh.advertiseService("humanoid_walking/walking_cmd",&LipWalkNode::walkingCmd,this);
    humanoidPropsCli = nh.serviceClient<humanoid_msgs::LoadHumanoidPropertiesSrv>("humanoid_model/properties");

    //Dynamic Reconfigure Server
    config_server_params.reset(new HumanoidLipWalkingParamsConfigServer(nh_private_params));
    HumanoidLipWalkingParamsConfigServer::CallbackType f_params = boost::bind(&LipWalkNode::reconfigParamsCallback, this, _1, _2);
    config_server_params->setCallback(f_params);


    config_server_cmd.reset(new HumanoidLipWalkingCmdConfigServer(nh_private_cmd));
    HumanoidLipWalkingCmdConfigServer::CallbackType f_cmd = boost::bind(&LipWalkNode::reconfigCmdCallback, this, _1, _2);
    config_server_cmd->setCallback(f_cmd);


    config_server_ctrl.reset(new HumanoidLipWalkingCtrlConfigServer(nh_private_ctrl));
    HumanoidLipWalkingCtrlConfigServer::CallbackType f_ctrl = boost::bind(&LipWalkNode::reconfigCtrlCallback, this, _1, _2);
    config_server_ctrl->setCallback(f_ctrl);


    footCom   = Eigen::Vector3d(0,0,0);
    refCom    = Eigen::Vector3d(0,0,0);
    sinFreq   = 1;
    sinAmp    = 0;
    sinPhase  = 0;
    tSin      = 0;
    nSin      = 0;
    tNormSin  = 0;
    startedLip= false;



    lipUpdate = true;

    configHumanoidNode(true);
    initWalkingSettings();
    initPosition();

    if(delayR != 1 || delayL != 1)    delayRLChanged = true;
    if(delayAll != 1)                 delayAllChanged = true;


    //Run Callback
    timerLip  = nh.createTimer(ros::Duration(lip_dt), &LipWalkNode::runCallBackLip,this);
    timerSend = nh.createTimer(ros::Duration(dt), &LipWalkNode::runCallBackSend,this);

}
LipWalkNode::~LipWalkNode()
{

}


void LipWalkNode::initWalkingSettings()
{
    //void pointMsgToEigen(const geometry_msgs::Point &m, Eigen::Vector3d &e);
    //void pointEigenToMsg(const Eigen::Vector3d &e, geometry_msgs::Point &m);
    humanoidPropsMsg.request.calcCOM       = true;
    humanoidPropsMsg.request.setComAsIkRef = true;
    humanoidPropsCli.call(humanoidPropsMsg);

    tf::pointMsgToEigen(humanoidPropsMsg.response.footComPoint, footCom);


    double zC         = MathUtils::absf(footCom(2));
    double legDist    = MathUtils::absf(footCom(1))*2;
    double xOffset    = MathUtils::absf(footCom(0));


    robotLip = LipWalk(tS,tD,zC,legDist,xOffset,stepH,zCCorr,lip_dt,curve);


    rFoot = BodyPointState(RLEG,BP_TOUCHING);
    lFoot = BodyPointState(LLEG,BP_TOUCHING);
    trunk = BodyPointState(TRUNK,BP_FREE);

    vecBody.resize(3);
    robotLip.joystick(0,0,0);
    //Stabilize LIP
    robotLip.stabilize();
    tNormSin = robotLip.tNormAll;
    ROS_INFO("[LIP_WALK] Walking Parameters Initialized");
    ROS_INFO("[LIP_WALK] TS: %4.3f",tS);
    ROS_INFO("[LIP_WALK] TD: %4.3f",tD);
    ROS_INFO("[LIP_WALK] zC: %4.3f",zC);
    ROS_INFO("[LIP_WALK] zCCorr: %4.3f",zCCorr);
    ROS_INFO("[LIP_WALK] legDist: %4.3f",legDist);
    ROS_INFO("[LIP_WALK] xOffset: %4.3f",xOffset);

    ROS_INFO("[LIP_WALK] DT: %4.3f",lip_dt);
    ROS_INFO("[LIP_WALK] CURVE: %d",curve);

}

void LipWalkNode::initPosition()
{
    robotLip.calcLIP();
    robotLip.updateComFoot();
    lFoot.pos = robotLip.lFoot;
    lFoot.rot = robotLip.lFootTeta;
    rFoot.pos = robotLip.rFoot;
    rFoot.rot = robotLip.rFootTeta;


    vecBody[0] = rFoot;
    vecBody[1] = lFoot;
    vecBody[2] = trunk;
    sendGoal(vecBody,dt);
    send2Topic();
}

void LipWalkNode::configHumanoidNode(bool lipBool)
{
    conf.bools.clear();
    bool_param.name = "calcIK";
    bool_param.value = lipBool;
    conf.bools.push_back(bool_param);
    srv_req.config = conf;
    ros::service::call("/humanoid_model/set_parameters", srv_req, srv_resp);

}

void LipWalkNode::reconfigParamsCallback(humanoid_msgs::HumanoidLipWalkingParamsConfig& config, uint32_t level)
{
    boost::mutex::scoped_lock lock(mutex_params);
    if(tS != config.tS || tD != config.tD || zCCorr != config.zCCorr || stepH  != config.stepH || curve != config.stepH)
    {
        tS     = config.tS;
        tD     = config.tD;
        zCCorr = config.zCCorr;
        stepH  = config.stepH;
        curve  = config.curve;

        lipUpdate = true;
    }
    sinAmp    = config.sinAmp;
    sinFreq   = config.sinFreq;
    sinPhase  = config.sinPhase;
    dt        = config.dt;
    lip_dt    = config.lip_dt;
    timerSend.setPeriod(ros::Duration(dt));
    timerLip.setPeriod(ros::Duration(lip_dt));


}


void LipWalkNode::reconfigCmdCallback(humanoid_msgs::HumanoidLipWalkingCmdConfig& config, uint32_t level)
{
    boost::mutex::scoped_lock lock(mutex_cmd);

    vx  = config.vx;
    vy  = config.vy;
    vz  = config.vz;
    slope     = config.slope;

    walk_flag  = config.walk_flag;
    //walking_flag = walk_flag;
    ROS_INFO("[LIP_WALK] Walking CMDs Updated");
    ROS_INFO("[LIP_WALK] Vx : %4.3f Vy: %4.3f Vz: %4.3f",vx,vy,vz);
}

void LipWalkNode::reconfigCtrlCallback(humanoid_msgs::HumanoidLipWalkingCtrlConfig& config, uint32_t level)
{
    boost::mutex::scoped_lock lock(mutex_ctrl);

    if(delayR != config.delayR || delayL != config.delayL )
    {
        delayR    = config.delayR;
        delayL    = config.delayL;
        delayRLChanged = true;
        delayAllChanged = false;
    }

    if(delayAll != config.delayAll)
    {
        delayAll  = config.delayAll;
        delayAllChanged = true;
        delayRLChanged  = false;
    }


    ROS_INFO("[LIP_WALK] Walking CTRL Updated");
    ROS_INFO("[LIP_WALK] delayR : %4.3f delayL: %4.3f delayAll: %4.3f",delayR,delayL,delayAll);

}

void LipWalkNode::runCallBackLip(const ros::TimerEvent&)
{
    if(walking_flag)
    {
        if(walk_flag) robotLip.joystick(vx,vy,vz);
        else          robotLip.joystick(0,0,0);
        robotLip.calcLIP();
        //std::cout << "TNORM: " << robotLip.tNorm << " TNORMALL: " << robotLip.tNormAll << std::endl;

        robotLip.updateComFoot();

        if(humanoidCtrlMsg.phase_ctrl_flag == false)
        {
            humanoidCtrlMsg.phaseFac = 0.;
        }
        if(robotLip.wState == SINGLE_SUPPORT && delayRLChanged)
        {
            if(robotLip.footAir == LEFT_FOOT)   timeFac = delayL;
            if(robotLip.footAir == RIGHT_FOOT)  timeFac = delayR;
            delayAllChanged = false;
            //ROS_WARN("TIMEFAC: %4.3f  PHASEFAC: %4.3f",timeFac,humanoidCtrlMsg.phaseFac);
            timerLip.setPeriod(ros::Duration(timeFac*lip_dt*(0.5*(humanoidCtrlMsg.phaseFac)/2. + 1.0)));
            //send2Topic();

        }
        if(delayAllChanged)
        {
            timeFac = delayAll;
            delayRLChanged = false;
            //ROS_WARN("TIMEFAC: %4.3f  PHASEFAC: %4.3f",timeFac,humanoidCtrlMsg.phaseFac);
            timerLip.setPeriod(ros::Duration(timeFac*lip_dt*(0.5*(humanoidCtrlMsg.phaseFac)/2. + 1.0)));
            //send2Topic();
        }
        if(humanoidCtrlMsg.slope_ctrl_flag)  slope = humanoidCtrlMsg.slope;
        robotLip.floorIncl = (slope/180.)*PI;

    }
    else
    {
        robotLip.stabilize();
        tNormSin = robotLip.tNormAll;
    }
    if(!walk_flag)
    {
        if(robotLip.stabilized()) walking_flag = false;
    }
    else
    {
        walking_flag = true;
    }
    if(lipUpdate)
    {
        double zC         = MathUtils::absf(footCom(2));
        double legDist    = MathUtils::absf(footCom(1))*2;
        double xOffset    = MathUtils::absf(footCom(0));

        robotLip.updateParams(tS,tD,zC,legDist,xOffset,stepH,zCCorr);
        robotLip.curve = curve;

        robotLip.stabilize();
        tNormSin = robotLip.tNormAll;
        robotLip.updateComFoot();


        ROS_INFO("[LIP_WALK] Walking Parameters Updated");
        ROS_INFO("[LIP_WALK] TS: %4.3f",tS);
        ROS_INFO("[LIP_WALK] TD: %4.3f",tD);
        ROS_INFO("[LIP_WALK] zC: %4.3f",zC);
        ROS_INFO("[LIP_WALK] zCCorr: %4.3f",zCCorr);
        ROS_INFO("[LIP_WALK] legDist: %4.3f",legDist);
        ROS_INFO("[LIP_WALK] xOffset: %4.3f",xOffset);
        ROS_INFO("[LIP_WALK] DT: %4.3f",lip_dt);
        ROS_INFO("[LIP_WALK] CURVE: %d",curve);

        send2Topic();

        lipUpdate  =false;
    }
}



void LipWalkNode::runCallBackSend(const ros::TimerEvent&)
{
    lFoot.pos = robotLip.lFoot;
    lFoot.rot = robotLip.lFootTeta;
    rFoot.pos = robotLip.rFoot;
    rFoot.rot = robotLip.rFootTeta;


    trunk.rot(1) = sin((robotLip.tNormAll + sinPhase)*3.14159*floor(sinFreq))*sinAmp;
    vecBody[2]   = trunk;


    vecBody[0] = rFoot;
    vecBody[1] = lFoot;

    if(!walk_flag)
    {
        /*vecBody[2]   = trunk;
        vecBody[0].pos   = Eigen::Vector3d(rFoot.pos;
        vecBody[1]   = lFoot;*/

    }

    sendGoal(vecBody,dt);
    send2Topic();
}
bool LipWalkNode::updateWalkingParams(humanoid_msgs::LipParamsSrv::Request  &params,
                                      humanoid_msgs::LipParamsSrv::Response &res)
{

    if(params.get_params)
    {
        res.tS       = tS;
        res.tD       = tD;
        res.stepH    = stepH;
        res.zCCorr   = zCCorr;
        res.delayR   = delayR;
        res.delayL   = delayL;
        res.delayAll = delayAll;
        res.vx       = vx;
        res.vy       = vy;
        res.vz       = vz;
    }
    else
    {
        tS     = params.tS;
        tD     = params.tD;
        stepH  = params.stepH;
        zCCorr = params.zCCorr;

        lipUpdate = true;
    }

    return true;
}


bool LipWalkNode::walkingCmd(humanoid_msgs::LipCmdSrv::Request  &cmd,
                             humanoid_msgs::LipCmdSrv::Response &res)
{
    ROS_INFO("[LIP_WALK] Walking CMDs Updated");
    ROS_INFO("[LIP_WALK] Vx : %4.3f Vy: %4.3f Vz: %4.3f",cmd.vx,cmd.vy,cmd.vz);
    vx  = cmd.vx;
    vy  = cmd.vy;
    vz  = cmd.vz;
    walk_flag  = cmd.walk_flag;
    reset_walk = cmd.reset_walk;
    return true;
}


void LipWalkNode::ctrlCallback(const humanoid_msgs::HumanoidControlMsgPtr &ctrl)
{
    humanoidCtrlMsg = *ctrl; 
}

void LipWalkNode::walkingCallback(const humanoid_msgs::WalkingMsgPtr &msg)
{
    vx        = msg->vx;
    vy        = msg->vy;
    vz        = msg->vz;
    walk_flag = msg->go;
    //walking_flag = walk_flag;
}

void LipWalkNode::sendGoal(const std::vector<BodyPointState> &vecBody,const double& dt)
{
    endEffMsg.header.stamp = ros::Time::now();
    endEffMsg.endEff.resize(int(vecBody.size()));
    for(int i = 0; i < int(vecBody.size());i++)
    {
        //ROS_INFO("DEBUG: I: %d  VEC_SIZE: %d",i,int(vecBody.size()));
        tf::pointEigenToMsg(vecBody[i].pos,endEffMsg.endEff[i].pos);
        tf::pointEigenToMsg(vecBody[i].rot,endEffMsg.endEff[i].rot);
        endEffMsg.endEff[i].type = vecBody[i].type;
        endEffMsg.endEff[i].flag = vecBody[i].flag;
        endEffMsg.endEff[i].dt   = vecBody[i].dt;
    }
    endEffTopic.publish(endEffMsg);


}

void LipWalkNode::humanoidPropsCallback(const humanoid_msgs::HumanoidPropertiesMsgPtr &msg)
{

    tf::pointMsgToEigen(msg->footComPoint, footCom);
    lipUpdate = true;

}

void LipWalkNode::send2Topic()
{
    Eigen::Vector3d footPosR  = (robotLip.footGround == RIGHT_FOOT)?(robotLip.rFootPos):(robotLip.lFootPos);
    //Vector3d footPos   = lipEvaSim.fIdealSSP;

    lipMsg.header.stamp = ros::Time::now();
    lipMsg.footStep.x   = robotLip.fIdealSSP(0);
    lipMsg.footStep.y   = robotLip.fIdealSSP(1);
    lipMsg.footStep.z   = robotLip.fIdealSSP(2);

    lipMsg.footStepR.x = footPosR(0);
    lipMsg.footStepR.y = footPosR(1);
    lipMsg.footStepR.z = footPosR(2);

    lipMsg.comPos.x = robotLip.com.pos(0);
    lipMsg.comPos.y = robotLip.com.pos(1);
    lipMsg.comPos.z = robotLip.com.pos(2);

    lipMsg.comVel.x = robotLip.com.vel(0);
    lipMsg.comVel.y = robotLip.com.vel(1);
    lipMsg.comVel.z = robotLip.com.vel(2);

    lipMsg.comAcc.x = robotLip.com.acc(0);
    lipMsg.comAcc.y = robotLip.com.acc(1);
    lipMsg.comAcc.z = robotLip.com.acc(2);

    lipMsg.zmp.x    = robotLip.zmp(0);
    lipMsg.zmp.y    = robotLip.zmp(1);
    lipMsg.zmp.z    = 0;

    lipMsg.footAir     = robotLip.footAir*5;
    lipMsg.footGround  = robotLip.footGround*5;

    lipMsg.wState      = robotLip.wState*5;

    paramsMsg.tS     = tS;
    paramsMsg.tD     = tD;
    paramsMsg.stepH  = stepH;
    paramsMsg.zCCorr = zCCorr;

    paramsMsg.vx     = vx;
    paramsMsg.vy     = vy;
    paramsMsg.vz     = vz;

    paramsMsg.delayL   = delayL;
    paramsMsg.delayR   = delayR;
    paramsMsg.delayAll = delayAll;

    paramsTopic.publish(paramsMsg);
    lipTopic.publish(lipMsg);
}


