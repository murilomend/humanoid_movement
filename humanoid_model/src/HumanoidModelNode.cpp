#include "humanoid_model/HumanoidModelNode.h"


HumanoidModelNode::HumanoidModelNode(ros::NodeHandle nh_, ros::NodeHandle nh_private_,ros::NodeHandle nh_private_walk_,ros::NodeHandle nh_private_movcreator_) : nh(nh_) , nh_private(nh_private_) , nh_private_walk(nh_private_walk_),nh_private_movcreator(nh_private_movcreator_)
{
    if (!nh_private.getParam ("dt", dt))
      dt = 0.07;

    //Publisher Stuff
    jointStateTopic     = nh.advertise<JointStateMsg>("humanoid_model/jointState", 1000);
    endEffStateTopic    = nh.advertise<EndEffStateMsg>("humanoid_model/endEffState", 1000);
    humanoidPropsTopic  = nh.advertise<humanoid_msgs::HumanoidPropertiesMsg>("humanoid_model/humanoid_properties", 1000);



    //Subscriber Stuff
    jointStateSubPtr.reset(new JointStateSub(nh,"humanoid_model/jointState",1));
    jointStateSubPtr->registerCallback(&HumanoidModelNode::jointStateCallback, this);

    endEffStateSubPtr.reset(new EndEffStateSub(nh,"humanoid_model/endEffState",1));
    endEffStateSubPtr->registerCallback(&HumanoidModelNode::endEffStateCallback, this);

    humanoidControlSubPtr.reset(new HumanoidControlSub(nh,"humanoid_control/cmd",1));
    humanoidControlSubPtr->registerCallback(&HumanoidModelNode::humanoidControlCallback, this);

    //Service Stuff
    mapCli    = nh.serviceClient<humanoid_msgs::LoadMapConfigsSrv>("humanoid_loadmap/load");
    humanoidPropsSrv  = nh.advertiseService("humanoid_model/properties",&HumanoidModelNode::humanoidPropertiesSrv,this);



    zmpPoint     = Eigen::Vector3d(0,0,0);
    comPoint     = Eigen::Vector3d(0,0,0);
    footComPoint = Eigen::Vector3d(0,0,0);

    trunk         = BodyPointState(TRUNK);

    loadMap();
    loadRobotParams();

    //Dynamic Reconfigure Server
    config_server.reset(new HumanoidModelConfigServer(nh_private_walk));
    HumanoidModelConfigServer::CallbackType f = boost::bind(&HumanoidModelNode::reconfigCallback, this, _1, _2);
    config_server->setCallback(f);

    config_server_mov.reset(new HumanoidModelMovCreatorConfigServer(nh_private_movcreator));
    HumanoidModelMovCreatorConfigServer::CallbackType f_mov = boost::bind(&HumanoidModelNode::reconfigMovCreatorCallback, this, _1, _2);
    config_server_mov->setCallback(f_mov);

    //Run Callback
    runTimer = nh.createTimer(ros::Duration(dt), &HumanoidModelNode::runCallBack,this);

}

void HumanoidModelNode::runCallBack(const ros::TimerEvent&)
{
    if(modelMovCreatorMsg.on  == true)
    {
        robot.setIKRef(Eigen::Vector3d(0,0,0));
        rLegBP.pos = Eigen::Vector3d(modelMovCreatorMsg.rFootX,-modelMovCreatorMsg.rFootY - robot.poseParams[P_POSE_LEG_OPEN_REF],modelMovCreatorMsg.rFootZ - robot.poseParams[P_POSE_LEG_SQUAT_REF]);
        lLegBP.pos = Eigen::Vector3d(modelMovCreatorMsg.lFootX,modelMovCreatorMsg.lFootY + robot.poseParams[P_POSE_LEG_OPEN_REF],modelMovCreatorMsg.lFootZ - robot.poseParams[P_POSE_LEG_SQUAT_REF]);

        rLegBP.rot = Eigen::Vector3d((modelMovCreatorMsg.rFootRX*PI)/180.,(modelMovCreatorMsg.rFootRY*PI)/180.,(modelMovCreatorMsg.rFootRZ*PI)/180.);
        lLegBP.rot = Eigen::Vector3d((modelMovCreatorMsg.lFootRX*PI)/180.,(modelMovCreatorMsg.lFootRY*PI)/180.,(modelMovCreatorMsg.lFootRZ*PI)/180.);

        std::vector<BodyPointState> bpVec;
        bpVec.push_back(rLegBP);
        bpVec.push_back(lLegBP);
        robot.setEndEffState(bpVec);

        robot.qState.pos[map.map(LARM0,ROBOT_IDS)] = -(modelMovCreatorMsg.lArm0*PI)/180.;
        robot.qState.pos[map.map(LARM1,ROBOT_IDS)] = (modelMovCreatorMsg.lArm1*PI)/180.;
        robot.qState.pos[map.map(LARM2,ROBOT_IDS)] = -(modelMovCreatorMsg.lArm2*PI)/180.;

        robot.qState.pos[map.map(RARM0,ROBOT_IDS)] = (modelMovCreatorMsg.rArm0*PI)/180.;
        robot.qState.pos[map.map(RARM1,ROBOT_IDS)] = -(modelMovCreatorMsg.rArm1*PI)/180.;
        robot.qState.pos[map.map(RARM2,ROBOT_IDS)] = (modelMovCreatorMsg.rArm2*PI)/180.;
        JointState qState;
        qState = robot.qState;
        qState.source = CONST_VEL;
        for(int i = 0; i < int(qState.vel.size());i++)
        {
            qState.vel[i] = 0.3;
        }
        qState.dt     = 3;
        if(modelMovCreatorMsg.toJointState) sendJointState(qState,dt);
        if(modelMovCreatorMsg.toMotorState) sendJointState(qState,dt);
        setRobotFlag  = true;
    }
    else
    {
        if(setRobotFlag)
        {
            robot.comOverride.pos(0) = comX;
            robot.comOverride.pos(1) = comY;
            robot.comOverride.pos(2) = comZ;
            setRobot(squat,open,incl,sideIncl,footIncl);
        }
        if(calcCOM)
        {


        }
        if(calcIK)
        {
            double arm0_ = 0;
            if(humanoidControlMsg.ctrl_flag == true)
            {
                /*if(humanoidControlMsg.torso_ctrl_flag == true)
                {
                    //TRUNK
                    trunk = robot.endEffectors[TRUNK];
                    trunk.rot    = Eigen::Vector3d(trunk.rot(0) + (humanoidControlMsg.torsoAddX/180.)*PI,trunk.rot(1) + (humanoidControlMsg.torsoAddY/180.)*PI,trunk.rot(2) + (humanoidControlMsg.torsoAddZ/180.)*PI);
                    if(trunk.rot(1) > (50./180.)*PI )
                    {
                        ROS_WARN("TRUNK_ROT_Y SATURATED: %4.3f",(trunk.rot(1)/PI)*180.);
                        trunk.rot(1) = (50./180.)*PI;
                    }
                    else if(trunk.rot(1) < (-50./180.)*PI )
                    {
                        ROS_WARN("TRUNK_ROT_Y SATURATED: %4.3f",(trunk.rot(1)/PI)*180.);
                        trunk.rot(1) = (-50./180.)*PI;
                    }
                    vecBody.push_back(trunk);
                }
                else
                {
                    trunk.rot    = Eigen::Vector3d((sideIncl/180.)*PI,(incl/180.)*PI,0);
                    vecBody.push_back(trunk);
                }*/
                if(humanoidControlMsg.arm_ctrl_flag == true)
                {
                    //ARM
                    arm0_ = ((arm0 + humanoidControlMsg.armAdd)/180.)*PI;
                    if(arm0_ > (50./180.)*PI )
                    {
                        ROS_WARN("ARM0 SATURATED: %4.3f",(arm0_/PI)*180.);
                        arm0_ = (50./180.)*PI;
                    }
                    else if(arm0_ < (-50./180.)*PI )
                    {
                        ROS_WARN("ARM0 SATURATED: %4.3f",(arm0_/PI)*180.);
                        arm0_ = (-50./180.)*PI;
                    }
                }
                else
                {
                    arm0_         = (arm0/180.)*PI;
                }

            }
            else
            {
                arm0_         = (arm0/180.)*PI;
            }
            if(!vecBodyBuff.empty())
            {
                if(humanoidControlMsg.foot_ctrl_flag)   footIncl = humanoidControlMsg.slope;
                else                                    footIncl = 0;
                vecBody[0].rot(1) = vecBodyBuff[0].rot(1) + (footIncl/180.)*PI;
                vecBody[1].rot(1) = vecBodyBuff[1].rot(1) + (footIncl/180.)*PI;
                vecBody[2].rot    = (vecBodyBuff[2].rot/180.)*PI + Eigen::Vector3d((sideIncl/180.)*PI,(incl/180.)*PI,0);


                /*double rPosX = 0.05*(humanoidControlMsg.rFootFac.pos.x)/2. + 1.0;
                double rPosY = 0.05*(humanoidControlMsg.rFootFac.pos.y)/2. + 1.0;
                double rPosZ = 0.05*(humanoidControlMsg.rFootFac.pos.z)/2. + 1.0;

                double rRotX = 0.1*(humanoidControlMsg.rFootFac.rot.x)/2.;
                double rRotY = 0.1*(humanoidControlMsg.rFootFac.rot.y)/2.;
                double rRotZ = 0.1*(humanoidControlMsg.rFootFac.rot.z)/2.;

                double lPosX = 0.05*(humanoidControlMsg.lFootFac.pos.x)/2. + 1.0;
                double lPosY = 0.05*(humanoidControlMsg.lFootFac.pos.y)/2. + 1.0;
                double lPosZ = 0.05*(humanoidControlMsg.lFootFac.pos.z)/2. + 1.0;

                double lRotX = 0.1*(humanoidControlMsg.lFootFac.rot.x)/2.;
                double lRotY = 0.1*(humanoidControlMsg.lFootFac.rot.y)/2.;
                double lRotZ = 0.1*(humanoidControlMsg.lFootFac.rot.z)/2.;


                vecBody[0].pos =  Eigen::Vector3d(vecBody[0].pos(0)*rPosX,vecBody[0].pos(1)*rPosY,vecBody[0].pos(2)*rPosZ);
                vecBody[0].rot =  Eigen::Vector3d(rRotX,rRotY,rRotZ);
                vecBody[1].pos =  Eigen::Vector3d(vecBody[1].pos(0)*lPosX,vecBody[1].pos(1)*lPosY,vecBody[1].pos(2)*lPosZ);
                vecBody[1].rot =  Eigen::Vector3d(lRotX,lRotY,lRotZ);*/

                //vecBody[0].pos =  Eigen::Vector3d(vecBody[0].pos(0)*humanoidControlMsg.rFootFac.pos.x,vecBody[0].pos(1)*humanoidControlMsg.rFootFac.pos.y,vecBody[0].pos(2)*humanoidControlMsg.rFootFac.pos.z);
                //vecBody[1].pos =  Eigen::Vector3d(vecBody[1].pos(0)*humanoidControlMsg.lFootFac.pos.x,vecBody[1].pos(1)*humanoidControlMsg.lFootFac.pos.y,vecBody[1].pos(2)*humanoidControlMsg.lFootFac.pos.z);

                robot.setEndEffState(vecBody);
                robot.setArmParams(arm0_,(arm1/180.)*PI,(arm2/180.)*PI);
                sendJointState(robot.qState,dt);
            }
        }
    }
}

void HumanoidModelNode::sendJointState(const JointState &jointState,const double &dt)
{
    jointMsg.header.stamp = ros::Time::now();
    for(int i = 0; i < int(jointState.pos.size());i++)
    {
        jointMsg.pos[i] = jointState.pos(i);
    }
    jointMsg.type   = jointState.type;
    jointMsg.source = HUMANOID_MODEL;
    jointMsg.dt     = jointState.dt;

    jointStateTopic.publish(jointMsg);
}

void HumanoidModelNode::jointStateCallback(const humanoid_msgs::JointStateMsgPtr &jointState)
{
    if(calcFK)
    {


    }
}

void HumanoidModelNode::endEffStateCallback(const humanoid_msgs::EndEffStateMsgPtr &endEffState)
{
    if(calcIK)
    {
        vecBodyBuff.clear();
        vecBodyBuff.resize(endEffState->endEff.size());
        vecBody.clear();
        vecBody.resize(endEffState->endEff.size());
        for(int i = 0; i < int(endEffState->endEff.size());i++)
        {
            tf::pointMsgToEigen(endEffState->endEff[i].pos,vecBodyBuff[i].pos);
            tf::pointMsgToEigen(endEffState->endEff[i].vel,vecBodyBuff[i].vel);
            tf::pointMsgToEigen(endEffState->endEff[i].acc,vecBodyBuff[i].acc);
            tf::pointMsgToEigen(endEffState->endEff[i].rot,vecBodyBuff[i].rot);

            vecBodyBuff[i].type =  endEffState->endEff[i].type;
            vecBodyBuff[i].flag =  endEffState->endEff[i].flag;
            vecBodyBuff[i].dt   =  endEffState->endEff[i].dt;

            vecBody[i] = vecBodyBuff[i];
        }
    }
}

void HumanoidModelNode::humanoidControlCallback(const humanoid_msgs::HumanoidControlMsgPtr &humanoidControl)
{
    this->humanoidControlMsg = *humanoidControl;
}

void HumanoidModelNode::loadRobotParams()
{
    robot = HumanoidModel();
    robot.map = map.map;
    //Parameter Server
    //Static Params
    if (!nh_private.getParam ("P_LEG_D0", robot.legParams[P_LEG_D0]))
      ROS_FATAL("[HUMANOID_MODEL] P_LEG_D0 param not found");
    if (!nh_private.getParam ("P_LEG_D1", robot.legParams[P_LEG_D1]))
      ROS_FATAL("[HUMANOID_MODEL] P_LEG_D1 param not found");
    if (!nh_private.getParam ("P_LEG_D2", robot.legParams[P_LEG_D2]))
      ROS_FATAL("[HUMANOID_MODEL] P_LEG_D2 param not found");
    if (!nh_private.getParam ("P_LEG_D3", robot.legParams[P_LEG_D3]))
      ROS_FATAL("[HUMANOID_MODEL] P_LEG_D3 param not found");
    if (!nh_private.getParam ("P_LEG_D4", robot.legParams[P_LEG_D4]))
      ROS_FATAL("[HUMANOID_MODEL] P_LEG_D4 param not found");

    if (!nh_private.getParam ("P_ARM_D0", robot.armParams[P_ARM_D0]))
      ROS_FATAL("[HUMANOID_MODEL] P_ARM_D0 param not found");
    if (!nh_private.getParam ("P_ARM_D1", robot.armParams[P_ARM_D1]))
      ROS_FATAL("[HUMANOID_MODEL] P_ARM_D1 param not found");
    if (!nh_private.getParam ("P_ARM_D2", robot.armParams[P_ARM_D2]))
      ROS_FATAL("[HUMANOID_MODEL] P_ARM_D2 param not found");

    if (!nh_private.getParam ("P_FOOT_W", robot.footParams[P_FOOT_W]))
      ROS_FATAL("[HUMANOID_MODEL] P_FOOT_W param not found");
    if (!nh_private.getParam ("P_FOOT_H", robot.footParams[P_FOOT_H]))
      ROS_FATAL("[HUMANOID_MODEL] P_FOOT_H param not found");
    if (!nh_private.getParam ("P_FOOT_X", robot.footParams[P_FOOT_X]))
      ROS_FATAL("[HUMANOID_MODEL] P_FOOT_X param not found");
    if (!nh_private.getParam ("P_FOOT_Y", robot.footParams[P_FOOT_Y]))
      ROS_FATAL("[HUMANOID_MODEL] P_FOOT_Y param not found");

    if (!nh_private.getParam ("override_com", override_com))
    {
        ROS_WARN("[HUMANOID_MODEL] override_com param not found. This feature wont be used");
        override_com = false;
    }
    if (!nh_private_walk.getParam ("comX", comX))
    {
        ROS_WARN("[HUMANOID_MODEL] comX param not found. Zero value will be set");
        comX =  0;
    }
    if (!nh_private_walk.getParam ("comY", comY))
    {
        ROS_WARN("[HUMANOID_MODEL] comY param not found. Zero value will be set");
        comY =  0;
    }
    if (!nh_private_walk.getParam ("comZ", comZ))
    {
        ROS_WARN("[HUMANOID_MODEL] comZ param not found. Zero value will be set");
        comZ = 0;
    }
    robot.comOverride.pos(0) = comX;
    robot.comOverride.pos(1) = comY;
    robot.comOverride.pos(2) = comZ;





    //Urdf
    if (!nh.getParam ("fUrdf", fUrdf))
      ROS_FATAL("[HUMANOID_MODEL] Urdf file address not found");
    robot.loadURDF(fUrdf.c_str());

    robot.robotDOF = robotDOF;
    robot.urdfDOF  = urdfDOF;
    robot.ikDOF = ikDOF;

    vecBody.resize(3);
    vecBodyBuff.resize(3);

    rLegBP = BodyPointState(RLEG,BP_TOUCHING);
    lLegBP = BodyPointState(LLEG,BP_TOUCHING);

    jointMsg.pos.resize(robotDOF);
    jointMsg.vel.resize(robotDOF);
    jointMsg.acc.resize(robotDOF);
    jointMsg.torq.resize(robotDOF);


    robot.qState       = JointState(robotDOF,ROBOT_IDS);
    //Dynamic Params
    if (!nh_private_walk.getParam ("squat", squat))
      squat =  0.05;
    if (!nh_private_walk.getParam ("open", open))
      open =  0.02;
    if (!nh_private_walk.getParam ("incl", incl))
      incl =  0;
    if (!nh_private_walk.getParam ("sideIncl", sideIncl))
      sideIncl =  0;
    if (!nh_private_walk.getParam ("footIncl", sideIncl))
      footIncl =  0;
    if (!nh_private_walk.getParam ("arm0", arm0))
      arm0 =  0;
    if (!nh_private_walk.getParam ("arm1", arm1))
      arm1 =  0;
    if (!nh_private_walk.getParam ("arm2", arm2))
      arm2 =  0;

    //Pose properties are derived from other params
    robot.poseParams[P_POSE_LEG_OPEN_REF]  = robot.legParams[P_LEG_D0];
    robot.poseParams[P_POSE_LEG_SQUAT_REF] = robot.legParams[P_LEG_D1] + robot.legParams[P_LEG_D2] + robot.legParams[P_LEG_D3] + robot.legParams[P_LEG_D4];
    robot.poseParams[P_POSE_TORSO_INC_REF] = 0;///<--------------------
    robot.poseParams[P_POSE_SIDE_INC_REF]  = 0;///<--------------------
    setRobot(squat,open,incl,sideIncl,footIncl);
    robot.qState.pos[map.map(LARM0,ROBOT_IDS)] = arm0;
    robot.qState.pos[map.map(LARM1,ROBOT_IDS)] = arm1;
    robot.qState.pos[map.map(LARM2,ROBOT_IDS)] = arm2;

    robot.qState.pos[map.map(RARM0,ROBOT_IDS)] = arm0;
    robot.qState.pos[map.map(RARM1,ROBOT_IDS)] = arm1;
    robot.qState.pos[map.map(RARM2,ROBOT_IDS)] = arm2;
}

bool HumanoidModelNode::humanoidPropertiesSrv(humanoid_msgs::LoadHumanoidPropertiesSrv::Request  &msg,
                                              humanoid_msgs::LoadHumanoidPropertiesSrv::Response &res)
{
    Eigen::Vector3d footCom;
    Eigen::Vector3d refCom;

    if(!msg.joint.pos.empty())  robot.setQState(JointState(msg.joint.pos.size(),msg.joint.type,msg.joint.source));
    if(!msg.endEff.endEff.empty())
    {
        std::vector<BodyPointState>  endEff;
        for(int i = 0; i < int(msg.endEff.endEff.size());i++)  endEff.push_back(BodyPointState(msg.endEff.endEff[i].type,msg.endEff.endEff[i].flag));
        robot.setEndEffState(endEff);
    }
    if(msg.calcCOM)
    {
        robot.getComVecs(footCom,refCom,override_com);
        comPoint     = refCom;
        footComPoint = footCom;
    }
    if(msg.calcZMP)
    {

    }
    if(msg.calcInvDyn)
    {

    }
    if(msg.setIkRef)
    {
        Eigen::Vector3d ikRef;
        tf::pointMsgToEigen(msg.ikRef, ikRef);
        robot.setIKRef(ikRef);
    }
    if(msg.setComAsIkRef)
    {
        robot.setIKRef(refCom);
    }

    tf::pointEigenToMsg(zmpPoint,res.zmpPoint);
    tf::pointEigenToMsg(comPoint,res.comPoint);
    tf::pointEigenToMsg(footComPoint,res.footComPoint);
    for(int i = 0; i < int(torq.size());i++) res.torq.push_back(torq(i));


    humanoidPropsMsg.zmpPoint     = res.zmpPoint;
    humanoidPropsMsg.comPoint     = res.comPoint;
    humanoidPropsMsg.footComPoint = res.footComPoint;
    humanoidPropsMsg.torq         = res.torq;
    humanoidPropsMsg.squat        = squat;
    humanoidPropsMsg.open         = open;
    humanoidPropsMsg.incl         = incl;
    humanoidPropsMsg.sideIncl     = sideIncl;
    humanoidPropsMsg.footIncl     = footIncl;
    sendHumanoidProps(humanoidPropsMsg);

    return true;


}

void HumanoidModelNode::setRobot(double squat, double open, double incl, double sideIncl,double footIncl)
{
    robot.setPoseParams(squat,open,(incl/180.)*PI,(sideIncl/180.)*PI,(footIncl/180.)*PI);
    Eigen::Vector3d footCom;
    Eigen::Vector3d refCom;
    robot.getComVecs(footCom,refCom,override_com);
    ROS_WARN("COM_POINT: %4.3f %4.3f %4.3f",refCom[0],refCom[1],refCom[2]);
    robot.setIKRef(refCom);
    comPoint     = refCom;
    footComPoint = footCom;

    tf::pointEigenToMsg(comPoint,humanoidPropsMsg.comPoint);
    tf::pointEigenToMsg(footComPoint,humanoidPropsMsg.footComPoint);



    humanoidPropsMsg.squat        = squat;
    humanoidPropsMsg.open         = open;
    humanoidPropsMsg.incl         = incl;
    humanoidPropsMsg.sideIncl     = sideIncl;
    humanoidPropsMsg.footIncl     = footIncl;
    sendHumanoidProps(humanoidPropsMsg);

    setRobotFlag = false;
}

void HumanoidModelNode::loadMap()
{
    mapMsg.request.update = false;
    if(mapCli.call(mapMsg))
    {
        int h = mapMsg.response.idMap.map.layout.dim[0].size;
        int w = mapMsg.response.idMap.map.layout.dim[1].size;

        std::vector<int> data =  mapMsg.response.idMap.map.data;
        Eigen::Map<Eigen::MatrixXi> mat(data.data(), h, w);
        map.map = mat;
        for(int i = 0; i < int(mapMsg.response.idMap.jNames.size());i++)
        {
            map.enumMap.insert(std::make_pair(mapMsg.response.idMap.jNames[i],i));
        }
        robotDOF = mapMsg.response.idMap.robotDOF;
        urdfDOF  = mapMsg.response.idMap.urdfDOF;
        ikDOF    = mapMsg.response.idMap.ikDOF;
        //map.print();
        ROS_INFO("[HUMANOID_MODEL] MAP WAS LOADED: ROBOT_DOF: %d  URDF_DOF: %d  IK_DOF: %d",robotDOF,urdfDOF,ikDOF);
    }

}

void HumanoidModelNode::sendHumanoidProps(const humanoid_msgs::HumanoidPropertiesMsg &msg)
{
    ROS_INFO("[HUMANOID_MODEL] HUMANOIDPROPS:  SQUAT = %4.3f  OPEN = %4.3f  INCL = %4.3f  ZC = %4.3f",msg.squat,msg.open,msg.incl,msg.footComPoint.z);
    humanoidPropsTopic.publish(msg);

}

void HumanoidModelNode::reconfigCallback(humanoid_msgs::HumanoidModelConfig& config, uint32_t level)
{
    boost::mutex::scoped_lock lock(mutex);
    squat     = config.squat;
    open      = config.open;
    incl      = config.incl;
    sideIncl  = config.sideIncl;
    footIncl  = config.footIncl;
    comX      = config.comX;
    comY      = config.comY;
    comZ      = config.comZ;
    arm0      = config.arm0;
    arm1      = config.arm1;
    arm2      = config.arm2;
    setRobotFlag = true;

    calcIK     = config.calcIK;
    calcFK     = config.calcFK;
    calcCOM    = config.calcCOM;
    calcZMP    = config.calcZMP;
    calcInvDyn = config.calcInvDyn;

    dt    = config.dt;
    runTimer.setPeriod(ros::Duration(dt));
}


void HumanoidModelNode::reconfigMovCreatorCallback(humanoid_msgs::HumanoidModelMovCreatorConfig& config, uint32_t level)
{
    boost::mutex::scoped_lock lock(mutex);

    modelMovCreatorMsg.rArm0      = config.rArm0;
    modelMovCreatorMsg.rArm1      = config.rArm1;
    modelMovCreatorMsg.rArm2      = config.rArm2;

    modelMovCreatorMsg.lArm0      = config.lArm0;
    modelMovCreatorMsg.lArm1      = config.lArm1;
    modelMovCreatorMsg.lArm2      = config.lArm2;

    modelMovCreatorMsg.rFootX    = config.rFootX;
    modelMovCreatorMsg.rFootY    = config.rFootY;
    modelMovCreatorMsg.rFootZ    = config.rFootZ;
    modelMovCreatorMsg.rFootRX   = config.rFootRX;
    modelMovCreatorMsg.rFootRY   = config.rFootRY;
    modelMovCreatorMsg.rFootRZ   = config.rFootRZ;

    modelMovCreatorMsg.lFootX    = config.lFootX;
    modelMovCreatorMsg.lFootY    = config.lFootY;
    modelMovCreatorMsg.lFootZ    = config.lFootZ;
    modelMovCreatorMsg.lFootRX   = config.lFootRX;
    modelMovCreatorMsg.lFootRY   = config.lFootRY;
    modelMovCreatorMsg.lFootRZ   = config.lFootRZ;

    modelMovCreatorMsg.on              = config.on;
    modelMovCreatorMsg.toJointState    = config.toJointState;
    modelMovCreatorMsg.toMotorState    = config.toMotorState;
}


HumanoidModelNode::~HumanoidModelNode()
{

}
