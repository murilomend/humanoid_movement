#include "humanoid_interface/HumanoidInterface.h"


HumanoidInterface::HumanoidInterface(ros::NodeHandle nh_, ros::NodeHandle nh_private_) : nh(nh_) , nh_private(nh_private_)
{

    //Service Server Stuff
    interfaceSrv = nh.advertiseService("humanoid_interface/cmd",&HumanoidInterface::interfaceService,this);

    //Dynamic Reconfigure Server
    config_server.reset(new HumanoidInterfaceConfigServer(nh_private));
    HumanoidInterfaceConfigServer::CallbackType f = boost::bind(&HumanoidInterface::reconfigCallback, this, _1, _2);
    config_server->setCallback(f);/**/



    if (!nh_private.getParam ("dt", dt))
      dt = 0.07;
    if (!nh_private.getParam ("robotPID", robotPID))
      robotPID = 0.07;
    if (!nh_private.getParam ("gazeboPID", gazeboPID))
      gazeboPID = 0.07;
    if (!nh_private.getParam ("toGazebo", toGazebo))
      toGazebo = false;
    if (!nh_private.getParam ("toRobot", toRobot))
      toRobot = true;
    if (!nh_private.getParam ("send2Motor", send2Motor))
      send2Motor = true;
    if (!nh_private.getParam ("readMotor", readMotor))
      readMotor = true;
    if (!nh_private.getParam ("pingAll", pingAll))
    {
      pingAll  = false;
      pingScan = false;
    }
    if (!nh_private.getParam ("testDT", testDT))
      testDT = false;

     readParam = 0;




    //Service Stuff
    mapCli    = nh.serviceClient<humanoid_msgs::LoadMapConfigsSrv>("humanoid_loadmap/load");

    //Publisher Stuff
    motorStateTopic     = nh.advertise<humanoid_msgs::MotorStateMsg>("humanoid_interface/motorState", 1000);
    motorPIDTopic       = nh.advertise<humanoid_msgs::MotorPIDMsg>("humanoid_interface/get_motorPID", 1000);
    jointStateTopic     = nh.advertise<humanoid_msgs::JointStateMsg>("humanoid_interface/get_jointState", 1000);


    //Subscriber Stuff
    jointStateSubPtr.reset(new JointStateSub(nh,"humanoid_model/jointState",1));
    jointStateSubPtr->registerCallback(&HumanoidInterface::jointStateCallback, this);

    jointHeadStateSubPtr.reset(new JointStateSub(nh,"humanoid_model/jointHeadState",1));
    jointHeadStateSubPtr->registerCallback(&HumanoidInterface::jointHeadStateCallback, this);

    motorPIDSubPtr.reset(new MotorPIDSub(nh,"humanoid_interface/set_motorPID",1));
    motorPIDSubPtr->registerCallback(&HumanoidInterface::motorPIDCallback, this);

    if(toGazebo)
    {
        jointStateGazeboSubPtr.reset(new JointStateGazeboSub(nh,"/SAKURA/joint_position_controller/state",1));
        jointStateGazeboSubPtr->registerCallback(&HumanoidInterface::jointStateGazeboCallback, this);
    }

    loadMap();

    if(toGazebo)
    {
        //Action stuff
        // tell the action client that we want to spin a thread by default
        trj_client = new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>("/SAKURA/joint_position_controller/follow_joint_trajectory/", true);


        // wait for action server to come up
        while(!trj_client->waitForServer(ros::Duration(5.0)))  ROS_WARN("[HUMANOID_INTERFACE] Waiting for the joint_trajectory_action server");
        initActionSettings();
        readFromGazebo(READ_PID);
        readFromGazebo(READ_POS);

    }
    count = 0;
    if(toRobot)
    {

       //Protocol 1.0

        // Load Paramameter For Connection
       dynamixel_info_scan1_ = new dynamixel_driver::DynamixelInfo;

       dynamixel_info_scan1_->lode_info.device_name      = std::string("/dev/ttyUSB1");
       dynamixel_info_scan1_->lode_info.baud_rate        = 1000000;
       dynamixel_info_scan1_->lode_info.protocol_version = 1.0;

       ROS_INFO("[HUMANOID_INTERFACE] DEVICE: %s  BAUD: %d  PROTOCOL: %f",dynamixel_info_scan1_->lode_info.device_name.c_str(),dynamixel_info_scan1_->lode_info.baud_rate,dynamixel_info_scan1_->lode_info.protocol_version);

      // Protocol 2.0
       // Load Paramameter For Connection
      dynamixel_info_scan2_ = new dynamixel_driver::DynamixelInfo;

      dynamixel_info_scan2_->lode_info.device_name      = std::string("/dev/ttyUSB0");
      dynamixel_info_scan2_->lode_info.baud_rate        = 1000000;
      dynamixel_info_scan2_->lode_info.protocol_version = 2.0;

      ROS_INFO("[HUMANOID_INTERFACE] DEVICE: %s  BAUD: %d  PROTOCOL: %f",dynamixel_info_scan2_->lode_info.device_name.c_str(),dynamixel_info_scan2_->lode_info.baud_rate,dynamixel_info_scan2_->lode_info.protocol_version);


      dynamixel_driver10_ = new dynamixel_driver::DynamixelDriver(dynamixel_info_scan1_->lode_info.device_name,
                                                                dynamixel_info_scan1_->lode_info.baud_rate,
                                                                dynamixel_info_scan1_->lode_info.protocol_version);

      dynamixel_driver21_ = new dynamixel_driver::DynamixelDriver(dynamixel_info_scan2_->lode_info.device_name,
                                                                dynamixel_info_scan2_->lode_info.baud_rate,
                                                                dynamixel_info_scan2_->lode_info.protocol_version);

      dynamixel_info_scan1_->lode_info.device_name      = std::string("/dev/ttyUSB0");
      dynamixel_info_scan2_->lode_info.device_name      = std::string("/dev/ttyUSB1");


      dynamixel_driver11_ = new dynamixel_driver::DynamixelDriver(dynamixel_info_scan1_->lode_info.device_name,
                                                                dynamixel_info_scan1_->lode_info.baud_rate,
                                                                dynamixel_info_scan1_->lode_info.protocol_version);

      dynamixel_driver20_ = new dynamixel_driver::DynamixelDriver(dynamixel_info_scan2_->lode_info.device_name,
                                                                dynamixel_info_scan2_->lode_info.baud_rate,
                                                                dynamixel_info_scan2_->lode_info.protocol_version);




        while (ids.empty())
        {
           ROS_INFO("SCANNING");
           ids = scanDynamixel();
           if(ids.empty()) ROS_ERROR("NO MOTORS DETECTED: PLEASE CHECK CONNECTION");
        }
        if(loadDynamixel())
        {
            ROS_INFO("[HUMANOID_INTERFACE] DYNAMIXELS WERE LOADED");
        }
        setTorque(false);
        setOperatingMode(3);
        //setShutdown(52);
        setShutdown(20);
        setTorque(true);

        //setReturnLevel(1);

        ROS_INFO("[HUMANOID_INTERFACE] FOUND MOTORS:");
        if(!dynamixel_info1_.empty())
        {
            ROS_INFO("[HUMANOID_INTERFACE] PROTOCOL 1.0: %d",int(dynamixel_info1_.size()));
            for (int i = 0;i < int(dynamixel_info1_.size());++i)
            {
                ROS_INFO("[HUMANOID_INTERFACE] ID: %d DEVICE: %s  BAUD: %d  PROTOCOL: %f",dynamixel_info1_[i]->model_id,dynamixel_info1_[i]->lode_info.device_name.c_str(),dynamixel_info1_[i]->lode_info.baud_rate,dynamixel_info1_[i]->lode_info.protocol_version);
            }
        }
        if(!dynamixel_info2_.empty())
        {
            ROS_INFO("[HUMANOID_INTERFACE] PROTOCOL 2.0: %d",int(dynamixel_info2_.size()));
            for (int i = 0;i < int(dynamixel_info2_.size());++i)
            {
                ROS_INFO("[HUMANOID_INTERFACE] ID: %d DEVICE: %s  BAUD: %d  PROTOCOL: %f",dynamixel_info2_[i]->model_id,dynamixel_info2_[i]->lode_info.device_name.c_str(),dynamixel_info2_[i]->lode_info.baud_rate,dynamixel_info2_[i]->lode_info.protocol_version);
            }
        }
        //readFromRobot(READ_PID);
        //readFromRobot(READ_POS);
        //jointState    = motor2JointState(map.map,jointReadState);
        //jointStateOld = motor2JointState(map.map,jointReadState);
        //jointState.dt = 2;

        for(int i = 0; i < robotDOF;i++)
        {
            motorPID.kp.push_back(850);
            motorPID.kd.push_back(0);
            motorPID.ki.push_back(0);
        }
    }
    //Run Callback
    runTimer  = nh.createTimer(ros::Duration(dt), &HumanoidInterface::runCallBack,this);
    firstCmd  = true;
    updatePID = false;


}

HumanoidInterface::~HumanoidInterface()
{

}

void HumanoidInterface::reconfigCallback(humanoid_msgs::HumanoidInterfaceConfig& config, uint32_t level)
{
    boost::mutex::scoped_lock lock(mutex);
    toRobot   = config.toRobot;
    toGazebo  = config.toGazebo;
    send2Motor= config.send2Motor;
    readMotor = config.readMotor;
    readParam = config.readParam;

    if(pingAll != config.pingAll) pingScan = true;
    pingAll = config.pingAll;
    testDT = config.testDT;

    dt    = config.dt;
    runTimer.setPeriod(ros::Duration(dt));
}


std::vector<int> HumanoidInterface::scanDynamixel()
{
    inversePorts  = false;
    //Procura ate o ID 30
    std::vector<int>  idVec;
    idVec.clear();
    idsProtocol.clear();
    idsProtocol.resize(robotDOF,0);
    int p1 = dynamixel_driver10_->scan(idVec,30);
    ROS_WARN("[HUMANOID_INTERFACE] P1: %d",p1);
    int p2 = dynamixel_driver21_->scan(idVec,30);
    ROS_WARN("[HUMANOID_INTERFACE] P2: %d",p2);

    if(p1 == 0 && p2 == 0)
    {
        idVec.clear();
        ROS_WARN("TRYING SCAN AGAIN");
        p1 = dynamixel_driver11_->scan(idVec,60);
        ROS_WARN("[HUMANOID_INTERFACE] P1: %d",p1);
        p2 = dynamixel_driver20_->scan(idVec,60);
        ROS_WARN("[HUMANOID_INTERFACE] P2: %d",p2);
        inversePorts = true;
    }
    ROS_INFO("[HUMANOID_INTERFACE] PROTOCOL 1.0");    
    for(int i = 0; i < p1;i++)
    {
        ROS_INFO("[HUMANOID_INTERFACE] FOUND ID: %d",idVec[i]);
        idsProtocol.push_back(1);
    }
    ROS_INFO("[HUMANOID_INTERFACE] PROTOCOL 2.0");
    for(int i = p1; i < (p1 + p2);i++)
    {
        ROS_INFO("[HUMANOID_INTERFACE] FOUND ID: %d",idVec[i]);
        idsProtocol.push_back(2);
    }
    return idVec;
}


void HumanoidInterface::clearDynamixelDriver()
{
    if(multi_driver_read1_ != NULL) delete multi_driver_read1_;
    if(multi_driver_read2_ != NULL) delete multi_driver_read2_;
    if(multi_driver1_ != NULL)      delete multi_driver1_;
    if(multi_driver2_ != NULL)      delete multi_driver2_;
    if(writeValue1_ != NULL)        delete writeValue1_;
    if(writeValue2_ != NULL)        delete writeValue2_;
    dynamixel_info_read1_.clear();
    dynamixel_info1_.clear();
    dynamixel_info_read2_.clear();
    dynamixel_info2_.clear();
}

bool HumanoidInterface::loadDynamixel()
{
    idsInverse1.clear();
    idsInverse2.clear();
    idsInverse1.resize(robotDOF);
    idsInverse2.resize(robotDOF);
    for(int i = 0; i < robotDOF;i++)
    {
        idsInverse1[i] = -1;
        idsInverse2[i] = -1;
    }
    multi_driver_read1_ = NULL;
    multi_driver_read2_ = NULL;
    multi_driver1_      = NULL;
    multi_driver2_      = NULL;
    dynamixel_info1_.clear();
    dynamixel_info2_.clear();
    dynamixel_info_read2_.clear();
    bool ret = true;
    for (int i = 0;i < int(ids.size());++i)
    {
        if(idsProtocol[i] == 1)
        {
            dynamixel_driver::DynamixelInfo* motor_info1_ = new dynamixel_driver::DynamixelInfo;
            motor_info1_->lode_info.device_name = (inversePorts)?std::string("/dev/ttyUSB0"):std::string("/dev/ttyUSB1");
            motor_info1_->lode_info.baud_rate   = 1000000;
            motor_info1_->lode_info.protocol_version = 1.0;
            motor_info1_->model_id = ids[i];
            idsInverse1[motor_info1_->model_id] = dynamixel_info1_.size();
            idsInverse2[motor_info1_->model_id] = -1;
            dynamixel_info1_.push_back(motor_info1_);
        }
        else if(idsProtocol[i] == 2)
        {
            dynamixel_driver::DynamixelInfo* motor_info2_ = new dynamixel_driver::DynamixelInfo;
            motor_info2_->lode_info.device_name = (inversePorts)?std::string("/dev/ttyUSB1"):std::string("/dev/ttyUSB0");
            motor_info2_->lode_info.baud_rate   = 1000000;
            motor_info2_->lode_info.protocol_version = 2.0;
            motor_info2_->model_id = ids[i];
            if(ids[i] >= 8)
            {
                dynamixel_info_read2_.push_back(motor_info2_);
            }
            idsInverse2[motor_info2_->model_id] = dynamixel_info2_.size();
            idsInverse1[motor_info2_->model_id] = -1;
            dynamixel_info2_.push_back(motor_info2_);
        }


    }

    if(!dynamixel_info1_.empty())
    {
        multi_driver1_      = new dynamixel_multi_driver::DynamixelMultiDriver(dynamixel_info1_[0]->lode_info.device_name,
                                                                     dynamixel_info1_[0]->lode_info.baud_rate,
                                                                     dynamixel_info1_[0]->lode_info.protocol_version);
        ret *=  multi_driver1_->loadDynamixel(dynamixel_info1_);
        if (!multi_driver1_->initSyncWrite())
          ROS_ERROR("[HUMANOID_INTERFACE] PROTOCOL 1.0: Init SyncWrite Failed!");
    }
    if(!dynamixel_info2_.empty())
    {

        multi_driver2_      = new dynamixel_multi_driver::DynamixelMultiDriver(dynamixel_info2_[0]->lode_info.device_name,
                                                                     dynamixel_info2_[0]->lode_info.baud_rate,
                                                                     dynamixel_info2_[0]->lode_info.protocol_version);
        ret *=  multi_driver2_->loadDynamixel(dynamixel_info2_);
        if (!multi_driver2_->initSyncWrite())
          ROS_ERROR("[HUMANOID_INTERFACE] PROTOCOL 2.0: Init SyncWrite Failed!");
    }
    if(!dynamixel_info_read2_.empty())
    {
        multi_driver_read2_ = new dynamixel_multi_driver::DynamixelMultiDriver(dynamixel_info_read2_[0]->lode_info.device_name,
                                                                     dynamixel_info_read2_[0]->lode_info.baud_rate,
                                                                     dynamixel_info_read2_[0]->lode_info.protocol_version);
        ret *=  multi_driver_read2_->loadDynamixel(dynamixel_info_read2_);
    }





    writeValue1_ = new WriteValue;
    writeValue2_ = new WriteValue;

    ROS_INFO("[HUMANOID_INTERFACE] DYNAMIXEL_INFO_SIZE: %d",int(dynamixel_info1_.size() + dynamixel_info2_.size()));
    return ret;
}

bool HumanoidInterface::setTorque(bool onoff)
{
    writeValue1_->torque.clear();
    writeValue2_->torque.clear();
    for (int i = 0;i < int(ids.size());++i)
    {
        if(idsProtocol[i] == 1)  writeValue1_->torque.push_back(onoff);
        if(idsProtocol[i] == 2)  writeValue2_->torque.push_back(onoff);

    }
    if(!writeValue1_->torque.empty())
    {
        if (!multi_driver1_->syncWriteTorque(writeValue1_->torque))
        {
            ROS_ERROR("[HUMANOID_INTERFACE] PROTOCOL 1.0: SyncWrite Torque Failed!");
            return false;
        }
    }
    if(!writeValue2_->torque.empty())
    {        
        if (!multi_driver2_->syncWriteTorque(writeValue2_->torque))
        {
            ROS_ERROR("[HUMANOID_INTERFACE] PROTOCOL 2.0: SyncWrite Torque Failed!");
            return false;
        }
    }

  return true;
}

bool HumanoidInterface::setTorque(std::vector<bool> onoff)
{
    writeValue1_->torque.clear();
    writeValue2_->torque.clear();


    for (int i = 0;i < int(ids.size());++i)
    {
        if(idsProtocol[i] == 1)  writeValue1_->torque.push_back(onoff[i]);
        if(idsProtocol[i] == 2)  writeValue2_->torque.push_back(onoff[i]);

    }

    if(!writeValue1_->torque.empty())
    {   
        if (!multi_driver1_->syncWriteTorque(writeValue1_->torque))
        {
            ROS_ERROR("[HUMANOID_INTERFACE] PROTOCOL 1.0: SyncWrite Torque Failed!");
            return false;
        }
    }
    if(!writeValue2_->torque.empty())
    {
        if (!multi_driver2_->syncWriteTorque(writeValue2_->torque))
        {
            ROS_ERROR("[HUMANOID_INTERFACE] PROTOCOL 2.0: SyncWrite Torque Failed!");
            return false;
        }
    }

  return true;
}

bool HumanoidInterface::setReturnLevel(unsigned int level)
{
    writeValue1_->returnLevel.clear();
    writeValue2_->returnLevel.clear();
    for (int i = 0;i < int(ids.size());++i)
    {
        if(idsProtocol[i] == 1)  writeValue1_->returnLevel.push_back(level);
        if(idsProtocol[i] == 2)  writeValue2_->returnLevel.push_back(level);

    }
    if(!writeValue1_->returnLevel.empty())
    {
        if (!multi_driver1_->syncWriteReturnLevel(writeValue1_->returnLevel))
        {
            ROS_ERROR("[HUMANOID_INTERFACE] PROTOCOL 1.0: SyncWrite Return Level Failed!");
            return false;
        }
    }
    if(!writeValue2_->returnLevel.empty())
    {
        if (!multi_driver2_->syncWriteReturnLevel(writeValue2_->returnLevel))
        {
            ROS_ERROR("[HUMANOID_INTERFACE] PROTOCOL 2.0: SyncWrite Return Level Failed!");
            return false;
        }
    }

  return true;
}


bool HumanoidInterface::setOperatingMode(unsigned int mode)
{
    writeValue1_->operMode.clear();
    writeValue2_->operMode.clear();

    for (int i = 0;i < int(ids.size());++i)
    {
        //if(idsProtocol[i] == 1)  writeValue1_->operMode.push_back(mode);
        if(idsProtocol[i] == 2)  writeValue2_->operMode.push_back(mode);
    }

    /*if(!writeValue1_->operMode.empty())
    {
        if (!multi_driver1_->syncWriteOperatingMode(writeValue1_->operMode))
        {
            ROS_ERROR("[HUMANOID_INTERFACE] PROTOCOL 1.0: SyncWrite Return Level Failed!");
            return false;
        }
    }*/

    if(!writeValue2_->operMode.empty())
    {
        if (!multi_driver2_->syncWriteOperatingMode(writeValue2_->operMode))
        {
            ROS_ERROR("[HUMANOID_INTERFACE] PROTOCOL 2.0: SyncWrite Return Level Failed!");
            return false;
        }
    }

  return true;
}


bool HumanoidInterface::setShutdown(unsigned int mode)
{
    writeValue1_->shutdown.clear();
    writeValue2_->shutdown.clear();

    for (int i = 0;i < int(ids.size());++i)
    {
        //if(idsProtocol[i] == 1)  writeValue1_->shutdown.push_back(mode);
        if(idsProtocol[i] == 2)  writeValue2_->shutdown.push_back(mode);
    }
    /*if(!writeValue1_->shutdown.empty())
    {
        if (!multi_driver1_->syncWriteShutdown(writeValue1_->shutdown))
        {
            ROS_ERROR("[HUMANOID_INTERFACE] PROTOCOL 1.0: SyncWrite Shutdown Failed!");
            return false;
        }
    }*/
    if(!writeValue2_->shutdown.empty())
    {
        if (!multi_driver2_->syncWriteShutdown(writeValue2_->shutdown))
        {
            ROS_ERROR("[HUMANOID_INTERFACE] PROTOCOL 2.0: SyncWrite Shutdown Failed!");
            return false;
        }
    }
  return true;
}



void HumanoidInterface::runCallBack(const ros::TimerEvent&)
{
    if(toGazebo)
    {
        if(send2Motor)
        {
            if(!jointState.pos.empty())
            {
                if(!jointHeadState.pos.empty())
                {
                    jointState.pos[map.map(HEAD0,ROBOT_IDS)] = jointHeadState.pos[map.map(HEAD0,ROBOT_IDS)];
                    jointState.pos[map.map(HEAD1,ROBOT_IDS)] = jointHeadState.pos[map.map(HEAD1,ROBOT_IDS)];
                    jointState.vel[map.map(HEAD0,ROBOT_IDS)] = jointHeadState.vel[map.map(HEAD0,ROBOT_IDS)];
                    jointState.vel[map.map(HEAD1,ROBOT_IDS)] = jointHeadState.vel[map.map(HEAD1,ROBOT_IDS)];
                }
                send2Gazebo(jointState,dt);
            }
        }
        if(updatePID)
        {
            updateGazeboPID(motorPID);
            motorPIDTopic.publish(motorPID);
            updatePID = false;
        }
        if(readMotor)
        {
            switch (readParam)
            {
            case 0:
                readFromGazebo(READ_POS);
                break;
            case 1:
                readFromGazebo(READ_VEL);
                break;
            case 2:
                readFromGazebo(READ_CURR);
                break;
            case 3:
                readFromGazebo(READ_TEMP);
                break;
            case 4:
                readFromGazebo(READ_CURR);
                readFromGazebo(READ_TEMP);
                break;
            case 5:
                readFromGazebo(READ_ERROR);
                break;
            default:
                break;
            }
        }
    }
    if(toRobot)
    {
        if(testDT)  lasttime=ros::Time::now();

        switch (jointState.source)
        {
        case MOTOR_HEAD:
                jointState.pos[map.map(HEAD0,ROBOT_IDS)] = jointHeadState.pos[map.map(HEAD0,ROBOT_IDS)];
                jointState.pos[map.map(HEAD1,ROBOT_IDS)] = jointHeadState.pos[map.map(HEAD1,ROBOT_IDS)];
            break;
        default:
            break;
        }
        if(send2Motor)
        {

            if(!jointStateOld.pos.empty() && !jointState.pos.empty())
            {
                if(!jointHeadState.pos.empty())
                {
                    jointState.pos[map.map(HEAD0,ROBOT_IDS)] = jointHeadState.pos[map.map(HEAD0,ROBOT_IDS)];
                    jointState.pos[map.map(HEAD1,ROBOT_IDS)] = jointHeadState.pos[map.map(HEAD1,ROBOT_IDS)];
                    jointState.vel[map.map(HEAD0,ROBOT_IDS)] = jointHeadState.vel[map.map(HEAD0,ROBOT_IDS)];
                    jointState.vel[map.map(HEAD1,ROBOT_IDS)] = jointHeadState.vel[map.map(HEAD1,ROBOT_IDS)];
                }
                if(firstCmd)
                {
                    jointState.source = CONST_VEL;
                    for(int i = 0; i < int(jointState.vel.size());i++)
                    {
                        jointState.vel[i] = 0.3;
                    }
                    jointState.dt     = 3;
                    firstCmd = false;
                }
                send2Robot(jointStateMsg2Motor(map.map,jointStateOld,jointState,dt),dt);
                if(jointState.source == CONST_VEL)  ros::Duration(jointState.dt).sleep();
            }
        }
        if(readMotor)
        {
            switch (readParam)
            {
            case 0:
                readFromRobot(READ_POS);
                break;
            case 1:
                readFromRobot(READ_VEL);
                break;
            case 2:
                readFromRobot(READ_CURR);
                break;
            case 3:
                readFromRobot(READ_TEMP);
                break;
            case 4:
                readFromRobot(READ_CURR);
                readFromRobot(READ_TEMP);
                break;
            case 5:
                readFromRobot(READ_ERROR);
                break;
            default:
                break;
            }
        }
        if(updatePID)
        {
            updateRobotPID(motorPID);
            //updateRobotP(20);
            motorPIDTopic.publish(motorPID);
            updatePID = false;

        }
        if(pingScan)
        {
            ROS_WARN("[HUMANOID_INTERFACE] PING SCAN");
            if(toRobot)
            {
                ids = scanDynamixel();
                clearDynamixelDriver();
                if(loadDynamixel())
                {
                    ROS_WARN("[HUMANOID_INTERFACE] DYNAMIXELS WERE RELOADED");
                    ROS_WARN("[HUMANOID_INTERFACE] DOFS: %3d",int(dynamixel_info1_.size() + dynamixel_info2_.size()));
                }
            }
            setTorque(true);
            //setTorque(true);
            pingScan = false;
        }
        if(testDT)
        {
            currtime=ros::Time::now();
            ros::Duration diff=currtime-lasttime;
            double diffD = diff.toSec();
            if(diffD > dt) ROS_INFO("[HUMANOID_INTERFACE] TIME DIFF: %4.6f",diffD);
            else           ROS_INFO("[HUMANOID_INTERFACE] TIME DIFF: %4.6f",diffD);

        }
    }


}


void HumanoidInterface::loadMap()
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
        ROS_INFO("[HUMANOID_INTERFACE] ROBOT_DOF: %d  URDF_DOF: %d  IK_DOF: %d",robotDOF,urdfDOF,ikDOF);
    }

}


void HumanoidInterface::initActionSettings()
{
    // Init name maps    
    goal.trajectory.joint_names.clear();

    goal.trajectory.joint_names.push_back("J_RARM0");
    goal.trajectory.joint_names.push_back("J_LARM0");
    goal.trajectory.joint_names.push_back("J_RARM1");
    goal.trajectory.joint_names.push_back("J_LARM1");
    goal.trajectory.joint_names.push_back("J_RARM2");
    goal.trajectory.joint_names.push_back("J_LARM2");
    goal.trajectory.joint_names.push_back("J_RLEG0");
    goal.trajectory.joint_names.push_back("J_LLEG0");


    goal.trajectory.joint_names.push_back("J_RLEG1");
    goal.trajectory.joint_names.push_back("J_LLEG1");
    goal.trajectory.joint_names.push_back("J_RLEG2");
    goal.trajectory.joint_names.push_back("J_LLEG2");
    goal.trajectory.joint_names.push_back("J_RLEG3");
    goal.trajectory.joint_names.push_back("J_LLEG3");
    goal.trajectory.joint_names.push_back("J_RFOOT");
    goal.trajectory.joint_names.push_back("J_LFOOT");
    goal.trajectory.joint_names.push_back("J_RLEG4");
    goal.trajectory.joint_names.push_back("J_LLEG4");


    goal.trajectory.joint_names.push_back("J_HEAD0");
    goal.trajectory.joint_names.push_back("J_HEAD1");


    goal.trajectory.points.resize(1);
    goal.trajectory.points[0].positions.resize(20);
    goal.trajectory.points[0].velocities.resize(20);
    for(int i = 0; i < 20;i++)
    {
        goal.trajectory.points[0].positions[i]  = 0;
        goal.trajectory.points[0].velocities[i] = 0;
    }
    goal.trajectory.points[0].time_from_start = ros::Duration(dt);
}

void HumanoidInterface::updateGazeboPID(const MotorPIDMsg &motorPID)
{
    //Dynamic Reconfigure
    for(int i = 0; i < int(goal.trajectory.joint_names.size());i++)
    {
        conf.doubles.clear();
        double_param.name = "p";
        double_param.value = motorPID.kp[i];
        conf.doubles.push_back(double_param);
        double_param.name = "d";
        double_param.value = motorPID.kd[i];
        conf.doubles.push_back(double_param);
        double_param.name = "i";
        double_param.value = motorPID.ki[i];
        conf.doubles.push_back(double_param);
        srv_req.config = conf;
        std::string str;
        str = "/SAKURA/joint_position_controller/gains/" + goal.trajectory.joint_names[i] + "/set_parameters" ;
        //std::cout << str << std::endl;
        //std::cout << "KP: " << motorPID.kp[i] << std::endl;
        ros::service::call(str, srv_req, srv_resp);        
    }

}
void HumanoidInterface::updateRobotPID(const MotorPIDMsg &motorPID)
{
    writeValue1_->kp.clear();
    writeValue2_->kp.clear();

    for(int i = 0; i < int(ids.size());i++)
    {
        int id = ids[i];
        //if(idsProtocol[i] == 1)  writeValue1_->kp.push_back(motorPID.kp[id]);
        if(idsProtocol[i] == 2)  writeValue2_->kp.push_back(motorPID.kp[id]);
    }
    /*if(!writeValue1_->kp.empty())
    {
        if (!multi_driver1_->syncWriteKpPositionGain(writeValue1_->kp))
        {
          ROS_ERROR("[HUMANOID_INTERFACE] PROTOCOL 1.0: SyncWrite KpPositionGain Failed!");
        }
    }*/
    if(!writeValue2_->kp.empty())
    {
        if (!multi_driver2_->syncWriteKpPositionGain(writeValue2_->kp))
        {
          ROS_ERROR("[HUMANOID_INTERFACE] PROTOCOL 2.0: SyncWrite KpPositionGain Failed!");
        }
    }
}

void HumanoidInterface::updateRobotP(const double kp)
{
    writeValue1_->kp.clear();
    writeValue2_->kp.clear();

    for(int i = 0; i < int(ids.size());i++)
    {
        //if(idsProtocol[i] == 1)  writeValue1_->kp.push_back(kp);
        if(idsProtocol[i] == 2)  writeValue2_->kp.push_back(kp);
    }
    /*if(!writeValue1_->kp.empty())
    {
        if (!multi_driver1_->syncWriteKpPositionGain(writeValue1_->kp))
        {
          ROS_ERROR("[HUMANOID_INTERFACE] PROTOCOL 1.0: SyncWrite KpPositionGain Failed!");
        }
    }*/
    if(!writeValue2_->kp.empty())
    {
        if (!multi_driver2_->syncWriteKpPositionGain(writeValue2_->kp))
        {
          ROS_ERROR("[HUMANOID_INTERFACE] PROTOCOL 2.0: SyncWrite KpPositionGain Failed!");
        }
    }
}



void HumanoidInterface::send2Gazebo(const JointStateMsg& qState,const double& dt)
{
    for(int i = 0; i < 20;i++)
    {
        goal.trajectory.points[0].positions[i]  = qState.pos[i];
        goal.trajectory.points[0].velocities[i] = 0; //<--------
    }

    goal.trajectory.points[0].time_from_start = ros::Duration(dt);
    goal.trajectory.header.stamp = ros::Time::now();
    trj_client->sendGoal(goal);
}

void HumanoidInterface::getRad2MotorConv(int index,double &rad2Motor,double &radVel2Motor)
{
    int id1 = idsInverse1[index];
    int id2 = idsInverse2[index];
    if(id1 == -1  && id2 == -1)
    {
        rad2Motor    = 1;
        radVel2Motor = 1;
    }
    else
    {
        int id  = (id1 == -1)?(id2):(id1);
        //ROS_WARN("ID1 = %d  ID2 = %d  ID = %d",id1,id2,id);

        if(id1 != -1)
        {
            rad2Motor = multi_driver1_->multi_dynamixel_[id]->value_of_max_radian_position_/(multi_driver1_->multi_dynamixel_[id]->max_radian_ - multi_driver1_->multi_dynamixel_[id]->min_radian_);
            radVel2Motor = multi_driver1_->multi_dynamixel_[id]->velocity_to_value_ratio_ ;
        }
        else if(id2 != -1)
        {
            rad2Motor = multi_driver2_->multi_dynamixel_[id]->value_of_max_radian_position_/(multi_driver2_->multi_dynamixel_[id]->max_radian_ - multi_driver2_->multi_dynamixel_[id]->min_radian_);
            radVel2Motor = multi_driver2_->multi_dynamixel_[id]->velocity_to_value_ratio_ ;
        }
    }
}


humanoid_msgs::JointStateMsg HumanoidInterface::jointStateMsg2Motor(const Eigen::MatrixXi &map,const JointStateMsg& q0,const JointStateMsg& q1,double dt)
{
    JointStateMsg q  = q1;
    double  deltaPos = 0;

    for(int i = 0; i < int(map.rows()) ;i++)
    {
        int index  = map(i,ROBOT_IDS);
        if(index!= -1)
        {
            int offset      = map(i,ROBOT_OFFSET);
            int motorRef    = map(i,MOTOR_REF);
            int motorCCWLim = map(i,MOTOR_CCWLIM);
            int motorCWLim  = map(i,MOTOR_CWLIM);

            double rad2motor    = 0;
            double radvel2motor = 0;
            getRad2MotorConv(index,rad2motor,radvel2motor);
            if(index != map(HEAD0,ROBOT_IDS)  && index != map(HEAD1,ROBOT_IDS))
            {
                q.pos[index]  = (q1.pos[index] +  offset)*rad2motor  + motorRef;
                deltaPos      = MathUtils::absf(q1.pos[index] - q0.pos[index]);
                q.vel[index]  = radvel2motor*((deltaPos)/(dt)) + 1;
            }
            else
            {
                q.pos[index]  = (q1.pos[index] +  offset)*rad2motor  + motorRef;
                q.vel[index]  = q1.vel[index]*radvel2motor +1;
                q.acc[index]  = 0;
            }
            if(q.source == CONST_VEL)  q.vel[index]  = q1.vel[index]*radvel2motor +1;
            if(q.pos[index] >= motorCCWLim  ||  q.pos[index] <= motorCWLim)
            {
                ROS_WARN("[HUMANOID_MODEL] MOTOR POSITION OUT OF RANGE");
                ROS_WARN("[HUMANOID_MODEL]     i: %d",i);
                ROS_WARN("[HUMANOID_MODEL]     MAP_ROWS: %d",int(map.rows()) );
                ROS_WARN("[HUMANOID_MODEL]     ID: %d",index);
                ROS_WARN("[HUMANOID_MODEL]     POSITION: : %f",q.pos[index]);
                ROS_WARN("[HUMANOID_MODEL]     LIMITS: : CW: %d   CCW: %d",motorCWLim,motorCCWLim);
                ROS_WARN("[HUMANOID_MODEL] ______________________________");
            }
            if(q.vel[index] >= 500 && q.vel[index]  <= 0)
            {
                ROS_WARN("[HUMANOID_MODEL] MOTOR VELOCITY > 500");
                ROS_WARN("[HUMANOID_MODEL] MOTOR VELOCITY SET TO 500");
                q.vel[index] = 500;

            }

        }
    }
    return  q;
}


humanoid_msgs::JointStateMsg HumanoidInterface::motor2JointState(const Eigen::MatrixXi &map,const JointStateMsg& qMotor)
{
    JointStateMsg qJoint = qMotor;
    for(int i = 0; i < int(map.rows()) ;i++)
    {
        int index  = map(i,ROBOT_IDS);
        if(index!= -1)
        {
            int offset      = map(i,ROBOT_OFFSET);
            int motorRef    = map(i,MOTOR_REF);
            double rad2motor    = 0;
            double radvel2motor = 0;
            getRad2MotorConv(index,rad2motor,radvel2motor);
            qJoint.pos[index] = (qMotor.pos[index] - motorRef)*(1./rad2motor) - offset;
        }
    }
    return  qJoint;
}



bool HumanoidInterface::send2Robot(const JointStateMsg& qState, const double &dt)
{
    writeValue1_->pos.clear();
    writeValue1_->prof_vel.clear();
    writeValue1_->prof_acc.clear();

    writeValue2_->pos.clear();
    writeValue2_->prof_vel.clear();
    writeValue2_->prof_acc.clear();



    for(int i = 0; i < int(ids.size());i++)
    {
        int id = ids[i];
        //ROS_INFO("ID: %d POS: %d  VEL: %d  ACC: %d",id,uint32_t(qState.pos[id]),uint32_t(qState.vel[id]),uint32_t(qState.acc[id]));
        if(idsProtocol[i] == 1)
        {
            writeValue1_->pos.push_back(uint32_t(qState.pos[id]));
            writeValue1_->prof_vel.push_back(uint32_t(qState.vel[id]));
            writeValue1_->prof_acc.push_back(uint32_t(qState.acc[id]));
        }
        if(idsProtocol[i] == 2)
        {
            writeValue2_->pos.push_back(uint32_t(qState.pos[id]));
            writeValue2_->prof_vel.push_back(uint32_t(qState.vel[id]));
            writeValue2_->prof_acc.push_back(uint32_t(qState.acc[id]));
        }

    }
    //ROS_INFO("__________________________________________________________________________________________________");
    if(!writeValue1_->pos.empty())
    {
        if (!multi_driver1_->syncWritePosVel1(writeValue1_->pos,writeValue1_->prof_vel))
        {
          ROS_ERROR("[HUMANOID_INTERFACE] PROTOCOL 1.0: SyncWrite Position/Velocity/Acceleration Failed!");
          return false;
        }
    }
    if(!writeValue2_->pos.empty())
    {
        if (!multi_driver2_->syncWritePosVel2(writeValue2_->pos,writeValue2_->prof_vel))
        {
          ROS_ERROR("[HUMANOID_INTERFACE] PROTOCOL 2.0: SyncWrite Position/Velocity/Acceleration Failed!");
          return false;
        }
    }
    return true;
}


void HumanoidInterface::jointStateGazeboCallback(const control_msgs::JointTrajectoryControllerStatePtr &msg)
{
    this->jointStateGazeboMsg = *msg;
}


void HumanoidInterface::readFromGazebo(int option,bool publish)
{
    switch (option)
    {
    case READ_ALL:


        break;
    case READ_PID:
        motorPIDMsg.kp.resize(goal.trajectory.joint_names.size());
        motorPIDMsg.ki.resize(goal.trajectory.joint_names.size());
        motorPIDMsg.kd.resize(goal.trajectory.joint_names.size());
        for(int i = 0; i < int(goal.trajectory.joint_names.size());i++)
        {
            std::string  str = "/SAKURA/joint_position_controller/gains/" +  goal.trajectory.joint_names[i] + "/p";
            motorPIDMsg.kp[i] = nh.param<double>(str, 0);
            str = "/SAKURA/joint_position_controller/gains/" +  goal.trajectory.joint_names[i] + "/i";
            motorPIDMsg.ki[i] = nh.param<double>(str, 0);
            str = "/SAKURA/joint_position_controller/gains/" +  goal.trajectory.joint_names[i] + "/d";
            motorPIDMsg.kd[i] = nh.param<double>(str, 0);
            std::cout << str << std::endl;
            std::cout << "Kp: " << motorPIDMsg.kp[i] << " Ki: " << motorPIDMsg.ki[i] << " Kd: " << motorPIDMsg.kd[i] << std::endl;
        }
        if(publish)  motorPIDTopic.publish(motorPIDMsg);

    case READ_POS:
        jointReadState.header.stamp = ros::Time::now();
        jointReadState.pos.resize(robotDOF);
        jointReadState.vel.resize(robotDOF);
        jointReadState.acc.resize(robotDOF);
        jointReadState.torq.resize(robotDOF);
        jointReadState.pos = jointStateGazeboMsg.actual.positions;
        if(publish)  jointStateTopic.publish(jointReadState);
        break;
    default:
        break;
    }


}
void HumanoidInterface::readFromRobot(int option,bool publish)
{

    switch (option)
    {
    case READ_ALL:
        break;
    case READ_PID:
        motorPIDMsg.kp.resize(robotDOF);
        motorPIDMsg.ki.resize(robotDOF);
        motorPIDMsg.kd.resize(robotDOF);
        if(!dynamixel_info_read2_.empty())
        {
            multi_driver_read2_->readMultiRegister("position_p_gain");
            multi_driver_read2_->readMultiRegister("position_i_gain");
            multi_driver_read2_->readMultiRegister("position_d_gain");
            for (std::vector<dynamixel_tool::DynamixelTool *>::size_type num = 0; num < multi_driver_read2_->multi_dynamixel_.size(); ++num)
            {
                int id = int(multi_driver_read2_->multi_dynamixel_[num]->id_);
                motorPIDMsg.kp[id] = multi_driver_read2_->read_value_["position_p_gain"]->at(num);
                motorPIDMsg.ki[id] = multi_driver_read2_->read_value_["position_i_gain"]->at(num);
                motorPIDMsg.kd[id] = multi_driver_read2_->read_value_["position_d_gain"]->at(num);
                //ROS_WARN("PID SIZE: %d  ID READ: %d  KP: %d",int(motorPIDMsg.kp.size()), id,int(motorPIDMsg.kp[id]));
            }
        }
        if(publish)  motorPIDTopic.publish(motorPIDMsg);
        //ROS_WARN("DONE");
    case READ_POS:
        jointReadState.header.stamp = ros::Time::now();
        jointReadState.pos.resize(robotDOF);
        jointReadState.vel.resize(robotDOF);
        jointReadState.acc.resize(robotDOF);
        jointReadState.torq.resize(robotDOF);
        if(!dynamixel_info1_.empty())
        {
            multi_driver1_->readMultiRegister("present_position");
            for (std::vector<dynamixel_tool::DynamixelTool *>::size_type num = 0; num < multi_driver1_->multi_dynamixel_.size(); ++num)
            {
                int id = int(multi_driver1_->multi_dynamixel_[num]->id_);
                jointReadState.pos[id] = multi_driver1_->read_value_["present_position"]->at(num);
                //ROS_WARN("POS SIZE: %d  ID READ: %d  POS: %d",int(motorPIDMsg.kp.size()), id,int(jointReadState.pos[id]));
            }
        }
        if(!dynamixel_info2_.empty())
        {
            multi_driver2_->readMultiRegister("present_position");
            for (std::vector<dynamixel_tool::DynamixelTool *>::size_type num = 0; num < multi_driver2_->multi_dynamixel_.size(); ++num)
            {
                int id = int(multi_driver2_->multi_dynamixel_[num]->id_);
                jointReadState.pos[id] = multi_driver2_->read_value_["present_position"]->at(num);
                //ROS_WARN("POS SIZE: %d  ID READ: %d  POS: %d",int(motorPIDMsg.kp.size()), id,int(jointReadState.pos[id]));
            }
        }
        if(publish)  jointStateTopic.publish(motor2JointState(map.map,jointReadState));
        //if(publish)  jointStateTopic.publish(jointReadState);
        //ROS_WARN("DONE");
        break;
    case READ_CURR:
        jointReadState.header.stamp = ros::Time::now();
        jointReadState.torq.resize(robotDOF);
        if(!dynamixel_info_read2_.empty())
        {
            multi_driver_read2_->readMultiRegister("present_current");
            for (std::vector<dynamixel_tool::DynamixelTool *>::size_type num = 0; num < multi_driver_read2_->multi_dynamixel_.size(); ++num)
            {
                int id = int(multi_driver_read2_->multi_dynamixel_[num]->id_);
                jointReadState.torq[id] = multi_driver_read2_->read_value_["present_current"]->at(num)*0.00336;
            }
        }
        if(publish)  jointStateTopic.publish(jointReadState);

        break;

    case READ_TEMP:
        motorStateMsg.header.stamp = ros::Time::now();
        motorStateMsg.temp.resize(robotDOF);
        if(!dynamixel_info_read2_.empty())
        {
            multi_driver_read2_->readMultiRegister("present_temperature");
            for (std::vector<dynamixel_tool::DynamixelTool *>::size_type num = 0; num < multi_driver_read2_->multi_dynamixel_.size(); ++num)
            {
                int id = int(multi_driver_read2_->multi_dynamixel_[num]->id_);
                motorStateMsg.temp[id] = multi_driver_read2_->read_value_["present_temperature"]->at(num);
            }
        }
        if(publish)  motorStateTopic.publish(motorStateMsg);

        break;
    case READ_ERROR:
        motorStateMsg.header.stamp = ros::Time::now();
        motorStateMsg.error.resize(robotDOF);
        if(!dynamixel_info_read2_.empty())
        {
            multi_driver_read2_->readMultiRegister("hardware_error_status");
            for (std::vector<dynamixel_tool::DynamixelTool *>::size_type num = 0; num < multi_driver_read2_->multi_dynamixel_.size(); ++num)
            {
                int id = int(multi_driver_read2_->multi_dynamixel_[num]->id_);
                motorStateMsg.error[id] = multi_driver_read2_->read_value_["hardware_error_status"]->at(num);
            }
        }
        if(publish)  motorStateTopic.publish(motorStateMsg);

        break;
    default:
        break;
    }



}


void HumanoidInterface::jointStateCallback(const humanoid_msgs::JointStateMsgPtr &jointState)
{
    this->jointStateOld = this->jointState;
    this->jointState    = *jointState;
}

void HumanoidInterface::jointHeadStateCallback(const humanoid_msgs::JointStateMsgPtr &jointHeadState)
{
    this->jointHeadState    = *jointHeadState;
}

void HumanoidInterface::motorPIDCallback(const humanoid_msgs::MotorPIDMsgPtr &motorPID)
{
    this->motorPID    = *motorPID;
    updatePID = true;
}


bool HumanoidInterface::interfaceService(humanoid_msgs::InterfaceSrv::Request  &msg,
                                         humanoid_msgs::InterfaceSrv::Response &res)
{
    if(msg.getPID)
    {
        if(toGazebo) readFromGazebo(READ_PID);
        if(toRobot)  readFromRobot(READ_PID);
        res.motorPID =  motorPIDMsg;
    }
    if(msg.scanMotors)
    {
        std::vector<int> idVec;
        idVec.clear();
        if(toRobot)
        {
            ids = scanDynamixel();
            clearDynamixelDriver();
            if(loadDynamixel())
            {
                ROS_WARN("[HUMANOID_INTERFACE] DYNAMIXELS WERE RELOADED");
                ROS_WARN("[HUMANOID_INTERFACE] DOFS: %3d",int(dynamixel_info1_.size() + dynamixel_info2_.size()));
            }
        }
        for(int i = 0; i < int(ids.size());i++) res.ids.push_back(ids[i]);
        if(toGazebo)  for(int i = 0; i < 20;i++) res.ids.push_back(i);

    }
    if(int(msg.setTorque.size()) > 1)
    {
        std::vector<bool> torqVec;
        torqVec.clear();
        for(int i = 0; i < int(msg.setTorque.size());i++) torqVec.push_back(msg.setTorque[i]);
        if(toRobot) setTorque(torqVec);

    }
    if(msg.getMotorPos)
    {
        if(toRobot)
        {
            readFromRobot(READ_POS);
            res.motorState.pos.clear();
            res.motorState = motor2JointState(map.map,jointReadState);
        }
        if(toGazebo)
        {
            readFromGazebo(READ_POS);
            res.motorState.pos.clear();
            res.motorState = jointReadState;
        }
    }
    if(msg.fall == true)
    {
        ROS_WARN("FALL");
        if(toRobot) updateRobotP(1);
    }
    else
    {
        if(toRobot && !motorPID.kp.empty()) updateRobotPID(motorPID);
    }
    if(msg.toGazebo || msg.toRobot) send2Motor = msg.send2Motor;


    return true;
}
