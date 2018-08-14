#include "humanoid_learning/HumanoidLearningNode.h"


HumanoidLearningNode::HumanoidLearningNode(ros::NodeHandle nh_, ros::NodeHandle nh_private_) : nh(nh_) , nh_private(nh_private_)
{
    //Parameter Server
    if (!nh_private.getParam ("dt", dt))
        dt =  0.002; //s
    if (!nh_private.getParam ("test",test))
        test =  false;
    if (!nh_private.getParam ("testTime",testTime))
        testTime =  30;//s

    //Publisher Stuff
    controlTopic  = nh.advertise<humanoid_msgs::HumanoidControlMsg>("humanoid_control/cmd", 1000);
    learningTopic  = nh.advertise<humanoid_msgs::LearningMsg>("humanoid_learning/status", 1000);
    impactTopic  = nh.advertise<humanoid_msgs::ImpactMsg>("humanoid_learning/impact", 1000);

    //Subscriber Stuffs
    imuEulerSubPtr.reset(new ImuSub(nh,"humanoid_control/imu_euler",1));
    imuEulerSubPtr->registerCallback(&HumanoidLearningNode::imuEulerCallback, this);

    lipParamsSubPtr.reset(new LipParamsSub(nh,"humanoid_walking/walking_params_state",1));
    lipParamsSubPtr->registerCallback(&HumanoidLearningNode::lipParamsCallback, this);

    humanoidPropsSubPtr.reset(new HumanoidPropsSub(nh,"humanoid_model/humanoid_properties",1));
    humanoidPropsSubPtr->registerCallback(&HumanoidLearningNode::humanoidPropsCallback, this);

    jointStateSubPtr.reset(new JointStateSub(nh,"humanoid_interface/get_jointState",1));
    jointStateSubPtr->registerCallback(&HumanoidLearningNode::jointStateCallback, this);

    lipFeedBackSubPtr.reset(new LipFeedBackSub(nh,"/humanoid_walking/lipFeedback",1));
    lipFeedBackSubPtr->registerCallback(&HumanoidLearningNode::lipFeedBackCallback, this);

    forceRSubPtr.reset(new ForceSub(nh,"ft_sensor_rSensor2",1));
    forceRSubPtr->registerCallback(&HumanoidLearningNode::forceRCallback, this);

    forceLSubPtr.reset(new ForceSub(nh,"ft_sensor_lSensor3",1));
    forceLSubPtr->registerCallback(&HumanoidLearningNode::forceLCallback, this);


    torqueR3SubPtr.reset(new ForceSub(nh,"ft_sensor_rleg3",1));
    torqueR4SubPtr.reset(new ForceSub(nh,"ft_sensor_rleg4",1));
    torqueRFSubPtr.reset(new ForceSub(nh,"ft_sensor_rfoot",1));

    torqueL3SubPtr.reset(new ForceSub(nh,"ft_sensor_lleg3",1));
    torqueL4SubPtr.reset(new ForceSub(nh,"ft_sensor_lleg4",1));
    torqueLFSubPtr.reset(new ForceSub(nh,"ft_sensor_lfoot",1));



    torqueSync.reset(new TorqueSync(TorqueSyncPolicy(10), *torqueR3SubPtr, *torqueR4SubPtr,*torqueRFSubPtr,*torqueL3SubPtr,*torqueL4SubPtr,*torqueLFSubPtr));
    torqueSync->registerCallback(boost::bind(&HumanoidLearningNode::torqueCallback, this, _1, _2, _3, _4, _5, _6));


    //Service Stuff
    mapCli       = nh.serviceClient<humanoid_msgs::LoadMapConfigsSrv>("humanoid_loadmap/load");
    lipCli       = nh.serviceClient<humanoid_msgs::LipParamsSrv>("humanoid_walking/walking_params");
    resetSrv     = nh.advertiseService("humanoid_learning/reset",&HumanoidLearningNode::reset,this);

    runTimer = nh.createTimer(ros::Duration(dt), &HumanoidLearningNode::runCallBack,this);

    loadMap();

    lipMsg.request.get_params = true;
    if(lipCli.call(lipMsg))   calcFreqRefs(lipMsg.response.tS,lipMsg.response.tD,lipMsg.response.delayAll);

    testPoint = 0;
    testBeginFlag = true;
    controlUpdateFlag = false;
    imuInit  = false;
    forceInit  =false;
    torqueInit = false;
    lastFootGround = 0;
    footGroundChanged = false;
    lFootOnGround     = false;
    rFootOnGround     = false;
    imuCount = 0;
    forceCount = 0;
    torqueCount = 0;
    wUpdate = 28;
    tUpdate = 28;
    fUpdate = 112;
    wPoints = 28;
    tPoints = 28;
    fPoints = 112;

    df = 1./(wUpdate*dt);
    resizeData(wUpdate,fUpdate,tUpdate);
    ROS_INFO("[HUMANOID_LEARNING] WPoints: %d  df: %4.3f  dt: %4.3f",wPoints,df,dt);
    //Dynamic Reconfigure Server
    config_server.reset(new HumanoidLearningConfigServer(nh_private));
    HumanoidLearningConfigServer::CallbackType f = boost::bind(&HumanoidLearningNode::reconfigCallback, this, _1, _2);
    config_server->setCallback(f);

}


HumanoidLearningNode::~HumanoidLearningNode()
{

}


void HumanoidLearningNode::reconfigCallback(humanoid_msgs::HumanoidLearningConfig& config, uint32_t level)
{
    boost::mutex::scoped_lock lock(mutex);
    /*learningMsg.x.freqRange = config.freqXRange;
    learningMsg.y.freqRange = config.freqYRange;
    learningMsg.z.freqRange = config.freqZRange;*/

    test                   = config.test;
    testBeginFlag          = test;
    testTime               = config.testTime;

    learningMsg.x.ampRange = config.ampXRange;
    learningMsg.y.ampRange = config.ampYRange;
    learningMsg.z.ampRange = config.ampZRange;

    learningMsg.x.meanRange = config.meanXRange;
    learningMsg.y.meanRange = config.meanYRange;
    learningMsg.z.meanRange = config.meanZRange;

    learningMsg.x.wFreq = learningMsg.y.wFreq = learningMsg.z.wFreq = config.wFreq;
    learningMsg.x.wAmp = learningMsg.y.wAmp = learningMsg.z.wAmp = config.wAmp;
    learningMsg.x.wMean = learningMsg.y.wMean = learningMsg.z.wMean = config.wMean;


}


void HumanoidLearningNode::resizeData(int nIMUSamples,int nForceSamples,int nTorqueSamples)
{
    ImuMsg  imuArray[nIMUSamples];
    imuArrayBatchMsg   =  ImuArrayMsg(imuArray,nIMUSamples);
    imuArrayMsg        =  ImuArrayMsg(imuArray,nIMUSamples);
    Complex  imuEulerMsgSample[nIMUSamples];
    imuEulerMsgXData   = CArray(imuEulerMsgSample,nIMUSamples);
    imuEulerMsgYData   = CArray(imuEulerMsgSample,nIMUSamples);
    imuEulerMsgZData   = CArray(imuEulerMsgSample,nIMUSamples);
    imuEulerMsgXFreq   = CArray(imuEulerMsgSample,nIMUSamples);
    imuEulerMsgYFreq   = CArray(imuEulerMsgSample,nIMUSamples);
    imuEulerMsgZFreq   = CArray(imuEulerMsgSample,nIMUSamples);

    Complex  forceMsgSample[nForceSamples];
    rForceData = CArray(forceMsgSample,nForceSamples);
    lForceData = CArray(forceMsgSample,nForceSamples);
    rForceFreq = CArray(forceMsgSample,nForceSamples);
    lForceFreq = CArray(forceMsgSample,nForceSamples);


    Complex  torqueMsgSample[nTorqueSamples];
    r3TorqueData = CArray(torqueMsgSample,nTorqueSamples);
    l3TorqueData = CArray(torqueMsgSample,nTorqueSamples);
    r3TorqueFreq = CArray(torqueMsgSample,nTorqueSamples);
    l3TorqueFreq = CArray(torqueMsgSample,nTorqueSamples);

    r4TorqueData = CArray(torqueMsgSample,nTorqueSamples);
    l4TorqueData = CArray(torqueMsgSample,nTorqueSamples);
    r4TorqueFreq = CArray(torqueMsgSample,nTorqueSamples);
    l4TorqueFreq = CArray(torqueMsgSample,nTorqueSamples);

    rfTorqueData = CArray(torqueMsgSample,nTorqueSamples);
    lfTorqueData = CArray(torqueMsgSample,nTorqueSamples);
    rfTorqueFreq = CArray(torqueMsgSample,nTorqueSamples);
    lfTorqueFreq = CArray(torqueMsgSample,nTorqueSamples);

    for(int i = 0 ; i < nIMUSamples; i++)
    {
        imuEulerMsgXData[i] = 0;
        imuEulerMsgYData[i] = 0;
        imuEulerMsgZData[i] = 0;
        imuEulerMsgXFreq[i] = 0;
        imuEulerMsgYFreq[i] = 0;
        imuEulerMsgZFreq[i] = 0;
    }
    for(int i = 0 ; i < nForceSamples; i++)
    {
        rForceData[i] = 0;
        lForceData[i] = 0;
        rForceFreq[i] = 0;
        lForceFreq[i] = 0;
    }
    for(int i = 0 ; i < nTorqueSamples; i++)
    {
        r3TorqueData[i] = 0;
        l3TorqueData[i] = 0;
        r3TorqueFreq[i] = 0;
        l3TorqueFreq[i] = 0;

        r4TorqueData[i] = 0;
        l4TorqueData[i] = 0;
        r4TorqueFreq[i] = 0;
        l4TorqueFreq[i] = 0;

        rfTorqueData[i] = 0;
        lfTorqueData[i] = 0;
        rfTorqueFreq[i] = 0;
        lfTorqueFreq[i] = 0;
    }

}

void HumanoidLearningNode::loadMap()
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


void HumanoidLearningNode::runCallBack(const ros::TimerEvent&)
{

    /*if(wPoints > 0 && imuInit)
    {
        calcFeatures(imuEulerMsgXFreq,maxXAmp,maxXFreq,meanX);
        calcFeatures(imuEulerMsgYFreq,maxYAmp,maxYFreq,meanY);
        calcFeatures(imuEulerMsgZFreq,maxZAmp,maxZFreq,meanZ);
        buildLearningMsg();
        send2Topic();
        //learningTopic.publish(learningVec.back());
        imuInit = false;
    }*/
    if(wPoints > 0 && fPoints > 0 && tPoints > 0 && imuInit && forceInit && torqueInit)
    {
        impactMsg.header.stamp = ros::Time::now();
        impactMsg.force.clear();
        impactMsg.gyroX.clear();
        impactMsg.gyroY.clear();
        impactMsg.gyroZ.clear();
        impactMsg.accX.clear();
        impactMsg.accY.clear();
        impactMsg.accZ.clear();
        impactMsg.angX.clear();
        impactMsg.angY.clear();

        for(int i = 0; i < fPoints;i++)
        {
            if(lastFootGround ==-5)         impactMsg.force.push_back(rForceFreq[i].real());
            else  if(lastFootGround == 5)   impactMsg.force.push_back(lForceFreq[i].real());
        }
        impactMsg.torque3.clear();
        impactMsg.torque4.clear();
        impactMsg.torqueF.clear();
        for(int i = 0; i < tPoints;i++)
        {
            if(lastFootGround ==-5)
            {
                impactMsg.torque3.push_back(r3TorqueFreq[i].real());
                impactMsg.torque4.push_back(r4TorqueFreq[i].real());
                impactMsg.torqueF.push_back(rfTorqueFreq[i].real());
            }
            else  if(lastFootGround == 5)
            {
                impactMsg.torque3.push_back(l3TorqueFreq[i].real());
                impactMsg.torque4.push_back(l4TorqueFreq[i].real());
                impactMsg.torqueF.push_back(lfTorqueFreq[i].real());
            }
        }

        for(int i = 0; i < wPoints;i++)
        {
            impactMsg.gyroX.push_back(imuArrayBatchMsg[i].angular_velocity.x);
            impactMsg.gyroY.push_back(imuArrayBatchMsg[i].angular_velocity.y);
            impactMsg.gyroZ.push_back(imuArrayBatchMsg[i].angular_velocity.z);
            impactMsg.accX.push_back(imuArrayBatchMsg[i].linear_acceleration.x);
            impactMsg.accY.push_back(imuArrayBatchMsg[i].linear_acceleration.y);
            impactMsg.accZ.push_back(imuArrayBatchMsg[i].linear_acceleration.z);
            impactMsg.angX.push_back(imuArrayBatchMsg[i].orientation.x);
            impactMsg.angY.push_back(imuArrayBatchMsg[i].orientation.y);
        }
        footGroundChanged = false;
        imuInit = false;
        forceInit = false;
        torqueInit = false;
        send2Topic();
        impactTopic.publish(impactMsg);/**/

        learningMsg.header.stamp = ros::Time::now();
        learningMsg.force.clear();
        for(int i = 0; i < fPoints;i++)
        {
            if(lastFootGround ==-5)         learningMsg.force.push_back(rForceFreq[i].real());
            else  if(lastFootGround == 5)   learningMsg.force.push_back(lForceFreq[i].real());
        }
        learningMsg.torque3.clear();
        learningMsg.torque4.clear();
        learningMsg.torqueF.clear();
        for(int i = 0; i < tPoints;i++)
        {
            if(lastFootGround ==-5)
            {
                learningMsg.torque3.push_back(r3TorqueFreq[i].real());
                learningMsg.torque4.push_back(r4TorqueFreq[i].real());
                learningMsg.torqueF.push_back(rfTorqueFreq[i].real());
            }
            else  if(lastFootGround == 5)
            {
                learningMsg.torque3.push_back(l3TorqueFreq[i].real());
                learningMsg.torque4.push_back(l4TorqueFreq[i].real());
                learningMsg.torqueF.push_back(lfTorqueFreq[i].real());
            }
        }
        learningMsg.imu.clear();
        for(int i = 0; i < wPoints;i++)
        {
            learningMsg.imu.push_back(imuArrayBatchMsg[i]);
        }
        footGroundChanged = false;
        send2Topic();
        forceInit = false;
        torqueInit = false;
        imuInit = false;
    }
    /*if(fPoints > 0 && forceInit)
    {
        learningMsg.header.stamp = ros::Time::now();
        learningMsg.rFoot.data.clear();
        learningMsg.lFoot.data.clear();
        double integralR = 0;
        double integralL = 0;
        double maxR = 0;
        double maxL = 0;
        for(int i = 0; i < fUpdate;i++)
        {
            integralR  += MathUtils::absf(rForceFreq[i].real());
            integralL  += MathUtils::absf(lForceFreq[i].real());
            learningMsg.rFoot.data.push_back(rForceFreq[i].real());
            learningMsg.lFoot.data.push_back(lForceFreq[i].real());
            if(MathUtils::absf(rForceFreq[i].real()) > maxR)  maxR = MathUtils::absf(rForceFreq[i].real());
            if(MathUtils::absf(lForceFreq[i].real()) > maxL)  maxL = MathUtils::absf(lForceFreq[i].real());
        }
        double rMaxAmp = 0,rMaxFreq = 0,rMean = 0;
        double lMaxAmp = 0,lMaxFreq = 0,lMean = 0;
        calcFeatures(rForceFreq,rMaxAmp,rMaxFreq,rMean);
        calcFeatures(rForceFreq,lMaxAmp,lMaxFreq,lMean);

        learningMsg.rFoot.integral = integralR - maxR;
        learningMsg.lFoot.integral = integralL - maxL;

        learningMsg.rFoot.bins.clear();
        learningMsg.lFoot.bins.clear();
        for(int i  = 0; i < fUpdate;i++)
        {
            learningMsg.rFoot.bins.push_back(MathUtils::norm(rForceFreq[i]));
            learningMsg.lFoot.bins.push_back(MathUtils::norm(lForceFreq[i]));
        }
        double     df = 1./(fUpdate*dt);
        learningMsg.rFoot.freq = rMaxFreq*df;
        learningMsg.lFoot.freq = lMaxFreq*df;
        learningMsg.rFoot.amp = rMaxAmp*(2./fUpdate);
        learningMsg.lFoot.amp = lMaxAmp*(2./fUpdate);
        learningMsg.rFoot.mean = rMean;
        learningMsg.lFoot.mean = lMean;
        learningMsg.df = df;
        send2Topic();
        forceInit = false;
    }*/
    /*if(test)
    {
        if(wPoints > 0 && imuInit)
        {
            calcFeatures(imuEulerMsgXFreq,maxXAmp,maxXFreq,meanX);
            calcFeatures(imuEulerMsgYFreq,maxYAmp,maxYFreq,meanY);
            calcFeatures(imuEulerMsgZFreq,maxZAmp,maxZFreq,meanZ);
            buildLearningMsg();
            learningTopic.publish(learningVec.back());
            imuInit = false;
        }
        sendControl();
    }
    else
    {
        calcControl();
    }*/
}

void HumanoidLearningNode::calcFeatures(CArray &sample,double &amp,double &freq,double &mean,double minFreq,double maxFreq)
{
    mean = MathUtils::mean(sample);
    MathUtils::fft(sample);
    getBiggerFreq(sample,amp,freq,minFreq,maxFreq);
}

void HumanoidLearningNode::sendControl()
{
    if(controlUpdateFlag)
    {
        double t = (ros::Time::now() - testBegin).toSec();
        if(t >= 0 && t <= testTime)
        {
            tf::pointEigenToMsg(Eigen::Vector3d(rFootFacPosX(t),rFootFacPosY(t),rFootFacPosZ(t)),controlMsg.rFootFac.pos);
            tf::pointEigenToMsg(Eigen::Vector3d(lFootFacPosX(t),lFootFacPosY(t),lFootFacPosZ(t)),controlMsg.lFootFac.pos);
            controlTopic.publish(controlMsg);
        }
    }
}


void HumanoidLearningNode::calcControl()
{
    std::vector<double>  time;
    std::vector<double>  rPosX;
    std::vector<double>  rPosY;
    std::vector<double>  rPosZ;
    std::vector<double>  lPosX;
    std::vector<double>  lPosY;
    std::vector<double>  lPosZ;

    for(int i = 0; i < int(learningVec.size());i++)
    {
        time.push_back(learningVec[i].header.stamp.toSec() - learningVec[0].header.stamp.toSec());
        rPosX.push_back(learningVec[i].x.perf);
        rPosY.push_back(learningVec[i].y.perf);
        rPosZ.push_back(learningVec[i].z.perf);
        lPosX.push_back(learningVec[i].x.perf);
        lPosY.push_back(learningVec[i].y.perf);
        lPosZ.push_back(learningVec[i].z.perf);
    }
    if(!learningVec.empty())
    {
        rFootFacPosX.set_points(time,rPosX);
        rFootFacPosY.set_points(time,rPosY);
        rFootFacPosZ.set_points(time,rPosZ);

        lFootFacPosX.set_points(time,lPosX);
        lFootFacPosY.set_points(time,lPosY);
        lFootFacPosZ.set_points(time,lPosZ);

        controlUpdateFlag  =true;
    }
}

void HumanoidLearningNode::getBiggerFreq(CArray &fft,double &max,double &maxIndex,double freqLower,double freqUpper)
{
   //Skips the first which is the DC
    int upIndex  = ceil(freqUpper/df);
    int lowIndex = ceil(freqLower/df);
    max = MathUtils::norm(fft[lowIndex]);
    maxIndex = lowIndex;
    for(int i = lowIndex + 1; i < upIndex;i++)
    {
        if(MathUtils::norm(fft[i].real()) > max)
        {
            maxIndex = i;
            max = MathUtils::norm(fft[i].real());
        }
    }
}

void HumanoidLearningNode::calcPerformance()
{
    /*learningMsg.x.freqPerf = MathUtils::absf(learningMsg.x.freq - learningMsg.x.freqRef)/(learningMsg.x.freqRange);
    learningMsg.y.freqPerf = MathUtils::absf(learningMsg.y.freq - learningMsg.y.freqRef)/(learningMsg.y.freqRange);
    learningMsg.z.freqPerf = MathUtils::absf(learningMsg.z.freq - learningMsg.z.freqRef)/(learningMsg.z.freqRange);*/

    learningMsg.x.freqPerf = MathUtils::absf(learningMsg.x.freq - learningMsg.x.freqRef)/(2*learningMsg.x.freqRef);
    learningMsg.y.freqPerf = MathUtils::absf(learningMsg.y.freq - learningMsg.y.freqRef)/(2*learningMsg.y.freqRef);
    learningMsg.z.freqPerf = MathUtils::absf(learningMsg.z.freq - learningMsg.z.freqRef)/(2*learningMsg.z.freqRef);

    learningMsg.x.ampPerf  = MathUtils::absf(learningMsg.x.amp - learningMsg.x.ampRef)/(learningMsg.x.ampRange);
    learningMsg.y.ampPerf  = MathUtils::absf(learningMsg.y.amp - learningMsg.y.ampRef)/(learningMsg.y.ampRange);
    learningMsg.z.ampPerf  = MathUtils::absf(learningMsg.z.amp - learningMsg.z.ampRef)/(learningMsg.z.ampRange);

    learningMsg.x.meanPerf = MathUtils::absf(MathUtils::absf(learningMsg.x.mean) - learningMsg.x.meanRef)/(learningMsg.x.meanRange);
    learningMsg.y.meanPerf = MathUtils::absf(MathUtils::absf(learningMsg.y.mean) - learningMsg.y.meanRef)/(learningMsg.y.meanRange);
    learningMsg.z.meanPerf = MathUtils::absf(MathUtils::absf(learningMsg.z.mean) - learningMsg.z.meanRef)/(learningMsg.z.meanRange);


    learningMsg.x.perf     = (learningMsg.x.freqPerf*learningMsg.x.wFreq + learningMsg.x.ampPerf*learningMsg.x.wAmp  + learningMsg.x.meanPerf*learningMsg.x.wMean)/(learningMsg.x.wFreq + learningMsg.x.wAmp + learningMsg.x.wMean);
    learningMsg.y.perf     = (learningMsg.y.freqPerf*learningMsg.y.wFreq + learningMsg.y.ampPerf*learningMsg.y.wAmp  + learningMsg.y.meanPerf*learningMsg.y.wMean)/(learningMsg.y.wFreq + learningMsg.y.wAmp + learningMsg.y.wMean);
    learningMsg.z.perf     = (learningMsg.z.freqPerf*learningMsg.z.wFreq + learningMsg.z.ampPerf*learningMsg.z.wAmp  + learningMsg.z.meanPerf*learningMsg.z.wMean)/(learningMsg.z.wFreq + learningMsg.z.wAmp + learningMsg.z.wMean);

    learningMsg.perf       = (learningMsg.x.perf + learningMsg.y.perf + learningMsg.z.perf);
    if(learningMsg.perf == 0)  learningMsg.perf = 9999.;
    else                       learningMsg.perf = 1./learningMsg.perf;

    //State
    learningMsg.state.accX = imuMsg.linear_acceleration.x;
    learningMsg.state.accY = imuMsg.linear_acceleration.y;
    learningMsg.state.accZ = imuMsg.linear_acceleration.z;

    learningMsg.state.gyroX = imuMsg.angular_velocity.x;
    learningMsg.state.gyroY = imuMsg.angular_velocity.y;
    learningMsg.state.gyroZ = imuMsg.angular_velocity.z;

    learningMsg.state.angX  = imuMsg.orientation.x;
    learningMsg.state.angY  = imuMsg.orientation.y;

}



void HumanoidLearningNode::send2Topic()
{
    learningTopic.publish(learningMsg);
}

void HumanoidLearningNode::buildLearningMsg()
{
    learningMsg.header.stamp = ros::Time::now();
    learningMsg.x.bins.clear();
    learningMsg.y.bins.clear();
    learningMsg.z.bins.clear();
    for(int i  = 0; i < wUpdate;i++)
    {
        learningMsg.x.bins.push_back(MathUtils::norm(imuEulerMsgXFreq[i]));
        learningMsg.y.bins.push_back(MathUtils::norm(imuEulerMsgYFreq[i]));
        learningMsg.z.bins.push_back(MathUtils::norm(imuEulerMsgZFreq[i]));
    }
    learningMsg.x.freq = maxXFreq*df;
    learningMsg.y.freq = maxYFreq*df;
    learningMsg.z.freq = maxZFreq*df;
    learningMsg.x.amp = maxXAmp*(2./wUpdate);
    learningMsg.y.amp = maxYAmp*(2./wUpdate);
    learningMsg.z.amp = maxZAmp*(2./wUpdate);
    learningMsg.x.mean = meanX;
    learningMsg.y.mean = meanY;
    learningMsg.z.mean = meanZ;
    learningMsg.df = df;
    calcPerformance();
    //learningVec.push_back(learningMsg);
}


int HumanoidLearningNode::calcWindowPoints(double totalDt,double dt)
{
   int windowPoints = 0;
   windowPoints = (2*totalDt)/dt;
   return windowPoints;
}

void HumanoidLearningNode::calcFreqRefs(double tS,double tD,double delayAll)
{
    learningMsg.x.freqRef = 1./(2*(tS + tD)*delayAll);
    learningMsg.y.freqRef = learningMsg.x.freqRef;
    learningMsg.z.freqRef = learningMsg.x.freqRef*2.;
    //ROS_INFO("FREQ_REF_X: %4.3f  FREQ_REF_Y: %4.3f  FREQ_REF_Z: %4.3f",learningMsg.x.freqRef,learningMsg.y.freqRef,learningMsg.z.freqRef);
}


void HumanoidLearningNode::imuEulerCallback(const sensor_msgs::ImuPtr &msg)
{
    //if(test)
    if(1)
    {
        imuArrayMsg = imuArrayMsg.cshift(1);
        imuArrayMsg[imuArrayMsg.size() - 1] = *msg;
        /*if(testBeginFlag)
        {
            testBegin = msg->header.stamp;
            testFinal = testBegin + ros::Duration(testTime);
            testBeginFlag = false;
            ROS_INFO("TEST_BEGIN");
        }
        if(msg->header.stamp > testFinal)
        {
            test = false;
            ROS_WARN("TEST_FINISHED");
            testBeginFlag = true;
        }*/

        /*imuEulerMsgXData  = imuEulerMsgXData.cshift(1);
        imuEulerMsgXData[imuEulerMsgXData.size() - 1] = msg->orientation.x;
        //imuEulerMsgXData[imuEulerMsgXData.size() - 1] = msg->linear_acceleration.x;

        imuEulerMsgYData  = imuEulerMsgYData.cshift(1);
        imuEulerMsgYData[imuEulerMsgYData.size() - 1] = msg->orientation.y;
        //imuEulerMsgYData[imuEulerMsgYData.size() - 1] = msg->linear_acceleration.y;

        imuEulerMsgZData  = imuEulerMsgZData.cshift(1);
        //imuEulerMsgZData[imuEulerMsgZData.size() - 1] = sqrt(msg->linear_acceleration.x*msg->linear_acceleration.x + msg->linear_acceleration.y*msg->linear_acceleration.y + msg->linear_acceleration.z*msg->linear_acceleration.z);
        imuEulerMsgZData[imuEulerMsgZData.size() - 1] = msg->linear_acceleration.z;*/


        if(footGroundChanged)  imuCount++;


        if(imuCount == wPoints)
        {
            imuArrayBatchMsg = imuArrayMsg;
            /*imuEulerMsgXFreq  = imuEulerMsgXData;
            imuEulerMsgYFreq  = imuEulerMsgYData;
            imuEulerMsgZFreq  = imuEulerMsgZData;
            imuMsg            = *msg;*/
            imuInit  = true;
            //footGroundChanged = false;
            imuCount = 0;
        }
    }
}

bool HumanoidLearningNode::reset(std_srvs::Empty::Request  &cmd,
                         std_srvs::Empty::Response &res)
{
    resizeData(wUpdate,fUpdate,tUpdate);
    return true;
}

void HumanoidLearningNode::lipParamsCallback(const humanoid_msgs::LipParamsMsgPtr &msg)
{
    this->lipParamsMsg = *msg;
    calcFreqRefs(msg->tS,msg->tD,msg->delayAll);
    lipParamsUpdate = true;
}

void HumanoidLearningNode::humanoidPropsCallback(const humanoid_msgs::HumanoidPropertiesMsgPtr &msg)
{
    this->humanoidPropsMsg = *msg;
}

void HumanoidLearningNode::jointStateCallback(const humanoid_msgs::JointStateMsgPtr &msg)
{
    this->jointStateMsg = *msg;
}

void HumanoidLearningNode::lipFeedBackCallback(const humanoid_msgs::LipFeedBackPtr &msg)
{
    this->lipFeedbackMsg = *msg;
    if(this->lipFeedbackMsg.footGround != lastFootGround)  footGroundChanged = true;
    lastFootGround = this->lipFeedbackMsg.footGround;
}

void HumanoidLearningNode::forceRCallback(const geometry_msgs::WrenchStampedPtr &msg)
{
    msg->wrench.force.y = -msg->wrench.force.y;
    rForceData  = rForceData.cshift(1);
    rForceData[rForceData.size() - 1] = msg->wrench.force.y;

    if(lastFootGround == -5  && footGroundChanged)  forceCount++;
    if(forceCount == fPoints && lastFootGround == -5)
    {
        rForceFreq  = rForceData;
        forceInit  = true;
        forceCount = 0;
    }

}
void HumanoidLearningNode::forceLCallback(const geometry_msgs::WrenchStampedPtr &msg)
{
    msg->wrench.force.y = -msg->wrench.force.y;
    lForceData  = lForceData.cshift(1);
    lForceData[lForceData.size() - 1] = msg->wrench.force.y;

    if(lastFootGround == 5 && footGroundChanged)  forceCount++;
    if(forceCount == fPoints && lastFootGround == 5)
    {
        lForceFreq  = lForceData;
        forceInit  = true;
        forceCount = 0;
    }
}


void HumanoidLearningNode::torqueCallback(const ForceMsg::ConstPtr& r3, const ForceMsg::ConstPtr& r4, const ForceMsg::ConstPtr &rf,
                                          const ForceMsg::ConstPtr &l3, const ForceMsg::ConstPtr &l4, const ForceMsg::ConstPtr &lf)
{

    //ROS_WARN("R3: %4.3f  R4: %4.3f  RF: %4.3f",(float)r3->wrench.torque.x,(float)r4->wrench.torque.x,(float)rf->wrench.torque.x);
    //ROS_WARN("L3: %4.3f  L4: %4.3f  LF: %4.3f",(float)l3->wrench.torque.x,(float)l4->wrench.torque.x,(float)lf->wrench.torque.x);
    r3TorqueData  = r3TorqueData.cshift(1);
    r3TorqueData[r3TorqueData.size() - 1] = r3->wrench.torque.x;

    r4TorqueData  = r4TorqueData.cshift(1);
    r4TorqueData[r4TorqueData.size() - 1] = r4->wrench.torque.x;

    rfTorqueData  = rfTorqueData.cshift(1);
    rfTorqueData[rfTorqueData.size() - 1] = rf->wrench.torque.x;

    l3TorqueData  = l3TorqueData.cshift(1);
    l3TorqueData[l3TorqueData.size() - 1] = l3->wrench.torque.x;

    l4TorqueData  = l4TorqueData.cshift(1);
    l4TorqueData[l4TorqueData.size() - 1] = l4->wrench.torque.x;

    lfTorqueData  = lfTorqueData.cshift(1);
    lfTorqueData[lfTorqueData.size() - 1] = lf->wrench.torque.x;

    //if(lastFootGround == -5  && footGroundChanged)  torqueCount++;
    if(!torqueInit  && footGroundChanged)  torqueCount++;
    //if(torqueCount == tPoints && lastFootGround == -5)
    if(torqueCount == tPoints)
    {
        r3TorqueFreq  = r3TorqueData;
        r4TorqueFreq  = r4TorqueData;
        rfTorqueFreq  = rfTorqueData;

        l3TorqueFreq  = l3TorqueData;
        l4TorqueFreq  = l4TorqueData;
        lfTorqueFreq  = lfTorqueData;
        torqueInit  = true;
        torqueCount = 0;
    }
}
