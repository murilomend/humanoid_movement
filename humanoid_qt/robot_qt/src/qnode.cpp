/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/robot_qt/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace robot_qt {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
	{}

QNode::~QNode()
{
    if(ros::isStarted())
    {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init()
{
	ros::init(init_argc,init_argv,"robot_qt");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
    gazeboResetCli    = n.serviceClient<std_srvs::Empty>("/gazebo/reset_world");
    walkParamCli      = n.serviceClient<humanoid_msgs::LipParamsSrv>("humanoid_walking/walking_params");
    walkCmdCli        = n.serviceClient<humanoid_msgs::LipCmdSrv>("humanoid_walking/walking_cmd");
    walkCtrlCli       = n.serviceClient<humanoid_msgs::LipCtrlSrv>("humanoid_walking/walking_ctrl");
    //Registering Subscribers
    readImu.reset(new SubImu(n,"/imu/data",1));
    readImu->registerCallback(&QNode::imuCallback, this);

    readFFT.reset(new SubFFT(n,"humanoid_walking/fftIMU",1));
    readFFT->registerCallback(&QNode::fftCallback, this);

    readLearning.reset(new SubLearning(n,"humanoid_learning/status",1));
    readLearning->registerCallback(&QNode::learningCallback, this);

    readParams.reset(new SubParams(n,"humanoid_walking/walking_params_state",1));
    readParams->registerCallback(&QNode::lipParamsCallback, this);

    readHumanoidProps.reset(new SubHumanoidProps(n,"humanoid_model/properties",1));
    readHumanoidProps->registerCallback(&QNode::humanoidPropsCallback, this);




	start();
	return true;
}

bool QNode::shutdown()
{
    ros::shutdown();
    return true;
}


void QNode::run()
{
    ros::Rate loop_rate(125);
    while ( ros::ok() )
    {
		ros::spinOnce();
		loop_rate.sleep();
	}
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

void QNode::sendModelParams(double squat,double open,double incl)
{
    conf.doubles.clear();
    double_param.name = "squat";
    double_param.value = squat;
    conf.doubles.push_back(double_param);
    double_param.name = "open";
    double_param.value = open;
    conf.doubles.push_back(double_param);
    double_param.name = "incl";
    double_param.value = incl;
    conf.doubles.push_back(double_param);
    srv_req.config = conf;
    ros::service::call("/humanoid_model/set_parameters", srv_req, srv_resp);

}

void QNode::imuCallback(const sensor_msgs::ImuPtr &imuMsg)
{
    //std::cout << "IMU_ACC: "<< std::endl << imuMsg->linear_acceleration << std::endl;
    //log(Info,std::string("IMU_ACC: ")+std::to_string(imuMsg->linear_acceleration.x));
    //std::vector<double> acc(3),gyro(3);
    Q_EMIT   sendImuUpdate(*imuMsg);
}

void QNode::fftCallback(const humanoid_msgs::FFTMsgPtr &fftMsg)
{
    Q_EMIT   sendFFTUpdate(*fftMsg);

}

void QNode::learningCallback(const humanoid_msgs::LearningMsgPtr &learningMsg)
{
    Q_EMIT   sendLearningUpdate(*learningMsg);

}

void QNode::lipParamsCallback(const humanoid_msgs::LipParamsMsgPtr &paramsMsg)
{
    Q_EMIT  sendParamsUpdate(*paramsMsg);
}

void QNode::humanoidPropsCallback(const humanoid_msgs::HumanoidPropertiesMsgPtr &humanoidPropsMsg)
{
    Q_EMIT  sendHumanoidPropsUpdate(*humanoidPropsMsg);

}

void QNode::sendWalkingParams(double tS,double tD, double stepH,double zCCorr)
{
    walkParamSrv.request.tS     = tS;
    walkParamSrv.request.tD     = tD;
    walkParamSrv.request.stepH  = stepH;
    walkParamSrv.request.zCCorr = zCCorr;
    walkParamCli.call(walkParamSrv);
}

void QNode::sendWalkingCmd(double vx, double vy, double vz, bool walk_flag)
{
    walkCmdSrv.request.vx         = vx;
    walkCmdSrv.request.vy         = vy;
    walkCmdSrv.request.vz         = vz;
    walkCmdSrv.request.walk_flag  = walk_flag;
    walkCmdCli.call(walkCmdSrv);
}

void QNode::sendWalkingCtrl(double delayR, double delayL, double delayAll, double kpLeg, double kpFoot, double kpGround)
{
    walkCtrlSrv.request.delayR         = delayR;
    walkCtrlSrv.request.delayL         = delayL;
    walkCtrlSrv.request.delayAll       = delayAll;
    /*walkCtrlSrv.request.kpLeg          = kpLeg;
    walkCtrlSrv.request.kpFoot         = kpFoot;
    walkCtrlSrv.request.kpGround       = kpGround;
    walkCtrlSrv.request.rFootFac.x     = 1;
    walkCtrlSrv.request.rFootFac.y     = 1;
    walkCtrlSrv.request.rFootFac.z     = 1;
    walkCtrlSrv.request.lFootFac.x     = 1;
    walkCtrlSrv.request.lFootFac.y     = 1;
    walkCtrlSrv.request.lFootFac.z     = 1;
    walkCtrlSrv.request.walk_ctrl      = false;*/
    walkCtrlCli.call(walkCtrlSrv);
}




void QNode::sendReset()
{
    gazeboResetCli.call(gazeboResetSrv);
}


}  // namespace robot_qt
