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

#include "../include/movcreator_qt/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace movcreator_qt {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
    {}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init() {
	ros::init(init_argc,init_argv,"movcreator_qt");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;

	// Add your ros communications here.
    interfaceCli = n.serviceClient<humanoid_msgs::InterfaceSrv>("humanoid_interface/cmd");
    motorSetCli = n.serviceClient<humanoid_msgs::MotorSetSrv>("motor_set_control/cmd");

	start();
	return true;
}

bool QNode::init(const std::string &master_url, const std::string &host_url) {
	std::map<std::string,std::string> remappings;
	remappings["__master"] = master_url;
	remappings["__hostname"] = host_url;
	ros::init(remappings,"movcreator_qt");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;

	// Add your ros communications here.

	start();
	return true;
}

void QNode::run() {
    ros::Rate loop_rate(25);
	while ( ros::ok() ) {
        ros::spinOnce();
        loop_rate.sleep();
	}
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}


void QNode::log( const LogLevel &level, const std::string &msg) {
	logging_model.insertRows(logging_model.rowCount(),1);
	std::stringstream logging_model_msg;
	switch ( level ) {
		case(Debug) : {
				ROS_DEBUG_STREAM(msg);
				logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Info) : {
				ROS_INFO_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Warn) : {
				ROS_WARN_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Error) : {
				ROS_ERROR_STREAM(msg);
				logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Fatal) : {
				ROS_FATAL_STREAM(msg);
				logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
				break;
		}
	}
	QVariant new_row(QString(logging_model_msg.str().c_str()));
	logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
    Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

void QNode::setTorque(std::vector<int> ids, std::vector<bool> torque)
{
    interfaceSrv.request.setTorque.clear();

    interfaceSrv.request.toGazebo = false;
    interfaceSrv.request.toRobot = true;
    interfaceSrv.request.scanMotors = false;
    interfaceSrv.request.getPID = false;
    interfaceSrv.request.getMotorPos = false;
    interfaceSrv.request.send2Motor = false;

    for(unsigned int i = 0; i < torque.size() ; i++ )
    {
        interfaceSrv.request.setTorque.push_back(torque[i]);
//        interfaceSrv.request.motorIds.push_back(ids[i]);
    }

    interfaceCli.call(interfaceSrv);
}

void QNode::scanMotor()
{
    interfaceSrv.request.toGazebo = false;
    interfaceSrv.request.toRobot = true;
    interfaceSrv.request.scanMotors = false;
    interfaceSrv.request.getPID = false;
    interfaceSrv.request.getMotorPos = true;
    interfaceSrv.request.send2Motor = false;

    if(interfaceCli.call(interfaceSrv))
    {
        motorsPosition.clear();
        for(unsigned int i = 0; i < interfaceSrv.response.motorState.pos.size();i++)
            motorsPosition.push_back((double)interfaceSrv.response.motorState.pos[i]);
        Q_EMIT  scanFinished();
    }
}

void QNode::scanActiveMotors()
{
    interfaceSrv.request.toGazebo = false;
    interfaceSrv.request.toRobot = true;
    interfaceSrv.request.scanMotors = true;
    interfaceSrv.request.getPID = false;
    interfaceSrv.request.getMotorPos = true;
    interfaceSrv.request.send2Motor = true;

    if(interfaceCli.call(interfaceSrv))
    {
        activeMotors.clear();
        for(unsigned int i = 0; i < interfaceSrv.response.ids.size();i++)
            activeMotors.push_back((int)interfaceSrv.response.ids[i]);
        Q_EMIT  idScanFinished();
    }
}

void QNode::playPage(std::string path)
{
    motorSetSrv.request.toGazebo = false;
    motorSetSrv.request.toRobot = true;
    motorSetSrv.request.page = path;
    motorSetSrv.request.pose.clear();


    interfaceSrv.request.toGazebo = false;
    interfaceSrv.request.toRobot = true;
    interfaceSrv.request.scanMotors = false;
    interfaceSrv.request.getPID = false;
    interfaceSrv.request.getMotorPos = false;
    interfaceSrv.request.send2Motor = true;
    interfaceCli.call(interfaceSrv);


    motorSetCli.call(motorSetSrv);
}

void QNode::goPose(std::string path)
{
    motorSetSrv.request.toGazebo = false;
    motorSetSrv.request.toRobot = true;
    motorSetSrv.request.pose = path;
    motorSetSrv.request.page.clear();

    interfaceSrv.request.toGazebo = false;
    interfaceSrv.request.toRobot = true;
    interfaceSrv.request.scanMotors = false;
    interfaceSrv.request.getPID = false;
    interfaceSrv.request.getMotorPos = false;
    interfaceSrv.request.send2Motor = true;
    interfaceCli.call(interfaceSrv);

    motorSetCli.call(motorSetSrv);
}

std::vector<int> QNode::getActiveMotors() const
{
    return activeMotors;
}

std::vector<double> QNode::getMotorsPosition() const
{
    return motorsPosition;
}



}  // namespace movcreator_qt
