/**
 * @file /include/movcreator_qt/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef movcreator_qt_QNODE_HPP_
#define movcreator_qt_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <string>
#include <QThread>
#include <QStringListModel>
#include <vector>
#include <message_filters/subscriber.h>
#include <humanoid_msgs/InterfaceSrv.h>
#include "humanoid_msgs/MotorSetSrv.h"


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace movcreator_qt {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	bool init(const std::string &master_url, const std::string &host_url);
	void run();

	/*********************
	** Logging
	**********************/
	enum LogLevel {
	         Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };

	QStringListModel* loggingModel() { return &logging_model; }
	void log( const LogLevel &level, const std::string &msg);
    std::vector<double> getMotorsPosition() const;

    std::vector<int> getActiveMotors() const;

public Q_SLOTS:
    void setTorque(std::vector<int> ids, std::vector<bool> torque);
    void scanMotor();
    void scanActiveMotors();
    void playPage(std::string path);
    void goPose(std::string path);
Q_SIGNALS:
	void loggingUpdated();
    void rosShutdown();
    void scanFinished();
    void idScanFinished();
private:
	int init_argc;
	char** init_argv;
    ros::ServiceClient interfaceCli;
    ros::ServiceClient motorSetCli;
    humanoid_msgs::InterfaceSrv interfaceSrv;
    humanoid_msgs::MotorSetSrv motorSetSrv;
    std::vector<double> motorsPosition;
    std::vector<int> activeMotors;
    QStringListModel logging_model;
};

}  // namespace movcreator_qt

#endif /* movcreator_qt_QNODE_HPP_ */
