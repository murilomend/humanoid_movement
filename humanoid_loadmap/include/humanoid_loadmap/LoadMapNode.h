#ifndef LOADMAPNODE_H
#define LOADMAPNODE_H


#include <iostream>
#include <fstream>
#include <string>
#include <unistd.h>
#include "humanoid_loadmap/Loader.h"
#include "humanoid_loadmap/Mapping.h"
#include "eigen_conversions/eigen_msg.h"


#include "ros/ros.h"
#include <ros/package.h>


#include <std_srvs/Empty.h>
#include <humanoid_msgs/IDMapMsg.h>
#include <humanoid_msgs/LoadMapConfigsSrv.h>



class LoadMapNode
{
public:


	//Members
	std::string fEnum;
	std::string fUrdf;
	std::string fRobot;
	std::string fIk;
	
	bool fEnumFlag;
	bool fUrdfFlag;
	bool fRobotFlag;
	bool fIkFlag;
	
	Mapping     map;
	
    //ROS Node
    ros::NodeHandle    nh;
    ros::NodeHandle    nh_private;


    
    //ROS  Services
    ros::ServiceServer loadSrv;
    ros::ServiceServer cleanSrv;
    ros::ServiceServer buildSrv;


	LoadMapNode(){};
    LoadMapNode(ros::NodeHandle nh_,ros::NodeHandle nh_private_);
    ~LoadMapNode();

    //Methods
	bool load(humanoid_msgs::LoadMapConfigsSrv::Request  &load,
			  humanoid_msgs::LoadMapConfigsSrv::Response &res);
	bool build(std_srvs::Empty::Request  &build,
	          std_srvs::Empty::Response &res);
	bool clean(std_srvs::Empty::Request  &clean,
	          std_srvs::Empty::Response &res);
	          
	
};

#endif // LOADMAPNODE_H

