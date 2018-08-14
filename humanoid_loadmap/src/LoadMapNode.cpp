#include "humanoid_loadmap/LoadMapNode.h"


LoadMapNode::LoadMapNode(ros::NodeHandle nh_, ros::NodeHandle nh_private_) : nh(nh_) , nh_private(nh_private_)
{
	
	fEnumFlag  = true;
	fUrdfFlag  = true;
	fRobotFlag = true;
	fIkFlag    = true;

	//Parameter Server
    if (!nh.getParam("fEnum", fEnum))
      fEnumFlag  = false;
    if (!nh.getParam ("fUrdf", fUrdf))
      fUrdfFlag  = false;
    if (!nh.getParam ("fRobot", fRobot))
      fRobotFlag = false;
    if (!nh.getParam ("fIk", fIk))
      fIkFlag    = false;

    //Server Stuff
    loadSrv     = nh.advertiseService("humanoid_loadmap/load",&LoadMapNode::load,this);
    buildSrv    = nh.advertiseService("humanoid_loadmap/build",&LoadMapNode::build,this);
    cleanSrv    = nh.advertiseService("humanoid_loadmap/clean",&LoadMapNode::clean,this);
    
    
	if(fEnumFlag && fUrdfFlag && fRobotFlag && fIkFlag)
    {		
		Loader::load(fEnum.c_str(),fRobot.c_str(),fUrdf.c_str(),fIk.c_str(),map);
		map.buildMap();
		map.print();
        ROS_INFO("[HUMANOID_LOADMAP]  ID Map is loaded and is been published on  humanoid_loadmap/id_map");
        ROS_INFO("[HUMANOID_LOADMAP] Service: humanoid_loadmap/load can be used for loading new id map files");
	}
	else
	{
        ROS_FATAL("[HUMANOID_LOADMAP] It is missing config files");
        ROS_FATAL("[HUMANOID_LOADMAP] Service: humanoid_loadmap/load can be used for loading id map files");
	}
	
	


}
LoadMapNode::~LoadMapNode()
{
}


bool LoadMapNode::load(humanoid_msgs::LoadMapConfigsSrv::Request  &load,
			  humanoid_msgs::LoadMapConfigsSrv::Response &res)
{
    if(load.update)
    {
		Loader::load(load.fEnum.c_str(),load.fRobot.c_str(),load.fUrdf.c_str(),load.fIk.c_str(),map);
		map.buildMap();
	}		
	res.idMap.header.stamp = ros::Time::now();
    tf::matrixEigenToMsg(map.map,res.idMap.map);


    res.idMap.jNames.resize(map.enumMap.size());
    res.idMap.robotDOF = map.idRobotMap.size();
    res.idMap.urdfDOF  = map.idUrdfMap.size();
    res.idMap.ikDOF    = map.idIKMap.size();
    std::string name;
    int         id;
    for (std::map<std::string,unsigned int>::iterator it=map.enumMap.begin(); it!=map.enumMap.end(); ++it)
    {
        name = it->first;
        id   = it->second;
        res.idMap.jNames[id] = name;
    }
    return true;
}
			  
bool LoadMapNode::build(std_srvs::Empty::Request  &build,
	          std_srvs::Empty::Response &res)
{
	map.buildMap();
	return true;
}
bool LoadMapNode::clean(std_srvs::Empty::Request  &clean,
	          std_srvs::Empty::Response &res)
{
	return true;

}
