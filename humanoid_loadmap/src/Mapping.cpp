#include "humanoid_loadmap/Mapping.h"


Mapping::~Mapping()
{

}

void  Mapping::buildMap()
{
    std::string name;
    int         id;
    int         xSize = MAP_TYPES_SIZE;
    int         ySize = enumMap.size();
    map = Eigen::MatrixXi(ySize,xSize);
    //std::cout << "xSize: " << xSize << " ySize: " << ySize << std::endl;
    for (std::map<std::string,unsigned int>::iterator it=enumMap.begin(); it!=enumMap.end(); ++it)
    {
        name = it->first;
        id   = it->second;
        //std::cout << name << " => " << id << '\n';
        //{ROBOT_IDS,URDF_IDS,IK_IDS,ROBOT_DIR,URDF_DIR,IK_DIR, ROBOT_OFFSET,URDF_OFFSET,IK_OFFSET, MAP_TYPES_SIZE};

        map(id,ROBOT_IDS)     = (idRobotMap.find(name)     == idRobotMap.end())?(-1):(idRobotMap[name]);
        map(id,URDF_IDS)      = (idUrdfMap.find(name)      == idUrdfMap.end())?(-1):(idUrdfMap[name]);
        map(id,IK_IDS)        = (idIKMap.find(name)        == idIKMap.end())?(-1):(idIKMap[name]);
        map(id,JOINT_BODY)    = (jointBodyMap.find(name)   == jointBodyMap.end())?(-1):(jointBodyMap[name]);
        map(id,ROBOT_DIR)     = (dirRobotMap.find(name)    == dirRobotMap.end())?(0):(dirRobotMap[name]);
        map(id,URDF_DIR)      = (dirUrdfMap.find(name)     == dirUrdfMap.end())?(0):(dirUrdfMap[name]);
        map(id,IK_DIR)        = (dirIKMap.find(name)       == dirIKMap.end())?(0):(dirIKMap[name]);
        map(id,ROBOT_OFFSET)  = (offsetRobotMap.find(name) == offsetRobotMap.end())?(0):(offsetRobotMap[name]);
        map(id,URDF_OFFSET)   = (offsetUrdfMap.find(name)  == offsetUrdfMap.end())?(0):(offsetUrdfMap[name]);
        map(id,IK_OFFSET)     = (offsetIKMap.find(name)    == offsetIKMap.end())?(0):(offsetIKMap[name]);
        map(id,MOTOR_REF)     = (motorRefMap.find(name)    == motorRefMap.end())?(0):(motorRefMap[name]);
        map(id,MOTOR_CCWLIM)  = (motorCCWLimMap.find(name) == motorCCWLimMap.end())?(0):(motorCCWLimMap[name]);
        map(id,MOTOR_CWLIM)   = (motorCWLimMap.find(name)  == motorCWLimMap.end())?(0):(motorCWLimMap[name]);
    }
}


void Mapping::print()
{
    std::string name;
    int         id;
    std::cout << "____________________________________________________________________" << std::endl;
    std::cout << "|      JOINT(ID)       |JOINT_BODY|ROBOT_IDS | URDF_IDS |  IK_IDS  |" << std::endl;
    std::cout << "|======================|==========|==========|==========|==========|" << std::endl;
    for (std::map<std::string,unsigned int>::iterator it=enumMap.begin(); it!=enumMap.end(); ++it)
    {
        name = it->first;
        id   = it->second;

        std::cout << "| " << std::setw(17) << name << "(" << std::setw(2) << id << ")|"
                  << std::setw(7)  << map(id,JOINT_BODY)     << "   |"
                  << std::setw(7)  << map(id,ROBOT_IDS)   << "   |"
                  << std::setw(7)  << map(id,URDF_IDS)    << "   |"
                  << std::setw(7)  << map(id,IK_IDS)      << "   |" <<  std::endl;
        std::cout << "--------------------------------------------------------------------" << std::endl;
    }

    std::cout << "__________________________________________________________________________________________" << std::endl;
    std::cout << "|      JOINT(ID)       | ROBOT_DIR| URDF_DIR |  IK_DIR  |ROBOT_OFFS|URDF_OFFS | IK_OFFSET|" << std::endl;
    std::cout << "|======================|==========|==========|==========|==========|==========|==========|" << std::endl;
    for (std::map<std::string,unsigned int>::iterator it=enumMap.begin(); it!=enumMap.end(); ++it)
    {
        name = it->first;
        id   = it->second;

        std::cout << "| " << std::setw(17) << name << "(" << std::setw(2) << id << ")|"
                  << std::setw(7)  << map(id,ROBOT_DIR)   << "   |"
                  << std::setw(7)  << map(id,URDF_DIR)    << "   |"
                  << std::setw(7)  << map(id,IK_DIR)      << "   |"
                  << std::setw(7)  << map(id,ROBOT_OFFSET)<< "   |"
                  << std::setw(7)  << map(id,URDF_OFFSET) << "   |"
                  << std::setw(7)  << map(id,IK_OFFSET)   << "   |"  <<  std::endl;
        std::cout << "------------------------------------------------------------------------------------------" << std::endl;
    }
    std::cout << "____________________________________________________________________" << std::endl;
    std::cout << "|      JOINT(ID)       |ROBOT_IDS |MOTOR_REF |CCW_LIMIT |CW_LIMIT  |" << std::endl;
    std::cout << "|======================|==========|==========|==========|==========|" << std::endl;
    for (std::map<std::string,unsigned int>::iterator it=enumMap.begin(); it!=enumMap.end(); ++it)
    {
        name = it->first;
        id   = it->second;

        std::cout << "| " << std::setw(17) << name << "(" << std::setw(2) << id << ")|"
                  << std::setw(7)  << map(id,ROBOT_IDS)   << "   |"
                  << std::setw(7)  << map(id,MOTOR_REF)    << "   |"
                  << std::setw(7)  << map(id,MOTOR_CCWLIM) << "   |"
                  << std::setw(7)  << map(id,MOTOR_CWLIM)   << "   |" <<  std::endl;
        std::cout << "--------------------------------------------------------------------" << std::endl;
    }
    std::cout << "cols: " << int(map.cols()) << std::endl;
    std::cout << "rows: " << int(map.rows()) << std::endl;
}
void   Mapping::convert(const Eigen::MatrixXi  &map,const JointState& stFrom, JointState& stTo, int ikType)
{
    unsigned int typeFrom = stFrom.type;
    unsigned int typeTo   = stTo.type;

    for(int i = 0; i < int(map.rows()) ;i++)
    {
        int indexFrom  = map(i,typeFrom);
        int indexTo    = map(i,typeTo);
        int type       = map(i,JOINT_BODY);

        if(( ikType == -1 && indexFrom != -1 && indexTo != -1)||(ikType != -1 && type == ikType))
        {
            int signalFrom = map(i,typeFrom+3);
            int signalTo   = map(i,typeTo+3);
            int offsetFrom = map(i,typeFrom+6);
            int offsetTo   = map(i,typeTo+6);
            //std::cout << "INdeX_FrOM: " << indexFrom <<  " INdeX_tO: " << indexTo << std::endl;
            stTo.pos[indexTo]  = stFrom.pos[indexFrom]*signalTo*signalFrom + offsetFrom + offsetTo;///<=====
            stTo.vel[indexTo]  = stFrom.vel[indexFrom]*signalTo*signalFrom;
            stTo.acc[indexTo]  = stFrom.acc[indexFrom]*signalTo*signalFrom;
            stTo.torq[indexTo] = stFrom.torq[indexFrom]*signalTo*signalFrom;
        }
    }
}


