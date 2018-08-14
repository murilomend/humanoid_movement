#include "humanoid_loadmap/Loader.h"

Loader::Loader()
{

}


void Loader::load(const char* fEnum,const char* fRobot,const char* fUrdf,const char* fIk,Mapping &map)
{
    loadEnumConfig(fEnum,map);
    loadRobotConfig(fRobot,map);
    loadURDF(fUrdf,map);
    loadIKConfig(fIk,map);
}

void Loader::loadEnumConfig(const char* filename,Mapping &map)
{
    xmlNodePtr enumNode;
    xmlNodePtr propertyNode;
    xmlNodePtr jointNode;
    xmlNodePtr bodyNode;
    enumNode = file2xmlnode(filename);
    if(!isnode(enumNode, (char*)"enum")) throw WrongNode();
    //First the bodies
    for(propertyNode = enumNode->children; propertyNode != NULL; propertyNode = propertyNode->next)
    {
        if(isnode(propertyNode, (char*)"bodyGlobal"))
        {
            for(bodyNode = propertyNode->children;bodyNode != NULL;bodyNode = bodyNode->next)
            {
                if(isnode(bodyNode, (char*)"body"))
                {
                    ///Joint stuff
                    std::string name     = parseprop2string(bodyNode, "name", "");
                    int         id       = parseprop2int(bodyNode, (char*)"id", 0);
                    map.bodyMap.insert(std::make_pair(name,id));
                }
            }
        }
    }
    //Then the joints
    for(propertyNode = enumNode->children; propertyNode != NULL; propertyNode = propertyNode->next)
    {
        //Found jointGlobal
        if(isnode(propertyNode, (char*)"jointGlobal"))
        {
            // Sweepping the jointGlobal properties
            for(jointNode = propertyNode->children;jointNode != NULL;jointNode = jointNode->next)
            {
                if(isnode(jointNode, (char*)"joint"))
                {
                    ///Joint stuff
                    std::string name = parseprop2string(jointNode, "name", "");
                    int          id  = parseprop2int(jointNode, (char*)"id", -1);
                    int        body  = map.bodyMap[parseprop2string(jointNode,"body", "")];
                    map.enumMap.insert(std::make_pair(name,id));
                    map.jointBodyMap.insert(std::make_pair(name,body));

                }
            }
        }
    }
    xmlFreeDoc(enumNode->doc);
}

void Loader::loadRobotConfig(const char* filename,Mapping &map)
{
    //Nodes
    xmlNodePtr robotNode;
    xmlNodePtr propertyNode;
    xmlNodePtr dimensionsNode;
    xmlNodePtr positionsNode;
    xmlNodePtr jointsIDsNode;
    robotNode = file2xmlnode(filename);
    if(!isnode(robotNode, (char*)"robot")) throw WrongNode();
    for(propertyNode = robotNode->children; propertyNode != NULL; propertyNode = propertyNode->next)
    {
        //Found JointIDs properties
        if(isnode(propertyNode, (char*)"jointIDs"))
        {
            // Sweepping the Dimension properties
            for(jointsIDsNode = propertyNode->children;jointsIDsNode != NULL;jointsIDsNode = jointsIDsNode->next)
            {
                if(isnode(jointsIDsNode, (char*)"joint"))
                {
                    ///Joint stuff
                    std::string name     = parseprop2string(jointsIDsNode, "name", "");
                    int         id       = parseprop2int(jointsIDsNode, (char*)"id", 0);
                    int         dir      = parseprop2int(jointsIDsNode, (char*)"dir",1);
                    int      offset      = parseprop2int(jointsIDsNode, (char*)"offset", 0);
                    int      motorRef    = parseprop2int(jointsIDsNode, (char*)"motorRef", 0);
                    int      motorCCWLim = parseprop2int(jointsIDsNode, (char*)"motorCCWLim", 0);
                    int      motorCWLim  = parseprop2int(jointsIDsNode, (char*)"motorCWLim", 0);


                    //std::cout << "NAME: " << name << " id: " << id << " dir: " << dir << " offset: " << offset << std::endl;
                    map.idRobotMap.insert(std::make_pair(name,id));
                    map.dirRobotMap.insert(std::make_pair(name,dir));
                    map.offsetRobotMap.insert(std::make_pair(name,offset));
                    map.motorRefMap.insert(std::make_pair(name,motorRef));
                    map.motorCCWLimMap.insert(std::make_pair(name,motorCCWLim));
                    map.motorCWLimMap.insert(std::make_pair(name,motorCWLim));
                }
                ///....
            }
        }
    }
}

void Loader::loadURDF(const char* filename,Mapping &map)
{
    RigidBodyDynamics::Model   urdfModel = RigidBodyDynamics::Model();
    if (!RigidBodyDynamics::Addons::URDFReadFromFile (filename, &urdfModel, false))
    {
        std::cerr << "Error loading model urdf" << std::endl;
        abort();
    }

    std::string bodyName;
    std::vector<std::string> jointNames;
    int         bodyId = 0;
    int         jointId = 0;
    int         dof = 0;
    std::vector<Eigen::Vector2i> idPairs;
    Eigen::Vector2i idPair;
    for (std::map<std::string,unsigned int>::iterator it=urdfModel.mBodyNameMap.begin(); it!=urdfModel.mBodyNameMap.end(); ++it)
    {
        bodyName   = it->first;
        bodyId     = it->second;
        dof        = urdfModel.mJoints[bodyId].mDoFCount;
        //The root = 0 will be excluded
        if(dof > 1) //<--- to exclude the root
        {
            for(int i = 0; i < dof; i++)
            {
                jointNames.push_back(bodyName +   "_" +  std::to_string(i));
                idPair(0) = bodyId - 1; //<--- to exclude the root
                idPair(1) = dof;
                idPairs.push_back(idPair);
            }
        }
        else if(dof == 1) //<--- to exclude the root
        {
            jointNames.push_back(bodyName);
            idPair(0) = bodyId - 1; //<--- to exclude the root
            idPair(1) = dof;
            idPairs.push_back(idPair);
        }
    }
    fixIds(idPairs);
    //Inserts the new ID order on the map
    for(int i = 0; i < int(idPairs.size());i++) map.idUrdfMap.insert(std::make_pair(jointNames[i],idPairs[i](0)));
    
	Eigen::VectorXd Q = Eigen::VectorXd::Zero(map.idUrdfMap.size());
    //Getting the axis directions
    for(unsigned int i = 0;i < int(urdfModel.mBodies.size());i++)
    {
        std::string name      = urdfModel.GetBodyName(i);
        Eigen::MatrixXd baseM = Eigen::MatrixXd(6,1);
        baseM << *urdfModel.mJoints[i].mJointAxes;
        Eigen::Vector3d baseV = Eigen::Vector3d(baseM(0),baseM(1),baseM(2));
        Eigen::Vector3d pos0 = RigidBodyDynamics::CalcBodyToBaseCoordinates(urdfModel,Q,i,Eigen::Vector3d(0,0,0));
        Eigen::Vector3d pos1 = RigidBodyDynamics::CalcBodyToBaseCoordinates(urdfModel,Q,i,baseV);
        Eigen::Vector3d axis = pos1 - pos0;

        double sum = axis(0) + axis(1) + axis(2);
        double dir = (sum >= 0)?(+1):(-1);
        int    offset = 0; ///<----------------
        map.dirUrdfMap.insert(std::make_pair(name,dir));
        map.offsetUrdfMap.insert(std::make_pair(name,offset));
    }

}


void Loader::loadIKConfig(const char* filename,Mapping &map)
{
    xmlNodePtr ikNode;
    xmlNodePtr jointNode;
    ikNode = file2xmlnode(filename);
    if(!isnode(ikNode, (char*)"ik")) throw WrongNode();
    for(jointNode = ikNode->children; jointNode != NULL; jointNode = jointNode->next)
    {
        if(isnode(jointNode, (char*)"joint"))
        {
            ///Joint stuff
            std::string name     = parseprop2string(jointNode, "name", "");
            int         id       = parseprop2int(jointNode, (char*)"id", 0);
            int         dir      = parseprop2int(jointNode, (char*)"dir",1);
            int      offset      = parseprop2int(jointNode, (char*)"offset", 0);
            map.idIKMap.insert(std::make_pair(name,id));
            map.dirIKMap.insert(std::make_pair(name,dir));
            map.offsetIKMap.insert(std::make_pair(name,offset));
        }
    }
    xmlFreeDoc(ikNode->doc);
}


void Loader::fixIds(std::vector<Eigen::Vector2i> &idPairs)
{
    /*for(int i = 0;i < int(idPairs.size());i++)
    {
        std::cout << idPairs[i](0) << " " << idPairs[i](1) << std::endl;
    }*/
    std::cout << std::endl;
    int index = 0;
    int min   = 999; //<==== Max ID Value?
    int dof   = 0;
    do
    {
        index = 0;
        min   = 999; //<==== Max ID Value?
        dof   = 0;
        for(int i = 0;i < int(idPairs.size());i++)
        {
            //std::cout << idPairs[i](0) << " " << idPairs[i](1) << std::endl;
            //It finds the first occurence of the minimum value with more than 1 DOF
            if((idPairs[i](0) < min)&&(idPairs[i](1) > 1))
            {
                dof   = idPairs[i](1);
                index = i;
                min   = idPairs[i](0);
            }
        }
        int inc = 0;
        for(int i = 0;i < int(idPairs.size());i++)
        {
            if(idPairs[i](0) == min)
            {
                idPairs[i](0) = min + inc;
                idPairs[i](1) = 1;
                inc++;
            }
            else if(idPairs[i](0) > min)
            {
                idPairs[i](0) += dof-1;
            }
        }
    }while(min < 999);

    /*for(int i = 0;i < int(idPairs.size());i++)
    {
        std::cout << idPairs[i](0) << " " << idPairs[i](1) << std::endl;
    }*/
}
