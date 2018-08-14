#include "humanoid_model/RobotModel.h"

RobotModel::RobotModel()
{

}

RobotModel::~RobotModel()
{

}

RobotModel::RobotModel(const char* filename,unsigned int robotDOF,unsigned int ikDOF)
{
    loadURDF(filename);
    initVars(robotDOF,ikDOF);
}

void RobotModel::loadURDF(const char* filename)
{
    urdfModel = RigidBodyDynamics::Model();
    if (!RigidBodyDynamics::Addons::URDFReadFromFile (filename, &urdfModel, false))
    {
        std::cerr << "Error loading model urdf" << std::endl;
        abort();
    }
}

void RobotModel::initVars(unsigned int robotDOF,unsigned int ikDOF)
{
    this->urdfDOF  = urdfModel.dof_count;
    this->robotDOF = robotDOF;
    this->ikDOF    = ikDOF;
}

void RobotModel::calcDynInv(const JointState &qState,bool update)
{
    JointState qStateRBDL = JointState(qState.nJoints,URDF_IDS);// = qState.toRBDL();
    Mapping::convert(map,qState,qStateRBDL);

    RigidBodyDynamics::InverseDynamics(urdfModel,qStateRBDL.pos,qStateRBDL.vel,qStateRBDL.acc,qStateRBDL.torq);
    if(update) qState;// = qStateRBDL.toROBOT();
}

BodyPointState RobotModel::calcCom(JointState &qState,BodyPointState &ref,bool update)
{
    JointState qStateRBDL = JointState(urdfDOF,URDF_IDS);// = qState.toRBDL();
    Mapping::convert(map,qState,qStateRBDL);
    RigidBodyDynamics::Math::Vector3d comPos;///<====
    RigidBodyDynamics::Math::Vector3d comVel;///<====
    RigidBodyDynamics::Math::Vector3d angMomentum;///<====
    //RigidBodyDynamics::Utils::CalcCenterOfMass(urdfModel,qStateRBDL.pos,qStateRBDL.vel,mass,pos);///<====
    std::cout << "COM_POS: " << std::endl << comPos << std::endl;
    RigidBodyDynamics::Utils::CalcCenterOfMass(urdfModel,qStateRBDL.pos,qStateRBDL.vel,mass,comPos,&comVel,&angMomentum);///<====
    std::cout << "COM_POS: " << std::endl << comPos << std::endl;
    std::cout << "COM_VEL: " << std::endl << comVel << std::endl;
    std::cout << "COM_MOM: " << std::endl << angMomentum << std::endl;
    if(update)
    {
        Mapping::convert(map,qStateRBDL,qState);
        //qState = qStateRBDL;//.toROBOT();
        com    = ref;
    }
    return ref;
}
