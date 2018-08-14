#include "humanoid_general/State.h"


JointState::JointState()
{

}

JointState::JointState(int nJoints,int type,int source)
{
    this->nJoints = nJoints;
    pos  = Eigen::VectorXd::Zero(nJoints);
    vel  = Eigen::VectorXd::Zero(nJoints);
    acc  = Eigen::VectorXd::Zero(nJoints);
    torq = Eigen::VectorXd::Zero(nJoints);
    dt   = 0;
    this->type   = type;
    this->source = source;
}


JointState::~JointState()
{


}

BodyPointState::BodyPointState()
{
    pos    = Eigen::Vector3d::Zero();
    vel    = Eigen::Vector3d::Zero();
    acc    = Eigen::Vector3d::Zero();
    rot    = Eigen::Vector3d::Zero();
    rotVel = Eigen::Vector3d::Zero();
    rotAcc = Eigen::Vector3d::Zero();
    dt     = 0;
    this->type = 0;
    this->flag = false;
}
BodyPointState::BodyPointState(unsigned int type, unsigned int flag)
{
    *this = BodyPointState();
    this->type = type;
    this->flag = flag;

}

BodyPointState::~BodyPointState()
{

}


