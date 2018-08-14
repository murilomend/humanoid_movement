#ifndef STATE_H
#define STATE_H

#include <eigen3/Eigen/Dense>
#include "Enums.h"



class JointState
{
public:
    //Members
    unsigned int    nJoints;
    Eigen::VectorXd pos;
    Eigen::VectorXd vel;
    Eigen::VectorXd acc;
    Eigen::VectorXd torq;
    double          dt;
    unsigned int    type;
    unsigned int    source;

    //Constructor
    JointState();
    JointState(int nJoints,int type = ROBOT_IDS,int source = NONE);///<-------------


    //Destructor
    ~JointState();

    //Methods
};


class BodyPointState
{
public:
    //Members
    Eigen::Vector3d pos;
    Eigen::Vector3d rot;
    Eigen::Vector3d vel;
    Eigen::Vector3d acc;
    Eigen::Vector3d rotVel;
    Eigen::Vector3d rotAcc;
    double          dt;
    unsigned int    type;
    unsigned int    flag;

    //Construtors
    BodyPointState();
    BodyPointState(unsigned int type,unsigned int flag = BP_FREE);

    //Destructors
    ~BodyPointState();

    //Methods


};


#endif // STATE_H
