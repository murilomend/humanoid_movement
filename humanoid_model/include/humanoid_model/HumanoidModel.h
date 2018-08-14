#ifndef HUMANOIDMODEL_H
#define HUMANOIDMODEL_H

#include "humanoid_model/RobotModel.h"
#include "humanoid_model/InvKin.h"
#include <eigen3/Eigen/Dense>

class HumanoidModel : public RobotModel
{
public:
    //Members
    Eigen::VectorXd  footParams;
    Eigen::VectorXd  legParams;
    Eigen::VectorXd  armParams;
    Eigen::VectorXd  poseParams;




    //Constructors
    HumanoidModel();

    //Destructor
    ~HumanoidModel();

    //Methods
    void initVars();
    void footCalc(const Eigen::Vector3d fPos, const Eigen::Vector3d fRot);
    void setComState(const BodyPointState &comState,bool update = true);
    void setGlobalRef(const BodyPointState &globalRef,bool update = true);
    void setIKRef(const BodyPointState &ikRef,bool update = true);
    void setIKRef(const Eigen::Vector3d &ikRef,bool update = true);
    void setEndEffState(const std::vector<BodyPointState> &endEffectors,bool update = true);
    void setQState(const JointState &qState,bool update = true);
    void updateQState();
    void updateEndEffState();
    void updateFloor(const Eigen::Vector3d &pos, const Eigen::Vector3d &rot);
    void getComVecs(Eigen::Vector3d &footCom, Eigen::Vector3d &refCom,double overrideCom = false);
    int  groundFoot();
    Eigen::Vector3d groundBodyPos();
    void setPoseParams(double legSquat, double legOpen, double torsoInc, double sideIncl, double footIncl);
    void setArmParams(double arm0,double arm1,double arm2);

    void calcCOM();

};

#endif // HUMANOIDMODEL_H
