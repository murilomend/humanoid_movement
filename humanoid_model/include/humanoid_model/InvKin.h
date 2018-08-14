#ifndef INVKIN_H
#define INVKIN_H

#include <eigen3/Eigen/Dense>
#include "humanoid_general/Enums.h"
#include "humanoid_general/MathUtils.h"

#define PI 3.14159 ///<<=============
class InvKin
{
public:
    //Members

    //Contructors
    InvKin();

    //Destructors
    ~InvKin();


    //Methods
    static Eigen::VectorXd sixDOF(const Eigen::Vector3d &refPosV, const Eigen::Vector3d &refRotV, const Eigen::Vector3d &footPosV, const Eigen::Vector3d &footRotV, const Eigen::VectorXd &legParams, const Eigen::MatrixXi &map);
    static Eigen::VectorXd threeDOF();
};

#endif // INVKIN_H
