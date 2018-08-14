#include "humanoid_model/InvKin.h"

InvKin::InvKin()
{

}



/*
void InvKin::setIK(const Eigen::Vector3d &rFoot, const Eigen::Vector3d &lFoot,const Matrix3d &rFootRot,const Matrix3d &lFootRot)
{
    setIK(rFoot,RIGHT_SIDE,rFootRot);
    setIK(lFoot,LEFT_SIDE,lFootRot);
}

void InvKin::setIK(const Vector3d &rFoot, const Vector3d &lFoot,const double &rRot,const double &lRot)
{
    setIK(rFoot,RIGHT_SIDE,rodriguesFormula(Matrix3d::Identity(),Vector3d(0,0,1),rRot));
    setIK(lFoot,LEFT_SIDE,rodriguesFormula(Matrix3d::Identity(),Vector3d(0,0,1),lRot));
}

void InvKin::setLegIKRef(const Vector3d &ref)
{
    legRef    = ref;
}/**/

Eigen::VectorXd InvKin::sixDOF(const Eigen::Vector3d &refPosV,const Eigen::Vector3d &refRotV,const Eigen::Vector3d &footPosV,const Eigen::Vector3d &footRotV,const Eigen::VectorXd &legParams,const Eigen::MatrixXi &map)
{
    double D = legParams(P_LEG_D0);
    double L = legParams(P_LEG_D1);
    double A = legParams(P_LEG_D2);
    double B = legParams(P_LEG_D3);///<================
    double F = legParams(P_LEG_D4);
    Eigen::Matrix3d refRot  = MathUtils::rpy2Mat(refRotV);
    Eigen::Matrix3d footRot = MathUtils::rpy2Mat(footRotV);
    Eigen::Vector3d refPos  = refPosV;
    Eigen::Vector3d footPos = footPosV + Eigen::Vector3d(0,0,F+L);

    /*std::cout << "A: " << A << " B: " << B << " D: " << D << std::endl;
    std::cout << "refPos: " << std::endl << refPos << std::endl;
    std::cout << "refRot: " << std::endl << refRot << std::endl;
    std::cout << "footPos: " << std::endl << footPos << std::endl;
    std::cout << "footRot: " << std::endl << footRot << std::endl;*/

    Eigen::Vector3d r = footRot.transpose()*(refPos + refRot*Eigen::Vector3d(0, D , 0) - footPos);
    double C = r.norm();
    double c5 = (C*C - A*A - B*B)/(2.*A*B);
    double q5 = 0;
    if     (c5 >= 1)	q5 = 0;
    else if(c5 <= -1)   q5 = PI;
    else                q5 = acos(c5);
    double q6a = asin((A/C)*sin(PI-q5));
    //cout << "r: " << r <<endl;
    double q7  = atan2(r(1),r(2));
    if     (q7 > PI/2.)  q7 = q7-PI;
    else if(q7 < -PI/2) q7 = q7+PI;
    double q6 = -atan2(r(0),MathUtils::sign(r(2))*sqrt(r(1)*r(1) + r(2)*r(2))) - q6a;

    Eigen::Matrix3d Rroll  = MathUtils::rodriguesFormula(Eigen::Matrix3d::Identity(),Eigen::Vector3d(1,0,0),-q7);
    Eigen::Matrix3d Rpitch = MathUtils::rodriguesFormula(Eigen::Matrix3d::Identity(),Eigen::Vector3d(0,1,0),-q5-q6);
    Eigen::Matrix3d R = refRot.transpose()*footRot*Rroll*Rpitch;
    double q2 = atan2(-R(0,1),R(1,1));
    double cz = cos(q2);
    double sz = sin(q2);
    double q3 = atan2(R(2,1),-R(0,1)*sz+R(1,1)*cz);
    double q4 = atan2(-R(2,0),R(2,2));

    Eigen::VectorXd q = Eigen::VectorXd::Zero(map.rows());
    //Eigen::VectorXd q = Eigen::VectorXd(21);
    q(map(LLEG0,IK_IDS))  = q2;
    q(map(LLEG1,IK_IDS))  = q3;
    q(map(LLEG2,IK_IDS))  = q4;
    q(map(LLEG3,IK_IDS))  = q5;
    q(map(LLEG4,IK_IDS))  = q6;
    q(map(LFOOT,IK_IDS))  = q7;

    q(map(RLEG0,IK_IDS))  = q2;
    q(map(RLEG1,IK_IDS))  = q3;
    q(map(RLEG2,IK_IDS))  = q4;
    q(map(RLEG3,IK_IDS))  = q5;
    q(map(RLEG4,IK_IDS))  = q6;
    q(map(RFOOT,IK_IDS))  = q7;

    return q;
}

Eigen::VectorXd InvKin::threeDOF()
{


}
