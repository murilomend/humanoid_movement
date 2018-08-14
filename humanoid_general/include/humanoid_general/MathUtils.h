#ifndef MATHUTILS_H
#define MATHUTILS_H

#include <eigen3/Eigen/Dense>
#include <iostream>
#include <complex>
#include <valarray>

typedef std::complex<double> Complex;
typedef std::valarray<Complex> CArray;


//const double PI = 3.141592653589793238460;

class MathUtils
{
public:
    MathUtils();
    static Eigen::Matrix3d vectorHat(const Eigen::Vector3d &vector);
    static Eigen::Matrix3d rodriguesFormula(const Eigen::Matrix3d &initialM,const Eigen::Vector3d &jAxis,const double teta);
    static double sign(double x);
    static double absf(double x);
    static void   fft(CArray &x);
    static Eigen::Matrix3d rpy2Mat(const Eigen::Vector3d &rpyRot);
    static Eigen::Vector3d mat2Rpy(const Eigen::Matrix3d &matRot);
    static Eigen::Matrix4d transfMat(const Eigen::Vector3d &rpy,const Eigen::Vector3d &xyz);
    static void decomposeTMat(const Eigen::Matrix4d &tMat, Eigen::Vector3d &rpy, Eigen::Vector3d &xyz);
    static void quatern2euler(double q0,double q1,double q2,double q3,double &phi,double &theta,double &psi);
    static double mean(const CArray &vec);
    static double norm(const Complex &num);

};

#endif // MATUTILS_H
