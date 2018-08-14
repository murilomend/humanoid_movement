#include "humanoid_general/MathUtils.h"

MathUtils::MathUtils()
{

}

Eigen::Matrix3d MathUtils::vectorHat(const Eigen::Vector3d &vector)
{
    Eigen::Matrix3d m;
    m << 0        , -vector(2) ,  vector(1),
         vector(2),       0    , -vector(0),
        -vector(1),  vector(0) ,      0    ;
    return m;
}

Eigen::Matrix3d MathUtils::rodriguesFormula(const Eigen::Matrix3d &initialM,const Eigen::Vector3d &jAxis,const double teta)
{
    Eigen::Matrix3d jAxisHat = vectorHat(jAxis);
    return (initialM + jAxisHat*sin(teta) + (jAxisHat*jAxisHat)*(1 - cos(teta)));

}

Eigen::Matrix3d MathUtils::rpy2Mat(const Eigen::Vector3d &rpyRot)
{
    Eigen::Matrix3d matX = rodriguesFormula(Eigen::Matrix3d::Identity(),Eigen::Vector3d(1,0,0),rpyRot(0));
    Eigen::Matrix3d matY = rodriguesFormula(Eigen::Matrix3d::Identity(),Eigen::Vector3d(0,1,0),rpyRot(1));
    Eigen::Matrix3d matZ = rodriguesFormula(Eigen::Matrix3d::Identity(),Eigen::Vector3d(0,0,1),rpyRot(2));

    return matZ*matY*matX;
}
Eigen::Matrix4d MathUtils::transfMat(const Eigen::Vector3d &rpy,const Eigen::Vector3d &xyz)
{
    Eigen::Matrix3d matRot = rpy2Mat(rpy);
    Eigen::Matrix4d T;
    T << matRot(0,0) , matRot(0,1) , matRot(0,2), xyz(0),
         matRot(1,0) , matRot(1,1) , matRot(1,2), xyz(1),
         matRot(2,0) , matRot(2,1) , matRot(2,2), xyz(2),
            0        ,      0      ,      0     ,   1   ;
    return  T;
}

void MathUtils::decomposeTMat(const Eigen::Matrix4d &tMat,Eigen::Vector3d &rpy,Eigen::Vector3d &xyz)
{
    Eigen::Matrix3d matRot;
    matRot(0,0) = tMat(0,0);
    matRot(1,0) = tMat(1,0);
    matRot(2,0) = tMat(2,0);
    matRot(0,1) = tMat(0,1);
    matRot(1,1) = tMat(1,1);
    matRot(2,1) = tMat(2,1);
    matRot(0,2) = tMat(0,2);
    matRot(1,2) = tMat(1,2);
    matRot(2,2) = tMat(2,2);
    xyz(0)      = tMat(0,3);
    xyz(1)      = tMat(1,3);
    xyz(2)      = tMat(2,3);

    rpy = mat2Rpy(matRot);

    //std::cout << "MATHUTIS:" << std::endl << "XYZ: " << std::endl << xyz << std::endl << "RPY: " << std::endl << rpy << std::endl;
}

Eigen::Vector3d MathUtils::mat2Rpy(const Eigen::Matrix3d &matRot)
{
    float sy = sqrt(matRot(0,0) * matRot(0,0) +  matRot(1,0) * matRot(1,0) );

    bool singular = sy < 1e-6; // If

    float x, y, z;
    if (!singular)
    {
        x = atan2(matRot(2,1) , matRot(2,2));
        y = atan2(-matRot(2,0), sy);
        z = atan2(matRot(1,0), matRot(0,0));
    }
    else
    {
        x = atan2(-matRot(1,2), matRot(1,1));
        y = atan2(-matRot(2,0), sy);
        z = 0;
    }
    return Eigen::Vector3d(x, y, z);
}


// Cooley-Tukey FFT (in-place, breadth-first, decimation-in-frequency)
// Better optimized but less intuitive
// !!! Warning : in some cases this code make result different from not optimased version above (need to fix bug)
// The bug is now fixed @2017/05/30
void MathUtils::fft(CArray &x)
{
    // DFT
    unsigned int N = x.size(), k = N, n;
    double thetaT = 3.14159265358979323846264338328L / N;
    Complex phiT = Complex(cos(thetaT), -sin(thetaT)), T;
    while (k > 1)
    {
        n = k;
        k >>= 1;
        phiT = phiT * phiT;
        T = 1.0L;
        for (unsigned int l = 0; l < k; l++)
        {
            for (unsigned int a = l; a < N; a += n)
            {
                unsigned int b = a + k;
                Complex t = x[a] - x[b];
                x[a] += x[b];
                x[b] = t * T;
            }
            T *= phiT;
        }
    }
    // Decimate
    unsigned int m = (unsigned int)log2(N);
    for (unsigned int a = 0; a < N; a++)
    {
        unsigned int b = a;
        // Reverse bits
        b = (((b & 0xaaaaaaaa) >> 1) | ((b & 0x55555555) << 1));
        b = (((b & 0xcccccccc) >> 2) | ((b & 0x33333333) << 2));
        b = (((b & 0xf0f0f0f0) >> 4) | ((b & 0x0f0f0f0f) << 4));
        b = (((b & 0xff00ff00) >> 8) | ((b & 0x00ff00ff) << 8));
        b = ((b >> 16) | (b << 16)) >> (32 - m);
        if (b > a)
        {
            Complex t = x[a];
            x[a] = x[b];
            x[b] = t;
        }
    }
    //// Normalize (This section make it not working correctly)
    //Complex f = 1.0 / sqrt(N);
    //for (unsigned int i = 0; i < N; i++)
    //	x[i] *= f;
}

double MathUtils::mean(const CArray &vec)
{
    double sum = 0;
    double value = 0;
    int    N = vec.size();
    for(int i = 0; i < N;i++)
    {
        value = vec[i].real();
        //vec[i] = Complex(value,0);
        sum += value;
    }
    return sum/((double)N);
}

double MathUtils::norm(const Complex &num)
{
    return  sqrt(num.real()*num.real() + num.imag()*num.imag());
}

void MathUtils::quatern2euler(double q0,double q1,double q2,double q3,double &phi,double &theta,double &psi)
{
    float R11 = 0,R21 = 0,R31 = 0,R32 = 0,R33 = 0;
    R11 = 2.*q0*q0-1+2.*q1*q1;
    R21 = 2.*(q1*q2-q0*q3);
    R31 = 2.*(q1*q3+q0*q2);
    R32 = 2.*(q2*q3-q0*q1);
    R33 = 2.*q0*q0-1+2.*q3*q3;

    phi = atan2(R32, R33 );
    theta = -atan(R31/sqrt(1-R31*R31) );
    //theta = -asin(R31/sqrt(1-R31*R31) );
    psi = atan2(R21, R11 );


    phi   *= (180./3.14159);
    theta *= (180./3.14159);
    psi   *= (180./3.14159);


}


double MathUtils::sign(double x)
{
    double ret = (x>=0)?(+1):(-1);
    return ret;
}

double MathUtils::absf(double x)
{
    double abs = (x>=0)?(x):(-x);
    return abs;
}
