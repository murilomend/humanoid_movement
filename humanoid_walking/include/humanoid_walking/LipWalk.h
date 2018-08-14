#ifndef LIPWALK_H
#define LIPWALK_H

#include <eigen3/Eigen/Dense>
#include "humanoid_general/MathUtils.h"
#include <stdio.h>
#include <vector>
#include <math.h>
#include <iostream>

#define GRAVITY  9.78588 //[m/s^2]
#define PI 3.14159


enum FootStep      {RIGHT_FOOT = -1,LEFT_FOOT = 1};
enum CoordIndex    {X = 0,Y = 1,Z = 2,TETA = 2};
enum TimeIndex     {INITIAL = 0,PRIOR = 0,CURRENT = 1,AFTER = 2,TARGET = 2,N_0 = 0,N_1 = 1, N_2 = 2};
enum WalkCurves    {SINE,SINE_2,CUSTOM, COSINE,COSINE_2};
enum WalkStates    {STOPPED,SINGLE_SUPPORT,DOUBLE_SUPPORT};
enum CommandOrigin {STAND_STILL,JOYSTICK,DEFAULT,STABILIZER};

//enum CoordIndex2   {};


struct BodyPoint
{
	Eigen::Vector3d pos;
    Eigen::Vector3d rot;
	Eigen::Vector3d vel;
	Eigen::Vector3d acc;	
	BodyPoint()
	{
		pos = Eigen::Vector3d(0,0,0);
        rot = Eigen::Vector3d(0,0,0);
		vel = Eigen::Vector3d(0,0,0);
		acc = Eigen::Vector3d(0,0,0);
	}
};


class LipWalk
{
public:
	//Walking constant parameters
	double tS; //Time to Single Support
	double tD; //Time to Double Support
	double weightA;
	double weightB;	
	
	//Walk variables
	int       footGround;
	int       footAir;
    int       firstFootStep;
    BodyPoint com; //Current COM
	Eigen::Vector2d  zmp;
	
    Eigen::Vector3d  rFootPos;
    Eigen::Vector3d  lFootPos;
    Eigen::Vector3d  rFootTeta;
    Eigen::Vector3d  lFootTeta;

    Eigen::Vector3d  rFoot;
    Eigen::Vector3d  lFoot;
	
	int    curve;
	double stepHeight;
	

	//Walk Auxiliar Variables
	//DSP
    BodyPoint comIniDSP; //Initial COM DSP
    BodyPoint comEndDSP; //Final COM DSP
	Eigen::Vector3d  footPosDSP;//Last footPos (next foot SSP)
    std::vector<Eigen::Vector3d> aDSP;//Polynomial coeficients
	
	//SSP
    BodyPoint comIniSSP; //Initial COM
    BodyPoint comEndSSP; //Final COM
	Eigen::Vector3d  footPosSSP;//Current SSP foot Position
	Eigen::Vector3d  footPriorSSP;//Calculated with no correction
	
	//Foot Air (Initial and End Conditions in SSP)
	Eigen::Vector3d  fIdealSSP;
	
    Eigen::Vector3d  fAirIniSSP;
    Eigen::Vector3d  fAirIniTetaSSP;
	
	Eigen::Vector3d  fGroundSSP;	
	
    Eigen::Vector3d  fAirEndSSP;
    Eigen::Vector3d  fAirEndTetaSSP;

    double           fPosGroundRef;
	
	
	
	//InputParameters
	int cOrigin;
	Eigen::Vector3d  sJoystick;
	Eigen::Vector3d  sStab;
	//  N   |  N_1  |  N_2 
    std::vector<Eigen::Vector3d>  s;
    double vX;
    double vY;
    double vTeta;


	//Robot Geometric Parameters
	double zC;      //COM height
    double zCCorr; //Correction for the zC length
	double legDist; //Distance between Legs
    double floorIncl;
    double xOffset;
	
	
	
	//Walking Derived parameters
	double tC; // Time constant
	double C;
	double S;
	
	//State Variables
    bool   update;
	int    wState;
	double tNorm;
    double tNormAll;
	double dt;
	double _t;
    double timeFac;
	int    tSInt;
	int    tDInt;
	int    tInt;


    //Buffer Variables
    double tSBuff;
    double tDBuff;
    double zCBuff;
    double zCCorrBuff;
    double stepHeightBuff;
    double legDistBuff;

    double tCBuff;
    double CBuff;
    double SBuff;

    int    tSIntBuff;
    int    tDIntBuff;

	
	
	//Walking Dynamic parameters
	
	
	LipWalk();
    LipWalk(double tS,double tD,double zC,double legDist,double xOffset,double stepHeight,double zCCorr = 1,double dt = 0.01,int curve = SINE,int firstFootStep = RIGHT_FOOT,int weightA = 10,int weightB = 1);
	~LipWalk();
	
	
	//Methods
	void init();
    void initParams(double tS,double tD,double zC,double legDist,double xOffset,double stepHeight,double zCCorr = 1,double dt = -1,int curve = -1,int firstFootStep = 0,int weightA = -1,int weightB = -1);
    void updateParams(double tS,double tD,double zC,double legDist,double xOffset,double stepHeight,double zCCorr);
	BodyPoint integralLIP(const BodyPoint &comIni,const Eigen::Vector3d &_footPos,double t = 0,double tC = -1);
    std::vector<Eigen::Vector3d> getDblPolynom(const BodyPoint &com0,const BodyPoint &com1,double tD = -1);
    BodyPoint calcDblSuport(const std::vector<Eigen::Vector3d> &a,double t);
	Eigen::Vector4d calcNextFootPos(Eigen::Vector3d &footPosPrior,const Eigen::Vector3d &sCurrent,const Eigen::Vector3d &sNext,const BodyPoint &comIni, int footGround);
	void setCOMFeetParams(double stepHeight,int curve);
    Eigen::VectorXd calcAirFootPos(const Eigen::Vector3d &footIni, const Eigen::Vector3d &footIniTeta, const Eigen::Vector3d &footEnd, const Eigen::Vector3d &footEndTeta, double stepHeight = -1, int curve = -1, double tNorm = -1);
	Eigen::Vector3d v2s(double vX,double vY,double vTeta);
	void updateS(int cOrigin = DEFAULT,double sX = 0,double sY = 0, double sTeta = 0);
	void joystick(double vX,double vY,double vTeta);
    void calcLIP(double timeFac = 1);
	void initState(int wState = -1);
    double updateStateNTime(double timeFac = 1);
	Eigen::Matrix3d calcRotTeta(double teta);
	void updateComFoot();
    void calcZMP();
    void stabilize(double tNorm = 0,int wState = DOUBLE_SUPPORT,int footGround = RIGHT_FOOT);
    bool stabilized();
    void reset();

	
};






#endif
