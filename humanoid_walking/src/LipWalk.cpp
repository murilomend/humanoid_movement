#include "humanoid_walking/LipWalk.h"

LipWalk::LipWalk()
{
	this->init();
	
	
}

LipWalk::LipWalk(double tS,double tD,double zC,double legDist,double xOffset,double stepHeight,double zCCorr,double dt,int curve,int firstFootStep,int weightA,int weightB)
{
	*this            = LipWalk();
    this->initParams(tS,tD,zC,legDist,xOffset,stepHeight,zCCorr,dt,curve,firstFootStep,weightA,weightB);
}


void LipWalk::initParams(double tS,double tD,double zC,double legDist,double xOffset,double stepHeight,double zCCorr,double dt,int curve,int firstFootStep,int weightA,int weightB)
{
	if(dt            != -1) this->dt      = dt;
	if(weightA       != -1) this->weightA = weightA;
	if(weightB       != -1) this->weightB = weightB;
	if(firstFootStep != 0 )
	{
		/*if(tD == 0)
		{
			this->footGround = firstFootStep*(-1);
			this->footAir    = firstFootStep;
		}
		else
		{
			this->footGround = firstFootStep;
			this->footAir    = firstFootStep*(-1);
		}*/
        this->firstFootStep = firstFootStep;
        this->footGround    = firstFootStep*(-1);
        this->footAir       = firstFootStep;
	}
	if(curve       != -1) this->curve = curve;
	
	this->tS         = tS; 
	this->tD         = tD; 
    this->zC         = zC;
    this->zCCorr     = zCCorr;
	this->stepHeight = stepHeight;
	this->legDist    = legDist;
    this->xOffset    = xOffset;
    this->floorIncl  = 0;
    this->fPosGroundRef = 0;
    this->tC = sqrt((zC*zCCorr)/GRAVITY);
	this->C  = cosh((tD + tS)/tC);
	this->S  = sinh((tD + tS)/tC);	
	
	this->com        = BodyPoint();  
    this->com.pos    = Eigen::Vector3d(xOffset,0,zC);
	
    this->rFootTeta  = Eigen::Vector3d(0,0,0);
    this->lFootTeta  = Eigen::Vector3d(0,0,0);
	
		
    this->rFootPos   = Eigen::Vector3d(0,-legDist/2.,0);
	this->lFootPos   = Eigen::Vector3d(0, legDist/2.,0);
	
    //this->com.pos    = Eigen::Vector3d(0,-legDist/2.,zC);

	this->fIdealSSP    =  (footAir == RIGHT_FOOT)?(rFootPos):(lFootPos); 
	
    this->vX           = 0;
    this->vY           = 0;
    this->vTeta        = 0;
	this->updateS(STAND_STILL);
    this->tSInt        = floor(tS/dt);
    this->tDInt        = floor(tD/dt);
    this->tSIntBuff    = floor(tS/dt);
    this->tDIntBuff    = floor(tD/dt);

	
	
    this->tNorm   = 0;
    this->tNormAll= 0;
    this->_t      = -dt;
    this->tInt    = -1;
    this->timeFac = 1;


    this->update = false;
	
}



void LipWalk::reset()
{
    init();
    initParams(this->tS,this->tD,this->zC,this->legDist,this->xOffset,this->stepHeight,this->zCCorr,this->dt,this->curve,this->firstFootStep,this->weightA,this->weightB);

}

void LipWalk::updateParams(double tS,double tD,double zC,double legDist,double xOffset,double stepHeight,double zCCorr)
{
    /*this->tS         = tS;
    this->tD         = tD;
    this->zC         = zC;
    this->zCCorr     = zCCorr;
    this->stepHeight = stepHeight;
    this->legDist    = legDist;

    this->tC = sqrt((zC*zCCorr)/GRAVITY);
    this->C  = cosh((tD + tS)/tC);
    this->S  = sinh((tD + tS)/tC);

    this->tSInt    = floor(tS/dt);
    this->tDInt    = floor(tD/dt);/**/

    std::cout << "tS: " << tS << " tD: " << tD << " zC: " << zC << " zCCorr: " << zCCorr << " stepHeigth: " << stepHeight << " legDist: " << legDist << std::endl;

    this->tSBuff         = tS;
    this->tDBuff         = tD;
    this->zCBuff         = zC;
    this->zCCorrBuff     = zCCorr;
    this->stepHeightBuff = stepHeight;
    this->legDistBuff    = legDist;

    this->tCBuff         = sqrt((zC*zCCorr)/GRAVITY);
    this->CBuff          = cosh((tD + tS)/tC);
    this->SBuff          = sinh((tD + tS)/tC);

    this->tSIntBuff      = floor(tS/dt);
    this->tDIntBuff      = floor(tD/dt);

    this->update         = true;

}


void LipWalk::init()
{
	this->zmp        = Eigen::Vector2d(0,0);
	this->comIniDSP  = BodyPoint();
	this->comEndDSP  = BodyPoint();
	this->footPosDSP = Eigen::Vector3d(0,0,0);
    this->aDSP       = std::vector<Eigen::Vector3d>(3);

	this->cOrigin      = DEFAULT;
	this->sJoystick    = Eigen::Vector3d(0,0,0);
	this->sStab        = Eigen::Vector3d(0,0,0);
	//  N_0  |  N_1  |  N_2 
    this->s              = std::vector<Eigen::Vector3d>(3);
	
	this->fIdealSSP      = Eigen::Vector3d(0,0,0);
	
	this->fAirIniSSP     = Eigen::Vector3d(0,0,0);
    this->fAirIniTetaSSP = Eigen::Vector3d(0,0,0);
	
	this->fGroundSSP     = Eigen::Vector3d(0,0,0);	
	
	this->fAirEndSSP     = Eigen::Vector3d(0,0,0);	
    this->fAirEndTetaSSP = Eigen::Vector3d(0,0,0);
	
	this->wState = DOUBLE_SUPPORT;
}


//For one Tsup period
BodyPoint LipWalk::integralLIP(const BodyPoint &comIni,const Eigen::Vector3d &_footPos,double t,double tC)
{
	if(tC == -1) tC = this->tC;
	BodyPoint com = BodyPoint();
	
	//Integrating the LIP equation from Tprev to Tprev+t
	com.pos = (comIni.pos - _footPos)*cosh(t/tC) + tC*comIni.vel*sinh(t/tC) + _footPos;
	com.vel = ((comIni.pos - _footPos)/(tC))*sinh(t/tC) + comIni.vel*cosh(t/tC);
    com.acc = (GRAVITY/(zC*zCCorr))*(com.pos - _footPos);
	//com.acc = (com.vel - comVelBuff)/dt;
	
	//Fixing Z
    com.pos[Z] = zC;
	com.vel[Z] = 0;
	com.acc[Z] = 0;
	return com;
}


std::vector<Eigen::Vector3d> LipWalk::getDblPolynom(const BodyPoint &com0,const BodyPoint &com1,double tD)
{
	if(tD == -1)  tD = this->tD;	
	Eigen::MatrixXd A(3,3);
	
	/*A <<    1  ,  tD  ,    pow(tD,2)/2,
			0  ,  1   ,     tD        ,
			0  ,  0   ,     1         ;
			
	Eigen::VectorXd bX(3),bY(3);
	
	bX << com1.pos[X] , com1.vel[X] , com1.acc[X] ;
	bY << com1.pos[Y] , com1.vel[Y] , com1.acc[Y] ;*/
	
	
	
	A <<     1  ,  0   ,      0        ,  
			 1  ,  tD  ,    pow(tD,2)/2,
			 0  ,  1   ,     tD        ;
			
			
	Eigen::VectorXd bX(3),bY(3);
	
	bX << com0.pos[X] , com1.pos[X] , com1.vel[X] ;
	bY << com0.pos[Y] , com1.pos[Y] , com1.vel[Y] ;
	
	
	// Solving Ax = b by the ColPivHouseholderQR Method	: speed: + 	Accuracy: ++ 
	Eigen::VectorXd x(3),y(3);
	x = A.fullPivHouseholderQr().solve(bX);
	y = A.fullPivHouseholderQr() .solve(bY);
	//z = A.colPivHouseholderQr().solve(bZ);
	
    std::vector<Eigen::Vector3d>  a = std::vector<Eigen::Vector3d>(3);
	a.at(0) << x[0] , y[0] , 0;
	a.at(1) << x[1] , y[1] , 0;
	a.at(2) << x[2] , y[2] , 0;

	return a;
	
}



BodyPoint LipWalk::calcDblSuport(const std::vector<Eigen::Vector3d> &a,double t)
{
	BodyPoint com = BodyPoint();
	//Com Position Using Polynom
	com.pos = a.at(0) + a.at(1)*t + a.at(2)*pow(t,2)*0.5;
	
	//Com Velocity Using PolynomDot
	com.vel = a.at(1) + a.at(2)*t;
	
	//Com Acceleration Using PolygnomDotDot
	com.acc = a.at(2);
	
	//Fixing Z
    com.pos[Z] = zC;
	com.vel[Z] = 0;
	com.acc[Z] = 0;
	
	return com;
}


Eigen::Vector4d LipWalk::calcNextFootPos(Eigen::Vector3d &footPosPrior,const Eigen::Vector3d &sCurrent,const Eigen::Vector3d &sNext,const BodyPoint &comIni, int footGround)
{
	//Calcutating Next foot Place
    footPosPrior = footPosPrior + calcRotTeta(sCurrent[TETA])*Eigen::Vector3d(sCurrent[X],-footGround*sCurrent[Y] , 0);
	
	//Calculating the Walking primtives
	//Position Primitive
	Eigen::Vector3d primPos;
    primPos = calcRotTeta(sNext[TETA])*Eigen::Vector3d(sNext[X]/2.,footGround*sNext[Y]/2.,0);
	//Speed Primitive
	Eigen::Vector3d primVel;
	primVel = calcRotTeta(sNext[TETA])*Eigen::Vector3d( ( (1+C)/(tC*S) )*primPos[X] , ( (C-1)/(tC*S) )*primPos[Y],primPos[Z] );
	
	//Calculating the Target State
	Eigen::Vector3d _footPos = Eigen::Vector3d(0,0,0);
	double D = weightA*(C-1)*(C-1) + weightB*(S/tC)*(S/tC);
	
	Eigen::Vector3d posD = footPosPrior + primPos;
	Eigen::Vector3d velD = primVel;

		
	Eigen::Vector3d comIniPos = Eigen::Vector3d(comIni.pos[X],comIni.pos[Y],0);
	Eigen::Vector3d comIniVel = Eigen::Vector3d(comIni.vel[X],comIni.vel[Y],0);
	
	_footPos = ((-weightA*(C-1))/D)*(posD - C*comIniPos - tC*S*comIniVel)- ((weightB*S)*(velD -(S/tC)*comIniPos - C*comIniVel))/(tC*D);

    //std::cout << "FOOTPOSPRIOR: " << std::endl << footPosPrior << std::endl;
    //std::cout << "_FOOTPOS: " << std::endl << _footPos << std::endl << std::endl;
    return Eigen::Vector4d(_footPos[X],_footPos[Y],_footPos[Z],sCurrent[TETA]);
    //return Eigen::Vector4d(-footPosPrior[X],-footPosPrior[Y],-footPosPrior[Z],sCurrent[TETA]);


						       
}





Eigen::VectorXd LipWalk::calcAirFootPos(const Eigen::Vector3d &footIni,const Eigen::Vector3d &footIniTeta,const Eigen::Vector3d &footEnd,const Eigen::Vector3d &footEndTeta,double stepHeight, int curve,double tNorm)
{
	//Setting initial values for normT, stepHeight, curve
	if(tNorm      == -1) tNorm      = this->tNorm;
	if(stepHeight == -1) stepHeight = this->stepHeight;
	if(curve      == -1) curve      = this->curve;
	
	double x = (footEnd[X] - footIni[X])*tNorm + footIni[X];
	double y = (footEnd[Y] - footIni[Y])*tNorm + footIni[Y];
	double z = 0;
    double tetaX = 0;
    double tetaY = 0;
    double tetaZ = (footEndTeta[Z] - footIniTeta[Z])*tNorm + footIniTeta[Z];

	//Which curve it is gonna be the air foot trajectory in the Z axis
	if     (curve == SINE     )   z = (sin(tNorm*PI))*stepHeight;
	else if(curve == COSINE   )   z = (cos(tNorm*PI))*stepHeight;
    else if(curve == SINE_2 )     z = (sin(tNorm*PI)*sin(tNorm*PI))*stepHeight;
	else if(curve == COSINE_2)    z = (cos(tNorm*PI)*cos(tNorm*PI))*stepHeight;
    else if(curve == CUSTOM)
    {
        z     = (sin(tNorm*PI)*sin(tNorm*PI))*stepHeight + (tNorm*3)/100.;
        tetaX = 0;
        tetaY = sin(tNorm*PI)*(10./180.)*PI;
    }

    Eigen::VectorXd ret = Eigen::VectorXd(6);
    ret(X) = x;ret(Y) = y;ret(Z) = z;
    ret(X+3) = tetaX;ret(Y+3) = tetaY;ret(Z+3) = tetaZ;
    return ret;
}


Eigen::Vector3d LipWalk::v2s(double vX,double vY,double vTeta)
{
	//Converting Velocity into `s` params
	double sX    = vX*(tS + tD);
	double sY    = vY*(tS + tD) + legDist;
	double sTeta = (vTeta/180)*PI*(tS + tD);
	
	return Eigen::Vector3d(sX,sY,sTeta);
}

void LipWalk::updateS(int cOrigin,double sX,double sY, double sTeta)
{
	if(cOrigin == JOYSTICK)
	{
		s.at(N_0)   = s.at(N_1);
        s.at(N_1)   = s.at(N_2);
        s.at(N_2)   = sJoystick;
	}
	else if(cOrigin == DEFAULT)
	{
		s.at(N_0) = s.at(N_1);
		s.at(N_1) = s.at(N_2);
		s.at(N_2) = s.at(N_2);
	}
	else if(cOrigin == STABILIZER)
	{
		s.at(N_0) = s.at(N_1);
		s.at(N_1) = s.at(N_2);
		s.at(N_2) = sStab;
	}
	else if(cOrigin == STAND_STILL)
	{
		s.at(N_0) = s.at(N_1) = s.at(N_2) = v2s(0,0,0);
	}
	
}

//Velocity : m/s  m/s  degrees/s
void LipWalk::joystick(double vX,double vY,double vTeta)
{
    this->vX     = vX;
    this->vY     = MathUtils::absf((vY*(1.+MathUtils::sign(vY)*footGround)*0.5));
    this->vTeta  = vTeta;//MathUtils::absf((vTeta*(1.+MathUtils::sign(vTeta)*footAir)*0.5));
    //if(tNorm == 0) this->vTeta += vTeta;

    sJoystick = v2s(this->vX,this->vY,this->vTeta);

    //std::cout << "vX : " << this->vX << " vY: " << this->vY << " vZ: " << this->vTeta << " sX: " << sJoystick[X] << " sY: " << sJoystick[Y] << " sZ: " << sJoystick[Z] << " footGround: " << footGround << std::endl;
	this->cOrigin = JOYSTICK;
}
void LipWalk::calcLIP(double timeFac)
{
    //Updating the state variables for the Walking and the 'time'
    double time = updateStateNTime(timeFac);
    if(tNorm == 0)initState(wState);
    if(tNorm == 0 && wState == SINGLE_SUPPORT)
    {
        updateS(this->cOrigin);
        this->cOrigin = DEFAULT;

        if(this->update)
        {
            ///Update Walking Parameters
            this->tS         = tSBuff;
            this->tD         = tDBuff;
            this->zC         = zCBuff;
            this->zCCorr     = zCCorrBuff;
            this->stepHeight = stepHeightBuff;
            this->legDist    = legDistBuff;

            this->tC = tCBuff;
            this->C  = CBuff;
            this->S  = SBuff;

            this->tSInt    = tSIntBuff;
            this->tDInt    = tDIntBuff;/**/
            this->update   = false;
            joystick(vX,vY,vTeta);

            stabilize(tNorm,wState,footGround);
        }

    }
    //Walking State Machine

    //state = DOUBLE SUPPORT
    if(this->wState == DOUBLE_SUPPORT)
    {
        //Calculating COM through tD
        com      = calcDblSuport(aDSP,time);
        /*cout << "DSP:" <<endl;
        cout << "COM(" << time << "): " << endl;
        cout << com.pos << endl << endl;*/
        //footPos doesnt change (footAir and footGround keep the same position)
        rFootPos = rFootPos;
        lFootPos = lFootPos;
    }
    //state = SINGLE SUPPORT
    else if(this->wState == SINGLE_SUPPORT)
    {
        com = integralLIP(comIniSSP,fGroundSSP,time);
        /*cout << "SSP:" <<endl;

        cout << "fGroundSSP == : " << endl;
        cout << fGroundSSP << endl << endl;

        cout << "COM(" << time << "): " << endl;
        cout << com.pos << endl << endl ;
        */

        //Update xFootPos and xFootTeta
        Eigen::Vector3d  fPosAir;
        Eigen::Vector3d  fTetaAir;


        Eigen::VectorXd fPosNTeta = calcAirFootPos(fAirIniSSP,fAirIniTetaSSP,fAirEndSSP,fAirEndTetaSSP);
        //Update xFootPos and xFootTeta
        fPosAir  = Eigen::Vector3d(fPosNTeta[0],fPosNTeta[1],fPosNTeta[2]);
        fTetaAir = Eigen::Vector3d(fPosNTeta[3],fPosNTeta[4],fPosNTeta[5]);

        Eigen::Vector3d  fPosGround  = (footGround == RIGHT_FOOT)?(rFootPos):(lFootPos);
        Eigen::Vector3d  fTetaGround = (footGround == RIGHT_FOOT)?(rFootTeta):(lFootTeta);



        //Calculating floor inclination


        fPosAir[Z]    += time*tan(floorIncl);
        fTetaAir[Y]    = -floorIncl;

        fPosGround[Z]  = fPosGroundRef - time*tan(floorIncl);
        fTetaGround[Y] = -floorIncl;

        if(tNorm == 1)  fPosGroundRef = fPosAir[Z];

        com.rot    += Eigen::Vector3d(0,0,dt*(this->vTeta/180.)*PI);

        rFootPos    = (footAir == RIGHT_FOOT)?(fPosAir):(fPosGround);
        lFootPos    = (footAir ==  LEFT_FOOT)?(fPosAir):(fPosGround);
        rFootTeta   = (footAir == RIGHT_FOOT)?(fTetaAir):(fTetaGround);
        lFootTeta   = (footAir ==  LEFT_FOOT)?(fTetaAir):(fTetaGround);


    }
    //this->_t += dt;
}


void LipWalk::initState(int wState)
{
    if(wState == -1) wState = this->wState;

    //state = DOUBLE SUPPORT
    if(this->wState == DOUBLE_SUPPORT)
    {
        //Finish SSP

        //Initialize DSP
        comIniDSP  = com;
        footPosDSP = (footAir == RIGHT_FOOT)?(rFootPos):(lFootPos);//<=--------------
        comEndDSP  = integralLIP(comIniDSP,footPosDSP,tD);
        aDSP       = getDblPolynom(comIniDSP,comEndDSP);

        /*cout << "INIT____DSP:" <<endl;
        cout << "COMINI == : " << endl;
        cout << com.pos << endl << endl;

        cout << "footPosDSP == : " << endl;
        cout << footPosDSP << endl << endl;


        cout << "COMEND == : " << endl;
        cout << comEndDSP.pos << endl << endl;*/
    }
    //state = SINGLE SUPPORT
    else if(this->wState == SINGLE_SUPPORT)
    {
        //Update footGround and footAir
        footGround = (-1)*footGround; // It changes the footGround
        footAir    = (-1)*footAir;    // It changes the footAir

        //Update footPosAir and fGroundSSP
        fAirIniSSP     = (footAir == RIGHT_FOOT)?(rFootPos):(lFootPos);
        fAirIniTetaSSP = (footAir == RIGHT_FOOT)?(rFootTeta):(lFootTeta);

        fGroundSSP     = (footGround == RIGHT_FOOT)?(rFootPos):(lFootPos);

        //Calculate next footStep
        comIniSSP  = (tD == 0)?(com):(comIniDSP);
        comEndSSP  = integralLIP(comIniSSP,fGroundSSP,tS+tD);
        Eigen::Vector4d  fPosNTeta = calcNextFootPos(fIdealSSP,s.at(N_0),s.at(N_1),comEndSSP,footGround);

        fAirEndSSP        = Eigen::Vector3d(fPosNTeta[0],fPosNTeta[1],fPosNTeta[2]);
        fAirEndTetaSSP[2] = fPosNTeta[3];
    }
}

double LipWalk::updateStateNTime(double timeFac)
{
    int tDIntFac = ceil(tDInt*timeFac);
    int tSIntFac = ceil(tSInt*timeFac);
    //std::cout << "______________________DEBUG______________________" << std::endl;
    if(wState == SINGLE_SUPPORT && tNorm >= 0.5)
    {
        if(timeFac != this->timeFac )
        {
            tInt = tNormAll*((double)(tDIntFac + tSIntFac));
            this->timeFac = timeFac;
            std::cout  << " timeFac: " << timeFac << std::endl;
        }
        ///^^^^^^^^^^^^^^^^^^^^^^^^^^CONSERTAR!!!^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

    }


    tInt ++;
    _t += dt*timeFac;


    if(tInt > tDIntFac + tSIntFac)
    {
        tInt = 0;
        tNorm = 0;
    }
    //DOUBLE SUPPORT PHASE
    if(tInt <= tDIntFac && tDIntFac != 0)
    {
        tNorm  = ((double)tInt)/((double)tDIntFac);
        wState = DOUBLE_SUPPORT;
    }
    //SINGLE SUPPORT PHASE
    else if((tInt > tDIntFac && tInt <= (tDIntFac + tSIntFac)) || tDIntFac == 0)
    {
        if(tDIntFac != 0)    tNorm      = ((double)(tInt -1 - tDIntFac))/((double)(tSIntFac-1));
        else 			     tNorm      = ((double)(tInt - tDIntFac))/((double)(tSIntFac));
        wState     = SINGLE_SUPPORT;
    }

    tNormAll = (((double)tInt)/((double)(tDIntFac + tSIntFac)));
    double time = tNormAll*(tD + tS);

/*
    std::cout << "tInt: " << tInt << " tDIntFac: " << tDIntFac << " tSIntFac: " << tSIntFac << " tDInt: " << tDInt << " tSInt: " << tSInt << std::endl;
    std::cout << "tS: " << tS << " tD: " << tD << std::endl;
    std::cout << "tNormAll: " << tNormAll << " tNorm: " << tNorm << " time: " << time << " timeFac: " << timeFac << std::endl;
    if(tNormAll > 1 || tNorm > 1)  std::cout << "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA_" << std::endl;
*/
    return time;
}

void LipWalk::stabilize(double tNorm,int wState,int footGround)
{
    double tAux = _t;
    for(double t = 0; t <= 60; t+= dt)
    {
        calcLIP();
        if(t >= 40 && this->tNorm == tNorm && this->wState == wState  && this->footGround == footGround )  break;
    }
    _t = tAux;
}

bool LipWalk::stabilized()
{
   if(this->tNorm == 0 && this->wState == DOUBLE_SUPPORT && MathUtils::absf(this->rFoot[X]) <= 0.01 && MathUtils::absf(this->lFoot[X]) <= 0.01)  return true;
   return false;;
}

Eigen::Matrix3d LipWalk::calcRotTeta(double teta)
{
	Eigen::Matrix3d m;
	m << cos(teta),-sin(teta), 0,
	     sin(teta), cos(teta), 0,
	     0        , 0        , 1;
	return m;
}



void LipWalk::updateComFoot()
{
	rFoot = (rFootPos - com.pos);
	lFoot = (lFootPos - com.pos);
    rFootTeta = -(rFootTeta - com.rot);
    lFootTeta = -(lFootTeta - com.rot);
}



void LipWalk::calcZMP()
{
    this->zmp[X] = com.pos[X] - ((com.pos[Z] - 0)*com.acc[X])/(com.acc[Z] + GRAVITY);
    this->zmp[Y] = com.pos[Y] - ((com.pos[Z] - 0)*com.acc[Y])/(com.acc[Z] + GRAVITY);
	
}


LipWalk::~LipWalk()
{

}
