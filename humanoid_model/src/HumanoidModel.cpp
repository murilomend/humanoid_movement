#include "humanoid_model/HumanoidModel.h"

HumanoidModel::HumanoidModel()
{
    initVars();

}

HumanoidModel::~HumanoidModel()
{

}



void HumanoidModel::initVars()
{
    legParams    = Eigen::VectorXd(PARAM_LEG_SIZE);
    armParams    = Eigen::VectorXd(PARAM_ARM_SIZE);
    footParams   = Eigen::VectorXd(PARAM_FOOT_SIZE);
    poseParams   = Eigen::VectorXd(PARAM_POSE_SIZE);


    endEffectors             = std::vector<BodyPointState>(BODY_TYPES_SIZE);
    endEffectors[LLEG]       = BodyPointState(LLEG,BP_FREE);
    endEffectors[RLEG]       = BodyPointState(RLEG,BP_FREE);
    endEffectors[LARM]       = BodyPointState(LARM,BP_FREE);
    endEffectors[RARM]       = BodyPointState(RARM,BP_FREE);
    endEffectors[HEAD]       = BodyPointState(HEAD,BP_FREE);
    endEffectors[TRUNK]      = BodyPointState(TRUNK,BP_FREE);
    endEffectors[COM]        = BodyPointState(COM,BP_FREE);
    endEffectors[WORLD_BODY] = BodyPointState(WORLD_BODY,BP_FREE);
}
void HumanoidModel::setComState(const BodyPointState &comState,bool update)
{



}
void HumanoidModel::setGlobalRef(const BodyPointState &globalRef,bool update)
{
    BodyPointState globalRefOld = this->globalRef;
    this->globalRef = globalRef;
    //UPDATE
    if(update)
    {
        //update ikRef
        ikRef.pos = (globalRef.pos - globalRefOld.pos)+ikRef.pos;
        //update qState
        //--
        //update floor
        //--
        //update endEff
        for(int i = 0; i < int(endEffectors.size());i++)
        {
            endEffectors[i].pos = (globalRef.pos - globalRefOld.pos)+endEffectors[i].pos;
        }
    }
}
void HumanoidModel::setIKRef(const BodyPointState &ikRef,bool update)
{
    //BodyPointState ikRefOld = this->ikRef;
    this->ikRef  = ikRef;
    //UPDATE
    if(update)
    {
        //update ikRef
        //--
        //update qState
        //--
        //update floor
        //--
        //update endEff
        /*for(int i = 0; i < int(endEffectors.size());i++)
        {
            //endEffectors[i].pos = (globalRef.pos - globalRefOld.pos)+endEffectors[i].pos;
        }*/
    }
    //update qState
    //update floor
    //update endeff
}

void HumanoidModel::setIKRef(const Eigen::Vector3d &ikRef,bool update)
{
    //endEffectors[LLEG].pos = endEffectors[LLEG].pos - this->ikRef.pos + ikRef;
    //endEffectors[RLEG].pos = endEffectors[RLEG].pos - this->ikRef.pos + ikRef;

    this->ikRef.pos  = ikRef;
    this->ikRef.rot  = Eigen::Vector3d(0,0,0);
}//Quando e setado ele nao muda as variaveis internas

void HumanoidModel::setEndEffState(const std::vector<BodyPointState> &endEffectors, bool update)
{
    //Calculate the inverse kinematics
    int nBodies = endEffectors.size();
    Eigen::VectorXd ikVec;
    JointState retState = JointState(this->robotDOF,ROBOT_IDS);
    JointState ikState = JointState(this->ikDOF,IK_IDS);
    Eigen::VectorXd legParams;
    for(int i = 0; i < nBodies;i++)
    {
        switch (endEffectors[i].type)
        {
        case LLEG:
            legParams = this->legParams;//For the LEFT LEG it sould be the same
            ikVec = InvKin::sixDOF(this->endEffectors[TRUNK].pos,this->endEffectors[TRUNK].rot,endEffectors[i].pos + ikRef.pos,endEffectors[i].rot + ikRef.rot,legParams,map);///<------
            ikState.pos = ikVec;
            //std::cout << "ikVec: " << std::endl << ikVec << std::endl;
            Mapping::convert(map,ikState,retState,endEffectors[i].type);
            //std::cout << "retState: " << std::endl << retState.pos << std::endl;
            this->endEffectors[LLEG] = endEffectors[i];
            break;
        case RLEG:
            legParams = this->legParams;
            legParams[P_LEG_D0] = -legParams[P_LEG_D0];//--->For the RIGHT LEG it should be inverted
            ikVec = InvKin::sixDOF(this->endEffectors[TRUNK].pos,this->endEffectors[TRUNK].rot,endEffectors[i].pos + ikRef.pos,endEffectors[i].rot + ikRef.rot,legParams,map);///<------
            ikState.pos = ikVec;
            //std::cout << "ikVec: " << std::endl << ikVec << std::endl;
            Mapping::convert(map,ikState,retState,endEffectors[i].type);
            //std::cout << "retState: " << std::endl << retState.pos << std::endl;
            this->endEffectors[RLEG] = endEffectors[i];
            break;
        case LARM:

            break;
        case RARM:

            break;
        case TRUNK:
            this->endEffectors[TRUNK] = endEffectors[i];
            //std::cout << "ENTROU" << std::endl; ///===>> Necessario??
            break;
        default:
            break;
        }
    }

    //UPDATE
    if(update)
    {
        //this->endEffectors = endEffectors;
        //update the qState
        this->qState = retState;
        //update floor
    }
}
void HumanoidModel::setQState(const JointState &qState,bool update)
{
    RigidBodyDynamics::UpdateKinematics(urdfModel,qState.pos,qState.vel,qState.acc);///<-------------
    //UPDATE
    if(update)
    {
        this->qState = qState;
        //update endEffState
        //update floor
    }
}

int HumanoidModel::groundFoot()
{
    //It returns the id of the first found touching body on the floor
    // -1 if any
    int id = -1;
    if     (endEffectors[RLEG].flag == BP_TOUCHING)  id = urdfModel.GetBodyId("RFOOT");
    else if(endEffectors[LLEG].flag == BP_TOUCHING)  id = urdfModel.GetBodyId("LFOOT");
    else                                             id = -1;
    return id;
}


Eigen::Vector3d HumanoidModel::groundBodyPos()
{
    //It returns the id of the first found touching body on the floor
    // -1 if any
    Eigen::Vector3d pos = Eigen::Vector3d(0,0,0);
    if     (endEffectors[RLEG].flag == BP_TOUCHING)  pos = endEffectors[RLEG].pos + ikRef.pos;
    else if(endEffectors[LLEG].flag == BP_TOUCHING)  pos = endEffectors[LLEG].pos + ikRef.pos;
    return pos;
}

void HumanoidModel::getComVecs(Eigen::Vector3d &footCom, Eigen::Vector3d &refCom, double overrideCom)
{
    //It converts the actual qState for URDF vector
    JointState qStateRBDL = JointState(urdfDOF,URDF_IDS);
    Mapping::convert(map,qState,qStateRBDL);

    //The position and atutude of the Trunk is also took into account
    Eigen::Vector3d rpyTrunk = endEffectors[TRUNK].rot;
    Eigen::Vector3d xyzTrunk = endEffectors[TRUNK].pos;


    //It gets the position of the body closest to the ground
    Eigen::Vector3d groundPos = groundBodyPos();


    //It then sets the qStateRDBL with the proper rotation of the Trunk:
    Eigen::Matrix4d trunkT  = MathUtils::transfMat(rpyTrunk,xyzTrunk);
    Eigen::Matrix4d wbT     = MathUtils::transfMat(endEffectors[WORLD_BODY].rot,endEffectors[WORLD_BODY].pos);//<do robot para o mundo
    Eigen::Matrix4d finalT = wbT*trunkT*wbT.transpose();

    MathUtils::decomposeTMat(finalT,rpyTrunk,xyzTrunk);

    qStateRBDL.pos(map(TRUNK_ROLL,URDF_IDS))  = rpyTrunk(0);//-endEffectors[TRUNK].rot(1);
    qStateRBDL.pos(map(TRUNK_PITCH,URDF_IDS)) = rpyTrunk(1);
    qStateRBDL.pos(map(TRUNK_YAW,URDF_IDS))   = rpyTrunk(2);

    qStateRBDL.pos(map(TRUNK_X,URDF_IDS))     = 0;
    qStateRBDL.pos(map(TRUNK_Y,URDF_IDS))     = 0;
    qStateRBDL.pos(map(TRUNK_Z,URDF_IDS))     = 0;


    //The position of the body touching the floor is obtained wrt. base coordinates
    //std::cout << "GROUNDPOS: " << groundPos << std::endl;
    //std::cout << "LLEG_FLAG: " << endEffectors[LLEG].flag << std::endl;
    //std::cout << "RLEG_FLAG: " << endEffectors[RLEG].flag << std::endl;
    //std::cout << "qStateRBDL:" << std::endl << qStateRBDL.pos << std::endl;

    //Then the remaining distance for the foot in Z is set:
    //double zCorr = 0;

    //std::cout << "ENDLLEGZCORR: " << std::endl << groundPos << std::endl;
    //zCorr = groundPos(2);//Distance between joint foot and ground

    //std::cout << "zCorr : " << std::endl << zCorr << std::endl;
    //The center of mass is then calculated wrt. the base coords.
    RigidBodyDynamics::Math::Vector3d comPos;///<====
    RigidBodyDynamics::Utils::CalcCenterOfMass(urdfModel,qStateRBDL.pos,qStateRBDL.vel,mass,comPos);

    //Finally the vector between foot and COM is obtained:
    //Eigen::Vector3d footCom = comPos - groundFoot;
    refCom   = comPos;
    //floorCom = -Eigen::Vector3d(0,0,zCorr)+comPos;
    footCom = -groundPos+comPos;
    if(overrideCom)
    {
        footCom(0) = -comOverride.pos(0);
        footCom(1) = comOverride.pos(1) + poseParams[P_POSE_LEG_OPEN] + poseParams[P_POSE_LEG_OPEN_REF];
        footCom(2) = comOverride.pos(2) - poseParams[P_POSE_LEG_SQUAT];
        refCom(0)  = -comOverride.pos(0);
        refCom(1)  = comOverride.pos(1);
        refCom(2)  = -(legParams[P_LEG_D1] + legParams[P_LEG_D2] + legParams[P_LEG_D3]+legParams[P_LEG_D4] - comOverride.pos(2));
    }
    /*std::cout << "comOverrride : " << std::endl << comOverride.pos << std::endl;
    std::cout << "refCom : " << std::endl << refCom << std::endl;
    std::cout << "footCom : " << std::endl << footCom << std::endl;*/

}


void HumanoidModel::setPoseParams(double legSquat,double legOpen,double torsoInc,double sideIncl,double footIncl)
{

    /*std::cout <<"LEGSQUAT: " << legSquat << std::endl;
    std::cout <<"TORSOINC: " << torsoInc << std::endl;
    std::cout <<"SIDEINC: " << sideIncl << std::endl;
    std::cout <<"LEGOPEN: " << legOpen << std::endl;*/


    poseParams[P_POSE_LEG_SQUAT] = legSquat;
    poseParams[P_POSE_LEG_OPEN]  = legOpen;
    poseParams[P_POSE_TORSO_INC] = torsoInc;
    poseParams[P_POSE_SIDE_INC] = sideIncl;

    legSquat -= poseParams[P_POSE_LEG_SQUAT_REF];
    legOpen  += poseParams[P_POSE_LEG_OPEN_REF];
    torsoInc += poseParams[P_POSE_TORSO_INC_REF];
    sideIncl += poseParams[P_POSE_SIDE_INC_REF];

/*    std::cout <<"LEGSQUAT_POS: " << legSquat << std::endl;
    std::cout <<"TORSOINC_POS: " << torsoInc << std::endl;
    std::cout <<"SIDEINC_POS: " << sideIncl << std::endl;
    std::cout <<"LEGOPEN_POS: " << legOpen << std::endl;*/

    endEffectors[TRUNK].pos = Eigen::Vector3d(0,0,0);
    endEffectors[TRUNK].rot = Eigen::Vector3d(sideIncl,torsoInc,0);

    endEffectors[LLEG].flag = BP_TOUCHING;
    endEffectors[LLEG].pos  = Eigen::Vector3d(0,legOpen,legSquat) - ikRef.pos;
    endEffectors[LLEG].rot  = Eigen::Vector3d(0,footIncl,0) - ikRef.rot;

    endEffectors[RLEG].flag = BP_TOUCHING;
    endEffectors[RLEG].pos  = Eigen::Vector3d(0,-legOpen,legSquat) - ikRef.pos;
    endEffectors[RLEG].rot  = Eigen::Vector3d(0,footIncl,0) - ikRef.rot;


    //std::cout << "ENDLLEG: " << std::endl << endEffectors[LLEG].pos << std::endl;
    updateEndEffState();
}



void HumanoidModel::setArmParams(double arm0,double arm1,double arm2)
{

    qState.pos[map(LARM0,ROBOT_IDS)] = arm0;
    qState.pos[map(LARM1,ROBOT_IDS)] = arm1;
    qState.pos[map(LARM2,ROBOT_IDS)] = -arm2;

    qState.pos[map(RARM0,ROBOT_IDS)] = -arm0;
    qState.pos[map(RARM1,ROBOT_IDS)] = -arm1;
    qState.pos[map(RARM2,ROBOT_IDS)] = arm2;
}


void HumanoidModel::calcCOM()
{


}
void HumanoidModel::updateQState()
{


}
void HumanoidModel::updateEndEffState()
{
    setEndEffState(this->endEffectors);

}
void HumanoidModel::updateFloor(const Eigen::Vector3d &pos, const Eigen::Vector3d &rot)
{

}

void HumanoidModel::footCalc(const Eigen::Vector3d fPos,const Eigen::Vector3d fRot)
{


}
