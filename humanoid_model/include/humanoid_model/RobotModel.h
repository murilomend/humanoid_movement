#ifndef ROBOTMODEL_H
#define ROBOTMODEL_H

#include <eigen3/Eigen/Dense>
#include "humanoid_general/State.h"
#include "humanoid_general/Enums.h"
#include "humanoid_loadmap/Mapping.h"
#include <rbdl/rbdl.h>
#ifndef RBDL_BUILD_ADDON_URDFREADER
    #error "Error: RBDL addon URDFReader not enabled."
#endif
#include <rbdl/addons/urdfreader/urdfreader.h>




class RobotModel
{
public:
    RigidBodyDynamics::Model   urdfModel;
    Eigen::MatrixXi            map;


    unsigned int       urdfDOF;
    unsigned int       robotDOF;
    unsigned int       ikDOF;
    double             mass;

    BodyPointState     com;
    BodyPointState     comOverride;

    JointState                  qState;
    std::vector<BodyPointState> endEffectors;

    BodyPointState     globalRef;
    BodyPointState     ikRef;
    Eigen::Vector4d    floor;


    //Constructors
    RobotModel();
    RobotModel(const char* filename,unsigned int robotDOF,unsigned int ikDOF);


    //Destructor
    ~RobotModel();


    //Methods
    void    loadURDF(const char* filename);
    void    initVars(unsigned int robotDOF, unsigned int ikDOF);
    void    calcDynInv(const JointState &qState,bool update = true);
    BodyPointState calcCom(JointState &qState,BodyPointState &ref,bool update = true);
    virtual void   setGlobalRef(const BodyPointState &globalRef,bool update = true){}
    virtual void   setIKRef(const BodyPointState &ikRef,bool update = true){}
    virtual void   setEndEffState(const BodyPointState &endEffectors,bool update = true){}
    virtual void   setQState(const JointState &qState,bool update = true){}
    //BodyPointState getEffState();
    //JointState     getQState();
};

#endif // ROBOTMODEL_H
