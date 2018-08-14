#ifndef MAPPING_H
#define MAPPING_H

#include <iostream>
#include <iomanip>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <map>
#include "humanoid_general/Enums.h"
#include "humanoid_general/State.h"



//Classe para converter de URDF para o Robo real
class Mapping
{
public:
    Eigen::MatrixXi                       map;
    std::map<std::string,unsigned int>    idRobotMap;
    std::map<std::string,unsigned int>    idUrdfMap;
    std::map<std::string,unsigned int>    idIKMap;
    std::map<std::string,unsigned int>    enumMap;
    std::map<std::string,unsigned int>    dirRobotMap;
    std::map<std::string,unsigned int>    dirUrdfMap;
    std::map<std::string,unsigned int>    dirIKMap;
    std::map<std::string,unsigned int>    offsetRobotMap;
    std::map<std::string,unsigned int>    offsetUrdfMap;
    std::map<std::string,unsigned int>    offsetIKMap;
    std::map<std::string,unsigned int>    bodyMap;
    std::map<std::string,unsigned int>    jointBodyMap;
    std::map<std::string,unsigned int>    motorRefMap;
    std::map<std::string,unsigned int>    motorCCWLimMap;
    std::map<std::string,unsigned int>    motorCWLimMap;

    Mapping(): idRobotMap(), idUrdfMap(), idIKMap(),enumMap(),
               dirRobotMap(),dirUrdfMap(),dirIKMap(),
               offsetRobotMap(),offsetUrdfMap(),offsetIKMap(),
               bodyMap(), jointBodyMap(),motorRefMap(),motorCCWLimMap(),motorCWLimMap() {}
    //Mapping(int robot);
    ~Mapping();

    //Metodos
    void         buildMap();
    void         print();
    static  void convert(const Eigen::MatrixXi  &map,const JointState& stFrom, JointState& stTo, int ikType = -1);
};


#endif

