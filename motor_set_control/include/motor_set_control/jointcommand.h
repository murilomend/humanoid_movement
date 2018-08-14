#ifndef JOINTCOMMAND_H
#define JOINTCOMMAND_H

//TODO: organizar os arquivos em suas respectivas pastas posteriormente
#include <humanoid_general/basicxml.hpp>
#include <iostream>
#include <string>
#include <cmath>
#include <stdlib.h>

//Essa classe é um tanto desnecessário ja que poderia ser representada pela JointState

class JointCommand
{
public:
    JointCommand();
    JointCommand(xmlNodePtr node);
    void readFromNode(xmlNodePtr node);
    xmlNodePtr toXmlNode();
    void writeNode(std::string fileName);
    void sendToMotor();

    double getPosition() const;
    void setPosition(double value);

    double getVelocity() const;
    void setVelocity(double value);

    double getAcceleration() const;
    void setAcceleration(double value);

    double getTorque() const;
    void setTorque(double value);

    unsigned int getId() const;
    void setId(unsigned int value);

private:
    unsigned int id;
    double position;
    double velocity;
    double acceleration;
    double torque;
};

#endif // JOINTCOMMAND_H
