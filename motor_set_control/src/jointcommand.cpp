#include "../include/motor_set_control/jointcommand.h"

JointCommand::JointCommand()
{

}

JointCommand::JointCommand(xmlNodePtr node)
{
    readFromNode(node);
}

void JointCommand::readFromNode(xmlNodePtr jointNode)
{
    if(!isnode(jointNode, (char*)"joint"))
    {
        //TODO: ROS ERROR
        std::cout<<"Erro na leitura do node joint\n";
        return;
    }
    std::string  st = std::string("-1");
    /*id = parseprop2int(jointNode, (char*)"id", -1);
    position = parseprop2double(jointNode, (char*)"pos", -1);
    velocity = parseprop2double(jointNode, (char*)"vel", -1);
    acceleration = parseprop2double(jointNode, (char*)"accel", -1);
    torque = parseprop2double(jointNode, (char*)"torque", -1);*/
    id = int(string2double(parseprop2string(jointNode, (char*)"id", st)));
    position = string2double(parseprop2string(jointNode, (char*)"pos", st));
    velocity = string2double(parseprop2string(jointNode, (char*)"vel", st));
    acceleration = string2double(parseprop2string(jointNode, (char*)"accel", st));
    torque = string2double(parseprop2string(jointNode, (char*)"torque", st));
}

void JointCommand::writeNode(std::string fileName)
{
    xmlNodePtr node;
    node = toXmlNode();
    xmlnode2file(node, fileName.c_str());
}

double JointCommand::getPosition() const
{
    return position;
}

void JointCommand::setPosition(double value)
{
    position = value;
}

double JointCommand::getVelocity() const
{
    return velocity;
}

void JointCommand::setVelocity(double value)
{
    velocity = value;
}

double JointCommand::getAcceleration() const
{
    return acceleration;
}

void JointCommand::setAcceleration(double value)
{
    acceleration = value;
}

double JointCommand::getTorque() const
{
    return torque;
}

void JointCommand::setTorque(double value)
{
    torque = value;
}

unsigned int JointCommand::getId() const
{
    return id;
}

void JointCommand::setId(unsigned int value)
{
    id = value;
}

xmlNodePtr JointCommand::toXmlNode()
{
    xmlNodePtr jointNode;
    jointNode = xmlNewNode(NULL, (xmlChar*)"joint");
    setintprop(jointNode, (char*)"id", id);
    setstringprop(jointNode, (char*)"pos",toString(position));
    setstringprop(jointNode, (char*)"vel", toString(velocity));
    setstringprop(jointNode, (char*)"accel", toString(acceleration));
    setstringprop(jointNode, (char*)"torque", toString(torque));
    return jointNode;
}
