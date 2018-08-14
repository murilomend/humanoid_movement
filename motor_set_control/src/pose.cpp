#include "../include/motor_set_control/pose.h"

Pose::Pose()
{

}

Pose::Pose(xmlNodePtr poseNode)
{
    readFromNode(poseNode);
}

void Pose::readFromNode(xmlNodePtr poseNode)
{
    joints.clear();
    xmlNodePtr node;

    if(!isnode(poseNode, (char*)"pose"))
    {
        //TODO: ROS ERROR
        std::cout<<"Erro na leitura da pose\n";
        return;
    }
    std::string  st = std::string("-1");
    time =  string2double(parseprop2string(poseNode, (char*)"time", st));
    pauseTime = string2double(parseprop2string(poseNode, (char*)"pause_time", st));

    for(node = poseNode->children; node!=NULL; node = node->next)
    {
        if(!isnode(node, (char*)"joint")) continue;
        joints.push_back(JointCommand(node));
    }
}

xmlNodePtr Pose::toXmlNode()
{
    xmlNodePtr node;
    node = xmlNewNode(NULL, (xmlChar*)"pose");

    setstringprop(node,(char*)"time",toString(time));
    setstringprop(node,(char*)"pause_time",toString(pauseTime));
    for(int i = 0 ; i < joints.size() ; i++)
    {
        xmlNodePtr nodePtr;
        nodePtr = joints[i].toXmlNode();
        xmlAddChild(node,nodePtr);
    }
    return node;
}

double Pose::getTime() const
{
    return time;
}

void Pose::setTime(double value)
{
    time = value;
}

double Pose::getPauseTime() const
{
    return pauseTime;
}

void Pose::setPauseTime(double value)
{
    pauseTime = value;
}

std::vector<JointCommand> Pose::getJoints() const
{
    return joints;
}

void Pose::setJoints(const std::vector<JointCommand> &value)
{
    joints = value;
}
