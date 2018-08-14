#ifndef POSE_H
#define POSE_H

#include <vector>
#include "jointcommand.h"
#include "humanoid_general/basicxml.hpp"

class Pose
{
public:
    Pose();
    Pose(xmlNodePtr poseNode);
    void readFromNode(xmlNodePtr poseNode);
    xmlNodePtr toXmlNode();

    double getTime() const;
    void setTime(double value);

    double getPauseTime() const;
    void setPauseTime(double value);

    std::vector<JointCommand> getJoints() const;
    void setJoints(const std::vector<JointCommand> &value);

private:
    double time;
    double pauseTime;
    std::vector<JointCommand> joints;
};

#endif // POSE_H
