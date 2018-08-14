#ifndef PAGE_H
#define PAGE_H

#include <vector>
#include <string>
#include "pose.h"
#include "humanoid_general/basicxml.hpp"

class Page
{
public:
    Page();
    Page(const std::vector<Pose> &value);
    Page(std::string fileName);
    Page(xmlNodePtr pageNode);
    void readFromNode(xmlNodePtr pageNode);
    xmlNodePtr toXmlNode();
    void writeNode(std::string fileName);

    std::vector<Pose> getPoses() const;
    void setPoses(const std::vector<Pose> &value);
    double getTotalTime();
private:
    std::vector<Pose> poses;
};

#endif // PAGE_H
