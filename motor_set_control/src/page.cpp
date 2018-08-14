#include "../include/motor_set_control/page.h"

Page::Page()
{

}

Page::Page(xmlNodePtr pageNode)
{
    readFromNode(pageNode);
}

Page::Page(std::string fileName)
{
    xmlNodePtr node;
    node = file2xmlnode(fileName.c_str());
    readFromNode(node);
    xmlFreeDoc(node->doc);
}

Page::Page(const std::vector<Pose> &value)
{
    setPoses(value);
}

void Page::readFromNode(xmlNodePtr pageNode)
{
    poses.clear();
    xmlNodePtr node;

    if(!isnode(pageNode, (char*)"page"))
    {
        //TODO: ROS ERROR
        std::cout<<"Erro na leitura da page\n";
        return;
    }
    for(node = pageNode->children; node!=NULL; node = node->next)
    {
        if(!isnode(node, (char*)"pose")) continue;
        poses.push_back(Pose(node));
    }
}

xmlNodePtr Page::toXmlNode()
{
    xmlNodePtr node;
    node = xmlNewNode(NULL, (xmlChar*)"page");

    for(int i = 0 ; i < poses.size() ; i++)
    {
        xmlNodePtr nodePtr;
        nodePtr = poses[i].toXmlNode();
        xmlAddChild(node,nodePtr);
    }
    return node;
}

void Page::writeNode(std::string fileName)
{
    xmlNodePtr node;
    node = toXmlNode();
    xmlnode2file(node, fileName.c_str());
}

std::vector<Pose> Page::getPoses() const
{
    return poses;
}

void Page::setPoses(const std::vector<Pose> &value)
{
    poses = value;
}

double Page::getTotalTime()
{
    double totalTime = 0;
    for(unsigned int i = 0; i < poses.size(); i ++)
    {
        totalTime += poses[i].getTime();
        totalTime += poses[i].getPauseTime();
    }
    return totalTime;
}

