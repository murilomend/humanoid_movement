#include "../include/motor_set_control/motorset.h"

MotorSet::MotorSet(ros::NodeHandle nh_, ros::NodeHandle nh_private_): nh(nh_), nh_private(nh_private_)
{
    if (!nh_private.getParam ("dt", dt))
        dt =  0.07; //s
    if (!nh_private.getParam ("toGazebo", toGazebo))
        toGazebo = false;
    if (!nh_private.getParam ("toRobot", toRobot))
        toRobot = true;

    //Service Server Stuff
    motorSetSrv = nh.advertiseService("motor_set_control/cmd",&MotorSet::motorSetService,this);

    //Publisher Stuff
    jointStateTopic = nh.advertise<JointStateMsg>("humanoid_model/jointState", 1000);

    //Service Stuff
    interfaceCli = nh.serviceClient<humanoid_msgs::InterfaceSrv>("humanoid_interface/cmd");
}


std::vector<double> MotorSet::getCurrentPose()
{
    std::vector<double> motorsPosition;

    interfaceSrv.request.toGazebo = toGazebo;
    interfaceSrv.request.toRobot = toRobot;
    interfaceSrv.request.send2Motor = true;
    interfaceSrv.request.scanMotors = false;
    interfaceSrv.request.getPID = false;
    interfaceSrv.request.getMotorPos = true;

    if(interfaceCli.call(interfaceSrv))
    {
        motorsPosition.clear();
        for(unsigned int i = 0; i < interfaceSrv.response.motorState.pos.size();i++)
        {
            motorsPosition.push_back((double)interfaceSrv.response.motorState.pos[i]);
        }
    }

    return motorsPosition;
}

bool MotorSet::motorSetService(humanoid_msgs::MotorSetSrv::Request &msg, humanoid_msgs::MotorSetSrv::Response &res)
{
    //A implementação é a mesma, utilizasse campos diferentes na mensagem só por organização
    if(!msg.page.empty())
    {
        std::string filePath(msg.page);
        Page page(filePath);
        if(msg.toRobot)
            linearInterpol(page,true);
        else
            linearInterpol(page,false);
    }
    if(!msg.pose.empty())
    {
        std::string filePath(msg.pose);
        Page page(filePath);
        if(msg.toRobot)
            linearInterpol(page,true);
        else
            linearInterpol(page,false);
    }
    return true;
}

void MotorSet::playPage(Page page,bool robot)
{
    std::vector<double> currentPose;
    if(robot)
        currentPose  = getCurrentPose();
    else
        currentPose.resize(21);

    //GAMBIARRA
    // for(int i = 0; i < 18; i++)
    // {
    //     if(currentPose[i] < 0.1 - M_PI)
    //     {
    //         ROS_ERROR("MOVCREATOR: Erro na leitura");
    //         return;
    //     }
    // }

    std::vector<double> time;
    std::vector<spline> interpolated;

    if(page.getPoses().size() > 0)
    {
        interpolated.resize(18);
        time.push_back(0.0);
        for(unsigned int i = 1; i < page.getPoses().size() + 1 ; i++)
            time.push_back(time[i - 1] + page.getPoses()[i - 1].getTime());

        time.push_back(time.back() + dt);
    }
    else
    {
        ROS_INFO("INVALID PAGE");
        return;
    }

    for(unsigned int j = 0; j < interpolated.size(); j++)
    {
        std::vector<double> angles;
        for(unsigned int i = 0; i < page.getPoses().size() + 1 ; i++)
        {
            if(i == 0)  //Primeira posição é a scanneada pelo robô
                angles.push_back(currentPose[j]);
            else
                angles.push_back(page.getPoses()[i - 1].getJoints()[j].getPosition());  // O [i - 1] é pq colocamos uma pose a mais, a atual
        }
        angles.push_back(page.getPoses().back().getJoints()[j].getPosition());
        interpolated[j].set_points(time,angles);
    }

    //Por enquanto o dt está fixo. Atualizar no futuro
    int numberOfPoints = std::ceil(page.getTotalTime() / dt);

    ros::Rate rate(1.0/dt);
    for(int i = 0; i < numberOfPoints ; i ++)
    {
        JointStateMsg msg;
        msg.header.stamp = ros::Time::now();
        msg.dt = dt;
        msg.type = 1;
        msg.pos.resize(21);
        msg.vel.resize(21);
        msg.acc.resize(21);
        msg.torq.resize(21);
        msg.source = MOTOR_SET;

        for(unsigned int j = 0; j < 21; j++) //21 Posições pq é assim que o humanoid_interface está definido atualmente
        {
            if(j < 18)  //Gambiarra
            {
                double newPosition = interpolated[j](double(i) * dt);
                msg.pos[j] = newPosition;
            }
            else
            {
                msg.pos[j] = 0;
            }
        }
            jointStateTopic.publish(msg);

        rate.sleep();
    }
}

void MotorSet::linearInterpol(Page page, bool robot)
{
    std::vector<double> currentPose;
    std::vector< std::vector<double> > toSend;

    if(robot)
        currentPose  = getCurrentPose();
    else
        currentPose.resize(21);

    //GAMBIARRA
    // for(int i = 0; i < 18; i++)
    // {
    //     if(currentPose[i] < 0.1 - M_PI)
    //     {
    //         ROS_ERROR("MOVCREATOR: Erro na leitura");
    //         return;
    //     }
    // }

    if(page.getPoses().size() > 0)
    {
        for(unsigned int i = 0; i < page.getPoses().size(); i++)
        {
            int points = ceil(page.getPoses()[i].getTime() / dt);

            for(int j = 0; j < points; j++) //Novos pontos que surgem com a interpolação
            {
                std::vector<double> pose;
                for(int m = 0; m < 18; m++) //Motores do corpo
                {
                    double initial;
                    double final;
                    if(i == 0)
                        initial = currentPose[m];
                    else
                        initial = page.getPoses()[i - 1].getJoints()[m].getPosition();
                    final = page.getPoses()[i].getJoints()[m].getPosition();
                    double dTheta = j*(final - initial)/(points * 1.0);
                    double newPosition = initial + dTheta;
                    pose.push_back(newPosition);
                }
                toSend.push_back(pose);
            }
        }
    }
    else
    {
        ROS_INFO("INVALID PAGE");
        return;
    }

    ros::Rate rate(1.0/dt);
    for(unsigned int i = 0; i < toSend.size() ; i ++)
    {
        JointStateMsg msg;
        msg.header.stamp = ros::Time::now();
        msg.dt = dt;
        msg.type = 1;
        msg.pos.resize(21);
        msg.vel.resize(21);
        msg.acc.resize(21);
        msg.torq.resize(21);
        msg.source = MOTOR_SET;

        for(unsigned int j = 0; j < 21; j++) //21 Posições pq é assim que o humanoid_interface está definido atualmente
        {
            if(j < 18)  //Motores do corpo
            {
                double newPosition = toSend[i][j];
                msg.pos[j] = newPosition;
            }
            else
            {
                msg.pos[j] = 0;
            }

        }
            jointStateTopic.publish(msg);

        rate.sleep();
    }
}
