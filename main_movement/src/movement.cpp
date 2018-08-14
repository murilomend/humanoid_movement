#include "../include/main_movement/movement.h"

namespace move {

    Movement::Movement(ros::NodeHandle nh_): nh(nh_)
    {
        //Service Server Stuff
        movementSrv = nh.advertiseService("main_movement/cmd",&Movement::movementService,this);

        //Publisher Stuff
        movementTopic = nh.advertise<MovementMsg>("main_movement/movement", 1000);
        jointStateTopic = nh.advertise<JointStateMsg>("humanoid_model/jointState", 1000);
        headJointStateTopic = nh.advertise<humanoid_msgs::JointStateMsg>("humanoid_model/jointHeadState", 1000);
        walkingTopic = nh.advertise<humanoid_msgs::WalkingMsg>("humanoid_walking/cmd",1000);

        //Service Stuff
        interfaceCli = nh.serviceClient<humanoid_msgs::InterfaceSrv>("humanoid_interface/cmd");
        motorSetCli = nh.serviceClient<humanoid_msgs::MotorSetSrv>("motor_set_control/cmd");

        //Subscriber Stuff
        imuSubPtr.reset(new ImuSub(nh,"humanoid_control/imu_euler",1));
        imuSubPtr->registerCallback(&Movement::imuCallback, this);

        dt = 0.07;
        xPos = 0;
        yPos = 0;
        desiredXPos = 0;
        desiredYPos = 0;
        roll = 0;
        pitch = 0;
        yaw = 0;

        walking = false;
        xHeadMove = false;
        yHeadMove = false;

        ros::Duration(0.3).sleep();   //Por algum motivo, se não tiver o delay a mensagem a seguir não é publicada
        sendHeadMessage(0.0,0.0,1.2); //Centralizar no inicio
        ros::Duration(1.5).sleep();   //Esperar a centralizaçao

        runTimer  = nh.createTimer(ros::Duration(dt), &Movement::runCallBack,this);
    }

    bool Movement::movementService(humanoid_msgs::MovementSrv::Request &msg, humanoid_msgs::MovementSrv::Response &res)
    {
        //FALLING
        if(msg.falling)
        {
            sendWalkingParam(false,0,0,0);
            Movement::HeadMoveMsg msg;
            msg.xMove = STOP;
            msg.yMove = STOP;
            updateHeadCommand(msg);
            fallSrv();
        }

        //WALKING MOVEMENT
        if(msg.walk.go == GO)
        {
            if(!walking)
                walking = true;

            sendWalkingParam(msg.walk.go,msg.walk.vx,msg.walk.vy,msg.walk.vz);
        }
        else if(msg.walk.go == STOP)
        {
            if(walking)
            {
                walking = false;
                sendWalkingParam(msg.walk.go,msg.walk.vx,msg.walk.vy,msg.walk.vz);
            }
        }

        //HEAD MOVEMENT
        bool update = false;

        //Ifs para verificar se é necessário modificar o movimento da cabeça e atualizar os estados internos se for o caso
        if(msg.head.xMove == GO)
        {
            if(!xHeadMove)
                xHeadMove = true;

            if(msg.head.xPos != desiredXPos)
                update = true;
        }
        else if(msg.head.xMove == STOP)
        {
            if(xHeadMove)
            {
                xHeadMove = false;
                update = true;
            }

        }

        if(msg.head.yMove == GO)
        {
            if(!yHeadMove)
                yHeadMove = true;

            if(msg.head.yPos != desiredYPos)
                update = true;
        }
        else if(msg.head.xMove == STOP)
        {
            if(yHeadMove)
            {
                yHeadMove = false;
                update = true;
            }
        }

        if(update)
            updateHeadCommand(msg.head);

        //MOTOR_SET MOVEMENT
        pageMovements(msg.move);
        return true;
    }

    void Movement::runCallBack(const ros::TimerEvent &)
    {
        if(xPos != desiredXPos || yPos != desiredYPos)
        {
            double velInRadians = (HEAD_VEL * M_PI)/180.0;
            double maxDisplacement = velInRadians*dt;

            if(fabs(desiredXPos - xPos) <= maxDisplacement)
                xPos = desiredXPos;
            else
            {
                if(desiredXPos > xPos)
                    xPos += maxDisplacement;
                else
                    xPos -= maxDisplacement;
            }

            if(fabs(desiredYPos - yPos) <= maxDisplacement)
                yPos = desiredYPos;
            else
            {
                if(desiredYPos > yPos)
                    yPos += maxDisplacement;
                else
                    yPos -= maxDisplacement;
            }
            sendHeadMessage(xPos,yPos,velInRadians);
        }
        else
        {
            xHeadMove = false;
            yHeadMove = false;
        }
        publish();
    }

    void Movement::publish()
    {
        MovementMsg msg;
        msg.xHead = xPos;
        msg.yHead = yPos;
        msg.roll = roll;
        msg.pitch = pitch;
        msg.yaw = yaw;

        //TODO: mandar informações do sensor

        movementTopic.publish(msg);
    }

    void Movement::pageMovements(int pageEnum)
    {
        bool toSend = true;
        std::string path;

        switch (pageEnum) {
        case RIGHT_LEG_KICK:

            break;

        case LEFT_LEG_KICK:

            break;
        case FRONT_STAND_UP:

            break;
        default:
            toSend = false;
            break;
        }

        if(toSend)
        {
            motorSetSrv.request.toGazebo = false;
            motorSetSrv.request.toRobot = true;
            motorSetSrv.request.page = path;
            motorSetSrv.request.pose.clear();

            motorSetCli.call(motorSetSrv);
            //TODO: possivelmente informar o behavior que o movimento foi finalizado
        }
    }

    void Movement::updateHeadCommand(Movement::HeadMoveMsg msg)
    {
        if(msg.xMove == GO)
            desiredXPos = msg.xPos;
        else
            desiredXPos = xPos; //posiçao atual, comando para parar

        if(msg.yMove ==GO)
            desiredYPos = msg.yPos;
        else
            desiredYPos = yPos; //posiçao atual, comando para parar
    }

    void Movement::sendHeadMessage(double x, double y, double vel)
    {
        JointStateMsg msg;
        msg.header.stamp = ros::Time::now();
        msg.type = 1;
        msg.pos.resize(21);
        msg.vel.resize(21);
        msg.source = MOTOR_HEAD;

        msg.pos[18] = x;
        msg.pos[19] = y;
        msg.vel[18] = vel;
        msg.vel[19] = vel;

        headJointStateTopic.publish(msg);
    }

    void Movement::sendWalkingParam(int go,double vx,double vy, double vz)
    {
        humanoid_msgs::WalkingMsg msg;
        msg.go = go;
        msg.vx = vx;
        msg.vy = vy;
        msg.vz = vz;
        walkingTopic.publish(msg);
    }

    void Movement::fallSrv()
    {
        humanoid_msgs::InterfaceSrv interfaceSrv;
        interfaceSrv.request.toGazebo = false;
        interfaceSrv.request.toRobot = true;
        interfaceSrv.request.scanMotors = false;
        interfaceSrv.request.getPID = false;
        interfaceSrv.request.getMotorPos = false;
        interfaceSrv.request.send2Motor = true;
        interfaceSrv.request.fall = true;
        interfaceCli.call(interfaceSrv);
    }

    void Movement::imuCallback(const sensor_msgs::ImuPtr &msg)
    {
        roll = msg->orientation.x;
        pitch = msg->orientation.y;
        yaw = msg->orientation.z;
    }
}
