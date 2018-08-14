#include "../include/movcreator_qt/poseinterface.h"


PoseInterface::PoseInterface(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::poseInterface)
{
    ui->setupUi(this);
}

PoseInterface::~PoseInterface()
{
    delete ui;
}

void PoseInterface::setActiveMotors(const std::vector<int> &value)
{
    activeMotors = value;
    setInterface();
}

void PoseInterface::setJointPositions(std::vector<double> positions)
{
    if(positions.size() != activeMotors.size())
        return;
    else
    {
        for(unsigned int i = 0; i < activeMotors.size();i++)
            jointValues[i]->setValue(positions[i]);
    }
}

//TODO: fazer toda a inicialização necessária para não precisar chamar o setActiveMotors
void PoseInterface::fromPose(Pose pose)
{
    std::vector<JointCommand> joints;
    joints = pose.getJoints();
    for(unsigned int i = 0; i < joints.size(); i++)
    {
       jointValues[i]->setValue(joints[i].getPosition() * (180.0/ M_PI));
    }
    timeBox->setValue(pose.getTime());
    pauseTimeBox->setValue(pose.getPauseTime());
}

Pose PoseInterface::toPose()
{
    std::vector<JointCommand> joints;
    for(unsigned int i = 0; i < activeMotors.size();i++)
    {
        JointCommand joint;
        joint.setId(activeMotors[i]);
        joint.setPosition((jointValues[i]->value())/(180.0/ M_PI));
        //Teste
        joint.setTorque(-1);
        joint.setVelocity(-1);
        joint.setAcceleration(-1);
        joints.push_back(joint);
    }
    Pose pose;
    pose.setTime(timeBox->value());
    pose.setPauseTime(pauseTimeBox->value());
    pose.setJoints(joints);
    return pose;
}

void PoseInterface::deselectAll()
{
    QDoubleSpinBox *spinBox = (QDoubleSpinBox *)sender();
    spinBox->findChild<QLineEdit*>()->deselect();
    ui->frame->setFocus();
}

void PoseInterface::enable()
{
    for(unsigned int i = 0; i < jointValues.size() ; i++)
    {
        jointValues[i]->setEnabled(true);
    }
    timeBox->setEnabled(true);
    pauseTimeBox->setEnabled(true);
}

void PoseInterface::disable()
{
    for(unsigned int i = 0; i < jointValues.size() ; i++)
    {
        jointValues[i]->setEnabled(false);
    }
    timeBox->setEnabled(false);
    pauseTimeBox->setEnabled(false);
}

void PoseInterface::setPoseNumber(int number)
{
    ui->poseLabel->setText(QString("Pose %1").arg(number));
}

void PoseInterface::setInterface()
{
    //TODO:If de unidade de motor ou graus
    for(unsigned int i = 0; i < activeMotors.size();i++)
    {
        jointValues.push_back(new QDoubleSpinBox);
        jointValues[i]->setValue(0);
        jointValues[i]->setDecimals(2);
        jointValues[i]->setRange(-180.0,180.0); //Mudar futuramente
        QObject::connect(jointValues[i], SIGNAL(valueChanged(double)), this, SLOT(deselectAll()), Qt::QueuedConnection);
        ui->verticalLayout->addWidget(jointValues[i]);
    }
    timeBox = new QDoubleSpinBox;
    timeBox->setDecimals(3);
    timeBox->setRange(0.1,10.0);
    timeBox->setValue(1.5);
    ui->verticalLayout->addWidget(timeBox);

    pauseTimeBox = new QDoubleSpinBox;
    pauseTimeBox->setDecimals(3);
    pauseTimeBox->setRange(0.0,10.0);
    ui->verticalLayout->addWidget(pauseTimeBox);
    ui->poseLabel->setFrameStyle(1);
    ui->poseLabel->setAlignment(Qt::AlignCenter);
}
