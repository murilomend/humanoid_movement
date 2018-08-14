#ifndef POSEINTERFACE_H
#define POSEINTERFACE_H

#include <QWidget>
#include <vector>
#include <QPushButton>
#include <QDoubleSpinBox>
#include <QLineEdit>
#include <motor_set_control/pose.h>
#include <motor_set_control/jointcommand.h>
#include "ui_poseinterface.h"

namespace Ui {
class poseInterface;
}

class PoseInterface : public QWidget
{
    Q_OBJECT

public:
    explicit PoseInterface(QWidget *parent = 0);
    ~PoseInterface();
    void setActiveMotors(const std::vector<int> &value);
    void setJointPositions(std::vector<double> positions);
    void fromPose(Pose pose);
    Pose toPose();
    void enable();
    void disable();
    void setPoseNumber(int number);
public Q_SLOTS:
    void deselectAll();
private:
    Ui::poseInterface *ui;
    std::vector<int> activeMotors;
    std::vector <QDoubleSpinBox *> jointValues;
    QDoubleSpinBox *timeBox;
    QDoubleSpinBox *pauseTimeBox;
    void setInterface();
};

#endif // POSEINTERFACE_H
