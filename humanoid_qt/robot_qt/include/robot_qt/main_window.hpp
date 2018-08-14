/**
 * @file /include/robot_qt/main_window.hpp
 *
 * @brief Qt based gui for robot_qt.
 *
 * @date November 2010
 **/
#ifndef robot_qt_MAIN_WINDOW_H
#define robot_qt_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "qcustomplot.hpp"
#include "ui_main_window.h"
#include "qnode.hpp"
#include <sensor_msgs/Imu.h>

/*****************************************************************************
** Namespace
*****************************************************************************/



namespace robot_qt {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();
    void setupRealTimeGraph(QCustomPlot *customPlot);
    void setupFFTRealTimeGraph(QCustomPlot *customPlot);
    void setupWalkingXYGraph(QCustomPlot *customPlot);
    void quatern2euler(double q0, double q1, double q2, double q3, double &phi, double &theta, double &psi);
    void paramsUpdate();
    double tS;
    double tD;
    double stepH;
    double zCCorr;
    double squat;
    double open;
    double incl;
    double vx;
    double vy;
    double vz;
    bool   rosCon;
    bool   walk_flag;

    double kpLeg;
    double kpFoot;
    double kpGround;

    double delayR;
    double delayL;
    double delayAll;

    double phiBuff;
    double thetaBuff;
    double psiBuff;
    double timeBuff;

    ParamsMsg paramsMsgBuff;
    HumanoidPropsMsg humanoidPropsMsgBuff;


public Q_SLOTS:
    void on_sliderTs_actionTriggered(int action);
    void on_sliderTd_actionTriggered(int action);
    void on_sliderStepH_actionTriggered(int action);
    void on_sliderZc_actionTriggered(int action);

    void on_spinBoxTs_editingFinished();
    void on_spinBoxTd_editingFinished();
    void on_spinBoxStepH_editingFinished();
    void on_spinBoxZc_editingFinished();

    void on_sliderSquat_actionTriggered(int action);
    void on_sliderOpen_actionTriggered(int action);
    void on_sliderIncl_actionTriggered(int action);
    void on_spinBoxSquat_editingFinished();
    void on_spinBoxOpen_editingFinished();
    void on_spinBoxIncl_editingFinished();


    void on_sliderVx_actionTriggered(int action);
    void on_sliderVy_actionTriggered(int action);
    void on_sliderVz_actionTriggered(int action);
    void on_spinBoxVx_editingFinished();
    void on_spinBoxVy_editingFinished();
    void on_spinBoxVz_editingFinished();


    void on_sliderKpLeg_actionTriggered(int action);
    void on_sliderKpFoot_actionTriggered(int action);
    void on_sliderKpGround_actionTriggered(int action);
    void on_spinBoxKpLeg_editingFinished();
    void on_spinBoxKpFoot_editingFinished();
    void on_spinBoxKpGround_editingFinished();


    void on_sliderDelayR_actionTriggered(int action);
    void on_sliderDelayL_actionTriggered(int action);
    void on_sliderDelayAll_actionTriggered(int action);
    void on_spinBoxDelayR_editingFinished();
    void on_spinBoxDelayL_editingFinished();
    void on_spinBoxDelayAll_editingFinished();

    void on_buttonConnect_clicked();
    void on_buttonResetRobot_clicked();
    void on_buttonUpdate_clicked();
    void on_buttonWalk_clicked();
    void getImuUpdate(const ImuMsg &imuMsg);
    void getFFTUpdate(const FFTMsg &fftMsg);
    void getLearningUpdate(const LearningMsg &learningMsg);
    void getParamsUpdate(const ParamsMsg &paramsMsg);
    void getHumanoidPropsUpdate(const HumanoidPropsMsg &humanoidPropsMsg);

Q_SIGNALS:

    void setWalkingParams(double tS,double tD, double stepH,double zCCorr);
    void setWalkingCmd(double vx,double vy, double vz,bool walk_flag);
    void setWalkingCtrl(double delayR,double delayL, double delayAll,double kpLeg,double kpFoot, double kpGround);
    void setModelParams(double squat,double open,double incl);
    void setReset();



private:
    Ui::MainWindowDesign *ui;

    QNode                *qnode;
    QTimer                dataTimer;
    QCPItemTracer        *itemDemoPhaseTracer;
};

}  // namespace robot_qt

#endif // robot_qt_MAIN_WINDOW_H

