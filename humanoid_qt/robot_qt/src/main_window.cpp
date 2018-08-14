/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui->
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/robot_qt/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace robot_qt {


using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindowDesign)
    , qnode(new QNode(argc,argv ))
{
    ui->setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    setupFFTRealTimeGraph(ui->plotFFT);
    qRegisterMetaType<ImuMsg>("ImuMsg");
    qRegisterMetaType<FFTMsg>("FFTMsg");
    qRegisterMetaType<LearningMsg>("LearningMsg");
    qRegisterMetaType<ParamsMsg>("ParamsMsg");
    qRegisterMetaType<HumanoidPropsMsg>("HumanoidPropsMsg");
    tS     = 0.8;
    tD     = 0.1;
    stepH  = 0.05;
    zCCorr = 1;
    squat  = 0.05;
    open   = 0.02;
    incl   = 0;
    vx = 0;
    vy = 0;
    vz = 0;

    kpLeg    = 100;
    kpFoot   = 100;
    kpGround = 100;

    delayR   = 1;
    delayL   = 1;
    delayAll = 1;

    phiBuff    = 0;
    thetaBuff  = 0;
    psiBuff    = 0;
    timeBuff   = 0;

    walk_flag = false;


    rosCon  = false;
    QObject::connect(qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
    QObject::connect(this,SIGNAL(setWalkingParams(double,double,double,double)),
                     qnode,SLOT(sendWalkingParams(double,double,double,double)));
    QObject::connect(this,SIGNAL(setWalkingCmd(double,double,double,bool)),
                     qnode,SLOT(sendWalkingCmd(double,double,double,bool)));
    QObject::connect(this,SIGNAL(setReset()),
                     qnode,SLOT(sendReset()));
    QObject::connect(this,SIGNAL(setWalkingCtrl(double,double,double,double,double,double)),
                     qnode,SLOT(sendWalkingCtrl(double,double,double,double,double,double)));
    QObject::connect(this,SIGNAL(setWalkingCtrl(double,double,double,double,double,double)),
                     qnode,SLOT(sendWalkingCtrl(double,double,double,double,double,double)));
    QObject::connect(qnode, SIGNAL(sendParamsUpdate(ParamsMsg)), this, SLOT(getParamsUpdate(ParamsMsg)));
    QObject::connect(qnode, SIGNAL(sendHumanoidPropsUpdate(HumanoidPropsMsg)), this, SLOT(getHumanoidPropsUpdate(HumanoidPropsMsg)));
    QObject::connect(this,SIGNAL(setModelParams(double,double,double)),
                     qnode,SLOT(sendModelParams(double,double,double)));


}

MainWindow::~MainWindow()
{
    delete ui;
    delete qnode;

}


}  // namespace robot_qt
/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/


void robot_qt::MainWindow::quatern2euler(double q0,double q1,double q2,double q3,double &phi,double &theta,double &psi)
{
    float R11 = 0,R21 = 0,R31 = 0,R32 = 0,R33 = 0;
    R11 = 2.*q0*q0-1+2.*q1*q1;
    R21 = 2.*(q1*q2-q0*q3);
    R31 = 2.*(q1*q3+q0*q2);
    R32 = 2.*(q2*q3-q0*q1);
    R33 = 2.*q0*q0-1+2.*q3*q3;

    phi = atan2(R32, R33 );
    theta = -atan(R31/sqrt(1-R31*R31) );
    //theta = -asin(R31/sqrt(1-R31*R31) );
    psi = atan2(R21, R11 );


    phi   *= (180./3.14159);
    theta *= (180./3.14159);
    psi   *= (180./3.14159);


}
void robot_qt::MainWindow::setupRealTimeGraph(QCustomPlot *customPlot)
{
    customPlot->addGraph(); // red line
    customPlot->graph(0)->setPen(QPen(QColor(255, 110, 40)));
    customPlot->addGraph(); // green line
    customPlot->graph(1)->setPen(QPen(QColor(110, 255, 40)));
    customPlot->addGraph(); // blue line
    customPlot->graph(2)->setPen(QPen(QColor(40, 110, 255)));


    QSharedPointer<QCPAxisTickerTime> timeTicker(new QCPAxisTickerTime);
    timeTicker->setTimeFormat("%h:%m:%s");
    customPlot->xAxis->setTicker(timeTicker);
    //customPlot->xAxis->setRange(-20, 20);
    customPlot->axisRect()->setupFullAxesBox();
    //customPlot->yAxis->setRange(-1.2, 1.2);
    customPlot->yAxis->setRange(-0.2, 0.2);

    // make left and bottom axes transfer their ranges to right and top axes:
    connect(customPlot->xAxis, SIGNAL(rangeChanged(QCPRange)), customPlot->xAxis2, SLOT(setRange(QCPRange)));
    connect(customPlot->yAxis, SIGNAL(rangeChanged(QCPRange)), customPlot->yAxis2, SLOT(setRange(QCPRange)));

    // setup a timer that repeatedly calls MainWindow::realtimeDataSlot:
    connect(qnode, SIGNAL(sendImuUpdate(ImuMsg)), this, SLOT(getImuUpdate(ImuMsg)));
    //dataTimer.start(0); // Interval 0 means to refresh as fast as possible
}

void robot_qt::MainWindow::setupFFTRealTimeGraph(QCustomPlot *customPlot)
{
/*    customPlot->addGraph(); // red line
    customPlot->graph(0)->setPen(QPen(QColor(255, 110, 40)));

    customPlot->addGraph(); // green line
    customPlot->graph(1)->setPen(QPen(QColor(40, 255, 110)));

    customPlot->addGraph(); // blue line
    customPlot->graph(2)->setPen(QPen(QColor(40, 110, 255)));

    customPlot->xAxis->setRange(0, 0.1);
    customPlot->axisRect()->setupFullAxesBox();

    customPlot->yAxis->setRange(0, 0);

    // make left and bottom axes transfer their ranges to right and top axes:
    connect(customPlot->xAxis, SIGNAL(rangeChanged(QCPRange)), customPlot->xAxis2, SLOT(setRange(QCPRange)));
    connect(customPlot->yAxis, SIGNAL(rangeChanged(QCPRange)), customPlot->yAxis2, SLOT(setRange(QCPRange)));

    // setup a timer that repeatedly calls MainWindow::realtimeDataSlot:
    //connect(qnode, SIGNAL(sendFFTUpdate(FFTMsg)), this, SLOT(getFFTUpdate(FFTMsg)));
    connect(qnode, SIGNAL(sendLearningUpdate(LearningMsg)), this, SLOT(getLearningUpdate(LearningMsg)));
    //dataTimer.start(0); // Interval 0 means to refresh as fast as possible
*/
        customPlot->addGraph();
        customPlot->graph(0)->setPen(QPen(QColor(255, 110, 40)));

        customPlot->addGraph();
        customPlot->graph(1)->setPen(QPen(QColor(40, 255, 110)));

        customPlot->addGraph();
        customPlot->graph(2)->setPen(QPen(QColor(40, 110, 255)));

        customPlot->addGraph();
        customPlot->graph(3)->setPen(QPen(QColor(40, 110, 110)));

        customPlot->addGraph();
        customPlot->graph(4)->setPen(QPen(QColor(40, 110, 40)));

        customPlot->addGraph();
        customPlot->graph(5)->setPen(QPen(QColor(255, 110, 255)));

        customPlot->addGraph();
        customPlot->graph(6)->setPen(QPen(QColor(110, 110, 255)));

        customPlot->addGraph();
        customPlot->graph(7)->setPen(QPen(QColor(40, 255, 255)));

        customPlot->addGraph();
        customPlot->graph(8)->setPen(QPen(QColor(40, 110, 40)));

        customPlot->addGraph();
        customPlot->graph(9)->setPen(QPen(QColor(0, 110, 0)));

        customPlot->xAxis->setRange(0, 0.1);
        customPlot->axisRect()->setupFullAxesBox();

        customPlot->yAxis->setRange(0, 0);

        // make left and bottom axes transfer their ranges to right and top axes:
        connect(customPlot->xAxis, SIGNAL(rangeChanged(QCPRange)), customPlot->xAxis2, SLOT(setRange(QCPRange)));
        connect(customPlot->yAxis, SIGNAL(rangeChanged(QCPRange)), customPlot->yAxis2, SLOT(setRange(QCPRange)));

        // setup a timer that repeatedly calls MainWindow::realtimeDataSlot:
        //connect(qnode, SIGNAL(sendFFTUpdate(FFTMsg)), this, SLOT(getFFTUpdate(FFTMsg)));
        connect(qnode, SIGNAL(sendLearningUpdate(LearningMsg)), this, SLOT(getLearningUpdate(LearningMsg)));
        //dataTimer.start(0); // Interval 0 means to refresh as fast as possible
}

void robot_qt::MainWindow::setupWalkingXYGraph(QCustomPlot *customPlot)
{
    // generate some data:
    QVector<double> x(5), y(5); // initialize with entries 0..100
    /*for (int i=0; i<101; ++i)
    {
      x[i] = i/50.0 - 1; // x goes from -1 to 1
      y[i] = x[i]*x[i];  // let's plot a quadratic function
    }*/

    x[0] = 0.1;
    y[0] = 0.1;

    x[1] = 0.5;
    y[1] = 0.1;

    x[2] = 0.5;
    y[2] = 0.5;

    x[3] = 0.1;
    y[3] = 0.5;

    x[4] = 0.1;
    y[4] = 0.1;
    // create graph and assign data to it:
    customPlot->addGraph();
    customPlot->graph(0)->setData(x, y);
    // give the axes some labels:
    customPlot->xAxis->setLabel("x");
    customPlot->yAxis->setLabel("y");
    // set axes ranges, so we see all data:
    customPlot->xAxis->setRange(-1, 1);
    customPlot->yAxis->setRange(0, 1);
}


void robot_qt::MainWindow::getImuUpdate(const ImuMsg &imuMsg)
{
    //log(LogLevel::Info,std::string("TIME: ") + std::to_string(time));
    //qnode->log(qnode->LogLevel::Info,std::string("TIME: ") + std::to_string(imuMsg.header.stamp.sec));
    double time = (double)imuMsg.header.stamp.sec + ((double)imuMsg.header.stamp.nsec)/1000000000;
    double q0 = (double)imuMsg.orientation.w;
    double q1 = (double)imuMsg.orientation.x;
    double q2 = (double)imuMsg.orientation.y;
    double q3 = (double)imuMsg.orientation.z;

    double phi = 0;
    double theta = 0;
    double psi = 0;

    quatern2euler(q0,-q1,-q2,-q3,phi,theta,psi);

    //ui->customPlot->graph(0)->addData(time, imuMsg.linear_acceleration.x);
    //ui->customPlot->graph(1)->addData(time, imuMsg.linear_acceleration.y);


    ui->customPlot->graph(0)->addData(time, phi);
    ui->customPlot->graph(1)->addData(time, theta);
    ui->customPlot->graph(2)->addData(time, 0);

    /*double phiDot = (phi - phiBuff)/(time - timeBuff);
    phiBuff  = phi;

    double thetaDot = (theta - thetaBuff)/(time - timeBuff);
    thetaBuff  = theta;

    timeBuff = time;
    ui->customPlot->graph(0)->addData(phi, phiDot);
    ui->customPlot->graph(1)->addData(theta, thetaDot);
    ui->customPlot->graph(2)->addData(0, 0);*/

    // rescale value (vertical) axis to fit the current data:
    ui->customPlot->graph(0)->rescaleValueAxis(true);
    ui->customPlot->graph(1)->rescaleValueAxis(true);
    ui->customPlot->graph(2)->rescaleValueAxis(true);
    ui->customPlot->xAxis->setRange(time, 8, Qt::AlignRight);
    ui->customPlot->replot();
}


void robot_qt::MainWindow::getFFTUpdate(const FFTMsg &fftMsg)
{
    //log(LogLevel::Info,std::string("TIME: ") + std::to_string(time));
    //qnode->log(qnode->LogLevel::Info,std::string("TIME: ") + std::to_string(imuMsg.header.stamp.sec));
    double time = (double)fftMsg.header.stamp.sec + ((double)fftMsg.header.stamp.nsec)/1000000000;
    int binSize = fftMsg.bins.size();
    QVector<double> x(binSize), y(binSize); // initialize with entries 0..100
    //for (int i=1; i<int(binSize/2.); ++i)
    //for (int i=1; i<int(100./fftMsg.df); ++i)
    for (int i=1; i<int(binSize/2.); ++i)
    {
      x[i] = (i)*fftMsg.df; // x goes from -1 to 1
      y[i] = fftMsg.bins[i]*(2./binSize);  // let's plot a quadratic function
    }

    ui->plotFFT->graph(0)->setData(x, y);
    // rescale value (vertical) axis to fit the current data:
    ui->plotFFT->graph(0)->rescaleValueAxis(true);
    ui->plotFFT->graph(0)->rescaleKeyAxis(true);

    //ui->customPlot->xAxis->setRange(time, 8, Qt::AlignRight);
    ui->plotFFT->replot();
}


void robot_qt::MainWindow::getLearningUpdate(const LearningMsg &learningMsg)
{
    //log(LogLevel::Info,std::string("TIME: ") + std::to_string(time));
    //qnode->log(qnode->LogLevel::Info,std::string("TIME: ") + std::to_string(imuMsg.header.stamp.sec));
    double time = (double)learningMsg.header.stamp.sec + ((double)learningMsg.header.stamp.nsec)/1000000000;
    /*int binSize = learningMsg.x.bins.size();
    QVector<double> xX(binSize), yX(binSize),xY(binSize), yY(binSize),xZ(binSize), yZ(binSize); // initialize with entries 0..100
    for (int i=1; i<int(binSize/2.); ++i)
    //for (int i=1; i<int(10./learningMsg.df); ++i)
    {
      xX[i] = (i)*learningMsg.df; // x goes from -1 to 1
      yX[i] = learningMsg.x.bins[i]*(2./binSize);  // let's plot a quadratic function
      xY[i] = (i)*learningMsg.df; // x goes from -1 to 1
      yY[i] = learningMsg.y.bins[i]*(2./binSize);  // let's plot a quadratic function
      xZ[i] = (i)*learningMsg.df; // x goes from -1 to 1
      yZ[i] = learningMsg.z.bins[i]*(2./binSize);  // let's plot a quadratic function
    }

    ui->plotFFT->graph(0)->setData(xX, yX);
    ui->plotFFT->graph(1)->setData(xY, yY);
    ui->plotFFT->graph(2)->setData(xZ, yZ);

    ui->plotFFT->graph(0)->rescaleValueAxis(true);
    ui->plotFFT->graph(0)->rescaleKeyAxis(true);
    ui->plotFFT->graph(1)->rescaleValueAxis(true);
    ui->plotFFT->graph(1)->rescaleKeyAxis(true);
    ui->plotFFT->graph(2)->rescaleValueAxis(true);
    ui->plotFFT->graph(2)->rescaleKeyAxis(true);

    ui->plotFFT->replot();
    */



    /*int dataSize = learningMsg.rFoot.data.size();
    QVector<double> xX(dataSize), yX(dataSize),xY(dataSize), yY(dataSize);
    for (int i=0; i<dataSize; ++i)
    {
      xX[i] = (i)*0.002;
      yX[i] = learningMsg.rFoot.data[i];
      xY[i] = (i)*0.002;
      yY[i] = learningMsg.lFoot.data[i];
    }*/



    int forceSize = learningMsg.force.size();
    int torqueSize   = learningMsg.torqueF.size();
    int imuSize   = learningMsg.imu.size();

    QVector<double> forceT(forceSize),torqueT(torqueSize),imuT(imuSize),torque(torqueSize),force(forceSize),gyroX(imuSize),gyroY(imuSize),gyroZ(imuSize),accX(imuSize),accY(imuSize),accZ(imuSize),angX(imuSize),angY(imuSize);

    for (int i=0; i<forceSize; ++i)
    {
        forceT[i] = i*0.002;
        force[i]  = learningMsg.force[i];
    }

    for (int i=0; i<torqueSize; ++i)
    {
        torqueT[i] = i*0.008;
        torque[i]  = learningMsg.torqueF[i];
    }

    for (int i=0; i<imuSize; ++i)
    {
        imuT[i]   = i*0.008;
        gyroX[i]  = learningMsg.imu[i].angular_velocity.x;
        gyroY[i]  = learningMsg.imu[i].angular_velocity.y;
        gyroZ[i]  = learningMsg.imu[i].angular_velocity.z;

        accX[i]   = learningMsg.imu[i].linear_acceleration.x;
        accY[i]   = learningMsg.imu[i].linear_acceleration.y;
        accZ[i]   = learningMsg.imu[i].linear_acceleration.z;

        angX[i]   = learningMsg.imu[i].orientation.x;
        angY[i]   = learningMsg.imu[i].orientation.y;
    }

    /*ui->plotFFT->graph(0)->setData(forceT, force);
    ui->plotFFT->graph(1)->setData(imuT, gyroX);
    ui->plotFFT->graph(2)->setData(imuT, gyroY);
    ui->plotFFT->graph(3)->setData(imuT, gyroZ);
    ui->plotFFT->graph(4)->setData(imuT, accX);
    ui->plotFFT->graph(5)->setData(imuT, accY);
    ui->plotFFT->graph(6)->setData(imuT, accZ);
    ui->plotFFT->graph(7)->setData(imuT, angX);
    ui->plotFFT->graph(8)->setData(imuT, angY);*/
    ui->plotFFT->graph(9)->setData(torqueT, torque);


    ui->plotFFT->graph(0)->rescaleValueAxis(true);
    ui->plotFFT->graph(0)->rescaleKeyAxis(true);
    ui->plotFFT->graph(1)->rescaleValueAxis(true);
    ui->plotFFT->graph(1)->rescaleKeyAxis(true);
    ui->plotFFT->graph(2)->rescaleValueAxis(true);
    ui->plotFFT->graph(2)->rescaleKeyAxis(true);
    ui->plotFFT->graph(3)->rescaleValueAxis(true);
    ui->plotFFT->graph(3)->rescaleKeyAxis(true);
    ui->plotFFT->graph(4)->rescaleValueAxis(true);
    ui->plotFFT->graph(4)->rescaleKeyAxis(true);
    ui->plotFFT->graph(5)->rescaleValueAxis(true);
    ui->plotFFT->graph(5)->rescaleKeyAxis(true);
    ui->plotFFT->graph(6)->rescaleValueAxis(true);
    ui->plotFFT->graph(6)->rescaleKeyAxis(true);
    ui->plotFFT->graph(7)->rescaleValueAxis(true);
    ui->plotFFT->graph(7)->rescaleKeyAxis(true);
    ui->plotFFT->graph(8)->rescaleValueAxis(true);
    ui->plotFFT->graph(8)->rescaleKeyAxis(true);
    ui->plotFFT->graph(9)->rescaleValueAxis(true);
    ui->plotFFT->graph(9)->rescaleKeyAxis(true);

    ui->plotFFT->replot();

/*    int binSize = learningMsg.rFoot.bins.size();
    QVector<double> xX(binSize), yX(binSize),xY(binSize), yY(binSize);
    for (int i=0; i<binSize; ++i)
    {
        xX[i] = (i)*learningMsg.df;
        yX[i] = learningMsg.x.bins[i]*(2./binSize);
        xY[i] = (i)*learningMsg.df;
        yY[i] = learningMsg.y.bins[i]*(2./binSize);
    }

    ui->plotFFT->graph(0)->setData(xX, yX);
    ui->plotFFT->graph(1)->setData(xY, yY);

    ui->plotFFT->graph(0)->rescaleValueAxis(true);
    ui->plotFFT->graph(0)->rescaleKeyAxis(true);
    ui->plotFFT->graph(1)->rescaleValueAxis(true);
    ui->plotFFT->graph(1)->rescaleKeyAxis(true);

    ui->plotFFT->replot();*/
}

void robot_qt::MainWindow::getParamsUpdate(const ParamsMsg &paramsMsg)
{
    paramsMsgBuff = paramsMsg;

}

void robot_qt::MainWindow::getHumanoidPropsUpdate(const HumanoidPropsMsg &humanoidPropsMsg)
{
    humanoidPropsMsgBuff = humanoidPropsMsg;
}

void robot_qt::MainWindow::paramsUpdate()
{
    tS     = paramsMsgBuff.tS;
    ui->spinBoxTs->setValue(tS);
    //on_spinBoxTs_editingFinished();

    tD     = paramsMsgBuff.tD;
    ui->spinBoxTd->setValue(tD);
    //on_spinBoxTd_editingFinished();

    stepH  = paramsMsgBuff.stepH;
    ui->spinBoxStepH->setValue(stepH);
    //on_spinBoxStepH_editingFinished();

    zCCorr = paramsMsgBuff.zCCorr;
    ui->spinBoxZc->setValue(zCCorr);
    //on_spinBoxZc_editingFinished();

    squat  = humanoidPropsMsgBuff.squat;
    ui->spinBoxSquat->setValue(squat);
    //on_spinBoxSquat_editingFinished();

    open   = humanoidPropsMsgBuff.open;
    ui->spinBoxOpen->setValue(open);
    //on_spinBoxOpen_editingFinished();

    incl   = humanoidPropsMsgBuff.incl;
    ui->spinBoxIncl->setValue(incl);
    //on_spinBoxIncl_editingFinished();

    vx     = paramsMsgBuff.vx;
    ui->spinBoxVx->setValue(vx);
    //on_spinBoxVx_editingFinished();

    vy     = paramsMsgBuff.vy;
    ui->spinBoxVy->setValue(vy);
    //on_spinBoxVy_editingFinished();

    vz     = paramsMsgBuff.vz;
    ui->spinBoxVz->setValue(vz);
    //on_spinBoxVz_editingFinished();

    //kpLeg    = paramsMsgBuff.kpLeg;
    ui->spinBoxKpLeg->setValue(kpLeg);
    //on_spinBoxKpLeg_editingFinished();

    //kpFoot   = paramsMsgBuff.kpFoot;
    ui->spinBoxKpFoot->setValue(kpFoot);
    //on_spinBoxKpFoot_editingFinished();

    //kpGround = paramsMsgBuff.kpGround;
    ui->spinBoxKpGround->setValue(kpGround);
    //on_spinBoxKpGround_editingFinished();

    delayR   = paramsMsgBuff.delayR;
    ui->spinBoxDelayR->setValue(delayR);
    //on_spinBoxDelayR_editingFinished();

    delayL   = paramsMsgBuff.delayL;
    ui->spinBoxDelayL->setValue(delayL);
    //on_spinBoxDelayL_editingFinished();

    delayAll = paramsMsgBuff.delayAll;
    ui->spinBoxDelayAll->setValue(delayAll);
    //on_spinBoxDelayAll_editingFinished();


}



void robot_qt::MainWindow::on_sliderTs_actionTriggered(int action)
{
    int value  = ui->sliderTs->value();
    int range  = ui->sliderTs->maximum() - ui->sliderTs->minimum();
    double valueD = ((double)value/(double)range)*ui->spinBoxTs->maximum();
    ui->spinBoxTs->setValue(valueD);
    tS = valueD;
    Q_EMIT setWalkingParams(tS,tD,stepH,zCCorr);
}

void robot_qt::MainWindow::on_sliderTd_actionTriggered(int action)
{
    int value  = ui->sliderTd->value();
    int range  = ui->sliderTd->maximum() - ui->sliderTd->minimum();
    double valueD = ((double)value/(double)range)*ui->spinBoxTd->maximum();
    ui->spinBoxTd->setValue(valueD);
    tD = valueD;
    Q_EMIT setWalkingParams(tS,tD,stepH,zCCorr);
}

void robot_qt::MainWindow::on_sliderStepH_actionTriggered(int action)
{
    int value  = ui->sliderStepH->value();
    int range  = ui->sliderStepH->maximum() - ui->sliderStepH->minimum();
    double valueD = ((double)value/(double)range)*ui->spinBoxStepH->maximum();
    ui->spinBoxStepH->setValue(valueD);
    stepH = valueD;
    Q_EMIT setWalkingParams(tS,tD,stepH,zCCorr);
}

void robot_qt::MainWindow::on_sliderZc_actionTriggered(int action)
{
    int value  = ui->sliderZc->value();
    int range  = ui->sliderZc->maximum() - ui->sliderZc->minimum();
    double valueD = ((double)value/(double)range)*ui->spinBoxZc->maximum();
    ui->spinBoxZc->setValue(valueD);
    zCCorr = valueD;
    Q_EMIT setWalkingParams(tS,tD,stepH,zCCorr);
}



void robot_qt::MainWindow::on_spinBoxTs_editingFinished()
{
    double valueD  = ((ui->spinBoxTs->value()-ui->spinBoxTs->minimum())/(ui->spinBoxTs->maximum() - ui->spinBoxTs->minimum()))*ui->sliderTs->maximum();
    int    value   = int(valueD);
    ui->sliderTs->setValue(value);
    tS = ui->spinBoxTs->value();
    Q_EMIT setWalkingParams(tS,tD,stepH,zCCorr);
}

void robot_qt::MainWindow::on_spinBoxTd_editingFinished()
{
    double valueD  = ((ui->spinBoxTd->value()-ui->spinBoxTd->minimum())/(ui->spinBoxTd->maximum() - ui->spinBoxTd->minimum()))*ui->sliderTd->maximum();
    int    value   = int(valueD);
    ui->sliderTd->setValue(value);
    tD = ui->spinBoxTd->value();
    Q_EMIT setWalkingParams(tS,tD,stepH,zCCorr);
}

void robot_qt::MainWindow::on_spinBoxStepH_editingFinished()
{
    double valueD  = ((ui->spinBoxStepH->value()-ui->spinBoxStepH->minimum())/(ui->spinBoxStepH->maximum() - ui->spinBoxStepH->minimum()))*ui->sliderStepH->maximum();
    int    value   = int(valueD);
    ui->sliderStepH->setValue(value);
    stepH = ui->spinBoxStepH->value();
    Q_EMIT setWalkingParams(tS,tD,stepH,zCCorr);
}

void robot_qt::MainWindow::on_spinBoxZc_editingFinished()
{
    double valueD  = ((ui->spinBoxZc->value()-ui->spinBoxZc->minimum())/(ui->spinBoxZc->maximum() - ui->spinBoxZc->minimum()))*ui->sliderZc->maximum();
    int    value   = int(valueD);
    ui->sliderZc->setValue(value);
    zCCorr = ui->spinBoxZc->value();
    Q_EMIT setWalkingParams(tS,tD,stepH,zCCorr);
}



void robot_qt::MainWindow::on_sliderSquat_actionTriggered(int action)
{
    int value  = ui->sliderSquat->value();
    int range  = ui->sliderSquat->maximum() - ui->sliderSquat->minimum();
    double valueD = ((double)value/(double)range)*ui->spinBoxSquat->maximum();
    ui->spinBoxSquat->setValue(valueD);
    squat = valueD;
    Q_EMIT setModelParams(squat,open,incl);
}

void robot_qt::MainWindow::on_sliderOpen_actionTriggered(int action)
{
    int value  = ui->sliderOpen->value();
    int range  = ui->sliderOpen->maximum() - ui->sliderOpen->minimum();
    double valueD = ((double)value/(double)range)*ui->spinBoxOpen->maximum();
    ui->spinBoxOpen->setValue(valueD);
    open = valueD;
    Q_EMIT setModelParams(squat,open,incl);
}

void robot_qt::MainWindow::on_sliderIncl_actionTriggered(int action)
{
    int value  = ui->sliderIncl->value();
    int range  = ui->sliderIncl->maximum() - ui->sliderIncl->minimum();
    double valueD = ((double)value/(double)range)*(ui->spinBoxIncl->maximum() - ui->spinBoxIncl->minimum()) + ui->spinBoxIncl->minimum();
    ui->spinBoxIncl->setValue(valueD);
    incl = valueD;
    Q_EMIT setModelParams(squat,open,incl);
}

void robot_qt::MainWindow::on_spinBoxSquat_editingFinished()
{
    double valueD  = ((ui->spinBoxSquat->value()-ui->spinBoxSquat->minimum())/(ui->spinBoxSquat->maximum() - ui->spinBoxSquat->minimum()))*ui->sliderSquat->maximum();
    int    value   = int(valueD);
    ui->sliderSquat->setValue(value);
    squat = ui->spinBoxSquat->value();
    Q_EMIT setModelParams(squat,open,incl);
}

void robot_qt::MainWindow::on_spinBoxOpen_editingFinished()
{
    double valueD  = ((ui->spinBoxOpen->value()-ui->spinBoxOpen->minimum())/(ui->spinBoxOpen->maximum() - ui->spinBoxOpen->minimum()))*ui->sliderOpen->maximum();
    int    value   = int(valueD);
    ui->sliderOpen->setValue(value);
    open = ui->spinBoxOpen->value();
    Q_EMIT setModelParams(squat,open,incl);
}

void robot_qt::MainWindow::on_spinBoxIncl_editingFinished()
{
    double valueD  = ((ui->spinBoxIncl->value()-ui->spinBoxIncl->minimum())/(ui->spinBoxIncl->maximum() - ui->spinBoxIncl->minimum()))*ui->sliderIncl->maximum();
    int    value   = int(valueD);
    ui->sliderIncl->setValue(value);
    incl = ui->spinBoxIncl->value();
    Q_EMIT setModelParams(squat,open,incl);
}





void robot_qt::MainWindow::on_sliderVx_actionTriggered(int action)
{
    int value  = ui->sliderVx->value();
    int range  = ui->sliderVx->maximum() - ui->sliderVx->minimum();
    double valueD = ((double)value/(double)range)*ui->spinBoxVx->maximum();
    ui->spinBoxVx->setValue(valueD);
    vx = valueD;
    Q_EMIT setWalkingCmd(vx,vy,vz,walk_flag);
}

void robot_qt::MainWindow::on_sliderVy_actionTriggered(int action)
{
    int value  = ui->sliderVy->value();
    int range  = ui->sliderVy->maximum() - ui->sliderVy->minimum();
    double valueD = ((double)value/(double)range)*ui->spinBoxVy->maximum();
    ui->spinBoxVy->setValue(valueD);
    vy = valueD;
    Q_EMIT setWalkingCmd(vx,vy,vz,walk_flag);
}

void robot_qt::MainWindow::on_sliderVz_actionTriggered(int action)
{
    int value  = ui->sliderVz->value();
    int range  = ui->sliderVz->maximum() - ui->sliderVz->minimum();
    double valueD = ((double)value/(double)range)*(ui->spinBoxVz->maximum() - ui->spinBoxVz->minimum()) + ui->spinBoxVz->minimum();
    ui->spinBoxVz->setValue(valueD);
    vz = valueD;
    Q_EMIT setWalkingCmd(vx,vy,vz,walk_flag);
}

void robot_qt::MainWindow::on_spinBoxVx_editingFinished()
{
    double valueD  = ((ui->spinBoxVx->value()-ui->spinBoxVx->minimum())/(ui->spinBoxVx->maximum() - ui->spinBoxVx->minimum()))*ui->sliderVx->maximum();
    int    value   = int(valueD);
    ui->sliderVx->setValue(value);
    vx = ui->spinBoxVx->value();
    Q_EMIT setWalkingCmd(vx,vy,vz,walk_flag);
}

void robot_qt::MainWindow::on_spinBoxVy_editingFinished()
{
    double valueD  = ((ui->spinBoxVy->value()-ui->spinBoxVy->minimum())/(ui->spinBoxVy->maximum() - ui->spinBoxVy->minimum()))*ui->sliderVy->maximum();
    int    value   = int(valueD);
    ui->sliderVy->setValue(value);
    vy = ui->spinBoxVy->value();
    Q_EMIT setWalkingCmd(vx,vy,vz,walk_flag);
}

void robot_qt::MainWindow::on_spinBoxVz_editingFinished()
{
    double valueD  = ((ui->spinBoxVz->value()-ui->spinBoxVz->minimum())/(ui->spinBoxVz->maximum() - ui->spinBoxVz->minimum()))*ui->sliderVz->maximum();
    int    value   = int(valueD);
    ui->sliderVz->setValue(value);
    vz = ui->spinBoxVz->value();
    Q_EMIT setWalkingCmd(vx,vy,vz,walk_flag);
}






void robot_qt::MainWindow::on_sliderKpLeg_actionTriggered(int action)
{
    int value  = ui->sliderKpLeg->value();
    int range  = ui->sliderKpLeg->maximum() - ui->sliderKpLeg->minimum();
    double valueD = ((double)value/(double)range)*ui->spinBoxKpLeg->maximum();
    ui->spinBoxKpLeg->setValue(valueD);
    kpLeg = valueD;
    Q_EMIT setWalkingCtrl(delayR,delayL,delayAll,kpLeg,kpFoot,kpGround);
}

void robot_qt::MainWindow::on_sliderKpFoot_actionTriggered(int action)
{
    int value  = ui->sliderKpFoot->value();
    int range  = ui->sliderKpFoot->maximum() - ui->sliderKpFoot->minimum();
    double valueD = ((double)value/(double)range)*ui->spinBoxKpFoot->maximum();
    ui->spinBoxKpFoot->setValue(valueD);
    kpFoot = valueD;
    Q_EMIT setWalkingCtrl(delayR,delayL,delayAll,kpLeg,kpFoot,kpGround);
}

void robot_qt::MainWindow::on_sliderKpGround_actionTriggered(int action)
{
    int value  = ui->sliderKpGround->value();
    int range  = ui->sliderKpGround->maximum() - ui->sliderKpGround->minimum();
    double valueD = ((double)value/(double)range)*(ui->spinBoxKpGround->maximum() - ui->spinBoxKpGround->minimum()) + ui->spinBoxKpGround->minimum();
    ui->spinBoxKpGround->setValue(valueD);
    kpGround = valueD;

}

void robot_qt::MainWindow::on_spinBoxKpLeg_editingFinished()
{
    double valueD  = ((ui->spinBoxKpLeg->value()-ui->spinBoxKpLeg->minimum())/(ui->spinBoxKpLeg->maximum() - ui->spinBoxKpLeg->minimum()))*ui->sliderKpLeg->maximum();
    int    value   = int(valueD);
    ui->sliderKpLeg->setValue(value);
    kpLeg = ui->spinBoxKpLeg->value();
    Q_EMIT setWalkingCtrl(delayR,delayL,delayAll,kpLeg,kpFoot,kpGround);
}

void robot_qt::MainWindow::on_spinBoxKpFoot_editingFinished()
{
    double valueD  = ((ui->spinBoxKpFoot->value()-ui->spinBoxKpFoot->minimum())/(ui->spinBoxKpFoot->maximum() - ui->spinBoxKpFoot->minimum()))*ui->sliderKpFoot->maximum();
    int    value   = int(valueD);
    ui->sliderKpFoot->setValue(value);
    kpFoot = ui->spinBoxKpFoot->value();
    Q_EMIT setWalkingCtrl(delayR,delayL,delayAll,kpLeg,kpFoot,kpGround);
}

void robot_qt::MainWindow::on_spinBoxKpGround_editingFinished()
{
    double valueD  = ((ui->spinBoxKpGround->value()-ui->spinBoxKpGround->minimum())/(ui->spinBoxKpGround->maximum() - ui->spinBoxKpGround->minimum()))*ui->sliderKpGround->maximum();
    int    value   = int(valueD);
    ui->sliderKpGround->setValue(value);
    kpGround = ui->spinBoxKpGround->value();
}





void robot_qt::MainWindow::on_sliderDelayR_actionTriggered(int action)
{
    int value  = ui->sliderDelayR->value();
    int range  = ui->sliderDelayR->maximum() - ui->sliderDelayR->minimum();
    double valueD = ((double)value/(double)range)*ui->spinBoxDelayR->maximum();
    ui->spinBoxDelayR->setValue(valueD);
    delayR = valueD;
    Q_EMIT setWalkingCtrl(delayR,delayL,delayAll,kpLeg,kpFoot,kpGround);
}

void robot_qt::MainWindow::on_sliderDelayL_actionTriggered(int action)
{
    int value  = ui->sliderDelayL->value();
    int range  = ui->sliderDelayL->maximum() - ui->sliderDelayL->minimum();
    double valueD = ((double)value/(double)range)*ui->spinBoxDelayL->maximum();
    ui->spinBoxDelayL->setValue(valueD);
    delayL = valueD;
    Q_EMIT setWalkingCtrl(delayR,delayL,delayAll,kpLeg,kpFoot,kpGround);
}

void robot_qt::MainWindow::on_sliderDelayAll_actionTriggered(int action)
{
    int value  = ui->sliderDelayAll->value();
    int range  = ui->sliderDelayAll->maximum() - ui->sliderDelayAll->minimum();
    double valueD = ((double)value/(double)range)*(ui->spinBoxDelayAll->maximum() - ui->spinBoxDelayAll->minimum()) + ui->spinBoxDelayAll->minimum();
    ui->spinBoxDelayAll->setValue(valueD);
    delayAll = valueD;
    Q_EMIT setWalkingCtrl(delayR,delayL,delayAll,kpLeg,kpFoot,kpGround);

}

void robot_qt::MainWindow::on_spinBoxDelayR_editingFinished()
{
    double valueD  = ((ui->spinBoxDelayR->value()-ui->spinBoxDelayR->minimum())/(ui->spinBoxDelayR->maximum() - ui->spinBoxDelayR->minimum()))*ui->sliderDelayR->maximum();
    int    value   = int(valueD);
    ui->sliderDelayR->setValue(value);
    delayR = ui->spinBoxDelayR->value();
    Q_EMIT setWalkingCtrl(delayR,delayL,delayAll,kpLeg,kpFoot,kpGround);
}

void robot_qt::MainWindow::on_spinBoxDelayL_editingFinished()
{
    double valueD  = ((ui->spinBoxDelayL->value()-ui->spinBoxDelayL->minimum())/(ui->spinBoxDelayL->maximum() - ui->spinBoxDelayL->minimum()))*ui->sliderDelayL->maximum();
    int    value   = int(valueD);
    ui->sliderDelayL->setValue(value);
    delayL = ui->spinBoxDelayL->value();
    Q_EMIT setWalkingCtrl(delayR,delayL,delayAll,kpLeg,kpFoot,kpGround);

}

void robot_qt::MainWindow::on_spinBoxDelayAll_editingFinished()
{
    double valueD  = ((ui->spinBoxDelayAll->value()-ui->spinBoxDelayAll->minimum())/(ui->spinBoxDelayAll->maximum() - ui->spinBoxDelayAll->minimum()))*ui->sliderDelayAll->maximum();
    int    value   = int(valueD);
    ui->sliderDelayAll->setValue(value);
    delayAll = ui->spinBoxDelayAll->value();
    Q_EMIT setWalkingCtrl(delayR,delayL,delayAll,kpLeg,kpFoot,kpGround);
}


void robot_qt::MainWindow::on_buttonConnect_clicked()
{
    if(rosCon == false)
    {
        if ( !qnode->init() )
        {
            QMessageBox msgBox;
            msgBox.setText("Couldn't find the ros master.");
            msgBox.exec();
            rosCon = false;
            close();
        }//Connect
        else
        {
            rosCon = true;
            ui->boxConfig->setEnabled(true);
            ui->buttonResetRobot->setEnabled(true);
            ui->buttonUpdate->setEnabled(true);
            ui->buttonWalk->setEnabled(true);
            paramsUpdate();
            setupRealTimeGraph(ui->customPlot);
            setupFFTRealTimeGraph(ui->plotFFT);

            ui->buttonConnect->setText(QString("Disconnect"));
        }

    }//Disconnect
    else if(rosCon == true)
    {
        rosCon = false;
        ui->boxConfig->setEnabled(false);
        ui->buttonResetRobot->setEnabled(false);
        ui->buttonUpdate->setEnabled(false);
        ui->buttonWalk->setEnabled(false);
        qnode->shutdown();
        ui->buttonConnect->setText(QString("Connect"));

    }
}

void robot_qt::MainWindow::on_buttonResetRobot_clicked()
{
    Q_EMIT  setReset();
}

void robot_qt::MainWindow::on_buttonUpdate_clicked()
{
    paramsUpdate();
}

void robot_qt::MainWindow::on_buttonWalk_clicked()
{
    if(walk_flag == false)
    {
        walk_flag = true;
        ui->buttonWalk->setText(QString("Stop"));
    }
    else
    {
        walk_flag = false;
        ui->buttonWalk->setText(QString("Walk"));
    }
    Q_EMIT setWalkingCmd(vx,vy,vz,walk_flag);

}
