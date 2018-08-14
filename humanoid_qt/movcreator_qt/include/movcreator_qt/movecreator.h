#ifndef MOVECREATOR_H
#define MOVECREATOR_H

#include <QMainWindow>
#include <QWidget>
#include <QScrollBar>
#include <QShortcut>
#include <QListView>
#include <iostream>
#include <string>
#include <vector>
#include <motor_set_control/page.h>
#include <motor_set_control/pose.h>
#include <motor_set_control/motorset.h>
#include "poseinterface.h"
#include <QFileDialog>
#include <QErrorMessage>
#include <QPushButton>
#include <QMenu>
#include <QMenuBar>
#include <QContextMenuEvent>
#include "ui_movecreator.h"
#include "qnode.hpp"
#include "ros/package.h"


namespace Ui {
class MoveCreator;
}
namespace movcreator_qt {
    class MoveCreator : public QMainWindow
    {
        Q_OBJECT

    public:
        explicit MoveCreator(int argc, char** argv, QWidget *parent = 0);
        ~MoveCreator();

    //Teste
        QAction *playAct;
        QAction *goPoseAct;
        QAction *saveAct;
        QAction *saveAsAct;
        QAction *loadAct;
        QAction *quitAct;

        QAction *addPoseAct;
        QAction *deletePoseAct;
        QAction *scanPoseAct;
        QAction *writePoseAct;
        QAction *copyPoseAct;
        QAction *cutPoseAct;
        QAction *pastePoseAct;
        QAction *moveLeftAct;
        QAction *moveRightAct;
        PoseInterface * createPoseInterface();

    public Q_SLOTS:
        void poseSelectionChanged(const QItemSelection & selected,const QItemSelection & deselected);
        void playPage();
        void goPose();
        void savePage();
        void saveAs();
        void loadPage();
        void quit();

        void addPose();
        void deletePose();
        void scanPose();
        void writePose();
        void copyPose();
        void cutPose();
        void pastePose();

        void turnOnOffMotor();
        void turnAllOn();
        void turnAllOff();

        void moveToLeft();
        void moveToRight();

        //////////////////////////////////// ROS /////////////////////////////////////
        void scanCompleted();
        void idsScanCompleted();
    Q_SIGNALS:
        void torqueSignal(std::vector<int> ids, std::vector<bool> torque);
        void requestScan();
        void scanActiveMotors();
        void playPageSignal(std::string path);
        void goPoseSignal(std::string path);
    protected:
    #ifndef QT_NO_CONTEXTMENU
        void contextMenuEvent(QContextMenuEvent *event) override;
    #endif // QT_NO_CONTEXTMENU
        bool eventFilter(QObject *obj, QEvent *event);
    private:
        QNode *qnode;
        bool signalReceived;
        /*
         * Variável que indica o indice da última pose criada na page
         */
        int lastPoseIndex;
        Pose currentPose;
//        MotorSet *motorSet;

        //Variável que indica os motores ativos no MoveCreator, isto é, os motores que foram reconhecidos e que são desejados no processo de criação da Page.
        std::vector<int> activeMotors;
        std::vector<double> scanValues;
        std::vector<bool> motorOn;
        std::vector<QPushButton *> motorButtons;
        std::vector<QPushButton *> scanButtons;
        std::vector<Pose> copiedPoses;

        Ui::MoveCreator *ui;
        QString fileName;
        Page generatePage();
        void setScanValues(std::vector<double> positions);
        void createConnections();
        void connectToROS();
        void setBasicLayout();
        void setupMotorLabels();
        void getActiveMotors();
        void setTorque();
        void scanThenSend();
        //Menu
        void createActions();
        void createMenus();

        QMenu *fileMenu;
        QMenu *editMenu;
        QMenu *helpMenu;

    };
}
#endif // MOVECREATOR_H
